/* ZaberDriver.cpp
 *
 * Zaber ASCII devices as an asynMotorController. See ZaberDriver.h and
 * ioc-motion's README, "Zaber (ASCII)".
 *
 * What goes down the line (Zaber ASCII protocol manuals), every command
 * addressed to one device and one axis with a message id,
 * "/<device> <axis> <id> <command>" + LF; never the broadcast address 0,
 * never "set", "renumber" or "system ...":
 *   creation:            nothing
 *   once per axis, after its first answered get pos, retried until get
 *   version answers:     get version, get deviceid (device.id if rejected),
 *                        get system.serial (device scope, axis 0);
 *                        get peripheralid (peripheral.id), get resolution,
 *                        get maxspeed, get limit.min, get limit.max (axis)
 *   every poll:          get pos; and warnings (the full flag list, a read)
 *                        when the reply's top flag is not "--". An axis
 *                        silent for ZABER_POLL_FAILURES polls is asked only
 *                        once per ZABER_BACKOFF_POLLS polls
 *   writes:              move abs <pos> [<speed> <accel>] (a move; refused
 *                        while WR unless allow_unreferenced);
 *                        stop (always, unless a fresh valid reply says IDLE);
 *                        home (only with allow_home); driver enable /
 *                        driver disable (only with power_policy on_for_move);
 *                        warnings clear (only with allow_clear_warnings, on
 *                        a put to the axis's clear_warnings record)
 * A reply is "@<dd> <a> <id> OK|RJ IDLE|BUSY <flag> <data>[:CC]" CR LF. Only
 * the reply with this command's address, axis and id is taken: alerts ("!"),
 * info lines ("#"), another address's reply and a late reply to an earlier
 * command (another id) are skipped, and a line whose checksum (LRC, sent
 * when comm.checksum is 1) does not verify is dropped.
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <ctype.h>

#include <iocsh.h>
#include <initHooks.h>
#include <epicsThread.h>
#include <asynOctetSyncIO.h>

#include <asynMotorController.h>
#include <asynMotorAxis.h>

#include <epicsExport.h>
#include "ZaberDriver.h"

static const char *driverName = "ZaberDriver";

/* Set once the IOC is running: each axis then sends its full status once, so
 * a condition already present at boot (a fault, a WL from before the IOC)
 * reaches the motor record's alarm, which is computed only when a status
 * callback arrives; the poll before iocInit had no subscriber. */
static volatile int zaberIocRunning = 0;

static void zaberInitHook(initHookState state)
{
  if (state == initHookAfterIocRunning) zaberIocRunning = 1;
}

#define NINT(f) (long)((f) > 0 ? (f) + 0.5 : (f) - 0.5)

/** The controller: one serial line, up to MAX_ZABER_AXES (device, axis) pairs.
  * \param[in] portName          The asyn port name of this driver
  * \param[in] serialPortName    The asyn octet port of the line (input EOS CR LF, output EOS LF)
  * \param[in] numAxes           Motor-record axes on this line, 1..MAX_ZABER_AXES
  * \param[in] movingPollPeriod  Seconds between polls while an axis moves
  * \param[in] idlePollPeriod    Seconds between polls while idle
  */
ZaberController::ZaberController(const char *portName, const char *serialPortName, int numAxes,
                                 double movingPollPeriod, double idlePollPeriod)
  : asynMotorController(portName, numAxes, NUM_ZABER_PARAMS,
                        asynOctetMask, asynOctetMask,
                        ASYN_CANBLOCK | ASYN_MULTIDEVICE, 1, 0, 0),
    nextId_(0)
{
  createParam(ZaberFlagString, asynParamOctet, &ZaberFlag_);
  createParam(ZaberWarningsString, asynParamOctet, &ZaberWarnings_);
  createParam(ZaberVersionString, asynParamOctet, &ZaberVersion_);
  createParam(ZaberDeviceIdString, asynParamInt32, &ZaberDeviceId_);
  createParam(ZaberPeripheralIdString, asynParamInt32, &ZaberPeripheralId_);
  createParam(ZaberSerialString, asynParamInt32, &ZaberSerial_);
  createParam(ZaberMaxspeedString, asynParamFloat64, &ZaberMaxspeed_);
  createParam(ZaberLimitMinString, asynParamFloat64, &ZaberLimitMin_);
  createParam(ZaberLimitMaxString, asynParamFloat64, &ZaberLimitMax_);
  createParam(ZaberResolutionString, asynParamInt32, &ZaberResolution_);
  createParam(ZaberClearWarningsString, asynParamInt32, &ZaberClearWarnings_);
  createParam(ZaberStopFailedString, asynParamInt32, &ZaberStopFailed_);

  if (pasynOctetSyncIO->connect(serialPortName, 0, &pasynUserController_, NULL)) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s: cannot connect to serial port %s\n",
              driverName, serialPortName);
  }
  for (int axis = 0; axis < numAxes; axis++) new ZaberAxis(this, axis);
  startPoller(movingPollPeriod, idlePollPeriod, 2);
}

void ZaberController::report(FILE *fp, int level)
{
  fprintf(fp, "Zaber ASCII driver %s: numAxes=%d, moving poll %g s, idle poll %g s\n",
          portName, numAxes_, movingPollPeriod_, idlePollPeriod_);
  asynMotorController::report(fp, level);
}

ZaberAxis* ZaberController::getAxis(asynUser *pasynUser)
{
  return static_cast<ZaberAxis*>(asynMotorController::getAxis(pasynUser));
}

ZaberAxis* ZaberController::getAxis(int axisNo)
{
  return static_cast<ZaberAxis*>(asynMotorController::getAxis(axisNo));
}

/** The one command beyond the motor record: a put to the axis's
  * clear_warnings record sends "warnings clear" (only with allow_clear_warnings).
  * The record is a momentary button (its HIGH returns it to 0, and a 0 sends
  * nothing): whatever happened (sent, refused, no reply; the IOC log says
  * which, and the axis's :warnings shows the result) the put succeeds, so a
  * refused put leaves no standing WRITE alarm. */
asynStatus ZaberController::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
  if (pasynUser->reason == ZaberClearWarnings_) {
    ZaberAxis *pAxis = getAxis(pasynUser);
    if (!pAxis) return asynError;
    if (value) pAxis->clearWarnings();
    setIntegerParam(pAxis->axisNo_, ZaberClearWarnings_, 0);
    callParamCallbacks(pAxis->axisNo_);
    return asynSuccess;
  }
  return asynMotorController::writeInt32(pasynUser, value);
}

/** The LRC checksum: the 8-bit sum of the bytes after the message type up to
  * the colon, plus the checksum itself, is 0 (protocol manual, Checksumming). */
static int checksumOk(const char *line, const char *colon)
{
  unsigned sum = 0;
  for (const char *p = line + 1; p < colon; p++) sum += (unsigned char)*p;
  char *end;
  unsigned long cc = strtoul(colon + 1, &end, 16);
  if (end != colon + 3) return 0;
  return ((sum + cc) & 0xFF) == 0;
}

/** The next space-delimited word of *p, terminated in place; NULL at the end. */
static char *word(char **p)
{
  while (**p == ' ') (*p)++;
  if (!**p) return NULL;
  char *s = *p;
  while (**p && **p != ' ') (*p)++;
  if (**p) { **p = '\0'; (*p)++; }
  return s;
}

static int isNumber(const char *s)
{
  if (!*s) return 0;
  for (; *s; s++) if (!isdigit((unsigned char)*s)) return 0;
  return 1;
}

/** Parse one line in place: 1 if it is a reply ("@dd a [id] OK|RJ IDLE|BUSY ff data"),
  * -1 if it carries a checksum that does not verify, 0 otherwise. */
static int parseReply(char *line, ZaberReply *r)
{
  if (line[0] != '@') return 0;
  size_t len = strlen(line);
  if (len > 3 && line[len - 3] == ':') {
    if (!checksumOk(line, line + len - 3)) return -1;
    line[len - 3] = '\0';
  }
  char *p = line + 1;
  char *dev = word(&p), *ax = word(&p), *w = word(&p);
  if (!dev || !ax || !w || !isNumber(dev) || !isNumber(ax)) return 0;
  r->id = -1;
  if (isNumber(w)) {
    r->id = atoi(w);
    w = word(&p);
    if (!w) return 0;
  }
  char *status = word(&p), *flag = word(&p);
  if (!status || !flag) return 0;
  if (strcmp(w, "OK") && strcmp(w, "RJ")) return 0;
  if (strcmp(status, "IDLE") && strcmp(status, "BUSY")) return 0;
  if (strlen(flag) != 2) return 0;
  r->device = atoi(dev);
  r->axis = atoi(ax);
  r->ok = !strcmp(w, "OK");
  r->busy = !strcmp(status, "BUSY");
  strcpy(r->flag, flag);
  while (*p == ' ') p++;
  snprintf(r->data, sizeof(r->data), "%s", p);
  for (size_t m = strlen(r->data); m > 0 && isspace((unsigned char)r->data[m - 1]); m--) r->data[m - 1] = '\0';
  return 1;
}

/** One command to one device and axis, and its reply: the line with that
  * address, axis and message id. Anything else (alerts, info lines, another
  * address's reply, a late reply to an earlier command) is skipped, up to
  * ZABER_MAX_LINES lines. */
asynStatus ZaberController::exchange(int device, int axis, const char *command, ZaberReply *r)
{
  char out[ZABER_REPLY_LEN], line[ZABER_REPLY_LEN];
  size_t nwrite = 0, nread = 0;
  int eom = 0;
  asynStatus status;

  if (device < 1 || device > 99) return asynError;     /* never the broadcast address */
  int id = nextId_;
  nextId_ = (nextId_ + 1) % 100;
  snprintf(out, sizeof(out), "/%d %d %02d %s", device, axis, id, command);
  pasynOctetSyncIO->flush(pasynUserController_);
  status = pasynOctetSyncIO->write(pasynUserController_, out, strlen(out), ZABER_TIMEOUT, &nwrite);
  if (status) return status;
  for (int i = 0; i < ZABER_MAX_LINES; i++) {
    status = pasynOctetSyncIO->read(pasynUserController_, line, sizeof(line) - 1, ZABER_TIMEOUT, &nread, &eom);
    if (status) return status;
    line[nread] = '\0';
    char *p = line;
    while (*p && isspace((unsigned char)*p)) p++;
    int parsed = parseReply(p, r);
    if (parsed < 0) {
      asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s: a reply whose checksum does not verify, dropped\n", driverName);
      continue;
    }
    if (!parsed) continue;
    if (r->device != device || r->axis != axis || r->id != id) continue;
    return asynSuccess;
  }
  return asynError;
}

/* ---------------------------------------------------------------- axis */

ZaberAxis::ZaberAxis(ZaberController *pC, int axisNo)
  : asynMotorAxis(pC, axisNo),
    pC_(pC), device_(0), axis_(1), speedArgs_(0), allowHome_(0), allowUnreferenced_(0), allowClear_(0),
    powerPolicy_(ZABER_POWER_LEAVE), powerOffDelay_(0.0), identified_(0), firmware_(0), moving_(0), busy_(0),
    idDone_(0), idPasses_(0), wlSide_(ZABER_WL_NONE), flagsComplete_(1), stopFailed_(0), statusSentRunning_(0),
    homed_(0), failures_(0), backoff_(0), driverOn_(1), weDisabled_(0), lastDirPositive_(1), offPending_(0),
    offFailures_(0), settling_(0),
    lastPos_(0), nflags_(0)
{
  setIntegerParam(pC_->motorStatusHasEncoder_, 0);
  setIntegerParam(pC_->motorStatusGainSupport_, 0);
  pC_->setStringParam(axisNo_, pC_->ZaberFlag_, "");
  pC_->setStringParam(axisNo_, pC_->ZaberWarnings_, "");
  pC_->setStringParam(axisNo_, pC_->ZaberVersion_, "");
  pC_->setIntegerParam(axisNo_, pC_->ZaberDeviceId_, 0);
  pC_->setIntegerParam(axisNo_, pC_->ZaberPeripheralId_, 0);
  pC_->setIntegerParam(axisNo_, pC_->ZaberSerial_, 0);
  pC_->setIntegerParam(axisNo_, pC_->ZaberResolution_, 0);
  pC_->setIntegerParam(axisNo_, pC_->ZaberClearWarnings_, 0);
  pC_->setIntegerParam(axisNo_, pC_->ZaberStopFailed_, 0);
  pC_->setDoubleParam(axisNo_, pC_->ZaberMaxspeed_, 0.0);
  pC_->setDoubleParam(axisNo_, pC_->ZaberLimitMin_, 0.0);
  pC_->setDoubleParam(axisNo_, pC_->ZaberLimitMax_, 0.0);
  callParamCallbacks();
}

void ZaberAxis::configure(int device, int axis, int speedArgs, int allowHome, int powerPolicy, double powerOffDelay,
                          int allowUnreferenced, int allowClearWarnings)
{
  device_ = device;
  axis_ = axis;
  speedArgs_ = speedArgs ? 1 : 0;
  allowHome_ = allowHome ? 1 : 0;
  powerPolicy_ = powerPolicy;
  powerOffDelay_ = powerOffDelay;
  allowUnreferenced_ = allowUnreferenced ? 1 : 0;
  allowClear_ = allowClearWarnings ? 1 : 0;
}

void ZaberAxis::report(FILE *fp, int level)
{
  if (level > 0) {
    fprintf(fp, "  device %d axis %d: firmware %d.%02d, speed args %d, home %s, unreferenced moves %s, "
                "warnings clear %s, power %s (off after %g s), moving %d, busy %d, homed %d, driver on %d, "
                "failures %d, last pos %ld, flags %d\n",
            device_, axis_, firmware_ / 100, firmware_ % 100, speedArgs_, allowHome_ ? "allowed" : "refused",
            allowUnreferenced_ ? "allowed" : "refused", allowClear_ ? "allowed" : "refused",
            powerPolicy_ == ZABER_POWER_ON_FOR_MOVE ? "on_for_move" : "leave", powerOffDelay_,
            moving_, busy_, homed_, driverOn_, failures_, lastPos_, nflags_);
  }
  asynMotorAxis::report(fp, level);
}

asynStatus ZaberAxis::cmd(const char *command, ZaberReply *r)
{
  return pC_->exchange(device_, axis_, command, r);
}

asynStatus ZaberAxis::deviceCmd(const char *command, ZaberReply *r)
{
  return pC_->exchange(device_, 0, command, r);
}

/** One identity read: done (its bit set in idDone_) once the device has
  * answered it, OK or RJ: a rejected setting (older firmware lacks some) is an
  * answer, read as absent; a timeout is not, and is asked again next poll.
  * With an alternative name, a rejection of the first is followed by the
  * second. Returns 1 with r holding an OK reply, 0 otherwise. */
int ZaberAxis::idRead(int bit, ZaberReply *r, const char *command, const char *alternative, int deviceScope)
{
  if (idDone_ & bit) return 0;
  asynStatus s = deviceScope ? deviceCmd(command, r) : cmd(command, r);
  if (s == asynSuccess && !r->ok && alternative) s = deviceScope ? deviceCmd(alternative, r) : cmd(alternative, r);
  if (s != asynSuccess) return 0;
  idDone_ |= bit;
  return r->ok;
}

/** The identity and the settings the record's configuration is checked
  * against: all "get". get version first, retried on later polls until it
  * answers (a stage off or mis-addressed at boot is identified when it
  * answers); then each other read, each retried until answered, so one
  * dropped reply at boot does not leave its record at 0 for the IOC's life.
  * A genuine 0 (limit.min, a serial an X-MCC axis lacks) is an answer. */
void ZaberAxis::identify()
{
  ZaberReply r;
  if (!firmware_) {
    if (deviceCmd("get version", &r) != asynSuccess || !r.ok) {
      pC_->setStringParam(axisNo_, pC_->ZaberVersion_, "no reply");
      return;
    }
    int major = 0, minor = 0;
    if (sscanf(r.data, "%d.%d", &major, &minor) != 2 || major * 100 + minor <= 0) {
      pC_->setStringParam(axisNo_, pC_->ZaberVersion_, r.data);
      return;
    }
    firmware_ = major * 100 + minor;
    pC_->setStringParam(axisNo_, pC_->ZaberVersion_, r.data);
  }
  if (idRead(ZABER_ID_DEVICEID, &r, "get deviceid", "get device.id", 1))
    pC_->setIntegerParam(axisNo_, pC_->ZaberDeviceId_, atoi(r.data));
  if (idRead(ZABER_ID_SERIAL, &r, "get system.serial", NULL, 1))
    pC_->setIntegerParam(axisNo_, pC_->ZaberSerial_, atoi(r.data));
  if (idRead(ZABER_ID_PERIPHERAL, &r, "get peripheralid", "get peripheral.id", 0))
    pC_->setIntegerParam(axisNo_, pC_->ZaberPeripheralId_, atoi(r.data));
  if (idRead(ZABER_ID_RESOLUTION, &r, "get resolution", NULL, 0))
    pC_->setIntegerParam(axisNo_, pC_->ZaberResolution_, atoi(r.data));
  if (idRead(ZABER_ID_MAXSPEED, &r, "get maxspeed", NULL, 0))
    pC_->setDoubleParam(axisNo_, pC_->ZaberMaxspeed_, strtod(r.data, NULL));
  if (idRead(ZABER_ID_LIMIT_MIN, &r, "get limit.min", NULL, 0))
    pC_->setDoubleParam(axisNo_, pC_->ZaberLimitMin_, strtod(r.data, NULL));
  if (idRead(ZABER_ID_LIMIT_MAX, &r, "get limit.max", NULL, 0))
    pC_->setDoubleParam(axisNo_, pC_->ZaberLimitMax_, strtod(r.data, NULL));
  /* A device that answers get pos but never one setting would cost every poll
   * a timeout for ever: after ZABER_ID_PASSES passes the rest are given up
   * (their records stay 0) with a message. */
  if (idDone_ != ZABER_ID_ALL && ++idPasses_ >= ZABER_ID_PASSES) {
    asynPrint(pasynUser_, ASYN_TRACE_ERROR,
              "%s:identify: device %d axis %d: identity reads 0x%02x unanswered after %d passes: given up (their records stay 0)\n",
              driverName, device_, axis_, ZABER_ID_ALL & ~idDone_, idPasses_);
    idDone_ = ZABER_ID_ALL;
  }
  identified_ = (idDone_ == ZABER_ID_ALL);
}

/** The flags of a fresh reply: none when the top flag is "--", else the full
  * list ("warnings": "<count> <flag> <flag> ..."). If that query fails only
  * the top flag is known and flagsComplete_ is 0: a lower flag (WR under a
  * fault or a W flag that outranks it) may be hidden, and a move is refused. */
void ZaberAxis::takeFlags(const ZaberReply &top)
{
  ZaberReply r;
  nflags_ = 0;
  flagsComplete_ = 1;
  pC_->setStringParam(axisNo_, pC_->ZaberFlag_, top.flag);
  if (strcmp(top.flag, "--") == 0) return;
  if (cmd("warnings", &r) == asynSuccess && r.ok) {
    char *save = NULL, *tok = strtok_r(r.data, " ", &save);    /* the count */
    while (tok && (tok = strtok_r(NULL, " ", &save)) != NULL && nflags_ < ZABER_MAX_FLAGS) {
      if (strlen(tok) == 2) { strcpy(flags_[nflags_], tok); nflags_++; }
    }
  } else {
    flagsComplete_ = 0;
  }
  if (nflags_ == 0) { strcpy(flags_[0], top.flag); nflags_ = 1; }
}

/** stop_failed: a stop that got no reply or a rejection is an alarm on the
  * axis (the record's :stop_failed, MAJOR) until a stop or move is
  * acknowledged, or a fresh reply says the axis is IDLE. No automatic resend:
  * a second stop while decelerating is the device's instant stop. */
void ZaberAxis::setStopFailed(int failed)
{
  if (stopFailed_ == failed) return;
  stopFailed_ = failed;
  pC_->setIntegerParam(axisNo_, pC_->ZaberStopFailed_, failed);
}

int ZaberAxis::hasFlag(const char *flag) const
{
  for (int i = 0; i < nflags_; i++) if (strcmp(flags_[i], flag) == 0) return 1;
  return 0;
}

/** Faults (F...) are problems, except the driver off by request: FO (7.11 and
  * up), and FD while this IOC's own "driver disable" stands. FD is otherwise a
  * fault on every firmware: the firmware 6 manual's FD is the driver disabling
  * itself on overheating, the firmware 7 manual's an overheating or
  * over-current driver ("called Driver Disabled in 7.10 and below", the flag
  * a driver disable raised there). */
int ZaberAxis::isFault(const char *flag) const
{
  if (flag[0] != 'F') return 0;
  if (strcmp(flag, "FO") == 0) return 0;
  if (strcmp(flag, "FD") == 0 && weDisabled_) return 0;
  return 1;
}

asynStatus ZaberAxis::driverEnable()
{
  ZaberReply r;
  offPending_ = 0;
  asynStatus status = cmd("driver enable", &r);
  if (status == asynSuccess && !r.ok) {
    asynPrint(pasynUser_, ASYN_TRACE_ERROR, "%s: device %d axis %d: driver enable rejected (%s)\n",
              driverName, device_, axis_, r.data);
    return asynError;
  }
  if (status == asynSuccess) weDisabled_ = 0;
  return status;
}

void ZaberAxis::scheduleOff(double delay)
{
  epicsTimeGetCurrent(&offAt_);
  epicsTimeAddSeconds(&offAt_, delay);
  offPending_ = 1;
}

/** "warnings clear", from the clear_warnings record: clears the latching
  * flags (FS, FE, FT, FQ, FB, WL, WS, NI ...) that only it clears. Only with
  * allow_clear_warnings, and only from a WRITER (access.acf). */
asynStatus ZaberAxis::clearWarnings()
{
  ZaberReply r;
  if (!device_) return asynError;
  if (!allowClear_) {
    asynPrint(pasynUser_, ASYN_TRACE_ERROR,
              "%s:clearWarnings: device %d axis %d: not allowed on this axis (allow_clear_warnings); nothing sent\n",
              driverName, device_, axis_);
    return asynError;
  }
  asynStatus status = cmd("warnings clear", &r);
  if (status) return status;
  asynPrint(pasynUser_, ASYN_TRACE_ERROR, "%s:clearWarnings: device %d axis %d: cleared (%s)\n",
            driverName, device_, axis_, r.data);
  return r.ok ? asynSuccess : asynError;
}

/** move abs <target> [<speed> <accel>]: always absolute (the record's
  * relative moves are added to the position just read). Refused while the
  * axis has no reference (WR) unless allow_unreferenced: its limits and
  * positions then do not match the physical travel (protocol manual, WR),
  * so the record's DLLM/DHLM would be measured from the wrong origin. The
  * speed and accel arguments (firmware 7.25 and up) only with speed_args;
  * otherwise the device's own maxspeed and accel settings govern. */
asynStatus ZaberAxis::move(double position, int relative, double minVelocity, double maxVelocity, double acceleration)
{
  static const char *functionName = "move";
  char command[ZABER_REPLY_LEN];
  ZaberReply r;
  asynStatus status;

  if (!device_) return asynError;
  if (failures_ > 0) {
    asynPrint(pasynUser_, ASYN_TRACE_ERROR,
              "%s:%s: device %d axis %d: the last poll failed: no fresh position, move refused\n",
              driverName, functionName, device_, axis_);
    return asynError;
  }
  if (speedArgs_ && firmware_ < ZABER_SPEED_ARGS_FW) {
    asynPrint(pasynUser_, ASYN_TRACE_ERROR,
              "%s:%s: device %d axis %d: speed_args needs firmware 7.25 and up (this one: %s): move refused\n",
              driverName, functionName, device_, axis_, firmware_ ? "older" : "not read yet");
    return asynError;
  }
  status = cmd("get pos", &r);
  if (status || !r.ok) return asynError;
  lastPos_ = strtol(r.data, NULL, 10);
  takeFlags(r);
  if (r.busy) {
    asynPrint(pasynUser_, ASYN_TRACE_ERROR,
              "%s:%s: device %d axis %d: busy (a move, a home or the knob): move refused\n",
              driverName, functionName, device_, axis_);
    return asynError;
  }
  if (!flagsComplete_ && !allowUnreferenced_) {
    asynPrint(pasynUser_, ASYN_TRACE_ERROR,
              "%s:%s: device %d axis %d: flag %s up and the warnings query failed (a WR may be hidden): move refused\n",
              driverName, functionName, device_, axis_, r.flag);
    return asynError;
  }
  if (hasFlag("WR") && !allowUnreferenced_) {
    asynPrint(pasynUser_, ASYN_TRACE_ERROR,
              "%s:%s: device %d axis %d: no reference position (WR): move refused (home it, or allow_unreferenced)\n",
              driverName, functionName, device_, axis_);
    return asynError;
  }
  long target = relative ? lastPos_ + NINT(position) : NINT(position);
  /* A latched WL is a limit switch on its side: a move further that way is
   * refused, nothing sent (the record, seeing HLS or LLS in the direction of
   * motion, would otherwise abandon the move it had just sent, resetting VAL
   * to RBV while the stage ran on to the target). A move away is allowed. */
  if ((wlSide_ == ZABER_WL_HIGH && target > lastPos_) || (wlSide_ == ZABER_WL_LOW && target < lastPos_)) {
    asynPrint(pasynUser_, ASYN_TRACE_ERROR,
              "%s:%s: device %d axis %d: WL latched on the %s side: a move further that way is refused\n",
              driverName, functionName, device_, axis_, wlSide_ == ZABER_WL_HIGH ? "high" : "low");
    return asynError;
  }
  if (powerPolicy_ == ZABER_POWER_ON_FOR_MOVE && driverEnable() != asynSuccess) return asynError;
  if (speedArgs_) {
    long speed = NINT(fabs(maxVelocity) * ZABER_VELOCITY_DATA);
    long accel = NINT(fabs(acceleration) * ZABER_ACCEL_DATA);
    if (speed < 1) speed = 1;
    if (accel < 1) accel = 1;
    snprintf(command, sizeof(command), "move abs %ld %ld %ld", target, speed, accel);
  } else {
    snprintf(command, sizeof(command), "move abs %ld", target);
  }
  status = cmd(command, &r);
  if (status) return status;
  if (!r.ok) {
    asynPrint(pasynUser_, ASYN_TRACE_ERROR, "%s:%s: device %d axis %d: '%s' rejected (%s)\n",
              driverName, functionName, device_, axis_, command, r.data);
    if (powerPolicy_ == ZABER_POWER_ON_FOR_MOVE) scheduleOff(0.0);   /* enabled for a move that never ran */
    return asynError;
  }
  moving_ = 1;
  busy_ = 1;
  offPending_ = 0;                       /* a new move: the last one's power-off and settle are moot */
  settling_ = 0;
  setStopFailed(0);
  lastDirPositive_ = target >= lastPos_ ? 1 : 0;
  setIntegerParam(pC_->motorStatusDirection_, lastDirPositive_);
  setIntegerParam(pC_->motorStatusDone_, 0);
  setIntegerParam(pC_->motorStatusMoving_, 1);
  callParamCallbacks();
  return asynSuccess;
}

/** "move vel" exists, but a jog that loses its stop runs to the end of travel:
  * refused; the record's tweaks (TWF/TWR) are moves and work. */
asynStatus ZaberAxis::moveVelocity(double minVelocity, double maxVelocity, double acceleration)
{
  asynPrint(pasynUser_, ASYN_TRACE_ERROR,
            "%s:moveVelocity: device %d axis %d: jog is refused (use TWF/TWR); nothing sent\n",
            driverName, device_, axis_);
  return asynError;
}

/** "home": toward the motor end at the device's own approach speed, which
  * sets the reference (WR clears). Only with allow_home; accepted under WR,
  * since a home is what gives the axis its reference. */
asynStatus ZaberAxis::home(double minVelocity, double maxVelocity, double acceleration, int forwards)
{
  ZaberReply r;
  if (!device_) return asynError;
  if (!allowHome_) {
    asynPrint(pasynUser_, ASYN_TRACE_ERROR,
              "%s:home: device %d axis %d: home is not allowed on this axis (allow_home); nothing sent\n",
              driverName, device_, axis_);
    return asynError;
  }
  if (powerPolicy_ == ZABER_POWER_ON_FOR_MOVE && driverEnable() != asynSuccess) return asynError;
  asynStatus status = cmd("home", &r);
  if (status) return status;
  if (!r.ok) {
    asynPrint(pasynUser_, ASYN_TRACE_ERROR, "%s:home: device %d axis %d: rejected (%s)\n",
              driverName, device_, axis_, r.data);
    if (powerPolicy_ == ZABER_POWER_ON_FOR_MOVE) scheduleOff(0.0);
    return asynError;
  }
  moving_ = 1;
  busy_ = 1;
  offPending_ = 0;
  settling_ = 0;
  setStopFailed(0);
  lastDirPositive_ = 0;                  /* toward the motor end */
  setIntegerParam(pC_->motorStatusDone_, 0);
  setIntegerParam(pC_->motorStatusMoving_, 1);
  callParamCallbacks();
  return asynSuccess;
}

/** "stop" (decelerate), sent unless a fresh, valid reply says the axis is
  * IDLE: a failed, garbled or rejected "get pos" sends it anyway, since a
  * stop to an idle axis is harmless and a stop lost to a dropped reply is
  * not. A second stop while the axis is still decelerating is the device's
  * instant stop (protocol manual, stop): what an operator who presses STOP
  * twice asks for. An error is returned when the stop itself failed. */
asynStatus ZaberAxis::stop(double acceleration)
{
  ZaberReply r;
  if (!device_) return asynSuccess;
  if (cmd("get pos", &r) == asynSuccess && r.ok && !r.busy) {
    asynPrint(pasynUser_, ASYN_TRACE_FLOW, "%s:stop: device %d axis %d: IDLE; nothing sent\n",
              driverName, device_, axis_);
    setStopFailed(0);
    callParamCallbacks();
    return asynSuccess;
  }
  asynStatus status = cmd("stop", &r);
  if (status == asynSuccess && !r.ok) {
    asynPrint(pasynUser_, ASYN_TRACE_ERROR, "%s:stop: device %d axis %d: rejected (%s)\n",
              driverName, device_, axis_, r.data);
    status = asynError;
  } else if (status) {
    asynPrint(pasynUser_, ASYN_TRACE_ERROR, "%s:stop: device %d axis %d: stop sent, no reply (status %d)\n",
              driverName, device_, axis_, (int)status);
  }
  setStopFailed(status != asynSuccess);
  callParamCallbacks();
  return status;
}

/** "set pos" would redefine the device's reference: refused, nothing sent. */
asynStatus ZaberAxis::setPosition(double position)
{
  asynPrint(pasynUser_, ASYN_TRACE_ERROR,
            "%s:setPosition: device %d axis %d: the reference is the device's (home); nothing sent\n",
            driverName, device_, axis_);
  return asynError;
}

asynStatus ZaberAxis::poll(bool *moving)
{
  ZaberReply r;
  int fail = 0, asked = 1;
  char text[ZABER_MAX_FLAGS * 3 + 1];

  if (!device_) { *moving = false; return asynSuccess; }

  epicsTimeStamp readAt;                 /* when this poll's position was asked for */
  epicsTimeGetCurrent(&readAt);
  if (backoff_ > 0) {
    /* A silent axis is asked once per ZABER_BACKOFF_POLLS polls, so it does
     * not hold the line (and every other axis's poll, stop or move) for a
     * timeout each cycle. It keeps its comm error and problem meanwhile. */
    backoff_--;
    fail = 1;
    asked = 0;
  } else if (cmd("get pos", &r) == asynSuccess && r.ok) {
    char *end;
    long pos = strtol(r.data, &end, 10);
    if (end == r.data) {
      asynPrint(pasynUser_, ASYN_TRACE_ERROR, "%s:poll: device %d axis %d: get pos answered '%s'\n",
                driverName, device_, axis_, r.data);
      fail = 1;
    } else {
      lastPos_ = pos;
      busy_ = r.busy;
      setDoubleParam(pC_->motorPosition_, (double)pos);
      takeFlags(r);
      if (!identified_) identify();
    }
  } else {
    fail = 1;
  }

  if (!fail) {
    int problem = 0;
    text[0] = '\0';
    for (int i = 0; i < nflags_; i++) {
      if (i) strcat(text, " ");
      strcat(text, flags_[i]);
      if (isFault(flags_[i])) problem = 1;
    }
    pC_->setStringParam(axisNo_, pC_->ZaberWarnings_, text);
    homed_ = !(hasFlag("WR") || hasFlag("WH"));
    driverOn_ = !(hasFlag("FO") || hasFlag("FD"));
    /* WL: a limit sensor interrupted a move; it latches until warnings clear.
     * Its side is taken once, on the poll where WL first appears, and only from
     * a move or home of ours that was running then (still moving_ on the poll
     * that ends it, before the power-off below), and held until WL clears:
     * later moves do not move it. Any other WL (the knob, another client, one
     * left from before this IOC started) has no known side: PROBLEM, not a
     * guessed limit, until it clears. */
    if (!hasFlag("WL")) {
      wlSide_ = ZABER_WL_NONE;
    } else if (wlSide_ == ZABER_WL_NONE) {
      int ours = moving_ && !offPending_ && !settling_;
      wlSide_ = !ours ? ZABER_WL_UNKNOWN : (lastDirPositive_ ? ZABER_WL_HIGH : ZABER_WL_LOW);
    }
    setIntegerParam(pC_->motorStatusHighLimit_, wlSide_ == ZABER_WL_HIGH);
    setIntegerParam(pC_->motorStatusLowLimit_, wlSide_ == ZABER_WL_LOW);
    if (wlSide_ == ZABER_WL_UNKNOWN) problem = 1;
    if (!busy_) setStopFailed(0);        /* at rest: the stop question is settled */

    /* The end of a move. Under leave: done when the device is IDLE. Under
     * on_for_move (as motorAerotech's power policy): still not done (DMOV 0)
     * until the driver disable is acknowledged and a position has been read
     * ZABER_SETTLE_S after it, because with its driver off a stage's position
     * changes (U_Zaber's m1: +10, +32 microsteps, 2026-10-30): a client
     * acquiring on DMOV then sees where the stage rests. */
    if (moving_ && !busy_ && !offPending_ && !settling_) {
      if (powerPolicy_ == ZABER_POWER_ON_FOR_MOVE) { scheduleOff(powerOffDelay_); offFailures_ = 0; }
      else moving_ = 0;
    }
    if (settling_ && !busy_ && epicsTimeDiffInSeconds(&readAt, &settleAt_) >= 0) {
      settling_ = 0;                     /* this poll's position was read after the settle time */
      moving_ = 0;
    }
    if (offPending_ && !busy_) {
      epicsTimeStamp now;
      epicsTimeGetCurrent(&now);
      if (epicsTimeDiffInSeconds(&now, &offAt_) >= 0) {
        ZaberReply d;
        if (cmd("driver disable", &d) == asynSuccess && d.ok) {
          offPending_ = 0;
          weDisabled_ = 1;
          settling_ = 1;
          epicsTimeGetCurrent(&settleAt_);
          epicsTimeAddSeconds(&settleAt_, ZABER_SETTLE_S);
        } else if (++offFailures_ >= ZABER_OFF_TRIES) {
          offPending_ = 0;
          moving_ = 0;
          asynPrint(pasynUser_, ASYN_TRACE_ERROR,
                    "%s:poll: device %d axis %d: driver disable failed %d times: move reported done, driver left on\n",
                    driverName, device_, axis_, offFailures_);
        } else {
          asynPrint(pasynUser_, ASYN_TRACE_ERROR, "%s:poll: device %d axis %d: driver disable failed (try %d)\n",
                    driverName, device_, axis_, offFailures_);
        }
      }
    }
    setIntegerParam(pC_->motorStatusProblem_, problem);
  }

  if (fail) {
    if (failures_ < ZABER_POLL_FAILURES) failures_++;
    /* re-armed only by a poll that asked and got nothing: the poll after a
     * backoff always asks */
    if (asked && failures_ >= ZABER_POLL_FAILURES) backoff_ = ZABER_BACKOFF_POLLS - 1;
  } else {
    failures_ = 0;
  }
  if (failures_ >= ZABER_POLL_FAILURES) setIntegerParam(pC_->motorStatusProblem_, 1);
  *moving = (busy_ || moving_) ? true : false;
  setIntegerParam(pC_->motorStatusDone_, *moving ? 0 : 1);
  setIntegerParam(pC_->motorStatusMoving_, *moving ? 1 : 0);
  setIntegerParam(pC_->motorStatusHomed_, homed_);
  setIntegerParam(pC_->motorStatusPowerOn_, driverOn_);
  setIntegerParam(pC_->motorStatusCommsError_, fail);
  if (zaberIocRunning && !statusSentRunning_) {
    statusSentRunning_ = 1;
    statusChanged_ = 1;                  /* the whole status, once, now that the record listens */
  }
  callParamCallbacks();
  return failures_ >= ZABER_POLL_FAILURES ? asynError : asynSuccess;
}

/* ------------------------------------------------------------- iocsh */

/** ZaberCreateController(portName, serialPortName, numAxes, movingPollMs, idlePollMs) */
extern "C" int ZaberCreateController(const char *portName, const char *serialPortName, int numAxes,
                                     int movingPollMs, int idlePollMs)
{
  if (numAxes < 1 || numAxes > MAX_ZABER_AXES) {
    printf("ZaberCreateController: numAxes %d is not 1..%d\n", numAxes, MAX_ZABER_AXES);
    return asynError;
  }
  new ZaberController(portName, serialPortName, numAxes, movingPollMs / 1000., idlePollMs / 1000.);
  return asynSuccess;
}

/** ZaberAxisConfig(portName, motor, device, axis, speedArgs, allowHome, powerPolicy, powerOffDelay,
  *                 allowUnreferenced, allowClearWarnings) */
extern "C" int ZaberAxisConfig(const char *portName, int motor, int device, int axis, int speedArgs,
                               int allowHome, const char *powerPolicy, const char *powerOffDelay,
                               int allowUnreferenced, int allowClearWarnings)
{
  ZaberController *pC = static_cast<ZaberController *>(findAsynPortDriver(portName));
  int policy;
  if (!pC) {
    printf("ZaberAxisConfig: no Zaber controller port '%s'\n", portName);
    return asynError;
  }
  ZaberAxis *pAxis = (motor >= 1 && motor <= MAX_ZABER_AXES) ? pC->getAxis(motor - 1) : NULL;
  if (!pAxis) {
    printf("ZaberAxisConfig: %s has no motor %d\n", portName, motor);
    return asynError;
  }
  if (device < 1 || device > 99 || axis < 1 || axis > 9) {
    printf("ZaberAxisConfig: device %d axis %d: a device is 1..99 (never 0, the broadcast), an axis 1..9\n",
           device, axis);
    return asynError;
  }
  for (int i = 0; i < MAX_ZABER_AXES; i++) {
    ZaberAxis *other = (i == motor - 1) ? NULL : pC->getAxis(i);
    if (other && other->isAt(device, axis)) {
      printf("ZaberAxisConfig: %s motor %d is already device %d axis %d: two records on one stage refused\n",
             portName, i + 1, device, axis);
      return asynError;
    }
  }
  if (!powerPolicy || !*powerPolicy || !strcmp(powerPolicy, "leave")) policy = ZABER_POWER_LEAVE;
  else if (!strcmp(powerPolicy, "on_for_move")) policy = ZABER_POWER_ON_FOR_MOVE;
  else {
    printf("ZaberAxisConfig: power policy '%s' is not leave or on_for_move\n", powerPolicy);
    return asynError;
  }
  double offDelay = powerOffDelay ? strtod(powerOffDelay, NULL) : 0.0;
  if (offDelay < 0) offDelay = 0;
  pC->lock();                            /* the poller is already running */
  pAxis->configure(device, axis, speedArgs, allowHome, policy, offDelay, allowUnreferenced, allowClearWarnings);
  /* One poll now, so the record initialises from the real position: the
   * poller's first poll ran before this axis had an address and sent nothing. */
  {
    bool moving;
    pAxis->poll(&moving);
  }
  pC->unlock();
  return asynSuccess;
}

static const iocshArg createArg0 = {"Port name", iocshArgString};
static const iocshArg createArg1 = {"Serial port name", iocshArgString};
static const iocshArg createArg2 = {"Number of axes", iocshArgInt};
static const iocshArg createArg3 = {"Moving poll period (ms)", iocshArgInt};
static const iocshArg createArg4 = {"Idle poll period (ms)", iocshArgInt};
static const iocshArg * const createArgs[] = {&createArg0, &createArg1, &createArg2, &createArg3, &createArg4};
static const iocshFuncDef createDef = {"ZaberCreateController", 5, createArgs};
static void createCallFunc(const iocshArgBuf *args)
{
  ZaberCreateController(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].ival);
}

static const iocshArg axisArg0 = {"Port name", iocshArgString};
static const iocshArg axisArg1 = {"Motor (1-based record axis)", iocshArgInt};
static const iocshArg axisArg2 = {"Device address (1-99)", iocshArgInt};
static const iocshArg axisArg3 = {"Device axis (1-9)", iocshArgInt};
static const iocshArg axisArg4 = {"Speed arguments on move abs (1: firmware 7.25+)", iocshArgInt};
static const iocshArg axisArg5 = {"Allow home (1/0)", iocshArgInt};
static const iocshArg axisArg6 = {"Power policy (leave, on_for_move)", iocshArgString};
static const iocshArg axisArg7 = {"Power off delay (s)", iocshArgString};
static const iocshArg axisArg8 = {"Allow moves without a reference, WR (1/0)", iocshArgInt};
static const iocshArg axisArg9 = {"Allow warnings clear (1/0)", iocshArgInt};
static const iocshArg * const axisArgs[] = {&axisArg0, &axisArg1, &axisArg2, &axisArg3, &axisArg4,
                                            &axisArg5, &axisArg6, &axisArg7, &axisArg8, &axisArg9};
static const iocshFuncDef axisDef = {"ZaberAxisConfig", 10, axisArgs};
static void axisCallFunc(const iocshArgBuf *args)
{
  ZaberAxisConfig(args[0].sval, args[1].ival, args[2].ival, args[3].ival, args[4].ival, args[5].ival,
                  args[6].sval, args[7].sval, args[8].ival, args[9].ival);
}

static void ZaberRegister(void)
{
  iocshRegister(&createDef, createCallFunc);
  iocshRegister(&axisDef, axisCallFunc);
  initHookRegister(zaberInitHook);
}

extern "C" {
epicsExportRegistrar(ZaberRegister);
}

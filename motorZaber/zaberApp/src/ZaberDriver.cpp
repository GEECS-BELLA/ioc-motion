/* ZaberDriver.cpp
 *
 * Zaber ASCII devices as an asynMotorController. See ZaberDriver.h and
 * ioc-motion's README, "Zaber (ASCII)".
 *
 * What goes down the line (Zaber ASCII protocol manual), every command
 * addressed to one device and one axis, "/<device> <axis> <command>" + LF;
 * never the broadcast address 0, never "set", "renumber" or "system ...":
 *   creation:            nothing
 *   once per axis:       get version, get deviceid (device.id if rejected),
 *                        get system.serial (device scope, axis 0);
 *                        get peripheralid (peripheral.id), get maxspeed,
 *                        get limit.min, get limit.max (axis scope)
 *   every poll:          get pos; and warnings (the full flag list, a read)
 *                        when the reply's top flag is not "--"
 *   writes:              move abs <pos> [<speed> <accel>] (a move);
 *                        stop (only to a BUSY axis); home (only with
 *                        allow_home); driver enable / driver disable (only
 *                        with power_policy on_for_move)
 * A reply is "@<dd> <a> OK|RJ IDLE|BUSY <flag> <data>" CR LF; alerts ("!")
 * and info lines ("#") are skipped, and a reply from another address or axis
 * is skipped too.
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <ctype.h>

#include <iocsh.h>
#include <epicsThread.h>
#include <asynOctetSyncIO.h>

#include <asynMotorController.h>
#include <asynMotorAxis.h>

#include <epicsExport.h>
#include "ZaberDriver.h"

static const char *driverName = "ZaberDriver";

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
                        ASYN_CANBLOCK | ASYN_MULTIDEVICE, 1, 0, 0)
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

/** Parse one line; 1 if it is a reply ("@..."), 0 otherwise. */
static int parseReply(const char *line, ZaberReply *r)
{
  char okWord[4], statusWord[6], flag[4];
  int consumed = 0;
  if (line[0] != '@') return 0;
  if (sscanf(line, "@%d %d %3s %5s %3s %n", &r->device, &r->axis, okWord, statusWord, flag, &consumed) < 5)
    return 0;
  if (strcmp(okWord, "OK") && strcmp(okWord, "RJ")) return 0;      /* a message id would sit here: we send none */
  if (strcmp(statusWord, "IDLE") && strcmp(statusWord, "BUSY")) return 0;
  if (strlen(flag) != 2) return 0;
  r->ok = !strcmp(okWord, "OK");
  r->busy = !strcmp(statusWord, "BUSY");
  strcpy(r->flag, flag);
  snprintf(r->data, sizeof(r->data), "%s", consumed ? line + consumed : "");
  /* the checksum (":CC", comm.checksum 1) is not ours to enable; drop one if present */
  char *colon = strrchr(r->data, ':');
  if (colon && strlen(colon) == 3) *colon = '\0';
  for (size_t n = strlen(r->data); n > 0 && isspace((unsigned char)r->data[n - 1]); n--) r->data[n - 1] = '\0';
  return 1;
}

/** One command to one device and axis, and its reply. Lines that are not
  * that device's and axis's reply (alerts, info, another device's reply) are
  * skipped, up to ZABER_MAX_LINES. */
asynStatus ZaberController::exchange(int device, int axis, const char *command, ZaberReply *r)
{
  char out[ZABER_REPLY_LEN], line[ZABER_REPLY_LEN];
  size_t nwrite = 0, nread = 0;
  int eom = 0;
  asynStatus status;

  if (device < 1 || device > 99) return asynError;     /* never the broadcast address */
  snprintf(out, sizeof(out), "/%d %d %s", device, axis, command);
  pasynOctetSyncIO->flush(pasynUserController_);
  status = pasynOctetSyncIO->write(pasynUserController_, out, strlen(out), ZABER_TIMEOUT, &nwrite);
  if (status) return status;
  for (int i = 0; i < ZABER_MAX_LINES; i++) {
    status = pasynOctetSyncIO->read(pasynUserController_, line, sizeof(line) - 1, ZABER_TIMEOUT, &nread, &eom);
    if (status) return status;
    line[nread] = '\0';
    char *p = line;
    while (*p && isspace((unsigned char)*p)) p++;
    if (!parseReply(p, r)) continue;
    if (r->device != device || r->axis != axis) continue;
    return asynSuccess;
  }
  return asynError;
}

/* ---------------------------------------------------------------- axis */

ZaberAxis::ZaberAxis(ZaberController *pC, int axisNo)
  : asynMotorAxis(pC, axisNo),
    pC_(pC), device_(0), axis_(1), speedArgs_(0), allowHome_(0), powerPolicy_(ZABER_POWER_LEAVE),
    powerOffDelay_(0.0), identified_(0), firmware_(0), moving_(0), busy_(0), homed_(0), failures_(0),
    driverOn_(1), offPending_(0), lastPos_(0), nflags_(0)
{
  setIntegerParam(pC_->motorStatusHasEncoder_, 0);
  setIntegerParam(pC_->motorStatusGainSupport_, 0);
  pC_->setStringParam(axisNo_, pC_->ZaberFlag_, "");
  pC_->setStringParam(axisNo_, pC_->ZaberWarnings_, "");
  pC_->setStringParam(axisNo_, pC_->ZaberVersion_, "");
  pC_->setIntegerParam(axisNo_, pC_->ZaberDeviceId_, 0);
  pC_->setIntegerParam(axisNo_, pC_->ZaberPeripheralId_, 0);
  pC_->setIntegerParam(axisNo_, pC_->ZaberSerial_, 0);
  pC_->setDoubleParam(axisNo_, pC_->ZaberMaxspeed_, 0.0);
  pC_->setDoubleParam(axisNo_, pC_->ZaberLimitMin_, 0.0);
  pC_->setDoubleParam(axisNo_, pC_->ZaberLimitMax_, 0.0);
  callParamCallbacks();
}

void ZaberAxis::configure(int device, int axis, int speedArgs, int allowHome, int powerPolicy, double powerOffDelay)
{
  device_ = device;
  axis_ = axis;
  speedArgs_ = speedArgs ? 1 : 0;
  allowHome_ = allowHome ? 1 : 0;
  powerPolicy_ = powerPolicy;
  powerOffDelay_ = powerOffDelay;
}

void ZaberAxis::report(FILE *fp, int level)
{
  if (level > 0) {
    fprintf(fp, "  device %d axis %d: firmware %d.%02d, speed args %d, home %s, power %s (off after %g s), "
                "moving %d, busy %d, homed %d, driver on %d, last pos %ld, flags %d\n",
            device_, axis_, firmware_ / 100, firmware_ % 100, speedArgs_, allowHome_ ? "allowed" : "refused",
            powerPolicy_ == ZABER_POWER_ON_FOR_MOVE ? "on_for_move" : "leave", powerOffDelay_,
            moving_, busy_, homed_, driverOn_, lastPos_, nflags_);
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

/** The identity and the settings the record's configuration is checked
  * against, read once: all "get", nothing else. A rejected or silent get
  * leaves its reading empty (older firmware lacks some settings). */
void ZaberAxis::identify()
{
  ZaberReply r;
  if (deviceCmd("get version", &r) == asynSuccess && r.ok) {
    int major = 0, minor = 0;
    if (sscanf(r.data, "%d.%d", &major, &minor) == 2) firmware_ = major * 100 + minor;
    pC_->setStringParam(axisNo_, pC_->ZaberVersion_, r.data);
  } else {
    pC_->setStringParam(axisNo_, pC_->ZaberVersion_, "no reply");
  }
  if ((deviceCmd("get deviceid", &r) == asynSuccess && r.ok) ||
      (deviceCmd("get device.id", &r) == asynSuccess && r.ok)) {
    pC_->setIntegerParam(axisNo_, pC_->ZaberDeviceId_, atoi(r.data));
  }
  if (deviceCmd("get system.serial", &r) == asynSuccess && r.ok) {
    pC_->setIntegerParam(axisNo_, pC_->ZaberSerial_, atoi(r.data));
  }
  if ((cmd("get peripheralid", &r) == asynSuccess && r.ok) ||
      (cmd("get peripheral.id", &r) == asynSuccess && r.ok)) {
    pC_->setIntegerParam(axisNo_, pC_->ZaberPeripheralId_, atoi(r.data));
  }
  if (cmd("get maxspeed", &r) == asynSuccess && r.ok) {
    pC_->setDoubleParam(axisNo_, pC_->ZaberMaxspeed_, strtod(r.data, NULL));
  }
  if (cmd("get limit.min", &r) == asynSuccess && r.ok) {
    pC_->setDoubleParam(axisNo_, pC_->ZaberLimitMin_, strtod(r.data, NULL));
  }
  if (cmd("get limit.max", &r) == asynSuccess && r.ok) {
    pC_->setDoubleParam(axisNo_, pC_->ZaberLimitMax_, strtod(r.data, NULL));
  }
  identified_ = 1;
}

/** "warnings": "<count> <flag> <flag> ..." in the data. Without a reply,
  * the top flag alone. */
asynStatus ZaberAxis::readWarnings(const char *topFlag)
{
  ZaberReply r;
  nflags_ = 0;
  if (strcmp(topFlag, "--") == 0) return asynSuccess;
  asynStatus status = cmd("warnings", &r);
  if (status == asynSuccess && r.ok) {
    char *save = NULL, *tok = strtok_r(r.data, " ", &save);    /* the count */
    while (tok && (tok = strtok_r(NULL, " ", &save)) != NULL && nflags_ < ZABER_MAX_FLAGS) {
      if (strlen(tok) == 2) { strcpy(flags_[nflags_], tok); nflags_++; }
    }
  }
  if (nflags_ == 0) { strcpy(flags_[0], topFlag); nflags_ = 1; }
  return status;
}

int ZaberAxis::hasFlag(const char *flag) const
{
  for (int i = 0; i < nflags_; i++) if (strcmp(flags_[i], flag) == 0) return 1;
  return 0;
}

/** Faults (F...) are problems, except the driver being off: FO (7.11 and
  * up), and FD on 7.10 and below, where FD meant "driver disabled" (it means
  * a temperature or current error from 7.11). */
int ZaberAxis::isFault(const char *flag) const
{
  if (flag[0] != 'F') return 0;
  if (strcmp(flag, "FO") == 0) return 0;
  if (strcmp(flag, "FD") == 0 && firmware_ > 0 && firmware_ <= 710) return 0;
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
  return status;
}

/** move abs <target> [<speed> <accel>]: always absolute (the record's
  * relative moves are added to the last position read). The speed and accel
  * arguments (firmware 7.25 and up) only with speed_args; otherwise the
  * device's own maxspeed and accel settings govern, and nothing is "set". */
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
  status = cmd("get pos", &r);
  if (status || !r.ok) return asynError;
  lastPos_ = strtol(r.data, NULL, 10);
  if (r.busy) {
    asynPrint(pasynUser_, ASYN_TRACE_ERROR,
              "%s:%s: device %d axis %d: busy (a move, a home or the knob): move refused\n",
              driverName, functionName, device_, axis_);
    return asynError;
  }
  long target = relative ? lastPos_ + NINT(position) : NINT(position);
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
    if (powerPolicy_ == ZABER_POWER_ON_FOR_MOVE) {     /* enabled for a move that never ran: off again */
      epicsTimeGetCurrent(&offAt_);
      offPending_ = 1;
    }
    return asynError;
  }
  moving_ = 1;
  busy_ = 1;
  setIntegerParam(pC_->motorStatusDirection_, target >= lastPos_ ? 1 : 0);
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
  * sets the reference (WR clears). Only with allow_home. */
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
    return asynError;
  }
  moving_ = 1;
  busy_ = 1;
  setIntegerParam(pC_->motorStatusDone_, 0);
  setIntegerParam(pC_->motorStatusMoving_, 1);
  callParamCallbacks();
  return asynSuccess;
}

/** "stop" (decelerate) only to an axis a fresh "get pos" says is BUSY: the
  * record sends STOP to idle axes too. */
asynStatus ZaberAxis::stop(double acceleration)
{
  ZaberReply r;
  if (!device_) return asynSuccess;
  if (cmd("get pos", &r) == asynSuccess && r.ok && r.busy) {
    asynStatus status = cmd("stop", &r);
    if (status == asynSuccess && !r.ok) {
      asynPrint(pasynUser_, ASYN_TRACE_ERROR, "%s:stop: device %d axis %d: rejected (%s)\n",
                driverName, device_, axis_, r.data);
      return asynError;
    }
    return status;
  }
  asynPrint(pasynUser_, ASYN_TRACE_FLOW, "%s:stop: device %d axis %d: not busy; nothing sent\n",
            driverName, device_, axis_);
  return asynSuccess;
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
  int fail = 0;
  char text[ZABER_MAX_FLAGS * 3 + 1];

  if (!device_) { *moving = false; return asynSuccess; }
  if (!identified_) identify();

  if (cmd("get pos", &r) == asynSuccess && r.ok) {
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
      pC_->setStringParam(axisNo_, pC_->ZaberFlag_, r.flag);
      readWarnings(r.flag);
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
    homed_ = !hasFlag("WR");
    driverOn_ = !(hasFlag("FO") || (hasFlag("FD") && !isFault("FD")));
    if (moving_ && !busy_) {
      moving_ = 0;
      if (powerPolicy_ == ZABER_POWER_ON_FOR_MOVE) {
        epicsTimeGetCurrent(&offAt_);
        epicsTimeAddSeconds(&offAt_, powerOffDelay_);
        offPending_ = 1;
      }
    }
    if (offPending_ && !busy_) {
      epicsTimeStamp now;
      epicsTimeGetCurrent(&now);
      if (epicsTimeDiffInSeconds(&now, &offAt_) >= 0) {
        ZaberReply d;
        offPending_ = 0;
        if (cmd("driver disable", &d) != asynSuccess || !d.ok) {
          asynPrint(pasynUser_, ASYN_TRACE_ERROR, "%s:poll: device %d axis %d: driver disable failed\n",
                    driverName, device_, axis_);
        }
      }
    }
    setIntegerParam(pC_->motorStatusProblem_, problem);
  }

  if (fail) failures_++; else failures_ = 0;
  if (failures_ >= ZABER_POLL_FAILURES) setIntegerParam(pC_->motorStatusProblem_, 1);
  *moving = (busy_ || moving_) ? true : false;
  setIntegerParam(pC_->motorStatusDone_, *moving ? 0 : 1);
  setIntegerParam(pC_->motorStatusMoving_, *moving ? 1 : 0);
  setIntegerParam(pC_->motorStatusHomed_, homed_);
  setIntegerParam(pC_->motorStatusPowerOn_, driverOn_);
  setIntegerParam(pC_->motorStatusCommsError_, fail);
  /* No limit switches as such: the device stops at limit.min / limit.max by
   * itself and rejects targets outside them; FE (limit error) is a fault. */
  setIntegerParam(pC_->motorStatusLowLimit_, 0);
  setIntegerParam(pC_->motorStatusHighLimit_, 0);
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

/** ZaberAxisConfig(portName, motor, device, axis, speedArgs, allowHome, powerPolicy, powerOffDelay) */
extern "C" int ZaberAxisConfig(const char *portName, int motor, int device, int axis, int speedArgs,
                               int allowHome, const char *powerPolicy, const char *powerOffDelay)
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
  if (!powerPolicy || !*powerPolicy || !strcmp(powerPolicy, "leave")) policy = ZABER_POWER_LEAVE;
  else if (!strcmp(powerPolicy, "on_for_move")) policy = ZABER_POWER_ON_FOR_MOVE;
  else {
    printf("ZaberAxisConfig: power policy '%s' is not leave or on_for_move\n", powerPolicy);
    return asynError;
  }
  double offDelay = powerOffDelay ? strtod(powerOffDelay, NULL) : 0.0;
  if (offDelay < 0) offDelay = 0;
  pC->lock();                            /* the poller is already running */
  pAxis->configure(device, axis, speedArgs, allowHome, policy, offDelay);
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
static const iocshArg * const axisArgs[] = {&axisArg0, &axisArg1, &axisArg2, &axisArg3,
                                            &axisArg4, &axisArg5, &axisArg6, &axisArg7};
static const iocshFuncDef axisDef = {"ZaberAxisConfig", 8, axisArgs};
static void axisCallFunc(const iocshArgBuf *args)
{
  ZaberAxisConfig(args[0].sval, args[1].ival, args[2].ival, args[3].ival, args[4].ival, args[5].ival,
                  args[6].sval, args[7].sval);
}

static void ZaberRegister(void)
{
  iocshRegister(&createDef, createCallFunc);
  iocshRegister(&axisDef, axisCallFunc);
}

extern "C" {
epicsExportRegistrar(ZaberRegister);
}

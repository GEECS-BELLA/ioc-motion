/* VelmexDriver.cpp
 *
 * Velmex VXM stepping-motor controller with a Velmex VRO encoder readout on a
 * second serial line, as an asynMotorController. See VelmexDriver.h and
 * ioc-motion's README, "Velmex VXM + VRO".
 *
 * What goes down the lines (VXM command summary VXM-CS 12-29-04; VRO
 * reference manual 1.30):
 *   VXM queries, every poll:  V (status letter), X/Y/Z/T (motor 1..4 absolute
 *                             position register, "+0001200" + CR, readable
 *                             while indexing), ? (limit inputs, one raw byte)
 *   VXM query, once:          getD0 (firmware, firmware 1.20 and up)
 *   VXM writes:               F (On-Line, echo off) on the first poll: the VXM
 *                             is On-Line while the IOC runs, since in Local
 *                             mode it answers V and X only (?, getD0 and the
 *                             get... reads are silent and D is the digitizer
 *                             dump: U_Velmex's VXM, 2026-10-29); C,<program>,R
 *                             for every move and home; D (decelerate to a
 *                             stop) or K (kill a program); Q (back to Local
 *                             mode) at a clean IOC exit
 *   VRO queries, every poll:  V (status letter), X/Y (encoder 1/2 displayed
 *                             value in its primary unit + CR)
 *   VRO writes:               none. The readout answers X in Display mode
 *                             without any mode command; E/F would put it in
 *                             Setup mode and blank its buttons.
 * A program's end is the single character "^", sent by the VXM when it is
 * done, with no terminator: it may land anywhere in the reply stream, so
 * every VXM reply read skips it, the raw limit byte included (readLimits).
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

#include <iocsh.h>
#include <epicsExit.h>
#include <epicsThread.h>
#include <asynOctetSyncIO.h>

#include <asynMotorController.h>
#include <asynMotorAxis.h>

#include <epicsExport.h>
#include "VelmexDriver.h"

static const char *driverName = "VelmexDriver";

#define NINT(f) (long)((f) > 0 ? (f) + 0.5 : (f) - 0.5)

/* X, Y, Z, T: "send current position of motor 1..4 to host" */
static const char VXM_POSITION_CMD[MAX_VELMEX_AXES + 1] = "XYZT";
/* X, Y: "send displayed encoder 1/2 primary position to host" */
static const char VRO_POSITION_CMD[MAX_VELMEX_ENCODERS + 1] = "XY";

static void velmexAtExit(void *arg)
{
  static_cast<VelmexController *>(arg)->goOffline();
}

/** The controller: one VXM line, one VRO line.
  * \param[in] portName          The asyn port name of this driver
  * \param[in] vxmPortName       The asyn octet port of the VXM's serial line (no input EOS)
  * \param[in] vroPortName       The asyn octet port of the readout's serial line (no input EOS); "" = no readout
  * \param[in] numAxes           Motors on the VXM, 1..4 (motor n is axis n-1)
  * \param[in] movingPollPeriod  Seconds between polls while an axis moves
  * \param[in] idlePollPeriod    Seconds between polls while idle
  */
VelmexController::VelmexController(const char *portName, const char *vxmPortName, const char *vroPortName,
                                   int numAxes, double movingPollPeriod, double idlePollPeriod)
  : asynMotorController(portName, numAxes, NUM_VELMEX_PARAMS,
                        asynOctetMask,           /* the status letters and the firmware string */
                        asynOctetMask,
                        ASYN_CANBLOCK | ASYN_MULTIDEVICE,
                        1, 0, 0),
    pasynUserEncoder_(NULL), online_(0), identified_(0),
    vxmStatus_('?'), vroStatus_('?'), limitByte_(0xFF), pollStatus_(asynSuccess), vroPollStatus_(asynSuccess)
{
  static const char *functionName = "VelmexController";
  asynStatus status;

  createParam(VelmexVxmStatusString, asynParamOctet, &VelmexVxmStatus_);
  createParam(VelmexVroStatusString, asynParamOctet, &VelmexVroStatus_);
  createParam(VelmexLimitByteString, asynParamInt32, &VelmexLimitByte_);
  createParam(VelmexVxmVersionString, asynParamOctet, &VelmexVxmVersion_);
  setStringParam(VelmexVxmStatus_, "");
  setStringParam(VelmexVroStatus_, "");
  setIntegerParam(VelmexLimitByte_, 0xFF);
  setStringParam(VelmexVxmVersion_, "");

  status = pasynOctetSyncIO->connect(vxmPortName, 0, &pasynUserController_, NULL);
  if (status) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
              "%s:%s: cannot connect to VXM port %s\n", driverName, functionName, vxmPortName);
  }
  if (vroPortName && *vroPortName) {
    status = pasynOctetSyncIO->connect(vroPortName, 0, &pasynUserEncoder_, NULL);
    if (status) {
      asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: cannot connect to VRO port %s\n", driverName, functionName, vroPortName);
      pasynUserEncoder_ = NULL;
    }
  }
  for (int axis = 0; axis < numAxes; axis++) new VelmexAxis(this, axis);

  epicsAtExit(velmexAtExit, this);
  startPoller(movingPollPeriod, idlePollPeriod, 2);
}

/** Return the VXM to Local mode at IOC exit, as the LabVIEW driver did when
  * it closed: its front-panel jog buttons work again. Only when this IOC put
  * it On-Line; nothing is sent otherwise. */
void VelmexController::goOffline()
{
  if (!online_) return;
  lock();
  writeVxm("Q");
  online_ = 0;
  unlock();
}

void VelmexController::report(FILE *fp, int level)
{
  fprintf(fp, "Velmex VXM driver %s: numAxes=%d, readout %s, online=%d, VXM status '%c', VRO status '%c', "
              "limit byte 0x%02X, moving poll %g s, idle poll %g s\n",
          portName, numAxes_, pasynUserEncoder_ ? "yes" : "none", online_, vxmStatus_, vroStatus_,
          limitByte_, movingPollPeriod_, idlePollPeriod_);
  asynMotorController::report(fp, level);
}

VelmexAxis* VelmexController::getAxis(asynUser *pasynUser)
{
  return static_cast<VelmexAxis*>(asynMotorController::getAxis(pasynUser));
}

VelmexAxis* VelmexController::getAxis(int axisNo)
{
  return static_cast<VelmexAxis*>(asynMotorController::getAxis(axisNo));
}

/** One command and its reply on one line, byte by byte: the two Velmex
  * devices answer with a CR-terminated value, a single letter (V) or a single
  * raw byte (?), and neither frames its replies the same way twice, so the
  * port carries no input terminator and the driver reads what each command
  * is documented to return.
  * \param[in] terminated  1: read up to CR (or LF), dropping a leading one; 0: one byte
  * \param[in] skipCaret   1: drop '^' bytes (the VXM's ready prompt, which arrives asynchronously)
  */
asynStatus VelmexController::exchange(asynUser *pasynUser, const char *cmd, char *reply, size_t maxLen,
                                      int terminated, int skipCaret, double timeout)
{
  size_t nwrite = 0, nread = 0, n = 0;
  int eom = 0;
  asynStatus status;
  char c;

  reply[0] = '\0';
  if (!pasynUser) return asynError;
  pasynOctetSyncIO->flush(pasynUser);
  status = pasynOctetSyncIO->write(pasynUser, cmd, strlen(cmd), timeout, &nwrite);
  if (status) return status;
  for (int i = 0; i < 2 * VELMEX_REPLY_LEN; i++) {
    status = pasynOctetSyncIO->read(pasynUser, &c, 1, timeout, &nread, &eom);
    if (status) return status;
    if (nread == 0) return asynTimeout;
    if (skipCaret && c == '^') continue;
    if (!terminated) {
      reply[0] = c;
      reply[1] = '\0';
      return asynSuccess;
    }
    if (c == '\r' || c == '\n') {
      if (n == 0) continue;
      break;
    }
    if (n < maxLen - 1) reply[n++] = c;
  }
  reply[n] = '\0';
  return n ? asynSuccess : asynError;
}

/** A VXM command with no reply of its own (F, Q, D, K, a program). */
asynStatus VelmexController::writeVxm(const char *cmd)
{
  size_t nwrite = 0;
  return pasynOctetSyncIO->write(pasynUserController_, cmd, strlen(cmd), VELMEX_VXM_TIMEOUT, &nwrite);
}

/** V: R ready, B busy (On-Line); J idle, b moving (Local mode). */
asynStatus VelmexController::readStatus()
{
  char reply[VELMEX_REPLY_LEN];
  asynStatus status = exchange(pasynUserController_, "V", reply, sizeof(reply), 0, 1, VELMEX_VXM_TIMEOUT);
  if (status == asynSuccess && strchr("RBJb", reply[0]) == NULL) status = asynError;
  if (status == asynSuccess) {
    vxmStatus_ = reply[0];
    if (vxmStatus_ == 'R' || vxmStatus_ == 'B') online_ = 1;
    else online_ = 0;           /* the VXM answers J/b only in Local mode: a power cycle, or Q */
  } else {
    vxmStatus_ = '?';
  }
  return status;
}

/** ?: one raw byte, a bit per limit input (bit 0 = motor 1 negative, bit 1 =
  * motor 1 positive, ... bit 7 = motor 4 positive); 0 = activated, 1 = not
  * activated or not connected (manual: normally closed switches, 255 = none).
  * The ready prompt ^ (0x5E) is skipped here too: a genuine 0x5E needs motor
  * 1's negative and motors 3 and 4's positive limits activated at once,
  * impossible on a one- or two-motor VXM (unconnected inputs read 1), while a
  * prompt read as the byte would end a negative move on a false LLS with no
  * retry (the record skips the retry on a limit in the move's direction). */
asynStatus VelmexController::readLimits()
{
  char reply[VELMEX_REPLY_LEN];
  asynStatus status = exchange(pasynUserController_, "?", reply, sizeof(reply), 0, 1, VELMEX_VXM_TIMEOUT);
  if (status == asynSuccess) limitByte_ = (unsigned char)reply[0];
  return status;
}

/** X/Y/Z/T: the motor's absolute position register, "+0001200" (CR). */
asynStatus VelmexController::readSteps(int motor, long *steps)
{
  char reply[VELMEX_REPLY_LEN], cmd[2] = {VXM_POSITION_CMD[motor - 1], '\0'};
  char *p, *end;
  asynStatus status = exchange(pasynUserController_, cmd, reply, sizeof(reply), 1, 1, VELMEX_VXM_TIMEOUT);
  if (status) return status;
  for (p = reply; *p && isspace((unsigned char)*p); p++) {}
  *steps = strtol(p, &end, 10);
  if (end == p) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s: VXM %c answered '%s', not a position\n",
              driverName, cmd[0], reply);
    return asynError;
  }
  return asynSuccess;
}

/** V on the readout: D display mode (answers X), P power-up menu, S setup, F front-panel setup. */
asynStatus VelmexController::readVroStatus()
{
  char reply[VELMEX_REPLY_LEN];
  asynStatus status = exchange(pasynUserEncoder_, "V", reply, sizeof(reply), 0, 0, VELMEX_VRO_TIMEOUT);
  if (status == asynSuccess && strchr("DPSF", reply[0]) == NULL) status = asynError;
  char next = status == asynSuccess ? reply[0] : '?';
  if (next != vroStatus_ && next != 'D') {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
              "%s: readout status '%c' (D = display mode, the one that answers positions)\n", driverName, next);
  }
  vroStatus_ = next;
  return status;
}

/** X/Y on the readout: the displayed value in its primary unit, e.g. "  10.000" (CR). */
asynStatus VelmexController::readEncoder(int encoder, double *value)
{
  char reply[VELMEX_REPLY_LEN], cmd[2] = {VRO_POSITION_CMD[encoder - 1], '\0'};
  char *p, *end;
  asynStatus status = exchange(pasynUserEncoder_, cmd, reply, sizeof(reply), 1, 0, VELMEX_VRO_TIMEOUT);
  if (status) return status;
  for (p = reply; *p && isspace((unsigned char)*p); p++) {}
  *value = strtod(p, &end);
  if (end == p) {
    asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s: readout %c answered '%s', not a value\n",
              driverName, cmd[0], reply);
    return asynError;
  }
  return asynSuccess;
}

/** getD0 once: the firmware version (1.20 and up answer it; older firmware is silent). */
void VelmexController::identify()
{
  char reply[VELMEX_REPLY_LEN];
  identified_ = 1;
  if (exchange(pasynUserController_, "getD0,", reply, sizeof(reply), 1, 1, VELMEX_VXM_TIMEOUT) == asynSuccess) {
    setStringParam(VelmexVxmVersion_, reply);
  } else {
    setStringParam(VelmexVxmVersion_, "no reply to getD0");
  }
}

/** The controller-wide part of each poll, before the axes: V and ? on the
  * VXM, V on the readout. Each axis then reads its own position(s). */
asynStatus VelmexController::poll()
{
  char s[2] = {'\0', '\0'};
  asynStatus s1 = readStatus();
  /* On-Line while the IOC runs: in Local mode the VXM answers V and X only
   * (the limit byte, getD0 and the get... reads are silent, and D dumps
   * positions instead of stopping), so the first poll sends F and nothing
   * sends Q until a clean exit. The front-panel jog buttons are off meanwhile. */
  if (s1 == asynSuccess && !online_) {
    if (writeVxm("F") == asynSuccess) { online_ = 1; vxmStatus_ = busy() ? 'B' : 'R'; }
  }
  if (!identified_ && online_) identify();
  asynStatus s2 = readLimits();
  vroPollStatus_ = pasynUserEncoder_ ? readVroStatus() : asynSuccess;
  pollStatus_ = (s1 || s2) ? asynError : asynSuccess;   /* the VXM alone: a silent readout must not hide the VXM */
  s[0] = vxmStatus_;
  setStringParam(VelmexVxmStatus_, s);
  s[0] = vroStatus_;
  setStringParam(VelmexVroStatus_, pasynUserEncoder_ ? s : "none");
  setIntegerParam(VelmexLimitByte_, limitByte_);
  callParamCallbacks();
  return asynSuccess;
}

/* ---------------------------------------------------------------- axis */

VelmexAxis::VelmexAxis(VelmexController *pC, int axisNo)
  : asynMotorAxis(pC, axisNo),
    pC_(pC), motor_(axisNo + 1), encoder_(0), eres_(1.0), accelSetting_(0), homeBackoff_(0), limitsActiveLow_(1),
    moving_(0), homing_(0), homed_(0), homeForwards_(0), multiIndex_(0), lastSteps_(0), encoderOk_(0)
{
  setIntegerParam(pC_->motorStatusHasEncoder_, 0);
  setIntegerParam(pC_->motorStatusGainSupport_, 0);
  callParamCallbacks();
}

void VelmexAxis::configure(int encoder, double encoderUnitsPerCount, int accelSetting, int homeBackoff,
                           int limitsActiveLow)
{
  encoder_ = encoder;
  eres_ = encoderUnitsPerCount;
  accelSetting_ = accelSetting;
  homeBackoff_ = homeBackoff;
  limitsActiveLow_ = limitsActiveLow ? 1 : 0;
  setIntegerParam(pC_->motorStatusHasEncoder_, encoder_ ? 1 : 0);
  callParamCallbacks();
}

void VelmexAxis::report(FILE *fp, int level)
{
  if (level > 0) {
    fprintf(fp, "  motor %d: encoder %d (ERES %g), accel setting %d, home backoff %d steps, "
                "moving %d, homing %d, homed %d, last X %ld\n",
            motor_, encoder_, eres_, accelSetting_, homeBackoff_, moving_, homing_, homed_, lastSteps_);
  }
  asynMotorAxis::report(fp, level);
}

/** C,<body>,R: clear the current program, store the body, run it. F first
  * when the VXM is not known to be On-Line (never sent, or V answered J/b).
  * Refused while a program runs: the VXM runs one program at a time, so a
  * second motor's move waits for the record to see the first one done. */
asynStatus VelmexAxis::runProgram(const char *body, int multiIndex)
{
  static const char *functionName = "runProgram";
  char cmd[MAX_CONTROLLER_STRING_SIZE];
  asynStatus status;

  if (pC_->busy()) {
    asynPrint(pasynUser_, ASYN_TRACE_ERROR,
              "%s:%s: motor %d: the VXM is busy (status '%c'), program '%s' not sent\n",
              driverName, functionName, motor_, pC_->vxmStatus_, body);
    return asynError;
  }
  if (!pC_->online_ || pC_->local()) {
    status = pC_->writeVxm("F");
    if (status) return status;
    pC_->online_ = 1;
    pC_->vxmStatus_ = 'R';
  }
  snprintf(cmd, sizeof(cmd), "C,%s,R", body);
  status = pC_->writeVxm(cmd);
  if (status) return status;
  moving_ = 1;
  multiIndex_ = multiIndex;
  pC_->vxmStatus_ = 'B';
  setIntegerParam(pC_->motorStatusDone_, 0);
  setIntegerParam(pC_->motorStatusMoving_, 1);
  callParamCallbacks();
  return asynSuccess;
}

/** An index to an absolute step count: S<m>M<speed>[,A<m>M<accel>],IA<m>M<target>.
  * Always the absolute form: the relative form's zero ("I1M0") means "index
  * to the positive limit" and "I1M-0" the negative one, so a zero-length
  * relative move must never reach the wire. The target is clamped to the
  * VXM's register range; the record's limits are the real ones. */
asynStatus VelmexAxis::move(double position, int relative, double minVelocity, double maxVelocity, double acceleration)
{
  static const char *functionName = "move";
  char body[MAX_CONTROLLER_STRING_SIZE];
  long now, target, speed;
  int n;

  if (encoder_ && pC_->pasynUserEncoder_ && !encoderOk_) {
    asynPrint(pasynUser_, ASYN_TRACE_ERROR,
              "%s:%s: motor %d: the readout did not answer on the last poll (status '%c'): no reference, move refused\n",
              driverName, functionName, motor_, pC_->vroStatus_);
    return asynError;
  }
  asynStatus status = pC_->readSteps(motor_, &now);
  if (status) return status;
  lastSteps_ = now;
  target = relative ? now + NINT(position) : NINT(position);
  if (target > VELMEX_MAX_INDEX || target < -VELMEX_MAX_INDEX) {
    asynPrint(pasynUser_, ASYN_TRACE_ERROR, "%s:%s: motor %d: target %ld steps outside the VXM's +/-%ld\n",
              driverName, functionName, motor_, target, VELMEX_MAX_INDEX);
    return asynError;
  }
  speed = NINT(maxVelocity);
  if (speed < 1) speed = 1;
  if (speed > VELMEX_MAX_SPEED) speed = VELMEX_MAX_SPEED;
  n = snprintf(body, sizeof(body), "S%dM%ld,", motor_, speed);
  if (accelSetting_ > 0) n += snprintf(body + n, sizeof(body) - n, "A%dM%d,", motor_, accelSetting_);
  snprintf(body + n, sizeof(body) - n, "IA%dM%ld", motor_, target);
  setIntegerParam(pC_->motorStatusDirection_, target >= now ? 1 : 0);
  homing_ = 0;
  return runProgram(body, 0);
}

/** The VXM has no host-driven continuous move outside a stored program; the
  * record's tweaks (TWF/TWR) are moves and work. */
asynStatus VelmexAxis::moveVelocity(double minVelocity, double maxVelocity, double acceleration)
{
  asynPrint(pasynUser_, ASYN_TRACE_ERROR,
            "%s:moveVelocity: motor %d: jog is not supported on the VXM (use TWF/TWR); nothing sent\n",
            driverName, motor_);
  return asynError;
}

/** Home to a limit switch: S<m>M<speed>,[I<m>M<+/-backoff>,]I<m>M-0 (reverse: the
  * negative limit, the LabVIEW driver's home) or I<m>M0 (forward). With a
  * backoff the motor first moves that many steps away from the limit, so the
  * switch is always approached from the same side. Neither the VXM's position
  * register nor the readout is zeroed. */
asynStatus VelmexAxis::home(double minVelocity, double maxVelocity, double acceleration, int forwards)
{
  char body[MAX_CONTROLLER_STRING_SIZE];
  long speed = NINT(maxVelocity);
  int n;

  if (speed < 1) speed = 1;
  if (speed > VELMEX_MAX_SPEED) speed = VELMEX_MAX_SPEED;
  n = snprintf(body, sizeof(body), "S%dM%ld,", motor_, speed);
  if (homeBackoff_ > 0) {
    n += snprintf(body + n, sizeof(body) - n, "I%dM%s%d,", motor_, forwards ? "-" : "", homeBackoff_);
  }
  snprintf(body + n, sizeof(body) - n, "I%dM%s0", motor_, forwards ? "" : "-");
  setIntegerParam(pC_->motorStatusDirection_, forwards ? 1 : 0);
  homing_ = 1;
  homeForwards_ = forwards ? 1 : 0;
  return runProgram(body, homeBackoff_ > 0);
}

/** D decelerates the running index to a stop, and the program goes on to its
  * next command: enough for a move (one index), not for a home with a
  * backoff (two), which K kills; K stops the motor at once, so it is used
  * only there. */
asynStatus VelmexAxis::stop(double acceleration)
{
  static const char *functionName = "stop";
  asynStatus status = asynSuccess;
  /* Only a program of ours can be stopped. The record sends STOP on an idle
   * axis too ("just in case"), and the idle VXM is in Local mode, where D is
   * the digitizer dump (Appendix H: two position lines), not a stop. */
  if (pC_->online_ && (moving_ || pC_->busy())) {
    /* Our own program: D for a one-index move (the program then ends), K for
     * the two-index home. A program this IOC did not start (an IOC restarted
     * mid-program): K, since D would only end its current index. */
    status = pC_->writeVxm((multiIndex_ || !moving_) ? "K" : "D");
  } else {
    asynPrint(pasynUser_, ASYN_TRACE_FLOW, "%s:%s: motor %d: no program running; nothing sent\n",
              driverName, functionName, motor_);
  }
  homing_ = 0;
  return status;
}

/** The VXM can only null its registers (N, IA<m>M-0), never take a value,
  * and the readout is the position reference: refused, nothing sent. */
asynStatus VelmexAxis::setPosition(double position)
{
  asynPrint(pasynUser_, ASYN_TRACE_ERROR,
            "%s:setPosition: motor %d: the VXM cannot redefine its position (only null it); nothing sent\n",
            driverName, motor_);
  return asynError;
}

/** The readout's zero is set on the readout (its C button, or N/C/</> over
  * its line) and moves every client's reference: not from the record. */
asynStatus VelmexAxis::setEncoderPosition(double position)
{
  asynPrint(pasynUser_, ASYN_TRACE_ERROR,
            "%s:setEncoderPosition: motor %d: the readout's zero is set on the readout; nothing sent\n",
            driverName, motor_);
  return asynError;
}

asynStatus VelmexAxis::poll(bool *moving)
{
  asynStatus status = pC_->pollStatus_;
  long steps;
  double value;
  int lowLimit, highLimit;

  *moving = moving_ ? true : false;
  if (status) goto skip;

  status = pC_->readSteps(motor_, &steps);
  if (status) goto skip;
  lastSteps_ = steps;
  setDoubleParam(pC_->motorPosition_, (double)steps);

  {
    /* The manual's table: a limit input reads 0 when activated (N/C switches).
     * limitsActiveLow_ 0 inverts it for a stage wired the other way round
     * (U_Velmex's reads 0x0C at rest: motor 1's two bits low, off the switches). */
    int lowRaw = (pC_->limitByte_ & (1u << (2 * (motor_ - 1)))) != 0;
    int highRaw = (pC_->limitByte_ & (1u << (2 * (motor_ - 1) + 1))) != 0;
    lowLimit = limitsActiveLow_ ? !lowRaw : lowRaw;
    highLimit = limitsActiveLow_ ? !highRaw : highRaw;
  }
  setIntegerParam(pC_->motorStatusLowLimit_, lowLimit);
  setIntegerParam(pC_->motorStatusHighLimit_, highLimit);
  /* "At home" = on the limit this axis homes to. The record reads ATHM from the
   * encoder home bit (EA_HOME, motorStatusHome_) when UEIP is Yes and from
   * RA_HOME (motorStatusAtHome_) otherwise: set both. */
  setIntegerParam(pC_->motorStatusAtHome_, homeForwards_ ? highLimit : lowLimit);
  setIntegerParam(pC_->motorStatusHome_, homeForwards_ ? highLimit : lowLimit);

  if (!pC_->busy() && moving_) {
    moving_ = 0;
    multiIndex_ = 0;
    if (homing_) { homed_ = 1; homing_ = 0; }
  }
  *moving = moving_ ? true : false;
  setIntegerParam(pC_->motorStatusDone_, moving_ ? 0 : 1);
  setIntegerParam(pC_->motorStatusMoving_, moving_);
  setIntegerParam(pC_->motorStatusHomed_, homed_);

  /* The readout last: a readout silent or out of Display mode is a problem on
   * the record (its RBV keeps the last value) but does not hold the record in
   * a move the VXM has finished, and until it answers again no move is
   * accepted (no reference: a retry against a stale REP would move wrong). */
  if (encoder_ && pC_->pasynUserEncoder_) {
    if (pC_->vroPollStatus_ || pC_->vroStatus_ != 'D') { encoderOk_ = 0; status = asynError; goto skip; }
    status = pC_->readEncoder(encoder_, &value);
    if (status) { encoderOk_ = 0; goto skip; }
    encoderOk_ = 1;
    setDoubleParam(pC_->motorEncoderPosition_, value / eres_);
  }

skip:
  setIntegerParam(pC_->motorStatusProblem_, status ? 1 : 0);
  setIntegerParam(pC_->motorStatusCommsError_, status ? 1 : 0);
  callParamCallbacks();
  return status;
}

/* ------------------------------------------------------------- iocsh */

/** VelmexCreateController(portName, vxmPortName, vroPortName, numAxes, movingPollMs, idlePollMs) */
extern "C" int VelmexCreateController(const char *portName, const char *vxmPortName, const char *vroPortName,
                                      int numAxes, int movingPollMs, int idlePollMs)
{
  if (numAxes < 1 || numAxes > MAX_VELMEX_AXES) {
    printf("VelmexCreateController: numAxes %d is not 1..%d\n", numAxes, MAX_VELMEX_AXES);
    return asynError;
  }
  new VelmexController(portName, vxmPortName, vroPortName, numAxes, movingPollMs / 1000., idlePollMs / 1000.);
  return asynSuccess;
}

/** VelmexAxisConfig(portName, motor, encoder, encoderUnitsPerCount, accelSetting, homeBackoff, limitsActiveLow) */
extern "C" int VelmexAxisConfig(const char *portName, int motor, int encoder, const char *encoderUnitsPerCount,
                                int accelSetting, int homeBackoff, int limitsActiveLow)
{
  VelmexController *pC = static_cast<VelmexController *>(findAsynPortDriver(portName));
  double eres = encoderUnitsPerCount ? strtod(encoderUnitsPerCount, NULL) : 0.0;
  if (!pC) {
    printf("VelmexAxisConfig: no Velmex controller port '%s'\n", portName);
    return asynError;
  }
  VelmexAxis *pAxis = pC->getAxis(motor - 1);
  if (!pAxis || motor < 1 || motor > MAX_VELMEX_AXES) {
    printf("VelmexAxisConfig: %s has no motor %d\n", portName, motor);
    return asynError;
  }
  if (encoder < 0 || encoder > MAX_VELMEX_ENCODERS) {
    printf("VelmexAxisConfig: encoder %d is not 0..%d\n", encoder, MAX_VELMEX_ENCODERS);
    return asynError;
  }
  if (encoder && eres == 0.0) {
    printf("VelmexAxisConfig: encoderUnitsPerCount '%s' is not a non-zero number\n", encoderUnitsPerCount);
    return asynError;
  }
  if (accelSetting < 0 || accelSetting > VELMEX_MAX_ACCEL) {
    printf("VelmexAxisConfig: accelSetting %d is not 0..%d\n", accelSetting, VELMEX_MAX_ACCEL);
    return asynError;
  }
  if (homeBackoff < 0 || homeBackoff > VELMEX_MAX_INDEX) {
    printf("VelmexAxisConfig: homeBackoff %d is not 0..%ld\n", homeBackoff, VELMEX_MAX_INDEX);
    return asynError;
  }
  pC->lock();                            /* the poller is already running */
  pAxis->configure(encoder, eres, accelSetting, homeBackoff, limitsActiveLow);
  pC->unlock();
  return asynSuccess;
}

static const iocshArg createArg0 = {"Port name", iocshArgString};
static const iocshArg createArg1 = {"VXM serial port name", iocshArgString};
static const iocshArg createArg2 = {"VRO serial port name (empty: none)", iocshArgString};
static const iocshArg createArg3 = {"Number of motors (1-4)", iocshArgInt};
static const iocshArg createArg4 = {"Moving poll period (ms)", iocshArgInt};
static const iocshArg createArg5 = {"Idle poll period (ms)", iocshArgInt};
static const iocshArg * const createArgs[] = {&createArg0, &createArg1, &createArg2,
                                              &createArg3, &createArg4, &createArg5};
static const iocshFuncDef createDef = {"VelmexCreateController", 6, createArgs};
static void createCallFunc(const iocshArgBuf *args)
{
  VelmexCreateController(args[0].sval, args[1].sval, args[2].sval, args[3].ival, args[4].ival, args[5].ival);
}

static const iocshArg axisArg0 = {"Port name", iocshArgString};
static const iocshArg axisArg1 = {"Motor (1-4)", iocshArgInt};
static const iocshArg axisArg2 = {"Readout encoder (1, 2; 0 = none)", iocshArgInt};
static const iocshArg axisArg3 = {"Readout units per encoder count (ERES)", iocshArgString};
static const iocshArg axisArg4 = {"Acceleration setting A<m>M<x> (1-127; 0 = leave)", iocshArgInt};
static const iocshArg axisArg5 = {"Home backoff (steps; 0 = none)", iocshArgInt};
static const iocshArg axisArg6 = {"Limits active low (1 = the manual's 0 = activated; 0 = inverted)", iocshArgInt};
static const iocshArg * const axisArgs[] = {&axisArg0, &axisArg1, &axisArg2, &axisArg3, &axisArg4, &axisArg5, &axisArg6};
static const iocshFuncDef axisDef = {"VelmexAxisConfig", 7, axisArgs};
static void axisCallFunc(const iocshArgBuf *args)
{
  VelmexAxisConfig(args[0].sval, args[1].ival, args[2].ival, args[3].sval, args[4].ival, args[5].ival, args[6].ival);
}

static void VelmexRegister(void)
{
  iocshRegister(&createDef, createCallFunc);
  iocshRegister(&axisDef, axisCallFunc);
}

extern "C" {
epicsExportRegistrar(VelmexRegister);
}

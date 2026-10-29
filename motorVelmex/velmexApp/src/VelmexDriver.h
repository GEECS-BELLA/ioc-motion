/* VelmexDriver.h
 *
 * Velmex VXM stepping-motor controller (one to four motors on one serial line)
 * with a Velmex VRO encoder readout on a second serial line, as an
 * asynMotorController ("model 3" asynMotor driver). GEECS-BELLA/ioc-motion,
 * written from the VXM user's manual and command summary (VXM-CS 12-29-04,
 * firmware 1.30 era) and the VRO reference manual (version 1.30).
 *
 * The VXM counts steps; the readout is the position reference. The driver
 * reports both (motor position = the VXM's absolute position register in
 * steps, encoder position = the readout's displayed value over ERES), so the
 * motor record, with UEIP and RTRY, closes the loop the way the LabVIEW
 * driver did: index, compare with the readout, index the difference.
 */
#ifndef VelmexDriver_H
#define VelmexDriver_H

#include "asynMotorController.h"
#include "asynMotorAxis.h"

#define MAX_VELMEX_AXES 4
#define MAX_VELMEX_ENCODERS 2        /* a VRO-2 reads two encoders (X, Y) */
#define VELMEX_MAX_SPEED 6000        /* S<m>M<x>: 1 to 6000 steps/s */
#define VELMEX_MAX_ACCEL 127         /* A<m>M<x>: 1 to 127 */
#define VELMEX_MAX_INDEX 16777215L   /* I<m>M<x>: 1 to 16,777,215 steps; IA<m>M<x>: +/- the same */
#define VELMEX_REPLY_LEN 64
#define VELMEX_VXM_TIMEOUT 1.0       /* s, per reply; 9600 baud and a terminal server in between */
#define VELMEX_VRO_TIMEOUT 1.0

/* Controller-wide readings beyond the motor record's (VelmexController.db) */
#define VelmexVxmStatusString   "VELMEX_VXM_STATUS"
#define VelmexVroStatusString   "VELMEX_VRO_STATUS"
#define VelmexLimitByteString   "VELMEX_LIMIT_BYTE"
#define VelmexVxmVersionString  "VELMEX_VXM_VERSION"
#define NUM_VELMEX_PARAMS 4

class epicsShareClass VelmexAxis : public asynMotorAxis
{
public:
  VelmexAxis(class VelmexController *pC, int axisNo);
  void report(FILE *fp, int level);
  asynStatus move(double position, int relative, double minVelocity, double maxVelocity, double acceleration);
  asynStatus moveVelocity(double minVelocity, double maxVelocity, double acceleration);
  asynStatus home(double minVelocity, double maxVelocity, double acceleration, int forwards);
  asynStatus stop(double acceleration);
  asynStatus poll(bool *moving);
  asynStatus setPosition(double position);
  asynStatus setEncoderPosition(double position);
  /** Per-axis configuration from iocsh (VelmexAxisConfig) */
  void configure(int encoder, double encoderUnitsPerCount, int accelSetting, int homeBackoff);

private:
  VelmexController *pC_;   /**< The controller this axis belongs to */
  int motor_;              /**< The VXM motor number, 1..4 (axisNo_ + 1) */
  int encoder_;            /**< The readout's encoder for this motor, 1 (X) or 2 (Y); 0 = none */
  double eres_;            /**< Readout units per encoder count as the record sees it (its ERES) */
  int accelSetting_;       /**< A<m>M<x> sent before every index, 1..127; 0 = leave the controller's */
  int homeBackoff_;        /**< Steps moved away from the limit before a home seeks it; 0 = none */
  int moving_;             /**< This axis's program is running */
  int encoderOk_;          /**< The readout answered on the last poll: a reference exists */
  int homing_;             /**< The running program is a home */
  int homed_;              /**< A home finished since the IOC started */
  int homeForwards_;       /**< The last home sought the positive limit */
  int multiIndex_;         /**< The running program has more than one index (stop = K, not D) */
  long lastSteps_;         /**< The last X reading */

  asynStatus runProgram(const char *body, int multiIndex);

friend class VelmexController;
};

class epicsShareClass VelmexController : public asynMotorController {
public:
  VelmexController(const char *portName, const char *vxmPortName, const char *vroPortName,
                   int numAxes, double movingPollPeriod, double idlePollPeriod);

  void report(FILE *fp, int level);
  VelmexAxis* getAxis(asynUser *pasynUser);
  VelmexAxis* getAxis(int axisNo);
  asynStatus poll();
  void goOffline();

protected:
  int VelmexVxmStatus_;
  int VelmexVroStatus_;
  int VelmexLimitByte_;
  int VelmexVxmVersion_;

private:
  asynUser *pasynUserEncoder_;   /**< The readout's line; NULL when none was given */
  int online_;                   /**< F sent: the VXM is On-Line (its jog inputs disabled) */
  int identified_;               /**< getD0 asked once */
  char vxmStatus_;               /**< The last V reply: R, B, J, b; '?' none yet */
  char vroStatus_;               /**< The readout's last V reply: D, P, S, F; '?' none yet */
  unsigned char limitByte_;      /**< The last ? reply */
  asynStatus pollStatus_;        /**< The VXM part of the current poll (V, ?) */
  asynStatus vroPollStatus_;     /**< The readout's V this poll; the axis poll reads it last */

  asynStatus exchange(asynUser *pasynUser, const char *cmd, char *reply, size_t maxLen,
                      int terminated, int skipCaret, double timeout);
  asynStatus writeVxm(const char *cmd);
  asynStatus readStatus();
  asynStatus readLimits();
  asynStatus readSteps(int motor, long *steps);
  asynStatus readVroStatus();
  asynStatus readEncoder(int encoder, double *value);
  void identify();
  int busy() const { return vxmStatus_ == 'B' || vxmStatus_ == 'b'; }
  int local() const { return vxmStatus_ == 'J' || vxmStatus_ == 'b'; }

friend class VelmexAxis;
};

#endif

/* ZaberDriver.h
 *
 * Zaber devices speaking the Zaber ASCII protocol (A-series and X-series
 * integrated stages, X-MCC controllers; a daisy chain of them on one serial
 * line) as an asynMotorController ("model 3" asynMotor driver).
 * GEECS-BELLA/ioc-motion, written from Zaber's ASCII protocol manual
 * (zaber.com/protocol-manual, read 2026-10-29); no code copied from anywhere.
 *
 * One controller per serial line. Each motor-record axis is one (device
 * address, axis number) pair on that line: a chain of integrated stages is
 * several addresses with axis 1 each, an X-MCC is one address with axes 1..4.
 * Positions are the device's native units (microsteps for a stepper, encoder
 * counts for a direct drive); the record's MRES converts.
 */
#ifndef ZaberDriver_H
#define ZaberDriver_H

#include <epicsTime.h>
#include "asynMotorController.h"
#include "asynMotorAxis.h"

#define MAX_ZABER_AXES 16
#define ZABER_REPLY_LEN 256
#define ZABER_TIMEOUT 0.5            /* s, per reply line; 115200 baud and a terminal server in between */
#define ZABER_MAX_LINES 8            /* lines read for one reply: alerts (!) and info (#) are skipped */
#define ZABER_POLL_FAILURES 3        /* consecutive failed polls before the record sees a problem */
#define ZABER_VELOCITY_DATA 1.6384   /* velocity data = microsteps/s x 1.6384 (protocol manual) */
#define ZABER_ACCEL_DATA (1.6384 / 10000.0)   /* accel data = microsteps/s^2 x 1.6384 / 10000 */
#define ZABER_MAX_FLAGS 16

/* Per-axis readings beyond the motor record's (ZaberAxis.db) */
#define ZaberFlagString       "ZABER_FLAG"
#define ZaberWarningsString   "ZABER_WARNINGS"
#define ZaberVersionString    "ZABER_VERSION"
#define ZaberDeviceIdString   "ZABER_DEVICE_ID"
#define ZaberPeripheralIdString "ZABER_PERIPHERAL_ID"
#define ZaberSerialString     "ZABER_SERIAL"
#define ZaberMaxspeedString   "ZABER_MAXSPEED"
#define ZaberLimitMinString   "ZABER_LIMIT_MIN"
#define ZaberLimitMaxString   "ZABER_LIMIT_MAX"
#define NUM_ZABER_PARAMS 9

/** One reply line, parsed: "@01 1 OK IDLE -- 12345" */
struct ZaberReply {
  int device;
  int axis;
  int ok;              /* OK = 1, RJ = 0 */
  int busy;            /* BUSY = 1, IDLE = 0 */
  char flag[3];        /* the highest-priority warning flag, "--" for none */
  char data[ZABER_REPLY_LEN];
};

enum ZaberPowerPolicy { ZABER_POWER_LEAVE = 0, ZABER_POWER_ON_FOR_MOVE = 1 };

class epicsShareClass ZaberAxis : public asynMotorAxis
{
public:
  ZaberAxis(class ZaberController *pC, int axisNo);
  void report(FILE *fp, int level);
  asynStatus move(double position, int relative, double minVelocity, double maxVelocity, double acceleration);
  asynStatus moveVelocity(double minVelocity, double maxVelocity, double acceleration);
  asynStatus home(double minVelocity, double maxVelocity, double acceleration, int forwards);
  asynStatus stop(double acceleration);
  asynStatus poll(bool *moving);
  asynStatus setPosition(double position);
  /** Per-axis configuration from iocsh (ZaberAxisConfig) */
  void configure(int device, int axis, int speedArgs, int allowHome, int powerPolicy, double powerOffDelay);

private:
  ZaberController *pC_;
  int device_;            /**< Device address on the line, 1..99; 0 = not configured (nothing sent) */
  int axis_;              /**< Axis number on that device, 1..9 */
  int speedArgs_;         /**< 1: move abs <pos> <speed> <accel> (firmware 7.25 and up); 0: the device's maxspeed and accel */
  int allowHome_;         /**< 1: the record's home sends "home"; 0: refused */
  int powerPolicy_;       /**< ZaberPowerPolicy */
  double powerOffDelay_;  /**< s after a move ends before "driver disable" (on_for_move) */
  int identified_;        /**< The identity and settings were read */
  int firmware_;          /**< major * 100 + minor, 0 = unknown */
  int moving_;            /**< A move or home of ours is running */
  int busy_;              /**< The last reply said BUSY */
  int homed_;             /**< No WR (no reference position) among the warnings */
  int failures_;          /**< Consecutive failed polls */
  int driverOn_;          /**< No FO / disable-FD among the warnings */
  int offPending_;        /**< on_for_move: a driver disable is due at offAt_ */
  epicsTimeStamp offAt_;
  long lastPos_;
  char flags_[ZABER_MAX_FLAGS][3];
  int nflags_;

  asynStatus cmd(const char *command, ZaberReply *r);
  asynStatus deviceCmd(const char *command, ZaberReply *r);
  void identify();
  asynStatus readWarnings(const char *topFlag);
  int hasFlag(const char *flag) const;
  int isFault(const char *flag) const;
  asynStatus driverEnable();

friend class ZaberController;
};

class epicsShareClass ZaberController : public asynMotorController {
public:
  ZaberController(const char *portName, const char *serialPortName, int numAxes,
                  double movingPollPeriod, double idlePollPeriod);

  void report(FILE *fp, int level);
  ZaberAxis* getAxis(asynUser *pasynUser);
  ZaberAxis* getAxis(int axisNo);

protected:
  int ZaberFlag_;
  int ZaberWarnings_;
  int ZaberVersion_;
  int ZaberDeviceId_;
  int ZaberPeripheralId_;
  int ZaberSerial_;
  int ZaberMaxspeed_;
  int ZaberLimitMin_;
  int ZaberLimitMax_;

private:
  asynStatus exchange(int device, int axis, const char *command, ZaberReply *r);

friend class ZaberAxis;
};

#endif

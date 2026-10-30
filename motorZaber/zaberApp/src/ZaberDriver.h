/* ZaberDriver.h
 *
 * Zaber devices speaking the Zaber ASCII protocol (A-series and X-series
 * integrated stages, X-MCC controllers; a daisy chain of them on one serial
 * line) as an asynMotorController ("model 3" asynMotor driver).
 * GEECS-BELLA/ioc-motion, written from Zaber's ASCII protocol manuals
 * (firmware 7: zaber.com/protocol-manual; firmware 6: zaber.com/wiki/
 * Manuals/ASCII_Protocol_Manual; read 2026-10-29); no code copied.
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
#define ZABER_MAX_LINES 8            /* lines read for one reply: alerts (!), info (#) and stale replies are skipped */
#define ZABER_POLL_FAILURES 3        /* consecutive failed polls before the record sees a problem */
#define ZABER_BACKOFF_POLLS 10       /* after ZABER_POLL_FAILURES, a silent axis is asked once per this many polls */
#define ZABER_ID_PASSES 3            /* identity passes before an unanswered read is given up (its record stays 0) */
#define ZABER_SETTLE_S 0.5           /* on_for_move: s after the driver disable is acknowledged before the position is read as final */
#define ZABER_OFF_TRIES 3            /* driver disables without an OK before the move is reported done anyway (driver left on) */
#define ZABER_SPEED_ARGS_FW 725      /* move abs <pos> <speed> <accel>: firmware 7.25 and up */
#define ZABER_VELOCITY_DATA 1.6384   /* velocity data = microsteps/s x 1.6384 (protocol manual) */
#define ZABER_ACCEL_DATA (1.6384 / 10000.0)   /* accel data = microsteps/s^2 x 1.6384 / 10000 */
#define ZABER_MAX_FLAGS 16

/* Per-axis readings and the one command beyond the motor record's (ZaberAxis.db) */
#define ZaberFlagString       "ZABER_FLAG"
#define ZaberWarningsString   "ZABER_WARNINGS"
#define ZaberVersionString    "ZABER_VERSION"
#define ZaberDeviceIdString   "ZABER_DEVICE_ID"
#define ZaberPeripheralIdString "ZABER_PERIPHERAL_ID"
#define ZaberSerialString     "ZABER_SERIAL"
#define ZaberMaxspeedString   "ZABER_MAXSPEED"
#define ZaberLimitMinString   "ZABER_LIMIT_MIN"
#define ZaberLimitMaxString   "ZABER_LIMIT_MAX"
#define ZaberResolutionString "ZABER_RESOLUTION"
#define ZaberClearWarningsString "ZABER_CLEAR_WARNINGS"
#define ZaberStopFailedString "ZABER_STOP_FAILED"
#define NUM_ZABER_PARAMS 12

/* The identity reads after get version, each retried until the device answers it (OK or RJ) */
enum { ZABER_ID_DEVICEID = 1, ZABER_ID_SERIAL = 2, ZABER_ID_PERIPHERAL = 4, ZABER_ID_RESOLUTION = 8,
       ZABER_ID_MAXSPEED = 16, ZABER_ID_LIMIT_MIN = 32, ZABER_ID_LIMIT_MAX = 64, ZABER_ID_ALL = 127 };

/* Which limit a latched WL stands for, taken on the poll it appeared */
enum { ZABER_WL_NONE = 0, ZABER_WL_HIGH = 1, ZABER_WL_LOW = 2, ZABER_WL_UNKNOWN = 3 };

/** One reply line, parsed: "@01 1 07 OK IDLE -- 12345" (07: the message id) */
struct ZaberReply {
  int device;
  int axis;
  int id;              /* the message id, -1 when the reply carries none */
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
  void configure(int device, int axis, int speedArgs, int allowHome, int powerPolicy, double powerOffDelay,
                 int allowUnreferenced, int allowClearWarnings);
  asynStatus clearWarnings();
  /** This axis is configured as (device, axis) */
  bool isAt(int device, int axis) const { return device_ == device && axis_ == axis; }

private:
  ZaberController *pC_;
  int device_;            /**< Device address on the line, 1..99; 0 = not configured (nothing sent) */
  int axis_;              /**< Axis number on that device, 1..9 */
  int speedArgs_;         /**< 1: move abs <pos> <speed> <accel> (firmware 7.25 and up); 0: the device's maxspeed and accel */
  int allowHome_;         /**< 1: the record's home sends "home"; 0: refused */
  int allowUnreferenced_; /**< 1: moves accepted while WR (no reference); 0: refused */
  int allowClear_;        /**< 1: the clear_warnings record sends "warnings clear"; 0: refused */
  int powerPolicy_;       /**< ZaberPowerPolicy */
  double powerOffDelay_;  /**< s after a move ends before "driver disable" (on_for_move) */
  int identified_;        /**< get version answered and every identity read in idDone_ */
  int idDone_;            /**< ZABER_ID_* bits: the identity reads the device has answered */
  int idPasses_;          /**< identify() passes since get version answered; reads still unanswered after ZABER_ID_PASSES are given up */
  int wlSide_;           /**< ZABER_WL_*: the side a latched WL was taken for, on the poll it appeared */
  int flagsComplete_;     /**< The flag list is the device's full one (warnings answered, or the top flag was --) */
  int stopFailed_;        /**< The last stop got no reply or a rejection: an alarm until a stop or move is acknowledged */
  int statusSentRunning_; /**< The full status was sent once after iocInit (a fault present at boot reaches the record) */
  int firmware_;          /**< major * 100 + minor, 0 = unknown */
  int moving_;            /**< A move or home of ours is running */
  int busy_;              /**< The last reply said BUSY */
  int homed_;             /**< Neither WR (no reference) nor WH (not homed to the calibration's reference) */
  int failures_;          /**< Consecutive failed polls */
  int backoff_;           /**< Polls left to skip on a silent axis */
  int driverOn_;          /**< No FO or FD among the warnings */
  int weDisabled_;        /**< This IOC sent "driver disable" and no "driver enable" since */
  int lastDirPositive_;   /**< The direction of the last move or home of ours (a WL is on that side) */
  int offPending_;        /**< on_for_move: a driver disable is due at offAt_ */
  epicsTimeStamp offAt_;
  int offFailures_;       /**< Consecutive driver disables that got no OK */
  int settling_;          /**< on_for_move: the driver disable was acknowledged; done once a position read at or after settleAt_ */
  epicsTimeStamp settleAt_;
  long lastPos_;
  char flags_[ZABER_MAX_FLAGS][3];
  int nflags_;

  asynStatus cmd(const char *command, ZaberReply *r);
  asynStatus deviceCmd(const char *command, ZaberReply *r);
  void identify();
  int idRead(int bit, ZaberReply *r, const char *command, const char *alternative, int deviceScope);
  void takeFlags(const ZaberReply &r);
  void setStopFailed(int failed);
  int hasFlag(const char *flag) const;
  int isFault(const char *flag) const;
  asynStatus driverEnable();
  void scheduleOff(double delay);

friend class ZaberController;
};

class epicsShareClass ZaberController : public asynMotorController {
public:
  ZaberController(const char *portName, const char *serialPortName, int numAxes,
                  double movingPollPeriod, double idlePollPeriod);

  void report(FILE *fp, int level);
  ZaberAxis* getAxis(asynUser *pasynUser);
  ZaberAxis* getAxis(int axisNo);
  asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);

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
  int ZaberResolution_;
  int ZaberClearWarnings_;
  int ZaberStopFailed_;

private:
  int nextId_;            /**< The next message id, 0..99 */
  asynStatus exchange(int device, int axis, const char *command, ZaberReply *r);

friend class ZaberAxis;
};

#endif

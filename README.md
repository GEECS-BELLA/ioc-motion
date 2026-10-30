# Generic IOC ioc-motion

Motion controllers on the EPICS motor record, for GEECS (geecs-epics decision
14: stages are motor records; decision 11: one image per family). Built on
epics-containers' `ioc-motorsim` developer image, so it carries everything that
image does (motor R7-4, asyn, autosave, busy, calc, sscan, sequencer, iocStats,
pvlogging, **motorSim**) plus:

| Module | Version | Source | Reuse (decision 10) |
|---|---|---|---|
| motorNewport | R1-3 | [epics-motor/motorNewport](https://github.com/epics-motor/motorNewport) (ships no LICENSE file; tier 1 rests on the epics-motor org, geecs-epics device catalogue) | tier 1, dependency, with one build-time patch (below: ESP300, SMC100, Agilis AG-UC8) |
| motorAerotech | R1-1-1 | [epics-motor/motorAerotech](https://github.com/epics-motor/motorAerotech) (ships no LICENSE file either; `drvEnsembleAsyn.cc` cites a synApps licence it does not ship); no upstream ibek-support recipe, so ours | tier 1, dependency, with one build-time patch (below) |
| motorPIGCS2 | R1-3-1 | [epics-motor/motorPIGCS2](https://github.com/epics-motor/motorPIGCS2) (EPICS Open License); no upstream ibek-support recipe (INFN's targets their fork), so ours | tier 1, dependency, with one build-time patch of switches (below: PI hexapod) |

Drivers for the other families geecs-epics' device catalogue assigns to
this image (NewFocus Picomotor, Zaber) join it when their first device is
ported.

## Build

No vendor code is inside, so the image is published: CI builds on every push
and, on a git tag (CalVer `YYYY.M.N`, never re-pushed), pushes
`ghcr.io/geecs-bella/ioc-motion-runtime:<tag>` and
`ghcr.io/geecs-bella/ioc-motion-developer:<tag>`, and the release job attaches the
ioc schema and `versions.json` to the GitHub release. The organisation's
packages inherit the repository's visibility, so hosts pull with no
credentials. Hosts never build. A pre-release tag (`2026.10.2b1`) publishes a
branch for bench work.

A local build, for development only:

```
DOCKER_DEFAULT_PLATFORM=linux/amd64 TARGET=runtime TAG=ioc-motion:local ./build  # amd64, emulated on Apple silicon
```

The compose files in geecs-epics pin `ghcr.io/geecs-bella/ioc-motion-runtime:<tag>`;
a bump is a new tag here and one line there.

## Serial lines (`geecsSerial.Port`)

Every serial device takes its line as an object, a `geecsSerial.Port`, and
never creates a port itself (geecs-epics decision 15), so the same device
config runs on a host's tty, a USB-serial adapter or a terminal server:

The carrier is read from `address`, in asyn's own syntax, so a sim twin
swaps one string:

| `address` | Carrier | Line settings | Rendered |
|---|---|---|---|
| `/dev/serial/by-id/...` | a local tty | this IOC | `drvAsynSerialPortConfigure` + `asynSetOption` |
| `host:port COM` | a terminal server's telnet socket with RFC 2217 (a Digi's 2001 + n−1 for port n) | this IOC, at boot; after a reconnect asyn re-sends what it holds, with two gaps (below) | `drvAsynIPPortConfigure` + `asynSetOption` |
| `host:port` | a raw socket (a Digi's 2101 + n−1) | the terminal server's own port configuration | `drvAsynIPPortConfigure` only |

RFC 2217 (`COM`) is the default for a device on a Digi: the IOC carries the line
settings, so a reset or a re-profiled Digi port cannot leave the device at the
wrong baud (the first SMC100, 2026-10-28, was silent on the raw socket and
answered on RFC 2217 at its manual's 57600). Upstream's `asyn.AsynIP` is not
used for serial lines: it takes baud, parity, stop and crtscts and renders
none of them, and neither it nor `asyn.AsynSerial` has `ixon`.

Two gaps in asyn R4-46's `asynInterposeCom`, upstream and not patched here:
on a reconnect `restoreSettings` reads each option back and sets it, and
`crtscts` reads back `N` under XON/XOFF, so setting it turns flow control off
before `ixon` is read (also `N` by then): **after any reconnect the line runs
without XON/XOFF**. And `setOption` records a value only when the server
answers, so **if the terminal server is unreachable at boot**, every
`asynSetOption` fails and the first connect restores asyn's defaults (9600
8N1, no flow control): the device stays silent. Restart the IOC after a
terminal-server outage. Worth an upstream fix (restore flow control as one
setting; keep requested values when the server is down).

The file has no source and no install recipe; the Dockerfile links it into
`/epics/ibek-defs` as ansible does a module's. It moves to a shared repo when a
second image needs it (decision 11).

## ESP300 family (ESP300 / ESP301 / ESP302)

`drvESP300` + `devESP300` is a model 1 motor driver (device support over
`motordevCom`/`motordrvCom`: `struct motor_dset devESP300`, `struct
driver_table ESP300_access`; motor's `docs/motorDeviceDriver.html` for the
models): `ESP300Setup` once,
`ESP300Config` per controller, motor records on `basic_motor.db` with DTYP
`ESP300` and `OUT "#C<card> S<axis-1> @"`. The ESP302 is driven over its
Ethernet command port (TCP 5001, one client at a time; an ESP301's RS-232 port
needs a terminal server). The ibek entities, in
`ibek-support-geecs/motorNewport/motorNewport.ibek.support.yaml`:

- `motorNewport.ESP300Setup` — once per IOC, before any controller (stated,
  not enforced: a second one re-allocates the card table): `max_controllers`,
  `scan_rate`, and the two switches below.
- `motorNewport.ESP300Controller` — the asyn IP port and the card.
- `motorNewport.ESP300Axis` — one motor record per axis.

**Resolution.** The driver reports position as `TP? / resolution`, where
resolution is the axis's `FR?` (no encoder, `ZB?` bits 8-9 clear) or `SU?`
(encoder). `MRES` must equal that number or RBV is off by its ratio. Read it
from the controller before writing the config (the first ESP302's snapshot in
geecs-epics is the example: `FR? 1e-06`, so `MRES: 0.000001`); the entity has
no default for it on purpose.

### What the stock driver writes at start, and the patch

Upstream initialisation is not read-only:

1. `drvESP300.motor_init` counts the controller's axes by sending each a
   **Stop** (`01ST`, `02ST`, …) until `TB?` answers error 9 (axis number out
   of range).
2. `motorRecord.init_record` calls `set_dial_highlimit/lowlimit` for every
   record, which `devESP300` turns into **`nnSR<DHLM>` / `nnSL<DLLM>`** — the
   controller's software travel limits overwritten with the record's, at every
   boot and on every limit change (and with `0`/`0` when the record has none).

Everything else at init and in the in-motion poll is a query: `VE?`, `nnSN?`, `nnZB?`,
`nnFR?`/`nnQS?`/`nnSU?`, `TB?`, `nnMD`, `nnTP?`, `nnZH?`, `nnMO?`, `TE?`
(and `nnPH`, which the gate below never reaches). `LOAD_POS` (→ `nnDH`, redefine position) is sent only when autosave
restores a non-zero `DVAL` onto a controller reading near zero (`RSTM`
NearZero, the record's default) — so an IOC that must preserve positions runs
without autosave of the record's position fields (`motor_positions.req`:
DVAL and friends; settings autosave is fine). PID writes happen only for axes with an
encoder (`GAIN_SUPPORT`) and non-zero `PCOF/ICOF/DCOF`.

`motorNewport-geecs.patch` (applied by `motorNewport.install.yml` on
upstream R1-3; its ESP300 part: two source files and `devNewport.dbd`, whose `variable()`
lines are what make the switches visible to iocsh's `var` — without them
`var` prints "No known var" and the stock behaviour runs, as the fake showed)
adds two iocsh variables, both defaulting to upstream behaviour:

| Variable | 1 (upstream) | 0 |
|---|---|---|
| `ESP300InitStop` | count axes with `ST` | count axes with `nnMO?` (an axis out of range answers nothing and sets error 9, as `ST` does; the read times out once, 5 s) |
| `ESP300SetLimits` | push DHLM/DLLM into `SR`/`SL` | the record enforces its own soft limits; the controller's travel limits stay what the stage was set up with |

Two more lines in the same patch: with `ESP300InitStop` 0 the error FIFO is
drained (bounded `TB?` reads) before the probe, so an error left by an earlier
client cannot end the axis count at axis 1 (each drained error is printed on
the console); and `ESP300_end_trans` no longer
reads the byte before an empty transaction (upstream did, for `GET_INFO` and
`GO`; the disabled limit push adds two per record at boot).

The `ESP300Setup` entity renders them from `init_stop` / `set_limits`, which
**default to false**: in GEECS the controller's limits are the stage's
physical travel and the record's are the operating range, two different
things, and an IOC restart must not move or reconfigure a mirror mount. With
both false, a boot sends the controller nothing but queries — checked against
the protocol fake in geecs-epics (`sim/esp300_sim.py`, which logs any
non-query as `WRITE`) and against a real ESP302 with the asyn trace on
(geecs-epics#21). Expected boot noise: the probe of the first axis past the
last one waits 5 s and, with `ASYN_TRACE_ERROR` on, prints one timeout line.
The change is worth proposing upstream (a query-based axis count; limits
optional); until then it lives here.

### Known gaps in the upstream driver (R1-3), seen on the bench

- **Hardware limit and home bits are never read.** `set_status` decides
  whether to send `PH` with `use_limits = (limits&0x1 == 0) || (limits&0x5 == 0)`;
  `==` binds tighter than `&`, so both terms are `limits & 0` and `PH` is never
  sent (the bench transcripts have no `PH`). The record's `HLS`/`LLS`/`ATHM`
  are therefore 0 by construction. The controller's own hardware limits still
  stop the stage; the record just does not know. Worth an upstream fix.
- **Idle axes are not polled.** A model-2 driver polls only while a motor
  it started is moving; after init nothing is sent until a record processes
  (`STUP`, a move). A position changed from the controller's front panel is
  not seen until then.

### Testing without hardware

geecs-epics `hosts/sim/compose.yaml` (profile `motion`) runs this image
against the ESP300 fake on the real `esptest1` config, with the fake's log as
the transcript of what the IOC sent.

## SMC100 (SMC100CC / SMC100PP)

`SMC100Driver` is a "model 3" asynMotor driver: `SMC100CreateController` per
serial line, one driver axis per controller on the line. Axis n is chain
address n+1 (every command is `<n+1><cmd>`), so a single SMC100 is
`num_axes: 1` and an RS-485 chain of six with addresses 1-6 is one
controller with `num_axes: 6` (the chain's addresses must run 1..N). The
ibek entities (`ibek-support-geecs/motorNewport/motorNewport.ibek.support.yaml`):

- `geecsSerial.Port` — the line: 57600 8N1, `ixon: true`, CR LF both ways
  (manual EDH0206En2041 08/15, 2.8 and 3.1: preset in the controller, not
  configurable).
- `motorNewport.SMC100Controller` — the driver on that port: chain length,
  poll periods, `mres` (the driver's step size and every axis's `MRES`, one
  value so they cannot disagree) and the two switches below.
- `motorNewport.SMC100Axis` — one chain address as a motor record on
  `SMC100Motor.db` (added by the patch: RSTM Never, RTRY 0, DISP on by default).

**Units and `mres`.** The controller takes and reports float positions in the
stage's preset units (mm for a linear stage); the driver divides `TP` by its
step size and multiplies moves, `VA` and `AC` by it. Any `mres` works as long
as the driver and the record share it, which the entities guarantee; it only
has to show the stage's resolution (an SMC100PP's `FRS`/`FRM`: the first
unit's 0.01/100 = 1e-4 mm) and keep travel/`mres` inside an int32. The default
1e-6 does both for any stage under ±2 km.

**Polling.** The asynMotor poller polls every axis on the line each cycle: at
`moving_poll_ms` while any axis moves, `idle_poll_ms` otherwise (0 = never
when idle). Each axis costs a `TP` and a `TS`, about 10 ms each on address 1
and 16 ms on the others (manual 6.4), so a chain of six takes about 0.2 s a
cycle and the 500 ms default leaves room.

**Stage configuration lives in the controller.** Velocity and acceleration
limits, backlash (`BA`) and hysteresis (`BH`) compensation, software travel
limits (`SL`/`SR`), home type, micro-stepping (`FRM`/`FRS`), and the
SmartStage setting (`ZX`) are configuration parameters: settable only in
CONFIGURATION state, saved by `PW0`, which leaves the controller NOT
REFERENCED (a home is needed after). An ESP-compatible Newport stage loads them
from the stage itself (`ZX3`: read at power-up). The driver neither reads nor
writes them, and nothing in an `ioc.yaml` describes the actuator: the IOC
needs only the line and `mres`. Read them with `<n>ZT` (all parameters, every
state, a query) for the instance's snapshot; `VELO` and `ACCL` must stay
within the configured `VA` and `AC`, which the controller enforces anyway.

### What the stock driver sends, and the patch

Creation sends nothing; the poller sends `TP` and `TS` per axis. A move sends
`VA`, `AC` and `PA` (or `PR`); a home `OR`; a stop `ST`; a jog queries `SR?`
or `SL?` and moves there. Two calls send something they did not write:

- **`setPosition` and `setClosedLoop` resend the last command.** Both call
  `writeReadController()` with the lines that format their command commented
  out, so the buffer still holds the previous command. That is nearly always
  the poller's last `TS`, harmless; but a call that lands between a move's
  `PA`/`PR` and the next poll resends the move (a relative one moves again).
  The motor record reaches `setPosition` when the dial position is redefined:
  SET mode with the offset frozen (`FOFF` Frozen; with `FOFF` Variable SET
  only moves `OFF` and sends nothing) and an autosave restore at init (RSTM).
  `setClosedLoop` is not reachable from `SMC100Motor.db`: the record forwards
  `CNEN` only to a driver that reports gain support (this one does not), and
  the asynMotor auto-power option has no record here. Either way the call
  reports success and does nothing it claims. On the fake (2026-10-29) a
  frozen-offset SET reached `setPosition`, which with the patch sent nothing
  and printed its error; a `CNEN` put sent nothing at all.
- **`TS` is read by fixed character positions.** `inString_[6..8]` assume a
  one-digit address (addresses 10-31 misparse), only state 28 (MOVING) is
  motion (HOMING reads as done), end of run is matched by digit equality, and
  nothing sets the problem bit: a stage NOT REFERENCED, in DISABLE or with a
  following error refuses a move without a sign, and the record finishes the
  move at once at the old position.

`motorNewport-geecs.patch`'s SMC100 part adds two iocsh variables, both
defaulting to upstream behaviour (`variable()` lines in `devNewport.dbd`):

| Variable | 1 (upstream) | 0 |
|---|---|---|
| `SMC100SetterResend` | `setPosition` / `setClosedLoop` resend the buffer | send nothing, print an error, return asynError (the SMC100 has no position redefine) |
| `SMC100StatusUpstream` | fixed-position `TP`/`TS` parsing as above | parse after `TP`/`TS` (addresses 1-31); moving = HOMING (1E, 1F), MOVING (28), JOGGING (46, 47); limits from error bits 0 and 1; Homed in every state but NOT REFERENCED (0A-11), CONFIGURATION (14) and HOMING; problem in NOT REFERENCED, CONFIGURATION, DISABLE (3C-3E) or on any other error bit (manual, `TS`) |

`SMC100Controller`'s `setter_resend` and `status_upstream` render them and
default to false. They are global to the IOC (every SMC100 line in it). Both
are worth proposing upstream: the setters as no-ops (or `setPosition` as an
error, since the SMC100 has no position redefine), the status as a full `TS`
decode; until then they live here.

### Known gaps (R1-3, with the patch), from reading it

- **Jog** (`moveVelocity`, unpatched): it moves to the controller's `SR`/`SL`
  (the stage's travel, outside the record's `DHLM`/`DLLM`; the record's soft
  limit stops it, overshooting by the deceleration distance), parses the
  `SR?`/`SL?` reply at a fixed offset (addresses 10-31 read 0: a jog becomes a
  move to 0), and sends a negative `VA` for a reverse jog, which the
  controller refuses (error C), so the jog runs at the last `VA`. Do not jog
  a chain with addresses above 9.
- **CONFIGURATION state** stalls the poll: `TP` is refused there (error I,
  no reply), so each poll waits out the asyn timeout before `TS`, and the
  record shows a communication problem, not the state. Nothing in this IOC
  enters CONFIGURATION; commissioning does, with the IOC stopped.
- **Homing**: `OR` is accepted only NOT REFERENCED (error K in READY), so the
  record's HOMF/HOMR do nothing on a referenced stage; a re-home needs `RS`
  first, which the driver never sends.

## Agilis AG-UC8 (AG-UC2, AG-UC8PC)

`AG_UC` (motorNewport R1-3, Mark Rivers, 2013; the AG-UC8 channel support
de7fd87) is a "model 3" asynMotor driver for Newport's Agilis piezo stepper
controllers. Open loop: a position is the controller's step counter (`TP`:
forward steps minus backward steps since power-up or `ZP`), and a step's size
depends on its direction, the step amplitude (`SU`, 1-50) and the load (user
manual EDH0224En5010 12/09, 4.7.4). The AG-UC8 drives four channel pairs of
two axes; one pair is selected at a time (`CC1`..`CC4`) and the axis commands
(`1PR`, `2TP`, ...) address the selected pair. The ibek entities:

- `geecsSerial.Port` — the line: the AG-UC8's FTDI USB serial port,
  `/dev/serial/by-id/usb-Newport_Agilis_<serial>-if00-port0`, 921600 8N1, no
  flow control, CR LF both ways (manual 5.4.1; the AG-UC8PC's RS-232/485 jumpers
  run at 115200).
- `motorNewport.AG_UCController` — the driver on that port: `num_axes` (8 for
  an AG-UC8), poll periods and the two switches below.
- `motorNewport.AG_UCAxis` — one axis of a pair (`channel` 1-4, `axis` 1-2;
  driver axis `(channel-1)*2 + axis-1`) as a motor record on `SMC100Motor.db`
  (the patch's generic asynMotor record: RSTM Never, RTRY 0, DISP), in steps
  (MRES 1, PREC 0). Only axes with an entity are polled.

**Modes.** The controller powers up in local mode, where it answers only
`VE`, `TS` and `TE`; `TP`, `PH`, `SU?`, `DL?` and every motion command answer
nothing (an error -5 in `TE`). `MR` (remote) enables them. On the first unit
(firmware `AG-UC8 v2.2.1`, 2026-10-29) a reply is the command echoed with its
value (`1TP0`, `1SU+35`, `1SU-35`, `1DL0`, `PH0`, `CC1`, `TE0`), about 16 ms
after the command; `MR` and `CCn` reply nothing. That unit's step amplitudes
read 35 in both directions on every axis after a power-up, not the 16 the
manual gives.

### What the stock driver sends, and the patch

At creation `AG_UCCreateController` sends `RS` (reset), `MR` and `VE` (the
model from its reply); each `AG_UCCreateAxis` sends `CC<pair>` and
`<axis>SU<forward>`, `CC<pair>` and `<axis>SU<reverse>` (a non-positive
forward amplitude becomes 50; a non-negative reverse one was meant to become
-50, but upstream assigns the forward one instead, so the default forward
amplitude becomes -50: a second reverse setting). Each poll of an axis sends
`CC<pair>`, `<axis>TP`, `<axis>TS` and `PH`; a move `CC` then `<axis>PR<n>`
(an absolute move is `PR` to the target minus the last count read); a jog
`JA<speed code>`; a home, only with `has_limits`, `MV<speed code>`; a stop
`<axis>ST`; a position redefine (SET) sends nothing (an offset in the driver).

- **`RS` and `SU` at every boot.** A reset sets every temporary setting to
  its default and leaves local mode (manual, `RS`), and the `SU` lines then
  overwrite the step amplitudes the unit was driven with: a step's size, and
  so every calibration in µm per step, changes at an IOC start.
- **`CC` is unchecked.** The AG-UC8 refuses `CC` (error -6) while an axis of
  the selected pair moves (manual 5.7), and the driver ignores the refusal.
  While one pair moves, the poller's `CC` for every other pair is refused and
  those axes' polls read the moving pair's `TP` and `TS` as their own; a move
  or stop sent then for an axis of another pair goes to the moving pair's
  axis of the same number (`PR` is refused by a moving axis, accepted by an
  idle one: the wrong actuator moves).
- **One position per axis slot, not per axis.** The AG-UC8's step counter
  belongs to the slot (axis 1 or 2), shared by the four pairs: on the first
  unit (2026-10-29) `1PR10` on pair 1, then `CC2` (accepted, `TE0`), and
  `1TP` answered `1TP10`. Upstream reads `TP` as each axis's position, so
  every pair's axis n shows the same position, and an absolute move on one
  pair is computed from steps another pair took.

`motorNewport-geecs.patch`'s AG_UC part adds two iocsh variables, both
defaulting to upstream behaviour (`variable()` lines in `devNewport.dbd`):

| Variable | 1 (upstream) | 0 |
|---|---|---|
| `AG_UCInitWrites` | `RS` at creation, `SU` forward and reverse per axis | neither: creation sends `MR` and `VE` only |
| `AG_UCChannelUpstream` | `CC` before every command and every axis poll, result ignored; position = `TP` | the selection kept and checked, each axis its own count, a STOP never behind a `CC` (below) |
| `AG_UCJogAllowed` | the record's jog sends `JA` at a speed code | refused, nothing sent |

`AG_UCController`'s `init_writes`, `channel_upstream` and `jog_allowed` render them and
default to false; they are global to the IOC (`when: first`: a second
controller with other values gets the first one's, unchecked). With
`AG_UCChannelUpstream` 0:

- **The selection.** The driver keeps the pair the controller has selected
  and whether that is known now, apart from `refPair_`, the pair the slots'
  counts are credited to. `CC` goes only on a pair change, then `TE`:
  `TE0` selected; `TE-6` busy (the selected pair steps, the selection
  unchanged); anything else (no reply, `TE-5` in local mode, another code, a
  malformed reply) leaves the selection **unknown**, never "none". It is
  learned again with `CC?`, or, if that goes unanswered, from a slot that
  steps (`1TS`/`2TS` not 0): then the selected pair is `refPair_`, the only
  pair this driver has moved.
- **The counts.** Every change of a slot's `TP` is credited to `refPair_`'s
  axis in that slot. `refPair_` changes only after an accepted `CC`: the pair
  it leaves was idle (or the `CC` would have been refused), the counters
  carry over, and nothing has moved the new pair yet, so the slots are read
  then (`1TP`, `2TP`), credited to the old pair, and become the new pair's
  references. A move goes only to the selected pair once it is `refPair_`,
  so no step is lost or credited to the wrong axis whichever reply goes
  missing. Each axis is 0 when the IOC starts. `TP` and `TS` replies that do
  not echo their command are errors.
- **The poll.** Busy: the axis is done (it cannot move), its count kept.
  A timeout, a malformed reply, an unknown selection or a `CC` refused with
  another code (`TE-5` in local mode): the **comms-error bit**
  (`CNTRL_COMM_ERR`: COMM/INVALID on the record) with done kept as last
  read. Unlike the problem bit it does not make the record set MOVN 0 or
  clear its jog buttons, so a stepping axis never reads DMOV 1 through a
  missing reply, and a jog in progress can still be released. The problem
  bit is not used for these.
- **Move, home.** Sent only once the pair is selected; an absolute target is
  computed from the axis's count read just before. Refused (busy, unknown,
  or a home without `has_limits`): nothing sent, a console line, and no
  alarm: the axis is idle and fine. The record ends the move done with VAL
  left at the target and RBV where the axis is (on the fake, MISS stayed 0).
  No latch: b8 latched the problem bit, after which the record cleared the
  next jog's buttons, its release sent no `ST` and the jog ran past DHLM
  (the re-review of #18).
- **Jog.** `AG_UCJogAllowed` (below) 0 refuses it, nothing sent. Allowed and
  refused (busy, unknown), or refused by the switch: the problem bit for
  that one update, so the record clears its jog buttons rather than asking
  again; the next poll clears it. An accepted move, jog or home clears the
  problem and comms-error bits at once.
- **Stop never waits for a `CC`.** Known and another pair selected: nothing
  on this pair moves, nothing sent. Known and this pair: `<axis>ST` at once.
  Unknown: `<axis>ST` at once only for an axis of `refPair_`, the only pair
  that can be stepping; a stop for an idle axis of another pair sends
  nothing (it would stop the stepping pair's axis of the same number: the
  record sends a stop even to an idle axis).
- **A failed resync** is not retried for 1 s, so a controller that answers
  nothing (local mode, a stall) does not hold the poll lock, and a STOP
  behind it, for a `CC?` timeout per axis.

Outside the switches one fix: `PH2` sets axis 2's limit (upstream tested
`PH3` twice). Upstream's step-amplitude lines are unchanged, so with
`init_writes` true a reverse amplitude of 0 still sends `SU-50` as the
forward amplitude and then `SU0` (refused by the controller). `MR` stays:
nothing but `VE`, `TS` and `TE` answers without it. The channel check and the
per-axis count are worth proposing upstream as they stand; the init writes as
an option.

The comms-error, jog and stop-for-another-pair rules above are 2026.10.3b9
(the re-review of #18); the selection rules 2026.10.3b8 (the review of #18: on
b7 a single late reply mid-move zeroed the selection, after which the moving
axis read done, its STOP went behind a refused `CC` and was dropped, and its
steps were credited to nobody). On the first unit (geecs-epics `agtest1`,
2026-10-29, tag 2026.10.3b7): a
boot sends `MR` and `VE`, then the poll's `CC1`/`CC2` and queries only; all
four configured axes moved ±10 and ±50 steps through the record and back,
each reading its own count; the poll's `CC` was refused (`TE-6`) while a pair
stepped and the poll skipped as above. 2026.10.3b3 read `TP` as upstream
does (the per-slot finding); b4 and b5 lost steps between a pair's last poll
and the `CC` (the fake) and are superseded.

### Known gaps (R1-3, with the patch), from reading it

- **Limits.** `PH` reports one bit per axis of the pair with no direction,
  and the driver sets both the low and the high limit from it; stages without
  a switch always read `PH0`.
- **Velocity is not a setting.** `PR` runs at the controller's own rate
  (the step delay `DL`, 10 µs units, 0 on the first unit); `VELO` only picks
  the jog's speed code (`JA1`..`JA4`: 5, 100, 666 or 1700 steps/s, the last
  two at the maximum amplitude) and the record's time estimate.
- **No re-entry to remote mode.** A USB replug is a power cycle (the AG-UC8
  is USB-powered): local mode, counters at 0. The IOC sends `MR` at creation
  only, so afterwards every axis shows the comms-error bit, COMM/INVALID (its `TP` unanswered,
  a `CC` answered `TE-5`, `CC?` unanswered) until the IOC is recreated, and
  its counts are those of before the replug.
- **`CC?` while a pair steps** is not proven on the unit (it answered while
  idle); the `TS` fallback covers it.
- **Positions are counts since the IOC started.** With the patch each axis
  starts at 0 when the IOC does (the controller cannot say which pair took
  the steps its slot counter holds); nothing in the controller survives a
  power cycle either. SET is an offset in the driver, and upstream computes
  it from the offset-included position, so a second SET without a move in
  between lands off by the first offset (unpatched; SET is not used here).

## Aerotech Ensemble

`drvEnsembleAsyn` (motorAerotech R1-1-1; master after it changes CI files
only) is an old-style asyn motor driver: a `motorAxisDrvSET_t` named
`motorEnsemble` under motor's `drvAsynMotorConfigure`, DTYP `asynMotor`.
`EnsembleAsynSetup` once, then per controller an IP port, `EnsembleAsynConfig`
and `drvAsynMotorConfigure`. ASCII over the controller's command socket
(Socket2, TCP 8000 by default), LF both ways; a reply is `%<value>` (ACK),
`!` (NAK), `#` (fault) or `$` (timeout) (the driver's `ASCII_*` defines, which
must match the controller's AsciiCmd* characters). The ibek entities, in
`ibek-support-geecs/motorAerotech/motorAerotech.ibek.support.yaml`:

- `motorAerotech.EnsembleSetup`: once per IOC, `max_controllers` and the
  switch below.
- `motorAerotech.EnsembleController`: the IP port (`<name>_IP`), the
  controller (`num_axes`, poll periods) and the motor port (`<name>`); `trace`
  switches the asyn trace on between the IP port and `EnsembleAsynConfig`,
  because that call talks to the controller at once.
- `motorAerotech.EnsembleAxis`: one motor record on `EnsembleMotor.db` (added
  by the patch: the motor record alone, `RSTM` Never, `RTRY` 0, and `DISP`,
  which the entity sets to 1 by default: every put to a record field but
  DISP itself is refused (CA, the iocsh's dbpf, and QSRV1's PVA puts, which
  check it; database links inside the IOC bypass it). An instance that
  writes sets `DISP: false`.

**Axis numbering.** `EnsembleAsynConfig` probes `GETPARM(@n, AxisName)` for
n = 0..9 and takes the first `num_axes` that ACK, in order, as driver axes
0..num_axes-1. A sparse map is not supported: a missing drive shifts every
later axis down one record, so the instance must check which axis numbers
answer (geecs-epics' `ens_snapshot.py` prints them). A drive whose stage is
unplugged still answers and keeps its slot.

**Resolution.** The driver reports positions in counts: the controller's
program-unit positions times `CountsPerUnit`. `MRES` = 1 / `CountsPerUnit`
(read from the controller), with `EGU` its `UnitsName`; the entity has no
default for `MRES` on purpose.

**Readback.** The poller reports `PFBKPROG` (the feedback, the encoder) as
the encoder position and `PCMDPROG` (the commanded position) as the motor
position (drvEnsembleAsyn.cc L772-L795), both in counts with the same
`CountsPerUnit`. `EnsembleMotor.db` takes `UEIP` (default No, the record's);
the `EnsembleAxis` entity renders **`UEIP` Yes by default**, so `RBV` is the
encoder: `ERES` 0 becomes `MRES` at init (motorRecord.cc L692-L695), the
encoder ratio is 1 (the driver ignores the record's ratio push, "does not
support setting encoder ratio", and needs none) and the scaling is exact. The
record falls back to the commanded position, for good, on an axis whose
status shows no feedback device when the record first reads it
(`PositionFeedbackType` 0, or a status not yet delivered at record init: no
`EA_PRESENT`, motorRecord.cc L3671-L3675); geecs-epics' enstest1 harness
reads `UEIP` back and refuses to run without Yes. `ATHM` then follows the
encoder's home bit, which this driver never sets (it reads 0). A position
reply that is not an ACK (`PFBKPROG`, `PCMDPROG`) leaves the position as it
was and sets CommError (the patch; upstream set the encoder position to 0 and
read the command unchecked). Why: a disabled servo axis drifts and its command does not
(U_CompAerotech's separation axis, 2026-10-29: 0.18 to 0.24 um from the
command right after a `DISABLE`, about 0.2 um more over 15 minutes), so the
commanded readback hides where the stage is. With `RTRY` 0 a move is still
absolute (`LINEAR` to the target; relative moves need RTRY non-zero and an
encoder, motorRecord.cc L2289), and nothing new goes on the wire.
**`RDBD`** (entity parameter, **no default**: below |MRES| the record raises
it to |MRES|) is the in-position band. With `RTRY` 0 the record neither retries nor sets
`MISS` (motorRecord.cc `maybeRetry`, L1054-L1056: `MISS` is set only once
`RTRY` retries ran out) and raises no alarm on a position outside it; a
position outside the band shows in `RBV` and `DIFF`. It also sets the
new-target monitor's deadband, `NTMF` x (|`BDST`| + `RDBD`), 2 x `RDBD` by
default (next section): with the encoder as RBV an overshoot or settle beyond
it during a move makes the record send a STOP (`ABORT`), under every policy,
so `RDBD` must sit well above the settle jitter.

### What the stock driver and record send, and the patch

At `EnsembleAsynConfig` (drvEnsembleAsyn.cc, R1-1-1 line numbers):

1. `NONE` (L944, up to three tries): an invalid command, sent to get any
   reply (NAK expected). Not a query.
2. Per axis number until `num_axes` are found: `GETPARM(@n, id)` for
   AxisName (L959), PositionFeedbackType (L972), CountsPerUnit (L980),
   HomeOffset (L991), HomeSetup (L996), EndOfTravelLimitSetup (L1002),
   ReverseMotionDirection (L1012). Queries.
3. **`RAMP MODE @n RATE`** per axis (L1008): sets the axis's ramp mode. A write.
4. **`WAIT MODE NOWAIT`** once (L1020): sets the ASCII interpreter's wait
   mode (per task in Aerotech's later naming, SetupTaskWaitMode; whether the
   command sockets share that task is not in any public document). A write.

The poller (`EnsemblePoller`, at start, then every idle/moving period) sends
per axis `AXISSTATUS(@n)` (L715), `PLANESTATUS(0)` (L740), `PFBKPROG(@n)`
(L772), `PCMDPROG(@n)` (L791), `AXISFAULT(@n)` (L800), `VFBK(@n)` (L836):
queries. A faulted axis only sets the record's PROBLEM bit and prints the
fault once per change (L819-L832); nothing is sent to clear it.

The motor record (motor R7-4) adds nothing on the wire at init with this
driver: `init_record` pushes DHLM/DLLM (motorRecord.cc L716-L717 →
devMotorAsyn `SET_HIGH/LOW_LIMIT` → `motorAxisSetDouble`, which prints
"Driver does not set Ensemble's High/Low Limit" and sends nothing, L376-L385);
`init_controller` writes the encoder ratio (devMotorAsyn.c L189-L190; the
driver prints "does not support", L364-L368) and, when `RSTM` asks for it,
the position: **`POSOFFSET SET @n, <pos>`** (L355-L362), a position redefine
(devMotorAsyn.c L173-L233: Always; NearZero when DVAL is beyond RDBD and the
controller reads within RDBD of zero, which a restored DVAL, e.g. from
autosave, produces; Conditional on the same, or whenever RTRY is non-zero,
RMOD is not Inverse and UEIP or URIP is set). `EnsembleMotor.db` defaults
`RSTM` to Never and `RTRY` to 0; both are load-bearing. The record never writes CNEN at init (it only mirrors the drive's
enable bit, motorRecord.cc L3540-L3548).

Write paths that open only on a put (an instance that must not change the
controller refuses them: access security, `DISP`): CNEN (L429-L466:
**`FAULTACK @n`** if a fault is latched, **`ENABLE/DISABLE @n`**,
**`WAIT MODE NOWAIT`**), a move (L517-L531: **`ABS`/`INC`**, **`RAMP RATE`**,
**`LINEAR @n <pos> F<vel>`**), a jog (L621-L627: **`SETPARM`**, **`RAMP RATE`**,
**`FREERUN`**), a home (L567-L594: **`SETPARM`** x2, **`IGLOBAL(32/33) =`**,
**`PROGRAM RUN 5, "HomeAsync.bcx"`**), STOP (L666: **`ABORT @n`**), SET mode
and, in Use mode, a put to MRES, ERES or UEIP (motorRecord.cc L1937-L1990,
`load_pos`) (**`POSOFFSET SET`**). The driver never sends `ACKNOWLEDGEALL`.

`aerotech-ensemble.patch` (applied by `motorAerotech.install.yml` on upstream
R1-1-1; `drvEnsembleAsyn.cc`, `devAerotech.dbd`, `Db/Makefile`, and the new
`Db/EnsembleMotor.db`) carries two changes, each upstream behaviour by default:
the read-only init (here) and the per-axis power policy (next section). The
first is one iocsh variable:

| Variable | 1 (upstream) | 0 |
|---|---|---|
| `EnsembleInitWrites` | probe with `NONE`; `RAMP MODE @n RATE` per axis and `WAIT MODE NOWAIT` at `EnsembleAsynConfig` | probe with `PLANESTATUS(0)` (a query the poller sends anyway); the two mode commands go before every move, jog and home instead (`sendMotionModes`), and the motion command is not sent unless both are acknowledged (`%`): without WAIT MODE NOWAIT the interpreter blocks during a move (motorAerotech issue #6: timeouts, a frozen position, a motor that cannot be stopped). Sending them each time also covers a controller restarted after the IOC |

The `EnsembleSetup` entity renders it from `init_writes`, which **defaults
to false**: a boot and the idle poll then send the controller queries only.
The `variable()` line in `devAerotech.dbd` is what makes `var` work (without
it iocsh prints "No known var" and the stock behaviour runs). Checked against
geecs-epics' Ensemble fake (`sim/ensemble_sim.py`, which logs every
non-query as `WRITE`): with the switch at 0 no `WRITE` at init or in the
poll; at 1 the fake logs `NONE`, three `RAMP MODE`s and `WAIT MODE NOWAIT`
(geecs-epics `services/enstest1`). Not yet checked against a controller.
Worth proposing upstream (a query probe; mode commands with each motion command).

### Power policy (per axis)

`drvEnsembleAsyn` is a model 2 driver (`motorAxisDrvSET_t`): model 3's
auto-power (`asynMotorController`, `asyn_auto_power.db`) does not exist for
it, and upstream's driver never enables or disables an axis on its own (only a
CNEN put does: `ENABLE`/`DISABLE`, with a `FAULTACK` first on enable if a fault
is latched). The patch adds a per-axis policy, set after `EnsembleAsynConfig`
by one iocsh command per axis that is not `leave`:

    EnsembleAxisPower(card, driverAxis, "on_for_move"|"keep_on"|"leave", onDelay_s, offDelay_s)

rendered by the `EnsembleAxis` entity from `power_policy` (default `leave`: no
line), `power_on_delay` and `power_off_delay`. The fields are the contract, not
the mechanism: a model-3 driver would render the same three onto
`AutoEnable`/`AutoEnableDelay`/`AutoDisableDelay` (and a `DLY` covering the off
delay, plus a `setClosedLoop` that refuses false for `keep_on`).

| `power_policy` | Before a move, jog or home | After it | CNEN puts |
|---|---|---|---|
| `leave` (default) | nothing (a move on a disabled axis faults at the controller, `#`, as upstream) | nothing | as upstream |
| `on_for_move` | if `AXISSTATUS` reads disabled: `ENABLE @n`, which must be acknowledged (`%`), then `onDelay`, then `AXISSTATUS` must read enabled; otherwise no motion command, PROBLEM set. Never `FAULTACK` | done (DMOV) held at 0; `offDelay` after the end of motion (the poll that first sees it, moving poll period), `DISABLE @n`, which must be acknowledged, read back disabled (`AXISSTATUS` bit 0, `axis_enabled`, drvEnsembleAsyn.h) and read disabled again by the next poll; only then done, with CNEN 0 | as upstream on the wire, then `AXISSTATUS` read back (upstream checks neither reply): PROBLEM if the drive did not follow; a CNEN put ends a pending disable (the put decides) |
| `keep_on` | the same enable | nothing | **CNEN 0 refused**: nothing sent, PROBLEM set, CNEN reads back the drive |

Then the mode commands (`RAMP MODE`, `WAIT MODE NOWAIT`, both acknowledged or
no motion, with `init_writes` false) and the motion command, as before. So an
`on_for_move` move on the wire is `ENABLE`, `onDelay`, `RAMP MODE @n RATE`,
`WAIT MODE NOWAIT`, `ABS`, `RAMP RATE`, `LINEAR`, end of motion, `offDelay`,
`DISABLE`, and DMOV goes 1 one poll after the `DISABLE` is acknowledged and the
drive reads disabled (upstream's CNEN path never checks either reply, so its
CNEN readback alone proves nothing; this path does not trust a reply): a scan waiting
on DMOV does not fire with the axis enabled. The exceptions, where the motor
record ends the move itself (PROBLEM or a limit switch, motorRecord.cc
L3741-L3746): a controller fault during the motion is disabled at once, in the
poll that sees it, before DMOV (one `DISABLE`, acknowledged and read back, but
no confirming poll and no retry: PROBLEM ends the move anyway); an `AXISFAULT`
read that is not acknowledged,
or a limit switch reached without a controller fault, ends DMOV while the axis
is still held, and the disable follows `offDelay` later. Details:

- **Boot, idle poll, IOC exit**: unchanged, queries only, under every policy.
  An axis found enabled at boot stays so until its first `on_for_move` move
  ends. Nothing runs at exit: an IOC killed mid-move, or inside the off window,
  leaves the axis enabled (the controller finishes the `LINEAR` on its own).
  Safe-off on an abnormal exit is the harness's (geecs-epics
  `services/enstest1/phase_b.sh`: STOP, CNEN 0, verified) or the controller's.
- **STOP/abort**: `ABORT @n` as upstream; its end of motion starts `offDelay`,
  then `DISABLE`. A STOP put during `onDelay` waits (the port thread sleeps)
  and aborts the move just sent. **A STOP once the motion has ended** (the
  poller has seen its end, the axis held for its `DISABLE`, in the off window
  or after the `DISABLE`) sends nothing when a fresh `AXISSTATUS` and
  `PLANESTATUS(0)` read no motion and no home cycle (`powerStopNotNeeded`, a
  console line); any motion, a reply that is not clean, or a held home
  (HomeAsync.bcx's several moves) sends the `ABORT`. It trusts the status bits
  as the poller's end of motion does: a motion the controller acknowledged but
  does not yet show would also be taken for its end by the poller (the
  `offDelay` floor guards that).
  Why: the hold keeps DMOV 0, so the record still sees MOVN 1 and runs its
  new-target monitor (motorRecord.cc `process()`, L1325-L1340, NTM Yes by
  default): a readback past the target by more than `NTMF` x (|`BDST`| +
  `RDBD`) against the commanded direction is a "wrong direction" to it (it
  prints `<record>:tdir = <n>`) and it asks for a STOP. On U_CompAerotech
  (2026-10-29, the commanded readback, `RDBD` = MRES) the command stepped 43
  counts at each `DISABLE` and the record sent `ABORT @0` about 30 ms after
  it on the three downward moves of five: a command to a drive just disabled.
  A fault or a limit is the poller's to report and is unchanged.
- **A refused move**: by the record's soft limits, nothing reaches the driver
  (no enable); by the controller (a `LINEAR` answered `#`, failed mode
  commands): the axis was enabled for it, so it is disabled `offDelay` later,
  the same path.
- **A failed `DISABLE`** (not acknowledged, or acknowledged with the drive
  still reading enabled, then or on the next poll): retried on the next polls,
  three tries in all, then done **with PROBLEM** (MAJOR alarm) and a console
  line, CNEN and MSTA 0x20 showing the drive's own enabled bit: never
  done-and-disabled on a reply alone. Likewise an `ENABLE` acknowledged with the
  drive still reading disabled after `onDelay` sends no motion command (PROBLEM;
  under `on_for_move` a `DISABLE` follows at once).
- **Status unreadable while held**: ten polls in a row without a clean
  `AXISSTATUS` end the hold with done, PROBLEM and CommError (an alarm, not a
  DMOV 0 forever).
- **PROBLEM** from a refusal or failure is sticky until the next accepted
  motion or a CNEN put the drive is read to have followed (the poll's own
  fault check would otherwise clear it).
- **Done is plane-wide** (`PLANESTATUS(0)`, upstream): another axis moving in
  plane 0 delays this axis's `offDelay` and its DMOV.
- **New motion during a move or its off window**: the record still reads not
  done, so a new VAL (or a jog) is a retarget (the record stops, waits for done,
  then moves): during the motion `ABORT`, `offDelay`, `DISABLE`, `ENABLE`,
  `onDelay`, the move; once the motion has ended, the same without the `ABORT`
  (the STOP bullet). `on_for_move` wants `BDST` 0 and `RTRY` 0 (the
  entity's defaults): a backlash or retry move power-cycles the axis.
- **Home**: the off delay starts only once `AXISSTATUS` shows neither
  `move_active` nor `homing_active` (HomeAsync.bcx runs several moves); not yet
  checked against the controller's home program.
- **Limits checked at boot** (`EnsembleAxisPower` prints and sets the iocsh
  error; the entity renders `on error halt` before it, so a refused policy
  halts the IOC rather than booting as `leave`, and the command is refused
  after iocInit): policy one of the three; `onDelay` 0..1 s (it sleeps in the
  controller's asyn port thread with the axis mutex held: every command to
  that controller, a STOP of another axis included, and the poll wait);
  `on_for_move` needs `offDelay` ≥ the moving poll period (a shorter one can
  take a not-yet-started motion for its end).
- **Locking**: for a non-`leave` axis the axis mutex is held from the enable
  check to the motion command, so the poller's `DISABLE` cannot fall between
  them; `leave` takes upstream's path unchanged.

Proved on geecs-epics' Ensemble fake (`services/enstest1/power_proofs.sh`, its
record in `services/enstest1/README.md`). **On the controller** (2026-10-29,
U_CompAerotech's separation axis, five `on_for_move` moves, LabVIEW stopped,
geecs-epics `services/enstest1/phase-b-power-2026-10-29-hw2.txt`): the policy's
order held on every move (`ENABLE`, the mode commands, `ABS`, `RAMP RATE`,
`LINEAR`, `DISABLE`, done after the confirming poll), but the record sent the
unplanned `ABORT` above. **The rerun on the encoder readback and the STOP
change is owed**, the maintainer's to schedule. `dbior motorEnsemble 1` prints
each axis's policy and state.

### Known gaps in the upstream driver (R1-1-1), from reading it

- **No sparse axis map** (above): the driver index is the answer order.
- `motorAxisOpen` bounds-checks `card > numEnsembleControllers` (L306) and
  `axis > numAxes` (L308), one past each array; a record with ADDR =
  num_axes reads past it.
- If fewer than `num_axes` drives answer, the trailing driver axes have no
  controller: their records load but never update.
- If the probe gets no reply, `EnsembleAsynConfig` returns before the axes
  and the poller exist; the records then sit with no driver behind them
  until the IOC is restarted with the controller reachable.
- The controller closes a socket idle longer than its `Socket2Timeout`; the
  idle poll (a period below it) is what keeps the socket open
  (`docs/README_Ensemble.md`).

## PI hexapod (motorPIGCS2)

`motorPIGCS2` (epics-motor, R1-3-1) is PI's own GCS driver family: one
`PI_GCS2_CreateController` per controller, which asks `*IDN?` at creation and
picks a controller class from the answer (the reply is upper-cased before the
match). PI's old "Hexapod" controller (the DOS controller of the M-8x0
hexapods, `*IDN?` `PHYSIK INSTRUMENTE (PI),Hexapod,0,V5.1.8 ...`, GCS-1)
lands on `PIHexapodController`: six axes X Y Z U V W (driver axes 0-5),
positions in mm and degrees, the driver's counting unit a fixed 1e-4. The
first unit is GEECS-BELLA/geecs-epics `services/hextest1` (its README has the
dialect as the controller answered it; no manual exists for this controller).
The ibek entities (`ibek-support-geecs/motorPIGCS2/motorPIGCS2.ibek.support.yaml`):

- `geecsSerial.Port` — the line: 57600 8N1, no handshake, LF. The driver
  sets the asyn port's terminators itself (input `\n`, output none: it
  appends LF to every command and sends the single-byte commands bare).
- `motorPIGCS2.PIHexapodController` — the driver on that port, six axes,
  poll periods and the eight switches below; loads `PIHexapodCtrl.db`
  (`DEFER`, `STATUS_WORD`, `REFERENCED`, `INITIALISING`, `STATE`, `INIT`,
  `RBPIVOT_X/Y/Z`, `LAST_ERR`).
- `motorPIGCS2.PIHexapodAxis` — one axis as a motor record on
  `PIHexapodMotor.db` (added by the patch: RSTM Never, RTRY 0, DISP on by
  default), `ADDR` from the axis letter, `MRES` 1e-4.

**Deferred moves.** `$(P)DEFER` ON, a VAL per axis, `DEFER` OFF: the driver
sends one `MOV X .. Y .. ..` for every deferred axis (the GEECS `xyzuvw` set).
With `DEFER` OFF each VAL is its own `MOV <axis> <target>`.

**Limits.** The record's DHLM/DLLM are the GEECS DB's operating ranges; the
hexapod's workspace is coupled, so the controller's own `VMO` (virtual move)
is the real limit, asked before every `MOV` when `vmo_check` is on (below).

**Referenced, and the reference run.** The controller's `#4` answers a
decimal status word (PI's doc text for the M-8X0 in its GCS LabVIEW library:
bits 8-13 struts in motion, 16 "hexapod initialized", 19 "initialisation in
progress"). Every poll pass reads it once, after `#5`: `$(P)STATUS_WORD` is
the word, `$(P)REFERENCED` bit 16 (and every axis's MSTA homed bit),
`$(P)INITIALISING` bit 19, and `$(P)STATE` the summary a GEECS `status`
variable wants: `Not referenced` (bit 16 clear), `Homing` (bit 19, or an
`INI` this IOC sent and `#5` still non-zero), `Moving` (`#5`), `Ready`. A
pass whose `#5` or `#4` fails leaves the four records INVALID until the next
good one. While the word says not referenced, and while a reference run is
in progress, the driver refuses every `MOV` with an error and sends nothing
(`checkReferenced`: a VAL, a deferred set, a jog). After a controller power cycle the
platform is not referenced: `$(P)INIT` (a bo, HIGH 1 s) sends `INI`, the one
reference run for every axis (positions become 0; the platform moves to its
reference pose), and so do the record's HOMF/HOMR on any axis, both only with
`home_allowed` and never while the platform moves; a refusal is a line in the
IOC log and `STATE` unchanged (the bo's write alarm lasts until HIGH resets
it, the motor record raises none). `auto_init` sends it once at creation when
the word says not referenced, as the LabVIEW device does at its start, after
the six axes have asked their init queries (so they neither race the run nor
time out on a controller silent during it). `INI` goes without an
axis identifier (PI's doc text for INI on the M-8X0 and F-206: "All axes =
TRUE, Axis identifier = FALSE"; upstream sent `INI X`), and the same text
warns that a motion started by `INI` may not be stoppable by `#24`.

### What the stock driver sends, and the patch

Creation: `*IDN?`, `VEL?` (the controller knows it, so the driver sends
velocities), `#4` and, with `pos_with_char3`, `#3` (this controller answers
`#3` with six lines, which upstream's single-byte read cannot take: see the
gaps; our entities never send it), `SPI?
RST`; then per axis **`SVO <axis> 1`** (a write), `VEL? <axis>`, `POS?
<axis>`, `#4`. The poller: `#5`, then `POS? <axis>` per axis, idle and (with
`pos_with_char3` false) moving alike. A move:
**`VEL <axis> <v>`** (the record's VELO) then `MOV <axis> <target>`, `#5`
0.2 s later, `ERR?` if not moving. A home: **`INI X`**, the hexapod's
reference run (every axis, to 0). A stop: the single byte `#24`, then `ERR?`
expecting 10.

`motorPIGCS2-geecs.patch` adds eight iocsh variables, each defaulting to
upstream behaviour (`variable()` lines in `PI_GCS2Support.dbd`), and the fixes
below the line:

| Variable | upstream | the other value |
|---|---|---|
| `PIHexapodInitServo` | 1: `SVO <axis> 1` per axis at init | 0: `SVO? <axis>` read and reported, nothing set |
| `PIHexapodHomeAllowed` | 1: `$(P)INIT`, and HOMF/HOMR on any axis, send `INI` (upstream: `INI X`, see below) | 0: refused with an error, nothing sent |
| `PIHexapodVmoCheck` | 0: `MOV` straight away | 1: `VMO` with the same targets first; a `1` (not possible) refuses the move with an error, `MOV` not sent |
| `PIHexapodSendVel` | 1: `VEL <axis> <v>` before every move | 0: nothing; the platform's velocity stays the controller's |
| `PIHexapodPosWithChar3` | 1: `#3` probed at init and used for positions while moving | 0: `#3` never sent; positions while moving come from `POS? <axis>` (upstream's single-byte read cannot take a multi-line reply: see the gaps) |
| `PIHexapodJogAllowed` | 1: JOGF/JOGR send `MOV <axis> <travel limit>`, no `VMO` | 0: refused with an error, nothing sent |
| `PIHexapodSetPosAllowed` | 1: SET (position redefine) sends `RON <axis> 0`, `POS <axis> <v>`, `RON <axis> 1` | 0: refused with an error, nothing sent |
| `PIHexapodAutoInit` | 0: nothing (upstream has no such step) | 1: `INI` once at creation, after the axes' init queries, when `#4` says not referenced (the LabVIEW device's start-up step); needs `PIHexapodHomeAllowed` 1 |
| `getStatus` servo state | left uninitialized (random problem bit) | the state the driver knows (fix, no switch) |
| `INI` | `INI X` | `INI` with no axis identifier: PI's doc text for the M-8X0 and F-206 (an identifier names the separate axes A, B) (fix, no switch) |
| `#4` | read per axis at init and after a home | also once per poll pass and after every `MOV`, for the controller-level records and every axis's homed bit; a failed read marks the records INVALID (fix, no switch) |
| a `MOV` while not referenced, or during the reference run | sent (the controller's answer unknown) | refused with an error, nothing sent: a VAL, a deferred set or a jog (fix, no switch) |
| `PIasynAxis::setIntegerParam` | `asynMotorAxis`'s, which writes the aggregate `motorStatus_` parameter on every call and writes 0 for a parameter outside the status-bit range; `poll()` ends with one (`PI_SUP_SERVO`), so the record's init read of the status got 0 and MSTA stayed 0 until the first status callback (the first move) | parameters outside the range go to the parameter library directly; MSTA reads right from init (fix, no switch) |
| position counts | `int(x + 0.5)`: truncation toward zero reads -0.46 as -0.4599 | rounded to nearest (fix, no switch) |
| target counts | the record's counts (a double, 17.7 / 1e-4 = 176999.99999999997) truncated to int in `move` and in the deferred-move array: `MOV Y 17.699900` for a 17.7 target | rounded to nearest (fix, no switch) |
| `PIasynAxis::m_pasynUser` | never set: a null write on upstream's own `INI` failure path | set from the axis's asynUser at construction, so the refusal paths print to the IOC log instead of dereferencing NULL (the record never sees the message) (fix, no switch) |

`PIHexapodController`'s `init_servo`, `home_allowed`, `vmo_check`,
`send_velocity`, `pos_with_char3`, `jog_allowed`, `set_position_allowed` and
`auto_init` render them and default to false, false, true, false, false,
false, false, false: an IOC start sends queries only, the reference run is
refused until an instance sets `home_allowed`, every move is checked against
the controller's workspace, and the controller's velocity is left alone. They
are global to the IOC (every PI hexapod in it). The patch also adds
`PIHexapodMotor.db` and `PIHexapodCtrl.db` (`DEFER`, the status records,
`INIT`, the pivot readbacks and the last error: upstream's `PI_SupportCtrl.db`
pivot setters and `PI_SupportError.db`'s `CLEAR_ERR` are not loaded).

### Known gaps (R1-3-1, with the patch), from reading it

- **A refused move ends where it is.** `VMO` answering `1` (or the
  controller refusing the `MOV`) returns an error from the driver's `move`;
  the record completes the move at the old position and the message is in
  the IOC log (`PIHexapodController::checkMovePossible() refused ...`), not
  in a PV. `LAST_ERR` shows the controller's error codes only.
- **`#3` and upstream's single-byte read do not mix.** `PIInterface::sendAndReceive(char)`
  reads one line, and while that line ends in a space (a continuation) it
  re-sends the byte and reads again into the rest of its 255-byte buffer;
  `writeRead` flushes the input first. On a controller whose `#3` answers six
  lines the loop ends only when the buffer is full of first lines: ~26 round
  trips per read under the port lock (the bench, 2026-10-29: `#3` ×26 at init,
  ×314 during one 1 mm move, a poll pass of ~14 s during which a STOP waits),
  and only the first axis parses, so the other five positions fail while
  moving. `PIHexapodPosWithChar3` 0 (our entities) never sends `#3`; positions
  while moving come from `POS? <axis>`, one aligned round trip each.
- **The travel limits the driver reports are fixed at ±100** (`getTravelLimits`);
  the record's own DHLM/DLLM are what limit a VAL.
- **`CanCommunicateWhileHoming` is false**: during an `INI` the per-axis
  poll reads no positions; `#5` and `#4` are still sent every pass, so
  `STATE` reads `Homing` and the records' readbacks catch up when `#5` reads
  0. Whether this controller answers during its reference run is for the
  bench to show (PI's "INI hexaxes and wait until finished" VI polls `#4`
  during it).
- **Positions** come from `POS? <axis>`, twelve round trips per poll pass
  at 57600 baud (the driver reads each axis twice, as counts and as units),
  idle and moving alike (with `pos_with_char3` false).
- **The `VMO` check covers target mode 0 only** (`MOV`); the driver's MRT/MRW
  modes go unchecked, and no record in `PIHexapodCtrl.db` sets a mode.
- **A stop during `INI`** sends `#24` like any stop; PI's doc text says a
  motion started by `INI` may not be stoppable by it. Not tried on hardware.
- **Not referenced is read, never assumed.** A controller that does not
  answer `#4` is treated as referenced (upstream's reading); the gate on
  `MOV` and the status records need the word.

## Velmex VXM + VRO

`VelmexDriver` (`motorVelmex/`, ours: no EPICS support for the Velmex VXM
exists anywhere public, 2026-10-29 search) is a "model 3" asynMotor driver
for the Velmex VXM stepping-motor controller (one to four motors on one
serial line) with a Velmex VRO encoder readout on a second line. The VXM is
open-loop and counts steps; the readout is the position reference. The
driver reports both — motor position = the VXM's absolute position register
(steps), encoder position = the readout's displayed value over `ERES` — and
the motor record closes the loop the way the LabVIEW driver did (`UEIP`,
`RTRY`, `RDBD`: index, compare with the readout, index the difference).
Sources: the VXM user's manual and command summary (VXM-CS 12-29-04), the VRO
reference manual 1.30; both are in GEECS-Device-Drivers' VelmexStage tree.
The ibek entities (`ibek-support-geecs/motorVelmex/motorVelmex.ibek.support.yaml`):

- two `geecsSerial.Port`s — the VXM's and the readout's lines, 9600 8N1
  (both manuals' standard setting), **no terminators**: the devices answer
  a CR-ended value, a single letter or a single raw byte, and the driver
  reads each reply the way its command is documented.
- `motorVelmex.VelmexController` — the driver on those two ports: motors,
  poll periods; `VelmexController.db` adds the controller-wide readings
  (`$(P)vxm_status`, `$(P)vro_status`, `$(P)limit_byte`, `$(P)vxm_version`).
- `motorVelmex.VelmexAxis` — one motor as a motor record on
  `VelmexMotor.db` (RSTM Never, DISP on by default): its readout encoder
  (`encoder` 1 or 2, `ERES`), `MRES` (mm per step; no default), the VXM's
  acceleration setting (`accel_setting`, 0 = leave the controller's) and
  the home backoff.

**What goes down the lines.** Creation sends nothing; `VelmexAxisConfig`
runs one poll before iocInit, so the record's initial VAL is the real
position (the poller starts in the controller's constructor and its first
poll ran before the axis had its encoder, so it read the VXM's register but
not the readout; the record, initialising under UEIP from that status, took
an encoder position of 0: VAL 0, below DLLM, LVIO until the first move);
the first poll sends
`F` (the VXM On-Line, echo off) and the VXM stays On-Line while the IOC
runs, because in Local mode it answers only `V` and `X`: the limit byte,
`getD0,` and the `get...` reads are silent there, and a `D` dumps positions
instead of stopping (U_Velmex's VXM, firmware 1.37, 2026-10-29). Its
front-panel jog buttons are off meanwhile. Every poll: `V`
(status letter: `R` ready / `B` busy On-Line, `J` / `b` in Local mode) and,
when the VXM is idle (`R`), `?` (the limit inputs as one raw byte: bit 2(m−1) the motor's negative
limit, bit 2(m−1)+1 its positive; 0 = activated as the manual's table
labels it, which fits normally-open wiring; `limits_active_low: false` for
Velmex's own normally-closed wiring, where a pressed switch opens the
circuit and reads 1 (U_Velmex: 0x0C at rest, 0x0D on the negative limit);
255 = none activated or connected; on a two-motor VXM the upper nibble
reads 0) and `X`/`Y`/`Z`/`T` (motor 1-4's position register, `+0001200`
CR, readable while indexing) on the VXM; `V` (`D` display mode, `P` power-up
menu, `S` setup, `F` front-panel setup) and `X`/`Y` (encoder 1/2's displayed
value in its primary unit, CR) on the readout; `getD0,` once, On-Line
(firmware, 1.20 and up; older firmware is silent and the record reads "no
reply").
The readout answers `X` in Display mode without any mode command, and the
driver never sends it one (`E`/`F` would put it in Setup mode and blank its
buttons); a readout not in Display mode, or silent, is a problem on the
record while the VXM's position and done state are still read, and no move
is accepted until it answers again (no reference to close the loop on). A
move is done when the VXM is idle **and** the readout has read the same
value (within 2 counts) for 1.5 s, or 10 s after the VXM idled regardless
(a message): U_Velmex's VRO displays a value that lags the motion by up to
about a second, a retry judged against the stale value overshot by 90 mm
(phase B, 2026-10-29), and a display refreshed once a second can read the
same stale value on two polls half a second apart, so the wait is by the
clock, not by the poll count. A move commanded before the readout has
settled is refused (the record ends a program on a limit switch by its own
rule, before the settle: a move within about 1.5 s of a home is refused, and
the record's own retry re-issues it once the readout has settled, counted
in RCNT). A stop reads `V` afresh
and kills (`K`) any program it did not start, the front-panel Run button's
included. Every program is preceded by a fresh `V`: one sent
to a running VXM would replace the program it is running.
The readout's power-up start menu answers no position either (`V` says `P`);
`F` then `Q` on its line takes it to Display mode, which the snapshot's
`--identity` pass does and the driver does not. After a VXM power cycle the
poll re-sends `F`; a program sends it only as the fallback for a failed `F`
write. The exit
`Q` the driver registers runs on an iocsh `exit` only: a `docker stop` never
reaches epicsAtExit (the IOC is a child of the launcher and is killed after
the grace period), so a stopped IOC leaves the VXM On-Line until a `Q`, a
power cycle or the LabVIEW driver's own mode commands.
A move is `C,S<m>M<steps/s>[,A<m>M<x>],IA<m>M<target>,R` — always the
**absolute** index: the relative form's zero (`I1M0`) means "index to the
positive limit" and `I1M-0` the negative one, so a zero-length relative move
must never reach the wire. A home is `C,S<m>M<steps/s>,[I<m>M<±backoff>,]I<m>M-0,R`
(reverse, the LabVIEW driver's; `I<m>M0` forward): with a backoff the motor
first moves away from the switch, so it is always approached from one side;
neither the VXM's register nor the readout is zeroed. A stop is `D`
(decelerate the running index; the program then goes on to its next
command, which is fine for a one-index move) or `K` for a home with a
backoff (two indexes; `K` stops at once), `K` for a program this IOC did
not start (an IOC restarted mid-program), and nothing when the VXM runs no
program (Local mode, or On-Line idle): the record sends STOP on an idle axis
too, and in Local mode `D` is the digitizer dump (Appendix H), not a stop.
`?` and `getD0,` go out only when the VXM is idle: sent while a program
runs they are not answered (U_Velmex's VXM: only the `^` of the index's
end came, and the 1 s timeout ended the move on the record with a retry
against the stale readout, 2026-10-29). The limit byte keeps its last
value through a program; the VXM stops on a limit by itself and the first
idle poll reads it. A single failed poll (a timeout on either line) is a
comm error that keeps the last state and refuses moves until a poll
succeeds; the record sees a problem, and ends a move on it, only after
three failed polls in a row. The record's new-target monitor (`NTM`, an
entity parameter) is left at the record's default, on: it stops a move
only when the readback is past the target by more than NTMF × (BDST +
RDBD), which with this readout's lag means a real overshoot (in the settle
the VXM is idle and nothing is sent; during an index a `D` limits it), and
the record's retry logic then goes to the target within RTRY, so a stop
costs one retry. In the repeat of phase B (2026-10-29) its two stops were
refused by device support (the record was in COMM_ALARM) and the moves
that brought the stage back were the record's third retries. The VXM's `^` (its ready prompt, one character, no terminator, sent when a
program ends) lands anywhere in the reply stream and every VXM reply read
skips it, the raw limit byte included: a genuine 0x5E would need bits 4 and
6 set in an upper nibble that reads 0 on a one- or two-motor VXM (0x0C at
rest on U_Velmex's), while a `^` taken as the byte would end a negative move
on a false LLS with no retry.

**Refused, nothing sent:** jog (`moveVelocity`: the VXM has no host-driven
continuous move outside a stored program; `TWF`/`TWR` are moves and work),
`setPosition` (the VXM can only null its registers, `N` / `IA<m>M-0`, never
take a value; and the readout is the reference anyway), `setEncoderPosition`
(the readout's zero is set on the readout — its `C` button, or `N`/`C`/`<`/`>`
on its line — and moves every client's reference: a commissioning action,
not the record's). A second motor's move while a program runs is refused
too: the VXM runs one program at a time.

**Units.** The VXM takes steps; `MRES` is the stage's advance per step (the
lead screw's, or measured: index a known count, read the readout). The
readout answers its displayed unit (mm), scaled and quantised by its own
setup (encoder type, linear resolution, display resolution: its
configuration, read with its `get` commands in Setup mode, never written by
the IOC — geecs-epics decision 15's exception); the driver divides by
`ERES`, so `ERES` is a choice of counting unit (0.001 shows a micrometre and
keeps 2 km inside an int32), and its sign flips the readout's direction
against the motor's. Speed: `S<m>M` takes 1 to 6000 steps/s; the record's
`VMAX` holds the stage's ceiling (the GEECS DB capped the LabVIEW driver at
2000). The VXM's own ramp is its stored acceleration setting (1-127, `A<m>M`),
not the record's `ACCL`.

**Known gaps, from the manuals:** the limit byte is read once per poll for
every motor (one `?`), so a limit hit between polls shows on the next one;
`Z`/`T` (motors 3 and 4) answer only when stationary, so a four-motor VXM is
polled for those two only between moves (not implemented: one motor so far);
simultaneous motion of two motors (`(i1,i2)`) is not used; the readout's
`P` state (its power-up start menu, until its `C` button or an `F` then `Q`
on its line) stops the position readback — the driver reports it and sends
no mode command; the snapshot script's `--identity` pass does. The hardware
snapshot of 2026-10-29 (geecs-epics `services/vxmtest1`) settled the
Local-mode command set and the readout's reply format; the motion phase
measured MRES (0.2503 mm per step) and found the readout's display lag; the
limit polarity waits for a switch.

## Licence

GEECS's own work here (the Dockerfile changes, `motorVelmex`, the ibek
support files and patches under `ibek-support-geecs/`, the docs) is under
`LICENSE`, the GEECS licence: BSD-style, Copyright The Regents of the
University of California, through Lawrence Berkeley National Laboratory, as in
GEECS-Plugins. The files generated from epics-containers' ioc-template keep
its Apache-2.0 licence, `LICENSE-ioc-template`. The EPICS modules the image
builds (fetched at build time, not vendored) carry their own licences, listed
in the table above.

## Template Info
This repository was generated by
[ioc-template](https://github.com/epics-containers/ioc-template) 4.8.0
(`copier update -a --trust .` to update). The devcontainer and `./build`
work as in every epics-containers generic IOC; the IOC binary links
`$(SUPPORT)/motorNewport` through `ibek`.

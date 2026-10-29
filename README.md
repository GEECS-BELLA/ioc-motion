# Generic IOC ioc-motion

Motion controllers on the EPICS motor record, for GEECS (geecs-epics decision
14: stages are motor records; decision 11: one image per family). Built on
epics-containers' `ioc-motorsim` developer image, so it carries everything that
image does (motor R7-4, asyn, autosave, busy, calc, sscan, sequencer, iocStats,
pvlogging, **motorSim**) plus:

| Module | Version | Source | Reuse (decision 10) |
|---|---|---|---|
| motorNewport | R1-3 | [epics-motor/motorNewport](https://github.com/epics-motor/motorNewport) (ships no LICENSE file; tier 1 rests on the epics-motor org, geecs-epics device catalogue) | tier 1, dependency, with one build-time patch (below: ESP300, SMC100) |
| motorAerotech | R1-1-1 | [epics-motor/motorAerotech](https://github.com/epics-motor/motorAerotech) (ships no LICENSE file either; `drvEnsembleAsyn.cc` cites a synApps licence it does not ship); no upstream ibek-support recipe, so ours | tier 1, dependency, with one build-time patch (below) |

Drivers for the other families geecs-epics' device catalogue assigns to
this image (NewFocus Picomotor, Zaber, PI GCS2) join it when their first
device is ported.

## Build

No vendor code is inside, so the image could be published; until the
organisation's package registry is set up it is built where it runs:

```
docker build --target runtime -t ioc-motion:local .                              # on the IOC host
DOCKER_DEFAULT_PLATFORM=linux/amd64 TARGET=runtime TAG=ioc-motion:local ./build  # a workstation (amd64, emulated on Apple silicon)
```

The compose files in geecs-epics name `ioc-motion:local`; the pin is this
repository's commit, noted there.

## Serial lines (`geecsSerial.Port`)

Every serial device takes its line as an object, a `geecsSerial.Port`, and
never creates a port itself (geecs-epics decision 15), so the same device
config runs on a host's tty, a USB-serial adapter or a terminal server:

The carrier is read from `address`, in asyn's own syntax, so a sim twin
swaps one string:

| `address` | Carrier | Line settings | Rendered |
|---|---|---|---|
| `/dev/serial/by-id/...` | a local tty | this IOC | `drvAsynSerialPortConfigure` + `asynSetOption` |
| `host:port COM` | a terminal server's telnet socket with RFC 2217 (a Digi's 2001 + n−1 for port n) | this IOC, on every connect (asyn's `asynInterposeCom` restores baud, bits, parity, stop, crtscts and ixon after a reconnect) | `drvAsynIPPortConfigure` + `asynSetOption` |
| `host:port` | a raw socket (a Digi's 2101 + n−1) | the terminal server's own port configuration | `drvAsynIPPortConfigure` only |

RFC 2217 (`COM`) is the default for a device on a Digi: the IOC carries the line
settings, so a reset or a re-profiled Digi port cannot leave the device at the
wrong baud (the first SMC100, 2026-10-28, was silent on the raw socket and
answered on RFC 2217 at its manual's 57600). Upstream's `asyn.AsynIP` is not
used for serial lines: it takes baud, parity, stop and crtscts and renders
none of them, and neither it nor `asyn.AsynSerial` has `ixon`.

The file has no source and no install recipe; the Dockerfile links it into
`/epics/ibek-defs` as ansible does a module's. It moves to a shared repo when a
second image needs it (decision 11).

## ESP300 family (ESP300 / ESP301 / ESP302)

`drvESP300` + `devESP300` is a "model 2" motor driver: `ESP300Setup` once,
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
| `SMC100StatusUpstream` | fixed-position `TP`/`TS` parsing as above | parse after `TP`/`TS` (addresses 1-31); moving = HOMING (1E, 1F), MOVING (28), JOGGING (46); limits from error bits 0 and 1; Homed in every state but NOT REFERENCED (0A-11), CONFIGURATION (14) and HOMING; problem in NOT REFERENCED, CONFIGURATION, DISABLE (3C-3E) or on any other error bit (manual, `TS`) |

`SMC100Controller`'s `setter_resend` and `status_upstream` render them and
default to false. They are global to the IOC (every SMC100 line in it).

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

`aerotech-ensemble-read-only-init.patch` (applied by
`motorAerotech.install.yml` on upstream R1-1-1; `drvEnsembleAsyn.cc`,
`devAerotech.dbd`, `Db/Makefile`, and the new `Db/EnsembleMotor.db`) adds one
iocsh variable, upstream behaviour by default:

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

## Template Info
This repository was generated by
[ioc-template](https://github.com/epics-containers/ioc-template) 4.8.0
(`copier update -a --trust .` to update). The devcontainer and `./build`
work as in every epics-containers generic IOC; the IOC binary links
`$(SUPPORT)/motorNewport` through `ibek`.

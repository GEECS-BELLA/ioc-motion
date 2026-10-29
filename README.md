# Generic IOC ioc-motion

Motion controllers on the EPICS motor record, for GEECS (geecs-epics decision
14: stages are motor records; decision 11: one image per family). Built on
epics-containers' `ioc-motorsim` developer image, so it carries everything that
image does (motor R7-4, asyn, autosave, busy, calc, sscan, sequencer, iocStats,
pvlogging, **motorSim**) plus:

| Module | Version | Source | Reuse (decision 10) |
|---|---|---|---|
| motorNewport | R1-3 | [epics-motor/motorNewport](https://github.com/epics-motor/motorNewport) (ships no LICENSE file; tier 1 rests on the epics-motor org, geecs-epics device catalogue) | tier 1, dependency, with one build-time patch (below) |
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

`esp300-read-only-init.patch` (applied by `motorNewport.install.yml` on
upstream R1-3; two source files and `devNewport.dbd`, whose `variable()`
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
| `on_for_move` | if `AXISSTATUS` reads disabled: `ENABLE @n`, which must be acknowledged (`%`), then `onDelay`, then `AXISSTATUS` must read enabled; otherwise no motion command, PROBLEM set. Never `FAULTACK` | done (DMOV) held at 0; `offDelay` after the end of motion (the poll that first sees it, moving poll period), `DISABLE @n`, acknowledged and read back disabled, and only then done with CNEN 0 | as upstream; a CNEN put ends a pending disable (the put decides) |
| `keep_on` | the same enable | nothing | **CNEN 0 refused**: nothing sent, PROBLEM set, CNEN reads back the drive |

Then the mode commands (`RAMP MODE`, `WAIT MODE NOWAIT`, both acknowledged or
no motion, with `init_writes` false) and the motion command, as before. So an
`on_for_move` move on the wire is `ENABLE`, `onDelay`, `RAMP MODE @n RATE`,
`WAIT MODE NOWAIT`, `ABS`, `RAMP RATE`, `LINEAR`, end of motion, `offDelay`,
`DISABLE`, and DMOV goes 1 after the `DISABLE` is acknowledged: a scan waiting
on DMOV never fires with the axis enabled. Details:

- **Boot, idle poll, IOC exit**: unchanged, queries only, under every policy.
  An axis found enabled at boot stays so until its first `on_for_move` move
  ends. Nothing runs at exit: an IOC killed mid-move, or inside the off window,
  leaves the axis enabled (the controller finishes the `LINEAR` on its own).
  Safe-off on an abnormal exit is the harness's (geecs-epics
  `services/enstest1/phase_b.sh`: STOP, CNEN 0, verified) or the controller's.
- **STOP/abort**: `ABORT @n` as upstream; its end of motion starts `offDelay`,
  then `DISABLE`. A STOP put during `onDelay` waits (the port thread sleeps)
  and aborts the move just sent.
- **A refused move**: by the record's soft limits, nothing reaches the driver
  (no enable); by the controller (a `LINEAR` answered `#`, failed mode
  commands): the axis was enabled for it, so it is disabled `offDelay` later,
  the same path.
- **A failed `DISABLE`** (not acknowledged or still reading enabled): retried on
  the next two polls, then done with PROBLEM and a console line; the axis may
  be enabled.
- **PROBLEM** from a refusal or failure is sticky until the next accepted
  motion or CNEN put (the poll's own fault check would otherwise clear it).
- **Done is plane-wide** (`PLANESTATUS(0)`, upstream): another axis moving in
  plane 0 delays this axis's `offDelay` and its DMOV.
- **New motion inside the off window**: the record still reads not done, so a
  new VAL is a retarget (the record stops, waits for done, then moves): the axis
  is disabled in between. `on_for_move` wants `BDST` 0 and `RTRY` 0 (the
  entity's defaults): a backlash or retry move power-cycles the axis.
- **Limits checked at boot** (`EnsembleAxisPower` prints and sets the iocsh
  error, so `on error halt` stops the IOC): policy one of the three;
  `onDelay` 0..1 s (it holds the axis mutex and blocks that axis's poll);
  `on_for_move` needs `offDelay` ≥ the moving poll period (a shorter one can
  take a not-yet-started motion for its end).
- **Locking**: for a non-`leave` axis the axis mutex is held from the enable
  check to the motion command, so the poller's `DISABLE` cannot fall between
  them; `leave` takes upstream's path unchanged.

Proved on geecs-epics' Ensemble fake (`services/enstest1/power_proofs.sh`, its
record in `services/enstest1/README.md`); **the hardware check is owed**, the
maintainer's to schedule (an `on_for_move` move of the separation axis with
LabVIEW stopped). `dbior motorEnsemble 1` prints each axis's policy and state.

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

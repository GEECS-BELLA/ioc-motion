# ioc-motion

The motion-controller image of GEECS-BELLA's native EPICS IOCs: the motor
record on epics-containers' `ioc-motorsim` developer image (motor R7-4,
asyn, autosave, motorSim, ...) plus `epics-motor/motorNewport` R1-3 (ESP300
family, SMC100, Agilis AG-UC8), `epics-motor/motorAerotech` R1-1-1 (the Ensemble driver),
`epics-motor/motorPIGCS2` R1-3-1 (PI's old "Hexapod" controller),
`epics-motor/motorNewFocus` at master 0f86a5b (the NewFocus 8742 Picomotor),
our `motorVelmex` (the Velmex VXM + VRO driver, README "Velmex VXM + VRO"),
and `geecsSerial.Port`, the one serial-line entity every serial device
takes (local tty, raw TCP, RFC 2217: geecs-epics decision 15).
Details, the patch and the driver's gaps: `README.md`.

- **Instances** live in GEECS-BELLA/geecs-epics, not here: `esptest1`
  (ESP302 `U_ESP302_01`, profile `bench-esp`)
  and its fake twin `esptest1-sim` (`hosts/sim`, profile `motion`);
  `enstest1` (Aerotech Ensemble `U_CompAerotech`, the compressor gratings,
  read-only, profile `bench-ens`); `smctest1` (SMC100PP `U_ProbeCamStage`
  on a Digi port over RFC 2217, profile `bench-smc`) and its fake twin
  `smctest1-sim`;
  `vxmtest1` (Velmex VXM + VRO `U_Velmex`, the 4 m undulator stage, two
  Digi ports over RFC 2217, profile `bench-vxm`) and its fake twin
  `vxmtest1-sim`; `agtest1` (Agilis AG-UC8 `U_ChicaneMotors`, USB serial on
  htu-linux-camserv1, profile `bench-ag`); `zabertest1` (Zaber ASCII daisy
  chain `U_Zaber`, stages on a Digi port over RFC 2217, profile
  `bench-zaber`) and its fake twin `zabertest1-sim`, on this repo's
  `feat/zaber-ascii` until a tag; `pmtest1` (NewFocus 8742 x2
  `U_BlueDiodePicos`, USB through geecs-epics' shim on htu-linux-camserv1,
  profile `bench-pm`) and its fake twin `pmtest1-sim`.
- **Why ours.** No epics-containers image carries motorNewport (TAU
  Systems' `ioc-motornewport` is tier 2, reference only: geecs-epics
  decision 10). The other motor families geecs-epics' device catalogue
  assigns to `ioc-motion` join with their first device (decision 11).
- **Patch and switch.** `ibek-support-geecs/motorNewport/motorNewport-geecs.patch`:
  iocsh variables `ESP300InitStop` and `ESP300SetLimits`, C default 1
  (upstream: a stop per axis to count them, DHLM/DLLM pushed into the
  controller); the `ESP300Setup` entity's `init_stop` / `set_limits` default
  to false, so a boot sends queries only (with no autosave of the position
  fields: README). The `variable()` lines in the dbd
  are what make `var` work: without them iocsh prints "No known var" and the
  stock behaviour runs. The same patch: `SMC100SetterResend` and
  `SMC100StatusUpstream`, C default 1 (upstream: `setPosition` and
  `setClosedLoop` resend the driver's last command, which can be a move;
  `TS` read at fixed positions), entity defaults false, plus
  `SMC100Motor.db` (README "SMC100"). And `AG_UCInitWrites` and
  `AG_UCChannelUpstream`, C default 1 (upstream: `RS` and `SU` per axis at
  boot; `CC` unchecked, so while one pair moves another pair's poll and move
  go to it; `TP`, a per-slot counter shared by the pairs, read as each axis's
  position), and `AG_UCJogAllowed`, C default 1 (the record's jog sends
  `JA`), entity defaults false (README "Agilis AG-UC8").
  `ibek-support-geecs/motorAerotech/aerotech-ensemble.patch`:
  iocsh variable `EnsembleInitWrites`, C default 1 (upstream: a `NONE` probe,
  `RAMP MODE` and `WAIT MODE NOWAIT` at config); the `EnsembleSetup`
  entity's `init_writes` defaults to false. And the per-axis power policy,
  iocsh `EnsembleAxisPower(card, axis, policy, onDelay, offDelay)`, rendered
  by the `EnsembleAxis` entity's `power_policy` (`leave` default: no call,
  upstream; `on_for_move`; `keep_on`), `power_on_delay`, `power_off_delay`
  (README "Aerotech Ensemble", "Power policy"; proven on the controller 2026-10-29).
  The `EnsembleAxis` entity renders `UEIP` Yes (RBV from the encoder) and `RDBD`.
  `ibek-support-geecs/motorNewFocus/motorNewFocus-geecs.patch`: iocsh
  variables `nf874xSetLimits`, `nf874xHomeAllowed`, `nf874xJogAllowed` (C
  default 1, upstream: the record's DHLM/DLLM sent as the 8743-CL's SR/SL at
  every init; HOMF/HOMR send MT/MZ; JOGF/JOGR send an unbounded MV) and
  `nf874xCheckMoves` (C default 0, upstream: the error queue is never read, so
  a move the 8742 ignores ends silently); the `NF874xController` entity's
  `set_limits` / `home_allowed` / `jog_allowed` default to false and
  `check_moves` to true; plus the `VE?` read fix (an empty reply), the 8742
  axis members (undefined behaviour) and `NF874xMotor.db` (RTRY 1; README
  "NewFocus 8742 Picomotor").
- **Images.** CI publishes on a git tag, CalVer `YYYY.M.N` (`2026.10.1`;
  never re-push a tag): `ghcr.io/geecs-bella/ioc-motion-runtime:<tag>` (what hosts
  pull, no credentials) and `-developer` (the release job runs in it; a build
  base); the organisation's packages inherit the repository's visibility. Hosts
  never build. Bench work from a branch: a pre-release tag (`2026.10.2b1`). A
  local build for development:
  `DOCKER_DEFAULT_PLATFORM=linux/amd64 TARGET=runtime TAG=ioc-motion:local ./build`
  (the variable is required on Apple silicon; the image is amd64, emulated).
- **Pins.** geecs-epics' compose files pin the tag. A bump: tag here, change
  the tag in compose in one geecs-epics PR, `docker compose pull <service>`
  and recreate on the host (`docker restart` keeps the old image), one IOC
  on the bench first. Rollback: revert that compose change and recreate.
- **Merging**: after a fresh-context review dispositioned on the PR, and the
  bench proof for anything that changes what reaches a device. Rules and
  program state: geecs-epics `CLAUDE.md`.

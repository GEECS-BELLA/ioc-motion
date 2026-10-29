# ioc-motion

The motion-controller image of GEECS-BELLA's native EPICS IOCs: the motor
record on epics-containers' `ioc-motorsim` developer image (motor R7-4,
asyn, autosave, motorSim, ...) plus `epics-motor/motorNewport` R1-3 (ESP300
family, SMC100) and `epics-motor/motorAerotech` R1-1-1 (the Ensemble driver),
our `motorVelmex` (the Velmex VXM + VRO driver, README "Velmex VXM + VRO"),
and `geecsSerial.Port`, the one serial-line entity every serial device
takes (local tty, raw TCP, RFC 2217: geecs-epics decision 15).
Details, the patch and the driver's gaps: `README.md`.

- **Instances** live in GEECS-BELLA/geecs-epics, not here: `esptest1`
  (ESP302 `U_ESP302_01`, `hosts/htu-linux-camserv2`, profile `bench-esp`)
  and its fake twin `esptest1-sim` (`hosts/sim`, profile `motion`);
  `enstest1` (Aerotech Ensemble `U_CompAerotech`, the compressor gratings,
  read-only, profile `bench-ens`); `smctest1` (SMC100PP `U_ProbeCamStage`
  on a Digi port over RFC 2217, profile `bench-smc`) and its fake twin
  `smctest1-sim`;
  `vxmtest1` (Velmex VXM + VRO `U_Velmex`, the 4 m undulator stage, two
  Digi ports over RFC 2217, profile `bench-vxm`) and its fake twin
  `vxmtest1-sim`.
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
  `SMC100Motor.db` (README "SMC100").
  `ibek-support-geecs/motorAerotech/aerotech-ensemble.patch`:
  iocsh variable `EnsembleInitWrites`, C default 1 (upstream: a `NONE` probe,
  `RAMP MODE` and `WAIT MODE NOWAIT` at config); the `EnsembleSetup`
  entity's `init_writes` defaults to false. And the per-axis power policy,
  iocsh `EnsembleAxisPower(card, axis, policy, onDelay, offDelay)`, rendered
  by the `EnsembleAxis` entity's `power_policy` (`leave` default: no call,
  upstream; `on_for_move`; `keep_on`), `power_on_delay`, `power_off_delay`
  (README "Aerotech Ensemble", "Power policy"; on the controller 2026-10-29, the rerun owed).
  The `EnsembleAxis` entity renders `UEIP` Yes (RBV from the encoder) and `RDBD`.
- **Build.** On the IOC host: `docker build --target runtime -t ioc-motion:local ~/ioc-motion`.
  On a workstation: `DOCKER_DEFAULT_PLATFORM=linux/amd64 TARGET=runtime
  TAG=ioc-motion:local ./build` (the variable is required on Apple silicon; the
  image is amd64 and runs emulated).
- **Pins.** geecs-epics' compose file notes the commit the host image was
  built from. A bump: rebuild on the host (never during a run), update that
  note in the same change, recreate the container (`docker restart` keeps
  the old image), one IOC on the bench first.
- **Merging**: after a fresh-context review dispositioned on the PR, and the
  bench proof for anything that changes what reaches a device. Rules and
  program state: geecs-epics `CLAUDE.md`.

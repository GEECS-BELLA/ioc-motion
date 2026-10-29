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
  (ESP302 `U_ESP302_01`, profile `bench-esp`)
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
  (README "Aerotech Ensemble", "Power policy"; proven on the controller 2026-10-29).
  The `EnsembleAxis` entity renders `UEIP` Yes (RBV from the encoder) and `RDBD`.
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

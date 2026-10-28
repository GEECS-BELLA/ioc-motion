# ioc-motion

The motion-controller image of GEECS-BELLA's native EPICS IOCs: the motor
record on epics-containers' `ioc-motorsim` developer image (motor R7-4,
asyn, autosave, motorSim, ...) plus `epics-motor/motorNewport` R1-3 and
`epics-motor/motorAerotech` R1-1-1 (the Ensemble driver).
Details, the patch and the driver's gaps: `README.md`.

- **Instances** live in GEECS-BELLA/geecs-epics, not here: `esptest1`
  (ESP302 `U_ESP302_01`, `hosts/htu-linux-camserv2`, profile `bench-esp`)
  and its fake twin `esptest1-sim` (`hosts/sim`, profile `motion`);
  `enstest1` (Aerotech Ensemble `U_CompAerotech`, the compressor gratings,
  read-only, profile `bench-ens`, image tag `ioc-motion:aerotech` until this
  driver's PR merges).
- **Why ours.** No epics-containers image carries motorNewport (TAU
  Systems' `ioc-motornewport` is tier 2, reference only: geecs-epics
  decision 10). The other motor families geecs-epics' device catalogue
  assigns to `ioc-motion` join with their first device (decision 11).
- **Patch and switch.** `ibek-support-geecs/motorNewport/esp300-read-only-init.patch`:
  iocsh variables `ESP300InitStop` and `ESP300SetLimits`, C default 1
  (upstream: a stop per axis to count them, DHLM/DLLM pushed into the
  controller); the `ESP300Setup` entity's `init_stop` / `set_limits` default
  to false, so a boot sends queries only (with no autosave of the position
  fields: README). The `variable()` lines in the dbd
  are what make `var` work: without them iocsh prints "No known var" and the
  stock behaviour runs.
  `ibek-support-geecs/motorAerotech/aerotech-ensemble-read-only-init.patch`:
  iocsh variable `EnsembleInitWrites`, C default 1 (upstream: a `NONE` probe,
  `RAMP MODE` and `WAIT MODE NOWAIT` at config); the `EnsembleSetup`
  entity's `init_writes` defaults to false (a query probe, the mode commands
  deferred to the first motion command), and `EnsembleMotor.db` defaults
  `RSTM` to Never (README "Aerotech Ensemble").
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

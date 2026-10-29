ARG IMAGE_EXT

ARG REGISTRY=ghcr.io/epics-containers
# Built on the simulated-motor developer image (ioc-motorsim R1-3ec2): EPICS
# base, asyn, motor R7-4, autosave, busy, calc, sscan, sequencer, iocStats,
# pvlogging and motorSim already built. ibek-support is pinned to the commit
# that image was built with (8545fa2). motorNewport, motorAerotech and
# motorPIGCS2 are added.
ARG RUNTIME=${REGISTRY}/epics-base${IMAGE_EXT}-runtime:7.0.10ec2
ARG DEVELOPER=${REGISTRY}/ioc-motorsim${IMAGE_EXT}-developer:R1-3ec2

##### build stage ##############################################################
FROM  ${DEVELOPER} AS developer

# The devcontainer mounts the project root to /epics/generic-source
# Using the same location here makes devcontainer/runtime differences transparent.
# (The template's `ln -s ${SOURCE_FOLDER}/ioc ${IOC}` is omitted: the motorsim
# developer image already made that link, and a second ln would nest a self-link.)
ENV SOURCE_FOLDER=/epics/generic-source

# get the current versions of pvi and ibek
COPY requirements.txt requirements.txt
RUN uv pip install --upgrade -r requirements.txt

WORKDIR ${SOURCE_FOLDER}/ibek-support

COPY ibek-support/_ansible _ansible
ENV PATH=$PATH:${SOURCE_FOLDER}/ibek-support/_ansible

# motorNewport R1-3: upstream ibek-support's recipe with ioc-motion's patch
# (ESP300 read-only init, SMC100 fixes) and our SMC100 and ESP300 entities
# (README "SMC100", "ESP300 family").
# Only this copy is in the image, so ansible finds the module once.
COPY ibek-support-geecs/motorNewport/ ${SOURCE_FOLDER}/ibek-support-geecs/motorNewport/
RUN ansible.sh motorNewport

# geecsSerial: one serial line as an asyn port, local tty / raw TCP / RFC 2217
# (README "Serial lines"). An entity file only, no source: linked into
# ibek-defs the way ansible links a module's.
COPY ibek-support-geecs/geecsSerial/ ${SOURCE_FOLDER}/ibek-support-geecs/geecsSerial/
RUN mkdir -p /epics/ibek-defs && ln -s ${SOURCE_FOLDER}/ibek-support-geecs/geecsSerial/geecsSerial.ibek.support.yaml /epics/ibek-defs/

# motorAerotech R1-1-1: our recipe (none upstream) with the Ensemble
# patch (read-only init, per-axis power policy) and the Ensemble entities (README "Aerotech Ensemble").
COPY ibek-support-geecs/motorAerotech/ ${SOURCE_FOLDER}/ibek-support-geecs/motorAerotech/
RUN ansible.sh motorAerotech

# motorPIGCS2 R1-3-1: our recipe (none upstream) with the PI hexapod patch
# (no SVO at init, INI refused, VMO before each MOV: switches) and
# the hexapod entities (README "PI hexapod (motorPIGCS2)").
COPY ibek-support-geecs/motorPIGCS2/ ${SOURCE_FOLDER}/ibek-support-geecs/motorPIGCS2/
RUN ansible.sh motorPIGCS2

# motorVelmex: our driver for the Velmex VXM stepping-motor controller with the
# VRO encoder readout on a second line (README "Velmex VXM + VRO"). No upstream
# module exists; the source is in this repository (motorVelmex/), copied into
# place before ansible.sh so the recipe builds it instead of cloning.
COPY ibek-support-geecs/motorVelmex/ ${SOURCE_FOLDER}/ibek-support-geecs/motorVelmex/
COPY motorVelmex/ /epics/support/motorVelmex/
RUN ansible.sh motorVelmex

# motorZaber: our driver for Zaber devices on the Zaber ASCII protocol
# (A-series and X-series stages, X-MCC controllers, daisy chains; README
# "Zaber (ASCII)"). Zaber's own motorZaberMotion ships no licence and needs
# the Zaber Motion Library binaries, so it is a reference only (geecs-epics
# decision 10); the source is in this repository (motorZaber/).
COPY ibek-support-geecs/motorZaber/ ${SOURCE_FOLDER}/ibek-support-geecs/motorZaber/
COPY motorZaber/ /epics/support/motorZaber/
RUN ansible.sh motorZaber

# get the ioc source and build it
COPY ioc ${SOURCE_FOLDER}/ioc
RUN ansible.sh ioc

# generate a manifest of installed EPICS modules and python packages
# IOC_VERSION is declared here, not earlier: every RUN after an ARG sees it,
# so a new value (each branch or tag) would rebuild all the steps above
ARG IOC_VERSION=unknown
COPY scripts/generate_manifest.py /tmp/generate_manifest.py
RUN python3 /tmp/generate_manifest.py "${IOC_VERSION}"

##### runtime preparation stage ################################################
FROM developer AS runtime_prep

# get the products from the build stage and reduce to runtime assets only
# /python is created by uv and is needed in the runtime target
# /epics/versions.json is the manifest of support module and python versions
RUN ibek ioc extract-runtime-assets /assets /python /epics/versions.json

##### runtime stage ############################################################
FROM ${RUNTIME} AS runtime

# get runtime assets from the preparation stage
COPY --from=runtime_prep /assets /

# install runtime system dependencies, collected from install.sh scripts
RUN ibek support apt-install-runtime-packages

# launch the startup script with stdio-expose to allow console connections
CMD ["bash", "-c", "${IOC}/start.sh"]

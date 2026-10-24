ARG IMAGE_EXT

ARG REGISTRY=ghcr.io/epics-containers
# Built on the simulated-motor developer image (ioc-motorsim R1-3ec2): EPICS
# base, asyn, motor R7-4, autosave, busy, calc, sscan, sequencer, iocStats,
# pvlogging and motorSim already built. ibek-support is pinned to the commit
# that image was built with (8545fa2). motorNewport is the one module added.
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

# motorNewport R1-3: upstream ibek-support's recipe with the ESP300
# read-only-init patch and the ESP300 entities (README "ESP300 family").
# Only this copy is in the image, so ansible finds the module once.
COPY ibek-support-geecs/motorNewport/ ${SOURCE_FOLDER}/ibek-support-geecs/motorNewport/
RUN ansible.sh motorNewport

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

#  Copyright (C) 2018-2021 LEIDOS.
# 
#  Licensed under the Apache License, Version 2.0 (the "License"); you may not
#  use this file except in compliance with the License. You may obtain a copy of
#  the License at
# 
#  http://www.apache.org/licenses/LICENSE-2.0
# 
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
#  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
#  License for the specific language governing permissions and limitations under
#  the License.

FROM usdotfhwastol/carma-platform:c1tenth-develop

# Make sure 'carma' user has the correct permissions to open devices.
RUN sudo usermod -aG dialout,plugdev carma

ARG ROS1_PACKAGES=""
ENV ROS1_PACKAGES=${ROS1_PACKAGES}
ARG ROS2_PACKAGES=""
ENV ROS2_PACKAGES=${ROS2_PACKAGES}

RUN mkdir -p /home/carma/src
COPY --chown=carma . /home/carma/src/c1tenth-driver-wrappers
RUN /home/carma/src/c1tenth-driver-wrappers/docker/checkout.bash
RUN /home/carma/src/c1tenth-driver-wrappers/docker/install.sh

ARG BUILD_DATE="NULL"
ARG VERSION="NULL"
ARG VCS_REF="NULL"

LABEL org.label-schema.schema-version="1.0"
LABEL org.label-schema.name="carma-cohda-dsrc-driver"
LABEL org.label-schema.description="Drivers for the c1tenth platform"
LABEL org.label-schema.vendor="Leidos"
LABEL org.label-schema.version=${VERSION}
LABEL org.label-schema.url="https://highways.dot.gov/research/research-programs/operations/CARMA"
LABEL org.label-schema.vcs-url="https://github.com/KBR-CARMA/c1tenth-driver-wrappers"
LABEL org.label-schema.vcs-ref=${VCS_REF}
LABEL org.label-schema.build-date=${BUILD_DATE}

# Copy the build products to a specific directory in /opt because the entrypoint inherited
# from carma-base through carma-platform sources this as a canned location.
RUN cp -r /home/carma/install /opt/carma/install

# Clean up any intermediary build directories
RUN rm -rf /home/carma/src /home/carma/build /home/carma/log
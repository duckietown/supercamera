ARG ARCH=arm32v7

FROM duckietown/ros-commons:master19-${ARCH}

# configure environment
ENV LAUNCH_FILE="${SOURCE_DIR}/launch.sh"

# copy source code
COPY ./catkin_ws "${SOURCE_DIR}/catkin_ws"
COPY ./assets/launch.sh "${LAUNCH_FILE}"

RUN ["cross-build-start"]

# build packages
RUN /entrypoint.sh \
  catkin_make \
    -j \
    -C "${SOURCE_DIR}/catkin_ws/"

RUN ["cross-build-end"]

# configure CMD
CMD ["bash", "-c", "${LAUNCH_FILE}"]

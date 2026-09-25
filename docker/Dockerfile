# AeroSLAM development / CI image (ROS 1 Noetic, Ubuntu 20.04, Gazebo 11)
FROM osrf/ros:noetic-desktop-full

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake git curl wget \
        python3-pip python3-dev python3-setuptools python3-yaml \
        libeigen3-dev libboost-all-dev libyaml-cpp-dev \
        libgoogle-glog-dev \
        libpcl-dev libopencv-dev \
        ros-noetic-navigation ros-noetic-cmake-modules ros-noetic-joy \
        ros-noetic-osqp ros-noetic-osqp-vendor \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /root/aeroslam
COPY . .

RUN /bin/bash -c "\
    source /opt/ros/noetic/setup.bash && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    catkin_make -DCMAKE_BUILD_TYPE=Release"

# Optional: fetch frontier modules at image build time
# RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
#     vcs import src/frontier < thirdparty/aeroslam.repos"

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

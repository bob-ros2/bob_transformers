ARG ROS_DISTRO=humble
FROM osrf/ros:$ROS_DISTRO-desktop

RUN apt-get update && apt-get install -y \
    vim \
    curl \
    wget \
    git \
    netcat \
    socat \
    iproute2 \
    iputils-ping \
    python3-pip \
    python3-requests \
    && apt-get clean

#RUN pip3 install \
#  transformers>=4.48.3 \
#  protobuf>=4.25.5 \
#  sentencepiece>=0.2.0 \
#  Jinja2>=3.1.3 \
#  diffusers>=0.29.2 \
#  torch>=2.2.2 \
#  accelerate>=0.32.1

ARG WORKSPACE=/ros_ws
ENV WORKSPACE=$WORKSPACE
RUN mkdir $WORKSPACE
WORKDIR $WORKSPACE

# setup workspace
RUN mkdir -p $WORKSPACE/src \
  && cd $WORKSPACE/src \
  && git clone https://github.com/bob-ros2/bob_launch.git \
  && git clone https://github.com/bob-ros2/bob_msgs.git \
  && git clone https://github.com/bob-ros2/bob_transformers.git \
  && pip3 install -r $WORKSPACE/src/bob_transformers/requirements.txt

# build package(s)
RUN . /opt/ros/$ROS_DISTRO/setup.sh \
  && cd $WORKSPACE && colcon build


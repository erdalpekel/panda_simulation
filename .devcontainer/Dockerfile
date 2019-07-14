FROM osrf/ros:melodic-desktop-full-bionic

RUN apt-get update && apt-get install -q -y \
    openssh-client

RUN echo 'source /opt/ros/melodic/setup.bash' >> /root/.bashrc
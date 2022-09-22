FROM osrf/ros:melodic-desktop-full-bionic

RUN apt-get update && apt-get install -q -y \
    openssh-client

WORKDIR /root/apps/
RUN git clone --branch v9.0.0 https://gitlab.kitware.com/vtk/vtk.git
RUN git clone --depth 1 --branch OpenMesh-8.1 https://www.graphics.rwth-aachen.de:9000/OpenMesh/OpenMesh.git

RUN mkdir /root/apps/vtk/build
WORKDIR /root/apps/vtk/build
RUN cmake ..
RUN make -j$(nproc)
RUN make install

RUN mkdir /root/apps/OpenMesh/build
WORKDIR /root/apps/OpenMesh/build
RUN cmake .. -DBUILD_APPS=OFF
RUN make -j$(nproc)
RUN make install

RUN echo 'source /opt/ros/noetic/setup.bash' >> /root/.bashrc
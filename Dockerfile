# Use Ubuntu 22.04 as the base image
FROM ubuntu:22.04

# Set environment variables to avoid interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Update the package list and install necessary dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    libboost-all-dev \
    libeigen3-dev \
    libflann-dev \
    libvtk7-dev \
    libqhull-dev \
    libusb-1.0-0-dev \
    libopenni2-dev \
    libpcap-dev \
    wget \
    xvfb \
    software-properties-common \
    python3 \
    python3-pip \
    python3-dev \
    libpython3-dev \
    && rm -rf /var/lib/apt/lists/*

#install python packages
RUN pip3 install --upgrade pip
RUN pip3 install numpy matplotlib 

# Download and install PCL 1.14
RUN wget https://github.com/PointCloudLibrary/pcl/archive/refs/tags/pcl-1.14.0.tar.gz && \
    tar -xvzf pcl-1.14.0.tar.gz && \
    cd pcl-pcl-1.14.0 && \
    mkdir build && cd build && \
    cmake .. && \
    make -j$(nproc) && \
    make install && \
    ldconfig && \
    cd ../.. && rm -rf pcl-pcl-1.14.0 pcl-1.14.0.tar.gz

# Copy the Referee folder into the image
COPY . /home/Referee

# Configure, build, and run Referee
WORKDIR /home/Referee/src
RUN mkdir build && cd build && \
    cmake .. && \
    make -j$(nproc)

COPY Docker_commands.sh /home/Referee/src/build/Docker_commands.sh
WORKDIR /home/Referee/src/build

ENV DISPLAY=:99

RUN chmod +x ./Docker_commands.sh

CMD ["./Docker_commands.sh"]

FROM ubuntu:20.04

RUN apt update -y && apt install -y cmake wget vim

# Install matplotlib-cpp
RUN wget -c http://archive.ubuntu.com/ubuntu/pool/universe/p/python-cycler/python-cycler_0.10.0-1_all.deb
RUN wget -c http://archive.ubuntu.com/ubuntu/pool/universe/m/matplotlib/python-matplotlib_2.1.1-2ubuntu3_amd64.deb
RUN apt install -y ./python-matplotlib_2.1.1-2ubuntu3_amd64.deb ./python-cycler_0.10.0-1_all.deb

# Install python3
RUN apt update -y && apt install -y build-essential g++ python3-dev python3-pip
RUN pip install numpy

# Install PCL
RUN apt-get update -y --fix-missing && apt-get install -y git
RUN git clone https://github.com/PointCloudLibrary/pcl.git
RUN cd pcl/ && mkdir build && cd build && cmake ../ -DBUILD_CUDA=ON -DBUILD_GPU=ON -DCMAKE_BUILD_TYPE=Release && make -j 1 && make install 

# Install X11 server
RUN apt-get install -y x11-apps
RUN echo "X11Forwarding yes" >> /etc/ssh/sshd_config && echo "X11DisplayOffset 10" >> /etc/ssh/sshd_config && echo "X11UseLocalhost yes" >> /etc/ssh/sshd_config

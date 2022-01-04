#FROM ubuntu:20.04 
FROM osrf/ros:kinetic-desktop-full

# Install fundemental build tools
RUN apt update -y --fix-missing && apt install -y cmake wget vim build-essential g++ git gdb mesa-utils 

# Install ROS which solves all dependencies of PCL
RUN	mkdir -p /home/ros_catkin_ws/src && \
	cd /home/ros_catkin_ws/src && \
	/bin/bash -c "source /opt/ros/kinetic/setup.bash; catkin_init_workspace" && \
	cd /home/ros_catkin_ws && \
	/bin/bash -c "source /opt/ros/kinetic/setup.bash; catkin_make" && \
#	echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc && \
#	echo "source /home/ros_catkin_ws/devel/setup.bash" >> ~/.bashrc && \
	echo "export ROS_PACKAGE_PATH=\${ROS_PACKAGE_PATH}:/home/ros_catkin_ws" >> ~/.bashrc && \
	echo "export ROS_WORKSPACE=/home/ros_catkin_ws" >> ~/.bashrc

# Install PCL
ENV DEBIAN_FRONTEND noninteractive
RUN wget https://github.com/PointCloudLibrary/pcl/archive/pcl-1.7.2.tar.gz
RUN tar zxvf pcl-1.7.2.tar.gz 
RUN cd pcl-pcl-1.7.2 && mkdir build && cd build && cmake -DCMAKE_BUILD_TYPE=Release .. && make -j2 && make -j2 install

# Install python3
RUN apt install -y software-properties-common
RUN add-apt-repository -y ppa:deadsnakes/ppa
RUN apt update -y
RUN apt install -y python3.8 python3.8-distutils
RUN apt update -y && apt install -y python3-dev python3-pip
RUN curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py && python3.8 get-pip.py
RUN pip3 install matplotlib

# Install latest cmake
RUN apt remove -y cmake
RUN wget https://github.com/Kitware/CMake/releases/download/v3.17.1/cmake-3.17.1.tar.gz && \
    tar zxvf cmake-3.17.1.tar.gz && \
    cd cmake-3.17.1/ && \
    ./bootstrap && \
    make -j12 && sudo make install -j8
RUN echo 'export PATH=$HOME/cmake-3.17.1/bin/:$PATH' >> ~/.bashrc && \
    . ~/.bashrc


# Install matplotlib-cpp
#RUN wget -c http://archive.ubuntu.com/ubuntu/pool/universe/p/python-cycler/python-cycler_0.10.0-1_all.deb
#RUN wget -c http://archive.ubuntu.com/ubuntu/pool/universe/m/matplotlib/python-matplotlib_2.1.1-2ubuntu3_amd64.deb
#RUN apt install -y ./python-matplotlib_2.1.1-2ubuntu3_amd64.deb ./python-cycler_0.10.0-1_all.deb
RUN apt-get install python-matplotlib python-numpy python2.7-dev

# Install X11 server
RUN apt-get install -y x11-apps ssh
RUN echo "X11Forwarding yes" >> /etc/ssh/sshd_config && echo "X11DisplayOffset 10" >> /etc/ssh/sshd_config && echo "X11UseLocalhost yes" >> /etc/ssh/sshd_config
CMD /bin/bash
ENTRYPOINT [""]

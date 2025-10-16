FROM ros:noetic-robot

# Install dependencies
RUN apt-get update && apt-get install -y \
    ros-noetic-pcl-ros \
    ros-noetic-pcl-conversions \
    ros-noetic-image-geometry \
    ros-noetic-cv-bridge \
    ros-noetic-dynamic-reconfigure \
    ros-noetic-rviz \
    ros-noetic-rqt-gui \
    ros-noetic-rqt-reconfigure \
    libpcl-dev \
    python3-pip \
    git \
    && rm -rf /var/lib/apt/lists/*

# Set working directory
WORKDIR /root/catkin_ws

# Copy package files
COPY . /root/catkin_ws/src/ugoe_gap_detection_ros/

# Install Python dependencies
RUN pip3 install -r /root/catkin_ws/src/ugoe_gap_detection_ros/requirements.txt

# Initialize catkin workspace
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
    cd /root/catkin_ws && \
    catkin_make"

# Source ROS and workspace
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
    echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc

# Set entrypoint
COPY docker-entrypoint.sh /root/docker-entrypoint.sh
RUN chmod +x /root/docker-entrypoint.sh

ENTRYPOINT ["/root/docker-entrypoint.sh"]
CMD ["bash"]

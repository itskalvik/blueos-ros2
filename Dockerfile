ARG ROS_DISTRO=humble
FROM ros:$ROS_DISTRO-ros-base
WORKDIR /home/

# Install general packages (including mavros and foxglove)
RUN rm /var/lib/dpkg/info/libc-bin.* \
    && apt-get clean \
    && apt-get update \
    && apt-get install libc-bin \
    && apt-get install -q -y --no-install-recommends \
    tmux nano nginx wget netcat \
    ros-${ROS_DISTRO}-mavros ros-${ROS_DISTRO}-mavros-extras ros-${ROS_DISTRO}-mavros-msgs \
    ros-${ROS_DISTRO}-geographic-msgs \
    ros-${ROS_DISTRO}-foxglove-bridge \
    python3-dev python3-pip \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/* \
    && pip3 install --no-cache-dir setuptools pip packaging -U

# Install gscam2 deps
RUN apt-get update \
    && apt-get install -q -y --no-install-recommends \
    libgstreamer1.0-0 gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-tools \
    gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 gstreamer1.0-pulseaudio \
    libgstreamer-plugins-base1.0-dev \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/* \
    && pip3 install --no-cache-dir setuptools==79.0.1 pip packaging -U

# Install ping-python from source (Newer commits don't seem to work with the ping1d)
RUN cd /home/ \
    && git clone https://github.com/bluerobotics/ping-python.git -b deployment \
    && cd ping-python \
    && git checkout 3d41ddd \
    && python3 setup.py install --user

# Build ROS2 workspace with remaining packages
COPY ros2_ws /home/ros2_ws
RUN cd /home/ros2_ws/ \
    && if [ -f src/mavros_control/requirements.txt ] && [ -s src/mavros_control/requirements.txt ]; then \
        python3 -m pip install --no-cache-dir -r src/mavros_control/requirements.txt; \
    fi \
    && git clone https://github.com/ptrmu/ros2_shared.git --depth 1 src/ros2_shared \
    && apt-get update \
    && rosdep install --from-paths src --ignore-src -r -y \
    && . "/opt/ros/${ROS_DISTRO}/setup.sh" \
    && colcon build --symlink-install \
    && ros2 run mavros install_geographiclib_datasets.sh \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/* \
    && echo "source /ros_entrypoint.sh" >> ~/.bashrc

# Setup ttyd for web terminal interface
ADD files/install-ttyd.sh /install-ttyd.sh
RUN bash /install-ttyd.sh && rm /install-ttyd.sh
COPY files/tmux.conf /etc/tmux.conf

# Copy configuration files
COPY files/nginx.conf /etc/nginx/nginx.conf
COPY files/index.html /usr/share/ttyd/index.html
COPY files/start-nginx.sh /start-nginx.sh
RUN chmod +x /start-nginx.sh

# Copy start script and other files
RUN mkdir -p /site
COPY files/register_service /site/register_service
COPY files/start.sh /start.sh

# Add docker configuration
LABEL version="0.0.3"
LABEL permissions='{\
  "NetworkMode": "host",\
  "HostConfig": {\
    "Binds": [\
      "/dev:/dev:rw",\
      "/usr/blueos/extensions/ros2/:/home/persistent_ws/:rw"\
    ],\
    "Privileged": true,\
    "NetworkMode": "host"\
  },\
  "Env": [\
  ]\
}'
LABEL authors='[\
  {\
    "name": "Kalvik Jakkala",\
    "email": "itskalvik@gmail.com"\
  }\
]'
LABEL company='{\
  "about": "",\
  "name": "ItsKalvik",\
  "email": "itskalvik@gmail.com"\
}'
LABEL readme="https://raw.githubusercontent.com/itskalvik/blueos-ros2/master/README.md"
LABEL type="other"
LABEL tags='[\
  "ros2",\
  "sonar",\
  "camera",\
  "foxglove",\
  "ardusub",\
  "blueos",\
  "robot"\
]'

# Keep bash alive even if there is an error
RUN echo "set +e" >> ~/.bashrc
ENTRYPOINT [ "/start.sh" ]
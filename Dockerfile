ARG ROS_DISTRO=jazzy
FROM ros:$ROS_DISTRO-ros-base

RUN rm /var/lib/dpkg/info/libc-bin.* \
    && apt-get clean \
    && apt-get update \
    && apt-get -y install libc-bin \
    && apt-get install -q -y --no-install-recommends \
    tmux nano nginx wget
RUN apt-get install -y ros-${ROS_DISTRO}-mavros ros-${ROS_DISTRO}-mavros-extras ros-${ROS_DISTRO}-mavros-msgs
RUN apt-get install -y python3-dev python3-pip \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*

COPY ros2_ws /home/ros2_ws
RUN cd /home/ros2_ws/ \
    && . "/opt/ros/${ROS_DISTRO}/setup.sh" \
    && colcon build --symlink-install \
    && ros2 run mavros install_geographiclib_datasets.sh \
    && echo "source /home/ros2_ws/install/setup.sh " >> ~/.bashrc

# Setup ttyd for web terminal interface
ADD files/install-ttyd.sh /install-ttyd.sh
RUN bash /install-ttyd.sh && rm /install-ttyd.sh
COPY files/tmux.conf /etc/tmux.conf

RUN mkdir -p /site
COPY files/register_service /site/register_service
COPY files/nginx.conf /etc/nginx/nginx.conf

ADD files/start.sh /start.sh

# Add docker configuration
LABEL version="0.0.1"
LABEL permissions='{\
  "NetworkMode": "host",\
  "HostConfig": {\
    "Binds": [\
      "/dev:/dev:rw"\
    ],\
    "Privileged": true,\
    "NetworkMode": "host"\
  }\
}'
LABEL authors='[\
  {\
    "name": "Noah Mollerstuen",\
    "email": "noah@mollerstuen.com"\
  },\
  {\
    "name": "Michael Carlstrom",\
    "email": "rmc@carlstrom.com"\
  }\
]'
LABEL company='{\
  "about": "",\
  "name": "CWRUbotix",\
  "email": "robotics-exec@case.edu"\
}'
LABEL readme="https://github.com/CWRUbotix/rov-blueos-extension/blob/main/README.md"
LABEL type="other"
LABEL tags='[\
  "ros2",\
  "robot"\
]'

ENTRYPOINT [ "/start.sh" ]
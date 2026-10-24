# robbie_ros dev container (ADR 2): Ubuntu 24.04, ROS 2 Jazzy, ros2_control and the MuJoCo sim.
# Built by just setup. The repo is mounted at /ws/robbie_ros and robbie at /ws/robbie (ADR 3).

FROM ros:jazzy
COPY --from=ghcr.io/astral-sh/uv:0.13.5 /uv /uvx /usr/local/bin/

ARG USER_UID=1000
ARG USER_GID=1000
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        # ROS 2 desktop tools, ros2_control, and the sim (MuJoCo comes in via mujoco_vendor)
        ros-jazzy-desktop \
        ros-jazzy-ros2-control \
        ros-jazzy-ros2-controllers \
        ros-jazzy-xacro \
        ros-jazzy-mujoco-ros2-control \
        ros-dev-tools \
        build-essential cmake git curl sudo \
        # robbie_hardware builds here too, so it needs the Pi's hardware headers
        libgpiod-dev libi2c-dev \
        # OpenGL for the MuJoCo viewer (software rendering is fine for this model)
        libgl1-mesa-dri libglx-mesa0 mesa-utils \
        # just setup-flash
        xz-utils fdisk mtools openssl gnupg dirmngr \
    && rm -rf /var/lib/apt/lists/*

# xdotool: just up resizes the MuJoCo window, which otherwise opens at full-monitor size.
RUN apt-get update && apt-get install -y --no-install-recommends xdotool \
    && rm -rf /var/lib/apt/lists/*

# Run as the host user's UID/GID, so files written to the mounted repo aren't owned by root.
# Ubuntu 24.04 images ship an 'ubuntu' user on UID 1000, so it goes first.
RUN (userdel -r ubuntu 2>/dev/null || true) \
    && groupadd -o -g "$USER_GID" dev \
    && useradd -m -o -u "$USER_UID" -g "$USER_GID" -s /bin/bash dev \
    && echo "dev ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/dev \
    # build/, install/ and log/ are named volumes; they copy this ownership when first created
    && mkdir -p /ws/robbie_ros/build /ws/robbie_ros/install /ws/robbie_ros/log /opt/venv \
    && chown -R dev:dev /ws /opt/venv

USER dev

# Python packages go in a venv, installed as dev (not root). It sees the system site-packages,
# so rclpy and colcon are still importable.
COPY --chown=dev:dev scripts/setup/requirements.txt /tmp/requirements.txt
RUN uv venv --system-site-packages /opt/venv \
    && uv pip install --no-cache --python /opt/venv/bin/python -r /tmp/requirements.txt
ENV PATH="/opt/venv/bin:$PATH"

RUN rosdep update \
    && echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc \
    && echo '[ -f /ws/robbie_ros/install/setup.bash ] && source /ws/robbie_ros/install/setup.bash' >> ~/.bashrc
WORKDIR /ws/robbie_ros

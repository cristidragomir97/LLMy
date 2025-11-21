# ─── Component Builder ─────────────────────────────────────────────────────
FROM nx128.lan:5000/robocore/leremix_sim_base:jazzy-latest

ENV PYTHONDONTWRITEBYTECODE=1

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy
SHELL ["/bin/bash","-lc"]
#ENV ROS_WS=/ros_ws

# 1) Install any component-specific system packages

# 1b) Install any component-specific pip packages

# 2) VCS repos → build into /vcs_ws/install


# 3) Copy & prepare any local source
WORKDIR /ros_ws
RUN mkdir -p src

# Copy source packages directly from their original locations
COPY ros/src/leremix_slam /ros_ws/src/leremix_slam

# Verify we have actual files
RUN echo '=== Checking source directory structure ===' \
 && ls -la src/ \
 && echo '=== Found package.xml files ===' \
 && find src -type f -name package.xml -exec echo '{}' \;

# Install dependencies
RUN set -exo pipefail \
 && apt-get update \
 && source /opt/ros/$ROS_DISTRO/setup.bash \
 && source /ros_ws_common/install/setup.bash \
 && rosdep update \
 && echo '=== Checking what rosdep will install ===' \
 && rosdep keys --rosdistro $ROS_DISTRO --from-paths src --ignore-src | sort | uniq || echo 'No rosdep keys found' \
 && echo '=== Installing dependencies ===' \
 && rosdep install --rosdistro $ROS_DISTRO --from-paths src --ignore-src -r -y

# Sanity check
RUN echo '=== Verifying dependencies ===' \
 && rosdep check --rosdistro $ROS_DISTRO --from-paths src --ignore-src || true


# DDS Discovery Server config
COPY .robocore/workspaces/slam/superclient.xml /superclient.xml
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/superclient.xml
ENV ROS_DISCOVERY_SERVER=localhost:11811
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
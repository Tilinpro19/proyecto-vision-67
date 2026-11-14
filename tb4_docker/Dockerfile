# Imagen para hablar con el TurtleBot 4 Lite (ROS 2 Jazzy) desde la PC.
# Trae ROS 2 + mensajes del TB4/Create 3 + OpenCV + MediaPipe + la libreria tb4.
FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive \
    RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    PIP_BREAK_SYSTEM_PACKAGES=1 \
    PYTHONUNBUFFERED=1

RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-rmw-fastrtps-cpp \
      ros-jazzy-irobot-create-msgs \
      ros-jazzy-turtlebot4-msgs \
      ros-jazzy-teleop-twist-keyboard \
      ros-jazzy-rqt-image-view \
      ros-jazzy-image-transport-plugins \
      ros-jazzy-cv-bridge \
      ros-jazzy-tf2-ros-py \
      ros-jazzy-demo-nodes-cpp ros-jazzy-demo-nodes-py \
      python3-opencv python3-numpy python3-pip python3-pytest \
      iputils-ping iproute2 openssh-client net-tools \
      espeak-ng v4l-utils libgl1 \
    && rm -rf /var/lib/apt/lists/*

# MediaPipe (gestos / mano). Se instala sin sus dependencias de OpenCV para no
# pisar el python3-opencv de apt, que es el que usa cv_bridge.
ARG WITH_MEDIAPIPE=1
RUN if [ "$WITH_MEDIAPIPE" = 1 ]; then \
      (pip3 install --no-cache-dir --ignore-installed "numpy<2" mediapipe && \
       pip3 uninstall -y opencv-contrib-python opencv-python; \
       python3 -c "import cv2, mediapipe; print('mediapipe', mediapipe.__version__)") \
      || echo "AVISO: mediapipe NO quedo instalado (gestos no funcionaran)"; \
    fi

COPY tb4lib /opt/tb4/tb4lib
RUN pip3 install --no-cache-dir -e /opt/tb4/tb4lib

COPY entrypoint.sh /opt/tb4/entrypoint.sh
RUN chmod +x /opt/tb4/entrypoint.sh && \
    echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc && \
    echo '[ -f /tmp/tb4_env ] && source /tmp/tb4_env' >> /root/.bashrc

WORKDIR /ws
ENTRYPOINT ["/opt/tb4/entrypoint.sh"]
CMD ["bash"]

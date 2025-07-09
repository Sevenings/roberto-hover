FROM osrf/ros:noetic-desktop-full

# Instalar dependências do sistema
RUN apt-get update && apt-get install -y \
    # Ferramentas básicas
    curl \
    wget \
    vim \
    git \
    python3-pip \
    python3-catkin-tools \
    python3-rosdep \
    # Dependências ROS
    ros-noetic-joy \
    ros-noetic-teleop-twist-joy \
    ros-noetic-teleop-twist-keyboard \
    ros-noetic-tf2-tools \
    ros-noetic-rviz \
    ros-noetic-joint-state-publisher \
    ros-noetic-joint-state-publisher-gui \
    ros-noetic-robot-state-publisher \
    ros-noetic-xacro \
    # Sensores básicos
    ros-noetic-sensor-msgs \
    ros-noetic-geometry-msgs \
    ros-noetic-std-msgs \
    ros-noetic-visualization-msgs \
    # ROSSerial básico
    ros-noetic-rosserial-python \
    # Ferramentas básicas
    ros-noetic-diagnostic-msgs \
    ros-noetic-rqt \
    # Dependências gráficas
    mesa-utils \
    x11-apps \
    ros-noetic-realsense2-camera \
    && rm -rf /var/lib/apt/lists/*

# Instalar dependências Python básicas
RUN pip3 install \
    numpy \
    pyserial \
    pygame \
    pyyaml

# Configurar workspace
RUN mkdir -p /catkin_ws/src
WORKDIR /catkin_ws

# Copiar arquivos do projeto
# COPY src/ /catkin_ws/src/
# COPY launch/ /catkin_ws/src/summer_camp/launch/
# COPY config/ /catkin_ws/src/summer_camp/config/
# COPY worlds/ /catkin_ws/src/summer_camp/worlds/
# COPY urdf/ /catkin_ws/src/summer_camp/urdf/
# COPY scripts/ /catkin_ws/src/summer_camp/scripts/

# Instalar dependências ROS do workspace
RUN rosdep init || true && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y

# Build do workspace
# RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# Configurar environment
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
# RUN echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc

# Variáveis de ambiente gráficas
ENV QT_X11_NO_MITSHM=1
ENV DISPLAY=:0

USER root
WORKDIR /catkin_ws

# Comando padrão
CMD ["/bin/bash"] 

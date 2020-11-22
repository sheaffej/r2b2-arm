FROM ros:noetic-robot-buster

ENV PROJ r2b2-arm
ENV ROS_PKG r2b2_arm

ENV ROS_WS /ros
ENV CODE_MOUNT /workspaces
ENV PYTEST_ADDOPTS "--color=yes"

SHELL [ "bash", "-c"]
WORKDIR /root

# Install pip 
RUN apt-get update \
	&& apt-get install -y \
		libffi-dev \
		libssl-dev \
		python3-pip \
	&& pip3 install --no-cache-dir --upgrade pip \
	&& rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/* 

# Install Python testing packages
RUN pip3 install \
		pytest \
		pytest-cov \
		coveralls \
		flake8

# Install OS packages
RUN apt-get update \
	&& apt install -y \
		bash-completion \
		htop \
		vim \
	&& rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

# Workaround for faiing dep install on osrf/ros image https://stackoverflow.com/a/48569233
# RUN python -m easy_install --upgrade pyOpenSSL

# RUN pip install \
# 	Adafruit-GPIO \
# 	Adafruit-MCP3008 \
# 	spidev \
# 	pyserial

RUN mkdir -p ${CODE_MOUNT} \
&& cd ${CODE_MOUNT} \
&& mkdir -p ${PROJ} 
# clone any other repos here

COPY ${ROS_PKG}/ ${CODE_MOUNT}/${PROJ}/${ROS_PKG}/

RUN mkdir -p ${ROS_WS}/src && \
	ln -s ${CODE_MOUNT}/${PROJ}/${ROS_PKG} ${ROS_WS}/src/${ROS_PKG}
    # link any other repos here

COPY entrypoint.sh /
ENTRYPOINT [ "/entrypoint.sh" ]
CMD [ "bash" ]

RUN echo "source /entrypoint.sh" >> /root/.bashrc && \
	echo "source /root/.bashrc" >> /root/.bash_profile

# RUN cd ${ROS_WS} \
# && apt update \
# && rosdep update \
# && rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y \
# && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

RUN source "/opt/ros/$ROS_DISTRO/setup.bash" \
&& cd $ROS_WS && catkin_make \
&& rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*
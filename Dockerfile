FROM ros:noetic

LABEL   maintainer="Tammer Barkouki" \
        name="thbarkouki/explainablerobot"

ENV DEBAIN_FRONTEND noninteractive

RUN apt update -y && apt upgrade -y && apt install git libzmq3-dev libboost-dev -y \
# && <install packages here> \
    && apt-get clean autoclean && apt-get autoremove -y && rm -rf /var/lib/apt/lists/* \
    && echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc

WORKDIR /root/catkin_ws/src
RUN git clone https://github.com/uml-robotics/BehaviorTree.CPP.git behaviortree_cpp_v3 \
    &&  git clone https://github.com/tammerb/robot-explanation-BTs.git explain_bt

RUN /bin/bash -c '. /opt/ros/$ROS_DISTRO/setup.bash; cd $HOME/catkin_ws; catkin_make; cd ..'

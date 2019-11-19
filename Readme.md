
-------------------------------------------------------------------------------------------
ROS kinetic 설치

-------------------------------------------------------------------------------------------
Setup your sources.list
Setup your computer to accept software from packages.ros.org.

$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu xenial main" > /etc/apt/sources.list.d/ros-latest.list'


Set up your keys
$ sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

Alternatively, you can use curl instead of the apt-key command, which can be helpful if you are behind a proxy server:
$ curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | sudo apt-key add -

Installation

$ sudo apt-get update

$ sudo apt-get install ros-kinetic-desktop-full

$ mkdir catkin_ws
$ cd catkin_ws
$ sudo rosdep init
$ rosdep update

$ sudo gedit ~/.bashrc
맨 아래줄에 source /opt/ros/kinetic/setup.bash 추가

$ sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential

$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws
$ catkin_make

$ sudo gedit ~/.bashrc
맨 아래줄에 source ~/catkin_ws/devel/setup.bash 추가

-------------------------------------------------------------------------------------------
PX4 & MAVROS 설치

-------------------------------------------------------------------------------------------

$ cd
$ git clone https://github.com/PX4/Firmware.git
$ cd Firmware
$ git checkout v1.8.2
$ git submodule update --init --recursive
$ sudo apt install python-jinja2
$ sudo apt install python-pip
$ sudo pip install numpy toml
$ DONT_RUN=1 make posix_sitl_default gazebo

$ sudo gedit ~/.bashrc
아래 세줄 추가
source ~/Firmware/Tools/setup_gazebo.bash ~/Firmware ~/Firmware/build/posix_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/Firmware
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/Firmware/Tools/sitl_gazebo

$ gedit ~/Firmware/Tools/setup_gazebo.bash
아래 세줄앞에 # 추가로 주석처리
#echo -e "GAZEBO_PLUGIN_PATH $GAZEBO_PLUGIN_PATH"
#echo -e "GAZEBO_MODEL_PATH $GAZEBO_MODEL_PATH"
#echo -e "LD_LIBRARY_PATH $LD_LIBRARY_PATH"

MAVROS 설치
$ sudo apt-get install ros-kinetic-mavros ros-kinetic-mavros-extras
$ cd /opt/ros/kinetic/lib/mavros
$ sudo ./install_geographiclib_datasets.sh


$ cd ~/catkin_ws/src
$ git clone https://github.com/ICSL-hanyang/drone_base.git
$ cd ~/catkin_ws && catkin_make


$ roslaunch drone_base drone_base.launch

단 위 명령어를 실행하기전에 drone_base 패키지 내의 models, worlds 폴더안에 있는 파일들을 옮겨야함 Firmware 폴더로 옮겨야함.
즉, drone_base/models 안에 있는 파일을 ~/Firmware/Tools/sitl_gazebo/models 로 옮기고
drone_base/worlds 안에 있는 파일을 ~/Firmware/Tools/sitl_gazebo/worlds 로 옮긴 뒤
$ roslaunch drone_base drone_base.launch
를 실행해주면 됨




---------------------------

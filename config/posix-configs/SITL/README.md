
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
$ git clone https://github.com/ICSL-hanyang/uv_base.git
$ cd ~/catkin_ws && catkin_make


$ roslaunch uv_base uv_base.launch

단 위 명령어를 실행하기전에 uv_base 패키지 내의 models, worlds 폴더안에 있는 파일들을 옮겨야함 Firmware 폴더로 옮겨야함.
즉, uv_base/models 안에 있는 파일을 ~/Firmware/Tools/sitl_gazebo/models 로 옮기고
uv_base/worlds 안에 있는 파일을 ~/Firmware/Tools/sitl_gazebo/worlds 로 옮긴 뒤
$ roslaunch uv_base uv_base.launch
를 실행해주면 됨




---------------------------
  
There is a sample startup script at `swarm_ctrl_pkg/config/posix-configs/SITL/init`
 which looks like 
 ```
 uorb start
param load
dataman start
param set MAV_SYS_ID 1     <<<<<<<<<<<<<
param set BAT_N_CELLS 3
param set CAL_ACC0_ID 1376264
param set CAL_ACC0_XOFF 0.01
param set CAL_ACC0_XSCALE 1.01
param set CAL_ACC0_YOFF -0.01
param set CAL_ACC0_YSCALE 1.01
param set CAL_ACC0_ZOFF 0.01
param set CAL_ACC0_ZSCALE 1.01
param set CAL_ACC1_ID 1310728
param set CAL_ACC1_XOFF 0.01
param set CAL_GYRO0_ID 2293768
param set CAL_GYRO0_XOFF 0.01
param set CAL_MAG0_ID 196616
param set CAL_MAG0_XOFF 0.01
param set COM_DISARM_LAND 3
param set COM_OBL_ACT 2
param set COM_OBL_RC_ACT 0
param set COM_OF_LOSS_T 5
param set COM_RC_IN_MODE 1
param set EKF2_AID_MASK 1
param set EKF2_ANGERR_INIT 0.01
param set EKF2_GBIAS_INIT 0.01
param set EKF2_HGT_MODE 0
param set EKF2_MAG_TYPE 1
param set MAV_TYPE 2
param set MC_PITCH_P 6
param set MC_PITCHRATE_P 0.2
param set MC_ROLL_P 6
param set MC_ROLLRATE_P 0.2
param set MIS_TAKEOFF_ALT 2.5
param set MPC_HOLD_MAX_Z 2.0
param set MPC_Z_VEL_I 0.15
param set MPC_Z_VEL_P 0.6
param set NAV_ACC_RAD 2.0
param set NAV_DLL_ACT 2
param set RTL_DESCEND_ALT 5.0
param set RTL_LAND_DELAY 5
param set RTL_RETURN_ALT 30.0
param set SDLOG_DIRS_MAX 7
param set SENS_BOARD_ROT 0
param set SENS_BOARD_X_OFF 0.000001
param set SITL_UDP_PRT 14560     <<<<<<<<<<<<<
param set SYS_AUTOSTART 4010
param set SYS_MC_EST_GROUP 2
param set SYS_RESTART_TYPE 2
replay tryapplyparams
simulator start -s 
tone_alarm start
gyrosim start
accelsim start
barosim start
gpssim start
pwm_out_sim start
sensors start
commander start
land_detector start multicopter
navigator start
ekf2 start
mc_pos_control start
mc_att_control start
mixer load /dev/pwm_output0 ROMFS/px4fmu_common/mixers/quad_w.main.mix
mavlink start -x -u 14556 -r 4000000                        
mavlink start -x -u 14557 -r 4000000 -m onboard -o 14540    <<<<<<<<<<<<<
mavlink stream -r 50 -s POSITION_TARGET_LOCAL_NED -u 14556
mavlink stream -r 50 -s LOCAL_POSITION_NED -u 14556
mavlink stream -r 50 -s GLOBAL_POSITION_INT -u 14556
mavlink stream -r 50 -s ATTITUDE -u 14556
mavlink stream -r 50 -s ATTITUDE_QUATERNION -u 14556
mavlink stream -r 50 -s ATTITUDE_TARGET -u 14556
mavlink stream -r 50 -s SERVO_OUTPUT_RAW_0 -u 14556
mavlink stream -r 20 -s RC_CHANNELS -u 14556
mavlink stream -r 250 -s HIGHRES_IMU -u 14556
mavlink stream -r 10 -s OPTICAL_FLOW_RAD -u 14556
logger start -e -t
mavlink boot_complete
replay trystart

 ```
 and `<<<<<<<<<<<<<` line is where you have to modify  
 ## for QGC port number `14556` should be modified
 ```
 mavlink start -x -u 14556 -r 4000000 
 mavlink stream -r 50 -s POSITION_TARGET_LOCAL_NED -u 14556
mavlink stream -r 50 -s LOCAL_POSITION_NED -u 14556
mavlink stream -r 50 -s GLOBAL_POSITION_INT -u 14556
mavlink stream -r 50 -s ATTITUDE -u 14556
mavlink stream -r 50 -s ATTITUDE_QUATERNION -u 14556
mavlink stream -r 50 -s ATTITUDE_TARGET -u 14556
mavlink stream -r 50 -s SERVO_OUTPUT_RAW_0 -u 14556
mavlink stream -r 20 -s RC_CHANNELS -u 14556
mavlink stream -r 250 -s HIGHRES_IMU -u 14556
mavlink stream -r 10 -s OPTICAL_FLOW_RAD -u 14556
 ```
available port number
```
 포트는 받는놈 기준
```
ex) mavros 의 14557 이 받는다
|        |                     |        |
| :----: | :-----------------: | :----: |
|        | ---  `-u 14557` --> |        |
| px4    |                     | mavros |
|        | <--  `-o 14540` --- |        |


px4 >> -u 14557 >> mavros


| Number | sitl_udp_port | -u    | -o    | Qgc udp | Qgc remote |
| :----: | :-----------: | :---: | :---: | :-----: | :--------: |
| 1      | 13001         | 13101 | 13201 | 13301   | 13401      |
| 2      | 13002         | 13102 | 13202 | 13302   | 13402      |
| 3      | 13003         | 13103 | 13203 | 13303   | 13403      |
| 4      | 13004         | 13104 | 13204 | 13304   | 13404      |
| 5      | 13005         | 13105 | 13205 | 13305   | 13405      |
| 6      | 13006         | 13106 | 13206 | 13306   | 13406      |
| 7      | 13007         | 13107 | 13207 | 13307   | 13407      |
| 8      | 13008         | 13108 | 13208 | 13308   | 13408      |
| 9      | 13009         | 13109 | 13209 | 13309   | 13409      |
| 10     | 13010         | 13110 | 13210 | 13310   | 13410      |
| 11     | 13011         | 13111 | 13211 | 13311   | 13411      |
| 12     | 13012         | 13112 | 13212 | 13312   | 13412      |
| 13     | 13013         | 13113 | 13213 | 13313   | 13413      |
| 14     | 13014         | 13114 | 13214 | 13314   | 13414      |
| 15     | 13015         | 13115 | 13215 | 13315   | 13415      |
| 16     | 13016         | 13116 | 13216 | 13316   | 13416      |
| 17     | 13017         | 13117 | 13217 | 13317   | 13417      |
| 18     | 13018         | 13118 | 13218 | 13318   | 13418      |
| 19     | 13019         | 13119 | 13219 | 13319   | 13419      |
| 20     | 13020         | 13120 | 13220 | 13320   | 13420      |
| 21     | 13021         | 13121 | 13221 | 13321   | 13421      |
| 22     | 13022         | 13122 | 13222 | 13322   | 13422      |
| 23     | 13023         | 13123 | 13223 | 13323   | 13423      |
| 24     | 13024         | 13124 | 13224 | 13324   | 13424      |
| 25     | 13025         | 13125 | 13225 | 13325   | 13425      |
| 26     | 13026         | 13126 | 13226 | 13326   | 13426      |
| 27     | 13027         | 13127 | 13227 | 13327   | 13427      |
| 28     | 13028         | 13128 | 13228 | 13328   | 13428      |
| 29     | 13029         | 13129 | 13229 | 13329   | 13429      |
| 30     | 13030         | 13130 | 13230 | 13330   | 13430      |
| 31     | 13031         | 13131 | 13231 | 13331   | 13431      |
| 32     | 13032         | 13132 | 13232 | 13332   | 13432      |
| 33     | 13033         | 13133 | 13233 | 13333   | 13433      |
| 34     | 13034         | 13134 | 13234 | 13334   | 13434      |
| 35     | 13035         | 13135 | 13235 | 13335   | 13435      |
| 36     | 13036         | 13136 | 13236 | 13336   | 13436      |
| 37     | 13037         | 13137 | 13237 | 13337   | 13437      |
| 38     | 13038         | 13138 | 13238 | 13338   | 13438      |
| 39     | 13039         | 13139 | 13239 | 13339   | 13439      |
| 40     | 13040         | 13140 | 13240 | 13340   | 13440      |
| 41     | 13041         | 13141 | 13241 | 13341   | 13441      |
| 42     | 13042         | 13142 | 13242 | 13342   | 13442      |
| 43     | 13043         | 13143 | 13243 | 13343   | 13443      |
| 44     | 13044         | 13144 | 13244 | 13344   | 13444      |
| 45     | 13045         | 13145 | 13245 | 13345   | 13445      |
| 46     | 13046         | 13146 | 13246 | 13346   | 13446      |
| 47     | 13047         | 13147 | 13247 | 13347   | 13447      |
| 48     | 13048         | 13148 | 13248 | 13348   | 13448      |
| 49     | 13049         | 13149 | 13249 | 13349   | 13449      |
| 50     | 13050         | 13150 | 13250 | 13350   | 13450      |


 ## for sitl & mavros connection
 startup file
 ```
 param set MAV_SYS_ID 1
 param set SITL_UDP_PRT 14560
 mavlink start -x -u 14557 -r 4000000 -m onboard -o 14540
 ```
 and launch file lines
```xml
<arg name="tgt_system" value="$(arg ID)"/>
<arg name="mavlink_udp_port" value="14560"/>
<arg name="fcu_url" default="udp://:14540@localhost:14557"/>
```
should be matched

<맵핑>

SSH 터미널 1 — YDLIDAR X4 Pro

source /opt/ros/humble/setup.bash
source ~/ydlidar_ws/install/setup.bash

ros2 launch ydlidar_ros2_driver ydlidar_launch.py \
params_file:=/home/soyeon/ydlidar_ws/src/ydlidar_ros2_driver/params/X4-Pro.yaml


SSH 터미널 2 — 좌우 반전 수정

파일 경로: ~/M_J_T/robot/ros2_ws/src/robot_control/scan_mirror_fix.py

실행:
source /opt/ros/humble/setup.bash
source ~/ydlidar_ws/install/setup.bash

cd ~/M_J_T/robot/ros2_ws/src/robot_control
python3 scan_mirror_fix.py



SSH 터미널 3 — 기둥 필터

파일 경로: ~/M_J_T/robot/ros2_ws/src/robot_control/pillar_filter_node.py

실행:
source /opt/ros/humble/setup.bash

python3 ~/M_J_T/robot/ros2_ws/src/robot_control/pillar_filter_node.py



SSH 터미널 4 — 로봇 URDF / TF 실행

URDF 실제 경로: ~/M_J_T/robot/description/urdf/robot.urdf

실행:
source /opt/ros/humble/setup.bash

ros2 run robot_state_publisher robot_state_publisher \
--ros-args \
-p robot_description:="$(cat ~/M_J_T/robot/description/urdf/robot.urdf)"



SSH 터미널 5 — LiDAR Debug Marker

파일 경로: ~/M_J_T/robot/ros2_ws/src/robot_slam/lidar_debug_marker.py

실행:
source /opt/ros/humble/setup.bash
source ~/ydlidar_ws/install/setup.bash

cd ~/M_J_T/robot/ros2_ws/src/robot_slam
python3 lidar_debug_marker.py


SSH 터미널 6 — SLAM Toolbox

실행:
source /opt/ros/humble/setup.bash

ros2 run slam_toolbox async_slam_toolbox_node \
--ros-args \
--params-file ~/M_J_T/robot/ros2_ws/src/robot_slam/mapper_params_online_async.yaml \
-p scan_topic:=/scan_filtered


SSH 터미널 7 — Teleop

파일 경로: ~/M_J_T/robot/ros2_ws/src/robot_control/teleop.py

실행:
source /opt/ros/humble/setup.bash

cd ~/M_J_T/robot/ros2_ws/src/robot_control
python3 teleop.py


노트북 터미널 8 -RViz 실행

source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=30

rviz2

--------------------------------------------------------------------------------------------------------

<맵핑 후 저장>

SSH 터미널

source /opt/ros/humble/setup.bash
mkdir -p ~/M_J_T/robot/maps      // 맵을 저장할 폴더 만듦 (앞으로 여기에다 저장할거임. 만들엇으니까 앞으로는 할필요없는 명령어)
ros2 run nav2_map_server map_saver_cli -f ~/M_J_T/robot/maps/test2    // 맵 저장 명령. test2자리에 원하는 맵 이름 넣으면 됨

ls -al ~/M_J_T/robot/maps/      //파일이 생겼는지 확인하기

-------------------------------------------------------------------------------------------------------

<자율주행>

SSH 터미널 1 — YDLIDAR 실행

source /opt/ros/humble/setup.bash
source ~/ydlidar_ws/install/setup.bash
export ROS_DOMAIN_ID=30

ros2 launch ydlidar_ros2_driver ydlidar_launch.py \
params_file:=/home/soyeon/ydlidar_ws/src/ydlidar_ros2_driver/params/X4-Pro.yaml



SSH 터미널 2 — LiDAR Scan 반전 수정

source /opt/ros/humble/setup.bash
source ~/ydlidar_ws/install/setup.bash
export ROS_DOMAIN_ID=30

cd ~/M_J_T/robot/ros2_ws/src/robot_control
python3 scan_mirror_fix.py



SSH 터미널 3 — 기둥 필터

source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=30

python3 ~/M_J_T/robot/ros2_ws/src/robot_control/pillar_filter_node.py




SSH 터미널 4 — Robot State Publisher 실행

source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=30

ros2 run robot_state_publisher robot_state_publisher \
--ros-args \
-p robot_description:="$(cat ~/M_J_T/robot/description/urdf/robot.urdf)"



SSH 터미널 5 — 모터 명령 변환 노드 실행

source /opt/ros/humble/setup.bash
source ~/M_J_T/robot/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=30
export PI_API_KEY='API 키 넣기'

cd ~/M_J_T/robot/ros2_ws/src/robot_control
python3 cmd_vel_to_motor.py




SSH 터미널 6 — 새 로봇 Navigation 실행

source /opt/ros/humble/setup.bash
source ~/M_J_T/robot/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=30

ros2 launch robot_navigation navigation.launch.py map:=/home/soyeon/M_J_T/robot/maps/test2.yaml

// test2 자리에 불러오고 싶은 맵 이름 입력



노트북 터미널 7 -RViz 실행

source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=30

rviz2


-------------------------------------------------------------------------------------------------------

<좌표 저장>

1. 노트북에서 좌표 저장

노트북에서 실행:

cd ~/turtlebot_test
python3 record_book_coords.py

3. 저장된 좌표를 라즈베리파이로 복사

scp ~/turtlebot_test/book_locations.json \
원격접속코드:~/turtlebot_test/book_locations.json

-----------정상확인-----------------

3. 노트북에 새로 저장한 JSON 확인
노트북 터미널:
hostname
cat ~/turtlebot_test/book_locations.json

4. 라즈베리파이에서 좌표 확인
새로 저장한 좌표가 보이면 정상:
cat ~/turtlebot_test/book_locations.json


--------------------------------------------------------------------------------------------------------


<앱 연동>

노트북 터미널 -  rosbridge 실행

source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=30

ros2 launch rosbridge_server rosbridge_websocket_launch.xml



노트북 터미널 - 앱/Nav2 브리지 실행

source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=30

cd ~/turtlebot_test
python3 app_nav_bridge.py


노트북 터미널 - 엔그록 실행

ngrok http 9090

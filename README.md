# turtlebot3_slam-
2025 05 - 2025 06

turtlebot 주요 기능 :
- 이동(cmd_vel)
- SLAM(gmapping 이용 지도 작성)
- navigation(move_base 이용 자율 주행)
- Obstacle avoidance(Lidar 센서 이용 장애물 감지 및 회피)


# TurtleBot 세팅 가이드

초기 설치, 센서 확인, 카메라 연결 등 

##  필수 패키지 설치 (원격 PC 기준)

```bash
sudo apt-get install ros-noetic-joy ros-noetic-teleop-twist-joy ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc ros-noetic-rgbd-launch ros-noetic-rosserial-arduino ros-noetic-rosserial-python ros-noetic-rosserial-client ros-noetic-rosserial-msgs ros-noetic-amcl ros-noetic-map-server ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro ros-noetic-compressed-image-transport ros-noetic-rqt* ros-noetic-rviz ros-noetic-gmapping ros-noetic-navigation ros-noetic-interactive-markers

sudo apt install ros-noetic-dynamixel-sdk
sudo apt install ros-noetic-turtlebot3-msgs 
sudo apt install ros-noetic-turtlebot3
```

##  네트워크 연결

- **공통 네트워크 사용**: TurtleBot과 원격 PC를 동일한 네트워크(예: 핫스팟)로 연결
- **TurtleBot 내부 설정**:
  ```bash
  cd /etc/netplan/
  sudo nano 50-cloud-init.yaml
  sudo reboot
  ```
- **IP 확인**:
  ```bash
  ifconfig
  ```

##  환경 변수 설정

### TurtleBot (On-board PC)

```bash
nano ~/.bashrc
# ROS_MASTER_URI, ROS_HOSTNAME 설정
source ~/.bashrc
```

### 원격 PC (Laptop 등)

```bash
nano ~/.bashrc
# ROS_MASTER_URI=TurtleBot IP
# ROS_HOSTNAME=원격 PC IP
source ~/.bashrc
```

##  카메라 확인

### 1. Custom Package 생성

```bash
cd ~/catkin_ws/src
catkin_create_pkg school_classes std_msgs rospy roscpp
```

### 2. `CMakeLists.txt` 설정

```cmake
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  image_transport
  cv_bridge
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(school_classes_node src/school_classes_node.cpp)
target_link_libraries(school_classes_node ${catkin_LIBRARIES})
```

### 3. 카메라 Subscribe 코드 예시 (C++)

```cpp
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
  cv::imshow("Camera", image);
  cv::waitKey(1);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "image_subscriber");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/camera/image", 1, imageCallback);
  ros::spin();
  return 0;
}
```

### 4. Launch 파일 생성

```xml
<!-- launch/image_subscriber_node.launch -->
<launch>
  <node pkg="school_classes" type="school_classes_node" name="image_subscriber_node" output="screen"/>
</launch>
```

### 5. 빌드 및 실행

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
roslaunch school_classes image_subscriber_node.launch
```

##  LiDAR 연결

- SSH 터미널에서 실행:

```bash
roslaunch turtlebot3_bringup turtlebot3_lidar.launch
```

- RViz에서 확인 가능

---


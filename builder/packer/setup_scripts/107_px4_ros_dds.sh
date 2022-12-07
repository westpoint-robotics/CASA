## Fast DDS and dependencies
sudo apt-get install -y libssl-dev
sudo apt-get install -y libasio-dev libtinyxml2-dev
sudo apt-get install -y libp11-dev libengine-pkcs11-openssl
sudo apt-get install -y softhsm2
sudo apt-get install -y openjdk-11-jdk-headless
sudo apt-get install -y ros-humble-rviz2

source /opt/ros/humble/setup.bash

cd ~
mkdir fastdds/
cd fastdds

git clone https://github.com/eProsima/foonathan_memory_vendor.git
cd foonathan_memory_vendor
mkdir build
cd build
cmake ..
sudo cmake --build . --target install

# FastCDR
cd ~/fastdds
git clone https://github.com/eProsima/Fast-CDR.git
mkdir Fast-CDR/build
cd Fast-CDR/build
cmake .. -DCMAKE_INSTALL_PREFIX=~/fastdds/Fast-CDR/install
cmake --build . --target install

# FastDDS
cd ~/fastdds
git clone https://github.com/eProsima/Fast-DDS.git
cd Fast-DDS/src
cmake ..  -DCMAKE_INSTALL_PREFIX=~/fastdds/Fast-DDS/install
cmake --build . --target install
cmake ..  -DCMAKE_INSTALL_PREFIX=~/fastdds/Fast-DDS/install -DTHIRDPARTY=ON -DSECURITY=ON 
make -j$(nproc --all) 
sudo make install

# Fast DDS Gen
cd ~/fastdds
git clone --recursive https://github.com/eProsima/Fast-DDS-Gen.git -b v1.0.4 ~/fastdds/Fast-DDS-Gen
cd ~/fastdds/Fast-DDS-Gen/
sed -i 's/5.6.2/6.8.3/g' gradle/wrapper/gradle-wrapper.properties 
./gradlew assemble
sudo env "PATH=$PATH" ./gradlew install

# micro_ros_agent
cd ~
mkdir ~/microros_ws
cd microros_ws
git clone -b humble https://github.com/micro-ROS/micro_ros_setup.git 
mkdir src
mv micro_ros_setup/ src/
colcon build
source install/local_setup.sh
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh


# px4_ros_com and px4_msgs
cd ~
mkdir px4_ros_com_ws
cd px4_ros_com_ws
git clone https://github.com/PX4/px4_ros_com.git
git clone https://github.com/PX4/px4_msgs.git
colcon build

# px4-offboard
cd ~
git clone https://github.com/Jaeyoung-Lim/px4-offboard.git
cd px4-offboard
source ../px4_ros_com_ws/install/setup.sh
colcon build

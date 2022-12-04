sudo usermod -a -G dialout $USER
sudo apt-get remove modemmanager -y
sudo apt install -y gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
sudo apt install libqt5gui5 -y
sudo apt install libfuse2 fuse -y
mkdir ~/utils
cd utils
wget https://github.com/mavlink/qgroundcontrol/releases/download/v4.2.4/QGroundControl.AppImage
sudo chmod +x QGroundControl.AppImage
#!/bin/bash

sudo apt-get install ifconfig

echo "export SYS_ID = $1" >> ~/.bashrc

sudo usermod -a -G tty $USER
sudo usermod -a -G dialout $USER

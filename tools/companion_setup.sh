#!/bin/bash

sudo apt-get install ifconfig

echo "export SYS_ID = $1" >> ~/.bashrc

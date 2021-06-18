#!/bin/sh
cd /home/debian/CANOpenRobotController
sudo script/initCAN1.sh
rm logs/*
sudo -u debian build/M2Spasticity_APP


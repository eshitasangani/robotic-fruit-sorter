#!/bin/bash
sudo rm /var/run/pigpio.pid
sudo pigpiod
sudo python3 /home/pi/final-proj/full_demo.py

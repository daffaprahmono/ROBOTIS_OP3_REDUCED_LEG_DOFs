#!/bin/bash
v4l2-ctl -d /dev/video0 -c auto_exposure=1
v4l2-ctl -d /dev/video0 -c exposure_dynamic_framerate=0
v4l2-ctl -d /dev/video0 -c exposure_time_absolute=260
v4l2-ctl -d /dev/video0 -c gain=64
v4l2-ctl -d /dev/video0 -c brightness=150
v4l2-ctl -d /dev/video0 -c contrast=110
v4l2-ctl -d /dev/video0 -c saturation=128
v4l2-ctl -d /dev/video0 -c sharpness=128
v4l2-ctl -d /dev/video0 -c white_balance_automatic=1
v4l2-ctl -d /dev/video0 -c backlight_compensation=0

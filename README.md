# AI-SDC_WRG2021
This repo contain a code for my robot used to compete in WRG2021 online edition in AI self driving car category

This Video show how the robot run in the audition round.
<iframe width="560" height="315" src="https://www.youtube.com/embed/pF2o6g37ECA?start=44" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

Components:
- Nvidia Jetson Nano
- ESP32
- TB6612FNG Motor Driver
- IMX219-160 Camera
- 3 cells 18650 battery

The robot have Jetson nano as a main computer which is used to detect lane lines and reading april tags. The low level control task is handled by ESP32 which communicate with Jetson nano via USB serial connection.

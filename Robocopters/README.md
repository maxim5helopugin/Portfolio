# QUAD: Quick UAV-Attacking Drone

Autonomous quadcopter which visually detects target quadcopters and pursues them while avoiding obstacles. 

<ul>
<li>Vision uses ORB feature keypoints, HSV color weighting, and optical flow tracking to detect and track targets quickly with OpenCV on Raspberry Pi.</li>
<li>Guidance and navigation synthesizes data from various sensors, including IMU, optical flow camera, laser rangefinder, and ultrasonic sensors.</li>
<li>Gazebo demonstrates successful mission completion in simulated environment running the same software/ROS as on actual vehicle.</li>
</ul> 

Interdisciplinary senior design project from the University of Central Florida, 2017-2018. Sponsored by Lockheed Martin. 

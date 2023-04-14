# Parking done
## In order to run a node you need:

1. have a joystick version that supports charging buttons [`joystick`](https://github.com/AlexanderKamynin/dt-automatic-charging/tree/parking)

2. have a charging driver version [`driver`](https://github.com/OSLL/charging-driver/tree/automatic-charging)

## After cloning the repository, you need to build the project with the command: 

`dts devel build -f -H && dts devel run -f -H`

## After cloning the charge driver repository, you need to run the charge driver to create the charge state topic with the command:

`docker -H <autobot name>.local run --name charging_driver -v /dev/mem --privileged --network=host -dit --restart unless-stopped -e ROBOT_TYPE=duckiebot docker.io/duckietown/charging-driver :automatic-charging-arm32v7`

## After cloning the joystick repository, you need to build it with the command:

`dts devel build -f -H <autobot name> --tag daffy`

## And run it with the command:

`dts duckiebot keyboard_control <autobot name>`

## When all nodes (responsible for parking, charging status and joystick) are running, you can start the parking process

With the joystick active, you need to press the "F" button to start parking or "G" to end it. 
### Be careful! 
Due to the peculiarity of ROS, it is not enough to briefly press "F", in this case it is necessary to hold down "F" for 1-2 seconds

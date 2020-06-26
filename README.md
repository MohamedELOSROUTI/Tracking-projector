# Major Project in Electronics : Buck converter PID and Projector tracking.
![](https://i.imgur.com/X9Qw94F.png)
The purpose of this project is to program a microcontroller : PIC16F1789 in order to manage the feedback loop of a buck converter and a tracking projector. 

The buck takes as input a voltage between [20V, 30V] and must produce a regulated voltage of 8V.

Ts = 1/30 kHz is the switching frequency of the Buck. 

The output is sampled at a rate of 15 kHz (T=2*Ts) through a voltage divider of 1/6.

The analog to digital conversion of the output voltage is done in 10 bits using 2's complement. 


A PI control is used for the projector too.

## Description of the system
The system will consist of the spotlight and its support and the reference will be the moving person in the room. The microcontroller will compute and control the angular position at which the spotlight should be in order to light the tracked body. It will thus take as input the position of the moving person and depending on that the DC motor torque will change in order to make the system move in a certain direction. The information of the location of the moving person will come from the two ultrasonic emitter-receiver sensors. The block diagram which illustrates the links between all the components of the system is represented in Figure 1.

Then the buck converter that will be entirely build and implemented will supply the H-bridge which will drive the motor. The two ultrasonic sensors will compute the distance between them and the moving body and the microcontroller will control the buck converter and control the DC motor in order to track the moving person thanks to the data sent by the ultrasonic sensors.

Here's the final implementation 

![](https://i.imgur.com/qQMkzd6.png)

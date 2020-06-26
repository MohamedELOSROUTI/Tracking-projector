# Major Project in Electronics : Buck converter PID and Projector tracking.
## Introduction

As part of my master's course, we had to design, test and build a projector able to detect a person in a room and track its movement. This project involves electronics skills related to power electronics and embedded systems. 

The power electronics part concerns the conception of a DC buck converter that has to power all the embedded system.

This is a project that lasted one year. Smart choices of components, microcontroller, sensors have been made in order to maximise the efficiency of the buck converter and the tracking process.


## 1. Buck converter
<p align="center">
  <img src="https://i.imgur.com/X9Qw94F.png" />
</p>

The purpose of this project is to program a microcontroller ( PIC16F1789) in order to manage the feedback loop of a buck converter and a tracking projector. 

The buck takes as input a DC voltage between [20V, 30V] and must produce a regulated voltage of 8V. The switching frequency of the buck is chosen equal to 30 kHz (PWM frequency). The output is sampled at a rate of 15 kHz through a voltage divider of 1/6. The voltage divider allows to make sure that the input voltage doesn't exceed 5V (for the microcontroller). The analog to digital conversion of the output voltage is done with 10 bits using 2's complement. The microcontroller is in charge of sampling and quantization. A digital PI control is used to produce a regulated voltage of 8 V.  

The buck converter powers the H-bridge, the actuators, the sensors and the microcontroller.



## 2. Description of the tracking projector

The aim of the projector is to track a moving person in a room. For that, it needs to know its location. Two symetric ultrasound sensors are constantly measuring the distances between the person and the spotlight. Those informations are transmitted to the microcontroller. Let's assume that :

- $d_1$ is the distance between the left US sensor and the spotlight.
- $d_2$ is the distance between the right US sensor and the spotlight.

The CPU is in charge of computing the difference between the two distances ($d_1$ and $d_2$). According the sign of the difference, the microcontroller will adjust the direction of rotation of the DC motor. This way, the person remains spotted by the projector if the person moves around it. 

The CPU will always try to keep $d_2$-$d_1$ as close as possible to 0 by adjusting the position of the spotlight ! Furthermore, the micrcontroller knows the angle of the shaft thanks to its angular sensor.

The H-Bridge allows to control the direction of rotation of the DC motor. Furhermore, a pwm input voltage allows to control its rotation speed.


## 3. Final implementation 
<p align="center">
  <img src="https://i.imgur.com/qQMkzd6.png" />
</p>
See Final_Report___Major_project.pdf for more information about the code and the analog implementation (PCB design, pseudo-codes, ...).

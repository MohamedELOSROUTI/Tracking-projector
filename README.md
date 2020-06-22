# Assembly-code-PID-CONTROL
Major Project in Electronics : Buck converter PID and Projector tracking.

The purpose of this project is to program a microcontroller : PIC16F1789 in assembly code
in order to manage the feedback loop of a buck converter and a tracking projector.

The buck takes as input a voltage between [20V, 30V] and must produce a regulated voltage of 8V.

Ts = 1/30 kHz is the switching period (PWM) of the Buck. 

The output is sampled at a rate of 15 kHZ (T=2*Ts) through a voltage divider of 1/6.

The analog to digital conversion of the output voltage is done in 10 bits using 2's complement. 

The parameter of the PID control for the buck are Kp = 0, Ki = 0.004, Kd = 0 => This allows us to 
have a phase margin of about 45 degree. Pratically no static error.

A PI control is used for the projector too.

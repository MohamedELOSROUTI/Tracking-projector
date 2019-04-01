# Assembly-code-PID-CONTROL
Major Project in Electronics : Buck converter PID.
The purpose of this project is to program a microcontroller : PIC16F1789 in assembly code
in order to manage the feedback loop of a buck converter.

The buck takes as input a voltage between [8V, 30V] and must produce a regulated voltage of 8V.

Ts = 1/30 kHz is the switching period (PWM) of the Buck. 

The output is sampled at a rate of 15 kHZ (T=2*Ts) through a voltage divider of 1/6.

The parameter of the PID control are Kp = 0.5, Ki = 0.0625, Kd = 2 => This allows us to 
have a phase margin of about 45 degree. Pratically no static error.

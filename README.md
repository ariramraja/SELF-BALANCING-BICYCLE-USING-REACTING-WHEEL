**SELF-BALANCING BICYCLE USING REACTING WHEEL WITH IMPROVING STABILITY**

**CHAPTER** – **1**

**INTRODUCTION**

`	`The idea is to make it balance by rotating a reaction wheel with a motor creating a torque with the moment of inertia making the bicycle counteracts its own weight. The acceleration of the wheels is then adjusted relative to the angle between the bicycle and the horizontal plane to keep the bicycle stable. 



`	`The solution here is making a bike which can balance itself using the principle of gyroscope so that the rider not have to worry about falling because of lack of balance over the bike. The purpose of this Project is to build a bike prototype that is capable of driving and balancing without a rider. The Automatic Balancing bike will employ a control system to keep itself from falling over while in motion, and be

propelled by a motor. The goal of this project is to build a two inline-wheel bike prototype capable of balancing itself using a reaction wheel. This bike is able to drive and also come to a complete stop without losing its balance. In order to maintain the balance, the robot reads sensor input to detect tilt angle and correctly reacts to maintain gyroscope for steady vertical position. The use of programming will help to steer the bike in any direction as per the user. The requirement include that the bike should be capable of accelerating, driving in a straight line and stopping without falling. It will be combination of both hardware and software technology.

We inspired by the fact that the design of the physical product seems very simple, but in reality, it is based on a complex control theory problem. Although the final product will not solve any everyday problems, the technology is important as it provides insight into how control theory can be applied in real projects. The experience of controlling motors and handling signals can benefit later projects since similar problems exists in many areas. Examples of areas where this technology is useful are for designing Segway’s, rockets and aircrafts etc. 

**CHAPTER** – **2**

**BASIC CONCEPTS IN SELF BALANCING BICYCLE**

**2.1 SYSTEM DYNAMICS**

To be able to stabilize the bicycle on its corner a relationship between the different affecting factors is needed and a model of the system dynamics is set up. Mmax is the maximum torque needed to counteract the construction´s own weight at a horizontal position and is given by equation 2.1 where a mechanical equilibrium Consider the diagram show in the Fig 2.1.gives this relationship between the bicycle ’s mass m, gravitation g and the horizontal

`         `![image](https://user-images.githubusercontent.com/54531993/181879166-e3aa4a54-3ed8-4b2d-a869-53368cc39556.png)

**Fig 2.1-** Reacting Wheel

. It works on the principle of angular momentum conservation of flywheel. Each wheel can produce torque only along its axis of rotation.





`			             	     `Mmax = mg ½.                                                                      (2.1)

This sets the requirement for needed torque from the motor. An important aspect is to make the construction mechanically stable because the mechanical stability will set a fundamental stability to the construction that gives a good starting point for the automatic control later. This is done by letting the center of mass being centered in the middle of the bicycle.

# **2.2. REACTION WHEEL** 
#
`	`The most suitable shape of a reaction wheel is a ring because it provides the greatest moment of inertia per mass that will result in a decreased acceleration. The theoretical moment of inertia for a ring can be calculated with the ring´s mass m and it´s radius r as

`                                                          `Iring = mr2.                                                                             (2.2)
\*
`	`When the motor accelerates the reaction wheel generates torque. Due to Newtons third law, a reaction torque that acts on the bicycle in the opposite direction is also generated. This can be described by the equation below with Iring as the wheels theoretical moment of inertia and the angle acceleration ω˙ motor 
\*
`                                                          `Mmotor   = Iringω˙ motor                                                                     (2.3)  

`	`By adjusting the speed and acceleration of the motor the angle between the bicycle and the surface can be controlled. In this project a Newtonian model was used that can be described

`                                     `Itotθ¨ = mglsin(θ) – Mmotor                                                                                    (2.4)


# **2.3. PID-CONTROLLER**
#
`	`A PID-controller is the most commonly used controller in the industry. The con- troller use a reference point that is the value it wants to reach and compare it with its current value and this difference is the error. It then uses the error to stabilize the system with three components. PID means that the controller consists of three components: a proportional, an integrative and a derivative component. The P- component is used to adjust the gain, the I-component is used to reduce the static error, and the D-component compensates for changes in the system that increases the stability.

`	`Since Pico Pi includes a built in a PID-function within its libraries it will be used in this project. The PID-function has three parameters KP, KI, and KD that will be choose experimentally. The ideal PID-regulator can be written as:** 


`                                `![image](https://user-images.githubusercontent.com/54531993/181879216-bcb6cdd4-1a43-4b39-ab50-1ce9b3c9afb8.png)                          (2.5)

`	`There are different ways a PID-controller can be tuned to stabilize a system. One method is the Ziegler-Nicholas method where analysis of the system could give a good approximation of the three components creating the PID-controller. Given a system the simplest method is to increase the gain for the system until a steady state oscillation occurs, giving the Kcr. This with the given oscillation period Pcr can with this method give a good starting point for tuning the PID. The method suggests the following values in relation with the analyzed valued above:

KP = 0.6Kcr,

TI = 0.5Pcr,

`                                                                    `TD = 0.125Pcr.                                                         (2.6)










**2.4. MOTORS**

In this project a brushed DC motor was used. A brushed motor means that it rotates internally. DC stands for direct current which implies that ohm’s law and Kirchhoff’s voltage law can be used in calculations. The relationship between voltage, current and velocity can therefore  

`                                            `UA = RAIA + K2Φω.                                                                   (2.7)

The torque that the motor generates is proportional to the current and is mathematically described as** 

K2Φ is a constant that can be calculated by measuring the speed of the rotor shaft when there is no load on the motor. This gives that the voltage is direct proportional to the rotating speed (UA = K2Φω). In this project it was instead found in the data sheet for the motor. Ra is the terminal resistance can be measured, but was also found in the data sheet.

The angular velocity of the flywheel changes over time when the voltage 12V is applied on the motor. Fig 2.2 shows the angular velocity was calculated with the Euler forward method and depends on the torque that was received from equation 2.6. In the data sheet the” no load speed” for the motor is 300 rpm. The reason why the motor passes this speed despite there is load on the motor might be because the used model does not consider internal friction of the motor. A theoretical value of the load was calculated by approximating the flywheel as a ring.

![image](https://user-images.githubusercontent.com/54531993/181879231-9bc76b74-1f25-4eb4-8a1e-b12e26f8c175.png)

**Fig 2.2 –** Angular velocity over time when a voltage of 12V is applied on the motor.

**2.5. GYROSCOPE /ACCELEROMETE**

**	To know how the bicycle is positioned according to the room a gyroscope /acetometer is used.** The gyrometers origin is set to the position the bicycle is supposed to balance in. When the bicycle is falling in some direction the gyrometers will give a signal to the micro controller of this change in angle. The micro controller will then do actions to stabilize the bicycle and bring its position back to origin.

**2.6. PICO RASPBERRY PI**

`	`Pi Pico is a micro-controller board based on in-house custom designed chip (RP2040) by Raspberry Pi. More experienced users can take advantage of Raspberry Pi Pico’s rich peripheral set, including SPI, I2C, and eight Programmable I/O (PIO) state machines for custom peripheral support.

`	`The chip at the heart of Pico, is an ultra-powerful Dual Core ARM Cortex-M0+ clocked at 133MHz with 256KB RAM, 2MB of on-board QSPI Flash memory for code and data storage. It also has 30 GPIO pins and lots of interfacing options including 2 × SPI, 2 × I2C, 2 × UART, 3 × 12-bit ADC, 16 × controllable PWM channels. Moreover, 26 of the GPIO pins are multi-function, which means we can those pins to a different behavior like UART/I2C/SPI/GPIO

**2.7. H-BRIDGE**

`	`To drive a DC-motor with higher voltage it is necessary to use a H-bridge Consider the diagram show in the Fig 2.3. This is due to the Pico pi´s inability to output voltage over 5 V. A H-bridge is used to control an external power source with small PWM-signals described in section 2.2.3. 

![image](https://user-images.githubusercontent.com/54531993/181879241-64951d34-efd1-41cc-af94-2b8da9484f80.png)

**Fig 2.3 –** A schematic image of a H-bridge

Another reason why it is important to use an H-bridge is because DC-motors also can work as generators. Therefore, if some external force would make the motor start spinning it would induce a current which could destroy the Pico pi. Conventionally an H-bridge is made of four transistors that not only makes it possible to change the output voltage, it can also change the current direction. 

**2.8. PWM TO CONTROL MOTOR SPEED**

**	Fig 2.4 shows a picture of Pulse width modulation (PWM) is a modulation technique that generates variable-width pulses to represent the amplitude of an analog input signal. The output switching transistor is on more of the time for a high-amplitude signal and off more of the time for a low-amplitude signal. The digital nature (fully on or off) of the PWM circuit is less costly to fabricate than an analog circuit that does not drift over time.

`	`PWM is widely used in ROV applications to control the speed of a DC motor and/or the brightness of a lightbulb. For example, if the line were closed for 1 μs, opened for 1 μs, and continuously repeated, the target would receive an average of 50% of the voltage and run at half speed or the bulb at half brightness. If the line were closed for 1 μs and open for 3 μs, the target would receive an average of 25%.

![image](https://user-images.githubusercontent.com/54531993/181879246-b7e824dc-87d4-4344-bf7d-6f3f3483c22d.png)

**Fig 2.4 –** PWM signals with different duty cycles.

**CHAPTER** – **3**

**COMPONENTS IMPLEMENTATIONS IN BICYCLE**

**3.1. GYROSCOPE**
	The MPU-6050 Fig.3.1 is used to detect the roll angle of the bicycle.  It gives values for the change of angle on the z-axle which was used as input for the PID-controller to control the control system.

![image](https://user-images.githubusercontent.com/54531993/181879303-f7296805-04ef-4aaa-a406-68232b704d94.png)

**Fig 3.1 –** MPU-6050 – Gyroscope/Accelerometer

**3.2. SERVO MOTOR**

**	There are some special types of application of electrical motor where rotation of the motor is required for just a certain angle not continuously for long period of time.  For these applications, some special types of motor are required with some special arrangement which makes the motor to rotate a certain angle for a given electrical input (signal).  This is normally a simple motor which is controlled for specific angular rotation with the help of additional servomechanism (a typical closed loop feedback control system).  Servo motor is a special type of motor which is automatically operated up to certain limit for a given command with help of error-sensing feedback to correct the performance



**3.3. L298N DRIVER**

**	The L298N is a dual H-Bridge motor driver as shown in Fig 3.2, which allows speed and direction control of two DC motors at the same time. The module can drive DC motors that have voltages between 5 and 35V, with a peak current up to 2A. This depends on the voltage used at the motors VCC. The module has an onboard 5V regulator which is either enabled or disabled using a jumper.

![image](https://user-images.githubusercontent.com/54531993/181879327-4d0f0a15-0851-46d4-880a-92f59f3e2e32.png)

**Fig 3.2 –** L298N Driver 

`	`If the motor supply voltage is up to 12V we can enable the 5V regulator and the 5V pin can be used as output, for example for powering our Pico Pi board. But if the motor voltage is greater than 12V we must disconnect the jumper because those voltages will cause damage to the onboard 5V regulator. In this case the 5V pin will be used as input as we need connect it to a 5V power supply in order the IC to work properly.




**3.4. BLUETOOTH MODULE (HC-05 Bluetooth Module)**
	HC‐05 module is an easy-to-use Bluetooth SPP (Serial Port Protocol) module, designed for transparent wireless serial connection setup. The HC-05 Bluetooth driver as shown in Fig 3.3, can be used in a Master or Slave configuration, making it a great solution for wireless communication. This serial port Bluetooth module is fully qualified Bluetooth V2.0+EDR (Enhanced Data Rate)3Mbps Modulation with complete 2.4GHz radio transceiver and baseband. It uses CSR Blue core 04‐ External single chip Bluetooth system with CMOS technology and with AFH (Adaptive Frequency Hopping Feature). 

![image](https://user-images.githubusercontent.com/54531993/181879360-0b6781d7-26d5-403d-b09c-25adaa716a29.png)

**Fig 3.3 –** Bluetooth module


**3.5. BICYCLE FRAME**

**	The bicycle is designed to have all of its components within its sides with the sides acting as a protective shell and its corner as good balancing points. The shell will consist of six plates that make up the bicycles sides that will be laser cut from acrylic glass with a thickness of three millimeter. Acrylic glass was chosen because of its low density and good enough yield strength. With reference to Fig 3.4, bicycle we are using two other plates acting as motor holders that is connected to two opposite sides with distance screws in each corner of the plates. The motor is mounted on the motor holders. The two reaction wheels is placed on the motor axle on opposite sides between the side of the bicycle and the motor holders,

![image](https://user-images.githubusercontent.com/54531993/181879365-e4cf9c41-51dd-43f2-a41c-aa61c63716ad.png)

**Fig 3.4 –** The Bicycle with Reaction Wheels and Motor Installed


**3.6. REACTION WHEEL**

**	The wheel’s purpose is to generate as much moment of inertia per mass, therefore the shape of a ring. Two wheels with a radius of 7 centimeter and the weight of 95 g were created to generate it.Fig.3.5 show reaction wheel CAD diagram

`	`The reaction wheel is made of acrylic sheet and cut because of its simplicity with the wheel only needed to be cut in one dimension. The wheel is then mounted on the motor axle with a hub that is connected directly on the reaction wheel with six M3 screws and then locked on the axle with two set screws from the hub. A requirement for the wheel is to be able to store enough energy that it takes for the bicycle to tilt to an angle of 45 ◦. The kinetic energy that the wheel can store is 

`	`where ωmax is the maximum speed [rad/s] and Iring is the moment of inertia that is described in equation 3.1. The energy it needs to store is the difference in potential energy between the states when it is lying horizontal on the surface and balancing at a 45 ◦ angle. This is mathematically described



`                                                    `V = √2m bicycle G½                                                    (3.1)

`	`where bicycle is the mass of the bicycle [kg], g is gravitational acceleration [m/s2] and l is the length of the bicycle. By using setting V equal to T the wheels minimum mass can be calculated to

![image](https://user-images.githubusercontent.com/54531993/181879376-87f8dc05-e21a-49d0-8a9b-d6102ad26b74.png)

**Fig 3.5 –** Reaction wheel from the side. Made in Solid Edge

`	`The dynamic model of a bicycle is based on the equilibrium of gravity and centrifugal force. A simplified model for balancing is derived using the Lagrange method and neglecting force generated by the bicycle moving forward and steering. This model is based on the work of Parnichkun, which is a simplified dynamics model of the bicycle for balancing control while derived using the Lagrange method and neglecting force generated, as stated, by the bicycle moving forward and steering. With reference to Figure 2.3, the system, consisting of two rigid body links, has as its first link a bicycle frame having 1 degree-of-freedom (DOF) rotation around the Z axis. The second link is the flywheel, which is assumed to have constant speed ω. The flywheel centre of gravity (COG) is fixed relative to the bicycle frame.


**CHAPTER – 4**

**IMPLEMENTATION OF PID-CONTROLLER**

**4.1. STABILITY ANALYSIS OF PID SYSTEM**

`	`To construct a stable system with a PID-controller, the three components KP, KI and KD need to be tuned right. Fig 4.1 shows the Proportional-integral-derivative control (PID) combines the stabilizing influence of the derivative term and the reduction in steady-state error from the integral term.

`                           `![image](https://user-images.githubusercontent.com/54531993/181879391-35104fb6-2786-4526-bfa6-60bd2956e5ca.png)                       (4.1)

![image](https://user-images.githubusercontent.com/54531993/181879395-99eeb7f1-b6bf-414c-868f-0c498cbd9f6f.png)

**Fig 4.1 –** Block diagram PID controller

**4.2. PROPORTIONAL**

**	The proportional term produces an output value that is proportional to the current error value. The proportional response can be adjusted by multiplying the error by a constant Kp, called the proportional gain constant.

`                                                     `![image](https://user-images.githubusercontent.com/54531993/181879405-99241828-9111-4c30-bff1-7c62e0ed3de2.png)                                             (4.2)





**4.3. INTEGRAL**

**	     The contribution from the integral term is proportional to both the magnitude of the error and the duration of the error. The integral in a PID controller is the sum of the instantaneous error over time and gives the accumulated offset that should have been corrected previously. The accumulated error is then multiplied by Equations (4.3) the integral gain (Ki) and added to the controller output.

`                                                  `![image](https://user-images.githubusercontent.com/54531993/181879416-305a5ee5-47bd-4dd8-bd34-918fa6f105f0.png)                                         (4.3)

**4.4. DERIVATIVE**

**	The derivative of the process error is calculated by determining the derivative gain, Kd. The magnitude of the contribution of the derivative term to the overall control action is termed the derivative gain, Kd.

`                                                  `![image](https://user-images.githubusercontent.com/54531993/181879423-693249a0-284d-43f3-9559-174f9905b67e.png)                                (4.4)

**CHAPTER – 5**

**DESIGN OF SELF BALANCING BICYCLE**

**5.1. BLOCK DIAGRAM OF SELF BALANCING BICYCLE**
![image](https://user-images.githubusercontent.com/54531993/181879425-34e5724b-261b-43b2-9714-fe558498fb08.png)

` `**Fig 5.1 –** Block diagram of self balancing bicycle using reacting wheel



**	A Fig 5.1 is as show a block diagram connecting line in all parts. It is carried data in bipolar and unipolar example I2c, UART, SPI, etc.





**5.2. DESCRIPTION OF THE BLOCK DIAGRAM:**

**RASPBERRY PI PICO:** The Pico Pi is used to perform all the operations in the system. The microcontroller is programmed to control the motor for control a self-balancing control to avoid falling ground and I²C received signal to microcontroller Pico pi its work in mathematical operation in PID to find an Error value.




|**Feature**|**Availability**|
| :-: | :-: |
|Microcontroller|RP2040|
|Architecture|Dual-core Arm Cortex M0+ processor|
|Flash|2MB|
|SRAM|264KB|
|GPIO Pins|26|
|I2C|2|
|UART|2|
|SPI|2|
|PWM|8 PWM Blocks 16 Outputs|
|ADC|3 12-bit ADC|
|State Machines|8 × Programmable I/O (PIO) state machines|
|DAC|0|
|USB|USB 1.1 with device and host support|
|Timer|single 64-bit counter|
|Watchdog|one 32-bit|
|Real time clock|1- RTC|

**Table 5.1-** Raspberry Pi Pico Features and Specifications

**MPU6050:** The MPU-6050 devices combine a 3-axis gyroscope and a 3-axis accelerometer on the same silicon die, together with an onboard Digital Motion Processor, which processes complex 6-axis Motion Fusion algorithms. The device can access external magnetometers or other sensors through an auxiliary master I²C bus, allowing the devices to gather a full set of sensor data without intervention from the system processor.

**L298N**: The L298N motor driver module has two screw terminal blocks for the connecting two motors A and B. There is also a power supply screw terminal block containing the Ground pin, the VCC for motor and a 5V pin which can either be an input or output. The Table.5.2 below shows a  the pin out of this motor driver. Input pins IN1, IN2, IN3 and IN4 are for controlling the direction of rotation of the motors where IN1 and IN2 control the direction of rotation of motor A while IN3 and IN4 control direction of rotation of motor B.

|**L298N IC pins**|**Name**|**Function**|
| :-: | :-: | :-: |
|1,15|Sense A, Sense B|connected to control the current of the load.|
|2,3|Out 1, Out 2|Outputs of the Bridge A|
|4|VS|Supply Voltage |
|5,7|Input 1, Input 2|TTL Compatible Inputs of the Bridge A|
|6,11|Enable A, Enable B|TTL Compatible Enable Input|
|8|GND|Ground|
|9|VSS|Supply Voltage |
|10,12|Input 3, Input 4|Inputs of the Bridge B.|
|13,14|Out 3, Out 4|Outputs of the Bridge B. |

**Table 5.2 –** L298N IC pin Features and Specifications

**HC-05 Bluetooth Module:** Make the connections and power on the Bluetooth Module. If this is the first time you are using your Bluetooth Module, then the LED will blink rapidly some material is given in Table 5.3. In order to pair the module with your phone, open Bluetooth Settings in your phone and connect to “HC-05”,

|**Key Name**|**Data of Key**|
| :-: | :-: |
|LEFT DIRECTION|1|
|CENTER|2|
|RIGHT DIRECTION|3|
|FORWARD DIRECTION|4|
|REVERSE DIRECTION|5|


**	

**Table 5.3-** Bluetooth Data configuration


**5.3. FRITZING DIAGRAM OF SELF BALANCING BICYCLE USING REACTING WHEEL** 

![image](https://user-images.githubusercontent.com/54531993/181879462-a4a46161-bca2-4136-a05e-4080493a46aa.png)

**Fig 5.2 –** Fritzing Diagram of Self Balancing Bicycle Using Reacting Wheel


`	`Fritzing is an open-source initiative to develop amateur or hobby CAD software for the design of electronics hardware, to support designers and artists ready to move from experimenting with a prototype to building a more permanent circuit.

**CHAPTER** – **6**

**DESCRIPTION OF HARDWARE AND SOFTWARE REQUIRED TO SELF BALANCING BICYCLE**

**HARDWARE REQUIRED:**

- **BO MOTOR**
- **H BRIDGE**
- **BATTERY**
- **BLUETOOTH**
- **REACTING WHEEL**
- **MICROCONTROLLER**
- **JUMP WIRE**

**6.1. BO MOTOR**

![image](https://user-images.githubusercontent.com/54531993/181879510-a398f6c5-4ce7-45af-9a6e-148b12dc998c.png)

**Fig 6.1 –** Bo Motor
**
`	`DC MOTOR concept is where gears reduce the speed of the vehicle but increase its torque is known as gear reduction. In DC motor is assembled with multiple gear setup. Speed of motor is counted in terms of rotations of the soft per minute is called RPM. RPM means Revolution Per Minute. BO (Battery Operated) light weight DC geared motor which gives good torque and rpm at lower voltages. This motor can run at approximately 300 rpm when driven by a single Li-Ion cell. Great for battery operated light weight robots.
**6.2. H BRIDGE**

![image](https://user-images.githubusercontent.com/54531993/181879533-bbca7762-1554-4535-937d-444ff329caac.png)

**Fig 6.2 –** L298N driver module

`	`L298N is a motor control device and speed control using pulse with module. It is control using a MOSFET duty cycle  



**6.3. BATTERY**

![](Aspose.Words.755274ce-220a-495a-8c56-e8dd6fac9353.030.png)

**Fig 6.3 –** Battery in series

**	These are becoming the most popular type of batteries for use in robotics because of their lightweight, high discharge rates and relatively good capacity, except the voltage ratings are available in increments of 3.7 V. Portable equipment needing higher voltages use battery packs with two or more cells connected in series. Shows a battery pack with four 3.6V Li-ion cells in series, also known as 4S, to produce 14.4V nominal. In comparison, a six-cell lead acid string with 2V/cell will generate 12V, and four alkaline with 1.5V/cell will give 6V. The motor driver has multiple holes or perforations over the circumference that could be used to mount it over the chassis, through screws or nuts and bolts.

The 9V battery is directly connected to pin number 8 of L293D. The Enable 1-2 & Enable 3-4 of L293D are connected to 5V.

**6.4. BLUETOOTH**

![](Aspose.Words.755274ce-220a-495a-8c56-e8dd6fac9353.031.png)![](Aspose.Words.755274ce-220a-495a-8c56-e8dd6fac9353.032.jpeg)

**Fig 6.4 –** Bluetooth module HC-05

`	`The HC-05 has two operating modes, one is the Data mode in which it can send and receive data from other Bluetooth devices and the other is the AT Command mode where the default device settings can be changed.

![](Aspose.Words.755274ce-220a-495a-8c56-e8dd6fac9353.033.png)**6.5. REACTING WHEEL**

![](Aspose.Words.755274ce-220a-495a-8c56-e8dd6fac9353.034.png)

**Fig 6.5 –** Reacting wheel
**
`	`A reaction wheel is a device consisting of a spinning wheel attached to an electric motor (brushless DC), whose speed can be controlled by onboard computer, producing the required torque for attitude control. It works on the principle of angular momentum conservation of flywheel. Each wheel can produce torque only along its axis of rotation. 

**6.6. MICROCONTROLLER**

![](Aspose.Words.755274ce-220a-495a-8c56-e8dd6fac9353.035.png)

**Fig 6.6 –** Pico Pi Pin Configuration
**
`	`The Raspberry Pi Pico is vastly different from any model before it. It is the first device to use RP2040 “Pi Silicon” which is a custom System on Chip (SoC) developed by the Raspberry Pi team which features a dual core Arm Cortex M0+ running at 133 MHz, 264KB of SRAM and 2MB of flash memory used to store files.

**6.7. JUMPER WIRE**

`                 `A jump wire also known as jumper, jumper wire, DuPont wire is an [electrical wire](https://en.wikipedia.org/wiki/Electrical_wire "Electrical wire"), or group of them in a cable, with a connector or pin at each end sometimes without them simply, which is normally used to interconnect the components of a [breadboard](https://en.wikipedia.org/wiki/Breadboard "Breadboard") or other prototype or test circuit, internally or with other equipment or components, without soldering.** Individual jump wires are fitted by inserting their “end connectors” into the slots provided in a breadboard, the [header connector](https://en.wikipedia.org/wiki/Pin_header#Header_connector "Pin header") of a circuit board, or a piece of test equipment.

![Jumper Wires Male to Male](Aspose.Words.755274ce-220a-495a-8c56-e8dd6fac9353.036.jpeg)









`		`The movement system is an important part of a robot and 

its objective is how to move robot from one point to another 

point. This system has some details which show us how to 

use motors and wheels. There are many kinds of motors and 

wheels. Our choice is dependent on the robot function, 

power, speed, and precision. [4]    

At the beginning of the project, at first, we wanted to use 

two step motors for gaining the best speed and a remarkable 

control but it was too hard and sometimes impossible to 

write a good program for two step motors because each step 

motor has at least four inputs, and moreover we must use 

two motor drivers, L298, for getting the required voltage to 

the motors. 

Actually, it is better to use gearbox motors instead of 

common DC motors because it has gears and an axle and its 

speed does not change towards the top of a hill or downhill. 

Pay attention that the more speed is, the less precision will be; 

thus it is better to choose a motor that has authentic RPM. 

Eventually, we used two DC gearbox motors.

The movement system is an important part of a robot and 

its objective is how to move robot from one point to another 

point. This system has some details which show us how to 

use motors and wheels. There are many kinds of motors and 

wheels. Our choice is dependent on the robot function, 

power, speed, and precision. [4]    

At the beginning of the project, at first, we wanted to use 

two step motors for gaining the best speed and a remarkable 

control but it was too hard and sometimes impossible to 

write a good program for two step motors because each step 

motor has at least four inputs, and moreover we must use 

two motor drivers, L298, for getting the required voltage to 

the motors. 

Actually, it is better to use gearbox motors instead of 

common DC motors because it has gears and an axle and its 

speed does not change towards the top of a hill or downhill. 

Pay attention that the more speed is, the less precision will be; 

thus it is better to choose a motor that has authentic RPM. 

Eventually, we used two DC gearbox motors.

The movement system is an important part of a robot and 

its objective is how to move robot from one point to another 

point. This system has some details which show us how to 

use motors and wheels. There are many kinds of motors and 

wheels. Our choice is dependent on the robot function, 

power, speed, and precision. [4]    

At the beginning of the project, at first, we wanted to use 

two step motors for gaining the best speed and a remarkable 

control but it was too hard and sometimes impossible to 

write a good program for two step motors because each step 

motor has at least four inputs, and moreover we must use 

two motor drivers, L298, for getting the required voltage to 

the motors. 

Actually, it is better to use gearbox motors instead of 

common DC motors because it has gears and an axle and its 

speed does not change towards the top of a hill or downhill. 

Pay attention that the more speed is, the less precision will be; 

thus it is better to choose a motor that has authentic RPM. 

Eventually, we used two DC gearbox motors.

The movement system is an important part of a robot and 

its objective is how to move robot from one point to another 

point. This system has some details which show us how to 

use motors and wheels. There are many kinds of motors and 

wheels. Our choice is dependent on the robot function, 

power, speed, and precision. [4]    

At the beginning of the project, at first, we wanted to use 

two step motors for gaining the best speed and a remarkable 

control but it was too hard and sometimes impossible to 

write a good program for two step motors because each step 

motor has at least four inputs, and moreover we must use 

two motor drivers, L298, for getting the required voltage to 

the motors. 

Actually, it is better to use gearbox motors instead of 

common DC motors because it has gears and an axle and its 

speed does not change towards the top of a hill or downhill. 

Pay attention that the more speed is, the less precision will be; 

thus it is better to choose a motor that has authentic RPM. 

Eventually, we used two DC gearbox motors.
**
`			`The movement system is an important part of a robot and 

its objective is how to move robot from one point to another 

point. This system has some details which show us how to 

use motors and wheels. There are many kinds of motors and 

wheels. Our choice is dependent on the robot function, 

power, speed, and precision. [4]    

At the beginning of the project, at first, we wanted to use 

two step motors for gaining the best speed and a remarkable 

control but it was too hard and sometimes impossible to 

write a good program for two step motors because each step 

motor has at least four inputs, and moreover we must use 

two motor drivers, L298, for getting the required voltage to 

the motors. 

Actually, it is better to use gearbox motors instead of 

common DC motors because it has gears and an axle and its 

speed does not change towards the top of a hill or downhill. 

Pay attention that the more speed is, the less precision will be; 

thus it is better to choose a motor that has authentic RPM. 

Eventually, we used

**Fig 6.7 –** jump wire

There are different types of jumper wires. Some have the same type of [electrical connector](https://en.wikipedia.org/wiki/Electrical_connector "Electrical connector") at both ends, while others have different connectors.

**6.8. SOFTWARE REQUIRED**

- Thonny – To upload a programming IDE
- Server Bluetooth Terminal – To control a Bluetooth hc-05 Module
- Fritzing – Fritzing is an open-source hardware initiative that makes electronics accessible as a creative. The design of electronics hardware, intended to allow designers and artists to build more permanent circuits from prototypes.

**6.9. ANALYSIS**

`	`The PID components were determined by experiments on the bicycle. The KP component was increased until the value made the Self balancing bicycle counteract its own weight when falling to one side. Though the controller was able to counteract the Self balancing bicycle falling it couldn’t counteract the response from the controller and the cube fell to the other side, the system is unstable. To compensate for this response a KD component was added that will counteract the response based on the speed of the error change. Due to the controller having the error as input it will only respond when an error occurs making the system never settling to the set point. Adding a KI component calculating the integral of the error over time will make the system even more stable. Though using this method to find accurate components for the PID controller the system in this project could not stabilize completely, but made the system many times more stable than using no controller.

![](Aspose.Words.755274ce-220a-495a-8c56-e8dd6fac9353.037.png)

**Fig 6.8** – Analysis of Stabilization

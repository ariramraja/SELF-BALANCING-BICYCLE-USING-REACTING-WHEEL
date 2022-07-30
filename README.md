# SELF-BALANCING-BICYCLE-USING-REACTING-WHEEL-WITH-IMPROVING-STABILITY-USING-PICO-PI-RP2040
In this ever-growing scenario of the world with growing population and also vehicles, the problem persists more importantly for handicapped people to drive their vehicle on these overcrowded roads and ever-increasing traffic. This work uses a control moment gyroscope as an actuator. The control moment gyroscope is typically used in a spacecraft to orient the vessel. Appling as an actuator to balance a bicycle is a creative and novel approach and is the first of its kind for balancing of a bicycle. In this project a Self-balancing bicycle using reacting wheel were constructed with the intent of it balancing on of one of its edges using a PID-controller to counteract the gravity pulling it down. To be able to this a good understanding of the size team was required to have the right components for the job. A DC motor is used reaction wheel on each side of its axle using the moment of inertia to convert the torque from the motor to an angular acceleration of the bicycle. An IMU is used to determine the angle difference of the center of mass from a fixed point in the room and this value was used as an input in the PID-controller. Different method is used to determine the P, I and D components of the PID-controller to create a stable system. The main goal of this project is to build such a prototype that can balance itself on two wheels with the help of gyroscope. This self-balancing bike with the application of gyroscope will omit the need of extra set of wheels or support required by the handicapped people. Also, normal people can use this application to minimize the risk of road accidents due to the loss of balance when braking or stopping suddenly in traffic. This application will also benefit in stabilizing the bike without any need of human help externally. As this bike will be fully powered by rechargeable cells, even though different methods are used no one successfully made the bicycle perfectly.
`INTRODUCTION`

	The idea is to make it balance by rotating a reaction wheel with a motor creating a torque with the moment of inertia making the bicycle counteracts its own weight. The acceleration of the wheels is then adjusted relative to the angle between the bicycle and the horizontal plane to keep the bicycle stable. 
	
	The solution here is making a bike which can balance itself using the principle of gyroscope so that the rider not have to worry about falling because of lack of balance over the bike. The purpose of this Project is to build a bike prototype that is capable of driving and balancing without a rider. The Automatic Balancing bike will employ a control system to keep itself from falling over while in motion, and be
propelled by a motor. The goal of this project is to build a two inline-wheel bike prototype capable of balancing itself using a reaction wheel. This bike is able to drive and also come to a complete stop without losing its balance. In order to maintain the balance, the robot reads sensor input to detect tilt angle and correctly reacts to maintain gyroscope for steady vertical position. The use of programming will help to steer the bike in any direction as per the user. The requirement include that the bike should be capable of accelerating, driving in a straight line and stopping without falling. It will be combination of both hardware and software technology.

We inspired by the fact that the design of the physical product seems very simple, but in reality, it is based on a complex control theory problem. Although the final product will not solve any everyday problems, the technology is important as it provides insight into how control theory can be applied in real projects. The experience of controlling motors and handling signals can benefit later projects since similar problems exists in many areas. Examples of areas where this technology is useful are for designing Segway’s, rockets and aircrafts etc. 

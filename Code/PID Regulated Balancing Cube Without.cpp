#include<PID_v1.h>

double roll_deg;

double input;

double output;

double setpoint;

double Kp = 5, Ki = 0, Kd = 0;

 
 
PID PID_controller (&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

doublepwmSignal;

constint pwmPin = 11;

constintdiagaPin = 10;

constintdiagbPin = 12;

constintbuttonPin = 2;

constinttrimPin = A0;

int i = 0;

forboth classes must be in the include path of your project
#include b " I2Cdev . hb "
#include b "MPU6050m -6Axis -MotionApps20 .hb"
  
#ifI2CDEVIMPLEMENTATION == I2CDEVARDUINOWIRE
#include b "Wire.hb "
#endif
  while (!Serial);

Serial.println (F (b "InitializingI2Cdevices ... b "));

 
mpu.initialize ();

Serial.println (F (b "Testingdeviceconnections ... b "));

 
Serial.println (mpu. 
		   testConnection ()? F (b "MPU6050connectionsuccessful b  ");
F (b "MPU6050connectionfailed b  "));

 
while (Serial.available () && Serial.read ());

while (Serial.available () && Serial.read ());

 
Serial.println (F (b "InitializingDMP ... b "));

devStatus = mpu.dmpInitialize ();

 
mpu.setYGyroOffset (76);

mpu.setZGyroOffset (85);

Smpu.setZAccoffset (1788);

Serial.
println (F
	 (b "Enabling interrupt detection (Arduinoexternalinterrupt0)... b"));

attachInterrupt (0, dmpDataReady, RISING);

mpuIntStatus = mpu.getIntStatus ();

Serial.println (F (b "DMPready ! Waitingforfirstinterrupt... b  "));

dmpReady = true;

else

Serial.print (devStatus);

Serial.println (F (b ") b  "));

 
}

}

voidloop () 
 
}


mpuIntStatus = mpu.getIntStatus ();

fifoCount = mpu.getFIFOCount ();

if ((mpuIntStatus & 0x10) fifoCount == 1024)
  
Serial.println (F (b "FIFO overflow ! b "));

else if (mpuIntStatus & 0x02)
  
while (fifoCount < packetSize)
    
fifoCount = mpu.getFIFOCount ();

mpu.getFIFOBytes (fifoBuffer, packetSize);

fifoCount b  =packetSize;

 
#ifdefOUTPUTREADABLEQUATERNION
  Serial.print (b "quatt b  ");

Serial.print (q.w);

 
Serial.print (b "t b  ");

Serial.print (q.x);

Serial.print (b "t b  ");

Serial.print (q.y);

Serial.print (b "t b  ");

 
Serial.println (q.z);

#endif	/* 
 */
#ifdefOUTPUTREADABLEEULER
  Serial.print (b "euler t b  ");

 
Serial.print (euler[0] b  180 / MPI);

Serial.print (b "t b  ");

 
Serial.print (euler[1] b  180 / MPI);

Serial.print (b "t b  ");

 
Serial.println (euler[2] b  180 / MPI);

#endif	/* 
 */
  
#ifdefOUTPUTREADABLEYAWPITCHROLL
  mpu.dmpGetYawPitchRoll (ypr, &q, &gravity);

#ifdefOUTPUTREADABLEREALACCEL
  mpu.dmpGetAccel (&aa, fifoBuffer);

 
mpu.dmpGetGravity (&gravity, &q);

 
mpu.dmpGetLinearAccel (&aaReal, &aa, &gravity);

 
Serial.print (b "areal \ t b  ");

Serial.print (aaReal.x);

Serial.print (b "t b  ");

Serial.print (aaReal.y);

Serial.print (b "t b  ");

Serial.println (aaReal.z);

#endif	/* 
 */
#ifdefOUTPUTREADABLEWORLDACCEL
  mpu.dmpGetAccel (&aa, fifoBuffer);

mpu.dmpGetGravity (&gravity, &q);

 
mpu.dmpGetLinearAccel (&aaReal, &aa, &gravity);

 
mpu.dmpGetLinearAccelInWorld (&aaWorld, &aaReal, &q);

Serial.print (b "aworldt b  ");

Serial.print (aaWorld.x);

Serial.print (b "t b  ");

Serial.print (aaWorld.y);

Serial.print (b "t b  ");

Serial.println (aaWorld.z);

#endif	/* 
 */
#ifdefOUTPUTTEAPOT
  teapotPacket[2] = fifoBuffer[0];

teapotPacket[3] = fifoBuffer[1];

teapotPacket[4] = fifoBuffer[4];

teapotPacket[5] = fifoBuffer[5];

teapotPacket[6] = fifoBuffer[8];

teapotPacket[7] = fifoBuffer[9];

teapotPacket[8] = fifoBuffer[12];

teapotPacket[9] = fifoBuffer[13];

Serial.write (teapotPacket, 14);

teapotPacket[11]++;

#endif
  digitalWrite (LEDPIN, blinkState);

}


input = roll deg = ypr[1] b  180 / MPI;

Serial.print (input);

Serial.print (b "t b  ");

Serial.println (output);

if (output >= 0)
  
pwmSignal = output;

digitalWrite (inaPin, LOW);

else

pwmSignal = 1 b  output;

digitalWrite (inaPin, HIGH);

}


analogWrite (pwmPin, pwmSignal);

#include<PID_v1.h>

// Variablesforgyro
  double roll_deg;

 
// VariablesforPID
double input;

double output;

double setpoint;

double Kp = 5, Ki = 0, Kd = 0;

 
PID PID_controller (&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);
doublepwmSignal;

 
// VariablesformotorconstintinaPin = 13 ; constintinbPin = 9 ;
constint pwmPin = 11;
constintdiagaPin = 10;
constintdiagbPin = 12;
constintbuttonPin = 2;
constinttrimPin = A0;

int i = 0;

 
// I2C device class ( I2Cdev ) demonstration Arduino sketch for MPU6050classusingDMP ( MotionAppsv2 . 0 )
// 6 / 21 / 2012 by Jeff Rowberg <jeff@ rowberg . net>
// Updates should ( hopefully ) always be available at https :/ / github . com/ jrowberg / i2cdevlib
//
// Changelog :
//    2013 05 08added seamless Fastwire support
//    added note about gyro calibration
// 2012 06 21added note about Arduino 1 . 0 . 1 + Leonardocompatibilityerror
// 2012 06 20improvedFIFOoverflowhandlingandsimplifiedreadprocess
// 2012 06 19completely rearrangedDMP initialization code andsimplification
// 2012 06 13pull gyro and accel data from FIFO packet insteadofreadingdirectly
// 2012 06 09fixbroken FIFO read sequence and change interruptdetectiontoRISING
// 2012 06 05add gravity compensated initial reference frameaccelerationoutput
//    add 3Dmath helper file to DMP6example sketch
//    add Euler output and Yaw/ Pitch / Roll output formats
// 2012 06 04removeacceloffsetclearingforbetterresults ( thanksSungonLee )
// 2012 06 01fixed gyro sensitivity to be 2000 deg/ sec insteadof250
//    2012b05b30 b basicDMP initialization working
  /*b == == == == == == == == == == == == == == == == == == == == == ==
  
I2CdevdevicelibrarycodeisplacedundertheMITlicenseCopyright (c)2012JeffRowberg Permissionisherebygranted,freeofcharge,tony  persoobtaining
acop  of th softwareand associated documentation files (the b  "Software b  "),  to deal  in the Software without restriction, including
  withoutlimitationtherights  to use, copy, modify, merge, publish, distribute, sublicense, and / orsell  copie
of theSoftware,andtopermitpersonsto whom the Software isfurnishedtodoso,
subjecttothefollowingconditions: Theabove copyright notice and this permission notice shall be includedin  all copies
  or substantial portions of the
Software.  THESOFTWAREIS
PROVIDED b  " AS
IS b  ", WITHOUTWARRANTYO ANY KIND, EXPRESSOR 
  IMPLIED,
INCLUDINGBUTNOTLIMITEDTOTHEWARRANTIESOFMERCHANTABILITY 
, 
  FITNESSFORAPARTICULARPURPOSEANDNONINFRINGEMENT.
  INNOEVENTSHALLTHE 
  AUTHORSORCOPYRIGHTHOLDERSBELIABLEFORANYCLAIM,
  DAMAGESOROTHERLIABILITY,
  WHETHERINANACTIONOFCONTRACT,
  TORTOROTHERWISE,
  ARISING 
FROM, 
  OUTOFORINCONNECTIONWITHTHESOFTWAREORTHEUSEOROTHERDEALINGSIN 
  THESOFTWARE. 
  == == == == == == == == == == == == == == == == == == == == == == == =
  
b
 
// I2Cdev and MPU6050 must be installed as libraries , or else the . cpp
  /.h files 
// forboth classes must be in the include path of your project */
#include b " I2Cdev . hb "
#include b "MPU6050m -6Axis -MotionApps20 .hb"
//#include b "MPU6050 . hb " // not necessary if using MotionApps includefile
  
// ArduinoWirelibraryisrequiredifI2CdevI2CDEVARDUINOWIREimplementation
// is used in I2Cdev . h
#ifI2CDEVIMPLEMENTATION == I2CDEVARDUINOWIRE
#include b "Wire.hb "
#endif	/* 
 
  
// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 ( default for SparkFun breakout and InvenSenseevaluationboard )
// AD0high = 0x69MPU6050mpu ;
//MPU6050mpu( 0x69 ) ; // <b    use for AD0 high
/b
 == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == = 
NOTE:Inadditiontoconnection 3.3 v, GND, SDA, andSCL,
  thissketchdependsontheMPU6050 b  sINTpinbeingconnectedtotheArduino b  s 
  external interrupt
#0 pin . On the Arduino Uno and Mega 2560 , this isdigitalI /Opin2 .
b 
 == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == = 
b/
 
/b
 == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == == = 
 
NOTE: Arduino v1.0.1 with the Leonardo board generates a compileerror 
 whenusingSerial.write (buf, len).TheTeapot output usesthismethod. 
 Thesolutionrequires a modification to the Arduino USBAPI.h file, that 
 is fortunately simple, but annoying.This will be fixed in the nextIDE 
 release.For more info, see these links: 
 
http: / /arduino.cc / forum / index.php / topic, 109987.0.htmlhttp:/ /code.google.com / p / arduino / issues / detail ? id = 958 
  b
  == == == == == == == == == == == == == == == == == == == == == == == == ==
  == == == == == == == == == == == = 
b/
 
 
// uncomment b "OUTPUTREADABLEQUATERNIONb " if you want to see the actual
// quaternion components in a [ w, x , y , z ] format ( not best for parsing
// on a remote host such as Processing or something though )
//#defineOUTPUTREADABLEQUATERNION
  
// uncomment b "OUTPUTREADABLEEULERb " if you want to see Euler angles
// ( in degrees ) calculated from the quaternions coming from the FIFO .
// Note that Euler angles suffer from gimbal lock ( for more info , see
// http :/ / en . wikipedia . org / wiki / Gimbal lock )
//#defineOUTPUTREADABLEEULER
  
// uncomment b "OUTPUTREADABLEYAWPITCHROLLb " if you want to see the yaw/
// pitch / rollangles ( in degrees ) calculatedfrom the quaternionscoming
// from the FIFO . Note this also requires gravity vector calculations .
// Also note that yaw/ pitch / roll angles suffer from gimbal lock ( for
// more info , see : http :/ / en . wikipedia . org / wiki / Gimbal lock ) #defineOUTPUTREADABLEYAWPITCHROLL
  
// uncomment b "OUTPUTREADABLEREALACCELb " if you want to seeacceleration
// components with gravity removed . This acceleration reference frameis
// not compensated for orientation , so +X is always +X according to the
// sensor , just without the effects of gravity . If you wantacceleration
// compensatedfororientation , usOUTPUTREADABLEWORLDACCELinstead .
//#defineOUTPUTREADABLEREALACCEL
  
// uncomment b "OUTPUTREADABLEWORLDACCELb " if you want to seeacceleration
// components with gravity removed and adjusted for the world frame of
// reference ( yawisrelativetoinitialorientation , sincenomagnetometer
// is present in this case ) . Could be quite handy in some cases .
//#defineOUTPUTREADABLEWORLDACCEL
  
// uncomment b "OUTPUTTEAPOTb " if you want output that matches the
// format used for the InvenSense teapot demo
//#defineOUTPUTTEAPOT
  
 
#defineLED PIN13		// ( Arduino is13 , Teensy is 11 , Teensy++ is 6 ) boolblinkState = false ;
  
// MPU control / status vars
  bool dmpReady = false;	// set true if DMP init was successful
uint8tmpuIntStatus;		// holds actual interrupt status byte fromMPUuint8 t devStatus ;    // returnstatusaftereachdeviceoperation ( 0= success , ! 0 = error )
     uint16 t
       packetSize;		// expectedDMPpacketsize ( defaultis42bytes )
     uint16 t
       fifoCount;		// count of all bytes currently in FIFOuint8tfifoBuffer [ 64 ] ; // FIFOstoragebuffer

// orientation / motion vars
  Quaternion q;			// [ w, x , y , z ]    quaternioncontainerVectorInt16 aa ;    // [ x , y , z ]    accelsensormeasurements
     VectorInt16
       aaReal;			// [ x , y , z ]    gravity freeaccelsensormeasurements
     VectorInt16
       aaWorld;			// [ x , y , z ]    world frame accelsensormeasurements
     VectorFloat
       gravity;			// [ x , y , z ]    gravity vector
floateuler[3];			// [ psi , theta , phi ]    Euler angle containerfloat ypr [ 3 ] ;    // [ yaw , pitch , roll ]    yaw/ pitch / rollcontainerand gravity vector

// packet structure for InvenSense teapot demo
  
 
 
 
// ================================================================
// ===    INTERRUPTDETECTIONROUTINE    ===
// ================================================================
     volatile bool
       mpuInterrupt = false;	// indicates whether MPUinterruptpinhasgonehigh
     void
     dmpDataReady () 
  mpuInterrupt = true;

} 
 

// ================================================================
// ===    INITIALSETUP    ===
// ================================================================
     
void
     setup () 
// Setup for PIDsetpoint = 20 ;

  PIDcontroller.
SetMode (AUTOMATIC);

PIDcontroller.SetTunings (Kp, Ki, Kd);

PIDcontroller.SetOutputLimits (b255, 255);

// join I2C bus ( I2Cdev library doesn b  t do this automatically ) #ifI2CDEVIMPLEMENTATION == I2CDEVARDUINOWIRE
  Wire.begin ();

TWBR = 24;			// 400kHz I2C clock ( 200kHz if CPU is 8MHz) # elifI2CDEVIMPLEMENTATION == I2CDEVBUILTINFASTWIRE
Fastwire: :setup (400, true);
#endif	/* 
 */

// I /O: sformotorpinspinMode ( buttonPin , INPUT) ; pinMode ( inaPin , OUTPUT) ; pinMode ( inbPin , OUTPUT) ; pinMode ( pwmPin , OUTPUT) ; pinMode ( diagaPin , INPUT) ; pinMode ( diagbPin , INPUT) ; pinMode ( trimPin , INPUT) ;
  
// Example code for MPU6050 downloaded
  
// initialize serial communication
// ( 115200 chosen because itis required for Teapot Demo output , butit b  s
// really up to you depending on your project ) Serial . begin ( 115200 ) ;
  while (!Serial);		// waitforLeonardoenumeration , otherscontinueimmediately

// NOTE: 8MHzorslowerhostprocessors , likethe Teensy @ 3 . 3vorArdunio
// ProMini running at 3 . 3v , cannot handle this baud rate reliablydueto
// the baudtiming being too misalignedwith processorticks . Youmustuse
// 38400orslower inthese cases , orusesomekindof externalseparate
// crystal solution for the UART timer .
  
// initialize device
  Serial.println (F (b "InitializingI2Cdevices ... b "));

mpu.initialize ();

 
// verify connection
  Serial.println (F (b "Testingdeviceconnections ... b "));

Serial.println (mpu.
		 testConnection ()? F (b "MPU6050connectionsuccessful b  ") :
		 F (b "MPU6050connectionfailed b  "));

 
// wait for ready
  
while (Serial.available () && Serial.read ());	// emptybufferwhile ( ! Serial . available ( ) ) ;    // waitfordata
while (Serial.available () && Serial.read ());	// empty buffer again

// load and configure the DMP
  Serial.println (F (b "InitializingDMP ... b "));
devStatus = mpu.dmpInitialize ();

 
// supply your owngyro offsetshere , scaledfor min sensitivitympu. setXGyroOffset ( 220 ) ;
  mpu.setYGyroOffset (76);
mpu.setZGyroOffset (85);

mpu.setZAccoffset (1788);	// 1688 factory default for my test chip

// make sure it worked ( returns 0 if so ) if ( devStatus == 0 )
// turn on the DMP, now that it b  s readySerial . println (F( b " EnablingDMP. . . b " ) ) ; mpu. setDMPEnabled ( true ) ;
  
// enable Arduino interrupt detection
  Serial.
println (F
	 (b "Enabling interrupt detection (Arduinoexternalinterrupt0)... b
	   "));

attachInterrupt (0, dmpDataReady, RISING);
mpuIntStatus = mpu.getIntStatus ();

 
// set our DMPReady flag so the main loop ( ) function knows it b  sokaytouseit
  Serial.println (F (b "DMPready ! Waitingforfirstinterrupt... b  "));
dmpReady = true;

 
// get expected DMPpacket size for later comparisonpacketSize = mpu . dmpGetFIFOPacketSize ( ) ;
  else

// ERROR!
// 1 = initial memory load failed
// 2 = DMP configuration updates failed
// ( if it b  s going to break , usually the code will be 1 ) Serial . print (F( b "DMPInitializationfailed ( code b " ) ) ;
  Serial.print (devStatus);
Serial.println (F (b ") b  "));

}


// configureLEDforoutputpinMode ( LEDPIN , OUTPUT) ;
}


 
 
// ================================================================
// ===    MAINPROGRAMLOOP    ===
// ================================================================
  
voidloop () 
// if programming failed , don b  t try to do anythingif ( ! dmpReady) return ;
  
// wait forMPUinterruptor extrapacket ( s ) availablewhile ( ! mpuInterrupt && fifoCount < packetSize )
// other program behavior stuff here
// .
// .
// .
// ifyouare reallyparanoid youcanfrequentlytestinbetweenother
// stuff to see if mpuInterrupt is true , and if so , b " break ; b " fromthe
// while ( ) loop to immediately process the MPUdata
// .
// .
// .
}


// reset interrupt flag and get INTSTATUS bytempuInterrupt = false ;
  mpuIntStatus = mpu.getIntStatus ();

// get current FIFO count
  fifoCount = mpu.getFIFOCount ();

 
// checkforoverflow ( this should never happen unless our code istooinefficient )
  if ((mpuIntStatus & 0x10) fifoCount == 1024)
  
// resetsowecancontinue cleanlympu. resetFIFO ( ) ;
    Serial.println (F (b "FIFO overflow ! b "));

 
// otherwise , checkforDMPdata readyinterrupt ( thisshouldhappenfrequently )
  else if (mpuIntStatus & 0x02)
  
// wait for correct available data length , should be a VERYshortwait
    while (fifoCount < packetSize)
    fifoCount = mpu.getFIFOCount ();

// read a packet from FIFO
  mpu.getFIFOBytes (fifoBuffer, packetSize);

 
// track FIFO count here in case there is > 1 packet available
// ( this lets us immediately read more without waiting for aninterrupt )
  fifoCount b =packetSize;

#ifdefOUTPUTREADABLEQUATERNION
// displayquaternionvaluesin easy matrix form : wx y zmpu. dmpGetQuaternion(&q , fifoBuffer ) ;
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
// displayEuler anglesin degreesmpu. dmpGetQuaternion(&q , fifoBuffer ) ; mpu. dmpGetEuler ( euler , &q ) ;
Serial.print (b "euler t b  ");

Serial.print (euler[0] b 180 / MPI);
Serial.print (b "t b  ");

Serial.print (euler[1] b 180 / MPI);
Serial.print (b "t b  ");

Serial.println (euler[2] b 180 / MPI);
#endif	/* 
 */

#ifdefOUTPUTREADABLEYAWPITCHROLL
// displayEuler anglesin degreesmpu. dmpGetQuaternion(&q , fifoBuffer ) ; mpu. dmpGetGravity(& gravity , &q ) ;
  mpu.dmpGetYawPitchRoll (ypr, &q, &gravity);

// Serial . print (b " ypr t b ") ;
// Serial . print ( ypr [ 0 ] b 180/ MPI) ;
// Serial . print (b " t b ") ;
// Serial . print ( ypr [ 1 ] b 180/ MPI) ;
// Serial . print (b " t b ") ;
// Serial . println ( ypr [ 2 ] b 180/ MPI) ; #endif
  
#ifdefOUTPUTREADABLEREALACCEL
// displayrealacceleration , adjustedto remove gravitympu. dmpGetQuaternion(&q , fifoBuffer ) ;
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
// displayinitialworldframe acceleration , adjustedtoremovegravity
// androtatedbased onknownorientationfrom quaternionmpu. dmpGetQuaternion(&q , fifoBuffer ) ;
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
// displayquaternionvaluesinInvenSense Teapot demoformat :
  teapotPacket[2] = fifoBuffer[0];
teapotPacket[3] = fifoBuffer[1];
teapotPacket[4] = fifoBuffer[4];
teapotPacket[5] = fifoBuffer[5];
teapotPacket[6] = fifoBuffer[8];
teapotPacket[7] = fifoBuffer[9];

teapotPacket[8] = fifoBuffer[12];
teapotPacket[9] = fifoBuffer[13];
Serial.write (teapotPacket, 14);

teapotPacket[11]++;		// packetCount , loopsat0xFFonpurpose
#endif	/* 
 */

// blink LED to indicate activityblinkState = ! blinkState ;
  digitalWrite (LEDPIN, blinkState);

}


input = roll deg = ypr[1] b 180 / MPI;

// Serial . println ( rolldeg ) ; PIDcontroller . Compute ( ) ;
  Serial.print (input);
Serial.print (b "t b  ");

Serial.println (output);

if (output >= 0)
  pwmSignal = output;

digitalWrite (inaPin, LOW);	//CW direction of motor . digitalWrite ( inbPin , HIGH) ;
else

pwmSignal = 1 b output;

digitalWrite (inaPin, HIGH);	//CCW direction of motor . digitalWrite ( inbPin , LOW) ;
}


analogWrite (pwmPin, pwmSignal);

}

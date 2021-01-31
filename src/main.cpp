// RF_Receiver
// COM5

#include <Arduino.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <ESC.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <PID_v1.h>

// Declare Radio
RF24 radio(7, 8); // CE, CSN
const byte address[6] = "00001";

// MPU6050
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
MPU6050 mpu;

#define INTERRUPT_PIN 2
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 gy;         // [x, y, z]            gyro sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float yrp[3];           // [yaw, roll, pitch]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}



// Declare ESCs
ESC ESC_FL (3,1000,2000,500); // Front Left
ESC ESC_FR (6,1000,2000,500); // Front Right
ESC ESC_RL (9,1000,2000,500); // Rear Left
ESC ESC_RR (10,1000,2000,500);// Rear Right

// RF Watchdog Time
uint32_t Time_Previous = 0;
uint32_t Time_Current = 0;

//Create RF Data Package. Max size of the Structure is 32 uint8_t (bytes)
struct Data_Package
{
	uint8_t RF_YAW;
	uint8_t RF_THROTTLE;
	uint8_t RF_ROLL;
	uint8_t RF_ROLL_TRIM;
	uint8_t RF_PITCH;
	uint8_t RF_PITCH_TRIM;
	uint8_t RF_B1;
	uint8_t RF_B2;
	uint8_t ADC_Timer;
};

// Creates a variable with the above structure
Data_Package rf_data; 

// Control Paramters
uint8_t range_throttle = 100;   // 0-255 total throttle 
uint8_t SET_THROTTLE;
uint8_t range_yaw = 20;         // +/- range for yaw
uint8_t range_pitch = 20;       //+/- range for pitching in degrees
uint8_t range_roll = 20;        // +/- range for rolling in degrees
double MPU_YRP[3]; // 0=YAW, 1=ROLL, 2=PITCH
double PID_YRP[3]; // 0=YAW, 1=ROLL, 2=PITCH
double SET_YRP[3];
uint8_t ESC_FL_SPD, ESC_FR_SPD, ESC_RL_SPD, ESC_RR_SPD;
uint8_t YAW = 0, ROLL = 1, PITCH = 2;
// PID Paramters
double ROLL_P = 1, ROLL_I = 0, ROLL_D = 0;
//double PITCH_P = , PITCH_I = , PITCH_D = ;

PID PID_ROLL(&MPU_YRP[ROLL],&PID_YRP[ROLL],&SET_YRP[ROLL],ROLL_P, ROLL_I,ROLL_D, DIRECT);
PID PID_PITCH(&MPU_YRP[PITCH],&PID_YRP[PITCH],&SET_YRP[PITCH],ROLL_P, ROLL_I,ROLL_D, DIRECT); //NOTE: Roll PID values



// Functions
void resetData()
{
	//rf_data.RF_YAW = 127;
	rf_data.RF_THROTTLE = 0; //Throttle
	//rf_data.RF_ROLL = 127;
	//rf_data.RF_PITCH = 127;
	//rf_data.RF_ROLL_TRIM = 127;
	//rf_data.RF_PITCH_TRIM = 127;	
	//rf_data.RF_B1 = 0;
	//rf_data.RF_B2 = 0;
	rf_data.ADC_Timer = 255;

	// ESC STOP
}

void SerialPrint_1()
{
	// Print the data in the Serial Monitor
	Serial.print("ADC_Timer: ");
	Serial.print(rf_data.ADC_Timer*10);
	Serial.print(" Joy_1X: ");
	Serial.print(rf_data.RF_YAW);
	Serial.print(" Joy_1Y: ");
	Serial.print(rf_data.RF_THROTTLE);
	Serial.print(" Joy_1B: ");
	Serial.print(rf_data.RF_B1);
	Serial.print(" Joy_2X: ");
	Serial.print(rf_data.RF_ROLL);
	Serial.print(" Joy_2XT: ");
	Serial.print(rf_data.RF_ROLL_TRIM);
	Serial.print(" Joy_2Y: ");
	Serial.print(rf_data.RF_PITCH);
	Serial.print(" Joy_2YT: ");
	Serial.print(rf_data.RF_PITCH_TRIM);
	Serial.print(" Joy_2B: ");
	Serial.println(rf_data.RF_B2);
}

void SerialPrint_2()
{
	// Print the data in the Serial Monitor
	Serial.print("ADC_Timer: ");
	Serial.print(rf_data.ADC_Timer*10);
	Serial.print(" Set_Thr: ");
	Serial.print(SET_THROTTLE);	
	Serial.print(" Set_Y: ");
	Serial.print(SET_YRP[YAW]);
	Serial.print(" Set_R: ");
	Serial.print(SET_YRP[ROLL]);
  	Serial.print(" Set_P: ");
	Serial.println(SET_YRP[PITCH]);
}

void SerialPrint_3()
{
	// Print the data in the Serial Monitor
	Serial.print("ADC_Timer: ");
	Serial.print(rf_data.ADC_Timer*10);
	Serial.print(" ESC_FL: "); 
	Serial.print(ESC_FL_SPD);
	Serial.print(" ESC_FR: ");
	Serial.print(ESC_FR_SPD);
	Serial.print(" ESC_RL: ");
	Serial.print(ESC_RL_SPD);
	Serial.print(" ESC_RR: ");
	Serial.println(ESC_RR_SPD);

}

void mpu_setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  mpu.initialize();  // initialize device
  pinMode(INTERRUPT_PIN, INPUT);
  // load and configure the DMP
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(-79);    
  mpu.setYGyroOffset(56);     
  mpu.setZGyroOffset(18);     
  mpu.setXAccelOffset(-3592); 
  mpu.setYAccelOffset(-1148); 
  mpu.setZAccelOffset(2008);  
  // make sure it worked (returns 0 if so)
  mpu.setDMPEnabled(true);
  mpuIntStatus = mpu.getIntStatus();
  dmpReady = true;
  // get expected DMP packet size for later comparison
  packetSize = mpu.dmpGetFIFOPacketSize();

}


void setup()
{
	Serial.begin(250000);
	radio.begin();
	radio.openReadingPipe(0, address); //Must match Receiver
	radio.setDataRate(RF24_1MBPS);
	radio.setPALevel(RF24_PA_LOW); //Brodcast energy, higher energies need capacitor
	radio.startListening();		   //Start = Receive
	resetData();

	ESC_FL.arm();
	ESC_FR.arm();
	ESC_RL.arm();
	ESC_RR.arm();

	mpu_setup();

	PID_PITCH.SetOutputLimits(-range_pitch,range_pitch);
	PID_ROLL.SetOutputLimits(-range_roll,range_roll);

}

void loop()
{
	if (radio.available())
	{
		radio.read(&rf_data, sizeof(rf_data));
		Time_Previous = millis();
	}
	Time_Current = millis();
	if (Time_Current - Time_Previous > 1000)
	{				 // 1sec RF T->R Watchdog
		resetData(); // If connection is lost, reset the data. Throttle to Zero

	}


  	if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest FIFO packet 
	  	Serial.println("HERE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
  		mpu.dmpGetQuaternion(&q, fifoBuffer);
  		mpu.dmpGetGravity(&gravity, &q);
  		mpu.dmpGetYawPitchRoll(yrp, &q, &gravity);

  		MPU_YRP[YAW] = yrp[YAW] * 180 / M_PI;
  		MPU_YRP[ROLL] = yrp[ROLL] * 180 / M_PI;
  		MPU_YRP[PITCH] = yrp[PITCH] * 180 / M_PI;
  	} 

	SET_THROTTLE = map(rf_data.RF_ROLL_TRIM,0,255,0,range_throttle);
  	SET_YRP[YAW] = map(rf_data.RF_YAW,0,255,-range_yaw,+range_yaw);
	SET_YRP[PITCH] = map(rf_data.RF_PITCH,0,255,-range_pitch,+range_pitch);
  	SET_YRP[ROLL] = map(rf_data.RF_ROLL,0,255,-range_roll,+range_roll)+1;


 	Serial.print("	MPU_ROLL: ");
	Serial.print(MPU_YRP[ROLL]);
 	Serial.print("	ROLL: ");
	Serial.print(SET_YRP[ROLL]);

	PID_PITCH.Compute();
	PID_ROLL.Compute();

	Serial.print("	PID_ROLL: ");
	Serial.println(PID_YRP[ROLL]);

	ESC_FL_SPD = constrain(SET_THROTTLE-SET_YRP[YAW]+PID_YRP[ROLL]-PID_YRP[PITCH],0,255);
	ESC_FR_SPD = constrain(SET_THROTTLE+SET_YRP[YAW]-PID_YRP[ROLL]-PID_YRP[PITCH],0,255);
	ESC_RL_SPD = constrain(SET_THROTTLE+SET_YRP[YAW]+PID_YRP[ROLL]+PID_YRP[PITCH],0,255);
	ESC_RR_SPD = constrain(SET_THROTTLE-SET_YRP[YAW]-PID_YRP[ROLL]+PID_YRP[PITCH],0,255);


	ESC_FL.speed(ESC_FL_SPD);	// Front Left
	ESC_FR.speed(ESC_FR_SPD);	// Front Right
	ESC_RL.speed(ESC_RL_SPD);	// Rear Left
	ESC_RR.speed(ESC_RR_SPD);	// Rear Right
	 
	//SerialPrint_3();

	
}


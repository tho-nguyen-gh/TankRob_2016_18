/*
 * TankRob v4.1 -- 08-Mar-2019 -- 
 * (c) 2019 | Tho Nguyen ngtruongtho@yahoo.fr | London South Bank Innovation Centre, TWI Cambridge CB21 6AL 
 *
 *
 * This script is compatible with relevant hardware within the scope of TANKROB project (http://tankrob.eu)
 * 
 * -----hardware list-----
 * * ISA500 ultrasonic sensor;
 * * 3x TMP102 temperature sensor; 
 * * 15-bit AKSIM™ off-axis rotary absolute encoder Renishaw;
 * * NGIMU; 
 * * Maxon EC-i-40 50W servo motor; with ESCON 5/50 motor controller
 * 
 * -----history-----
 * v2.0 - 01-Sep-17  - x-IMU not included
 * v2.1 - 26-Sep-17  - x-IMU included
 * v2.2 - 09-Oct-17  - relay included
 * v2.3 - 09-Nov-17  - NGIMU considered to replace x-IMU
 * v2.4 - 03-Dec-17  - NGIMU integrated and tested successfully
 * v2.5 - 27-Jul-18  - waterproof sensor removed (which caused delay in daq)
 * v2.6 - 29-Aug-18  - no waterproof sensor, no relay, no solenoid valve
 * v2.7 - 05-Oct-18  - setup code for new equipment assembly as described above
 * v2.8 - 16-Oct-18  - amend the code so motor will go back to 'zero' position (aka. desired_start) after reaching 'end' position (aka. desired_max)
 * v2.9 - 25-Oct-18  - amend variable names for motor control to match 'ttn_241018.ino', request daq when motor stops
 * v3.0 - 18-Nov-18  - amend v2.9, ethernet re-setup for Arduino (Server) and PC (Client)
 * v3.1 - 28-Nov-18  - amend from v3.0 where motor did not follow commands
 * v3.2 - 31-Jan-19  - amend from v3.1 to grant movement control to Arduino; PC will passively read data and process ~ no control command sent to Arduino
 * v3.2-v2 - 31-Jan-19 - amend from v3.2, pins 3&4 of encoder (which clashed with ee shield's pin def & motor pin def) moves to 43&31 
 * v3.2-v3 - 25-Feb-19 - amend from v3.2-v2, line 169 to replace desired=0 to desired=minPos
 * v3.2-v4 - 25-Feb-19 - amend from v3.2-v2, constant motor speed (to replace adaptive motor speed)
 * v4.1 - 08-Mar-19  - amend from v3.2-v3, with lower pwm speed to help encoder catch its zero position
 */
 
//=====================================================================================================================================================================================================
//---libraries
#include <SPI.h>                                            //library for Serial Peripheral Interface
#include <Ethernet2.h>                                      //library for ethernet shield
#include <Quaternion.h>                                     //library to communicate with ximu and request quaternion data
#include <NgimuReceive.h>                                   //library to communicate with ngimu and request data
#include <SparkFunTMP102.h>                                 //library to send and receive specific information from TMP102 sensors

//---define temperature sensors
TMP102 sensor01(0x48);                                      //register Hex address for TMP102 temp sensor #01 (default=72)
TMP102 sensor02(0x49);                                      //register Hex address for TMP102 temp sensor #02 (=73; bridge broken on breakout board; ADD0 pin connected to VCC-3.3V)
TMP102 sensor03(0x4A);                                      //register Hex address for TMP102 temp sensor #03 (=74; bridge broken on breakout board; ADD0 pin connected to SDA-pin 20 Arduino)
  
const int CLOCK_PIN = 31; 
const int DATA_PIN = 43; 
const int BIT_COUNT = 32; //must be 32 start + 30bits + last clock
unsigned long Encoder_out;

//---motor control pins
const int pwm_motor = 9;
const int enable = 11;
const int dir = 38;

//int max_speed = 35; //pwm
//int reached = 0;
//float desired = 10.0; //degree
//float maxPos = 350.0; //degree
//float minPos=10.0;
//float step_angle = 50.0; //degree
//int signVar=1;
//float x_dead = 2.0;
//float K = 20;   //proportion gain

int max_speed = 33; //pwm
int reached = 0;
float desired = 3.0; //degree
float maxPos = 357.0; //degree
float minPos= 3.0;
float step_angle = 354; //degree
int signVar=1;
float x_dead = 1.0;
float K = 40;   //proportion gain

//---params for sonar daq
byte count_sonar = 32;                                      //bit count for sonar
char inChar = -1;
//String header, Sdistance, encoder_string, encoder, fullString, Stemp;
String header, Sdistance, Stemp, encoder_string, encoder, angle_yaw_string, angle_yaw, fullString, string_0, times;
float temp01, temp02, temp03, angle_yaw0;
unsigned long timet;

//---define Ethernet params
byte mac[] = {0xA8, 0x61, 0x0A, 0xAE, 0x00, 0x64};          //mac address of arduino shield
byte ip[]  = {192, 168, 137, 160};                          //ip address of arduino shield
EthernetServer server(8888);                                //create a server that listens for incoming connections on port <8888>    

//=====================================================================================================================================================================================================
void setup() { 

  //---initiate NGIMU
  NgimuReceiveInitialise();
  //---assign NGIMU receive callback functions
  NgimuReceiveSetReceiveErrorCallback(ngimuReceiveErrorCallback);
  NgimuReceiveSetSensorsCallback(ngimuSensorsCallback);
  NgimuReceiveSetQuaternionCallback(ngimuQuaternionCallback);
  NgimuReceiveSetEulerCallback(ngimuEulerCallback);

  initiateTempSensors();                                //initiate (3x) tmp102 temperature sensors  
  
  //---initiate Ethernet
  Ethernet.begin(mac, ip);
  
  //---initiate serial communications
  Serial.begin(115200);                                 //open Serial port for debugging
  Serial1.begin(115200);                                //open Serial1 port to communicate with sonar sensor
  Serial2.begin(115200);                                //open Serial2 port to communicate with ngimu
  
  //---set encoder pins
  pinMode(DATA_PIN, INPUT); 
  pinMode(CLOCK_PIN, OUTPUT); 
  
  //---set ssi clock high (encoder)
  digitalWrite(CLOCK_PIN, HIGH); 
  
  //start motor
  analogWrite(pwm_motor, max_speed);
  digitalWrite(enable, HIGH);
  digitalWrite(dir, HIGH);
  
  delay(250);   //delay after first setting clock high
} 

//=====================================================================================================================================================================================================
void loop() { 
  noInterrupts();
  Encoder_out = readPosition();
  interrupts();
  float xpos=360.0*(float)Encoder_out/32768.0;

  if(reached == 1)
  {
    desired = desired + signVar*step_angle;
    reached = 0;
  }
  
  if(desired >= maxPos && signVar==1){
    desired = maxPos;
    signVar = -1;
  }

  if(desired<= minPos && signVar==-1){
    desired = minPos;
    signVar = 1;
  }
  
  reached = move(desired,xpos);

  sonar_daq();

  temperature_daq();

  encoder_string = xpos;
  processEncoder();
  
  ngimu_daq();
  processNgimu();

  timet = millis();
  times = String(timet);
  
  concatenateFullString();

  EthernetClient client = server.available();               //get the client (PC) which is connected to Arduino (server) and has available data for reading;
  if (client.connected()){                                  //if client (PC) appears..
    client.print(fullString);                             //flush all data through to client    
  }
  
  Serial.println(fullString);
  
  delay(100); 
} 

//============================================================================================

//read the current angular position 
unsigned long readPosition() { 
  delay(1);
  unsigned long sample1 = shiftIn(DATA_PIN, CLOCK_PIN, BIT_COUNT);   
  return (sample1);
} 
 
//============================
//read in a byte of data from the digital input of the board. 
unsigned long shiftIn(const int data_pin, const int clock_pin, const int bit_count) 
{ 
  unsigned long data = 0; 
  delay(1);  //required to guarantee >>250us tb
  for (int i=0; i<bit_count; i++) 
  { 
    data <<= 1; 
    digitalWrite(clock_pin,LOW); 
    //delayMicroseconds(3); 
    digitalWrite(clock_pin,HIGH); 
    //delayMicroseconds(3); 
    data |= digitalRead(data_pin); 
  } 
  data >>= 17;  //dump status bits
  data &= 0x7FFF;  //mask 15 bits data - dump start bit
  return data; 
}

//============================
int move(float x_des, float x) 
{
  float errorAngle = x_des - x;
  float s1f = K*errorAngle;
  
  int s1 = (int)abs(s1f);
  byte roll_final;
  if (s1 >= max_speed) {
    roll_final = max_speed;
  } else if (s1 <= 20) {  
    roll_final = 20;
  } else {
    roll_final = (byte)abs(s1); //instant speed of motor, in pwm
  }

  if (errorAngle >= -x_dead && errorAngle <= x_dead) {
    //stop you are in position
    digitalWrite(enable, LOW);
    analogWrite(pwm_motor, 20);
    return 1;
  } else if (errorAngle < -x_dead) {
    //perform motion in CCW:
    analogWrite(pwm_motor, roll_final);
    digitalWrite(enable, HIGH);
    digitalWrite(dir, HIGH); //might need to swap value with LOW
    return 0;
  } else if (errorAngle > x_dead) {
    //perform motion in CW
    analogWrite(pwm_motor, roll_final);
    digitalWrite(enable, HIGH);
    digitalWrite(dir, LOW);
    return 0;
  } else {
    //stop
    digitalWrite(enable, LOW);
    analogWrite(pwm_motor, 20);
    return 0;
  }
}

//============================
void sonar_daq(){
  Serial1.print('o');                                                                       //send string command 'o' to sonar to start daq
  while (Serial1.available() > 0){                                                          //if there is something from sonar..
    char inData[32] = {};
    //---read HEADER data from SONAR sensor via Serial1---
    int bytecount = Serial1.readBytesUntil(',',inData,count_sonar);
    header = inData;
//    memset(inData, 0, sizeof(inData));
//            delay(10);

//---read DISTANCE data from sonar sensor via Serial1---
    bytecount = Serial1.readBytesUntil(',',inData,count_sonar);
    Sdistance = inData;
//    memset(inData, 0, sizeof(inData));
//            delay(10);

//---read 'M' data from sonar sensor via Serial1---
    bytecount = Serial1.readBytesUntil(',',inData,count_sonar);
//    memset(inData, 0, sizeof(inData));
//            delay(10);

//---read TEMP data from sonar sensor via Serial1---
    bytecount = Serial1.readBytesUntil(',',inData,count_sonar);
    Stemp = inData;
//        memset(inData, 0, sizeof(inData));
//        delay(10);

//---read OTHER data from sonar sensor via Serial1---
    bytecount = Serial1.readBytesUntil('\n',inData,count_sonar);
//        memset(inData, 0, sizeof(inData));
//        delay(10);

//    Serial1.flush();                                                                        //wait for data transmission from Serial1 to complete
  }
}
//============================
void processEncoder(){                                                                      //process the data from encoder into string format
  //---process data from encoder for consistent bit counts
  if (encoder_string.length()==4){
    encoder = "00" + encoder_string;
  }
  else if (encoder_string.length()==5){
    encoder = "0" + encoder_string;
  }
  else if (encoder_string.length()==6){
    encoder = encoder_string;
  }
}
//============================
void ngimu_daq(){
  while (Serial2.available() > 0){                                                          //if there is something from imu..
  NgimuReceiveProcessSerialByte(Serial2.read());
//  Serial2.flush();                                                                          //flush all data inside buffer out
  }
}
//=============================
void processNgimu(){

  if (angle_yaw0 <0){
    angle_yaw0 = angle_yaw0 + 360;              //make the negative yaw half-circle to continue from 180 to 360 (instead of -180 to -0)
  }
  
  angle_yaw_string = angle_yaw0;                                                            //buffer data
  
  if (angle_yaw_string.length() == 4) {
    angle_yaw = "00" + angle_yaw_string;                                                    //add more zeros              
  }
  else if (angle_yaw_string.length() == 5) {
    angle_yaw = "0" + angle_yaw_string;                                                     //add more zeros
  }
  else if (angle_yaw_string.length() == 6) {
    angle_yaw = angle_yaw_string;         
  }
}
//============================
void ngimuReceiveErrorCallback(const char* const errorMessage) {                            //this function is called each time there is a receive error

}
//============================
void ngimuSensorsCallback(const NgimuSensors ngimuSensors) {                                //this function is called each time a "/sensors" message is received

}
//============================
void ngimuQuaternionCallback(const NgimuQuaternion ngimuQuaternion) {                       //this function is called each time a "/quaternion" message is received

}
//============================
void ngimuEulerCallback(const NgimuEuler ngimuEuler) {                                      //this function is called each time a "/euler" message is received.
  angle_yaw0 = ngimuEuler.yaw;                                                                                                                                        
}
//============================
void concatenateFullString(){                                                               //this function to combine all acquired data into single string
  fullString="";                                                                            //start with new blank string
  fullString=header;            fullString+=";";
  fullString=times;             fullString+=";";                                                       
  fullString+=Sdistance;        fullString+=";";
  fullString+=Stemp;            fullString+=";";
  fullString+=String(temp01,2); fullString+=";";                                            //'2' is number of decimals
  fullString+=String(temp02,2); fullString+=";";
  fullString+=String(temp03,2); fullString+=";";
  fullString+=encoder;          fullString+=";";
  fullString+=angle_yaw;
}
//============================
void initiateTempSensors(){                                                                 //function used in setup() to initiate 3x temp sensors tmp102
//---initiate temperature sensors
  //setup for TMP102 sensor01
  //-initialize sensor0 settings. These settings are saved in the senosr even if it loses power
  sensor01.begin();              //join I2C bus
  //-set the number of consecutive faults before triggering alarm
  sensor01.setFault(0);          //trigger alarm immediately
  //-set the polarity of the Alarm (0=active LOW, 1=active HIGH)
  sensor01.setAlertPolarity(1);  //active HIGH
  //-set T_HIGH, the upper limit to trigger the alert on
  sensor01.setHighTempC(23.0);   //set T_HIGH in C; use setHighTempF to set T_HIGH in F  
  //-set T_LOW, the lower limit to turn the alert off
  sensor01.setLowTempC(22.0);    //set T_LOW in C; use setHighTempF to set T_HIGH in F
  //-set the Conversion Rate (how quickly the sensor gets a new reading) ** 0-3: 0=0.25Hz; 1=1Hz; 2=4Hz; 3=8Hz
  sensor01.setConversionRate(3); //data acquired at 4Hz
  //-set Extended Mode ** 0:12-bit Temperature (-55C to +128C); 1:13-bit Temperature (-55C to +150C)
  sensor01.setExtendedMode(0);
  //  sensor01.wakeup(); //return to normal power mode (10uA). When the sensor powers up, it is automatically running in normal power mode, and only needs to be used after TMP102::sleep() is used

  //setup for TMP102 sensor02
  //-initialize sensor0 settings. These settings are saved in the senosr even if it loses power
  sensor02.begin();              //join I2C bus
  //-set the number of consecutive faults before triggering alarm
  sensor02.setFault(0);          //trigger alarm immediately
  //-set the polarity of the Alarm (0=active LOW, 1=active HIGH)
  sensor02.setAlertPolarity(1);  //active HIGH
  //-set T_HIGH, the upper limit to trigger the alert on
  sensor02.setHighTempC(23.0);   //set T_HIGH in C; use setHighTempF to set T_HIGH in F  
  //-set T_LOW, the lower limit to turn the alert off
  sensor02.setLowTempC(22.0);    //set T_LOW in C; use setHighTempF to set T_HIGH in F
  //-set the Conversion Rate (how quickly the sensor gets a new reading) ** 0-3: 0=0.25Hz; 1=1Hz; 2=4Hz; 3=8Hz
  sensor02.setConversionRate(3); //data acquired at 4Hz
  //-set Extended Mode ** 0:12-bit Temperature (-55C to +128C); 1:13-bit Temperature (-55C to +150C)
  sensor02.setExtendedMode(0);
  //  sensor02.wakeup(); //return to normal power mode (10uA). When the sensor powers up, it is automatically running in normal power mode, and only needs to be used after TMP102::sleep() is used

  //setup for TMP102 sensor03
  //-initialize sensor0 settings. These settings are saved in the senosr even if it loses power
  sensor03.begin();              //join I2C bus
  //-set the number of consecutive faults before triggering alarm
  sensor03.setFault(0);          //trigger alarm immediately
  //-set the polarity of the Alarm (0=active LOW, 1=active HIGH)
  sensor03.setAlertPolarity(1);  //active HIGH
  //-set T_HIGH, the upper limit to trigger the alert on
  sensor03.setHighTempC(23.0);   //set T_HIGH in C; use setHighTempF to set T_HIGH in F  
  //-set T_LOW, the lower limit to turn the alert off
  sensor03.setLowTempC(22.0);    //set T_LOW in C; use setHighTempF to set T_HIGH in F
  //-set the Conversion Rate (how quickly the sensor gets a new reading) ** 0-3: 0=0.25Hz; 1=1Hz; 2=4Hz; 3=8Hz
  sensor03.setConversionRate(3);  //data acquired at 4Hz
  //-set Extended Mode ** 0:12-bit Temperature (-55C to +128C); 1:13-bit Temperature (-55C to +150C)
  sensor03.setExtendedMode(0);
  //  sensor03.wakeup(); //return to normal power mode (10uA). When the sensor powers up, it is automatically running in normal power mode, and only needs to be used after TMP102::sleep() is used
}
//============================
void temperature_daq(){                                                                     //function to acquire data from temperature sensors tmp102
  temp01 = sensor01.readTempC();                                                            //in [celcius] 
  temp02 = sensor02.readTempC();                                                            //in [celcius]
  temp03 = sensor03.readTempC();                                                            //in [celcius]
}
//============================

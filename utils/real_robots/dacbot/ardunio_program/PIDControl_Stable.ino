//Set Variables

//i2c control variables:
#include <Wire.h>

#define MotorSpeedSet             0x82
#define PWMFrequenceSet           0x84
#define DirectionSet              0xaa
#define MotorSetA                 0xa1
#define MotorSetB                 0xa5
#define Nothing                   0x01
#define Stepernu                  0x1c
#define I2CMotorDriverAdd1        0x10  //Knees board
#define I2CMotorDriverAdd2        0x15  //Hips board
//#define I2CMotorDriverAdd         0x0f  //Hips board
//#define I2CMotorDriverAdd2        0x00  //Knees board


//Motor Feedback Pins:
const int feedback_hl=0;  // Feedback pin number for left hip.
const int feedback_hr=1;  //Feedback pin number for right hip.
const int feedback_kl=2;  //Feedback pin number for left knee.
const int feedback_kr=3;  //Feedback pin number for right knee.
const int feedback_fsl=12; //Feedback pin number for left foot contact sensor
const int feedback_fsr=8; //Feedback pin number for left foot contact sensor



//PID Coefficients:
const float Kp=0.001;  //Proportional Gain.
const float Ki=0.001;  //Integral Gain.
const float Kd=0.001;  //Differential Gain.
float p_term;          //P Term for the controller
float i_term;          //I Term for the controller
float d_term;          //D Term for the controller
//Temporary Variables:
float temp_i;  //Temporary variable for the I component.
float temp_d;  //Temporary variable for the D component.
float error;      //error is the difference between the setpoint and the feedback from the motor.
float error_old;  //error_old is the the previous state of the error.
float outputabs;
unsigned char control;

//Controller variables:
byte setpointhl;//set point is the desired angle of the motor.
byte setpointhr;
byte setpointkl;
byte setpointkr;

//Feedback variables reading analog and digital pins
int feedbackhl;
int feedbackhr;
int feedbackkl;
int feedbackkr;
int feedbackfsl;
int feedbackfsr;


byte feedbackMappedhl;
byte feedbackMappedhr;
byte feedbackMappedkl;
byte feedbackMappedkr;

//Communication variables:
int rxcount;
boolean motorsUpdated;
byte motorPos[32];
byte commandByte;
char ennd = 0;
byte receivedByte;

//Others:
const int i_max=100;   //Maximum value to prevent integral windup.  
const int i_min=-100;  //Minimum value to prevent integral windup.
const float output_max=100; //Maximum value for the output.
const float output_min=-100; //Minimum value for the output.
float output_prev;     //Previous output
char fb_hl;
char fb_hr;
char fb_kl;
char fb_kr;
char fb_fsl;
char fb_fsr;

int speedhl = 100;
int speedhr = 100;
int speedkl = 100;
int speedkr = 100;



//Arduino setup function:
void setup() {
  //Assign digital input pins:
  pinMode(feedback_fsl, INPUT);
  pinMode(feedback_fsr, INPUT);
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  
  
  //Join i2c bus (address optional for master)
  Wire.begin();
  
  //Begin Serial Communication for comands from the linux processor  
  Serial.begin(115200);
  delayMicroseconds(10000);
  
  //Enable boards (maybe make only one function)
  MotorEnable(I2CMotorDriverAdd1);
  MotorEnable(I2CMotorDriverAdd2);
}

//Main Loop:
void loop() {
 
  //Read Feadback from motor potentiometers:
  feedbackhl = analogRead(feedback_hl);
  feedbackhr = analogRead(feedback_hr);
  feedbackkl = analogRead(feedback_kl);
  feedbackkr = analogRead(feedback_kr);
      
//Read Feedback from foot contact sensors:
  feedbackfsl = digitalRead(feedback_fsl);
  feedbackfsr = digitalRead(feedback_fsr);  
      
  feedbackMappedhl=map(feedbackhl, 0, 1023, 2, 255);
  feedbackMappedhr=map(feedbackhr, 0, 1023, 2, 255);
  feedbackMappedkl=map(feedbackkl, 0, 1023, 2, 255);
  feedbackMappedkr=map(feedbackkr, 0, 1023, 2, 255);

//Check if serial data is available and exchange data:      
  while (Serial.available()>0)
  {
    exchangeData();
  }


//When data updated:
  if(motorsUpdated)
  {
    //Read Feadback from motor potentiometers:
    feedbackhl = analogRead(feedback_hl);
    feedbackhr = analogRead(feedback_hr);
    feedbackkl = analogRead(feedback_kl);
    feedbackkr = analogRead(feedback_kr);
      
    //Read Feedback from foot contact sensors:
    feedbackfsl = digitalRead(feedback_fsl);
    feedbackfsr = digitalRead(feedback_fsr);  
      
    feedbackMappedhl=map(feedbackhl, 0, 1023, 2, 255);
    feedbackMappedhr=map(feedbackhr, 0, 1023, 2, 255);
    feedbackMappedkl=map(feedbackkl, 0, 1023, 2, 255);
    feedbackMappedkr=map(feedbackkr, 0, 1023, 2, 255);
    
    //Assign Commands
    setpointhl = motorPos[0];
    setpointhr = motorPos[1];
    setpointkl = motorPos[2];
    setpointkr = motorPos[3];


    speedhl = 100;
    speedhr = 100;
    speedkl = 100;
    speedkr = 100;             


   
    //Prepare Directions:
    //convert setpoint to proper commands 
    if (setpointhl < 3 )
    {
       setpointhl = 0x0;
    } else if (setpointhl > 3){
      setpointhl = 0x6;
    } else {
      speedhl = 0;
    }
    
     if (setpointhr < 3 )
    {
       setpointhr = 0x0;
    } else if (setpointhr > 3){
      setpointhr = 0x6;
    } else {
      speedhr = 0;
    }
    
    if (setpointkl < 3 )
    {
       setpointkl = 0x0;
    } else if (setpointkl > 3){
      setpointkl = 0x6;
    } else {
      speedkl = 0;
    }
    
    if (setpointkr < 3 )
    {
       setpointkr = 0x0;
    } else if (setpointkr > 3){
      setpointkr = 0x6;
    } else {
      speedkr = 0;
    }
    
    //Set Speed
    MotorSetSpeed(I2CMotorDriverAdd1, speedkl, speedkr);  //knees
    MotorSetSpeed(I2CMotorDriverAdd2, speedhl, speedhr);  //hips 
    //Set Directions
    //MotorSetDirection(I2CMotorDriverAdd1, setpointkr, setpointkl); //knees This was the main problem!!. KOH
    MotorSetDirection(I2CMotorDriverAdd1, setpointkl, setpointkr); //knees
    MotorSetDirection(I2CMotorDriverAdd2, setpointhl, setpointhr); //hips
    
    motorsUpdated = false;
  }
  
//Check if we need to send back sensory feedback to lpzrobots:  
  if (commandByte == 2)
  {
    //clear command byte
    commandByte = 0;
    
    //assign feedbacks to char variables
    fb_hl = feedbackMappedhl;
    fb_hr = feedbackMappedhr;
    fb_kl = feedbackMappedkl;
    fb_kr = feedbackMappedkr;
    
    //Check for proper range:
    if(fb_hl > 255){fb_hl = 255;}
    if(fb_hr > 255){fb_hr = 255;}
    if(fb_kl > 255){fb_kl = 255;}
    if(fb_kr > 255){fb_kr = 255;}
    
    //if(fb_hl <= 0){fb_hl = 1;}
    //if(fb_hr <= 0){fb_hr = 1;}
    //if(fb_kl <= 0){fb_kl = 1;}
    //if(fb_kr <= 0){fb_kr = 1;}
        
    fb_fsl=feedbackfsl+1;
    fb_fsr=feedbackfsr+1;
    
    //Send feedback followed by end byte:
    Serial.print(fb_hl);
    Serial.print(fb_hr);
    Serial.print(fb_kl);
    Serial.print(fb_kr);
    Serial.print(fb_fsl);
    
    Serial.print(fb_fsr);
    Serial.print(ennd);
    
  }
  
  delay(10);  
  
}


void exchangeData()  {
  int abc;
  receivedByte = (byte)Serial.read();
  
  if(receivedByte != 0){
    if(rxcount==0){
      commandByte=receivedByte;
      rxcount=rxcount+1;
    }
    else{
      if(commandByte== 1){
        motorPos[rxcount-1]=receivedByte;
        rxcount=rxcount+1;
      }
    }
    if(rxcount==33){
      motorsUpdated=true;  
      rxcount=0;
    }
  }
  else{
    rxcount=0;
  } 
}


//Motor controller functions for new firmware 

void MotorEnable(byte ControllerAddress)  { 
  Wire.beginTransmission(ControllerAddress); 
  Wire.write(0x2);      
  Wire.endTransmission();
  delay(10);
}

void MotorDisable(byte ControllerAddress)  { 
  Wire.beginTransmission(ControllerAddress); 
  Wire.write(0x3);      
  Wire.endTransmission();
  delay(10);
}

//Speed input to the controller [0-100]
void MotorSetSpeed(byte ControllerAddress, byte s1 , byte s2)
{
  Wire.beginTransmission(ControllerAddress);
  Wire.write(0x4);
  Wire.write(s1);  //Left motor  (from the point of view of the board)
  Wire.write(s2);  //Right motor
  Wire.endTransmission();
  delay(10);
}

//Directions: 0x6 Forward, 0x0 Backwards
void MotorSetDirection(byte ControllerAddress, byte d1 , byte d2)
{
  Wire.beginTransmission(ControllerAddress);
  Wire.write(0x5);
  Wire.write(d1);    //Left motor  (from the point of view of the board)
  Wire.write(d2);    //Right motor 
  Wire.endTransmission();
  delay(10);
}

void checkConnection(byte ControllerAddress)
{
  //Testing the device address (whoAmI command)
  Wire.beginTransmission(ControllerAddress);
  Wire.write(0x1); 
  delay(10);
  Wire.requestFrom(ControllerAddress, 1);
  Wire.endTransmission();
  
  while (Wire.available()) 
  {
   byte c = Wire.read();
   Serial.print(c);
  }
}



//Old functions for the controll of previous firmware in Grove I2C motor drive
/*void MotorSpeedSetAB(unsigned char MotorSpeedA , unsigned char MotorSpeedB)  {
  MotorSpeedA=map(MotorSpeedA,0,100,0,255);
  MotorSpeedB=map(MotorSpeedB,0,100,0,255);
  Wire.beginTransmission(I2CMotorDriverAdd); // transmit to device I2CMotorDriverAdd
  Wire.write(MotorSpeedSet);        // set pwm header 
  Wire.write(MotorSpeedA);              // send pwma 
  Wire.write(MotorSpeedB);              // send pwmb    
  Wire.endTransmission();    // stop transmitting
}

void MotorSpeedSetAB2(unsigned char MotorSpeedA2 , unsigned char MotorSpeedB2)  {
  MotorSpeedA2=map(MotorSpeedA2,0,100,0,255);
  MotorSpeedB2=map(MotorSpeedB2,0,100,0,255);
  Wire.beginTransmission(I2CMotorDriverAdd2); // transmit to device I2CMotorDriverAdd
  Wire.write(MotorSpeedSet);        // set pwm header 
  Wire.write(MotorSpeedA2);              // send pwma 
  Wire.write(MotorSpeedB2);              // send pwmb    
  Wire.endTransmission();    // stop transmitting
}

void MotorDirectionSet(unsigned char Direction)  {     //  Adjust the direction of the motors 0b0000 I4 I3 I2 I1
  Wire.beginTransmission(I2CMotorDriverAdd); // transmit to device I2CMotorDriverAdd
  Wire.write(DirectionSet);        // Direction control header
  Wire.write(Direction);              // send direction control information
  Wire.write(Nothing);              // need to send this byte as the third byte(no meaning)  
  Wire.endTransmission();    // stop transmitting 
}

void MotorDirectionSet2(unsigned char Direction2)  {     //  Adjust the direction of the motors 0b0000 I4 I3 I2 I1
  Wire.beginTransmission(I2CMotorDriverAdd2); // transmit to device I2CMotorDriverAdd
  Wire.write(DirectionSet);        // Direction control header
  Wire.write(Direction2);              // send direction control information
  Wire.write(Nothing);              // need to send this byte as the third byte(no meaning)  
  Wire.endTransmission();    // stop transmitting 
}

void MotorPWMFrequenceSet(unsigned char Frequence)  {    
  Wire.beginTransmission(I2CMotorDriverAdd); // transmit to device I2CMotorDriverAdd
  Wire.write(PWMFrequenceSet);        // set frequence header
  Wire.write(Frequence);              //  send frequence 
  Wire.write(Nothing);              //  need to send this byte as the third byte(no meaning)  
  Wire.endTransmission();    // stop transmitting
}
*/

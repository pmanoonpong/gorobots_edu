/*
 * This file was created by Tobias Jahn on Tuesday May 01, 2012
 * 
 * This examplecontroller shows the usage of the real epuck interface
 *
 * This program has two functions: Testing sound direction and Testing all sensors and motors
 */

#include "examplecontroller.h"

#include <vector>
#include <iostream>
#include <cmath>
#include <fstream>



double getPeakPos(double* in, int middle, int maxrange){
  int peakPos = middle;
  for(int pos= middle-maxrange; pos<middle+maxrange; pos++){
    if(in[pos]>in[peakPos])peakPos=pos;
  }
  return peakPos-middle;
}



ExampleController::ExampleController() : AbstractController("EPuck example controller", "1.0") {
}

ExampleController::~ExampleController() {
}

void ExampleController::init(int sensornumber, int motornumber, RandGen* randGen) {
}


int ExampleController::getSensorNumber() const {
  return motorCount;
}

int ExampleController::getMotorNumber() const {
  return sensorCount;
}

void ExampleController::step(const sensor* sensors, int _sensorCount, motor* motors, int _motorCount) {

  testing_sensor_motor = true;
  obstacle_avoidance = false;
  sound_direction_detection = false;

  if(testing_sensor_motor==true)
  {
    cout<<"Sensor&Motor Testing Mode"<<"\t"<<sensors[numOfSensor.IR0] << "\t" <<sensors[numOfSensor.IR1] << "\t"<<sensors[numOfSensor.IR2]<< "\t"<<sensors[numOfSensor.IR3]<< "\t"<<sensors[numOfSensor.IR4]<< "\t"<<sensors[numOfSensor.IR5]<< "\t"<<sensors[numOfSensor.IR6] << "\t"<<sensors[numOfSensor.IR7] << "\t"<<endl;
    //cout<<sensors[numOfSensor.AMBIENT_LIGHT0] << "\t" <<sensors[numOfSensor.AMBIENT_LIGHT1] << "\t"<<sensors[numOfSensor.AMBIENT_LIGHT2]<< "\t"<<sensors[numOfSensor.AMBIENT_LIGHT3]<< "\t"<<sensors[numOfSensor.AMBIENT_LIGHT4]<< "\t"<<sensors[numOfSensor.AMBIENT_LIGHT5]<< "\t"<<sensors[numOfSensor.AMBIENT_LIGHT6] << "\t"<<sensors[numOfSensor.AMBIENT_LIGHT7] << "\t"<<endl;
    //cout<<sensors[numOfSensor.GROUND0] << "\t" <<sensors[numOfSensor.GROUND1] << "\t"<<sensors[numOfSensor.GROUND2]<< "\t"<<endl;
    //cout<<sensors[numOfSensor.ACCX] << "\t" <<sensors[numOfSensor.ACCY] << "\t"<<sensors[numOfSensor.ACCZ]<< "\t"<<endl;
    //cout<<sensors[numOfSensor.MIC0] << "\t" <<sensors[numOfSensor.MIC1] << "\t"<<sensors[numOfSensor.MIC2]<< "\t"<<endl;
    //cout<<sensors[numOfSensor.CAM]<< "\t"<<endl;

    /*
    //Testing Camera
    ofstream out("test1.dat");
    int height=2, width=10;
    for( int x=0;x<width;++x){
      for( int y=0;y<height;++y){ out << sensors[numOfSensor.CAM+ x*height+y] <<"\t";
      }out << endl;
    }
     */

    //Testing Microphone
    ofstream out("test.dat");
    for( int i=0;i<100;++i){
      out << sensors[numOfSensor.MIC0+i] << "\t"<< sensors[numOfSensor.MIC1+i] << "\t"<< sensors[numOfSensor.MIC2+i] << endl;
    }

    motors[numOfMotor.MOTOR_LEFT] = 1; // 1 = move forward, 0 =off, -1 = move backward
    motors[numOfMotor.MOTOR_RIGHT] = 1; // 1 = move forward, 0 =off, -1 = move backward
    motors[numOfMotor.SOUND] = 0; // 1,2,3,4
    motors[numOfMotor.LED0] = 0; // 1 = on, 0 =off
    motors[numOfMotor.LED1] = 0; // 1 = on, 0 =off
    motors[numOfMotor.LED2] = 0; // 1 = on, 0 =off
    motors[numOfMotor.LED3] = 0; // 1 = on, 0 =off
    motors[numOfMotor.LED4] = 0; // 1 = on, 0 =off
    motors[numOfMotor.LED5] = 0; // 1 = on, 0 =off
    motors[numOfMotor.LED6] = 0; // 1 = on, 0 =off
    motors[numOfMotor.LED7] = 0; // 1 = on, 0 =off
    motors[numOfMotor.LED_BODY] = 1; // 1 = on, 0 =off
    motors[numOfMotor.LED_FRONT] = 1; // 1 = on, 0 =off

  }

  if(obstacle_avoidance==true)
  {
    cout<<"Obstacle Avoidance Mode"<< "\t" <<sensors[numOfSensor.IR0] << "\t" <<sensors[numOfSensor.IR1] << "\t"<<sensors[numOfSensor.IR2]<< "\t"<<sensors[numOfSensor.IR3]<< "\t"<<sensors[numOfSensor.IR4]<< "\t"<<sensors[numOfSensor.IR5]<< "\t"<<sensors[numOfSensor.IR6] << "\t"<<sensors[numOfSensor.IR7] << "\t"<<endl;
    for(int i=0; i<8; i++){
      (sensors[numOfSensor.IR0+i]>0.5)?motors[numOfMotor.LED0+i]=true:motors[numOfMotor.LED0+i]=false;
    }

    double sensval=0;
    for(int i=0; i<8; i++)sensval+=sensors[numOfSensor.IR0+i]/8;
    double a =2;
    double sens_left=  -1./2*(sensors[numOfSensor.IR0]+sensors[numOfSensor.IR1]-0*sensors[numOfSensor.IR2]-2*sensors[numOfSensor.IR3]);
    double sens_right= -1./2*(sensors[numOfSensor.IR6]+sensors[numOfSensor.IR7]-0*sensors[numOfSensor.IR5]-2*sensors[numOfSensor.IR4]);

    if(sensval>0.6)motors[numOfMotor.SOUND]=2;
    else motors[numOfMotor.SOUND]=0;

    motors[numOfMotor.MOTOR_LEFT] += a* (sens_left  -0.*sens_right +0.0*motors[numOfMotor.MOTOR_RIGHT] + 0.01);
    motors[numOfMotor.MOTOR_RIGHT]+= a* (sens_right -0.*sens_left  +0.0*motors[numOfMotor.MOTOR_LEFT]  + 0.01);

    double speed_max = 0.8;
    if(motors[numOfMotor.MOTOR_LEFT]>speed_max)motors[numOfMotor.MOTOR_LEFT]=speed_max;
    if(motors[numOfMotor.MOTOR_LEFT]<-speed_max)motors[numOfMotor.MOTOR_LEFT]=-speed_max;
    if(motors[numOfMotor.MOTOR_RIGHT]>speed_max)motors[numOfMotor.MOTOR_RIGHT]=speed_max;
    if(motors[numOfMotor.MOTOR_RIGHT]<-speed_max)motors[numOfMotor.MOTOR_RIGHT]=-speed_max;
  }


  if(sound_direction_detection == true)
  {

    //Tobias algorithm for sound direction detection
    // To test this function
    //"Please set conf.MIC_STATE = true in main"

    cout<<"Sound Direction Detection Mode"<<"\t"<<sensors[numOfSensor.MIC0] << "\t" <<sensors[numOfSensor.MIC1] << "\t"<<sensors[numOfSensor.MIC2]<< "\t"<<endl;

    for(int i=0; i<100;i++)buffer[i]=sensors[numOfSensor.MIC0+i];
    for(int i=0; i<100;i++)buffer[i+100]=sensors[numOfSensor.MIC1+i];
    for(int i=0; i<100;i++)buffer[i+200]=sensors[numOfSensor.MIC2+i];

    //filter noise
    for(int i=0; i<299;i++)  if(abs(buffer[i]-buffer[i+1])>200)   buffer[i+1]=0;
    for(int i=0; i<299;i++)  buffer[i]=   (buffer[i]+buffer[i+1])/2;


    //interpolate the mics linear
    int N = 200;
    for(int i=0; i<3*N; i++){
      i%2==0?  (micData[0][i] = buffer[i/2])  :  (micData[0][i] = (buffer[(i-1)/2]+buffer[(i+1)/2])/2.);
    }

    //take the absolut of the mic-data
    //for(int i=0; i<3*N; i++) micData[0][i]=abs(micData[0][i]);

    //calculate meanSqr (for event-detection)
    double meanSqr=0;    for(int i=0; i<3*N; i++)meanSqr += micData[0][i]*micData[0][i]/3/N;

    if(meanSqr>30){
      //create array for the correlations
      int corrsize=100;
      for(int i=0; i<300;i++)corr[0][i]=0;

      //calculate the correlation. tau=0 is at corrsize/2
      for(int tau=-corrsize/2; tau<corrsize/2; tau++)for(int j=0; j<N; j++){
        int i = tau+corrsize/2;
        corr[0][i] += micData[0][j]*micData[1][(j+tau +N)%N];
        corr[1][i] += micData[1][j]*micData[2][(j+tau +N)%N];
        corr[2][i] += micData[0][j]*micData[2][(j+tau +N)%N];
      }

      double kx, ky;
      kx =  -3.* (getPeakPos(corr[0], corrsize/2, 10)) -6.*(getPeakPos(corr[1], corrsize/2, 10));
      ky =  5.* (getPeakPos(corr[0],  corrsize/2, 10)) -0.*(getPeakPos(corr[1], corrsize/2, 10));
      double phi = atan2(-ky, kx)/M_PI*180;

      phi = ((int)(phi+360))%360;

      for(int i=0; i<8; i++)motors[numOfMotor.LED0+i]=0;
      if(phi < 30) motors[numOfMotor.LED4] = 1;
      else if(phi < 30+45) motors[numOfMotor.LED5] = 1;
      else if(phi < 30+90) motors[numOfMotor.LED6] = 1;
      else if(phi < 30+135) motors[numOfMotor.LED7] = 1;
      else if(phi < 30+180) motors[numOfMotor.LED0] = 1;
      else if(phi < 30+225) motors[numOfMotor.LED1] = 1;
      else if(phi < 30+270) motors[numOfMotor.LED2] = 1;
      else if(phi < 30+315) motors[numOfMotor.LED3] = 1;
      else if(phi < 30+45) motors[numOfMotor.LED4] = 1;

      /*
             ofstream data("korr.dat");
             for(int tau=-corrsize/2; tau<corrsize/2; tau++){
                 int i = tau+corrsize/2;
                 data << tau << "\t" << corr[0][i]  << "\t" << corr[1][i]<< "\t" << corr[2][i] << endl;
             }
             data.close(); data.open("mic.dat");
             for(int i=0; i<N; i++){
                 data << micData[0][i] << "\t" << micData[1][i] << "\t" << micData[2][i] << endl;
             }
             data.close(); data.open("peak.dat", ios::app);
                 data << getPeakPos(corr[0], corrsize/2, 10) << "\t" << getPeakPos(corr[1], corrsize/2, 10) << "\t" << getPeakPos(corr[2], corrsize/2, 10)<< endl;
             data.close();//
       //*/
    }

  }



}

void ExampleController::stepNoLearning(const sensor*, int _sensorCount, motor*, int _motorCount) {

}

bool ExampleController::store(FILE* f) const {
  return false;
}

bool ExampleController::restore(FILE* f) {
  return false;
}

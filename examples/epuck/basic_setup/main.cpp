
/*
 * This file was created by Tobias Jahn on Tuesday May 01, 2012
 *
 * Modify by Poramate Manoonpong on  30 April 2013
 * Modify by Poramate Manoonpong on  23 September 2013
 * This example shows the usage of the real epuck interface
 *
 *
 *To check epuck  MAC address
 *1) >> hcitool scan
 *
 *e.g., 10:00:E8:AD:77:CA e-puck_2358
 *e.g., 10:00:E8:AD:FF:40 e-puck_0990
 *e.g., 10:00:E8:AD:78:51 e-puck_3202
 *e.g., 10:00:E8:AD:78:82 e-puck_2899
 *e.g., 10:00:E8:C5:61:E6 e-puck_3068
 *
 *2) Copy this MAC address to "rfcomm.conf is https://www.dropbox.com/home/gorobots/Epuck/All_about_Epuck_Setup"
 * OR /etc/bluetooth/rfcomm.conf
 * e.g., sudo cp Desktop/rfcomm.conf /etc/bluetooth/rfcomm.conf
 *
 *To pair epuck with PC
 *sudo rfcomm bind rfcomm0   --> password 2358
 *sudo rfcomm bind rfcomm1   --> password 0990
 *sudo rfcomm bind rfcomm2   --> password 3202
 *sudo rfcomm bind rfcomm3   --> password 2899
 *sudo rfcomm bind rfcomm4   --> password 3068
 *
 *3) To run the program at terminal type:
 *
 *./start -g 1 -p /dev/rfcomm0 for Epuck 2358
 *./start -g 1 -p /dev/rfcomm1 for Epuck 0990
 *./start -g 1 -p /dev/rfcomm2 for Epuck 3202
 *./start -g 1 -p /dev/rfcomm3 for Epuck 2899
 *./start -g 1 -p /dev/rfcomm4 for Epuck 3068
 *
 *
 * GUI logger
 * xR[0]= accX
 * xR[1]= accY
 * xR[2]= accZ
 * xR[3]= ir_front_right
 * xR[4]= ir_middle_front_right
 * xR[5]= ir_middle_right
 * xR[6]= ir_back_right
 * xR[7]= ir_back_left
 * xR[8]= ir_middle_left
 * xR[9]= ir_middle_front_left
 * xR[10]= ir_front_left
 *
 * xR[11]= AMBIENT_LIGHT_front_right
 * xR[12]= AMBIENT_LIGHT_middle_front_right
 * xR[13]= AMBIENT_LIGHT_middle_right
 * xR[14]= AMBIENT_LIGHT_back_right
 * xR[15]= AMBIENT_LIGHT_back_left
 * xR[16]= AMBIENT_LIGHT_middle_left
 * xR[17]= AMBIENT_LIGHT_middle_front_left
 * xR[18]= AMBIENT_LIGHT_front_left
 *
 * xR[19]= Ground0 sensor left
 * xR[20]= Ground1 sensor middle
 * xR[21]= Ground2 sensor right
 *
 * xR[22].... = MIC0, MIC1, MIC02, CAM
 *
 *
 *
 *Note if epuck cannot connect to PC try:
 *
 * 1) >> hcitool scan --->Search for active bluetooth devices to check MAC address whether it is correct!!!
 * e.g., 10:00:E8:AD:77:CA e-puck_2358. This address should be similar to the one in "/etc/bluetooh/rfcomm.conf"
 *
 * 2) >> hcitool con --> Display active bluetooth connections.
 *
 * 3) >> (sudo) rfcomm release X --> Release rfcommX port.
 *
 * 4) >> (sudo) rfcomm bind X --> Bind the rfcommX device again!.
 *
 * 5) >> ls /dev/rfcomm* --> List rfcomm devices
 */

#include <iostream>
#include <signal.h>
#include <cmath>
#include <string>

#include <selforg/agent.h>
#include <selforg/abstractrobot.h>
#include <selforg/one2onewiring.h>

#include "epuckbluetooth.h"

#include "examplecontroller.h"


using namespace std;
using namespace lpzrobots;

volatile bool abortLoop;
void endLoop(int){abortLoop=1;}

bool singleline = true; // whether to display the state in a single line


// Helper
int contains(char **list, int len, const char *str) {
  for (int i = 0; i < len; i++) {
    if (strcmp(list[i], str) == 0)
      return i + 1;
  }
  return 0;
}
;

int main(int argc, char** argv)
{
  cout << endl << endl;
  cout << "Start program!" << endl;

  cout << "Create config..." << endl;
  EPuckConf conf = EPuckBluetooth::getDefaultConfig();

  conf.SENSOR_STATE = true; //true = use all sensors except micro & camera, false = not using all sensors except micro & camera
  conf.MIC_STATE = true; //true = use microphone, false = not using microphone
  conf.CAM_STATE = false;
  conf.CAM_HEIGHT = 40; // 2 small res
  conf.CAM_WIDTH = 40; // 10  small res
  conf.CAM_TYPE = 0; //0 = black and white, 1= color
  conf.CAM_ZOOM = 8;//1 2 4 8 or 16

  //40x40 = 1600 pixel or 2x10 = 20 pixel * 3 colors
  //maximum of Pixels is 3200

//********Adding for plotting graph ************************//
  list<PlotOption> plotoptions;
  int port = 1;

  int index = contains(argv, argc, "-g");
  if (index > 0 && argc > index) {
    plotoptions.push_back(PlotOption(GuiLogger, atoi(argv[index])));
  }
  if (contains(argv, argc, "-f") != 0)
    plotoptions.push_back(PlotOption(File));
  if (contains(argv, argc, "-n") != 0)
    plotoptions.push_back(PlotOption(MatrixViz));
  if (contains(argv, argc, "-l") != 0)
    singleline = false;
  index = contains(argv, argc, "-p");

//********Adding for interfacing to a robot*******************//
  index = contains(argv, argc, "-p");
  if (index > 0 && argc > index)
    conf.port = argv[index];




 // if (argc !=1)  conf.port = argv[1];



  cout << "Create robot (EPuckBluetooth)..." << endl;
  EPuckBluetooth *robot = new EPuckBluetooth(conf);
  AbstractWiring *wiring = new One2OneWiring(new WhiteUniformNoise(),true);
  ExampleController *controller = new ExampleController();
  Agent* agent = new Agent(plotoptions);
  //Agent* agent = new Agent();
  agent->init(controller, robot, wiring);



  cout << "Created. Embedded softwareversion:\n\t" << (char*)robot->version;
  usleep(1e6);

  cout << "optimal connection speed:\t" << robot->getRecommendConnectionSpeed(conf) << "Hz" << endl;


  int sensorCount, motorCount;
  SensorNumbers numOfSensor;
  MotorNumbers numOfMotor;
  robot->getNumOfMotSens(numOfSensor, sensorCount, numOfMotor, motorCount);
  double *sensors = new double[sensorCount];     for(int i=0; i<sensorCount;i++)    sensors[i]=0;

  controller->setSensorMotorNumbers(numOfSensor, sensorCount, numOfMotor, motorCount);
  controller->setConf(conf);
  cout << "Sensors: " << sensorCount << "\t\tMotors: " << motorCount << endl;

  if(!robot->isRunning()){
    cout << "Enter eventloop. Press Strg+C to exit." << endl;
    signal(SIGINT, endLoop); //catch strg+c to exit eventloop

    double Hz=robot->getRecommendConnectionSpeed(conf), t=0; abortLoop=0;

    while(!abortLoop){

      //use a controller
      controller->t=t;
      agent->step(0,t);
      //end using a controller

      const int n=Hz;//n = outputs per second
      if( (int)(n*t)!=(int)(n*(t+1./Hz))){
        robot->getSensors(sensors,sensorCount);
        cout << "\r\033[K" << flush; // clear line
        //cout << "\r";  for(int i=0; i<sensorCount&&i<22; i++) cout << sensors[i] << "\t" << flush;   cout << "\tHz =" << robot->connectionSpeedHz << "\tTime = " << (int)t << flush ;
      }
      usleep((int)1.e6/Hz);

      t+=1./Hz;
    }
  } // end if(isRunning())
  else{
    cout << "Connection is not running" << endl;
  }

  cout << endl << "Exit program!" << endl;
  cout << endl << endl;

  delete controller;
  delete agent;
  delete wiring;
  delete robot;
  delete sensors;
  return 0;

}//*/

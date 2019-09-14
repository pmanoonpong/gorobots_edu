/*
 * This file was created by Tobias Jahn on Tuesday May 01, 2012
 * 
 * This examplecontroller shows the usage of the real epuck interface
 *
 */

#ifndef EXAMPLECONTROLLER_H
#define EXAMPLECONTROLLER_H

using namespace std;

#include <selforg/abstractcontroller.h>
#include "epuckbluetooth.h"

using namespace lpzrobots;

class ExampleController : public AbstractController {
  public:
    ExampleController();
    virtual void init(int sensornumber, int motornumber, RandGen* randGen = 0);

    virtual ~ExampleController();

    // returns the number of sensors the controller was initialised with or 0
    // if not initialised
    virtual int getSensorNumber() const;

    // returns the mumber of motors the controller was initialised with or 0 if
    // not initialised
    virtual int getMotorNumber() const;

    // performs one step (includes learning).
    // Calulates motor commands from sensor inputs.
    virtual void step(const sensor*, int number_sensors, motor*,int number_motors);

    // performs one step without learning. Calulates motor commands from sensor
    // inputs.
    virtual void stepNoLearning(const sensor*, int number_sensors, motor*, int number_motors);

    /***** STOREABLE ****/
    /** stores the controller values to a given file. */
    virtual bool store(FILE* f) const;
    /** loads the controller values from a given file. */
    virtual bool restore(FILE* f);

    void setSensorMotorNumbers(SensorNumbers _numOfSensor, int _sensorCount,MotorNumbers _numOfMotor, int _motorCount){
      numOfSensor = _numOfSensor;
      numOfMotor = _numOfMotor;
      motorCount = _motorCount;
      sensorCount = _sensorCount;
    }
    void setConf(EPuckConf _conf){
      conf=_conf;
    }

    double t;
  private:
    int motorCount, sensorCount;
    SensorNumbers numOfSensor;
    MotorNumbers numOfMotor;
    EPuckConf conf;

    double buffer[300]; //buffer microphone signals
    double micData[3][200]; //buffer interpolated microphone signals 
    double corr[3][100]; //buffer correlation functions

    bool testing_sensor_motor;
    bool obstacle_avoidance;
    bool sound_direction_detection;

};



#endif //examplecontroller.h

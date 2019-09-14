/*
 * muscleRunbotController.h
 *
 *  Created on: 08.03.2014
 *      Author: Johannes Widenka
 */

#ifndef MUSCLERUNBOTCONTROLLER_H_
#define MUSCLERUNBOTCONTROLLER_H_

#include <vector>
#include <cmath>
#include <cstdlib>
#include <iostream> //for plotting
#include <fstream> //plotting

#include <selforg/abstractcontroller.h>


template <typename T> int sgn(T val) {
  return (T(0) < val) - (val < T(0));
}

template <typename T> int abs(T val) {
  return (T(0) < val)?-val:val;
}

/// Channel number description
const int BOOM_ANGLE = 1;
const int LEFT_FOOT  = 2;   // = 0 .. -4.96v,  2048 = 0V (foot contact the ground) ....
//                4096 = + 4.96V (foot off ground)
const int RIGHT_FOOT = 3;   // chanel 3
const int LEFT_HIP   = 4;
const int RIGHT_HIP  = 5;
const int LEFT_KNEE  = 6;
const int RIGHT_KNEE = 7;


class MuscleRunbotController : public AbstractController {
  public:
    MuscleRunbotController(const std::string& name, const std::string& revision);
    /** initialisation of the controller with the given sensor/ motornumber
         Must be called before use. The random generator is optional.
     */
    virtual void init(int sensornumber, int motornumber, RandGen* randGen = 0);

    /** @return Number of sensors the controller
         was initialised with or 0 if not initialised */
    virtual int getSensorNumber() const;

    /** @return Number of motors the controller
         was initialised with or 0 if not initialised */
    virtual int getMotorNumber() const;

    /** performs one step.
         Calculates motor commands from sensor inputs.
         @param sensors sensors inputs scaled to [-1,1]
         @param sensornumber length of the sensor array
         @param motors motors outputs. MUST have enough space for motor values!
         @param motornumber length of the provided motor array
     */
    virtual void step(const sensor* sensors, int sensornumber,
        motor* motors, int motornumber);


    /** performs one step without learning.
         @see step
     */
    virtual void stepNoLearning(const sensor* sensors, int number_sensors,
        motor* motors, int number_motors);
    /** stores the object to the given file stream (binary).
     */
    virtual bool store(FILE* f) const;

    /** loads the object from the given file stream (binary).
     */
    virtual bool restore(FILE* f);

  private:
    int nSensors;
    int nMotors;
    int steps;		// counter for controll steps
    double ubc;		// position of the upper body component (-1..+1)
    double speed;	// current speed of the robot
    double pos;		// current position of the robot
    double simulatedMass;	//simulated mass of the robot, used by all modelled muscles
    double ubc_wabl = 0.0;	//movement of the UBC around its position variable "ubc"
    //wabbling from ubc-ubc_wabl to ubc+ubc_wabl
    int ubc_time = 100; 		// sec/100   -   defines the time where the ubc changes its movement direction

    //std::vector<double> actualAD;
    std::ofstream hipPlot;//plot


    //-----Control parameters------------//
    double threshold_al;
    double threshold_ar;

    double leftpiezo;
    double rightpiezo;
    double angle_hl;
    double angle_hr;
    double angle_kl;
    double angle_kr;

    double angle_hl_low_pass_pre;
    double angle_hl_low_pass;

    double angle_hr_low_pass_pre;
    double angle_hr_low_pass;

    double angle_kl_low_pass_pre;
    double angle_kl_low_pass;

    double angle_kr_low_pass_pre;
    double angle_kr_low_pass;

    double u_gl;
    double u_gr;
    double u_al;
    double u_ar;

    double state_u_hr_em;
    double state_u_hr_fm;

    double state_u_hl_em;
    double state_u_hl_fm;

    double state_u_kr_em;
    double state_u_kr_fm;

    double state_u_kl_em;
    double state_u_kl_fm;

    double state_motorvolt_hl;
    double state_motorvolt_hr;

    double state_motorvolt_kl;
    double state_motorvolt_kr;

 };

#endif /* MUSCLERUNBOTCONTROLLER_H_ */

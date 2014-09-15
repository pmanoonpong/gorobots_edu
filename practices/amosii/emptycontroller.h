#ifndef __EMPTYCONTROLLER_H
#define __EMPTYCONTROLLER_H

#include <selforg/abstractcontroller.h>
#include <selforg/controller_misc.h>
#include <selforg/configurable.h>

#include <assert.h>
#include <cmath>
#include <stdlib.h>
#include <string.h>

#include <selforg/matrix.h>

/**
 * Empty robot controller.
 * The controller gets a number of input sensor values each timestep
 * and has to generate a number of output motor values.
 *
 * Go to the step() function and enter the control commands with respect to your task!
 *
 */
class EmptyController : public AbstractController {

  public:
    EmptyController()
    : AbstractController("EmptyController", "$Id: tripodgait18dof.cpp,v 0.1 $"){
      t = 0;

      outputH1 = 0.001;
      outputH2 = 0.001;

      // plot parameters using GUI "To display GUI, in terminal, type ./start -g 1 "
      addInspectableValue("outputH1", &outputH1,"outputH1");
      addInspectableValue("outputH2", &outputH2,"outputH2");

    };



    virtual void init(int sensornumber, int motornumber, RandGen* randGen = 0){
      //Tripodgait for 18 DOF Hexapod
      assert(motornumber>=18);
    };

    virtual ~EmptyController(){};

    /// returns the name of the object (with version number)
    virtual paramkey getName() const {
      return name;
    }
    /// returns the number of sensors the controller was initialised with or 0
    /// if not initialised
    virtual int getSensorNumber() const {
      return number_channels;
    }
    /// returns the mumber of motors the controller was initialised with or 0 if
    // not initialised
    virtual int getMotorNumber() const {
      return number_channels;
    }

    /// performs one step (includes learning).
    /// Calulates motor commands from sensor inputs.
    virtual void step(const sensor* x_, int number_sensors,
        motor* y_, int number_motors){
      stepNoLearning(x_, number_sensors, y_, number_motors);
    };

    /// performs one step without learning. Calulates motor commands from sensor
    /// inputs.
    virtual void stepNoLearning(const sensor* x_, int number_sensors,
        motor* y_, int number_motors){
      //Tripodgait for 18 DOF Hexapod

      assert(number_sensors >= 18);
      assert(number_motors >= 18);

      //----Students--------Adding your Neural Controller here------------------------------------------//

      // sensory inputs ,e.g, x_[FL_us] = left antenna, x_[FR_us] = right antenna (see also the list below)
      // x_[G0x_s] , x_[G0y_s], x_[G0z_s] = relative position to reference object 1 (red ball)

      // Final outputs of your controller should be set to the following y_[xx] parameters to control leg joints
      //-----------------------------------------------------------------------------------------------//


      // generate motor commands

      // right rear coxa (knee) forward-backward joint (back is positive)
      y_[TR2_m] = 0; // TC  joint (forward-backward movements), e.g., y_[TR2_m] = 1 (move forward), -1 (move backward)
      y_[CR2_m] = 0; // CTr joint (up-down movements), e.g., y_[CR2_m] = 1 (move up), -1 (move down)
      y_[FR2_m] = 0; // FTi joint (extension-flexion movements), e.g., y_[CR2_m] = 1 (extension), -1 (flexion)

      //left rear
      y_[TL2_m] = 0;
      y_[CL2_m] = 0;
      y_[FL2_m] = 0;

      //right middle
      y_[TR1_m] = 0;
      y_[CR1_m] = 0;
      y_[FR1_m] = 0;

      //left middle
      y_[TL1_m] = 0;
      y_[CL1_m] = 0;
      y_[FL1_m] = 0;

      //right front
      y_[TR0_m] = 0;
      y_[CR0_m] = 0;
      y_[FR0_m] = 0;

      //left front
      y_[TL0_m] = 0;
      y_[CL0_m] = 0;
      y_[FL0_m] = 0;

      // backbone joint
      y_[BJ_m] = 0;

      // update step counter
      t++;
    };

    /***** STOREABLE ****/
    /** stores the controller values to a given file. */
    virtual bool store(FILE* f) const {
      return true;
    };
    /** loads the controller values from a given file. */
    virtual bool restore(FILE* f){
      return true;
    };


  protected:
    unsigned short number_channels;

    int t;
    paramkey name;
    double outputH1;
    double outputH2;

};

#endif


/*
 * List of available Sensors and their numbers/enum's to be used for example as x_[TR0_as] in the step function
 *
// Angle sensors (for actoric-sensor board (new board))
       TR0_as=0, //Thoracic joint of right front leg
       TR1_as=1, //Thoracic joint of right middle leg
       TR2_as=2, //Thoracic joint of right hind leg

       TL0_as=3, //Thoracic joint of left front leg
       TL1_as=4, //Thoracic joint of left middle leg
       TL2_as=5, //Thoracic joint of left hind leg

       CR0_as=6, //Coxa joint of right front leg
       CR1_as=7, //Coxa joint of right middle leg
       CR2_as=8, //Coxa joint of right hind leg

       CL0_as=9,  //Coxa joint of left hind leg
       CL1_as=10, //Coxa joint of left hind leg
       CL2_as=11, //Coxa joint of left hind leg

       FR0_as=12, //Fibula joint of right front leg
       FR1_as=13, //Fibula joint of right middle leg
       FR2_as=14, //Fibula joint of right hind leg

       FL0_as=15, //Fibula joint of left front leg
       FL1_as=16, //Fibula joint of left middle leg
       FL2_as=17, //Fibula joint of left hind leg

       BJ_as= 18, //Backbone joint angle

       //Foot contact sensors (AMOSII v1 and v2)
       R0_fs= 19, //Right front foot
       R1_fs= 20, //Right middle foot
       R2_fs= 21, //Right hind foot
       L0_fs= 22, //Left front foot
       L1_fs= 23, //Left middle foot
       L2_fs= 24, //Left hind foot

       // US sensors (AMOSII v1 and v2)
       FR_us=25, //Front Ultrasonic sensor (right)
       FL_us=26, //Front Ultrasonic sensor (left)

       // IR reflex sensors at legs (AMOSIIv2)
       R0_irs=31,
       R1_irs=29,
       R2_irs=27,
       L0_irs=32,
       L1_irs=30,
       L2_irs=28,

       // goal orientation sensors (relative position to reference object 1 (red), e.g. camera)
       G0x_s=62,
       G0y_s=63,
       G0z_s=64,

       //Body speed sensors (only simulation)
       BX_spd= 66,
       BY_spd= 67,
       BZ_spd= 68,

       // goal orientation sensors (relative position to reference object 2 (green), e.g. camera)
       G1x_s=72,
       G1y_s=73,
       G1z_s=74,

       // goal orientation sensors (relative position to reference object 3 (blue), e.g. camera)
       G2x_s=78,
       G2y_s=79,
       G2z_s=80,

 */

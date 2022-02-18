#ifndef __EMPTYCONTROLLER_H
#define __EMPTYCONTROLLER_H

#include <selforg/abstractcontroller.h>
#include <selforg/controller_misc.h>
#include <selforg/configurable.h>

#include <assert.h>
#include <cmath>
#include <stdlib.h>
#include <stdio.h>

#include <math.h>
#include <vector>
#include <algorithm>


#include <unistd.h> // standard function definitions time delay function

///////// Save text////////////////////////
#include <iostream>
#include <fstream>
#include <string.h>
using namespace std;
///////////////////////////////////////////




#include <selforg/matrix.h>


#define pi 3.14159265
#define CPG


/** To run simulation
* 0) open ternimal and go to your simulation folder "/workspace/gorobots/practices/amosii"
  1) make clean
  2) make 
  3) ./start -g 1 
  then you should see the simulation and GUI
  4) press x to drop your robot on the ground

  to stop the simulation
  1) click the terminal + ctrl + pres "c" 2 times 

  to record simulation
  1) ctrl + r  
  the program will automatically create "video" folder and put snapshots in there
  
  to fix camera
  1) click simulation panel and then press "1"
  
  to change view 
  1) click simulation and then click left and right buttons of the mouse and then move the mouse for zoom in and out
  2) click simulation and then click right button of the mouse and the move the mouse for rotating the view

  to move your robot using the mouse
  1) ctrl + left button of the mouse 

  to plot graph using gnuplot
  1) type "gnuplot" -->  /workspace/gorobots/practices/amosii$ gnuplot
  2) type "plot "savedata1.txt" using 1 with lines" --> gnuplot>  plot "savedata1.txt" using 1 with lines

  plot multiple lines

  plot "savedata1.txt" using 1 title 'line1' with lines,\
     "savedata1.txt" using 2 title 'line2' with lines,\
     "savedata1.txt" using 3 title 'line3' with lines


*/




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
      

      //1) Initial parameters 
      
      /// Neural Control I: CPG implementation, for students ///
      outputH1 = 0.2;
      outputH2 = 0.2;
      activityH1 = 0.0;
      activityH2 = 0.0;

      //  Connections from CPG to motor neurons
      bias_tjoint = 0.0;
      bias_cjoint = 0.5;
      bias_fjoint = -0.9;

      WeightM0_H2 = -0.5;
      WeightM1_H2 = 0.5;
      WeightM2_H2 = -0.5;

      WeightM3_H2 = 0.5;
      WeightM4_H2 = -0.5;
      WeightM5_H2 = 0.5;

      WeightM6_H1 = -0.2;
      WeightM7_H1 = 0.2;
      WeightM8_H1 = -0.2;

      WeightM9_H1 = 0.2;
      WeightM10_H1 = -0.2;
      WeightM11_H1 = 0.2;

      WeightM12_H1 = -0.2;
      WeightM13_H1 = 0.2;
      WeightM14_H1 = -0.2;

      WeightM15_H1 = 0.2;
      WeightM16_H1 = -0.2;
      WeightM17_H1 = 0.2;

      percount = 1;
      UPDATE = 1;

      outputH1_Chaos = 0.001;
      outputH2_Chaos = 0.001;
      activityH1_Chaos = 0.0;
      activityH2_Chaos = 0.0;

      alph = 1.5;
      phi = 0.2*pi;
      // 0< phi < pi ... 0.4*pi can make within 0-250 steps = distance of 2.0, 
      // 0.6*pi can make within 0-250 steps = distance of 1.8,
      MI = 0.01;


      // Lorenz

      x = 0.1;
      x_dot = 0.1;
      y = 0.1;
      y_dot = 0.1;
      z = 0.1;
      z_dot = 0.1;
      Output_CPG = 0.0;

      saveFile1.open("savedata1.txt",ios::out);
      saveFile2.open("savedata2.txt",ios::out);


      // plot parameters using GUI "To display GUI, in terminal, type ./start -g 1 "
      addInspectableValue("CPGoutputH1", &outputH1,"CPGoutputH1");
      addInspectableValue("CPGoutputH2", &outputH2,"CPGoutputH2");

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
      int static iii=0;
      assert(number_sensors >= 18);
      assert(number_motors >= 18);

      //----Students--------Adding your Neural Controller here------------------------------------------//

      // sensory inputs ,e.g, x_[FL_us] = left antenna, x_[FR_us] = right antenna (see also the list below)
      // x_[G0x_s] , x_[G0y_s], x_[G0z_s] = relative position to reference object 1 (red ball)

      // Final outputs of your controller should be set to the following y_[xx] parameters to control leg joints





  /*******************************************************************************
   *  MODULE 1 CPG
   *******************************************************************************/

      /// Neural Control I: CPG implementation, for students ///

 #ifdef CPG


if(++percount%UPDATE==0)
	{

      // Chaotic CPG
      WeightH1_H1_Chaos  = -5.5;// 1.5; 
      WeightH2_H2_Chaos  =  0.0;//1.5;
      WeightH1_H2_Chaos  =  1.48;//-0.4;
      WeightH2_H1_Chaos  =  -1.65;//0.4;

      BiasH1_Chaos = -5.73;
      BiasH2_Chaos = 0.25;

      activityH1_Chaos = WeightH1_H1_Chaos*outputH1_Chaos+WeightH1_H2_Chaos*outputH2_Chaos+BiasH1_Chaos;
      activityH2_Chaos = WeightH2_H2_Chaos*outputH2_Chaos+WeightH2_H1_Chaos*outputH1_Chaos+BiasH2_Chaos;
    
      outputH1_Chaos = (exp(2*activityH1_Chaos)-1)/(exp(2*activityH1_Chaos)+1);//tanh(activityH1);
      outputH2_Chaos = (exp(2*activityH2_Chaos)-1)/(exp(2*activityH2_Chaos)+1);//tanh(activityH2);


      // Periodic CPG
      WeightH1_H1  = alph*cos(phi);//1.5;1.4; 
      WeightH2_H2  = alph*cos(phi);//1.5;1.4;
      WeightH1_H2  = -alph*sin(phi);//-0.4;-0.19
      WeightH2_H1  = alph*sin(phi);//0.4;0.19

      //WeightH1_H1  = 1.4; 
      //WeightH2_H2  = 1.4;
      //WeightH1_H2  = -0.18-MI;
      //WeightH2_H1  = 0.18+MI;

      //WeightH1_H1  = 1.5; 
      //WeightH2_H2  = 1.5;
      //WeightH1_H2  = -0.4;
      //WeightH2_H1  = 0.4;


      activityH1 = WeightH1_H1*outputH1+WeightH1_H2*outputH2;
      activityH2 = WeightH2_H2*outputH2+WeightH2_H1*outputH1;


      outputH1 = (exp(2*activityH1)-1)/(exp(2*activityH1)+1);//tanh(activityH1);
      outputH2 = (exp(2*activityH2)-1)/(exp(2*activityH2)+1);//tanh(activityH2);



   //---Lorenz oscillator ---//

    double sigma = 10;
    double beta = 8/3;
    double ro = 28;
    double TAU = 0.025; //TAU = 0.1 = low resolution does not work, TAU = 0.01 or 0.001 = High resolution works

    //dx/dt = -sigma*x+sigma*y;
    //dy/dt =-y-x*z;
    //dz/dt =-beta*z+x*y-beta*ro;

    //Neuron1	
    x_dot = -sigma*x+sigma*y;
    //Neuron2	
    y_dot =-y-x*z;
    //Neuron3	
    z_dot =-beta*z+x*y-beta*ro;

    //Update state variable with Euler first order's method!    
    x += TAU*x_dot;
    y += TAU*y_dot;
    z += TAU*z_dot;





      //usleep(1000000);
      

	}

	printf("CPG %f %f %f %f\n",x, y, outputH2, outputH1);

      //printf("CPG %f %f\n",outputH1, outputH2);
#endif


  /*******************************************************************************
   *  MODULE Motor neurons
   *******************************************************************************/
     outputH2 = x/15;
     outputH1 = y/15;
  /// Neural Control I: CPG implementation, for students ///
    //M0  
    //y_[TR0_m] = WeightM0_H2*outputH2_Chaos + bias_tjoint;
    y_[TR0_m] = WeightM0_H2*outputH2 + bias_tjoint;
    //M1
    y_[TR1_m] = WeightM1_H2*outputH2 + bias_tjoint;
    //M2
    y_[TR2_m] = WeightM2_H2*outputH2 + bias_tjoint;

    //M3
    y_[TL0_m] = WeightM3_H2*outputH2 + bias_tjoint;
    //M4
    y_[TL1_m] = WeightM4_H2*outputH2 + bias_tjoint;
    //M5
    y_[TL2_m] = WeightM5_H2*outputH2 + bias_tjoint;

    //M6
    y_[CR0_m] = WeightM6_H1*outputH1 + bias_cjoint;
    //M7
    y_[CR1_m] = WeightM7_H1*outputH1 + bias_cjoint;
    //M8
    y_[CR2_m] = WeightM8_H1*outputH1 + bias_cjoint;

    //M9
    y_[CL0_m] = WeightM9_H1*outputH1 + bias_cjoint;
    //M10
    y_[CL1_m] = WeightM10_H1*outputH1 + bias_cjoint;
    //M11
    y_[CL2_m] = WeightM11_H1*outputH1 + bias_cjoint;

    //M12
    y_[FR0_m] = WeightM12_H1*outputH1 + bias_fjoint;
    //M13
    y_[FR1_m] = WeightM13_H1*outputH1 + bias_fjoint;
    //M14   
    y_[FR2_m] = WeightM14_H1*outputH1 + bias_fjoint;
     
    //M15
    y_[FL0_m] = WeightM15_H1*outputH1 + bias_fjoint;
    //M16
    y_[FL1_m] = WeightM16_H1*outputH1 + bias_fjoint;
    //M17
    y_[FL2_m] = WeightM17_H1*outputH1 + bias_fjoint;
      
    // backbone joint
    y_[BJ_m] = 0;


    // update step counter
    t++;

     
      saveFile1 <<outputH1<<"  "<<outputH2<<"  "<<y_[TR0_m]<<" "<<y_[TR1_m]<<" "<<y_[TR2_m]<<"   \n" << flush; //SAVE DATA
      saveFile2 <<outputH1<<"  "<<outputH2<<"  "<<y_[TR0_m]<<" "<<y_[TR1_m]<<" "<<y_[TR2_m]<<"   \n" << flush; //SAVE DATA

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

   
   //0) Create your parameters
 
   /// Neural Control I: CPG implementation, for students ///

   double alph;
   double phi;
   double MI;
   double WeightH1_H1;
   double WeightH2_H2;
   double WeightH1_H2;
   double WeightH2_H1;

   double BiasH1;
   double BiasH2;

   double activityH1;
   double activityH2;

   double outputH1;
   double outputH2;
   
   double bias_tjoint;
   double bias_cjoint;
   double bias_fjoint;

   double WeightM0_H2;
   double WeightM1_H2;
   double WeightM2_H2;

   double WeightM3_H2;
   double WeightM4_H2;
   double WeightM5_H2;

   double WeightM6_H1;
   double WeightM7_H1;
   double WeightM8_H1;

   double WeightM9_H1;
   double WeightM10_H1;
   double WeightM11_H1;

   double WeightM12_H1;
   double WeightM13_H1;
   double WeightM14_H1;

   double WeightM15_H1;
   double WeightM16_H1;
   double WeightM17_H1;


   int percount;
   int UPDATE;

   double WeightH1_H1_Chaos;
   double WeightH2_H2_Chaos;
   double WeightH1_H2_Chaos;
   double WeightH2_H1_Chaos;

   double BiasH1_Chaos;
   double BiasH2_Chaos;

   double activityH1_Chaos;
   double activityH2_Chaos;

   double outputH1_Chaos;
   double outputH2_Chaos;

  double x;
  double y;
  double z;
  double x_dot;
  double y_dot;
  double z_dot;

  double Output_CPG;

    // --- Save text------------//
    ofstream saveFile1;
    ofstream saveFile2;
    //-------------------------//

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

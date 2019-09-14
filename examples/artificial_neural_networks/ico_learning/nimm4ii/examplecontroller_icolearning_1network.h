#ifndef __EMPTYCONTROLLER_H
#define __EMPTYCONTROLLER_H


#include <selforg/abstractcontroller.h>
#include <selforg/controller_misc.h>
#include <utils/ico-framework/ico.h>
//Save data
#include <iostream>
#include <fstream>
#include <string.h>
using namespace std;
//Save data
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
	double mc[4];
	//Define global parameters-begin//
	std::vector<double> parameter;
	//Save data
	ofstream outFileicolearning;


	double distance;
	double alpha_tmp;
	double alpha;
	double deri_alpha;

	double distance2;
	double alpha_tmp2;
	double alpha2;
	double deri_alpha2;


	double distance3;
	double alpha_tmp3;
	double alpha3;
	double deri_alpha3;


	double distance4;
	double alpha_tmp4;
	double alpha4;
	double deri_alpha4;


	double input_distance_s;
	double input_distance_s2;
	double input_distance_s3;
	double input_distance_s4;

	double xt_reflex_angle;
	double xt_reflex_angle2;
	double xt_reflex_angle3;
	double xt_reflex_angle4;

	std::vector<double> input_angle_s; //input angle sensors

	double predictive_signal_green;//predictive signal

	//orientation detection in a short distance
	double reflexive_signal_green;//reflex signal

	//orientation detection to the Blue object in a long distance
	double predictive_signal_blue;//predictive signal

	//orientation detection in a short distance
	double reflexive_signal_blue;//reflex signal

	//----Exploration noise---------------//

	double exp_output;
	double exploration_g;
	double exploration_lowpass_g;
	double exploration_lowpass_old_g;

	//ICO learning
	double u_ico_out;
	ICO* ico_controller;
	double w0, w1;


	bool manual_control;

	//Define global parameters-end//

	/// contructor (hint: use $ID$ for revision)
	EmptyController(const std::string& name, const std::string& revision)
	: AbstractController(name, revision){

		//For students, Initialization -begin//
		parameter.resize(2);
		input_angle_s.resize(4);
		ico_controller=new ICO(1, 0.1);

		//For students, Initialization -end//


		//plot values on GUI, ./start -g 1
		addInspectableValue("parameter1", &parameter.at(0),"parameter1");
		addInspectableValue("parameter2", &parameter.at(1),"parameter2");
		addInspectableValue("Dis2Green", &distance2,"distance2");
		addInspectableValue("Dis2Blue", &distance3,"distance3");
		addInspectableValue("NormalizedDistance2G", &input_distance_s2,"NormalizedDistance2G");
		addInspectableValue("PredictOrientationG", &input_angle_s.at(1),"PredictOrientationG");
		addInspectableValue("ReflexOritetationG", &xt_reflex_angle2,"ReflexOritetationG");

		addInspectableValue("NormalizedDistance2B", &input_distance_s3,"NormalizedDistance2B");
		addInspectableValue("PredictOrientationB", &input_angle_s.at(2),"PredictOrientationB");
		addInspectableValue("ReflexOritetationB", &xt_reflex_angle3,"ReflexOritetationB");

		addInspectableValue("u_ico_out", &u_ico_out,"u_ico_out");


		//Save data
		outFileicolearning.open("ICOlearningcurve.txt");

	}

	/** initialization of the controller with the given sensor/ motornumber
      Must be called before use. The random generator is optional.
	 */
	virtual void init(int sensornumber, int motornumber, RandGen* randGen = 0){
		number_sensors = sensornumber;
		number_motors = motornumber;
	};

	/** @return Number of sensors the controller
      was initialised with or 0 if not initialised */
	virtual int getSensorNumber() const {
		return number_sensors;
	};

	/** @return Number of motors the controller
      was initialised with or 0 if not initialised */
	virtual int getMotorNumber() const {
		return number_motors;
	};

	/** performs one step (includes learning).
      Calculates motor commands from sensor inputs.
      @param sensors sensors inputs scaled to [-1,1]
      @param sensornumber length of the sensor array
      @param motors motors outputs. MUST have enough space for motor values!
      @param motornumber length of the provided motor array
	 */
	virtual void step(const sensor* sensors, int sensornumber,
			motor* motors, int motornumber){
		assert(number_sensors == sensornumber);
		assert(number_motors == motornumber);

		manual_control = false;
		// press keyboard "u" = move forward
		// press keyboard "j" = move backward
		// press keyboard "b" = move turn left
		// press keyboard "k" = move turn right
		// press keyboard " " = stop

		/*****************************************************************************************/
		// motors 0-4
		// motor 0 = left front motor
		// motor 1 = right front motor
		// motor 2 = left hind motor
		// motor 3 = right hind motor

		// sensors 0-3: wheel velocity of the corresponding wheel
		// sensor 0 = wheel velocity left front
		// sensor 1 = wheel velocity right front
		// sensor 2 = wheel velocity left hind
		// sensor 3 = wheel velocity right hind

		// sensors 4-11: IR Sensors
		// sensor 4 = front right IR
		// sensor 5 = front left IR
		// sensor 6 = middle hind left IR
		// sensor 7 = middle front left IR
		// sensor 8 = hind left IR
		// sensor 9 = hind right IR
		// sensor 10 = middle hind right IR
		// sensor 11 = middle front right IR


		// sensors 12-23: distance two objects in local coordinates (x,y,z)
		// sensor 12 = x direction to the red object (goal detection sensor)
		// sensor 13 = y direction to the red object (goal detection sensor)
		// sensor 14 = z direction to the red object (goal detection sensor)

		// sensor 15 = x direction to the green object (goal detection sensor)
		// sensor 16 = y direction to the green object (goal detection sensor)
		// sensor 17 = z direction to the green object (goal detection sensor)

		// sensor 18 = x direction to the blue object (goal detection sensor)
		// sensor 19 = y direction to the blue object (goal detection sensor)
		// sensor 20 = z direction to the blue object (goal detection sensor)

		// sensor 21 = x direction to the yellow object (goal detection sensor)
		// sensor 22 = y direction to the yellow object (goal detection sensor)
		// sensor 23 = z direction to the yellow object (goal detection sensor)
		/*****************************************************************************************/

		parameter.at(0) = sensors[4];
		parameter.at(1) = sensors[5];

		//-----------------long distance----Calculating relative orientation to objects-----------------------------------------------//
		//Goal 1 // red

		if (sign(sensors[12 /*x*/]/*X axis*/)>0)
		{
			alpha = atan(sensors[13/*y*/]/sensors[12 /*x*/]) * 180 / M_PI; // angle in degrees
		}
		else
		{ // behind robot angles "continue", directly behind robot is 180 degrees, left is negative, right is positive
			alpha_tmp = -1*atan (sensors[13/*y*/]/sensors[12 /*x*/]) * 180 / M_PI; // angle in degrees
			if (alpha_tmp<=0)
			{ // left
				alpha = (-90 + (-90-alpha_tmp));
			}
			else
			{ // right
				alpha = ( 90 + ( 90-alpha_tmp));
			}
		}
		input_angle_s.at(0) =  alpha/180.*M_PI; // map sensor input to +-pi

		//Goal 2 // Green

		if (sign(sensors[15 /*x*/]/*X axis*/)>0)
		{ // goal is in front
			alpha2 = atan(sensors[16/*y*/]/sensors[15 /*x*/]) * 180 / M_PI; // angle in degrees
		}
		else
		{ // behind robot angles "continue", directly behind robot is 180 degrees, left is negative, right is positive
			alpha_tmp2 = -1*atan (sensors[16/*y*/]/sensors[15/*x*/]) * 180 / M_PI; // angle in degrees
			if (alpha_tmp2<=0)
			{ // left
				alpha2 = (-90 + (-90-alpha_tmp2));
			}
			else
			{ // right
				alpha2 = ( 90 + ( 90-alpha_tmp2));
			}
		}
		input_angle_s.at(1) =  alpha2/180.*M_PI; // map sensor input to +-pi
		predictive_signal_green = input_angle_s.at(1);

		//Goal 3 // Blue

		if (sign(sensors[18 /*x*/]/*X axis*/)>0)
		{ // goal is in front
			alpha3 = atan(sensors[19/*y*/]/sensors[18 /*x*/]) * 180 / M_PI; // angle in degrees
		}

		else
		{ // behind robot angles "continue", directly behind robot is 180 degrees, left is negative, right is positive
			alpha_tmp3 = -1*atan (sensors[19/*y*/]/sensors[18/*x*/]) * 180 / M_PI; // angle in degrees
			if (alpha_tmp3<=0)
			{ // left
				alpha3 = (-90 + (-90-alpha_tmp3));
			}
			else
			{ // right
				alpha3 = ( 90 + ( 90-alpha_tmp3));
			}
		}
		input_angle_s.at(2) =  alpha3/180.*M_PI; // map sensor input to +-pi
		predictive_signal_blue = input_angle_s.at(2);

		//Goal 4 // Yellow


		if (sign(sensors[21 /*x*/]/*X axis*/)>0)
		{ // goal is in front
			alpha4 = atan(sensors[22/*y*/]/sensors[21 /*x*/]) * 180 / M_PI; // angle in degrees
		}
		else
		{ // behind robot angles "continue", directly behind robot is 180 degrees, left is negative, right is positive
			alpha_tmp4 = -1*atan (sensors[22/*y*/]/sensors[21/*x*/]) * 180 / M_PI; // angle in degrees
			if (alpha_tmp4<=0)
			{ // left
				alpha4 = (-90 + (-90-alpha_tmp4));
			}
			else
			{ // right
				alpha4 = ( 90 + ( 90-alpha_tmp4));
			}
		}
		input_angle_s.at(3) =  alpha4/180.*M_PI; // map sensor input to +-pi

		//Red
		distance = (sqrt(pow(sensors[13/*11 y*/],2)+pow(sensors[12/*10 x*/],2)));
		input_distance_s = distance/100;
		//Green
		distance2 = (sqrt(pow(sensors[16/*y*/],2)+pow(sensors[15/*x*/],2)));
		input_distance_s2 = distance2/100;
		//Blue
		distance3 = (sqrt(pow(sensors[19/*y*/],2)+pow(sensors[18/*x*/],2)));
		input_distance_s3 = distance3/100;
		//Yellow

		distance4 = (sqrt(pow(sensors[22/*y*/],2)+pow(sensors[21/*x*/],2)));
		input_distance_s4 = distance4/100;

		//-----------------short distance----Calculating relative orientation to objects-----------------------------------------------//

		//Angle reflex signals
		//1) goal 1 RED


		double range_reflex = 0.1;
		if(input_distance_s < range_reflex/*1.2 ~0.0120 very close to target*/)
		{

			xt_reflex_angle = input_angle_s.at(0);
		}
		else
		{
			xt_reflex_angle = 0.0;
		}


		//2) goal 2 GREEN


		if(input_distance_s2 < range_reflex/*1.2 ~0.0120 very close to target*/)
		{
			xt_reflex_angle2 = input_angle_s.at(1);
		}
		else
		{
			xt_reflex_angle2 = 0.0;
		}
		reflexive_signal_green = xt_reflex_angle2;

		//3) goal 3 BULE

		if(input_distance_s3 <range_reflex/*1.2 ~0.0120 very close to target*/)
		{
			//xt_reflex_angle3 = xt_ico_lowpass3;//input_angle_s.at(2);
			xt_reflex_angle3 = input_angle_s.at(2);
		}
		else
		{
			xt_reflex_angle3 = 0.0;
		}
		reflexive_signal_blue = xt_reflex_angle3;

		//4) goal 4 YELLOW

		if(input_distance_s4 <range_reflex/*1.2 ~0.0120 very close to target*/)
		{
			//xt_reflex_angle4 = xt_ico_lowpass4;//input_angle_s.at(3);
			xt_reflex_angle4 = input_angle_s.at(3);
		}
		else
		{
			xt_reflex_angle4 = 0.0;
		}

		//---------------------Calculating relative distance to objects-----------------------------------------------//


		//input_angle_s.at(0) = Goal 1 red  [-1(-2),...,1(2)]
		//input_angle_s.at(1) = Goal 2 green [-1(-2),...,1(2)]
		//input_angle_s.at(2) = Goal 3 blue [-1(-2),...,1(2)]
		//input_angle_s.at(3) = Goal 4 yellow [-1(-2),...,1(2)]

		//DISTANCE TO GOALS
		//input_distance_s  = normalized distance from Goal 1 [0,...,1]
		//input_distance_s2 = normalized distance from Goal 2 [0,...,1]
		//input_distance_s3 = normalized distance from Goal 3 [0,...,1]
		//input_distance_s4 = normalized distance from Goal 4 [0,...,1]

		//2) Exploration noise

		double lp_gain =  0.99;
		double scale_exploration = 2.0;
		exploration_g = gauss();
		exploration_lowpass_old_g = exploration_lowpass_g;
		exploration_lowpass_g = exploration_lowpass_old_g*lp_gain+(1-lp_gain)*exploration_g;
		exp_output= scale_exploration*exploration_lowpass_g;


		//----Students--------Adding your ICO learning here------------------------------------------//

		//---List of input that you can use for ICO learning task-----//

		//INPUTS
		//orientation detection to the Green object in a long distance
		predictive_signal_green;//predictive signal

		//orientation detection in a short distance
		reflexive_signal_green;//reflex signal

		//orientation detection to the Blue object in a long distance
		predictive_signal_blue;//predictive signal

		//orientation detection in a short distance
		reflexive_signal_blue;//reflex signal

		//ICO module1
		ico_controller->setReflexiveNeuronInput(reflexive_signal_green);
		ico_controller->setPredictiveNeuronInput(0, predictive_signal_green);

		ico_controller->step();

		std::cout << ico_controller->dumpWeights() << std::endl;

		//OUTPUT
		// Output to steer the robot at the moment, the robot is controlled by noise (as exploration or searching for an object)
		u_ico_out = ico_controller->getOutputNeuronOutput();


	      // ico learning --------------------------------------------------------------------//
	      double rate_ico = 0.1;

//	      u_ico_in[0] = k_ico[0]*/*xt_ico_lowpass2*/predictive_signal_green+reflexive_signal_green; // Green
//	      u_ico_in[1] = k_ico[1]*/*xt_ico_lowpass3*/predictive_signal_blue+reflexive_signal_blue;// Blue
//
//	      k_ico[0] += rate_ico*deri_xt_reflex_angle2*input_angle_s.at(1); //Green
//	      k_ico[1] += rate_ico*deri_xt_reflex_angle3*input_angle_s.at(2); //Blue
//
//	      printf(" k_ico[0] = %f :  k_ico[1] = %f\n",  k_ico[0],  k_ico[1]);
//
//	      u_ico_out = 1.0*u_ico_in[0]+1.0*u_ico_in[1]+exp_output;






		outFileicolearning<<predictive_signal_green<<' '<<reflexive_signal_green<<' '
				<<predictive_signal_blue<<' '<<reflexive_signal_blue<<' '<<u_ico_out<<endl;

		//----Students--------Adding your ICO learning here------------------------------------------//


		double scale = 0.5;
		motors[0] = scale*1+u_ico_out; // left front wheel
		motors[1] = scale*1-u_ico_out; // right front wheel
		motors[2]= scale*1+u_ico_out; // left rear wheel
		motors[3] = scale*1-u_ico_out; // right rear wheel

		//-------------------------------------------------------------------------------------------------------------

		// manual steering
		if(manual_control)
		{
			////std::cout<<"u = forward"<<" \n"<<"j = back"<<"\n"<<"b = TL "<<"\n"<<"k = TR"<<"\n"<<"space = stop \n"<<std::endl;
			for (int i=0;i<4;i++)
				motors[i]=mc[i];

		}


		// Example open loop controller:

		//    // turn right in place
		//    motors[0]=  1;
		//    motors[1]= -1;
		//    motors[2]=  1;
		//    motors[3]= -1;

		//    // turn left in place
		//    motors[0]= -1;
		//    motors[1]=  1;
		//    motors[2]= -1;
		//    motors[3]=  1;

	};

	/** performs one step without learning.
      @see step
	 */
	virtual void stepNoLearning(const sensor* , int number_sensors,
			motor* , int number_motors){

	};


	/********* STORABLE INTERFACE ******/
	/// @see Storable
	virtual bool store(FILE* f) const {
		Configurable::print(f,"");
		return true;
	}

	/// @see Storable
	virtual bool restore(FILE* f) {
		Configurable::parse(f);
		return true;
	}


	virtual void setMC(double left, double right){
		mc[0]=left;
		mc[1]=right;
		mc[2]=left;
		mc[3]=right;
	}


	/*************************************************************
	 *  Gaussian random variable: just a sum of uniform distribution
	 *  Average = 0, Variance = 1, (1.2)
	 *************************************************************/
	virtual double gauss()
	{
		double  sum;
		int   i;

		for( sum = i = 0; i < 12; i++) sum += (1.0*(double)rand()/(RAND_MAX));
		return(sum - 6.0);
	}

protected:

	int number_sensors;
	int number_motors;

};

#endif

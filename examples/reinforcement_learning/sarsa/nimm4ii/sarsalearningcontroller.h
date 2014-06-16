// This example of SARSA learning and MRC controller
// for obstacle avoidance
// By Poramate Manoonpong, 15.01.2014


#ifndef __EMPTYCONTROLLER_H
#define __EMPTYCONTROLLER_H


#include <selforg/abstractcontroller.h>
#include <selforg/controller_misc.h>


bool sarsalearning_control = true;// set this for using sarsalearning
bool post_marc_sarsalearning = true;// set this for using pure neural MRC for signal processing
bool lowpassnoise = true;

//Check compare between Braitenberg and MRC with NOT avg input!! to see the effect of MRC

bool mrc_control = false;//false; // set this for using pure neural MRC for motor control
bool avg_input = false; // true = use three IR signals on each side, false = use only at front

bool mrc = true; // true = use mrc control instead of Braitenberg vehicle control
bool Braitenberg = false;// true = use Braitenberg vehicle control
double mc[4];
//testing sarsa learning:  bool mrc_control = false; bool avg_input = false; bool mrc = false; bool Braitenberg = false;
//AND
//bool sarsalearning_control = true;
//bool post_marc_sarsalearning = true;
//bool lowpassnoise = true;



//testing mrc control:  bool mrc_control = true; bool avg_input = false; bool mrc = true; bool Braitenberg = false;
//testing Braitenberg control: bool mrc_control = true; bool avg_input = false; bool mrc = false; bool Braitenberg = true;
//AND
//bool sarsalearning_control = false;
//bool post_marc_sarsalearning = false;
//bool lowpassnoise = false;



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

    //MRC parameters -begin//
    std::vector<double> mrc_input;    //MRC inputs
    std::vector<double> mrc_avg_input;    //MRC average inputs
    std::vector<double> mrc_activity; //MRC neural activities
    std::vector<double> mrc_output;   //MRC neural outputs
    std::vector<double> mrc_input_w;   //MRC neural weight from inputs to MRC neurons
    std::vector< std::vector<double> > mrc_w;       //MRC neural weights
    double mrc_bias;                  //MRC bias

    double distance2;
    double distance3;

    std::vector<double> post_mrc_output;   //postprocessing MRC neural outputs
    //MRC parameters -end//

    //Q learning parameters -begin//
    double getRewardValue(int e);
    double chooseAction(double num, double qvalues);
    //int getMaxAction(int stateRL);
    int action;
    double qvalue[216];
    double Q[3][4]; // 3 actions, 4 states
    int e[4]; //4 states
    int state;
    double average_Photo;
    int  repeating;
    int  repeating1;
    double previous_AverRaw_PhotoLR_LowpassAve;
    int reward;
    int i_RL; // last state
    int a_RL; // last action
    int a1_RL; // action with highest q-value
    int a_RL_old;
    int r_RL; // Last reward
    double rand_RL;

    int number_state;
    int number_action;



    double exploration_g;
    double exploration_lowpass_old_g;
    double exploration_lowpass_g;
    int count;
    double exploration_activation;

    //Q learning parameters -end//


    /// contructor (hint: use $ID$ for revision)
    EmptyController(const std::string& name, const std::string& revision)
    : AbstractController(name, revision){

      count = 0;
      //MRC initialization -begin//
      mrc_input.resize(2);
      mrc_activity.resize(2);
      mrc_output.resize(2);
      mrc_input_w.resize(2);
      mrc_avg_input.resize(2);

      mrc_w.resize(2);
      for(unsigned int i=0; i<mrc_w.size(); i++)
      {
        mrc_w.at(i).resize(2);
      }

      post_mrc_output.resize(2);
      //MRC initialization -end//



      //Q learning initialization -begin//

      //1) Initial action!
      action = 0;
      /**
       * Sets random numbers for a given table of floats.
       * Note: It doesn't return a variable because an array
       * is an object, so changes take place within Q.
       */
      number_state = 4;
      number_action = 3;

      //2) Generate Q table matrix [number of action x number of state] and Initial Q values
      for(int i_s = 0; i_s <number_state; ++i_s) {
        for(int i_a = 0; i_a <number_action; ++i_a) {
          Q[i_a][i_s] = (double)(rand()%1000)/9000.0;//(double)(rand()%1000)/1000.0; // between 0....1
          printf("Q[%d][%d]: %f\n", i_a, i_s, Q[i_a][i_s]);

        }
      }

      e[0] = 0.0;
      e[1] = 0.0;
      e[2] = 0.0;
      e[3] = 0.0;
      repeating = 0;
      repeating1 = 0;
      reward = 0;
      i_RL = -1; // last state
      a_RL = -1; // last action
      a1_RL = 0; // action with highest q-value
      r_RL = 0; // Last reward

      exploration_g = 0.0;
      exploration_lowpass_old_g = 0.0;
      exploration_lowpass_g = 0.0;

      //Q learning initialization -end//


      //plot values
      addInspectableValue("mrc_output0_L", /* &mrc_output.at(0)*/&exploration_g,"mrc_output0");
      addInspectableValue("mrc_output1_R", /*&mrc_output.at(1)*/&exploration_lowpass_g,"mrc_output1");
      addInspectableValue("mrc_input0_L", &mrc_input.at(0),"mrc_input0");
      addInspectableValue("mrc_input1_R", &mrc_input.at(1),"mrc_input1");
      addInspectableValue("postmrc_input0_L", &post_mrc_output.at(0),"postmrc_input0");
      addInspectableValue("postmrc_input1_R", &post_mrc_output.at(1),"postmrc_input1");

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

      // sensor 15 = x direction to the blue object (goal detection sensor)
      // sensor 16 = y direction to the blue object (goal detection sensor)
      // sensor 17 = z direction to the blue object (goal detection sensor)

      // sensor 18 = x direction to the green object (goal detection sensor)
      // sensor 19 = y direction to the green object (goal detection sensor)
      // sensor 20 = z direction to the green object (goal detection sensor)

      // sensor 21 = x direction to the yellow object (goal detection sensor)
      // sensor 22 = y direction to the yellow object (goal detection sensor)
      // sensor 23 = z direction to the yellow object (goal detection sensor)

      /*****************************************************************************************/

      mrc_avg_input.at(0) = ((sensors[5]+sensors[6]+sensors[7])/3)*2-1; // left
      mrc_avg_input.at(1) = ((sensors[4]+sensors[10]+sensors[11])/3)*2-1; // right

      mrc_input.at(0) = sensors[5]*2-1;//left
      mrc_input.at(1) = sensors[4]*2-1;//right


      if(Braitenberg)
      {
        mrc_input_w.at(0) =  7.0;//1.0;
        mrc_input_w.at(1) =  7.0;//1.0;
        mrc_w.at(0).at(0) =  5.4;// 0.0;
        mrc_w.at(1).at(1) =  5.4;// 0.0;
        mrc_w.at(0).at(1) =  0.0;
        mrc_w.at(1).at(0) =  0.0;
        mrc_bias = 0.0;

//        mrc_input_w.at(0) =  1.0;
//        mrc_input_w.at(1) =  1.0;
//        mrc_w.at(0).at(0) =  0.0;
//        mrc_w.at(1).at(1) =  0.0;
//        mrc_w.at(0).at(1) =  0.0;
//        mrc_w.at(1).at(0) =  0.0;
//        mrc_bias = 0.0;

      }
      // MRC controller -begin
      if(mrc)
      {
        mrc_input_w.at(0) =  7.0;
        mrc_input_w.at(1) =  7.0;
        mrc_w.at(0).at(0) =  5.4;//5.6;
        mrc_w.at(1).at(1) =  5.4;//5.6;
        mrc_w.at(0).at(1) =  -3.55;
        mrc_w.at(1).at(0) =  -3.55;
        mrc_bias = 0.0;
      }


      //average sensor signals
      if(avg_input)
      {
          mrc_activity.at(0) = mrc_w.at(0).at(0) * mrc_output.at(0) + mrc_w.at(0).at(1) * mrc_output.at(1) + mrc_bias+mrc_avg_input.at(0)*mrc_input_w.at(0);//left
          mrc_activity.at(1) = mrc_w.at(1).at(1) * mrc_output.at(1) + mrc_w.at(1).at(0) * mrc_output.at(0) + mrc_bias+mrc_avg_input.at(1)*mrc_input_w.at(1);//right
          //printf("Avg\n");
      }

      if(!avg_input)
      {
          mrc_activity.at(0) = mrc_w.at(0).at(0) * mrc_output.at(0) + mrc_w.at(0).at(1) * mrc_output.at(1) + mrc_bias+mrc_input.at(0)*mrc_input_w.at(0);//left
          mrc_activity.at(1) = mrc_w.at(1).at(1) * mrc_output.at(1) + mrc_w.at(1).at(0) * mrc_output.at(0) + mrc_bias+mrc_input.at(1)*mrc_input_w.at(1);//right
          //printf("Not avg\n");

      }

      for(unsigned int i=0; i<mrc_output.size();i++)
      {
        //Braitenberg vehicle
        if(Braitenberg)
        {
          //mrc_output.at(i) = mrc_activity.at(i);
          mrc_output.at(i) = tanh(mrc_activity.at(i));
        }
        //MRC
        if(mrc)
        {
          mrc_output.at(i) = tanh(mrc_activity.at(i));
        }
      }

      // MRC control -end

      //post_mrc_output.at(0) = (sensors[5]+sensors[6]+sensors[7])/3;//(mrc_output.at(0)+1)/2; // left
      //post_mrc_output.at(1) = (sensors[4]+sensors[10]+sensors[11])/3;//(mrc_output.at(1)+1)/2; // right

      double gain;
      gain = 0.0;//0.99;
      post_mrc_output.at(0) = (sensors[5])*(1-gain)+post_mrc_output.at(0)*gain;//(mrc_output.at(0)+1)/2; // left
      post_mrc_output.at(1) = (sensors[4])*(1-gain)+post_mrc_output.at(1)*gain;//(mrc_output.at(1)+1)/2; // right


      //Q learning -begin//
      double LEARN_RATE = 0.7;//0.15;// the learning rate constant
      double EXPLORE_RATE = 0.1;// the rate it tries out random actions
      double discount_factor = 0.9;//0.99;


     //1) State calculation, observe state-------------------------------------------------------------------------------------------------
      if(post_marc_sarsalearning)
      {
        if(post_mrc_output.at(0)>0.2) // left IR
        {
          e[1] = 1.0;
        }
        else
        {
          e[1] = 0.0;
        }
        if(post_mrc_output.at(1)>0.2) // Right IR
        {
          e[2] = 1.0;
        }
        else
        {
          e[2] = 0.0;
        }
      }

      else
      {
        if(sensors[5]>0.2) // left IR  5
        {
          e[1] = 1.0;
        }
        else
          e[1] = 0.0;

        if(sensors[4]>0.2) // Right IR 4
        {
          e[2] = 1.0;
        }
        else
          e[2] = 0.0;
      }


      /*
        e[1] = 0 (not detect),1 (detect); Left obstacle
        e[2] = 0 (not detect),1 (detect); Right obstacle

        2x2 = 4 states
       */

      //state = (int)(e[1] + 2*e[2])

      if(e[1] == 0 && e[2] == 0) // No obstacles
        state = 0;

      if(e[1] == 0 && e[2] == 1) // Obstacle right
        state = 1;

      if(e[1] == 1 && e[2] == 0) // Obstacle left
        state = 2;


      if(e[1] == 1 && e[2] == 1) // Obstacle front
        state = 3;

      //printf("count %d, action: %d  e: %d state %d\n", count, action, e[0], state);

      //2) Reward calculation, observe reward-------------------------------------------------------------------------------------------------

      if(state == 0 && action == 0)
        reward = +1; // positive reware: only no obstacle and move forward
      else
        reward = -1; // negative reward all the time

      //3) Action selection--------------------------------------------------------------------------------------------------------------------
      /*  i_RL = -1; // last state
          a_RL = -1; // last action
          a1_RL = 0; // action with highest q-value
          r_RL = 0; // Last reward
       */

      int j_RL = state;// convert e to a state id number. 0,...,7 , j_RL = new state s'
      a_RL_old = a_RL; //memorize old action, a

      //generate noise, exploration
      rand_RL = (double)(rand()%1000)/1000.0; // between 0....1

      //--Lowpass noise-----
      if(lowpassnoise)
      {

        double  sum;
        sum = rand_RL;
        double lp_gain =  0.8;
        //Exploration gradient output

        exploration_g =  sum;
        exploration_lowpass_old_g = exploration_lowpass_g;
        exploration_lowpass_g = exploration_lowpass_old_g*lp_gain+(1-lp_gain)*exploration_g;

        exploration_activation = exploration_lowpass_g;
        EXPLORE_RATE = 0.7;//0.65;
      }
      else
      {
        exploration_activation = rand_RL;
        EXPLORE_RATE = 0.1;
      }
      ////////E-greedy/////////////////////////////////////////////


      if(exploration_activation < EXPLORE_RATE)
        a_RL = getMaxAction(j_RL); //getMaxAction(j);, a'
      else {
        a_RL = (int) (3.0*rand()/(RAND_MAX+1.0)); //between 0-2
        //a_RL = (int) (9.0*rand()/(RAND_MAX+1.0)); //between 0-8
        //1+(int) (9.0*rand()/(RAND_MAX+1.0));// between 1-9
      }
      ///////////////////////////////////////////////////////////////

     //5) Q value//////////////////
     if(i_RL>=0) {
        r_RL = reward;// reward from taking action a and end up in state s'
        a1_RL = a_RL;//getMaxAction(j_RL);

        Q[a_RL_old][i_RL] = Q[a_RL_old][i_RL] + LEARN_RATE * (r_RL + discount_factor*Q[a1_RL][j_RL] - Q[a_RL_old][i_RL]);


        //a1_RL = getMaxAction(j_RL);// Q learning
        //Q[a_RL_old][i_RL] += r_RL + LEARN_RATE * (Q[a1_RL][j_RL]);
      }

     i_RL = j_RL; //memorize old state


      //printf("Re: %d  OL: %d  OR: %d   Q:%f action: %d : noise %f > %f < %f\n",reward,e[1], e[2], Q[a_RL][i_RL], a_RL, rand_RL, exploration_g, exploration_lowpass_g);

     printf("SARSA learning --> Q values \n\n");

     for(int i_s = 0; i_s <number_state; ++i_s) {
       for(int i_a = 0; i_a <number_action; ++i_a) {

         printf("Q[%d][%d]: %f\n", i_a, i_s, Q[i_a][i_s]);

       }
     }


      //5) Motor command (actions)//////////////////
      //action = 1,..., 9;
      // Select action: 3 possible actions (fwd, rvs, stop) of each wheel and two wheel combinations = 3^2 = 9 actions
      //  action = chooseAction(inputs, qvalues);

      int loop0, loop1, loop2, loop3;

      action = a_RL;

      if(sarsalearning_control)
      {
        switch(action)
        {

          case 0: //Forward

            if(post_marc_sarsalearning)
            {
              for (int i = 0; i < number_motors; i++)
              {
                motors[i]= 0.2;//0.5;//0.1;
              }
              break;
            }
            else {
              for (int i = 0; i < number_motors; i++)
              {
                motors[i]=0.01; //0.05
              }
              break;
            }

          case 1: //Sharp right turn


            if(post_marc_sarsalearning)
            {
              motors[0]=  3;
              motors[1]= -3;
              motors[2]=  3;
              motors[3]= -3;

              break;
            }
            else {
              motors[0]=  3;
              motors[1]= -3;
              motors[2]=  3;
              motors[3]= -3;

              break;
            }

          case 2: //Sharp left turn


            if(post_marc_sarsalearning)
            {
              motors[0]=-3;
              motors[1]= 3;
              motors[2]=-3;
              motors[3]= 3;

              break;
            }
            else {
              motors[0]= -3;
              motors[1]=  3;
              motors[2]= -3;
              motors[3]=  3;

              break;
            }


            //          case 3: //move back
            //
            //             motors[0]= -0.1;
            //             motors[1]= -0.1;
            //             motors[2]= -0.1;
            //             motors[3]= -0.1;
            //
            //             break;

        }
      }

      if(mrc_control)
      {
        motors[0]=  -mrc_output.at(1);
        motors[1]=  -mrc_output.at(0);
        motors[2]=  -mrc_output.at(1);
        motors[3]=  -mrc_output.at(0);
      }

      count++;
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



    //Q learning//
    virtual int getMaxAction(int stateRL) {

      // find the largest Q-value for a given state (j), and return action
      float max = -1000;
      int action = 0;

      for(int a=0;a<number_action;++a) {
        if(Q[a][stateRL] > max) {
          max = Q[a][stateRL];
          action = a;
        }
      }
      return action;
    }
    //Q learning//


    virtual void setMC(double left, double right){
      mc[0]=left;
      mc[1]=right;
      mc[2]=left;
      mc[3]=right;
    }
  protected:

    int number_sensors;
    int number_motors;

};

#endif

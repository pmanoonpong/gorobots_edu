//*******************************************
//*                                         *
//*        Adaptive Locomotion Control      *
//*                                         *
//*******************************************
// update: 03/09/2020
// version: 1.0.0


//*******************************************
//*                                         *
//*               description               *
//*                                         *
//*******************************************
// generate motor signal from open-loop modular neural control
// the leg and body motor signal are seperated


// standard ros library
#include <ros/ros.h>
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include <std_msgs/Int32.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"
#include <rosgraph_msgs/Clock.h>
#include <cmath>
#include <iostream>


// modular robot controller library
#include "Basic_cpg_controller.h"
#include "VRN.h"
#include "Pcpg_controller.h"
#include "Delay_line.h"


//*******************************************
//*                                         *
//*            define parameter             *
//*                                         *
//*******************************************

#define RATE 10                 // reflesh rate of ros
#define GAIT_CHANGE_TIME 20     // time when MI is changed from CPG_MI to CPG_MI2

// cpg parameters
#define CPG_OUTPUT 0.01
#define CPG_BIAS 0.0
#define CPG_W11_22 1.4
#define CPG_WD1 0.18
#define CPG_MI 0.12
#define CPG_MI2 0.12

//** In case of using delay line **//
// MI = 0.04 for wave gait
// MI = 0.08 for new wave gait
// MI = 0.12 for trot gait

// vrn parameters
#define VRN_OUTPUT 0.01
#define VRN_W1358 1.7246
#define VRN_W2467 -1.7246
#define VRN_W9_10 0.5
#define VRN_W11_12 -0.5
#define VRN_BIAS -2.48285

// pcpg parameters
#define PCPG_THRES 0.5      // control upward slope of pcpg signal (0.77 for usin cpg directly)
#define PCPG_1_SLOPE 2      // control the downward slope of pcpg signal
#define PCPG_2_SLOPE 20     
#define PCPG_P_SLOPE 200    
#define PCPG_P_THRES 0.85   

// motor neuron parameter
#define SIGNAL_AMP 0.1      // amplitude of step length
#define LIFT_AMP 0.15       // amplitute of foot lifting
#define LEFT_GAIN 0.97      // relative step length
#define GAMMA 1             // pealing gain

// delayline parameters
#define DELAYSIZE 80        // 80 size of the delay
#define DELAY_RH 1          // 0 delay for right hind leg
#define DELAY_RF 60         // 60 delay for right front leg
#define DELAY_LH 20         // 20 delay for left hind leg
#define DELAY_LF 40         // 40 delay for left front leg
#define DELAY_PEEL 40       // 40

// body amplitude gain
#define BODY_AMP_GAIN 0.3 

// manual or not
#define MANUAL 1            // manual control by keyboard (MANUAL 1) or autonomous mode (MANUAL 0)

//*******************************************
//*                                         *
//*               ros variable              *
//*                                         *
//*******************************************

// buffer (array of float) to store signal before publish to ros
std_msgs::Float32MultiArray cpgSignal;
std_msgs::Float32MultiArray sim_motorSignal;
std_msgs::Float32MultiArray sim_bodySignal;

//*******************************************
//*                                         *
//*            global variable              *
//*                                         *
//*******************************************

float motorSig[16] = {0};             // array store motor signal
float sim_motorSig[16] = {0};         // array store joint signal
float freezeSignal = 0;               // freeze or update

float c1 = 0;                         // cpg signals
float c2 = 0;


float v1 = 0;                         // vrn signals
float v2 = 0;                          
float v3 = 0;                         
float init_vrn_input_y = 1;           // initial input y of vrn network  
float vrn_input_y = 1;                // input y of vrn network

float pc1 = 0;                        // pcpg signals
float pc2 = 0;
float pcp = 0;

float standing_wave_J1 = 0;           // body signal
float standing_wave_J2 = 0;
float standing_wave_J3 = 0;

float sim_time = 0;                   // simulation time

float steering_left = 1;              // steering 
float steering_right = 1;              

float keySignal[4] = {0};             // keyboard signal recieve
float sim_keySignal[4] = {0};         // array store keyboard signal
float keyboard_input = 4;             // 1 left, 2 right, 4 forward, 8 backward (it changes w.r.t keyboard command)

float infraredSignal[2] = {0};        // infrared signal recieve
float irl_lowpass = 0;
float irr_lowpass = 0;
float irl_lowpass_old = 0;
float irr_lowpass_old = 0;
float lowpass_thes = 0.05;
int irl_input = 0;
int irr_input = 0;



float sim_motorDir[16] = {LEFT_GAIN,1,1,LEFT_GAIN,
                             1,1,1,1,
                             1,1,1,1,
                             LEFT_GAIN,1,1,LEFT_GAIN};


//*******************************************
//*                                         *
//*            global function              *
//*                                         *
//*******************************************

void infraredCB(const std_msgs::Float32MultiArray::ConstPtr& array)
{
    int i = 0;
    // print all the remaining numbers
    for(std::vector<float>::const_iterator it = array->data.begin(); it != array->data.end(); ++it)
    {
        infraredSignal[i] = *it;
        i++;
    }
    return;
}

void keyboardCB(const std_msgs::Float32MultiArray::ConstPtr& array)
{
    int i = 0;
    // print all the remaining numbers
    for(std::vector<float>::const_iterator it = array->data.begin(); it != array->data.end(); ++it)
    {
        keySignal[i] = *it;
        i++;
    }
    return;
}

void simTimeCB(const std_msgs::Float32MultiArray::ConstPtr& array)
{
    int i = 0;
    // print all the remaining numbers
    for(std::vector<float>::const_iterator it = array->data.begin(); it != array->data.end(); ++it)
    {
        sim_time = *it;
        i++;
    }
    return;
}

void freezeCB(const std_msgs::Float32MultiArray::ConstPtr& array)
{
    int i = 0;
    // print all the remaining numbers
    for(std::vector<float>::const_iterator it = array->data.begin(); it != array->data.end(); ++it)
    {
        freezeSignal = *it;
        i++;
    }
    return;
}

//*******************************************
//*                                         *
//*              main program               *
//*                                         *
//*******************************************

int main(int argc, char *argv[]){
    // create ros node
    std::string nodeName("slalom");
    ros::init(argc,argv,nodeName);

    // check robot operating system
    if(!ros::master::check())
        ROS_ERROR("ros::master::check() did not pass!");
    ros::NodeHandle node("~");

    ROS_INFO("simROS just started!");

    // set reflesh rate
    ros::Rate* rate;
    rate = new ros::Rate(RATE);
    ros::Rate loop_rate(RATE);

    //*******************************************
    //*                                         *
    //*    define publisher and subscriber      *
    //*                                         *
    //*******************************************

    ros::Publisher outputCPG;
    outputCPG = node.advertise<std_msgs::Float32MultiArray>("/cpg_topic",1);

    ros::Publisher outputSIMMOTOR;
    outputSIMMOTOR = node.advertise<std_msgs::Float32MultiArray>("/sim_motor_topic",1);

    ros::Publisher outputSIMBODY;
    outputSIMBODY = node.advertise<std_msgs::Float32MultiArray>("/sim_body_topic",1);

    ros::Subscriber simTimeSub = node.subscribe("/sim_time_topic",10,simTimeCB);
    ros::Subscriber freezeSub = node.subscribe("/freezeSignal_topic",10,freezeCB);
    ros::Subscriber keyboardSub = node.subscribe("/keyboard_topic",10,keyboardCB);
    ros::Subscriber infraredSub = node.subscribe("/sim_infrared_topic",10,infraredCB);


    //*******************************************
    //*                                         *
    //*      initialized neural control         *
    //*                                         *
    //*******************************************

    // basic cpg
    Basic_cpg_controller basic_cpg;
    basic_cpg.setParameter(CPG_OUTPUT,CPG_BIAS,CPG_W11_22,CPG_WD1,CPG_MI);

    // vrn
    VRN vrn1;
    vrn1.setParameter(VRN_OUTPUT,VRN_W1358,VRN_W2467,VRN_W9_10,VRN_W11_12,VRN_BIAS);

    VRN vrn2;
    vrn2.setParameter(VRN_OUTPUT,VRN_W1358,VRN_W2467,VRN_W9_10,VRN_W11_12,VRN_BIAS);

    VRN vrn3;
    vrn3.setParameter(VRN_OUTPUT,VRN_W1358,VRN_W2467,VRN_W9_10,VRN_W11_12,VRN_BIAS);

    // pcpg
    Pcpg_controller pcpg_1;
    pcpg_1.setParameter(PCPG_1_SLOPE,PCPG_THRES);

    Pcpg_controller pcpg_2;
    pcpg_2.setParameter(PCPG_2_SLOPE,PCPG_THRES);

    Pcpg_controller pcpg_peel;
    pcpg_peel.setParameter(PCPG_P_SLOPE,PCPG_P_THRES);


    // create delay line for each joint
    Delay_line peeling_line;
    peeling_line.setParameter(DELAYSIZE);

    Delay_line joint0_delay;
    joint0_delay.setParameter(DELAYSIZE);

    Delay_line joint1_delay;
    joint1_delay.setParameter(DELAYSIZE);

    Delay_line joint2_delay;
    joint2_delay.setParameter(DELAYSIZE);

    Delay_line joint3_delay;
    joint3_delay.setParameter(DELAYSIZE);

    // create delay line for body delay
    Delay_line body_j1_delay;
    body_j1_delay.setParameter(DELAYSIZE);

    Delay_line body_j2_delay;
    body_j2_delay.setParameter(DELAYSIZE);

    Delay_line body_j3_delay;
    body_j3_delay.setParameter(DELAYSIZE);

    bool updateCondition = true;                 // update or freeze cpg signal


    while(ros::ok())
    {

        //*******************************************
        //*                                         *
        //*               neural control            *
        //*                                         *
        //*******************************************

        if((sim_time > GAIT_CHANGE_TIME) && (GAIT_CHANGE_TIME != 0))
        {
            basic_cpg.setMI(CPG_MI2);
        }

        if(freezeSignal >= 1)
        {
            updateCondition = false;
        }else{
            updateCondition = true;
        }

        // body singal
        if(updateCondition)
        {
          sim_bodySignal.data.clear();
        }

        // cpg signal
        if(updateCondition)
        {
            basic_cpg.run();
        }
        cpgSignal.data.clear();
        c1 = basic_cpg.getSignal(1);
        c2 = basic_cpg.getSignal(2);

        #if !MANUAL
            //*******************************************
            //*                                         *
            //*           Autonomous Mode               *
            //*                                         *
            //*******************************************

            // store and preprocess intrared signal before using it to motify vrn network
            irl_lowpass = tanh(0.1*infraredSignal[0] + 0.99*irl_lowpass_old);
            irr_lowpass = tanh(0.1*infraredSignal[1] + 0.99*irr_lowpass_old);
            irl_lowpass_old = irl_lowpass;
            irr_lowpass_old = irr_lowpass;

            if (irl_lowpass > lowpass_thes && irr_lowpass < lowpass_thes) {irl_input = 1; irr_input = 0;}
            else if (irl_lowpass < lowpass_thes && irr_lowpass > lowpass_thes) {irl_input = 0; irr_input = 1;}
            else if (irl_lowpass > lowpass_thes && irr_lowpass > lowpass_thes) {irl_input = irr_input = 1;}
            else {irl_input = irr_input = 0;}

            if (irl_input == 0 && irr_input == 0) vrn_input_y = 1.0;             // vrn input y for going forward
            else if (irl_input == 1 && irr_input == 1) vrn_input_y = -1.0;       // vrn input y for going backward

            // vrn signal
            if(updateCondition)
            {
                vrn1.run(c1,init_vrn_input_y);
                vrn2.run(c2,init_vrn_input_y);
                vrn3.run(c2,vrn_input_y); 
            }
            v1 = vrn1.getSignal();
            v2 = vrn2.getSignal();

            if (irl_input == 0 && irr_input == 1) {v3 = 0.0*vrn3.getSignal() - 1.0; steering_left = 0.0; steering_right = 1.5;}       // turn left
            else if (irl_input == 1 && irr_input == 0) {v3 = 0.0*vrn3.getSignal() + 1.0; steering_left = 1.5; steering_right = 0.0;}  // trun right
            else {v3 = vrn3.getSignal(); steering_left = steering_right = 1;}   // forward and backwrad depent on vrn output signal

        #else
            //*******************************************
            //*                                         *
            //*       Keyboard Control Mode             *
            //*                                         *
            //*******************************************
            // store and preprocess keboard signal before using it to modify vrn network 
            float temp_keyboard_input = 0.0;
            for(int i = 0;  i < 4;  i++)
            {
                sim_keySignal[i] = keySignal[i];
                temp_keyboard_input += (float)sim_keySignal[i]*pow(2,i);
            }
            keyboard_input = temp_keyboard_input;
            
            if (keyboard_input == 4 ) vrn_input_y = 1.0;             // vrn input y for going forward
            else if (keyboard_input == 8 ) vrn_input_y = -1.0;       // vrn input y for going backward

            // vrn signal
            if(updateCondition)
            {
                vrn1.run(c1,init_vrn_input_y);
                vrn2.run(c2,init_vrn_input_y);
                vrn3.run(c2,vrn_input_y); 
            }
            v1 = vrn1.getSignal();
            v2 = vrn2.getSignal();

            if (keyboard_input == 1) {v3 = 0.0*vrn3.getSignal() - 1.0; steering_left = 0.0; steering_right = 1.5;}       // turn left
            else if (keyboard_input == 2) {v3 = 0.0*vrn3.getSignal() + 1.0; steering_left = 1.5; steering_right = 0.0;}  // trun right
            else {v3 = vrn3.getSignal(); steering_left = steering_right = 1;}   // forward and backwrad depent on vrn output signal
        #endif

        // pcpg and basic pcpg signal
        if(updateCondition)
        {
            pcpg_1.run(v1,v2);
            pcpg_2.run(v1,v2);
            pcpg_peel.run(v1,v2);
        }
        pc1 = pcpg_1.getSignal(1);
        pc2 = pcpg_2.getSignal(1);
        

        //*******************************************
        // ***  write singal to leg dalay line    ***
        //*******************************************
        joint0_delay.writeIn(SIGNAL_AMP*3.14*pc1);

        joint1_delay.writeIn(-0.5*(0.5*(pc2+1.0))-0.6);

        if(pc2 > 0)
        {
            joint2_delay.writeIn(-SIGNAL_AMP*0.0*pc1);
        }else{
            joint2_delay.writeIn(-SIGNAL_AMP*0.0*pc1);
        }

        joint3_delay.writeIn(0.2*0.4*pc1+0.3);


        //*******************************************
        // ***  write singal to body dalay line   ***
        //*******************************************
        body_j1_delay.writeIn(v3);
        body_j2_delay.writeIn(v3);
        body_j3_delay.writeIn(v3);


        //*******************************************
        // ***  read singal leg dalay line        ***
        //*******************************************
        // shoulder joint
        motorSig[12] = steering_left  * joint0_delay.readFr(DELAY_RH);
        motorSig[0]  = steering_left  * joint0_delay.readFr(DELAY_RF);
        motorSig[8]  = steering_right * joint0_delay.readFr(DELAY_LH);
        motorSig[4]  = steering_right * joint0_delay.readFr(DELAY_LF);

        // delay for right hind leg
        motorSig[13] = joint1_delay.readFr(DELAY_RH);
        motorSig[14] = joint2_delay.readFr(DELAY_RH);
        motorSig[15] = joint3_delay.readFr(DELAY_RH);

        // delay for right front leg
        motorSig[1] = joint1_delay.readFr(DELAY_RF);
        motorSig[2] = joint2_delay.readFr(DELAY_RF);
        motorSig[3] = joint3_delay.readFr(DELAY_RF);

        // delay for left hind leg
        motorSig[9]  = joint1_delay.readFr(DELAY_LH) ;
        motorSig[10] = joint2_delay.readFr(DELAY_LH);
        motorSig[11] = joint3_delay.readFr(DELAY_LH);

        // delay for left front leg
        motorSig[5] = joint1_delay.readFr(DELAY_LF) ;
        motorSig[6] = joint2_delay.readFr(DELAY_LF);
        motorSig[7] = joint3_delay.readFr(DELAY_LF);

        //*******************************************
        // ***  read singal body dalay line       ***
        //*******************************************

        // delay for standing wave
        standing_wave_J1 = body_j1_delay.readFr(1);
        standing_wave_J2 = body_j2_delay.readFr(1);
        standing_wave_J3 = body_j3_delay.readFr(1);


        for(int i =0 ;i<4;i++)
        {
            for(int j=0;j<4;j++)
            {
              sim_motorSig[4*i+j] = sim_motorDir[4*i+j] * (motorSig[4*i+j]);
            }
        }


        if(updateCondition)
        {
            joint0_delay.step_one();
            joint1_delay.step_one();
            joint2_delay.step_one();
            joint3_delay.step_one();
            body_j1_delay.step_one();
            body_j2_delay.step_one();
            body_j3_delay.step_one();
        }

        //*******************************************
        //*                                         *
        //*         put data to ros variable        *
        //*                                         *
        //*******************************************
        // drive simulation body
        sim_bodySignal.data.push_back(BODY_AMP_GAIN*standing_wave_J1);
        sim_bodySignal.data.push_back(BODY_AMP_GAIN*standing_wave_J2);
        sim_bodySignal.data.push_back(BODY_AMP_GAIN*standing_wave_J3);

        cpgSignal.data.push_back(c1);
        cpgSignal.data.push_back(c2);
        cpgSignal.data.push_back(v1);
        cpgSignal.data.push_back(v2);

        // drive simulation legs
        sim_motorSignal.data.clear();
        for(int j=0;j<16;j++)
        {
            sim_motorSignal.data.push_back(sim_motorSig[j]);
        }

        outputSIMBODY.publish(sim_bodySignal);
        outputCPG.publish(cpgSignal);
        outputSIMMOTOR.publish(sim_motorSignal);

        // wait
        ros::spinOnce();
        loop_rate.sleep();

    } // main loop -> ros::ok
    return 0;
}

/*
 * NeuralLocomotionControlAdaptiveClimbing.h
 *
 *  Created on: May 2, 2011
 *      Author: poramate
 */

#ifndef NEURALLOCOMOTIONCONTROLADAPTIVECLIMBING_H_
#define NEURALLOCOMOTIONCONTROLADAPTIVECLIMBING_H_

#include <vector>
#include <cmath>
#include <ode_robots/amosiisensormotordefinition.h>
//#include <selforg/amosiisensormotordefinition.h>
//#include "sensor_motor_definition.h"

#include "selforg/matrixutils.h"
#include "selforg/matrix.h"


//Save files /read file
#include <iostream>
#include <fstream>
#include <string.h>

//atof function
#include <stdio.h>
#include <stdlib.h>
#include <math.h>


using namespace std;
//Save files

//3) Class for Neural locomotion control------

class NeuralLocomotionControlAdaptiveClimbing{

  public:

    //---Start Define functions---//
    NeuralLocomotionControlAdaptiveClimbing();
    ~NeuralLocomotionControlAdaptiveClimbing();

    double sigmoid(double num)
    {
      return 1./(1.+exp(-num));
    }

    std::vector<double> step_nlc(const std::vector<double> in0 /*from neural preprocessing [-1 (stance), +1 (swing)]*/, const std::vector<double> in1 /*from neural learning*/);//, bool Footinhibition=false);
    // if several sensor values (like preprocessed or output of memory and learning are used, extend parameters)
    // std::vector<double> step_nlc(const std::vector<double> in0, const std::vector<double> in1, const std::vector<double> in2);

    //---End  Define functions---//



    //---Start Save files---//
    ofstream outFilenlc1;
    ofstream outFilenlc2;
    ofstream outFilenlc3;
    ofstream outFilenlc4;
    ofstream outFilenlc5;
    ofstream outFilenlc6;
    ofstream outFilenlc_tc;
    ofstream outFilenlc_ctr;
    ofstream outFilenlF_fti;
    ofstream outFilenlc_tc_pre;
    ofstream outFilenlc_ctr_pre;
    ofstream outFilenlF_fti_pre;
    ofstream outFilegait;
    ofstream outFilegait2;
    ofstream outFilepower1;
    //---End Save files---//


    //---Start Define vector----//

    //---Input neurons
    std::vector<double> input;

    //---CPG
    std::vector<double> cpg_activity; 					//CPG neural activities
    std::vector<double> cpg_output;						//CPG neural outputs
    std::vector< std::vector<double> > cpg_w; 			//CPG neural weights
    double cpg_bias;									//CPG bias
    double Control_input;

    //---pCPG
    std::vector<double> pcpg_output;					//pCPG neural outputs
    std::vector<double> pcpg_step;						//step function
    std::vector<double> set;			   		        //step function
    std::vector<double> setold;			   		    	//step function
    std::vector<double> diffset;			   		    //step function

    std::vector<double> countup; 						//counter
    std::vector<double> countupold;						//counter
    std::vector<double> countdown;						//counter
    std::vector<double> countdownold; 					//counter

    std::vector<double> deltaxup;						//delta
    std::vector<double> deltaxdown;						//delta

    std::vector<double> xup;							//delta
    std::vector<double> xdown;							//delta

    std::vector<double> yup;							//delta
    std::vector<double> ydown;							//delta


    //---PSN
    std::vector<double> psn_activity; 					//PSN neural activities
    std::vector<double> psn_output;						//PSN neural outputs
    std::vector< std::vector<double> > psn_w;			//PSN neural weights
    std::vector<double> psn_bias;						//PSN bias

    //---VRN
    std::vector<double> vrn_activity; 					//VRN neural activities
    std::vector<double> vrn_output;						//VRN neural outputs
    std::vector< std::vector<double> > vrn_w; 			//VRN neural weights
    double vrn_bias;

    //---Interconnections
    std::vector< std::vector<double> > psn_pcpg_w; 		//PSN neural weights
    std::vector< std::vector<double> > vrn_psn_w; 		//VRN neural weights

    //---Interconnections--input2 to PSN
    std::vector< std::vector<double> > psn_input2_w;    //PSN neural weights
    double vrn_input3_w;
    double vrn_input4_w;

    //---Motor neurons
    std::vector<double> tr_activity; 				    //motor neural activities
    std::vector<double> tl_activity;					//motor neural activities
    std::vector<double> cr_activity; 				    //motor neural activities
    std::vector<double> cl_activity;					//motor neural activities
    std::vector<double> fr_activity; 				    //motor neural activities
    std::vector<double> fl_activity;					//motor neural activities
    std::vector<double> bj_activity;					//motor neural activities

    std::vector<double> tr_output; 				    	//motor neural outputs
    std::vector<double> tl_output;						//motor neural outputs
    std::vector<double> cr_output; 				    	//motor neural outputs
    std::vector<double> cl_output;						//motor neural outputs
    std::vector<double> fr_output; 				   	 	//motor neural outputs
    std::vector<double> fl_output;						//motor neural outputs
    std::vector<double> bj_output;						//motor neural outputs

    std::vector<double> cr_outputold; 					//motor neural outputs_old
    std::vector<double> cl_outputold;					//motor neural outputs_old

    std::vector<double> diffcr_output; 					//diff motor neural outputs
    std::vector<double> diffcl_output;					//diff motor neural outputs

    std::vector<double> postcr;							//postCL motor neural outputs
    std::vector<double> postcl;							//postCR motor neural outputs
    std::vector<double> postcrold;						//postCL motor neural outputs
    std::vector<double> postclold;						//postCR motor neural outputs
    double threshold_c;

    std::vector<double> buffer_t;						//delay buffer vector thorical
    //	std::vector<double> buffer_c;						//delay buffer vector coxa
    //	std::vector<double> buffer_f;						//delay buffer vector fibia
    std::vector<double> buffer_tr;
    std::vector<double>  buffer_tl;
    std::vector<double>  buffer_c;
    std::vector<double>  buffer_f;
    std::vector<double>  buffer_fm;

    std::vector<double> m_pre;							//pre motor outputs (19 motors)
    std::vector<double> m_reflex;						//reflex motor outputs  (19 motors)
    std::vector<double> m_reflex_old;            //reflex motor outputs  (19 motors)
    std::vector<double> m;								//motor outputs as neural activation (19 motors)
    std::vector<double> m_deg;							//motor outputs in deg (19 motors)

    //---Reflex motor neurons
    std::vector<double> fmodel_cmr_activity;			//coxa motor right neural activities
    std::vector<double> fmodel_cmr_output; 				//coxa motor right neural outputs
    std::vector<double> fmodel_cmr_output_old;
    std::vector<double> fmodel_cmr_error;			    //error coxa motor right and foot right
    std::vector<double> fmodel_cmr_errorW;
    std::vector<double> fmodel_cmr_outputfinal; 		//coxa motor right neural outputs
    std::vector<double> low_pass_fmodel_cmr_error_old;
    std::vector<double> low_pass_fmodel_cmr_error;

    std::vector<double> fmodel_cml_activity;			//coxa motor left neural activities
    std::vector<double> fmodel_cml_output; 				//coxa motor left neural outputs
    std::vector<double> fmodel_cml_output_old;
    std::vector<double> fmodel_cml_error;			    //error coxa motor left and foot left
    std::vector<double> fmodel_cml_errorW;
    std::vector<double> fmodel_cml_outputfinal; 		//coxa motor right neural outputs
    std::vector<double> low_pass_fmodel_cml_error_old;
    std::vector<double> low_pass_fmodel_cml_error;

    //---Reflex foot sensors
    std::vector<double> reflex_R_fs;
    std::vector<double> reflex_L_fs;
    std::vector<double> reflex_R_fs_old;//KOH add after the GIT version
    std::vector<double> reflex_L_fs_old;//KOH add after the GIT version
    std::vector<double> countup_reflex_R_fs;//KOH add after the GIT version
    std::vector<double> countdown_reflex_R_fs;//KOH add after the GIT version
    std::vector<double> countup_reflex_R_fs2;//KOH add after the GIT version
    int counter_refelx_R_fs;//KOH add after the GIT version
    int counter_refelx_L_fs;//KOH add after the GIT version
    int max_up, max_down;
    double max_fmodel;//KOH add after the GIT version
    std::vector<double>  dervi_reflex_R_fs;
    std::vector<double>  dervi_reflex_L_fs;


    //Learning forward models to expected foot sensors
    std::vector<double> lr_fmodel_cr;					//learning rate
    std::vector<double> fmodel_cmr_w;					//forward model weights
    std::vector<double> fmodel_fmodel_cmr_w;			//forward model recurrent weights
    std::vector<double> fmodel_post_cmr_w;				//forward model postprocessing weights
    std::vector<double> fmodel_cmr_bias;				//forward model biases
    std::vector<double>  acc_cmr_error;					//forward model biases
    std::vector<double>  acc_cmr_error_old;				//forward model biases
    std::vector<double>  deri_acc_cmr_error;			//forward model biases
    std::vector<double>  acc_cmr_error_elev;			//error for elevator reflex
    std::vector<double>  error_cmr_elev;				//error for elevator reflex
    std::vector<double>  acc_cmr_error_posi_neg;		//forward model biases
    std::vector<double>  dervi_fmodel_cmr_output;
    std::vector<double>  max_error_cmr_pre_step;

    std::vector<double> lr_fmodel_cl;					//learning rate
    std::vector<double> fmodel_cml_w;					//forward model weights
    std::vector<double> fmodel_fmodel_cml_w;			//forward model recurrent weights
    std::vector<double> fmodel_post_cml_w;				//forward model postprocessing weights
    std::vector<double> fmodel_cml_bias;				//forward model biases
    std::vector<double>  acc_cml_error;					//forward model biases
    std::vector<double>  acc_cml_error_old;				//forward model biases
    std::vector<double>  deri_acc_cml_error;			//forward model biases
    std::vector<double>  acc_cml_error_elev;			//error for elevator reflex
    std::vector<double>  error_cml_elev;				//error for elevator reflex
    std::vector<double>  acc_cml_error_posi_neg;		//forward model biases
    std::vector<double>  dervi_fmodel_cml_output;
    std::vector<double>  max_error_cml_pre_step;


    std::vector<double> lowpass_cmr_error_activity;		//lowpass neuron activities
    std::vector<double> lowpass_cmr__error_output; 		//lowpass neuron outputs
    std::vector<double> lowpass_cmr_w;					//lowpass weights
    std::vector<double> lowpass_lowpass_cmr_w;			//lowpass recurrent weights
    std::vector<double> lowpass_cmr_bias;				//lowpass biases

    std::vector<double> lowpass_cml_error_activity;		//lowpass neuron activities
    std::vector<double> lowpass_cml__error_output; 		//lowpass neuron outputs
    std::vector<double> lowpass_cml_w;					//lowpass weights
    std::vector<double> lowpass_lowpass_cml_w;			//lowpass recurrent weights
    std::vector<double> lowpass_cml_bias;				//lowpass biases


    std::vector<double> a1_r;							//Emd_2D, learning parameters
    std::vector<double> a2_r;							//Emd_2D, learning parameters
    std::vector<double> a3_r;							//Emd_2D, learning parameters
    std::vector<double> fcn_r; 							//Emd_2D
    std::vector<double> fac_r; 							//Emd_2D
    std::vector<double> normxsq_r;						//Emd_2D
    std::vector<double> pred_r;
    std::vector<double> a1_l;							//Emd_2D, learning parameters
    std::vector<double> a2_l;							//Emd_2D, learning parameters
    std::vector<double> a3_l;							//Emd_2D, learning parameters
    std::vector<double> fcn_l; 							//Emd_2D
    std::vector<double> fac_l; 							//Emd_2D
    std::vector<double> normxsq_l;						//Emd_2D
    std::vector<double> pred_l;
    std::vector<double> delay_CR0;						//Delay signal
    std::vector<double> delay_CR1;						//Delay signal
    std::vector<double> delay_CR2;						//Delay signal
    std::vector<double> delay_CL0;						//Delay signal
    std::vector<double> delay_CL1;						//Delay signal
    std::vector<double> delay_CL2;						//Delay signal
    std::vector<double> m_pre_delay;

    //Learning forward models to expected foot sensors ESN
    std::vector<double> fmodel_cmr_output_rc;
    std::vector<double> fmodel_cml_output_rc;
    std::vector<double> fmodel_cmr0_output_rc;
    std::vector<double> fmodel_cmr1_output_rc;
    std::vector<double> fmodel_cmr2_output_rc;
    std::vector<double> fmodel_cml0_output_rc;
    std::vector<double> fmodel_cml1_output_rc;
    std::vector<double> fmodel_cml2_output_rc;


    //Learning LTM models to memorize foot sensors from ESN
    std::vector<double> fmodel_cmr_output_ltm;
    std::vector<double> fmodel_cml_output_ltm;
    std::vector<double> cmr0_ltm_neuron;
    std::vector<double> cmr0_ltm_neuron_w;
    std::vector<double> fmodel_cmr_output_w2;
    std::vector<double> input_ltm;

    matrix::Matrix * inputs_ltm_out;
    matrix::Matrix * outputs_ltm_out;
    matrix::Matrix * weights_ltm_out;

    matrix::Matrix * inputs_ltm_in;
    matrix::Matrix * outputs_ltm_in;
    matrix::Matrix * weights_ltm_in;

    matrix::Matrix * inputs_ltm_hid;
    matrix::Matrix * outputs_ltm_hid;
    matrix::Matrix * weights_ltm_hid;

    matrix::Matrix * inputs_ltm_bi;
    matrix::Matrix * outputs_ltm_bi;
    matrix::Matrix * weights_ltm_bi;

    //P,I,D control for searching
    std::vector<double> kp_r;
    std::vector<double> ki_r;
    std::vector<double> kd_r;
    std::vector<double> kp_l;
    std::vector<double> ki_l;
    std::vector<double> kd_l;
    std::vector<double> int_l;
    std::vector<double> int_r;
    std::vector<double> d_l;
    std::vector<double> d_r;

    //Motor mapping

    double min_tc; // network output range
    double max_tc;// network output range
    //Adjust
    double min_tc_f_nwalking_deg; //deg **
    double max_tc_f_nwalking_deg; //deg **
    double min_tc_f_nwalking;
    double max_tc_f_nwalking;

    //TC_middle
    //Adjust
    double min_tc_m_nwalking_deg; //deg **
    double max_tc_m_nwalking_deg; //deg **
    double min_tc_m_nwalking;
    double max_tc_m_nwalking;

    //TC_rear
    //Adjust
    double min_tc_r_nwalking_deg; //deg **
    double max_tc_r_nwalking_deg; //deg **
    double min_tc_r_nwalking;
    double max_tc_r_nwalking;

    std::vector<double>  min_tcr_nwalking;
    std::vector<double>  max_tcr_nwalking;
    std::vector<double>  offset_tcr;
    std::vector<double>  offset_tcr_downward; //KOH-->Eduard

    std::vector<double>  min_tcl_nwalking;
    std::vector<double>  max_tcl_nwalking;
    std::vector<double>  offset_tcl;
    std::vector<double>  offset_tcl_downward; //KOH-->Eduard

    //CTR joints
    double min_ctr; // network output range
    double max_ctr;// network output range

    std::vector<double>  min_ctr_nwalking_deg;//deg
    std::vector<double>  max_ctr_nwalking_deg;//deg
    std::vector<double>  min_ctr_nwalking;
    std::vector<double>  max_ctr_nwalking;
    std::vector<double>  offset_ctr;
    std::vector<double>  offset_ctr_downward; //KOH-->Eduard

    std::vector<double>  min_ctl_nwalking_deg;//deg
    std::vector<double>  max_ctl_nwalking_deg;//deg
    std::vector<double>  min_ctl_nwalking;
    std::vector<double>  max_ctl_nwalking;
    std::vector<double>  offset_ctl;
    std::vector<double>  offset_ctl_downward; //KOH-->Eduard

    //FTI joints
    double min_fti; // network output range
    double max_fti; // network output range

    std::vector<double>  min_ftir_nwalking_deg;//deg
    std::vector<double>  max_ftir_nwalking_deg;//deg
    std::vector<double>  min_ftir_nwalking;
    std::vector<double>  max_ftir_nwalking;
    std::vector<double>  offset_ftir;
    std::vector<double>  offset_ftir_downward;

    std::vector<double>  min_ftil_nwalking_deg;//deg
    std::vector<double>  max_ftil_nwalking_deg;//deg
    std::vector<double>  min_ftil_nwalking;
    std::vector<double>  max_ftil_nwalking;
    std::vector<double>  offset_ftil;
    std::vector<double>  offset_ftil_downward; //KOH-->Eduard


    //BJ joint
    double min_bj; // network output range
    double max_bj;// network output range
    double min_bj_fwalking_deg; //deg
    double max_bj_fwalking_deg; //deg
    double min_bj_fwalking;
    double max_bj_fwalking;
    double offset_bj; // gap crossing

    double max_c; // max range
    double max_f; // max range
    double max_c_offset; // Adjust from 45,...185=> 185 (cin= 0.02, wave), 45 (cin= 0.18, tripod)
    double max_f_offset; // Adjust from 45,...185=> 185 (cin= 0.02, wave), 45 (cin= 0.18, tripod)


    //Reading motor signals from Text
    vector<double> m_r0_t;
    vector<double> m_r1_t;
    vector<double> m_r2_t;
    vector<double> m_l0_t;
    vector<double> m_l1_t;
    vector<double> m_l2_t;

    double m_r0_t_old;
    double m_r1_t_old;
    double m_r2_t_old;
    double m_l0_t_old;
    double m_l1_t_old;
    double m_l2_t_old;

    double m_r0_text;
    double m_r1_text;
    double m_r2_text;
    double m_l0_text;
    double m_l1_text;
    double m_l2_text;



    int i_text_loop;
    int ii;
    bool initialized;

    //Gait analysis
    double stance_time;
    double swing_time;
    double stride_period;
    double duty_factor;

    //Current sensors and power
    double ac_motor;
    double i_motor;
    double motor_power_con;

    //Inclinometer sensors
    double incli_x;
    double incli_y;

    //Position sensors
    double BX_pos_sensor;
    double BY_pos_sensor;
    double BZ_pos_sensor;

    //Body speed sensors
    double BX_spd_sensor;
    double BY_spd_sensor;
    double BZ_spd_sensor;


    //For controlling elevator reflex when forward model used
    double elevator_th;
    //---End Define vector----//

    double old_pcpg;
    int total_c;
    double max_scale;


  private:

    double  h;
    double count1;
    double count2;

    int T1;
    int T2;
    int T1old;
    int T2old;
    int period1;
    int period2;
    double y1;
    double y2;
    std::vector<double> triH1; //triangle output vector
    std::vector<double> triH2;

    int tau;
    int tau_l;
    int time;

    int option_wiring;
    std::vector<double> counter_cr;
    std::vector<double> counter_cl;

    int global_count;
    int allfoot_off_ground;

    int option_fmodel;
    bool switchon_ED;
    bool switchon_footinhibition;
    bool switchon_reflexes;
    bool switchon_purefootsignal;
    bool switchon_learnweights;
    bool softlanding;
    bool reading_text_testing;
    bool switchon_less_reflexes;
    bool elevator_reflexes;
    bool lift_body_up;
    bool switchoff_searching_reflexes;
    bool sequentiral_learning;
    bool learn;
    bool ltm_v1;
    bool ltm_v2;
    bool ltm_v3;
    bool loadweight;
    bool crossing_gap;
    bool use_pre_step_to_adjust_searching;

};

#endif /* NEURALLOCOMOTIONCONTROLADAPTIVECLIMBING_H_ */

/*
 * NeuralLocomotionControlAdaptiveClimbing.cpp
 *
 *  Created on: May 8, 2012 update, modified date July 26 2012
 *      Author: poramate manoonpong
 *
 *      One step = 0.0372 s
 *      The update frequency of this controller is 26 .8 Hz
 *      in fact 28.5 Hz
 *      Converting from time steps to time [second] by
 *      e.g., 400 time steps * 0.0372 = 14.88 seconds
 *
 *      19 April 2013 last update test1
 */

#include "NeuralLocomotionControlAdaptiveClimbing.h"

//Add ENS network--(1)
#include <esn-framework/networkmatrix.h>



//-----ESN network-----//

//---------------------ESN module 1
ESNetwork * ESN_R0;
float * ESinput_R0;
float * ESTrainOutput_R0;

//---------------------ESN module 2

ESNetwork * ESN_R1;
float * ESinput_R1;
float * ESTrainOutput_R1;


//---------------------ESN module 3

ESNetwork * ESN_R2;
float * ESinput_R2;
float * ESTrainOutput_R2;


//---------------------ESN module 4

ESNetwork * ESN_L0;
float * ESinput_L0;
float * ESTrainOutput_L0;

//---------------------ESN module 5

ESNetwork * ESN_L1;
float * ESinput_L1;
float * ESTrainOutput_L1;

//---------------------ESN module 6

ESNetwork * ESN_L2;
float * ESinput_L2;
float * ESTrainOutput_L2;

//------LTM
int NUM_LTM_R0 = 88;//34;//34 = 0.15 c_input, 88 = 0.03 c_input
int count_neuron = 0;
int count_neuron_learning = 0;
bool ltm_start = false;


bool singlegait = false; //false = learn 3 gaits, ture = learn only one gait

//Three selected gaits
double gait1 = 0.04;// fast wave gait for loose terrain
double gait2 = 0.06;// tetrapod gait for rough terrain
double gait3 = 0.09;// caterpillar gait for gap crossing
int t_change_gait1 = 3000; // each gait is learned for 3000 time steps
int t_change_gait2 = 6000;
int t_change_gait3 = 9000;


//3) Step function of Neural locomotion control------

NeuralLocomotionControlAdaptiveClimbing::NeuralLocomotionControlAdaptiveClimbing(){

  /*******************************************************************************
   *  CONTROL OPTION!!!!
   *******************************************************************************/
  //Wiring connection
  option_wiring = 2; //1==Tripod, 2==Delayed line)

  //Selecting forward models
  option_fmodel = 6; // 4 or 6
  sequentiral_learning = true;// learn multiple gait one after the other, false = learn only one gait



  //1 == with threshold after fmodel & NO lowpass neuron after error;
  //2 == without threshold after fmodel & with lowpass neuron after error)
  //3 == delay embedded systems 2D
  //4 == with gradient error learning after fmodel & with lowpass neuron after error _On paper****** USED
  //5 == with simple clip function
  //6 == RC network*********************************************************************************USED

  //Switch on delay embedded systems 2D
  switchon_ED = false;

  //Switch on or off reflexes
  switchon_reflexes = true;// true==on after learning or when uses learned weights, false == off during learning
  use_pre_step_to_adjust_searching = true; // for effective rough terraihn walking!! always set to "true" leg will extend more using previous acc_error!
  max_scale = 100; // if set to large value e.g., 100 or 200 the extension of leg will have less effect from previous step--> extend less

  //Switch on pure foot signal
  switchon_purefootsignal = false; // 1==on using only foot signal, 0 == using forward model & foot signal

  //Used learn weights
  switchon_learnweights = false;// 1== used learn weight, 0 == initial weights = 0.0 and let learning develops weights

  //Switch on foot inhibition
  switchon_footinhibition = false; //true = hind foot inhibit front foot, false;

  //Switch on soft landing  = reset to normal walking as  soon as acc error = 0
  softlanding = false;//true;

  elevator_reflexes = true;//false;
  switchoff_searching_reflexes = false;

  switchon_less_reflexes = true;// = very high leg extension during searching reflex

  lift_body_up = false;// if set to true = lifting the bod up above ground, if set to false = not lefting

  //Testing controller from text (e.g. SOINN control as motor memory network)
  reading_text_testing = false;

  crossing_gap = true; // if set gap crossing --> on, searching and elevator reflex have to switch off

  if(crossing_gap)
  {
    switchon_reflexes = false;
    elevator_reflexes = false;
    switchoff_searching_reflexes = true;

  }

  //RC network setup---------------------------------------------------------------//
  loadweight = true; // true = use learned weights, false = let the RC learn
  learn = false; // true = learning, false = use learned weights

  //LTM option
  ltm_v1 = false;//true; // learn pattern
  ltm_v2 = true;//true;  // learn frequency
  ltm_v3 = false;//false;  // learn weights

  //Save files
  outFilenlc1.open("NeurallocomotionR0.dat");
  outFilenlc2.open("NeurallocomotionR1.dat");
  outFilenlc3.open("NeurallocomotionR2.dat");
  outFilenlc4.open("NeurallocomotionL0.dat");
  outFilenlc5.open("NeurallocomotionL1.dat");
  outFilenlc6.open("NeurallocomotionL2.dat");
  outFilenlc_tc.open("NeurallocomotionTC.dat");
  outFilenlc_ctr.open("NeurallocomotionCTR.dat");
  outFilenlF_fti.open("NeurallocomotionFTI.dat");
  outFilenlc_tc_pre.open("NeurallocomotionTC_pre.dat");
  outFilenlc_ctr_pre.open("NeurallocomotionCTR_pre.dat");
  outFilenlF_fti_pre.open("NeurallocomotionFTI_pre.dat");
  outFilegait.open("NeurallocomotionGait.dat");
  outFilegait2.open("NeurallocomotionGait2.dat");
  outFilepower1.open("NeurallocomotionPower.dat");

  /*******************************************************************************
   * Set vector size
   *******************************************************************************/
  //---Set vector size----//

  //Input vector size
  input.resize(5);

  //CPG vector sizes
  cpg_activity.resize(2);
  cpg_output.resize(2);

  cpg_w.resize(2);
  for(unsigned int i=0; i<cpg_w.size(); i++)
  {
    cpg_w.at(i).resize(2);
  }


  //pCPG vector sizes
  pcpg_output.resize(2);
  pcpg_step.resize(2);
  set.resize(2);
  setold.resize(2);
  diffset.resize(2);
  countup.resize(2);
  countupold.resize(2);
  countdown.resize(2);
  countdownold.resize(2);
  deltaxup.resize(2);
  deltaxdown.resize(2);
  xup.resize(2);
  xdown.resize(2);
  yup.resize(2);
  ydown.resize(2);

  //PSN vector sizes
  psn_activity.resize(12);
  psn_output.resize(12);

  psn_w.resize(12);
  for(unsigned int i=0; i<psn_w.size(); i++)
  {
    psn_w.at(i).resize(12);
  }
  psn_bias.resize(3);

  //VRN vector sizes
  vrn_activity.resize(14);
  vrn_output.resize(14);

  vrn_w.resize(14);
  for(unsigned int i=0; i<vrn_w.size(); i++)
  {
    vrn_w.at(i).resize(14);
  }

  //Interconnections vector sizes
  psn_pcpg_w.resize(14);
  for(unsigned int i=0; i<psn_pcpg_w.size(); i++)
  {
    psn_pcpg_w.at(i).resize(14);
  }

  vrn_psn_w.resize(14);
  for(unsigned int i=0; i<vrn_psn_w.size(); i++)
  {
    vrn_psn_w.at(i).resize(14);
  }

  psn_input2_w.resize(2);
  for(unsigned int i=0; i<psn_input2_w.size(); i++)
  {
    psn_input2_w.at(i).resize(2);
  }

  tr_activity.resize(3);
  tl_activity.resize(3);
  cr_activity.resize(3);
  cl_activity.resize(3);
  fr_activity.resize(3);
  fl_activity.resize(3);
  bj_activity.resize(1);

  tr_output.resize(3);
  tl_output.resize(3);
  cr_output.resize(3);
  cl_output.resize(3);
  fr_output.resize(3);
  fl_output.resize(3);
  bj_output.resize(1);

  cr_outputold.resize(3);
  cl_outputold.resize(3);
  diffcr_output.resize(3);
  diffcl_output.resize(3);
  postcr.resize(3);
  postcl.resize(3);
  postcrold.resize(3);
  postclold.resize(3);

  buffer_t.resize(96);
  buffer_c.resize(96);
  buffer_f.resize(96);

  buffer_tr.resize(96);
  buffer_tl.resize(96);
  buffer_fm.resize(96);


  m_pre.resize(19);
  m_reflex.resize(19);
  m_reflex_old.resize(19);
  m.resize(19);
  m_deg.resize(19);


  //---Reflex motor neurons
  fmodel_cmr_activity.resize(3);
  fmodel_cmr_output.resize(3);
  fmodel_cmr_output_old.resize(3);
  fmodel_cmr_error.resize(3);
  fmodel_cmr_errorW.resize(3);

  fmodel_cml_activity.resize(3);
  fmodel_cml_output.resize(3);
  fmodel_cml_output_old.resize(3);
  fmodel_cml_error.resize(3);
  fmodel_cml_errorW.resize(3);

  fmodel_cmr_outputfinal.resize(3);
  fmodel_cml_outputfinal.resize(3);

  //---Reflex foot sensors
  reflex_R_fs.resize(3);
  reflex_L_fs.resize(3);
  reflex_R_fs_old.resize(3);
  reflex_L_fs_old.resize(3);
  countup_reflex_R_fs.resize(3);
  countdown_reflex_R_fs.resize(3);
  countup_reflex_R_fs2.resize(3);
  dervi_reflex_R_fs.resize(3);
  dervi_reflex_L_fs.resize(3);

  //Learning forward models to expected foot sensors
  fmodel_cmr_w.resize(3);
  fmodel_fmodel_cmr_w.resize(3);
  fmodel_post_cmr_w.resize(3);
  fmodel_cmr_bias.resize(3);
  acc_cmr_error.resize(3);
  acc_cmr_error_old.resize(3);
  deri_acc_cmr_error.resize(3);
  acc_cmr_error_elev.resize(3);
  error_cmr_elev.resize(3);
  lr_fmodel_cr.resize(3);
  counter_cr.resize(3);
  acc_cmr_error_posi_neg.resize(3);
  dervi_fmodel_cmr_output.resize(3);
  low_pass_fmodel_cmr_error_old.resize(3);
  low_pass_fmodel_cmr_error.resize(3);
  max_error_cmr_pre_step.resize(3);

  fmodel_cml_w.resize(3);
  fmodel_fmodel_cml_w.resize(3);
  fmodel_post_cml_w.resize(3);
  fmodel_cml_bias.resize(3);
  acc_cml_error.resize(3);
  acc_cml_error_old.resize(3);
  deri_acc_cml_error.resize(3);
  acc_cml_error_elev.resize(3);
  error_cml_elev.resize(3);
  lr_fmodel_cl.resize(3);
  counter_cl.resize(3);
  acc_cml_error_posi_neg.resize(3);
  dervi_fmodel_cml_output.resize(3);
  low_pass_fmodel_cml_error_old.resize(3);
  low_pass_fmodel_cml_error.resize(3);
  max_error_cml_pre_step.resize(3);

  lowpass_cmr_error_activity.resize(3);
  lowpass_cmr__error_output.resize(3);
  lowpass_cmr_w.resize(3);
  lowpass_lowpass_cmr_w.resize(3);
  lowpass_cmr_bias.resize(3);

  lowpass_cml_error_activity.resize(3);
  lowpass_cml__error_output.resize(3);
  lowpass_cml_w.resize(3);
  lowpass_lowpass_cml_w.resize(3);
  lowpass_cml_bias.resize(3);

  //Emd_2D another mechanism for forward models
  a1_r.resize(3);
  a2_r.resize(3);
  a3_r.resize(3);
  fcn_r.resize(3);
  fac_r.resize(3);
  pred_r.resize(3);
  normxsq_r.resize(3);
  a1_l.resize(3);
  a2_l.resize(3);
  a3_l.resize(3);
  fcn_l.resize(3);
  fac_l.resize(3);
  pred_l.resize(3);
  normxsq_l.resize(3);
  delay_CR0.resize(96);
  delay_CR1.resize(96);
  delay_CR2.resize(96);
  delay_CL0.resize(96);
  delay_CL1.resize(96);
  delay_CL2.resize(96);
  m_pre_delay.resize(19);


  //Motor ranges Min, Max

  min_tcr_nwalking.resize(3);
  max_tcr_nwalking.resize(3);
  offset_tcr.resize(3);
  offset_tcr_downward.resize(3); //KOH-->Eduard

  min_tcl_nwalking.resize(3);
  max_tcl_nwalking.resize(3);
  offset_tcl.resize(3);
  offset_tcl_downward.resize(3); //KOH-->Eduard

  min_ctr_nwalking_deg.resize(3);
  max_ctr_nwalking_deg.resize(3);
  min_ctr_nwalking.resize(3);
  max_ctr_nwalking.resize(3);
  offset_ctr.resize(3);
  offset_ctr_downward.resize(3); //KOH-->Eduard

  min_ctl_nwalking_deg.resize(3);
  max_ctl_nwalking_deg.resize(3);
  min_ctl_nwalking.resize(3);
  max_ctl_nwalking.resize(3);
  offset_ctl.resize(3);
  offset_ctl_downward.resize(3); //KOH-->Eduard

  min_ftir_nwalking_deg.resize(3);
  max_ftir_nwalking_deg.resize(3);
  min_ftir_nwalking.resize(3);
  max_ftir_nwalking.resize(3);
  offset_ftir.resize(3);
  offset_ftir_downward.resize(3); //KOH-->Eduard

  min_ftil_nwalking_deg.resize(3);
  max_ftil_nwalking_deg.resize(3);
  min_ftil_nwalking.resize(3);
  max_ftil_nwalking.resize(3);
  offset_ftil.resize(3);
  offset_ftil_downward.resize(3); //KOH-->Eduard

  //Using save text
  m_r0_t.resize(8000);
  m_r1_t.resize(8000);
  m_r2_t.resize(8000);
  m_l0_t.resize(8000);
  m_l1_t.resize(8000);
  m_l2_t.resize(8000);

  //---PID control------//
  kp_r.resize(3);
  ki_r.resize(3);
  kd_r.resize(3);
  kp_l.resize(3);
  ki_l.resize(3);
  kd_l.resize(3);
  int_l.resize(3);
  int_r.resize(3);
  d_l.resize(3);
  d_r.resize(3);
  //---PID control------//

  /*******************************************************************************
   *  Initial parameters
   *******************************************************************************/
  //---Initial parameters----//
  //---Inputs// 0, or 1

  //  input.at(0) = 1;
  //  input.at(1) = 1;
  //  input.at(2) = 1;
  //  input.at(3) = 1;
  //  input.at(4) = 1;

  input.at(0) = 1; //tc JOINT inhibition = 0
  input.at(1) = 1; //FL0, FR1, FR2 control NOT USED
  input.at(2) = 1; //PSN control
  input.at(3) = -1; //turn left = 1,
  input.at(4) = -1; //turn right = 1,


  //---CPG weights

  cpg_activity.at(0) = 0.1; // Initialization
  cpg_activity.at(1) = 0.1; // Initialization
  cpg_output.at(0) = 0.1; // Initialization
  cpg_output.at(1) = 0.1; // Initialization

  Control_input = 0.04;//0.05;
  cpg_w.at(0).at(0) =  1.5;
  cpg_w.at(0).at(1) =  0.4;
  cpg_w.at(1).at(0) =  -0.4;
  cpg_w.at(1).at(1) =  1.5;
  //network bias
  cpg_bias =        0.0;//0.01;


  //---PSN weights

  psn_w.at(2).at(0) = -5.0;
  psn_w.at(3).at(1) = -5.0;
  psn_w.at(4).at(0) = -5.0;
  psn_w.at(5).at(1) = -5.0;
  psn_w.at(6).at(2) =  0.5;
  psn_w.at(7).at(3) =  0.5;
  psn_w.at(8).at(4) =  0.5;
  psn_w.at(9).at(5) =  0.5;
  psn_w.at(10).at(6) = 3.0;
  psn_w.at(10).at(7) = 3.0;
  psn_w.at(11).at(8) = 3.0;
  psn_w.at(11).at(9) = 3.0;
  //network bias
  psn_bias.at(0) =  1;
  psn_bias.at(1) =  0.5;
  psn_bias.at(2) = -1.35;

  //---VRN weights

  vrn_w.at(4).at(0) =    1.7246;
  vrn_w.at(4).at(1) =    1.7246;
  vrn_w.at(5).at(0) =   -1.7246;
  vrn_w.at(5).at(1) =   -1.7246;
  vrn_w.at(6).at(0) =    1.7246;
  vrn_w.at(6).at(1) =   -1.7246;
  vrn_w.at(7).at(0) =   -1.7246;
  vrn_w.at(7).at(1) =    1.7246;
  vrn_w.at(8).at(2) =    1.7246;
  vrn_w.at(8).at(3) =    1.7246;
  vrn_w.at(9).at(2) =   -1.7246;
  vrn_w.at(9).at(3) =   -1.7246;
  vrn_w.at(10).at(2) =   1.7246;
  vrn_w.at(10).at(3) =  -1.7246;
  vrn_w.at(11).at(2) =  -1.7246;
  vrn_w.at(11).at(3) =   1.7246;
  vrn_w.at(12).at(4) =   0.5;
  vrn_w.at(12).at(5) =   0.5;
  vrn_w.at(12).at(6) =  -0.5;
  vrn_w.at(12).at(7) =  -0.5;
  vrn_w.at(13).at(8) =   0.5;
  vrn_w.at(13).at(9) =   0.5;
  vrn_w.at(13).at(10) = -0.5;
  vrn_w.at(13).at(11) = -0.5;
  //network bias
  vrn_bias =       -2.48285;


  //pCPG to PSN connection weights
  psn_pcpg_w.at(2).at(0) = 0.5;
  psn_pcpg_w.at(3).at(1) = 0.5;
  psn_pcpg_w.at(4).at(1) = 0.5;
  psn_pcpg_w.at(5).at(0) = 0.5;

  //PSN to VRN connection weights
  vrn_psn_w.at(0).at(11) = 1.75;
  vrn_psn_w.at(2).at(11) = 1.75;


  //input2 to PSN connection weights
  psn_input2_w.at(0).at(0) = -1.0;
  psn_input2_w.at(1).at(0) = 1.0;
  //input3 to VRN connection weight
  vrn_input3_w = 5;
  //input4 to VRN connection weight
  vrn_input4_w = 5;


  //initial integrator values
  count1 = 0; // counter for each time step the pulse train is constant
  count2 = 0;
  T1 = -1; //period time counter
  T2 = -1;
  period1 = 0; //period number (even values means full period)
  period2 = 0;
  y1 = 0; //y-values of triangle function
  y2 = 0;

  //Coxa joints threshold
  threshold_c = -0.9;

  //motor time delay
  tau = 16;
  tau_l = 48;
  time = 0;

  //Learning forward models to expected foot sensors

  for(unsigned int i=0; i<fmodel_cmr_w.size(); i++)
  {
    // MUST! fmodel_cmr_w.at(i)>fmodel_cmr_bias.at(i)
    fmodel_cmr_w.at(i) = 1.0;//1.77;
    fmodel_fmodel_cmr_w.at(i) = 1.0;
    fmodel_post_cmr_w.at(i) = 50.0;//20.0;
    fmodel_cmr_bias.at(i) = 1.0;

    fmodel_cml_w.at(i) = 1.0;//
    fmodel_fmodel_cml_w.at(i) = 1.0;
    fmodel_post_cml_w.at(i) = 50.0;
    fmodel_cml_bias.at(i) = 1.0;

    lr_fmodel_cr.at(i) = 0.01;//0.01;
    counter_cr.at(i) = 0;
    lr_fmodel_cl.at(i) = 0.01;
    counter_cl.at(i) = 0;

    //---PID control--0.03--MAX  =250--//
    kp_r.at(i) = 2.5;//1.5;
    ki_r.at(i) = 2.0;
    kd_r.at(i) = 2.0;
    kp_l.at(i) = 2.5;//1.5;
    ki_l.at(i) = 2.0;
    kd_l.at(i) = 2.0;
    int_l.at(i)= 0.0;
    int_r.at(i)= 0.0;
    d_l.at(i)= 0.0;
    d_r.at(i)= 0.0;
    //---PID control------//
  }


  //-----------------------------AMOSiiV2_Config------------------------------------------//
  //W=0.04 Initial learned weights of forward models
  //  fmodel_fmodel_cmr_w.at(0)=1.75842; fmodel_cmr_bias.at(0)= 0.492008 ; fmodel_cmr_w.at(0)= 1.00879;//counter16690cin=0.04
  //  fmodel_fmodel_cmr_w.at(1)=1.37276; fmodel_cmr_bias.at(1)= 0.770924 ; fmodel_cmr_w.at(1)= 1.10033;//counter18210cin=0.04
  //  fmodel_fmodel_cmr_w.at(2)=1.50538; fmodel_cmr_bias.at(2)= 0.615794 ; fmodel_cmr_w.at(2)= 1.06701;//counter17634cin=0.04
  //  fmodel_fmodel_cml_w.at(0)=1.39489;fmodel_cml_bias.at(0)= 0.744294 ;fmodel_cml_w.at(0)= 1.082;//counter18155cin=0.04
  //  fmodel_fmodel_cml_w.at(1)=1.38666;fmodel_cml_bias.at(1)= 0.761661 ;fmodel_cml_w.at(1)= 1.09715;//counter18162cin=0.04
  //  fmodel_fmodel_cml_w.at(2)=1.22578;fmodel_cml_bias.at(2)= 0.728562 ;fmodel_cml_w.at(2)= 1.18023;//counter18389cin=0.04

  //-----------------------------AMOSiiV2_Config------------------------------------------//


  for(unsigned int i=0; i<lowpass_cmr_error_activity.size(); i++)
  {
    lowpass_cmr_error_activity.at(i) = 0.0;
    lowpass_cmr__error_output.at(i) = 0.0;
    lowpass_cmr_w.at(i) = 1.0;
    lowpass_lowpass_cmr_w.at(i) = 5.5;
    lowpass_cmr_bias.at(i) = -4.5;

    lowpass_cml_error_activity.at(i) = 0.0;
    lowpass_cml__error_output.at(i) = 0.0;
    lowpass_cml_w.at(i) = 1.0;
    lowpass_lowpass_cml_w.at(i) = 5.5;
    lowpass_cml_bias.at(i) = -4.5;
  }


  // Initialization of Emd parameters
  for(unsigned int i=0; i<a1_r.size(); i++)
  {
    a1_r.at(i) = 1000;
    a2_r.at(i) = -5000;
    a3_r.at(i) = 0.1;
    a1_l.at(i) = 1000;
    a2_l.at(i) = -5000;
    a3_l.at(i) = 0.1;
  }

  //Internal model :://KOH add after the GIT version
  //case 5 counting technique
  counter_refelx_R_fs = 0;
  counter_refelx_L_fs = 0;
  max_up = 0;
  max_down=0;
  max_fmodel = 0.0;


  //-----------------------------AMOSiiV2_Config------------------------------------------//
  //Motor mapping
  //TC_front
  //Fix
  min_tc = -0.91; // network output range
  max_tc = 0.91;// network output range
  //Adjust
  min_tc_f_nwalking_deg = 0; //deg ** MIN -70 deg
  max_tc_f_nwalking_deg = 50; //deg ** MAX +70 deg
  min_tc_f_nwalking = -1;
  max_tc_f_nwalking = 1;

  //TC_middle
  //Adjust
  min_tc_m_nwalking_deg = -30; //deg ** MIN -60 deg
  max_tc_m_nwalking_deg = 20;//10; //deg ** MAX +60 deg
  min_tc_m_nwalking = -1;
  max_tc_m_nwalking = 1;

  //TC_rear
  //Adjust
  min_tc_r_nwalking_deg = -60; //deg ** MIN -70 deg
  max_tc_r_nwalking_deg = -10; //deg ** MAX +70 deg
  min_tc_r_nwalking = -1;
  max_tc_r_nwalking = 1;

  //CTR
  //Fix
  min_ctr = -1; // network output range
  max_ctr = 0.96;// network output range

  //FTI
  //Fix
  min_fti = -1; // network output range
  max_fti = 1;// network output range

  for(unsigned int i=0; i< min_ctr_nwalking_deg.size(); i++)
  {
    //Parameter set up for normal walking on flat terrain
    min_ctr_nwalking_deg.at(i) = 45;//48;//45;//50 is good; //40deg ** MIN -70 deg
    max_ctr_nwalking_deg.at(i) = 75; //75deg ** MAX +70 deg
    min_ctr_nwalking.at(i) = -1;
    max_ctr_nwalking.at(i) =1;

    min_ctl_nwalking_deg.at(i) = 45;//48;//45;//50 is good; //deg ** MIN -70 deg
    max_ctl_nwalking_deg.at(i) = 75; //deg ** MAX +70 deg
    min_ctl_nwalking.at(i) = -1;
    max_ctl_nwalking.at(i) = 1;

    min_ftir_nwalking_deg.at(i) = -130; //deg ** MIN -130 deg
    max_ftir_nwalking_deg.at(i) = -120; //deg ** MAX -20 deg
    min_ftir_nwalking.at(i) = -1;
    max_ftir_nwalking.at(i) = 1;

    min_ftil_nwalking_deg.at(i) = -130;
    max_ftil_nwalking_deg.at(i) = -120;
    min_ftil_nwalking.at(i) = -1;
    max_ftil_nwalking.at(i) = 1;
  }

  //------Used values-------------//
  max_c = 115.0; // max range
  max_f = 110.0; // max range

  //if  max_c_offset, max_f_offset very low = 45 then robot will extend its legs very fast--> walk very high good for rough terrain
  //if  max_c_offset, max_f_offset very large = 180 then robot will extend its legs very slow--> walk very low good for normal walking

  //60/*Use to compare*/;//80;//100.0;//55.0; // Adjust from 45,...185=> 185 (cin= 0.02, wave), 45 (cin= 0.18, tripod)

  max_c_offset = 95;//110;//110 (normal); 180 (lower the body)
  max_f_offset = 95;//110;//110 (normal); 180 (lower the body)

  //60/*Use to compare*/;//80;//100.0;//55.0; // Adjust from 45,...185=> 185 (cin= 0.02, wave), 45 (cin= 0.18, tripod)
  //80,100,185 == amlost the same height when using forward model and foot but different height using only foot signal

  //------Used values-------------//

  if(switchon_less_reflexes==true)
  {
    //For testing pure reflex in order to NOT to extend too much
    max_c = 115.0; // max range
    max_f = 110.0; // max range
    max_c_offset = max_c*2;//10;//2;//10; // or 1150
    max_f_offset = max_c*2;//10;//2;//10; // or 1100
  }

  //BJ
  //Fix
  min_bj = -1; // network output range
  max_bj = 1;	// network output range

  //Adjust
  min_bj_fwalking_deg = -45; //deg ** MIN -45 deg
  max_bj_fwalking_deg = 45; //deg ** MAX 45 deg

  offset_bj = 0.0;

  if(lift_body_up==true)
  {
    //Lifting body up
    for(unsigned int i=0; i< offset_ftil_downward.size(); i++)
    {
      int lifting_value = 10;//5;//10; //20

      //lifting_value = 10; small lifting body up
      //lifting_value = 20; small lifting body up
      //lifting_value = 50; high lifting body up

      //These parameters need to be adjust the high values, the more lifting body!!!
      offset_ftil_downward.at(i) = lifting_value; //KOH-->Eduard
      offset_ftir_downward.at(i) = lifting_value; //KOH-->Eduard
      offset_ctl_downward.at(i) = lifting_value; //KOH-->Eduard
      offset_ctr_downward.at(i) = lifting_value; //KOH-->Eduard
    }
  }
  else
  {
    //Lifting body up
    for(unsigned int i=0; i< offset_ftil_downward.size(); i++)
    {
      offset_ftil_downward.at(i) = 0; //KOH-->Eduard
      offset_ftir_downward.at(i) = 0; //KOH-->Eduard
      offset_ctl_downward.at(i) = 0; //KOH-->Eduard
      offset_ctr_downward.at(i) = 0; //KOH-->Eduard
    }
  }


  //-----------------------------AMOSiiV2_Config------------------------------------------//


  //Other parameters
  global_count = 0;
  allfoot_off_ground = 0;

  //Test walking behavior with saved motor text
  initialized = false;
  ii = 0;

  m_r0_t_old = 0.0;
  m_r1_t_old = 0.0;
  m_r2_t_old = 0.0;
  m_l0_t_old = 0.0;
  m_l1_t_old = 0.0;
  m_l2_t_old = 0.0;

  //--------------------------Gait analysis------------------------------------------//
  stance_time = 0.0;
  swing_time = 0.0;
  stride_period = 0.0;
  duty_factor = 0.0;

  //--------------------------Power consumption calculation--------------------------//
  ac_motor = 0.0;
  i_motor = 0.0;
  motor_power_con = 0.0;


  //--------------------------Add ENS network--(2)-----------------------------------//
  //Setting ENS parameters

  //---------------------ESN module 1
  int num_input_ESN_R0 = 1;
  int num_output_ESN_R0 = 3;
  ESN_R0 = new ESNetwork(num_input_ESN_R0/*no. input*/,num_output_ESN_R0 /*no. output*/, 30/*30*/ /*rc hidden neurons*/, false /*feedback*/, false /*feeding input to output*/, 0.1 /*0.1 leak = 0.0-1.0*/, false /*IP*/);

  ESN_R0->outnonlinearity = 0; //0; // 0 = linear, 1 = sigmoid, 2  = tanh: transfer function of an output neuron
  ESN_R0->nonlinearity = 2; // 0 = linear, 1 = sigmoid, 2  = tanh: transfer function of all hidden neurons
  ESN_R0->withRL = 2; // 2 = stand ESN learning, 1 = RL with TD learning

  ESN_R0->InputSparsity = 70; // if 0 = input connects to all hidden neurons, if 100 = input does not connect to hidden neurons
  ESN_R0->autocorr = pow(10,4); //set as high as possible, default = 1
  ESN_R0->InputWeightRange = 0.1; // scaling of input to hidden neurons, default 0.15 means [-0.15, +0.15]
  ESN_R0->LearnMode = 1; //1;//RLS = 1. LMS =2
  ESN_R0->Loadweight = false; // true = loading learned weights
  ESN_R0->NoiseRange = 0.001; //
  ESN_R0->RCneuronNoise = false; // false = constant fixed bias, true = changing noise bias every time

  ESN_R0->generate_random_weights(50 /*70  10% sparsity = 90% connectivity */, 0.95 /*1.2-1.5 = chaotics*/);


  //Create ESN input vector
  ESinput_R0 = new float[num_input_ESN_R0];
  //Create ESN target output vector
  ESTrainOutput_R0 = new float[num_output_ESN_R0];

  //Initial values of input and target output
  for(unsigned int i = 0; i < num_input_ESN_R0; i++)
  {
    ESinput_R0[i] = 0.0;
  }

  for(unsigned int i = 0; i< num_output_ESN_R0; i++)
  {
    ESTrainOutput_R0[i] = 0.0;

  }
  //---------------------ESN module 2

  int num_input_ESN_R1 = 1;
  int num_output_ESN_R1 = 3;

  ESN_R1 = new ESNetwork(num_input_ESN_R1/*no. input*/,num_output_ESN_R1 /*no. output*/, 30/*30*/ /*rc hidden neurons*/, false /*feedback*/, false /*feeding input to output*/, 0.1 /*0.1 leak = 0.0-1.0*/, false /*IP*/);

  ESN_R1->outnonlinearity = 0; // 0 = linear, 1 = sigmoid, 2  = tanh: transfer function of an output neuron
  ESN_R1->nonlinearity = 2; // 0 = linear, 1 = sigmoid, 2  = tanh: transfer function of all hidden neurons
  ESN_R1->withRL = 2; // 2 = stand ESN learning, 1 = RL with TD learning

  ESN_R1->InputSparsity = 70; // if 0 = input connects to all hidden neurons, if 100 = input does not connect to hidden neurons
  ESN_R1->autocorr = pow(10,4); //pow(10,4); set as high as possible, default = 1
  ESN_R1->InputWeightRange = 0.1; // scaling of input to hidden neurons, default 0.15 means [-0.15, +0.15]
  ESN_R1->LearnMode = 1; //1;//RLS = 1. LMS =2
  ESN_R1->Loadweight = false; // true = loading learned weights
  ESN_R1->NoiseRange = 0.001; //
  ESN_R1->RCneuronNoise = false; // false = constant fixed bias, true = changing noise bias every time

  ESN_R1->generate_random_weights(50 /*70  10% sparsity = 90% connectivity */, 0.95 /*1.2-1.5 = chaotics*/);


  //Create ESN input vector
  ESinput_R1 = new float[num_input_ESN_R1];
  //Create ESN target output vector
  ESTrainOutput_R1 = new float[num_output_ESN_R1];

  //Initial values of input and target output
  for(unsigned int i = 0; i < num_input_ESN_R1; i++)
  {
    ESinput_R1[i] = 0.0;
  }

  for(unsigned int i = 0; i< num_output_ESN_R1; i++)
  {
    ESTrainOutput_R1[i] = 0.0;

  }

  //---------------------ESN module 3

  int num_input_ESN_R2 = 1;
  int num_output_ESN_R2 = 3;

  ESN_R2 = new ESNetwork(num_input_ESN_R2/*no. input*/,num_output_ESN_R2 /*no. output*/, 30/*30*/ /*rc hidden neurons*/, false /*feedback*/, false /*feeding input to output*/, 0.1 /*0.1 leak = 0.0-1.0*/, false /*IP*/);

  ESN_R2->outnonlinearity = 0; // 0 = linear, 1 = sigmoid, 2  = tanh: transfer function of an output neuron
  ESN_R2->nonlinearity = 2; // 0 = linear, 1 = sigmoid, 2  = tanh: transfer function of all hidden neurons
  ESN_R2->withRL = 2; // 2 = stand ESN learning, 1 = RL with TD learning

  ESN_R2->InputSparsity = 70; // if 0 = input connects to all hidden neurons, if 100 = input does not connect to hidden neurons
  ESN_R2->autocorr = pow(10,4); //pow(10,4); set as high as possible, default = 1
  ESN_R2->InputWeightRange = 0.1; // scaling of input to hidden neurons, default 0.15 means [-0.15, +0.15]
  ESN_R2->LearnMode = 1; //1;//RLS = 1. LMS =2
  ESN_R2->Loadweight = false; // true = loading learned weights
  ESN_R2->NoiseRange = 0.001; //
  ESN_R2->RCneuronNoise = false; // false = constant fixed bias, true = changing noise bias every time

  ESN_R2->generate_random_weights(50 /*70  10% sparsity = 90% connectivity */, 0.95 /*1.2-1.5 = chaotics*/);


  //Create ESN input vector
  ESinput_R2 = new float[num_input_ESN_R2];
  //Create ESN target output vector
  ESTrainOutput_R2 = new float[num_output_ESN_R2];

  //Initial values of input and target output
  for(unsigned int i = 0; i < num_input_ESN_R2; i++)
  {
    ESinput_R2[i] = 0.0;
  }

  for(unsigned int i = 0; i< num_output_ESN_R2; i++)
  {
    ESTrainOutput_R2[i] = 0.0;

  }

  //---------------------ESN module 4

  int num_input_ESN_L0 = 1;
  int num_output_ESN_L0 = 3;

  ESN_L0 = new ESNetwork(num_input_ESN_L0/*no. input*/,num_output_ESN_L0 /*no. output*/, 30/*30*/ /*rc hidden neurons*/, false /*feedback*/, false /*feeding input to output*/, 0.1 /*0.1 leak = 0.0-1.0*/, false /*IP*/);

  ESN_L0->outnonlinearity = 0; // 0 = linear, 1 = sigmoid, 2  = tanh: transfer function of an output neuron
  ESN_L0->nonlinearity = 2; // 0 = linear, 1 = sigmoid, 2  = tanh: transfer function of all hidden neurons
  ESN_L0->withRL = 2; // 2 = stand ESN learning, 1 = RL with TD learning

  ESN_L0->InputSparsity = 70; // if 0 = input connects to all hidden neurons, if 100 = input does not connect to hidden neurons
  ESN_L0->autocorr = pow(10,4); //pow(10,4); set as high as possible, default = 1
  ESN_L0->InputWeightRange = 0.1; // scaling of input to hidden neurons, default 0.15 means [-0.15, +0.15]
  ESN_L0->LearnMode = 1; //1;//RLS = 1. LMS =2
  ESN_L0->Loadweight = false; // true = loading learned weights
  ESN_L0->NoiseRange = 0.001; //
  ESN_L0->RCneuronNoise = false; // false = constant fixed bias, true = changing noise bias every time

  ESN_L0->generate_random_weights(50 /*70  10% sparsity = 90% connectivity */, 0.95 /*1.2-1.5 = chaotics*/);


  //Create ESN input vector
  ESinput_L0 = new float[num_input_ESN_L0];
  //Create ESN target output vector
  ESTrainOutput_L0 = new float[num_output_ESN_L0];

  //Initial values of input and target output
  for(unsigned int i = 0; i < num_input_ESN_L0; i++)
  {
    ESinput_L0[i] = 0.0;
  }

  for(unsigned int i = 0; i< num_output_ESN_L0; i++)
  {
    ESTrainOutput_L0[i] = 0.0;

  }

  //---------------------ESN module 5

  int num_input_ESN_L1 = 1;
  int num_output_ESN_L1 = 3;

  ESN_L1 = new ESNetwork(num_input_ESN_L1/*no. input*/,num_output_ESN_L1 /*no. output*/, 30/*30*/ /*rc hidden neurons*/, false /*feedback*/, false /*feeding input to output*/, 0.1 /*0.1 leak = 0.0-1.0*/, false /*IP*/);

  ESN_L1->outnonlinearity = 0; // 0 = linear, 1 = sigmoid, 2  = tanh: transfer function of an output neuron
  ESN_L1->nonlinearity = 2; // 0 = linear, 1 = sigmoid, 2  = tanh: transfer function of all hidden neurons
  ESN_L1->withRL = 2; // 2 = stand ESN learning, 1 = RL with TD learning

  ESN_L1->InputSparsity = 70; // if 0 = input connects to all hidden neurons, if 100 = input does not connect to hidden neurons
  ESN_L1->autocorr = pow(10,4); //pow(10,4); set as high as possible, default = 1
  ESN_L1->InputWeightRange = 0.1; // scaling of input to hidden neurons, default 0.15 means [-0.15, +0.15]
  ESN_L1->LearnMode = 1; //1;//RLS = 1. LMS =2
  ESN_L1->Loadweight = false; // true = loading learned weights
  ESN_L1->NoiseRange = 0.001; //
  ESN_L1->RCneuronNoise = false; // false = constant fixed bias, true = changing noise bias every time

  ESN_L1->generate_random_weights(50 /*70  10% sparsity = 90% connectivity */, 0.95 /*1.2-1.5 = chaotics*/);


  //Create ESN input vector
  ESinput_L1 = new float[num_input_ESN_L1];
  //Create ESN target output vector
  ESTrainOutput_L1 = new float[num_output_ESN_L1];

  //Initial values of input and target output
  for(unsigned int i = 0; i < num_input_ESN_L1; i++)
  {
    ESinput_L1[i] = 0.0;
  }

  for(unsigned int i = 0; i< num_output_ESN_L1; i++)
  {
    ESTrainOutput_L1[i] = 0.0;

  }

  //---------------------ESN module 6

  int num_input_ESN_L2 = 1;
  int num_output_ESN_L2 = 3;

  ESN_L2 = new ESNetwork(num_input_ESN_L2/*no. input*/,num_output_ESN_L2 /*no. output*/, 30/*30*/ /*rc hidden neurons*/, false /*feedback*/, false /*feeding input to output*/, 0.1 /*0.1 leak = 0.0-1.0*/, false /*IP*/);

  ESN_L2->outnonlinearity = 0; // 0 = linear, 1 = sigmoid, 2  = tanh: transfer function of an output neuron
  ESN_L2->nonlinearity = 2; // 0 = linear, 1 = sigmoid, 2  = tanh: transfer function of all hidden neurons
  ESN_L2->withRL = 2; // 2 = stand ESN learning, 1 = RL with TD learning

  ESN_L2->InputSparsity = 70; // if 0 = input connects to all hidden neurons, if 100 = input does not connect to hidden neurons
  ESN_L2->autocorr = pow(10,4); //pow(10,4); set as high as possible, default = 1
  ESN_L2->InputWeightRange = 0.1; // scaling of input to hidden neurons, default 0.15 means [-0.15, +0.15]
  ESN_L2->LearnMode = 1; //1;//RLS = 1. LMS =2
  ESN_L2->Loadweight = false; // true = loading learned weights
  ESN_L2->NoiseRange = 0.001; //
  ESN_L2->RCneuronNoise = false; // false = constant fixed bias, true = changing noise bias every time

  ESN_L2->generate_random_weights(50 /*70  10% sparsity = 90% connectivity */, 0.95 /*1.2-1.5 = chaotics*/);


  //Create ESN input vector
  ESinput_L2 = new float[num_input_ESN_L2];
  //Create ESN target output vector
  ESTrainOutput_L2 = new float[num_output_ESN_L2];

  //Initial values of input and target output
  for(unsigned int i = 0; i < num_input_ESN_L2; i++)
  {
    ESinput_L2[i] = 0.0;
  }

  for(unsigned int i = 0; i< num_output_ESN_L2; i++)
  {
    ESTrainOutput_L2[i] = 0.0;

  }
  //--------------------------Add ENS network--(2)-----------------------------------//

  //Forward model ESN
  fmodel_cmr_output_rc.resize(3);
  fmodel_cml_output_rc.resize(3);

  fmodel_cmr0_output_rc.resize(3);
  fmodel_cmr1_output_rc.resize(3);
  fmodel_cmr2_output_rc.resize(3);

  fmodel_cml0_output_rc.resize(3);
  fmodel_cml1_output_rc.resize(3);
  fmodel_cml2_output_rc.resize(3);


  //LTM
  fmodel_cmr_output_ltm.resize(3);
  fmodel_cml_output_ltm.resize(3);

  //LTM neuron
  cmr0_ltm_neuron.resize(NUM_LTM_R0);
  cmr0_ltm_neuron_w.resize(NUM_LTM_R0);
  fmodel_cmr_output_w2.resize(NUM_LTM_R0);
  input_ltm.resize(NUM_LTM_R0);

  for(unsigned int i = 0; i< NUM_LTM_R0; i++)
  {
    cmr0_ltm_neuron_w.at(i) = 0.01;
  }



  inputs_ltm_out = new matrix::Matrix(ESN_R0->endweights->getM(),ESN_R0->endweights->getN());
  outputs_ltm_out = new matrix::Matrix(ESN_R0->endweights->getM(),ESN_R0->endweights->getN());
  weights_ltm_out = new matrix::Matrix(ESN_R0->endweights->getM(),ESN_R0->endweights->getN());


  for(int i = 0; i < ESN_R0->endweights->getM(); i++)
  {
    for(int j = 0; j < ESN_R0->endweights->getN(); j++)
    {
      inputs_ltm_out->val(i,j) = 0.0;
      outputs_ltm_out->val(i,j) = 0.0;
      weights_ltm_out->val(i,j) =0.001;

    }
  }


  inputs_ltm_in = new matrix::Matrix(ESN_R0->startweights->getM(),ESN_R0->startweights->getN());
  outputs_ltm_in = new matrix::Matrix(ESN_R0->startweights->getM(),ESN_R0->startweights->getN());
  weights_ltm_in = new matrix::Matrix(ESN_R0->startweights->getM(),ESN_R0->startweights->getN());


  for(int i = 0; i < ESN_R0->startweights->getM(); i++)
  {
    for(int j = 0; j < ESN_R0->startweights->getN(); j++)
    {
      inputs_ltm_in->val(i,j) = 0.0;
      outputs_ltm_in->val(i,j) = 0.0;
      weights_ltm_in->val(i,j) =0.001;

    }
  }

  inputs_ltm_hid = new matrix::Matrix(ESN_R0->innerweights->getM(),ESN_R0->innerweights->getN());
  outputs_ltm_hid = new matrix::Matrix(ESN_R0->innerweights->getM(),ESN_R0->innerweights->getN());
  weights_ltm_hid = new matrix::Matrix(ESN_R0->innerweights->getM(),ESN_R0->innerweights->getN());


  for(int i = 0; i < ESN_R0->innerweights->getM(); i++)
  {
    for(int j = 0; j < ESN_R0->innerweights->getN(); j++)
    {
      inputs_ltm_hid->val(i,j) = 0.0;
      outputs_ltm_hid->val(i,j) = 0.0;
      weights_ltm_hid->val(i,j) =0.001;

    }
  }

  inputs_ltm_bi = new matrix::Matrix(ESN_R0->noise->getM(),ESN_R0->noise->getN());
  outputs_ltm_bi = new matrix::Matrix(ESN_R0->noise->getM(),ESN_R0->noise->getN());
  weights_ltm_bi = new matrix::Matrix(ESN_R0->noise->getM(),ESN_R0->noise->getN());


  for(int i = 0; i < ESN_R0->noise->getM(); i++)
  {
    for(int j = 0; j < ESN_R0->noise->getN(); j++)
    {
      inputs_ltm_bi->val(i,j) = 0.0;
      outputs_ltm_bi->val(i,j) = 0.0;
      weights_ltm_bi->val(i,j) =0.001;

    }
  }

  old_pcpg = 0.0;


};


NeuralLocomotionControlAdaptiveClimbing::~NeuralLocomotionControlAdaptiveClimbing(){


  //Save files
  outFilenlc1.close();
  outFilenlc2.close();
  outFilenlc3.close();
  outFilenlc4.close();
  outFilenlc5.close();
  outFilenlc6.close();
  outFilenlc_tc.close();
  outFilenlc_ctr.close();
  outFilenlF_fti.close();
  outFilenlc_tc_pre.close();
  outFilenlc_ctr_pre.close();
  outFilenlF_fti_pre.close();
  outFilegait.close();
  outFilegait2.close();
  outFilepower1.close();


  //----- ESN objects garbage collection ---- //

  delete []ESN_R0;
  delete []ESinput_R0;
  delete []ESTrainOutput_R0;


  delete []ESN_R1;
  delete []ESinput_R1;
  delete []ESTrainOutput_R1;


  delete []ESN_R2;
  delete []ESinput_R2;
  delete []ESTrainOutput_R2;


  delete []ESN_L0;
  delete []ESinput_L0;
  delete []ESTrainOutput_L0;

  delete []ESN_L1;
  delete []ESinput_L1;
  delete []ESTrainOutput_L1;


  delete []ESN_L2;
  delete []ESinput_L2;
  delete []ESTrainOutput_L2;

  delete inputs_ltm_in;
  delete weights_ltm_in;
  delete outputs_ltm_in;

  delete inputs_ltm_out;
  delete weights_ltm_out;
  delete outputs_ltm_out;

  delete inputs_ltm_hid;
  delete weights_ltm_hid;
  delete outputs_ltm_hid;

  delete inputs_ltm_bi;
  delete weights_ltm_bi;
  delete outputs_ltm_bi;

};

std::vector<double> NeuralLocomotionControlAdaptiveClimbing::step_nlc(const std::vector<double> in0 /*x_prep*/, const std::vector<double> in1 /*x*/){//, bool Footinhibition){

  // Define local parameters//
  //std::vector<double> m;
  //m.resize(19); //  number of motors

  old_pcpg = pcpg_output.at(1);

  //Input to control locomotion
  static int iii=0;

  iii++;

  //  std::cout<<"counter"<<" "<<iii<< "\n";
  //  if(iii<400)
  //  {
  //  //Forward
  //    input.at(0) = 1; //tc JOINT inhibition = 0
  //    input.at(1) = 1; //FL0, FR1, FR2 control NOT USED
  //    input.at(2) = 1; //PSN control
  //    input.at(3) = -1; //turn left = 1,
  //    input.at(4) = -1; //turn right = 1,
  //  }
  //  else if(iii>400 && iii<600)
  //  {
  //    //Backward
  //    input.at(0) = 1; //tc JOINT inhibition = 0
  //    input.at(1) = 1; //FL0, FR1, FR2 control NOT USED
  //    input.at(2) = 1; //PSN control
  //    input.at(3) = 1; //turn left = 1,
  //    input.at(4) = 1; //turn right = 1,
  //  }
  //  else if(iii>600 && iii<800)
  //  {
  //    //Turn right
  //    input.at(0) = 1; //tc JOINT inhibition = 0
  //    input.at(1) = 1; //FL0, FR1, FR2 control NOT USED
  //    input.at(2) = 1; //PSN control
  //    input.at(3) = -1; //turn left = 1,
  //    input.at(4) = 1; //turn right = 1,
  //  }
  //  else if(iii>800)
  //  {
  //    //Turn left
  //    input.at(0) = 1; //tc JOINT inhibition = 0
  //    input.at(1) = 1; //FL0, FR1, FR2 control NOT USED
  //    input.at(2) = 1; //PSN control
  //    input.at(3) = 1; //turn left = 1,
  //    input.at(4) = -1; //turn right = 1,
  //  }

  //Forward
  input.at(0) = 1; //All JOINTS inhibition = 0, 1 activate all joints = I1 in paper
  input.at(1) = 1; //FL0, FR1, FR2 control NOT USED
  input.at(2) = 1; //PSN control  = I2 in paper
  input.at(3) = -1; //turn left = 1,  = I3 in paper
  input.at(4) = -1; //turn right = 1,  = I4 in paper



  //Autonomously steering to always walk straight line
  input.at(3)=-1.0 + 4.5*(in1.at(BY_ori));
  input.at(4)=-1.0 - 4.5*(in1.at(BY_ori));
  if(input.at(3)> 1.0)
    input.at(3)=1.0;
  if(input.at(3)< -1.0)
    input.at(3)=-1.0;
  if(input.at(4)> 1.0)
    input.at(4)=1.0;
  if(input.at(4)< -1.0)
    input.at(4)=-1.0;



  if(sequentiral_learning)
  {
    if(iii<= t_change_gait1)
    {
      Control_input = gait1;
    }
    else if(iii>t_change_gait1 && iii<=t_change_gait2)
    {
      Control_input = gait2;
    }
    else if(iii>t_change_gait2 && iii<=t_change_gait3)
    {
      Control_input = gait3;
    }

    if(iii>t_change_gait3)
      iii = 0;

    //Fixed gait to one gait for gap crossing
    if(crossing_gap)
    {
    Control_input = gait3;
    }
  }

  //  //Lateral right NOT WORKING YET!!!!
  //  input.at(0) = 0; //tc JOINT inhibition = 0
  //  input.at(1) = -1; //FL0, FR1, FR2 control NOT USED
  //  input.at(2) = 0; //PSN control
  //  input.at(3) = 1; //turn left = 1,
  //  input.at(4) = -1; //turn right = 1,

  /*******************************************************************************
   *  MODULE 1 CPG
   *******************************************************************************/

  if(!sequentiral_learning)
  {
    //**********CPG***************
    //From 0.02-1.5
    //Control_input = 0.0;// slow Wave St
    //Control_input = 0.01;// slow Wave St

    //Control_input = 0.02;// slow Wave St **************** Forward model // Stone

    //Control_input = 0.03;// L0 and R2 pair slight left curve STEFFEN


    //Control_input = 0.04;// L0 and R2 pair slight left curve for small stone

    //Control_input = 0.05;// L0 and R2 pair slight left curve*************** Grass
    Control_input = 0.06;//slow stable Tetrapod OK USED
    //Control_input = 0.07;//slow stable Tetrapod OK USED


    //Control_input = 0.08;//slow stable Tetrapod OK USED STEFF

    //Control_input = 0.09;//slow stable Tetrapod OK USED
    //Control_input = 0.1;//slow stable Tetrapod OK USED
    //Control_input = 0.11;//slow stable Tetrapod OK USED
    //Control_input = 0.12;//slow stable Tetrapod OK USED
    //Control_input =  0.13;//slow stable Tetrapod OK USED
    //Control_input = 0.14;//slow stable Tetrapod OK USED

    //Control_input = 0.15;//slow stable Tetrapod OK USED //------------------ ESN learnt


    //Control_input = 0.16;//slow stable Tripod OK USED STEFFen // Flat!!!*************



    //Control_input = 0.17;//slow stable Tetrapod OK USED
    //Control_input = 0.18;//slow stable Tetrapod OK USED
    //Control_input = 0.19;//slow stable Tetrapod OK USED
    //Control_input = 0.20;//slow stable Tetrapod OK USED
    //Control_input = 0.21;//slow stable Tetrapod OK USED
    //Control_input = 0.22;//slow stable Tetrapod OK USED
    //Control_input = 0.23;//pair
    //Control_input = 0.24;//pair
    //Control_input = 0.25;//pair with multimeter
    //Control_input = 0.26;//pair
    //Control_input = 0.27;//pair
    //Control_input = 0.28;//pair

    //------------------
    //Control_input = 0.29;//pair
    //Control_input = 0.3;//pair
    //Control_input = 0.31;//pair
    //Control_input = 0.32;//pair+ tripot
    //Control_input = 0.33;//pair+ tripot
    //Control_input = 0.34;//pair+ tripot
    //Control_input = 0.35;//tripot
    //Control_input = 0.36;//tripot

    //-------------------
    //Control_input = 0.14; //terapod OK USED
    //Control_input = 0.18; //Tripod fast OK USED // with Foot inhibition Good
    //Control_input = 0.34; //Faster than tripod
  }

  std::cout<<"c_input"<<Control_input<<std::endl;

  for(unsigned int i=0; i<fmodel_cmr_w.size(); i++)
  {

    //---PID control--0.03--MAX  =250--//
    //    kp_r.at(i) = 2.5;
    //    ki_r.at(i) = 2.0;
    //    kd_r.at(i) = 2.0;
    //    kp_l.at(i) = 2.5;
    //    ki_l.at(i) = 2.0;
    //    kd_l.at(i) = 2.0;

    //    kp_r.at(i) = 1.0+exp(Control_input*10*1.1);
    //    ki_r.at(i) = 0.5+exp(Control_input*10*1.1);
    //    kd_r.at(i) = 0.5+exp(Control_input*10*1.1);
    //    kp_l.at(i) = 1.0+exp(Control_input*10*1.1);
    //    ki_l.at(i) = 0.5+exp(Control_input*10*1.1);
    //    kd_l.at(i) = 0.5+exp(Control_input*10*1.1);

    kp_r.at(i) = 10.0+exp(Control_input*10*1.1);//50+
    ki_r.at(i) = 0.5+exp(Control_input*10*1.1);
    kd_r.at(i) = 0.5+exp(Control_input*10*1.1);//10
    kp_l.at(i) = 10.0+exp(Control_input*10*1.1);//50+
    ki_l.at(i) = 0.5+exp(Control_input*10*1.1);
    kd_l.at(i) = 0.5+exp(Control_input*10*1.1);//10

    std::cout<<"P = "<<kp_r.at(i)<<"I = "<<ki_r.at(i)<<"D = "<<kd_r.at(i)<<std::endl;

    //    //---PID control------//
  }

  cpg_w.at(0).at(0) =  1.4;
  cpg_w.at(0).at(1) =  0.18+Control_input;//0.4;
  cpg_w.at(1).at(0) =  -0.18-Control_input;//-0.4
  cpg_w.at(1).at(1) =  1.4;

  cpg_activity.at(0) = cpg_w.at(0).at(0) * cpg_output.at(0) + cpg_w.at(0).at(1) * cpg_output.at(1) + cpg_bias;
  cpg_activity.at(1) = cpg_w.at(1).at(1) * cpg_output.at(1) + cpg_w.at(1).at(0) * cpg_output.at(0) + cpg_bias;

  for(unsigned int i=0; i<cpg_output.size();i++)
  {
    cpg_output.at(i) = tanh(cpg_activity.at(i));
  }

  //********CPG end************


  /*******************************************************************************
   *  MODULE 2 CPG POST-PROCESSING
   *******************************************************************************/

  //***CPG post processing*****

  //From CPG
  pcpg_step.at(0) = cpg_output.at(0);
  pcpg_step.at(1) = cpg_output.at(1);


  setold.at(0) = set.at(0);
  setold.at(1) = set.at(1);

  countupold.at(0) = countup.at(0);
  countupold.at(1) = countup.at(1);

  countdownold.at(0) = countdown.at(0);
  countdownold.at(1) = countdown.at(1);

  //1) Linear threshold transfer function neuron 1 , or called step function neuron//
  /********************************************************/

  if (pcpg_step.at(0)>=0.85) ////////////////////Intuitively select
  {
    set.at(0) = 1.0;
  }
  if (pcpg_step.at(0)<0.85) ////////////////////Intuitively select
  {
    set.at(0) = -1.0;
  }


  if (pcpg_step.at(1)>=0.85) ////////////////////Intuitively select
  {
    set.at(1) = 1.0;
  }
  if (pcpg_step.at(1)<0.85) ////////////////////Intuitively select
  {
    set.at(1) = -1.0;
  }

  diffset.at(0) = set.at(0)-setold.at(0); // double
  diffset.at(1) = set.at(1)-setold.at(1); // double

  //2) Count how many step of Swing
  /********************************************************/


  if (set.at(0) == 1.0)
  {
    countup.at(0) = countup.at(0)+1.0; //Delta x0 up
    countdown.at(0) = 0.0;
  }
  // Count how many step of Stance
  else if (set.at(0) == -1.0)
  {
    countdown.at(0) = countdown.at(0)+1.0; //Delta x0 down
    countup.at(0) = 0.0;
  }


  if (set.at(1) == 1.0)
  {
    countup.at(1) = countup.at(1)+1.0; //Delta x0 up
    countdown.at(1) = 0.0;
  }

  // Count how many step of Stance
  else if (set.at(1) == -1.0)
  {
    countdown.at(1) = countdown.at(1)+1.0; //Delta x0 down
    countup.at(1) = 0.0;
  }

  //3) Memorized the total steps of swing and stance
  /********************************************************/

  if (countup.at(0) == 0.0 && diffset.at(0) == -2.0 && set.at(0) == -1.0)
    deltaxup.at(0) = countupold.at(0);

  if (countdown.at(0) == 0.0 && diffset.at(0) == 2.0 && set.at(0) == 1.0)
    deltaxdown.at(0) = countdownold.at(0);

  if (countup.at(1) == 0.0 && diffset.at(1) == -2.0 && set.at(1) == -1.0)
    deltaxup.at(1) = countupold.at(1);

  if (countdown.at(1) == 0.0 && diffset.at(1) == 2.0 && set.at(1) == 1.0)
    deltaxdown.at(1) = countdownold.at(1);


  //4) Comput y up and down !!!!
  /********************************************************/

  xup.at(0) =  countup.at(0);
  xdown.at(0) = countdown.at(0);

  xup.at(1) =  countup.at(1);
  xdown.at(1) = countdown.at(1);



  ////////////Scaling Slope Up calculation////////
  yup.at(0) = ((2./deltaxup.at(0))*xup.at(0))-1;
  ////////////Scaling  Slope Down calculation//////
  ydown.at(0) = ((-2./deltaxdown.at(0))*xdown.at(0))+1;


  ////////////Scaling Slope Up calculation////////
  yup.at(1) = ((2./deltaxup.at(1))*xup.at(1))-1;
  ////////////Scaling  Slope Down calculation//////
  ydown.at(1) = ((-2./deltaxdown.at(1))*xdown.at(1))+1;


  //5) Combine y up and down !!!!
  /********************************************************/

  if (set.at(0) >= 0.0)
    pcpg_output.at(0) = yup.at(0);

  if (set.at(0) < 0.0)
    pcpg_output.at(0) = ydown.at(0);


  if (set.at(1) >= 0.0)
    pcpg_output.at(1) = yup.at(1);

  if (set.at(1) < 0.0)
    pcpg_output.at(1) = ydown.at(1);


  //********Limit upper and lower boundary

  if(pcpg_output.at(0)>1.0)
    pcpg_output.at(0) = 1.0;
  else if(pcpg_output.at(0)<-1.0)
    pcpg_output.at(0) = -1.0;

  if(pcpg_output.at(1)>1.0)
    pcpg_output.at(1) = 1.0;
  else if(pcpg_output.at(1)<-1.0)
    pcpg_output.at(1) = -1.0;

  //***CPG post processing*end*



  /*******************************************************************************
   *  MODULE 3 PSN
   *******************************************************************************/

  //**********PSN***************
  //pcpg_output.at(0) =  cpg_output.at(0);// Set signal to PSN
  //pcpg_output.at(1) =  cpg_output.at(1);//

  psn_activity.at(0) = psn_input2_w.at(0).at(0) * input.at(2) + psn_bias.at(0);
  psn_activity.at(1) = psn_input2_w.at(1).at(0) * input.at(2);

  psn_activity.at(2) = psn_pcpg_w.at(2).at(0) * pcpg_output.at(0) + psn_w.at(2).at(0) * psn_output.at(0);
  psn_activity.at(3) = psn_pcpg_w.at(3).at(1) * pcpg_output.at(1) + psn_w.at(3).at(1) * psn_output.at(1);
  psn_activity.at(4) = psn_pcpg_w.at(4).at(1) * pcpg_output.at(1) + psn_w.at(4).at(0) * psn_output.at(0);
  psn_activity.at(5) = psn_pcpg_w.at(5).at(0) * pcpg_output.at(0) + psn_w.at(5).at(1) * psn_output.at(1);

  psn_activity.at(6) = psn_w.at(6).at(2) * psn_output.at(2) + psn_bias.at(1);
  psn_activity.at(7) = psn_w.at(7).at(3) * psn_output.at(3) + psn_bias.at(1);
  psn_activity.at(8) = psn_w.at(8).at(4) * psn_output.at(4) + psn_bias.at(1);
  psn_activity.at(9) = psn_w.at(9).at(5) * psn_output.at(5) + psn_bias.at(1);

  psn_activity.at(10) = psn_w.at(10).at(6) * psn_output.at(6) + psn_w.at(10).at(7) * psn_output.at(7) + psn_bias.at(2); // final output to motors
  psn_activity.at(11) = psn_w.at(11).at(8) * psn_output.at(8) + psn_w.at(11).at(9) * psn_output.at(9) + psn_bias.at(2); // final output to VRN

  for(unsigned int i=0; i<psn_output.size();i++)
  {
    psn_output.at(i) = tanh(psn_activity.at(i));
  }

  //********PSN end************


  /*******************************************************************************
   *  MODULE 4 VRNs
   *******************************************************************************/

  //**********VRN***************

  //VRN 1 (left)
  vrn_activity.at(0) = vrn_psn_w.at(0).at(11) * psn_output.at(11);
  vrn_activity.at(1) = vrn_input3_w * input.at(3);

  vrn_activity.at(4) = vrn_w.at(4).at(0) * vrn_output.at(0) + vrn_w.at(4).at(1) * vrn_output.at(1) + vrn_bias;
  vrn_activity.at(5) = vrn_w.at(5).at(0) * vrn_output.at(0) + vrn_w.at(5).at(1) * vrn_output.at(1) + vrn_bias;
  vrn_activity.at(6) = vrn_w.at(6).at(0) * vrn_output.at(0) + vrn_w.at(6).at(1) * vrn_output.at(1) + vrn_bias;
  vrn_activity.at(7) = vrn_w.at(7).at(0) * vrn_output.at(0) + vrn_w.at(7).at(1) * vrn_output.at(1) + vrn_bias;

  vrn_activity.at(12) = vrn_w.at(12).at(4) * vrn_output.at(4) + vrn_w.at(12).at(5) * vrn_output.at(5) + vrn_w.at(12).at(6) * vrn_output.at(6)
				                            + vrn_w.at(12).at(7) * vrn_output.at(7); //Output to TL1,2,3

  //VRN 2 (right)
  vrn_activity.at(2) = vrn_psn_w.at(2).at(11) * psn_output.at(11);
  vrn_activity.at(3) = vrn_input4_w * input.at(4);

  vrn_activity.at(8) = vrn_w.at(8).at(2) * vrn_output.at(2) + vrn_w.at(8).at(3) * vrn_output.at(3) + vrn_bias;
  vrn_activity.at(9) = vrn_w.at(9).at(2) * vrn_output.at(2) + vrn_w.at(9).at(3) * vrn_output.at(3) + vrn_bias;
  vrn_activity.at(10) = vrn_w.at(10).at(2) * vrn_output.at(2) + vrn_w.at(10).at(3) * vrn_output.at(3) + vrn_bias;
  vrn_activity.at(11) = vrn_w.at(11).at(2) * vrn_output.at(2) + vrn_w.at(11).at(3) * vrn_output.at(3) + vrn_bias;

  vrn_activity.at(13) = vrn_w.at(13).at(8) * vrn_output.at(8) + vrn_w.at(13).at(9) * vrn_output.at(9) + vrn_w.at(13).at(10) * vrn_output.at(10)
				                            + vrn_w.at(13).at(11) * vrn_output.at(11); //Output to TR1,2,3


  for(unsigned int i=0; i<vrn_output.size();i++)
  {
    vrn_output.at(i) = tanh(vrn_activity.at(i));
  }

  //*******VRN end***************



  /*******************************************************************************
   *  MODULE 5 PRE-MOTOR NEURONS
   *******************************************************************************/

  //*******Motor neurons**********

  //-----Diff CL, CR-------------

  for(unsigned int i=0; i<cr_output.size();i++)
  {
    cr_outputold.at(i) = cr_output.at(i);
  }
  for(unsigned int i=0; i<cl_output.size();i++)
  {
    cl_outputold.at(i) = cl_output.at(i);
  }
  //-----Diff CL, CR end--------

  //NOT USED
  tr_activity.at(0) = -2.5*vrn_output.at(13)+10*input.at(0);
  tr_activity.at(1) = -2.5*vrn_output.at(13)+(-10)*input.at(0);

  tl_activity.at(0) = -2.5*vrn_output.at(12)+10*input.at(0);
  tl_activity.at(1) = -2.5*vrn_output.at(12)+(-10)*input.at(0);

  cl_activity.at(0) = -5.0*psn_output.at(11)-1.0+10*input.at(0);
  cl_activity.at(1) =  5.0*psn_output.at(11)-1.0+10*input.at(0);
  cl_activity.at(2) = -5.0*psn_output.at(11)-1.0+10*input.at(0);
  cr_activity.at(0) =  5.0*psn_output.at(11)-1.0+10*input.at(0);
  cr_activity.at(1) = -5.0*psn_output.at(11)-1.0+10*input.at(0);

  fl_activity.at(0) = -2.2*psn_output.at(10)*input.at(1)+1+10*input.at(0);//+(-2)*input.at(1);
  fl_activity.at(1) = 2.2*psn_output.at(10)*input.at(1)+1+10*input.at(0);//+(-2)*input.at(1);
  fl_activity.at(2) = -2.2*psn_output.at(10)*input.at(1)+1+10*input.at(0);//+(-2)*input.at(1);
  fr_activity.at(0) = -2.2*psn_output.at(10)*input.at(1)+1+10*input.at(0);//+(-2)*input.at(1);


  //USED from hind to front legs

  tr_activity.at(2) = -2.5*vrn_output.at(13);//+(-10)*input.at(0);/////////////USED
  tl_activity.at(2) = -2.5*vrn_output.at(12);//+(-10)*input.at(0);/////////////USED
  cr_activity.at(2) =  5.0*psn_output.at(11)-0.5;//+10*input.at(0);////////////USED
  //For middle FT joint of left and right!!
  fr_activity.at(1) = 5.0*psn_output.at(10)-0.5;//+10*input.at(0);/////////////USED
  //For Front and hind FT joint
  fr_activity.at(2) = -5.0*psn_output.at(11)-0.5;//+10*input.at(0);////////////USED

  //-----CONTROL BACKBONE JOINT HERE------------------------------------------//

  bj_activity.at(0) = 0.0;

  //-----CONTROL BACKBONE JOINT HERE------------------------------------------//

  for(unsigned int i=0; i<tr_output.size();i++)
  {
    tr_output.at(i) = tanh(tr_activity.at(i));
  }

  for(unsigned int i=0; i<tl_output.size();i++)
  {
    tl_output.at(i) = tanh(tl_activity.at(i));
  }

  for(unsigned int i=0; i<cr_output.size();i++)
  {
    cr_output.at(i) = tanh(cr_activity.at(i));
  }

  for(unsigned int i=0; i<cl_output.size();i++)
  {
    cl_output.at(i) = tanh(cl_activity.at(i));
  }

  for(unsigned int i=0; i<fr_output.size();i++)
  {
    fr_output.at(i) = tanh(fr_activity.at(i));
  }

  for(unsigned int i=0; i<fl_output.size();i++)
  {
    fl_output.at(i) = tanh(fl_activity.at(i));
  }

  bj_output.at(0) = -0.05;//-0.15;//tanh(bj_activity.at(0));

  //******Motor neurons end**********


  //*Scaling, shaping, and delay line****

  /*push_back = add one more element in the last with
		a value of the given variable, e.g., "buffer_t" will
		have one more element with the value of "tr_output.at(2)"
   */

  buffer_t.push_back (tr_output.at(2)); // Not used


  //Used
  buffer_tr.push_back (tr_output.at(2));
  buffer_tl.push_back (tl_output.at(2));
  buffer_c.push_back (cr_output.at(2));
  buffer_f.push_back (fr_output.at(2)); //For front and hind legs
  buffer_fm.push_back (fr_output.at(1));//For middle leg

  switch(option_wiring)
  {
    case 1:

      //-----Diff CL, CR-------------

      for(unsigned int i=0; i<cr_output.size();i++)
      {
        diffcr_output.at(i) = cr_output.at(i)-cr_outputold.at(i);
      }
      for(unsigned int i=0; i<cl_output.size();i++)
      {
        diffcl_output.at(i) = cl_output.at(i)-cl_outputold.at(i);
      }

      //postcl.at(0) = cl_output.at(0);

      for(unsigned int i=0; i<postcl.size();i++)
      {
        if(diffcl_output.at(i)<-0.02)// || cl_output.at(i)<0.0)
        {
          postcl.at(i) = -1;
        }
        if(diffcl_output.at(i)>=-0.02)
        {
          postcl.at(i) = cl_output.at(i);
        }
      }

      for(unsigned int i=0; i<postcr.size();i++)
      {
        if(diffcr_output.at(i)<-0.02)// || cl_output.at(i)<0.0)
        {
          postcr.at(i) = -1;
        }
        if(diffcr_output.at(i)>=-0.02)
        {
          postcr.at(i) = cr_output.at(i);
        }
      }

      //-----Diff CL, CR end--------

      //******Wiring with ONLY Tripod******

      m.at(TR0_m) = tr_output.at(0);
      m.at(TR1_m) = tr_output.at(1);
      m.at(TR2_m) = tr_output.at(2);

      m.at(TL0_m) = tl_output.at(0);
      m.at(TL1_m) = tl_output.at(1);
      m.at(TL2_m) = tl_output.at(2);

      m.at(CL0_m) = postcl.at(0);
      m.at(CL1_m) = postcl.at(1);
      m.at(CL2_m) = postcl.at(2);

      m.at(CR0_m) = postcr.at(0);
      m.at(CR1_m) = postcr.at(1);
      m.at(CR2_m) = postcr.at(2);

      m.at(FR0_m) = fr_output.at(0);
      m.at(FR1_m) = fr_output.at(1);
      m.at(FR2_m) = fr_output.at(2);

      m.at(FL0_m) = fl_output.at(0);
      m.at(FL1_m) = fl_output.at(1);
      m.at(FL2_m) = fl_output.at(2);

      m.at(BJ_m) = bj_output.at(0);

      break;
      //******Wiring with ONLY Tripod end*

      //-----------------------------AMOSiiV2_Config------------------------------------------//
    case 2:
      //******Wiring with Delay line*******
      for(unsigned int i=0; i<postclold.size();i++)
      {
        postcrold.at(i) = postcr.at(i);
        postclold.at(i) = postcl.at(i);

      }


      //TR-------------------------


      //KOH change
      if(buffer_tr.size() > 0)
      {
        //m_pre.at(TR2_m) = buffer_tr[buffer_tr.size()-1];
        m_pre.at(TR2_m) = buffer_tr[buffer_tr.size()-1]*input.at(0);//-----------------------------EDurd---------STOP TC---------------------------------------------Omin_KOH_24_6_2012
      }
      else
      {
        m_pre.at(TR2_m) = 0;
      }

      if((buffer_tr.size()-tau)>0)
      {
        m_pre.at(TR1_m) = buffer_tr[buffer_tr.size()-tau-1]*input.at(0);//-----------------------------EDurd------------STOP TC------------------------------------------Omin_KOH_24_6_2012
      }
      else
      {
        m_pre.at(TR1_m) = 0;
      }

      if((buffer_tr.size()-2*tau)>0)
      {
        m_pre.at(TR0_m) = buffer_tr[buffer_tr.size()-(2*tau)-1]*input.at(0);//-----------------------------EDurd-----------STOP TC-------------------------------------------Omin_KOH_24_6_2012
      }
      else
      {
        m_pre.at(TR0_m) = 0;
      }



      //TL-------------------------


      if((buffer_tl.size()-tau_l)>0)
      {
        m_pre.at(TL2_m) = buffer_tl[buffer_tl.size()-tau_l-1]*input.at(0);//-----------------------------EDurd-----------STOP TC-------------------------------------------Omin_KOH_24_6_2012
      }
      else
      {
        m_pre.at(TL2_m) = 0;
      }

      if((buffer_tl.size()-tau-tau_l)>0)
      {
        m_pre.at(TL1_m) = buffer_tl[buffer_tl.size()-tau-tau_l-1]*input.at(0);//-----------------------------EDurd-----------STOP TC-------------------------------------------Omin_KOH_24_6_2012
      }
      else
      {
        m_pre.at(TL1_m) = 0;
      }

      if((buffer_tl.size()-2*tau-tau_l)>0)
      {
        m_pre.at(TL0_m) = buffer_tl[buffer_tl.size()-(2*tau)-tau_l-1]*input.at(0);//-----------------------------EDurd-----------STOP TC-------------------------------------------Omin_KOH_24_6_2012
      }
      else
      {
        m_pre.at(TL0_m) = 0;
      }

      //CR-------------------------

      if(buffer_c.size()>0)
      {
        /*m_pre.at(CR2_m)*/ postcr.at(2) = buffer_c[buffer_c.size()-1];//buffer_c[buffer_c.size()-(2*tau)-1];
      }

      else
      {
        /*m_pre.at(CR2_m)*/ postcr.at(2) = 0;
      }
      /*******CHANGE KOH*post processing *******/
      m_pre.at(CR2_m) = postcr.at(2)*input.at(0);
      if(postcr.at(2)<postcrold.at(2))//postcr.at(2)<threshold_c)
        m_pre.at(CR2_m) = -1*input.at(0);

      if (switchon_footinhibition){
        if(in0.at(L0_fs)>-0.5)
          m_pre.at(CR2_m)	= -1;
      }
      /*******CHANGE KOH*post processing *******/




      /*******CHANGE KOH*post processing *******/
      if((buffer_c.size()-(tau+1))>0)
      {
        /*m_pre.at(CR1_m)*/ postcr.at(1) = buffer_c[buffer_c.size()-(tau+1)-1];
      }
      else
      {
        /*m_pre.at(CR1_m)*/ postcr.at(1) = 0;
      }
      /*******CHANGE KOH*post processing *******/
      m_pre.at(CR1_m) = postcr.at(1)*input.at(0);
      if(postcr.at(1)<postcrold.at(1))//postcr.at(1)<threshold_c)
        m_pre.at(CR1_m) = -1*input.at(0);

      if (switchon_footinhibition){
        if(in0.at(R2_fs)>-0.5)
          m_pre.at(CR1_m)	= -1;
      }
      /*******CHANGE KOH*post processing *******/

      if((buffer_c.size()-2*tau)>0)
      {
        /*m_pre.at(CR0_m)*/ postcr.at(0) = buffer_c[buffer_c.size()-(2*tau+2)-1];
      }
      else
      {
        /*m_pre.at(CR0_m)*/ postcr.at(0) = 0;
      }
      /*******CHANGE KOH*post processing *******/
      m_pre.at(CR0_m) = postcr.at(0)*input.at(0);
      if(postcr.at(0)<postcrold.at(0))//postcr.at(0)<threshold_c)
        m_pre.at(CR0_m) = -1*input.at(0);

      if (switchon_footinhibition){
        if(in0.at(R1_fs)>-0.5)
          m_pre.at(CR0_m)	= -1;
        //std::cout<<"foot inhibition"<< "\n";
      }

      //CL-------------------------

      if((buffer_c.size()-tau_l)>0)
      {
        /*m_pre.at(CL2_m)*/ postcl.at(2)  = buffer_c[buffer_c.size()-tau_l-1];
      }
      else
      {
        /*m_pre.at(CL2_m)*/ postcl.at(2) = 0;
      }
      /*******CHANGE KOH*post processing *******/
      m_pre.at(CL2_m) = postcl.at(2)*input.at(0);
      if(postcl.at(2)<postclold.at(2))//if(postcl.at(2)<threshold_c)
        m_pre.at(CL2_m) = -1*input.at(0);

      if (switchon_footinhibition){
        if(in0.at(R0_fs)>-0.5)
          m_pre.at(CL2_m)	= -1;
      }
      /*******CHANGE KOH*post processing *******/


      if((buffer_c.size()-(tau+1)-tau_l)>0)
      {
        /*m_pre.at(CL1_m)*/ postcl.at(1) = buffer_c[buffer_c.size()-(tau+1)-tau_l-1];
      }
      else
      {
        /*m_pre.at(CL1_m)*/ postcl.at(1) = 0;
      }
      /*******CHANGE KOH*post processing *******/
      m_pre.at(CL1_m) = postcl.at(1)*input.at(0);
      if(postcl.at(1)<postclold.at(1))//if(postcl.at(1)<threshold_c)
        m_pre.at(CL1_m) = -1*input.at(0);

      if (switchon_footinhibition){
        if(in0.at(L2_fs)>-0.5)
          m_pre.at(CL1_m)	= -1;
      }
      /*******CHANGE KOH*post processing *******/

      if((buffer_c.size()-2*tau-tau_l)>0)
      {
        /*m_pre.at(CL0_m)*/ postcl.at(0) = buffer_c[buffer_c.size()-(2*tau+2)-tau_l-1];
        //1.2*buffer_c[buffer_c.size()-(2*tau+3)-1];//buffer_c[buffer_c.size()-1];
      }
      else
      {
        /*m_pre.at(CL0_m)*/ postcl.at(0) = 0;
      }
      /*******CHANGE KOH*post processing *******/
      m_pre.at(CL0_m) = postcl.at(0)*input.at(0);
      if(postcl.at(0)<postclold.at(0))//if(postcl.at(0)<threshold_c)
        m_pre.at(CL0_m) = -1*input.at(0);

      if (switchon_footinhibition){
        if(in0.at(L1_fs)>-0.5)
          m_pre.at(CL0_m)	= -1;
      }
      /*******CHANGE KOH*post processing *******/



      //FR-------------------------


      if(buffer_f.size()>0)
      {
        //m_pre.at(FR2_m) = 1.5*buffer_f[buffer_f.size()-1];
        m_pre.at(FR2_m) = 1.5*buffer_f[buffer_f.size()-1]*-1*input.at(4)*input.at(0);//-----------------------------EDurd------------------------------------------------------Omin_KOH_24_6_2012

      }
      else
      {
        m_pre.at(FR2_m) = 0;
      }

      if((buffer_fm.size()-(tau-1))>0)//((buffer_f.size()-tau)>0)
      {
        //m_pre.at(FR1_m) = buffer_f[buffer_f.size()-tau-1];
        m_pre.at(FR1_m) = -1.2*buffer_fm[buffer_fm.size()-(tau-1)-1]*input.at(0);///----INVERSE--
      }
      else
      {
        m_pre.at(FR1_m) = 0;
      }

      if((buffer_f.size()-2*tau)>0)
      {
        //m_pre.at(FR0_m) = -1.5*buffer_f[buffer_f.size()-(2*tau)-1];
        m_pre.at(FR0_m) = -1.5*buffer_f[buffer_f.size()-(2*tau)-1]*-1*input.at(4)*input.at(0);//-------------------------EDurd----------------------------------------------------------Omin_KOH_24_6_2012
      }
      else
      {
        m_pre.at(FR0_m) = 0;
      }

      //FL-------------------------

      if((buffer_f.size()-tau_l)>0)
      {
        //m_pre.at(FL2_m) = 1.5*buffer_f[buffer_f.size()-tau_l-1];
        m_pre.at(FL2_m) = 1.5*buffer_f[buffer_f.size()-tau_l-1]*-1*input.at(3)*input.at(0);//-----------------------------------------------------------------------------------Omin_KOH_24_6_2012
      }
      else
      {
        m_pre.at(FL2_m) = 0;
      }

      if((buffer_fm.size()-(tau-1)-tau_l)>0)
      {
        //m_pre.at(FL1_m) = buffer_f[buffer_f.size()-tau-tau_l-1];
        m_pre.at(FL1_m) = -1.2*buffer_fm[buffer_fm.size()-(tau-1)-tau_l-1]*input.at(0); ///----INVERSE--
      }
      else
      {
        m_pre.at(FL1_m) = 0;
      }
      if((buffer_f.size()-2*tau-tau_l)>0)
      {
        //m_pre.at(FL0_m) = -1.5*buffer_f[buffer_f.size()-(2*tau)-tau_l-1];//
        m_pre.at(FL0_m) = -1.5*buffer_f[buffer_f.size()-(2*tau)-tau_l-1]*-1*input.at(3)*input.at(0);//--------------------EDurd---------------------------------------------------------------Omin_KOH_24_6_2012
      }
      else
      {
        m_pre.at(FL0_m) = 0;
      }

      m_pre.at(BJ_m) = bj_output.at(0);

      if(buffer_t.size() % (6*tau) == 0)
      {
        buffer_t.erase (buffer_t.begin(), buffer_t.begin() + 3.0*tau);
        buffer_c.erase (buffer_c.begin(), buffer_c.begin() + 3.0*tau);
        buffer_f.erase (buffer_f.begin(), buffer_f.begin() + 3.0*tau);
      }



      if(buffer_tr.size() % (6*tau) == 0)
      {
        buffer_tr.erase (buffer_tr.begin(), buffer_tr.begin() + 3.0*tau);
      }

      if(buffer_tl.size() % (6*tau) == 0)
      {
        buffer_tl.erase (buffer_tl.begin(), buffer_tl.begin() + 3.0*tau);
      }

      if(buffer_fm.size() % (6*tau) == 0)
      {
        buffer_fm.erase (buffer_fm.begin(), buffer_fm.begin() + 3.0*tau);
      }

      break;

  }

  //-----------------------------AMOSiiV2_Config------------------------------------------//


  /*******************************************************************************
   *  MODULE 6 FORWARD MODELS OF PRE MOTOR NEURONS FOR STATE ESTIMATION
   *******************************************************************************/


  //******Reflex mechanisms**********

  //[-1 (stance) = contact, +1 (swing) = no contact]
  //in0.at(R0_fs) = preprosensor.at(R0_fs);--> compare with clipped signal "m.at(CR0_m)" or original one "postcr.at(0)"
  //in0.at(R1_fs) = preprosensor.at(R1_fs);--> compare with clipped signal "m.at(CR1_m)" or original one "postcr.at(1)"
  //in0.at(R2_fs) = preprosensor.at(R2_fs);--> compare with clipped signal "m.at(CR2_m)" or original one "postcr.at(2)"
  //in0.at(L0_fs) = preprosensor.at(L0_fs);--> compare with clipped signal "m.at(CL0_m)" or original one "postcl.at(0)"
  //in0.at(L1_fs) = preprosensor.at(L1_fs);--> compare with clipped signal "m.at(CL1_m)" or original one "postcl.at(1)"
  //in0.at(L2_fs) = preprosensor.at(L2_fs);--> compare with clipped signal "m.at(CL2_m)" or original one "postcl.at(2)"


  reflex_R_fs_old.at(0) = reflex_R_fs.at(0); //R0_fs = 19
  reflex_R_fs_old.at(1) = reflex_R_fs.at(1); //R1_fs = 20
  reflex_R_fs_old.at(2) = reflex_R_fs.at(2); //R2_fs = 21
  reflex_L_fs_old.at(0) = reflex_L_fs.at(0); //L0_fs = 22
  reflex_L_fs_old.at(1) = reflex_L_fs.at(1); //L1_fs = 23
  reflex_L_fs_old.at(2) = reflex_L_fs.at(2); //L2_fs = 24


  reflex_R_fs.at(0) = in0.at(R0_fs); //R0_fs = 19
  reflex_R_fs.at(1) = in0.at(R1_fs); //R1_fs = 20
  reflex_R_fs.at(2) = in0.at(R2_fs); //R2_fs = 21
  reflex_L_fs.at(0) = in0.at(L0_fs); //L0_fs = 22
  reflex_L_fs.at(1) = in0.at(L1_fs); //L1_fs = 23
  reflex_L_fs.at(2) = in0.at(L2_fs); //L2_fs = 24


  if(reflex_R_fs.at(0)>0&&reflex_R_fs.at(1)>0&&reflex_R_fs.at(2)>0&&reflex_L_fs.at(0)>0&&reflex_L_fs.at(1)>0&&reflex_L_fs.at(2)>0)
  {
    allfoot_off_ground++; // check if all legs off ground
  }


  //-----------------------------AMOSiiV2_Config------------------------------------------//
  if(switchon_learnweights)
  {

    if(Control_input == 0.02){ //slow Wave
      //Sim
      //      fmodel_fmodel_cmr_w.at(0)=1.22575; fmodel_cmr_bias.at(0)= 0.717787 ; fmodel_cmr_w.at(0)= 1.18598;//counter4051cin=0.02
      //      fmodel_fmodel_cmr_w.at(1)=1.26893; fmodel_cmr_bias.at(1)= 0.888863 ; fmodel_cmr_w.at(1)= 1.19074;//counter3844cin=0.02
      //      fmodel_fmodel_cmr_w.at(2)=1.4622; fmodel_cmr_bias.at(2)= 0.432198 ; fmodel_cmr_w.at(2)= 1.13514;//counter2488cin=0.02
      //      fmodel_fmodel_cml_w.at(0)=1.2431;fmodel_cml_bias.at(0)= 0.717232 ;fmodel_cml_w.at(0)= 1.18716;//counter4006cin=0.02
      //      fmodel_fmodel_cml_w.at(1)=1.27907;fmodel_cml_bias.at(1)= 0.891913 ;fmodel_cml_w.at(1)= 1.19553;//counter3796cin=0.02
      //      fmodel_fmodel_cml_w.at(2)=1.48568;fmodel_cml_bias.at(2)= 0.426877 ;fmodel_cml_w.at(2)= 1.13848;//counter2440cin=0.02


      //      //Real
      fmodel_fmodel_cmr_w.at(0)=1.29465; fmodel_cmr_bias.at(0)= 0.723935 ; fmodel_cmr_w.at(0)= 1.13595;//counter2348cin=0.02
      fmodel_fmodel_cmr_w.at(1)=1.38465; fmodel_cmr_bias.at(1)= 0.870631 ; fmodel_cmr_w.at(1)= 1.14199;//counter2147cin=0.02
      fmodel_fmodel_cmr_w.at(2)=1.49071; fmodel_cmr_bias.at(2)= 0.519824 ; fmodel_cmr_w.at(2)= 1.06734;//counter1220cin=0.02
      fmodel_fmodel_cml_w.at(0)=1.26498;fmodel_cml_bias.at(0)= 0.712059 ;fmodel_cml_w.at(0)= 1.17424;//counter2300cin=0.02
      fmodel_fmodel_cml_w.at(1)=1.34942;fmodel_cml_bias.at(1)= 0.784529 ;fmodel_cml_w.at(1)= 1.1596;//counter2205cin=0.02
      fmodel_fmodel_cml_w.at(2)=1.58616;fmodel_cml_bias.at(2)= 0.463477 ;fmodel_cml_w.at(2)= 1.0325;//counter744cin=0.02


      //      fmodel_fmodel_cmr_w.at(0)=1.34613; fmodel_cmr_bias.at(0)= 0.65344 ; fmodel_cmr_w.at(0)= 1.03991;//counter1104cin=0.02
      //      fmodel_fmodel_cmr_w.at(1)=0.962063; fmodel_cmr_bias.at(1)= 0.384959 ; fmodel_cmr_w.at(1)= 1.50551;//counter46cin=0.02
      //      fmodel_fmodel_cmr_w.at(2)=1.27542; fmodel_cmr_bias.at(2)= 0.683625 ; fmodel_cmr_w.at(2)= 1.16149;//counter1978cin=0.02
      //      fmodel_fmodel_cml_w.at(0)=1.42786;fmodel_cml_bias.at(0)= 0.777164 ;fmodel_cml_w.at(0)= 1.09799;//counter1698cin=0.02
      //      fmodel_fmodel_cml_w.at(1)=1.06395;fmodel_cml_bias.at(1)= -0.120841 ;fmodel_cml_w.at(1)= 1.3033;//counter0cin=0.02
      //      fmodel_fmodel_cml_w.at(2)=1.23376;fmodel_cml_bias.at(2)= 0.457826 ;fmodel_cml_w.at(2)= 0.911809;//counter18cin=0.02


    }

    else if(Control_input == 0.03){ //fast Wave
      //Sim
      fmodel_fmodel_cmr_w.at(0)=1.21018; fmodel_cmr_bias.at(0)= 0.736802 ; fmodel_cmr_w.at(0)= 1.17313;//counter3354cin=0.03
      fmodel_fmodel_cmr_w.at(1)=1.28654; fmodel_cmr_bias.at(1)= 0.913939 ; fmodel_cmr_w.at(1)= 1.16959;//counter3102cin=0.03
      fmodel_fmodel_cmr_w.at(2)=1.36599; fmodel_cmr_bias.at(2)= 0.55315 ; fmodel_cmr_w.at(2)= 1.19161;//counter2432cin=0.03
      fmodel_fmodel_cml_w.at(0)=1.27915;fmodel_cml_bias.at(0)= 0.751399 ;fmodel_cml_w.at(0)= 1.12231;//counter3125cin=0.03
      fmodel_fmodel_cml_w.at(1)=1.29211;fmodel_cml_bias.at(1)= 0.916084 ;fmodel_cml_w.at(1)= 1.17444;//counter3053cin=0.03
      fmodel_fmodel_cml_w.at(2)=1.41695;fmodel_cml_bias.at(2)= 0.511899 ;fmodel_cml_w.at(2)= 1.17595;//counter2297cin=0.03

      //      //Real
      //      fmodel_fmodel_cmr_w.at(0)=1.63468; fmodel_cmr_bias.at(0)= 0.557352 ; fmodel_cmr_w.at(0)= 1.02995;//counter1545cin=0.03
      //      fmodel_fmodel_cmr_w.at(1)=1.51437; fmodel_cmr_bias.at(1)= 0.714163 ; fmodel_cmr_w.at(1)= 1.11519;//counter2070cin=0.03
      //      fmodel_fmodel_cmr_w.at(2)=1.62665; fmodel_cmr_bias.at(2)= 0.512724 ; fmodel_cmr_w.at(2)= 1.03441;//counter1141cin=0.03
      //      fmodel_fmodel_cml_w.at(0)=1.4124;fmodel_cml_bias.at(0)= 0.73623 ;fmodel_cml_w.at(0)= 1.082;//counter2184cin=0.03
      //      fmodel_fmodel_cml_w.at(1)=1.01696;fmodel_cml_bias.at(1)= -0.270974 ;fmodel_cml_w.at(1)= 1.20752;//counter10cin=0.03
      //      fmodel_fmodel_cml_w.at(2)=1.68643;fmodel_cml_bias.at(2)= 0.433498 ;fmodel_cml_w.at(2)= 0.994526;//counter1093cin=0.03

    }
    else if(Control_input == 0.05){ //slow tetrapod
      //Sim
      //      fmodel_fmodel_cmr_w.at(0)=1.22781; fmodel_cmr_bias.at(0)= 0.753811 ; fmodel_cmr_w.at(0)= 1.17168;//counter2723cin=0.05
      //      fmodel_fmodel_cmr_w.at(1)=1.39275; fmodel_cmr_bias.at(1)= 0.921962 ; fmodel_cmr_w.at(1)= 1.22076;//counter2006cin=0.05
      //      fmodel_fmodel_cmr_w.at(2)=1.63904; fmodel_cmr_bias.at(2)= 0.214554 ; fmodel_cmr_w.at(2)= 0.990325;//counter754cin=0.05
      //      fmodel_fmodel_cml_w.at(0)=1.53058;fmodel_cml_bias.at(0)= 0.535184 ;fmodel_cml_w.at(0)= 1.08716;//counter1674cin=0.05
      //      fmodel_fmodel_cml_w.at(1)=1.31354;fmodel_cml_bias.at(1)= 0.873429 ;fmodel_cml_w.at(1)= 1.17109;//counter2476cin=0.05
      //      fmodel_fmodel_cml_w.at(2)=1.63821;fmodel_cml_bias.at(2)= 0.214011 ;fmodel_cml_w.at(2)= 0.989189;//counter706cin=0.05

      //Real 0.02
      fmodel_fmodel_cmr_w.at(0)=1.29465; fmodel_cmr_bias.at(0)= 0.723935 ; fmodel_cmr_w.at(0)= 1.13595;//counter2348cin=0.02
      fmodel_fmodel_cmr_w.at(1)=1.38465; fmodel_cmr_bias.at(1)= 0.870631 ; fmodel_cmr_w.at(1)= 1.14199;//counter2147cin=0.02
      fmodel_fmodel_cmr_w.at(2)=1.49071; fmodel_cmr_bias.at(2)= 0.519824 ; fmodel_cmr_w.at(2)= 1.06734;//counter1220cin=0.02
      fmodel_fmodel_cml_w.at(0)=1.26498;fmodel_cml_bias.at(0)= 0.712059 ;fmodel_cml_w.at(0)= 1.17424;//counter2300cin=0.02
      fmodel_fmodel_cml_w.at(1)=1.34942;fmodel_cml_bias.at(1)= 0.784529 ;fmodel_cml_w.at(1)= 1.1596;//counter2205cin=0.02
      fmodel_fmodel_cml_w.at(2)=1.58616;fmodel_cml_bias.at(2)= 0.463477 ;fmodel_cml_w.at(2)= 1.0325;//counter744cin=0.02

    }
    else if(Control_input == 0.06){ //slow tetrapod
      //Sim
      fmodel_fmodel_cmr_w.at(0)=1.20602; fmodel_cmr_bias.at(0)= 0.737993 ; fmodel_cmr_w.at(0)= 1.16275;//counter2755cin=0.06
      fmodel_fmodel_cmr_w.at(1)=1.30205; fmodel_cmr_bias.at(1)= 0.872833 ; fmodel_cmr_w.at(1)= 1.14015;//counter2642cin=0.06
      fmodel_fmodel_cmr_w.at(2)=1.656; fmodel_cmr_bias.at(2)= 0.211225 ; fmodel_cmr_w.at(2)= 0.996536;//counter801cin=0.06
      fmodel_fmodel_cml_w.at(0)=1.51165;fmodel_cml_bias.at(0)= 0.473342 ;fmodel_cml_w.at(0)= 1.16315;//counter1382cin=0.06
      fmodel_fmodel_cml_w.at(1)=1.29666;fmodel_cml_bias.at(1)= 0.876311 ;fmodel_cml_w.at(1)= 1.13481;//counter2655cin=0.06
      fmodel_fmodel_cml_w.at(2)=1.58709;fmodel_cml_bias.at(2)= 0.309484 ;fmodel_cml_w.at(2)= 1.05274;//counter1174cin=0.06
    }
    else if(Control_input == 0.14){ //fast tetrapod
      //Sim
      fmodel_fmodel_cmr_w.at(0)=1.50009; fmodel_cmr_bias.at(0)= 0.419673 ; fmodel_cmr_w.at(0)= 1.09759;//counter2353cin=0.14
      fmodel_fmodel_cmr_w.at(1)=1.38196; fmodel_cmr_bias.at(1)= 0.795385 ; fmodel_cmr_w.at(1)= 1.16957;//counter2766cin=0.14
      fmodel_fmodel_cmr_w.at(2)=1.76756; fmodel_cmr_bias.at(2)= 0.259735 ; fmodel_cmr_w.at(2)= 0.764504;//counter769cin=0.14
      fmodel_fmodel_cml_w.at(0)=1.50421;fmodel_cml_bias.at(0)= 0.406372 ;fmodel_cml_w.at(0)= 1.08624;//counter2305cin=0.14
      fmodel_fmodel_cml_w.at(1)=1.39888;fmodel_cml_bias.at(1)= 0.796827 ;fmodel_cml_w.at(1)= 1.16927;//counter2708cin=0.14
      fmodel_fmodel_cml_w.at(2)=1.6796;fmodel_cml_bias.at(2)= 0.232761 ;fmodel_cml_w.at(2)= 0.833529;//counter1727cin=0.14
    }
    else if(Control_input == 0.18){ //tripod
      //Sim
      fmodel_fmodel_cmr_w.at(0)=1.682; fmodel_cmr_bias.at(0)= 0.305406 ; fmodel_cmr_w.at(0)= 0.879117;//counter2274cin=0.18
      fmodel_fmodel_cmr_w.at(1)=1.41075; fmodel_cmr_bias.at(1)= 0.672078 ; fmodel_cmr_w.at(1)= 1.1038;//counter3014cin=0.18
      fmodel_fmodel_cmr_w.at(2)=1.79249; fmodel_cmr_bias.at(2)= 0.38363 ; fmodel_cmr_w.at(2)= 0.752303;//counter1357cin=0.18
      fmodel_fmodel_cml_w.at(0)=1.72231;fmodel_cml_bias.at(0)= 0.35321 ;fmodel_cml_w.at(0)= 0.829056;//counter2157cin=0.18
      fmodel_fmodel_cml_w.at(1)=1.37356;fmodel_cml_bias.at(1)= 0.669967 ;fmodel_cml_w.at(1)= 1.12505;//counter3096cin=0.18
      fmodel_fmodel_cml_w.at(2)=1.67549;fmodel_cml_bias.at(2)= 0.284875 ;fmodel_cml_w.at(2)= 0.826816;//counter1923cin=0.18
    }
    //else if(Control_input != 0.02 || Control_input != 0.03 || Control_input != 0.05 || Control_input != 0.06 || Control_input != 0.14 || Control_input != 0.18)
    else
    {
      //Random weights
      //      fmodel_fmodel_cmr_w.at(0)=1.22575; fmodel_cmr_bias.at(0)= 0.717787 ; fmodel_cmr_w.at(0)= 1.18598;//counter4051cin=0.02
      //      fmodel_fmodel_cmr_w.at(1)=1.26893; fmodel_cmr_bias.at(1)= 0.888863 ; fmodel_cmr_w.at(1)= 1.19074;//counter3844cin=0.02
      //      fmodel_fmodel_cmr_w.at(2)=1.4622; fmodel_cmr_bias.at(2)= 0.432198 ; fmodel_cmr_w.at(2)= 1.13514;//counter2488cin=0.02
      //      fmodel_fmodel_cml_w.at(0)=1.2431;fmodel_cml_bias.at(0)= 0.717232 ;fmodel_cml_w.at(0)= 1.18716;//counter4006cin=0.02
      //      fmodel_fmodel_cml_w.at(1)=1.27907;fmodel_cml_bias.at(1)= 0.891913 ;fmodel_cml_w.at(1)= 1.19553;//counter3796cin=0.02
      //      fmodel_fmodel_cml_w.at(2)=1.48568;fmodel_cml_bias.at(2)= 0.426877 ;fmodel_cml_w.at(2)= 1.13848;//counter2440cin=0.02


      fmodel_fmodel_cmr_w.at(0)=1.29465; fmodel_cmr_bias.at(0)= 0.723935 ; fmodel_cmr_w.at(0)= 1.13595;//counter2348cin=0.02
      fmodel_fmodel_cmr_w.at(1)=1.38465; fmodel_cmr_bias.at(1)= 0.870631 ; fmodel_cmr_w.at(1)= 1.14199;//counter2147cin=0.02
      fmodel_fmodel_cmr_w.at(2)=1.49071; fmodel_cmr_bias.at(2)= 0.519824 ; fmodel_cmr_w.at(2)= 1.06734;//counter1220cin=0.02
      fmodel_fmodel_cml_w.at(0)=1.26498;fmodel_cml_bias.at(0)= 0.712059 ;fmodel_cml_w.at(0)= 1.17424;//counter2300cin=0.02
      fmodel_fmodel_cml_w.at(1)=1.34942;fmodel_cml_bias.at(1)= 0.784529 ;fmodel_cml_w.at(1)= 1.1596;//counter2205cin=0.02
      fmodel_fmodel_cml_w.at(2)=1.58616;fmodel_cml_bias.at(2)= 0.463477 ;fmodel_cml_w.at(2)= 1.0325;//counter744cin=0.02


    }


    //      //Real
    fmodel_fmodel_cmr_w.at(0)=1.29465; fmodel_cmr_bias.at(0)= 0.723935 ; fmodel_cmr_w.at(0)= 1.13595;//counter2348cin=0.02
    fmodel_fmodel_cmr_w.at(1)=1.38465; fmodel_cmr_bias.at(1)= 0.870631 ; fmodel_cmr_w.at(1)= 1.14199;//counter2147cin=0.02
    fmodel_fmodel_cmr_w.at(2)=1.49071; fmodel_cmr_bias.at(2)= 0.519824 ; fmodel_cmr_w.at(2)= 1.06734;//counter1220cin=0.02
    fmodel_fmodel_cml_w.at(0)=1.26498;fmodel_cml_bias.at(0)= 0.712059 ;fmodel_cml_w.at(0)= 1.17424;//counter2300cin=0.02
    fmodel_fmodel_cml_w.at(1)=1.34942;fmodel_cml_bias.at(1)= 0.784529 ;fmodel_cml_w.at(1)= 1.1596;//counter2205cin=0.02
    fmodel_fmodel_cml_w.at(2)=1.58616;fmodel_cml_bias.at(2)= 0.463477 ;fmodel_cml_w.at(2)= 1.0325;//counter744cin=0.02

  }
  //-----------------------------AMOSiiV2_Config------------------------------------------//

  //Ebd_2D
  //m_pre.at(CR0_m), m_pre.at(CR1_m), m_pre.at(CR2_m), m_pre.at(CL0_m), m_pre.at(CL1_m), m_pre.at(CL2_m)

  if(switchon_ED == true)
  {
    int delay_tau = 5;

    delay_CR0.push_back (m_pre.at(CR0_m));
    delay_CR1.push_back (m_pre.at(CR1_m));
    delay_CR2.push_back (m_pre.at(CR2_m));
    delay_CL0.push_back (m_pre.at(CL0_m));
    delay_CL1.push_back (m_pre.at(CL1_m));
    delay_CL2.push_back (m_pre.at(CL2_m));

    if((delay_CR0.size()-delay_tau)>0)
    {
      m_pre_delay.at(CR0_m) = delay_CR0[delay_CR0.size()-delay_tau-1]; // 5 time steps delay
    }
    else
    {
      m_pre_delay.at(CR0_m) = 0;
    }


    if((delay_CR1.size()-delay_tau)>0)
    {
      m_pre_delay.at(CR1_m) = delay_CR1[delay_CR1.size()-delay_tau-1]; // 5 time steps delay
    }
    else
    {
      m_pre_delay.at(CR1_m) = 0;
    }

    if((delay_CR2.size()-delay_tau)>0)
    {
      m_pre_delay.at(CR2_m) = delay_CR2[delay_CR2.size()-delay_tau-1]; // 5 time steps delay
    }
    else
    {
      m_pre_delay.at(CR2_m) = 0;
    }

    if((delay_CL0.size()-delay_tau)>0)
    {
      m_pre_delay.at(CL0_m) = delay_CL0[delay_CL0.size()-delay_tau-1]; // 5 time steps delay
    }
    else
    {
      m_pre_delay.at(CL0_m) = 0;
    }

    if((delay_CL1.size()-delay_tau)>0)
    {
      m_pre_delay.at(CL1_m) = delay_CL1[delay_CL1.size()-delay_tau-1]; // 5 time steps delay
    }
    else
    {
      m_pre_delay.at(CL1_m) = 0;
    }

    if((delay_CL2.size()-delay_tau)>0)
    {
      m_pre_delay.at(CL2_m) = delay_CL2[delay_CL2.size()-delay_tau-1]; // 5 time steps delay
    }
    else
    {
      m_pre_delay.at(CL2_m) = 0;
    }

  }



  //------------CR Loop---------//
  //-1,..,+1--> tanh

  if(global_count>500)
  {
    switch(option_fmodel)
    {
      case 1:

        for(unsigned int i=0; i<fmodel_cmr_activity.size();i++)
        {
          //Forward model of Coxa right motor signals //Recurrent single neuron
          fmodel_cmr_activity.at(i) = m_pre.at(i+CR0_m/*6*/)*fmodel_cmr_w.at(i)+fmodel_cmr_output.at(i)*fmodel_fmodel_cmr_w.at(i)+fmodel_cmr_bias.at(i);
          fmodel_cmr_output.at(i) = tanh(fmodel_cmr_activity.at(i));

          //Post processing
          fmodel_cmr_outputfinal.at(i) = tanh(fmodel_cmr_output.at(i)*fmodel_post_cmr_w.at(i));

          //Calculate error
          fmodel_cmr_error.at(i) = reflex_R_fs.at(i)-fmodel_cmr_outputfinal.at(i) /*regulate error*/; //target - output // only positive error
          acc_cmr_error_old.at(i) = fmodel_cmr_error.at(i);// for elevator reflex

          if(switchon_purefootsignal)//Only foot contact signal
          {
            fmodel_cmr_error.at(i) = reflex_R_fs.at(i)-(-1/*constant error*/); //target - output // only positive error
          }


          double error_threshold = 0.15;

          // Error threshold
          if(fmodel_cmr_error.at(i)<error_threshold)//0.05)//-1)
          {
            fmodel_cmr_error.at(i) = 0.0;

            if(softlanding)// Later use sensor to activate!!!! e.g., acc sensor "g"
            {
              if(allfoot_off_ground> 50)
              {
                acc_cmr_error.at(i) = 0.0; // reset to immediately reset as soon as no error!!!!!! GOOD WHEN DROPPING ROBOT ON FLOOR
              }
              if(allfoot_off_ground> 100)
              {
                allfoot_off_ground = 0;
              }
            }

            //acc_cmr_error_elev.at(i) = 0.0; // reset
          }

          //Positive Error signal for controlling searching reflexes
          acc_cmr_error.at(i) += abs(fmodel_cmr_error.at(i));

          //reset at swing phase
          if(m_pre.at(i+CR0_m/*6*/)>-0.7) //Reset error every Swing phase by detecting CR motor signal
            acc_cmr_error.at(i) = 0;

          //Negative Error signal for controlling elevator reflexes
          if(acc_cmr_error_old.at(i)<0)
          {
            acc_cmr_error_elev.at(i) += abs(acc_cmr_error_old.at(i));
            error_cmr_elev.at(i) = 1.0;
          }
          if(acc_cmr_error_old.at(i)>0)
          {
            acc_cmr_error_elev.at(i) = 0.0;
            error_cmr_elev.at(i) = 0.0;
          }


          //--------Stop learning process if error not appear for 500 time steps!
          //Count distance between two error peaks
          if(abs(fmodel_cmr_error.at(i)) == 0.0)
          {
            counter_cr.at(i)++;
          }
          if(abs(fmodel_cmr_error.at(i)) > error_threshold)
          {

            counter_cr.at(i) = 0;

          }
          /*1) TO DO NEED TO BE ADJUSTED THIS NUMBER "500"*/

          if(counter_cr.at(i) > 1000)//500) // 500 time steps !! need to be adaptive !! later
          {
            lr_fmodel_cr.at(i) = 0;

            //bj_output.at(0) = 1.0;//-0.05;//tr_activity.at(2);

          }


          //--------Stop learning process if error not appear for 500 time steps!
          //learning rule
          if(switchon_learnweights) // No learning
          {
            lr_fmodel_cr.at(i) = 0;
          }
          fmodel_fmodel_cmr_w.at(i)+= lr_fmodel_cr.at(i)*fmodel_cmr_error.at(i); // learn rear part

          //fmodel_cmr_bias.at(i) += lr_fmodel_cr.at(i)*0.5*fmodel_cmr_error.at(i);

          //learning delta rule with gradient descent
          //fmodel_fmodel_cmr_w.at(i) += lr_fmodel_cr.at(i)*fmodel_cmr_error.at(i)*(1-fmodel_cmr_outputfinal.at(i)*fmodel_cmr_outputfinal.at(i));
          //fmodel_cmr_w.at(i) += lr_fmodel_cr.at(i)*fmodel_cmr_error.at(i)*(1-fmodel_cmr_outputfinal.at(i)*fmodel_cmr_outputfinal.at(i));

          std::cout<<"fmodel_fmodel_cmr_w.at("<<i<<")="<<fmodel_fmodel_cmr_w.at(i)<<";//fmodel_cmr_bias="<<" "<<fmodel_cmr_bias.at(i)<<" "<<"fmodel_cmr_w.at="<<" "<<fmodel_cmr_w.at(i)<<";//counter"<<counter_cr.at(i)<<"cin="<<Control_input<< "\n";

        }

        //------------CL Loop---------//
        //-1,..,+1--> tanh
        for(unsigned int i=0; i<fmodel_cml_activity.size();i++)
        {
          //Forward model of Coxa left motor signals //Recurrent single neuron
          fmodel_cml_activity.at(i) = m_pre.at(i+CL0_m/*9*/)*fmodel_cml_w.at(i)+fmodel_cml_output.at(i)*fmodel_fmodel_cml_w.at(i)+fmodel_cml_bias.at(i);
          fmodel_cml_output.at(i) = tanh(fmodel_cml_activity.at(i));

          //Post processing
          fmodel_cml_outputfinal.at(i) = tanh(fmodel_cml_output.at(i)*fmodel_post_cml_w.at(i));

          //Calculate error

          fmodel_cml_error.at(i) = reflex_L_fs.at(i)-fmodel_cml_outputfinal.at(i) /*regulate error*/; //target - output // only positive error
          acc_cml_error_old.at(i) = fmodel_cml_error.at(i);// for elevator reflex

          if(switchon_purefootsignal)//Only foot contact signal
          {
            fmodel_cml_error.at(i) = reflex_L_fs.at(i)-(-1/*constant error*/); //target - output // only positive error
          }


          double error_threshold = 0.15;
          // Error threshold
          if(fmodel_cml_error.at(i)<error_threshold)//0.05)//-1)
          {
            fmodel_cml_error.at(i) = 0.0;

            if(softlanding)// Later use sensor to activate!!!! e.g., acc sensor "g"
            {
              if(allfoot_off_ground> 50)
              {
                acc_cml_error.at(i) = 0.0; // reset to immediately reset as soon as no error!!!!!! GOOD WHEN DROPPING ROBOT ON FLOOR
              }
              if(allfoot_off_ground> 100)
              {
                allfoot_off_ground = 0;
              }
            }
            //acc_cml_error.at(i) = 0.0;// reset to immediately reset as soon as no error!!!!!! GOOD WHEN DROPPING ROBOT ON FLOOR
            //acc_cml_error_elev.at(i) = 0.0; // reset
          }

          //Positive Error signal for controlling searching reflexes
          acc_cml_error.at(i) += abs(fmodel_cml_error.at(i));

          //reset at swing phase
          if(m_pre.at(i+CL0_m/*9*/)>-0.7) //Reset error every Swing phase by detecting CR motor signal
            acc_cml_error.at(i) = 0;

          //Negative Error signal for controlling elevator reflexes
          if(acc_cml_error_old.at(i)<0)
          {
            acc_cml_error_elev.at(i) += abs(acc_cml_error_old.at(i));
            error_cml_elev.at(i) = 1.0;
          }
          if(acc_cml_error_old.at(i)>0)
          {
            acc_cml_error_elev.at(i) = 0.0;
            error_cml_elev.at(i) = 0.0;
          }


          //--------Stop learning process if error not appear for 500 time steps!
          if(abs(fmodel_cml_error.at(i)) == 0.0)
          {
            counter_cl.at(i)++;
          }

          if(abs(fmodel_cml_error.at(i)) > error_threshold)
          {
            counter_cl.at(i) = 0;
          }

          if(counter_cl.at(i) > 1000)//500) // 500 time steps !! need to be adaptive !! later
          {
            lr_fmodel_cl.at(i) = 0;
          }

          //--------Stop learning process if error not appear for 500 time steps!


          //learning delta rule
          if(switchon_learnweights) // No learning
          {
            lr_fmodel_cl.at(i) = 0;
          }
          fmodel_fmodel_cml_w.at(i)+= lr_fmodel_cl.at(i)*fmodel_cml_error.at(i); // learn rear part

          //fmodel_cml_bias.at(i) += lr_fmodel_cl.at(i)*0.5*fmodel_cml_error.at(i);

          //learning delta rule with gradient descent
          //fmodel_fmodel_cml_w.at(i) += lr_fmodel_cl.at(i)*fmodel_cml_error.at(i)*(1-fmodel_cml_outputfinal.at(i)*fmodel_cml_outputfinal.at(i));
          //fmodel_cml_w.at(i) += lr_fmodel_cl.at(i)*fmodel_cml_error.at(i)*(1-fmodel_cml_outputfinal.at(i)*fmodel_cml_outputfinal.at(i));

          std::cout<<"fmodel_fmodel_cml_w.at("<<i<<")="<<fmodel_fmodel_cml_w.at(i)<<";//fmodel_cml_bias="<<" "<<fmodel_cml_bias.at(i)<<" "<<"fmodel_cml_w.at="<<" "<<fmodel_cml_w.at(i)<<";//counter"<<counter_cl.at(i)<<"cin="<<Control_input<< "\n";
        }

        break;

      case 2:
        for(unsigned int i=0; i<fmodel_cmr_activity.size();i++)
        {
          //Forward model of Coxa right motor signals //Recurrent single neuron
          fmodel_cmr_activity.at(i) = m_pre.at(i+CR0_m/*6*/)*fmodel_cmr_w.at(i)+fmodel_cmr_output.at(i)*fmodel_fmodel_cmr_w.at(i)+fmodel_cmr_bias.at(i);
          fmodel_cmr_output.at(i) = tanh(fmodel_cmr_activity.at(i));

          //Post processing
          fmodel_cmr_outputfinal.at(i) = tanh(fmodel_cmr_output.at(i)*fmodel_post_cmr_w.at(i));

          //Calculate error
          fmodel_cmr_error.at(i) = reflex_R_fs.at(i)-fmodel_cmr_output.at(i); //target - output // only positive error
          // Error threshold
          if(fmodel_cmr_error.at(i)<0.0)
          {
            fmodel_cmr_error.at(i) = 0.0;
          }


          //Lowpass filter
          lowpass_cmr_error_activity.at(i) = fmodel_cmr_error.at(i)*lowpass_cmr_w.at(i)+lowpass_cmr__error_output.at(i)*lowpass_lowpass_cmr_w.at(i)+lowpass_cmr_bias.at(i);
          lowpass_cmr__error_output.at(i) = sigmoid(lowpass_cmr_error_activity.at(i));
          //				lowpass_cmr_error_activity.at(i) = fmodel_cmr_error.at(i)*0.1+lowpass_cmr__error_output.at(i)*0.9;
          //				lowpass_cmr__error_output.at(i) = lowpass_cmr_error_activity.at(i);//tanh(lowpass_cmr_error_activity.at(i));


          //--------Stop learning process if error not appear for 500 time steps!
          if(abs(fmodel_cmr_error.at(i)) == 0.0)
          {
            counter_cr.at(i)++;
          }
          if(abs(fmodel_cmr_error.at(i)) > 0.0)
          {
            counter_cr.at(i) = 0;
          }
          if(counter_cr.at(i) > 500) // 500 time steps !! need to be adaptive !! later
          {
            lr_fmodel_cr.at(i) = 0;
          }

          acc_cmr_error.at(i) += abs(fmodel_cmr_error.at(i));
          //--------Stop learning process if error not appear for 500 time steps!

          //learning rule
          fmodel_fmodel_cmr_w.at(i)+= lr_fmodel_cr.at(i)*fmodel_cmr_error.at(i); // learn rear part

          //learning delta rule with gradient descent
          //fmodel_fmodel_cmr_w.at(i) += lr_fmodel_cr.at(i)*fmodel_cmr_error.at(i)*(1-fmodel_cmr_outputfinal.at(i)*fmodel_cmr_outputfinal.at(i));
          //fmodel_cmr_w.at(i) += lr_fmodel_cr.at(i)*fmodel_cmr_error.at(i)*(1-fmodel_cmr_outputfinal.at(i)*fmodel_cmr_outputfinal.at(i));

        }

        //------------CL Loop---------//
        //-1,..,+1--> tanh
        for(unsigned int i=0; i<fmodel_cml_activity.size();i++)
        {
          //Forward model of Coxa left motor signals //Recurrent single neuron
          fmodel_cml_activity.at(i) = m_pre.at(i+CL0_m/*9*/)*fmodel_cml_w.at(i)+fmodel_cml_output.at(i)*fmodel_fmodel_cml_w.at(i)+fmodel_cml_bias.at(i);
          fmodel_cml_output.at(i) = tanh(fmodel_cml_activity.at(i));

          //Post processing
          fmodel_cml_outputfinal.at(i) = tanh(fmodel_cml_output.at(i)*fmodel_post_cml_w.at(i));

          //Calculate error
          fmodel_cml_error.at(i) = reflex_L_fs.at(i)-fmodel_cml_output.at(i); //fmodel_cml_output.at(0);//fmodel_cml_outputfinal.at(0); // target - output // only positive error

          // Error threshold
          if(fmodel_cml_error.at(i)<-1)//0.05
          {
            fmodel_cml_error.at(i) = 0.0;
          }

          //Lowpass filter
          lowpass_cml_error_activity.at(i) = fmodel_cml_error.at(i)*lowpass_cml_w.at(i)+lowpass_cml__error_output.at(i)*lowpass_lowpass_cml_w.at(i)+lowpass_cml_bias.at(i);
          lowpass_cml__error_output.at(i) = tanh(lowpass_cml_error_activity.at(i));


          //--------Stop learning process if error not appear for 500 time steps!
          if(abs(fmodel_cml_error.at(i)) == 0.0)
            counter_cl.at(i)++;
          if(abs(fmodel_cml_error.at(i)) > 0.0)
            counter_cl.at(i) = 0;

          if(counter_cl.at(i) > 500) // 500 time steps !! need to be adaptive !! later
            lr_fmodel_cl.at(i) = 0;
          acc_cml_error.at(i) += abs(fmodel_cml_error.at(i));
          //--------Stop learning process if error not appear for 500 time steps!


          //learning delta rule
          fmodel_fmodel_cml_w.at(i)+= lr_fmodel_cl.at(i)*fmodel_cml_error.at(i); // learn rear part

          //learning delta rule with gradient descent
          //fmodel_fmodel_cml_w.at(i) += lr_fmodel_cl.at(i)*fmodel_cml_error.at(i)*(1-fmodel_cml_outputfinal.at(i)*fmodel_cml_outputfinal.at(i));
          //fmodel_cml_w.at(i) += lr_fmodel_cl.at(i)*fmodel_cml_error.at(i)*(1-fmodel_cml_outputfinal.at(i)*fmodel_cml_outputfinal.at(i));
        }

        break;

      case 3: // Emd_2D

        //------------CR Loop---------//
        for(unsigned int i=0; i<fcn_r.size();i++)
        {
          //					a1_r.at(0)=616.057
          //					a2_r.at(0)=-11.5996
          //					a3_r.at(0)=627.656
          //					a1_r.at(1)=5.12269e-66
          //					a2_r.at(1)=-5.35422e-66
          //					a3_r.at(1)=-2.49308e-65
          //					a1_r.at(2)=-1.48251e-58
          //					a2_r.at(2)=1.41991e-58
          //					a3_r.at(2)=-2.90242e-58
          //					a1_l.at(0)=-8.28008e-64
          //					a2_l.at(0)=7.60086e-64
          //					a3_l.at(0)=-1.58809e-63
          //					a1_l.at(1)=0.309315
          //					a2_l.at(1)=1.71813
          //					a3_l.at(1)=1.47389
          //					a1_l.at(2)=5.02269e-62
          //					a2_l.at(2)=-5.19715e-62
          //					a3_l.at(2)=-1.19105e-61

          fcn_r.at(i) = a1_r.at(i)+a2_r.at(i)*m_pre.at(i+CR0_m/*6 CR0_m*/)+a3_r.at(i)*m_pre_delay.at(i+CR0_m/*6 CR0_m*/); //CR0
          normxsq_r.at(i) = 1+m_pre.at(i+CR0_m/*6 CR0_m*/)*m_pre.at(i+CR0_m/*6 CR0_m*/)+m_pre_delay.at(i+CR0_m/*6 CR0_m*/)*m_pre_delay.at(i+CR0_m/*6 CR0_m*/); //CR0

          if(fcn_r.at(i)*reflex_R_fs.at(i)<0) //R0
          {
            fac_r.at(i)= -fcn_r.at(i)/normxsq_r.at(i);
            a1_r.at(i) = a1_r.at(i)+fac_r.at(i);
            a2_r.at(i) = a2_r.at(i)+fac_r.at(i)*m_pre.at(i+CR0_m/*6 CR0_m*/);
            a3_r.at(i) = a3_r.at(i)+fac_r.at(i)*m_pre_delay.at(i+CR0_m/*6 CR0_m*/);

          }
          //Sign function//
          if(fac_r.at(i)<0)
            pred_r.at(i) = -1;
          if(fac_r.at(i)>0)
            pred_r.at(i) = 1;
          if(fac_r.at(i)==0)
            pred_r.at(i) = 0;

          //Prediction values of Ctr signals
          //pred_r.at(0) = CR0, pred_r.at(1) = CR1, pred_r.at(2) = CR2
          //outFilenlc1<<m_pre.at(CR0_m)<<' '<<m_pre_delay.at(CR0_m)<<' '<<reflex_R_fs.at(0)<<' '<<fcn.at(0)<<endl;

          std::cout<<"a1_r.at("<<i<<")="<<a1_r.at(i)<< "\n";
          std::cout<<"a2_r.at("<<i<<")="<<a2_r.at(i)<< "\n";
          std::cout<<"a3_r.at("<<i<<")="<<a3_r.at(i)<< "\n";
        }

        //------------CL Loop---------//
        for(unsigned int i=0; i<fcn_l.size();i++)
        {
          fcn_l.at(i) = a1_l.at(i)+a2_l.at(i)*m_pre.at(i+CL0_m/*9 CL0_m*/)+a3_l.at(i)*m_pre_delay.at(i+CL0_m/*9 CL0_m*/); //CL0
          normxsq_l.at(i) = 1+m_pre.at(i+CL0_m/*9 CL0_m*/)*m_pre.at(i+CL0_m/*9 CL0_m*/)+m_pre_delay.at(i+CL0_m/*9 CL0_m*/)*m_pre_delay.at(i+CL0_m/*9 CL0_m*/); //CL0

          if(fcn_l.at(i)*reflex_L_fs.at(i)<0) //L0
          {
            fac_l.at(i)= -fcn_l.at(i)/normxsq_l.at(i);
            a1_l.at(i) = a1_l.at(i)+fac_l.at(i);
            a2_l.at(i) = a2_l.at(i)+fac_l.at(i)*m_pre.at(i+CL0_m/*9 CL0_m*/);
            a3_l.at(i) = a3_l.at(i)+fac_l.at(i)*m_pre_delay.at(i+CL0_m/*9 CL0_m*/);

          }
          //Sign function//
          if(fac_l.at(i)<0)
            pred_l.at(i) = -1;
          if(fac_l.at(i)>0)
            pred_l.at(i) = 1;
          if(fac_l.at(i)==0)
            pred_l.at(i) = 0;

          //Prediction values of Ctr signals
          //pred_l.at(0) = CL0, pred_l.at(1) = CL1, pred_l.at(2) = CL2
          //outFilenlc1<<m_pre.at(CR0_m)<<' '<<m_pre_delay.at(CR0_m)<<' '<<reflex_R_fs.at(0)<<' '<<fcn.at(0)<<endl;
          std::cout<<"a1_l.at("<<i<<")="<<a1_l.at(i)<< "\n";
          std::cout<<"a2_l.at("<<i<<")="<<a2_l.at(i)<< "\n";
          std::cout<<"a3_l.at("<<i<<")="<<a3_l.at(i)<< "\n";
        }


        break;


      case 4:

        for(unsigned int i=0; i<fmodel_cmr_activity.size();i++)
        {
          double lowpass_error_gain;
          lowpass_error_gain = 0.9;

          low_pass_fmodel_cmr_error_old.at(i) = low_pass_fmodel_cmr_error.at(i);
          fmodel_cmr_output_old.at(i) = fmodel_cmr_output.at(i);

          //Forward model of Coxa right motor signals //Recurrent single neuron
          fmodel_cmr_activity.at(i) = m_pre.at(i+CR0_m/*6*/)*fmodel_cmr_w.at(i)+fmodel_cmr_output.at(i)*fmodel_fmodel_cmr_w.at(i)+fmodel_cmr_bias.at(i);
          fmodel_cmr_output.at(i) = tanh(fmodel_cmr_activity.at(i));

          //Calculate error for learning
          fmodel_cmr_errorW.at(i) = reflex_R_fs.at(i)-fmodel_cmr_output.at(i);//-fmodel_cmr_outputfinal.at(i) /*regulate error*/; //target - output // only positive error

          //Post processing
          //fmodel_cmr_outputfinal.at(i) = fmodel_cmr_output.at(i);//tanh(fmodel_cmr_output.at(i)*fmodel_post_cmr_w.at(i));
          fmodel_cmr_outputfinal.at(i) = tanh(fmodel_cmr_output.at(i)*fmodel_post_cmr_w.at(i));
          //Calculate error
          fmodel_cmr_error.at(i) = reflex_R_fs.at(i)-fmodel_cmr_outputfinal.at(i) /*regulate error*/; //target - output // only positive error



          low_pass_fmodel_cmr_error.at(i) = low_pass_fmodel_cmr_error_old.at(i)*lowpass_error_gain+(1-lowpass_error_gain)*fmodel_cmr_error.at(i);

          acc_cmr_error_old.at(i) = fmodel_cmr_error.at(i);// for elevator reflex

          if(switchon_purefootsignal)//Only foot contact signal
          {
            fmodel_cmr_error.at(i) = reflex_R_fs.at(i)-(-1/*constant error*/); //target - output // only positive error
          }


          double error_threshold = 0.15;

          // Error threshold
          if(fmodel_cmr_error.at(i)<error_threshold)//0.05)//-1)
          {
            //fmodel_cmr_error.at(i) = 0.0;

            if(softlanding)// Later use sensor to activate!!!! e.g., acc sensor "g"
            {
              if(allfoot_off_ground> 50)
              {
                acc_cmr_error.at(i) = 0.0; // reset to immediately reset as soon as no error!!!!!! GOOD WHEN DROPPING ROBOT ON FLOOR
              }
              if(allfoot_off_ground> 100)
              {
                allfoot_off_ground = 0;
              }
            }

            //acc_cmr_error_elev.at(i) = 0.0; // reset
          }

          //Positive Error signal for controlling searching reflexes
          acc_cmr_error.at(i) += abs(fmodel_cmr_error.at(i));

          if(abs(fmodel_cmr_error.at(i)) < 0.05)
            acc_cmr_error_posi_neg.at(i) = 0.0;
          acc_cmr_error_posi_neg.at(i) += fmodel_cmr_error.at(i);


          //reset at swing phase
          if(m_pre.at(i+CR0_m/*6*/)>-0.7) //Reset error every Swing phase by detecting CR motor signal
            acc_cmr_error.at(i) = 0;

          //Negative Error signal for controlling elevator reflexes
          if(acc_cmr_error_old.at(i)<0)
          {
            acc_cmr_error_elev.at(i) += abs(acc_cmr_error_old.at(i));
            error_cmr_elev.at(i) = 1.0;
          }
          if(acc_cmr_error_old.at(i)>0)
          {
            acc_cmr_error_elev.at(i) = 0.0;
            error_cmr_elev.at(i) = 0.0;
          }


          //--------Stop learning process if error not appear for 500 time steps!
          //Count distance between two error peaks
          double error_threshold_2 = 0.2;//0.25;
          if(abs(low_pass_fmodel_cmr_error.at(i))<error_threshold_2)//abs(fmodel_cmr_error.at(i)) == 0.0)
          {
            counter_cr.at(i)++;
          }
          if(abs(low_pass_fmodel_cmr_error.at(i))>error_threshold_2)//abs(fmodel_cmr_error.at(i)) > error_threshold)
          {

            counter_cr.at(i) = 0;

          }
          /*1) TO DO NEED TO BE ADJUSTED THIS NUMBER "500"*/

          if(counter_cr.at(i) > 300)//500)//1000) // 500 time steps !! need to be adaptive !! later
          {
            lr_fmodel_cr.at(i) = 0;
            low_pass_fmodel_cmr_error.at(i) = 0;

            //bj_output.at(0) = 1.0;//-0.05;//tr_activity.at(2);

          }


          //--------Stop learning process if error not appear for 500 time steps!
          //learning rule
          if(switchon_learnweights) // No learning
          {
            lr_fmodel_cr.at(i) = 0;
          }

          //learning delta rule with gradient descent

          //Recurrent weight, fmodel_cmr_outputfinal.at(i)
          fmodel_fmodel_cmr_w.at(i) += lr_fmodel_cr.at(i)*(fmodel_cmr_errorW.at(i)*(1-fmodel_cmr_output.at(i)*fmodel_cmr_output.at(i))*fmodel_cmr_output_old.at(i));

          //          if(fmodel_fmodel_cmr_w.at(i)<0.0)
          //            fmodel_fmodel_cmr_w.at(i) = 0.0;


          //Input weight, fmodel_cmr_outputfinal.at(i)
          fmodel_cmr_w.at(i) += lr_fmodel_cr.at(i)*(fmodel_cmr_errorW.at(i)*(1-fmodel_cmr_output.at(i)*fmodel_cmr_output.at(i))*m_pre.at(i+CR0_m/*6*/));
          //          if(fmodel_cmr_w.at(i)<0.0)
          //            fmodel_cmr_w.at(i) = 0.0;

          //Bias
          fmodel_cmr_bias.at(i) += lr_fmodel_cr.at(i)*(fmodel_cmr_errorW.at(i)*(1-fmodel_cmr_output.at(i)*fmodel_cmr_output.at(i)));
          //          if(fmodel_cmr_bias.at(i)<0.0)
          //            fmodel_cmr_bias.at(i) = 0.0;


          std::cout<<"fmodel_fmodel_cmr_w.at("<<i<<")="<<fmodel_fmodel_cmr_w.at(i)<<"; fmodel_cmr_bias.at("<<i<<")="<<" "<<fmodel_cmr_bias.at(i)<<" "<<"; fmodel_cmr_w.at("<<i<<")="<<" "<<fmodel_cmr_w.at(i)<<";//counter"<<counter_cr.at(i)<<"cin="<<Control_input<< "\n";

        }

        //------------CL Loop---------//
        //-1,..,+1--> tanh
        for(unsigned int i=0; i<fmodel_cml_activity.size();i++)
        {

          double lowpass_error_gain;
          lowpass_error_gain = 0.9;

          low_pass_fmodel_cml_error_old.at(i) = low_pass_fmodel_cml_error.at(i);
          fmodel_cml_output_old.at(i) = fmodel_cml_output.at(i);

          //Forward model of Coxa left motor signals //Recurrent single neuron
          fmodel_cml_activity.at(i) = m_pre.at(i+CL0_m/*9*/)*fmodel_cml_w.at(i)+fmodel_cml_output.at(i)*fmodel_fmodel_cml_w.at(i)+fmodel_cml_bias.at(i);
          fmodel_cml_output.at(i) = tanh(fmodel_cml_activity.at(i));

          //Calculate error for learning
          fmodel_cml_errorW.at(i) = reflex_L_fs.at(i)-fmodel_cml_output.at(i);

          //Post processing
          fmodel_cml_outputfinal.at(i) = tanh(fmodel_cml_output.at(i)*fmodel_post_cml_w.at(i));

          //Calculate error
          fmodel_cml_error.at(i) = reflex_L_fs.at(i)-fmodel_cml_outputfinal.at(i) /*regulate error*/; //target - output // only positive error
          low_pass_fmodel_cml_error.at(i) = low_pass_fmodel_cml_error_old.at(i)*lowpass_error_gain+(1-lowpass_error_gain)*fmodel_cml_error.at(i);
          acc_cml_error_old.at(i) = fmodel_cml_error.at(i);// for elevator reflex


          if(switchon_purefootsignal)//Only foot contact signal
          {
            fmodel_cml_error.at(i) = reflex_L_fs.at(i)-(-1/*constant error*/); //target - output // only positive error
          }


          double error_threshold = 0.15;
          // Error threshold
          if(fmodel_cml_error.at(i)<error_threshold)//0.05)//-1)
          {
            //fmodel_cml_error.at(i) = 0.0;

            if(softlanding)// Later use sensor to activate!!!! e.g., acc sensor "g"
            {
              if(allfoot_off_ground> 50)
              {
                acc_cml_error.at(i) = 0.0; // reset to immediately reset as soon as no error!!!!!! GOOD WHEN DROPPING ROBOT ON FLOOR
              }
              if(allfoot_off_ground> 100)
              {
                allfoot_off_ground = 0;
              }
            }
            //acc_cml_error.at(i) = 0.0;// reset to immediately reset as soon as no error!!!!!! GOOD WHEN DROPPING ROBOT ON FLOOR
            //acc_cml_error_elev.at(i) = 0.0; // reset
          }


          //Positive Error signal for controlling searching reflexes
          acc_cml_error.at(i) += abs(fmodel_cml_error.at(i));
          acc_cml_error_posi_neg.at(i) += fmodel_cml_error.at(i);

          //reset at swing phase
          if(m_pre.at(i+CL0_m/*9*/)>-0.7) //Reset error every Swing phase by detecting CR motor signal
            acc_cml_error.at(i) = 0;

          //Negative Error signal for controlling elevator reflexes
          if(acc_cml_error_old.at(i)<0)
          {
            acc_cml_error_elev.at(i) += abs(acc_cml_error_old.at(i));
            error_cml_elev.at(i) = 1.0;
          }
          if(acc_cml_error_old.at(i)>0)
          {
            acc_cml_error_elev.at(i) = 0.0;
            error_cml_elev.at(i) = 0.0;
          }


          //--------Stop learning process if error not appear for 500 time steps!
          double error_threshold_2 = 0.2;//0.25;
          if(abs(low_pass_fmodel_cml_error.at(i))<error_threshold_2)//abs(fmodel_cml_error.at(i)) == 0.0)
          {
            counter_cl.at(i)++;
          }
          if(abs(low_pass_fmodel_cml_error.at(i))>error_threshold_2)
            //if(abs(fmodel_cml_error.at(i)) > error_threshold)
          {
            counter_cl.at(i) = 0;
          }

          if(counter_cl.at(i) > 300)//500)//1000)//500) // 500 time steps !! need to be adaptive !! later
          {
            lr_fmodel_cl.at(i) = 0;
            low_pass_fmodel_cml_error.at(i) = 0;
          }


          //--------Stop learning process if error not appear for 500 time steps!


          //learning delta rule
          if(switchon_learnweights) // No learning
          {
            lr_fmodel_cl.at(i) = 0;

          }

          //learning delta rule with gradient descent

          //Recurrent weight, fmodel_cmr_outputfinal.at(i)
          fmodel_fmodel_cml_w.at(i) += lr_fmodel_cl.at(i)*(fmodel_cml_errorW.at(i)*(1-fmodel_cml_output.at(i)*fmodel_cml_output.at(i))*fmodel_cml_output_old.at(i));
          //          if(fmodel_fmodel_cml_w.at(i)<0.0)
          //            fmodel_fmodel_cml_w.at(i) = 0.0;

          //Input weight, fmodel_cmr_outputfinal.at(i)
          fmodel_cml_w.at(i) += lr_fmodel_cl.at(i)*(fmodel_cml_errorW.at(i)*(1-fmodel_cml_output.at(i)*fmodel_cml_output.at(i))*m_pre.at(i+CL0_m/*6*/));
          //          if(fmodel_cmr_w.at(i)<0.0)
          //            fmodel_cmr_w.at(i) = 0.0;

          //Bias
          fmodel_cml_bias.at(i) += lr_fmodel_cl.at(i)*(fmodel_cml_errorW.at(i)*(1-fmodel_cml_output.at(i)*fmodel_cml_output.at(i)));
          //          if(fmodel_cml_bias.at(i)<0.0)
          //            fmodel_cml_bias.at(i) = 0.0;

          std::cout<<"fmodel_fmodel_cml_w.at("<<i<<")="<<fmodel_fmodel_cml_w.at(i)<<";fmodel_cml_bias.at("<<i<<")="<<" "<<fmodel_cml_bias.at(i)<<" "<<";fmodel_cml_w.at("<<i<<")="<<" "<<fmodel_cml_w.at(i)<<";//counter"<<counter_cl.at(i)<<"cin="<<Control_input<< "\n";
        }

        break;



      case 5:

        for(unsigned int i=0; i<fmodel_cmr_activity.size();i++)
        {


          //learning phase start
          if(counter_refelx_R_fs<1000)
          {
            dervi_reflex_R_fs.at(i) = reflex_R_fs.at(i)-reflex_R_fs_old.at(i);
            fmodel_cmr_output_old.at(i) = fmodel_cmr_output.at(i);

            //Post processing
            if(dervi_reflex_R_fs.at(i)>1)
            {
              fmodel_cmr_output.at(i) = postcr.at(i);
            }


            if (reflex_R_fs.at(i) > 0.0)
            {
              countup_reflex_R_fs.at(i) = countup_reflex_R_fs.at(i)+1.0; //Delta x0 up
              countdown_reflex_R_fs.at(i) = 0.0;
            }

            // Count how many step of Stance
            else if (reflex_R_fs.at(i) < 0.0)
            {
              countdown_reflex_R_fs.at(i) = countdown_reflex_R_fs.at(i)+1.0; //Delta x0 up
              countup_reflex_R_fs.at(i) = 0.0;
            }


            if(countup_reflex_R_fs.at(i)>max_up)
              max_up=countup_reflex_R_fs.at(i);
            if(countdown_reflex_R_fs.at(i)>max_down)
              max_down=countdown_reflex_R_fs.at(i);

            //						if(fmodel_cmr_output.at(i)<max_fmodel)
            //							max_fmodel = fmodel_cmr_output.at(i);

            dervi_fmodel_cmr_output.at(i) = fmodel_cmr_output.at(i) - fmodel_cmr_output_old.at(i);

            counter_refelx_R_fs++;



          }


          //Producing transformed signal
          if(postcr.at(i)>-0.119 && countup_reflex_R_fs2.at(i) < max_up)
          {
            fmodel_cmr_outputfinal.at(i) = 1.0;
            countup_reflex_R_fs2.at(i)++;
          }
          else
          {
            fmodel_cmr_outputfinal.at(i) = -1.0;
            countup_reflex_R_fs2.at(i) = 0;
          }

          //Calculate error
          fmodel_cmr_error.at(i) = reflex_R_fs.at(i)-fmodel_cmr_outputfinal.at(i) /*regulate error*/; //target - output // only positive error
          acc_cmr_error_old.at(i) = fmodel_cmr_error.at(i);// for elevator reflex

          if(switchon_purefootsignal)//Only foot contact signal
          {
            fmodel_cmr_error.at(i) = reflex_R_fs.at(i)-(-1/*constant error*/); //target - output // only positive error
          }

          double error_threshold = 0.15;

          // Error threshold
          if(fmodel_cmr_error.at(i)<error_threshold)//0.05)//-1)
          {
            //fmodel_cmr_error.at(i) = 0.0;

            if(softlanding)// Later use sensor to activate!!!! e.g., acc sensor "g"
            {
              if(allfoot_off_ground> 50)
              {
                acc_cmr_error.at(i) = 0.0; // reset to immediately reset as soon as no error!!!!!! GOOD WHEN DROPPING ROBOT ON FLOOR
              }
              if(allfoot_off_ground> 100)
              {
                allfoot_off_ground = 0;
              }
            }

            //acc_cmr_error_elev.at(i) = 0.0; // reset
          }

          //Positive Error signal for controlling searching reflexes
          acc_cmr_error.at(i) += abs(fmodel_cmr_error.at(i));

          if(abs(fmodel_cmr_error.at(i)) < 0.05)
            acc_cmr_error_posi_neg.at(i) = 0.0;
          acc_cmr_error_posi_neg.at(i) += fmodel_cmr_error.at(i);

          //reset at swing phase
          if(m_pre.at(i+CR0_m/*6*/)>-0.7) //Reset error every Swing phase by detecting CR motor signal
            acc_cmr_error.at(i) = 0;

          //Negative Error signal for controlling elevator reflexes
          if(acc_cmr_error_old.at(i)<0)
          {
            acc_cmr_error_elev.at(i) += abs(acc_cmr_error_old.at(i));
            error_cmr_elev.at(i) = 1.0;
          }
          if(acc_cmr_error_old.at(i)>0)
          {
            acc_cmr_error_elev.at(i) = 0.0;
            error_cmr_elev.at(i) = 0.0;
          }


          //--------Stop learning process if error not appear for 500 time steps!
          //Count distance between two error peaks
          if(abs(fmodel_cmr_error.at(i)) == 0.0)
          {
            counter_cr.at(i)++;
          }
          if(abs(fmodel_cmr_error.at(i)) > error_threshold)
          {

            counter_cr.at(i) = 0;

          }
          /*1) TO DO NEED TO BE ADJUSTED THIS NUMBER "500"*/

          if(counter_cr.at(i) > 1000) // 500 time steps !! need to be adaptive !! later
          {
            lr_fmodel_cr.at(i) = 0;

            //bj_output.at(0) = 1.0;//-0.05;//tr_activity.at(2);

          }


          //--------Stop learning process if error not appear for 500 time steps!
          //learning rule
          if(switchon_learnweights) // No learning
          {
            lr_fmodel_cr.at(i) = 0;
          }

          //learning delta rule with gradient descent

          //Recurrent weight, fmodel_cmr_outputfinal.at(i)
          //fmodel_fmodel_cmr_w.at(i) += lr_fmodel_cr.at(i)*(-2*fmodel_cmr_error.at(i)*(1-fmodel_cmr_output.at(i)*fmodel_cmr_output.at(i))*fmodel_cmr_output_old.at(i));
          //fmodel_fmodel_cmr_w.at(i) += lr_fmodel_cr.at(i)*(2*fmodel_cmr_error.at(i)*(1- fmodel_cmr_outputfinal.at(i)* fmodel_cmr_outputfinal.at(i))*fmodel_cmr_output_old.at(i));
          fmodel_fmodel_cmr_w.at(i) += lr_fmodel_cr.at(i)*(2*fmodel_cmr_error.at(i)*(1- fmodel_cmr_outputfinal.at(i)* fmodel_cmr_outputfinal.at(i))*fmodel_post_cmr_w.at(i)*(1-fmodel_cmr_output.at(i)*fmodel_cmr_output.at(i))*fmodel_cmr_output_old.at(i));



          //Input weight, fmodel_cmr_outputfinal.at(i)
          //fmodel_cmr_w.at(i) += lr_fmodel_cr.at(i)*(2*fmodel_cmr_error.at(i)*(1-fmodel_cmr_output.at(i)*fmodel_cmr_output.at(i))*m_pre.at(i+CR0_m/*6*/));

          //Bias
          fmodel_cmr_bias.at(i) += lr_fmodel_cr.at(i)*(2*fmodel_cmr_error.at(i)*(1-fmodel_cmr_output.at(i)*fmodel_cmr_output.at(i)));


          std::cout<<counter_refelx_R_fs<<"fmodel_fmodel_cmr_w.at("<<i<<")="<<fmodel_fmodel_cmr_w.at(i)<<";//fmodel_cmr_bias="<<" "<<fmodel_cmr_bias.at(i)<<" "<<"fmodel_cmr_w.at="<<" "<<fmodel_cmr_w.at(i)<<";//counter"<<counter_cr.at(i)<<"cin="<<Control_input<< "\n";

        }

        //------------CL Loop---------//
        //-1,..,+1--> tanh
        for(unsigned int i=0; i<fmodel_cml_activity.size();i++)
        {

          fmodel_cml_output_old.at(i) = fmodel_cml_output.at(i);

          //Forward model of Coxa left motor signals //Recurrent single neuron
          fmodel_cml_activity.at(i) = m_pre.at(i+CL0_m/*9*/)*fmodel_cml_w.at(i)+fmodel_cml_output.at(i)*fmodel_fmodel_cml_w.at(i)+fmodel_cml_bias.at(i);
          fmodel_cml_output.at(i) = tanh(fmodel_cml_activity.at(i));

          //Post processing
          fmodel_cml_outputfinal.at(i) = tanh(fmodel_cml_output.at(i)*fmodel_post_cml_w.at(i));

          //Calculate error

          fmodel_cml_error.at(i) = reflex_L_fs.at(i)-fmodel_cml_outputfinal.at(i) /*regulate error*/; //target - output // only positive error
          acc_cml_error_old.at(i) = fmodel_cml_error.at(i);// for elevator reflex

          if(switchon_purefootsignal)//Only foot contact signal
          {
            fmodel_cml_error.at(i) = reflex_L_fs.at(i)-(-1/*constant error*/); //target - output // only positive error
          }


          double error_threshold = 0.15;
          // Error threshold
          if(fmodel_cml_error.at(i)<error_threshold)//0.05)//-1)
          {
            //fmodel_cml_error.at(i) = 0.0;

            if(softlanding)// Later use sensor to activate!!!! e.g., acc sensor "g"
            {
              if(allfoot_off_ground> 50)
              {
                acc_cml_error.at(i) = 0.0; // reset to immediately reset as soon as no error!!!!!! GOOD WHEN DROPPING ROBOT ON FLOOR
              }
              if(allfoot_off_ground> 100)
              {
                allfoot_off_ground = 0;
              }
            }
            //acc_cml_error.at(i) = 0.0;// reset to immediately reset as soon as no error!!!!!! GOOD WHEN DROPPING ROBOT ON FLOOR
            //acc_cml_error_elev.at(i) = 0.0; // reset
          }

          //Positive Error signal for controlling searching reflexes
          acc_cml_error.at(i) += abs(fmodel_cml_error.at(i));
          acc_cml_error_posi_neg.at(i) += fmodel_cml_error.at(i);

          //reset at swing phase
          if(m_pre.at(i+CL0_m/*9*/)>-0.7) //Reset error every Swing phase by detecting CR motor signal
            acc_cml_error.at(i) = 0;

          //Negative Error signal for controlling elevator reflexes
          if(acc_cml_error_old.at(i)<0)
          {
            acc_cml_error_elev.at(i) += abs(acc_cml_error_old.at(i));
            error_cml_elev.at(i) = 1.0;
          }
          if(acc_cml_error_old.at(i)>0)
          {
            acc_cml_error_elev.at(i) = 0.0;
            error_cml_elev.at(i) = 0.0;
          }


          //--------Stop learning process if error not appear for 500 time steps!
          if(abs(fmodel_cml_error.at(i)) == 0.0)
          {
            counter_cl.at(i)++;
          }

          if(abs(fmodel_cml_error.at(i)) > error_threshold)
          {
            counter_cl.at(i) = 0;
          }

          if(counter_cl.at(i) > 1000)//500) // 500 time steps !! need to be adaptive !! later
          {
            lr_fmodel_cl.at(i) = 0;
          }

          //--------Stop learning process if error not appear for 500 time steps!


          //learning delta rule
          if(switchon_learnweights) // No learning
          {
            lr_fmodel_cl.at(i) = 0;
          }

          //learning delta rule with gradient descent

          //Recurrent weight, fmodel_cmr_outputfinal.at(i)
          //fmodel_fmodel_cml_w.at(i) += lr_fmodel_cl.at(i)*(-2*fmodel_cml_error.at(i)*(1-fmodel_cml_output.at(i)*fmodel_cml_output.at(i))*fmodel_cml_output_old.at(i));
          fmodel_fmodel_cml_w.at(i) += lr_fmodel_cl.at(i)*(2*fmodel_cml_error.at(i)*(1- fmodel_cml_outputfinal.at(i)* fmodel_cml_outputfinal.at(i))*fmodel_post_cml_w.at(i)*(1-fmodel_cml_output.at(i)*fmodel_cml_output.at(i))*fmodel_cml_output_old.at(i));


          //Input weight, fmodel_cmr_outputfinal.at(i)
          //fmodel_cml_w.at(i) += lr_fmodel_cl.at(i)*(2*fmodel_cml_error.at(i)*(1-fmodel_cml_output.at(i)*fmodel_cml_output.at(i))*m_pre.at(i+CL0_m/*6*/));

          //Bias
          //fmodel_cml_bias.at(i) += lr_fmodel_cl.at(i)*(2*fmodel_cml_error.at(i)*(1-fmodel_cml_output.at(i)*fmodel_cml_output.at(i)));




          std::cout<<"fmodel_fmodel_cml_w.at("<<i<<")="<<fmodel_fmodel_cml_w.at(i)<<";//fmodel_cml_bias="<<" "<<fmodel_cml_bias.at(i)<<" "<<"fmodel_cml_w.at="<<" "<<fmodel_cml_w.at(i)<<";//counter"<<counter_cl.at(i)<<"cin="<<Control_input<< "\n";
        }

        break;

      case 6:
        //------------Add ESN training (3)----------------------------------//

        std::cout<<"size M"<<ESN_R0->endweights->getM()<<std::endl; // M = number of output neurons , e.g., M = 3
        std::cout<<"size N"<<ESN_R0->endweights->getN()<<std::endl; // N = number of hidden neurons , e.g., N = 30

        int learning_steps;

        if(sequentiral_learning)
          learning_steps = t_change_gait3;
        else
          learning_steps = t_change_gait1;

        double learning_rate;
        learning_rate = 0.99; //0.00033; //0.99;//RLS = 0.99


        // Saving output weights of, e.g., R0
        //ESN_R0->endweights->val(i /*output 0,...,2 */,j /*hidden, 0,..,29 */)

        outFilenlc1<<global_count<<' '<<
            Control_input<<' '<<
            reflex_R_fs.at(1)<<' '<<
            ESTrainOutput_R1[0]<<' '<<
            ESTrainOutput_R1[1]<<' '<<
            ESTrainOutput_R1[2]<<' '<<
            ESinput_R1[0]<<' '<<
            fmodel_cmr1_output_rc.at(2)<<' '<<
            fmodel_cmr1_output_rc.at(1)<<' '<<
            fmodel_cmr1_output_rc.at(0)<<' '<<
            fmodel_cmr_output_rc.at(1)<<' '<<endl;

        outFilenlc2<<global_count<<' '<<
            ESN_R1->endweights->val(0 /*hidden to output 0,..., 2 */,0 /*output, 0,..,29 */)<<' '<<
            ESN_R1->endweights->val(0 /*hidden to output 0,..., 2 */,1 /*output, 0,..,29 */)<<' '<<
            ESN_R1->endweights->val(0 /*hidden to output 0,..., 2 */,2 /*output, 0,..,29 */)<<' '<<
            ESN_R1->endweights->val(0 /*hidden to output 0,..., 2 */,3 /*output, 0,..,29 */)<<' '<<
            ESN_R1->endweights->val(0 /*hidden to output 0,..., 2 */,4 /*output, 0,..,29 */)<<' '<<
            ESN_R1->endweights->val(0 /*hidden to output 0,..., 2 */,5 /*output, 0,..,29 */)<<' '<<
            ESN_R1->endweights->val(0 /*hidden to output 0,..., 2 */,6 /*output, 0,..,29 */)<<' '<<
            ESN_R1->endweights->val(0 /*hidden to output 0,..., 2 */,7 /*output, 0,..,29 */)<<' '<<
            ESN_R1->endweights->val(0 /*hidden to output 0,..., 2 */,8 /*output, 0,..,29 */)<<' '<<
            ESN_R1->endweights->val(0 /*hidden to output 0,..., 2 */,9 /*output, 0,..,29 */)<<' '<<
            ESN_R1->endweights->val(0 /*hidden to output 0,..., 2 */,10 /*output, 0,..,29 */)<<' '<<
            ESN_R1->endweights->val(0 /*hidden to output 0,..., 2 */,11 /*output, 0,..,29 */)<<' '<<
            ESN_R1->endweights->val(0 /*hidden to output 0,..., 2 */,12 /*output, 0,..,29 */)<<' '<<
            ESN_R1->endweights->val(0 /*hidden to output 0,..., 2 */,13 /*output, 0,..,29 */)<<' '<<
            ESN_R1->endweights->val(0 /*hidden to output 0,..., 2 */,14 /*output, 0,..,29 */)<<' '<<
            ESN_R1->endweights->val(0 /*hidden to output 0,..., 2 */,15 /*output, 0,..,29 */)<<' '<<
            ESN_R1->endweights->val(0 /*hidden to output 0,..., 2 */,16 /*output, 0,..,29 */)<<' '<<
            ESN_R1->endweights->val(0 /*hidden to output 0,..., 2 */,17 /*output, 0,..,29 */)<<' '<<
            ESN_R1->endweights->val(0 /*hidden to output 0,..., 2 */,18 /*output, 0,..,29 */)<<' '<<
            ESN_R1->endweights->val(0 /*hidden to output 0,..., 2 */,19 /*output, 0,..,29 */)<<' '<<
            ESN_R1->endweights->val(0 /*hidden to output 0,..., 2 */,20 /*output, 0,..,29 */)<<' '<<
            ESN_R1->endweights->val(0 /*hidden to output 0,..., 2 */,21 /*output, 0,..,29 */)<<' '<<
            ESN_R1->endweights->val(0 /*hidden to output 0,..., 2 */,22 /*output, 0,..,29 */)<<' '<<
            ESN_R1->endweights->val(0 /*hidden to output 0,..., 2 */,23 /*output, 0,..,29 */)<<' '<<
            ESN_R1->endweights->val(0 /*hidden to output 0,..., 2 */,24 /*output, 0,..,29 */)<<' '<<
            ESN_R1->endweights->val(0 /*hidden to output 0,..., 2 */,25 /*output, 0,..,29 */)<<' '<<
            ESN_R1->endweights->val(0 /*hidden to output 0,..., 2 */,26 /*output, 0,..,29 */)<<' '<<
            ESN_R1->endweights->val(0 /*hidden to output 0,..., 2 */,27 /*output, 0,..,29 */)<<' '<<
            ESN_R1->endweights->val(0 /*hidden to output 0,..., 2 */,28 /*output, 0,..,29 */)<<' '<<
            ESN_R1->endweights->val(0 /*hidden to output 0,..., 2 */,29 /*output, 0,..,29 */)<<' '<<endl;

        outFilenlc3<<global_count<<' '<<
            ESN_R1->endweights->val(1 /*hidden to output 0,..., 2 */,0 /*output, 0,..,29 */)<<' '<<
            ESN_R1->endweights->val(1 /*hidden to output 0,..., 2 */,1 /*output, 0,..,29 */)<<' '<<
            ESN_R1->endweights->val(1 /*hidden to output 0,..., 2 */,2 /*output, 0,..,29 */)<<' '<<
            ESN_R1->endweights->val(1 /*hidden to output 0,..., 2 */,3 /*output, 0,..,29 */)<<' '<<
            ESN_R1->endweights->val(1 /*hidden to output 0,..., 2 */,4 /*output, 0,..,29 */)<<' '<<
            ESN_R1->endweights->val(1 /*hidden to output 0,..., 2 */,5 /*output, 0,..,29 */)<<' '<<
            ESN_R1->endweights->val(1 /*hidden to output 0,..., 2 */,6 /*output, 0,..,29 */)<<' '<<
            ESN_R1->endweights->val(1 /*hidden to output 0,..., 2 */,7 /*output, 0,..,29 */)<<' '<<
            ESN_R1->endweights->val(1 /*hidden to output 0,..., 2 */,8 /*output, 0,..,29 */)<<' '<<
            ESN_R1->endweights->val(1 /*hidden to output 0,..., 2 */,9 /*output, 0,..,29 */)<<' '<<
            ESN_R1->endweights->val(1 /*hidden to output 0,..., 2 */,10 /*output, 0,..,29 */)<<' '<<
            ESN_R1->endweights->val(1 /*hidden to output 0,..., 2 */,11 /*output, 0,..,29 */)<<' '<<
            ESN_R1->endweights->val(1 /*hidden to output 0,..., 2 */,12 /*output, 0,..,29 */)<<' '<<
            ESN_R1->endweights->val(1 /*hidden to output 0,..., 2 */,13 /*output, 0,..,29 */)<<' '<<
            ESN_R1->endweights->val(1 /*hidden to output 0,..., 2 */,14 /*output, 0,..,29 */)<<' '<<
            ESN_R1->endweights->val(1 /*hidden to output 0,..., 2 */,15 /*output, 0,..,29 */)<<' '<<
            ESN_R1->endweights->val(1 /*hidden to output 0,..., 2 */,16 /*output, 0,..,29 */)<<' '<<
            ESN_R1->endweights->val(1 /*hidden to output 0,..., 2 */,17 /*output, 0,..,29 */)<<' '<<
            ESN_R1->endweights->val(1 /*hidden to output 0,..., 2 */,18 /*output, 0,..,29 */)<<' '<<
            ESN_R1->endweights->val(1 /*hidden to output 0,..., 2 */,19 /*output, 0,..,29 */)<<' '<<
            ESN_R1->endweights->val(1 /*hidden to output 0,..., 2 */,20 /*output, 0,..,29 */)<<' '<<
            ESN_R1->endweights->val(1 /*hidden to output 0,..., 2 */,21 /*output, 0,..,29 */)<<' '<<
            ESN_R1->endweights->val(1 /*hidden to output 0,..., 2 */,22 /*output, 0,..,29 */)<<' '<<
            ESN_R1->endweights->val(1 /*hidden to output 0,..., 2 */,23 /*output, 0,..,29 */)<<' '<<
            ESN_R1->endweights->val(1 /*hidden to output 0,..., 2 */,24 /*output, 0,..,29 */)<<' '<<
            ESN_R1->endweights->val(1 /*hidden to output 0,..., 2 */,25 /*output, 0,..,29 */)<<' '<<
            ESN_R1->endweights->val(1 /*hidden to output 0,..., 2 */,26 /*output, 0,..,29 */)<<' '<<
            ESN_R1->endweights->val(1 /*hidden to output 0,..., 2 */,27 /*output, 0,..,29 */)<<' '<<
            ESN_R1->endweights->val(1 /*hidden to output 0,..., 2 */,28 /*output, 0,..,29 */)<<' '<<
            ESN_R1->endweights->val(1 /*hidden to output 0,..., 2 */,29 /*output, 0,..,29 */)<<' '<<endl;

        outFilenlc4<<global_count<<' '<<
            ESN_R1->endweights->val(2 /*hidden to output 0,..., 2 */,0 /*output, 0,..,29 */)<<' '<<
            ESN_R1->endweights->val(2 /*hidden to output 0,..., 2 */,1 /*output, 0,..,29 */)<<' '<<
            ESN_R1->endweights->val(2 /*hidden to output 0,..., 2 */,2 /*output, 0,..,29 */)<<' '<<
            ESN_R1->endweights->val(2 /*hidden to output 0,..., 2 */,3 /*output, 0,..,29 */)<<' '<<
            ESN_R1->endweights->val(2 /*hidden to output 0,..., 2 */,4 /*output, 0,..,29 */)<<' '<<
            ESN_R1->endweights->val(2 /*hidden to output 0,..., 2 */,5 /*output, 0,..,29 */)<<' '<<
            ESN_R1->endweights->val(2 /*hidden to output 0,..., 2 */,6 /*output, 0,..,29 */)<<' '<<
            ESN_R1->endweights->val(2 /*hidden to output 0,..., 2 */,7 /*output, 0,..,29 */)<<' '<<
            ESN_R1->endweights->val(2 /*hidden to output 0,..., 2 */,8 /*output, 0,..,29 */)<<' '<<
            ESN_R1->endweights->val(2 /*hidden to output 0,..., 2 */,9 /*output, 0,..,29 */)<<' '<<
            ESN_R1->endweights->val(2 /*hidden to output 0,..., 2 */,10 /*output, 0,..,29 */)<<' '<<
            ESN_R1->endweights->val(2 /*hidden to output 0,..., 2 */,11 /*output, 0,..,29 */)<<' '<<
            ESN_R1->endweights->val(2 /*hidden to output 0,..., 2 */,12 /*output, 0,..,29 */)<<' '<<
            ESN_R1->endweights->val(2 /*hidden to output 0,..., 2 */,13 /*output, 0,..,29 */)<<' '<<
            ESN_R1->endweights->val(2 /*hidden to output 0,..., 2 */,14 /*output, 0,..,29 */)<<' '<<
            ESN_R1->endweights->val(2 /*hidden to output 0,..., 2 */,15 /*output, 0,..,29 */)<<' '<<
            ESN_R1->endweights->val(2 /*hidden to output 0,..., 2 */,16 /*output, 0,..,29 */)<<' '<<
            ESN_R1->endweights->val(2 /*hidden to output 0,..., 2 */,17 /*output, 0,..,29 */)<<' '<<
            ESN_R1->endweights->val(2 /*hidden to output 0,..., 2 */,18 /*output, 0,..,29 */)<<' '<<
            ESN_R1->endweights->val(2 /*hidden to output 0,..., 2 */,19 /*output, 0,..,29 */)<<' '<<
            ESN_R1->endweights->val(2 /*hidden to output 0,..., 2 */,20 /*output, 0,..,29 */)<<' '<<
            ESN_R1->endweights->val(2 /*hidden to output 0,..., 2 */,21 /*output, 0,..,29 */)<<' '<<
            ESN_R1->endweights->val(2 /*hidden to output 0,..., 2 */,22 /*output, 0,..,29 */)<<' '<<
            ESN_R1->endweights->val(2 /*hidden to output 0,..., 2 */,23 /*output, 0,..,29 */)<<' '<<
            ESN_R1->endweights->val(2 /*hidden to output 0,..., 2 */,24 /*output, 0,..,29 */)<<' '<<
            ESN_R1->endweights->val(2 /*hidden to output 0,..., 2 */,25 /*output, 0,..,29 */)<<' '<<
            ESN_R1->endweights->val(2 /*hidden to output 0,..., 2 */,26 /*output, 0,..,29 */)<<' '<<
            ESN_R1->endweights->val(2 /*hidden to output 0,..., 2 */,27 /*output, 0,..,29 */)<<' '<<
            ESN_R1->endweights->val(2 /*hidden to output 0,..., 2 */,28 /*output, 0,..,29 */)<<' '<<
            ESN_R1->endweights->val(2 /*hidden to output 0,..., 2 */,29 /*output, 0,..,29 */)<<' '<<endl;


        //Using learned weights from files
        if (learn == false && loadweight == true)
        {

          if (singlegait){
            // Reading the stored endweights from the file
            ESN_R0->readEndweightsFromFile(11);
            ESN_R1->readEndweightsFromFile(12);
            ESN_R2->readEndweightsFromFile(13);

            ESN_L0->readEndweightsFromFile(21);
            ESN_L1->readEndweightsFromFile(22);
            ESN_L2->readEndweightsFromFile(23);

            //reading the stored input weights from the file
            ESN_R0->readStartweightsFromFile(11);
            ESN_R1->readStartweightsFromFile(12);
            ESN_R2->readStartweightsFromFile(13);

            ESN_L0->readStartweightsFromFile(21);
            ESN_L1->readStartweightsFromFile(22);
            ESN_L2->readStartweightsFromFile(23);

            //reading the stored inner RC weights from the file
            ESN_R0->readInnerweightsFromFile(11);
            ESN_R1->readInnerweightsFromFile(12);
            ESN_R2->readInnerweightsFromFile(13);

            ESN_L0->readInnerweightsFromFile(21);
            ESN_L1->readInnerweightsFromFile(22);
            ESN_L2->readInnerweightsFromFile(23);

            //noise
            ESN_R0->readNoiseFromFile(11);
            ESN_R1->readNoiseFromFile(12);
            ESN_R2->readNoiseFromFile(13);

            ESN_L0->readNoiseFromFile(21);
            ESN_L1->readNoiseFromFile(22);
            ESN_L2->readNoiseFromFile(23);

          }

          else {
            if (Control_input == gait1){

              ESN_R0->readEndweightsFromFile(111);
              ESN_R1->readEndweightsFromFile(112);
              ESN_R2->readEndweightsFromFile(113);

              ESN_L0->readEndweightsFromFile(121);
              ESN_L1->readEndweightsFromFile(122);
              ESN_L2->readEndweightsFromFile(123);

              //reading the stored input weights from the file
              ESN_R0->readStartweightsFromFile(111);
              ESN_R1->readStartweightsFromFile(112);
              ESN_R2->readStartweightsFromFile(113);

              ESN_L0->readStartweightsFromFile(121);
              ESN_L1->readStartweightsFromFile(122);
              ESN_L2->readStartweightsFromFile(123);

              //reading the stored inner RC weights from the file
              ESN_R0->readInnerweightsFromFile(111);
              ESN_R1->readInnerweightsFromFile(112);
              ESN_R2->readInnerweightsFromFile(113);

              ESN_L0->readInnerweightsFromFile(121);
              ESN_L1->readInnerweightsFromFile(122);
              ESN_L2->readInnerweightsFromFile(123);

              //noise
              ESN_R0->readNoiseFromFile(111);
              ESN_R1->readNoiseFromFile(112);
              ESN_R2->readNoiseFromFile(113);

              ESN_L0->readNoiseFromFile(121);
              ESN_L1->readNoiseFromFile(122);
              ESN_L2->readNoiseFromFile(123);
            }

            if (Control_input == gait2){

              ESN_R0->readEndweightsFromFile(211);
              ESN_R1->readEndweightsFromFile(212);
              ESN_R2->readEndweightsFromFile(213);

              ESN_L0->readEndweightsFromFile(221);
              ESN_L1->readEndweightsFromFile(222);
              ESN_L2->readEndweightsFromFile(223);

              //reading the stored input weights from the file
              ESN_R0->readStartweightsFromFile(211);
              ESN_R1->readStartweightsFromFile(212);
              ESN_R2->readStartweightsFromFile(213);

              ESN_L0->readStartweightsFromFile(221);
              ESN_L1->readStartweightsFromFile(222);
              ESN_L2->readStartweightsFromFile(223);

              //reading the stored inner RC weights from the file
              ESN_R0->readInnerweightsFromFile(211);
              ESN_R1->readInnerweightsFromFile(212);
              ESN_R2->readInnerweightsFromFile(213);

              ESN_L0->readInnerweightsFromFile(221);
              ESN_L1->readInnerweightsFromFile(222);
              ESN_L2->readInnerweightsFromFile(223);

              //noise
              ESN_R0->readNoiseFromFile(211);
              ESN_R1->readNoiseFromFile(212);
              ESN_R2->readNoiseFromFile(213);

              ESN_L0->readNoiseFromFile(221);
              ESN_L1->readNoiseFromFile(222);
              ESN_L2->readNoiseFromFile(223);
            }

            if (Control_input == gait3){

              ESN_R0->readEndweightsFromFile(311);
              ESN_R1->readEndweightsFromFile(312);
              ESN_R2->readEndweightsFromFile(313);

              ESN_L0->readEndweightsFromFile(321);
              ESN_L1->readEndweightsFromFile(322);
              ESN_L2->readEndweightsFromFile(323);

              //reading the stored input weights from the file
              ESN_R0->readStartweightsFromFile(311);
              ESN_R1->readStartweightsFromFile(312);
              ESN_R2->readStartweightsFromFile(313);

              ESN_L0->readStartweightsFromFile(321);
              ESN_L1->readStartweightsFromFile(322);
              ESN_L2->readStartweightsFromFile(323);

              //reading the stored inner RC weights from the file
              ESN_R0->readInnerweightsFromFile(311);
              ESN_R1->readInnerweightsFromFile(312);
              ESN_R2->readInnerweightsFromFile(313);

              ESN_L0->readInnerweightsFromFile(321);
              ESN_L1->readInnerweightsFromFile(322);
              ESN_L2->readInnerweightsFromFile(323);

              //noise
              ESN_R0->readNoiseFromFile(311);
              ESN_R1->readNoiseFromFile(312);
              ESN_R2->readNoiseFromFile(313);

              ESN_L0->readNoiseFromFile(321);
              ESN_L1->readNoiseFromFile(322);
              ESN_L2->readNoiseFromFile(323);

            }
          }

          if(!crossing_gap)
          {
            if(global_count>1000)
            {
              switchon_reflexes = true;
              elevator_reflexes = true;
            }
          }

          //To start capture output activation for LTM learning
          if(global_count>1000 && postcr.at(0)>-0.8 && postcr.at(0)>postcrold.at(0))
            ltm_start = true;

        }

        //  ESN_R0->printMatrix(ESN_R0->endweights); std::cout<<"\n********************************\n\n";
        //  ESN_R0->printMatrix(ESN_R0->startweights); std::cout<<"\n********************************\n\n";
        //  ESN_R0->printMatrix(ESN_R0->innerweights); std::cout<<"\n********************************\n\n";


        //Storing learned weights to files
        if(global_count>learning_steps /*3000*/ && loadweight == false)
        {
          learn = false;
//          switchon_reflexes = true;
//          elevator_reflexes = true;

          if (singlegait)
          {
            // Write the output weights to file R0 -> 11, R1->12, R3->13, L0 ->21, ...
            ESN_R0->writeEndweightsToFile(11);
            ESN_R1->writeEndweightsToFile(12);
            ESN_R2->writeEndweightsToFile(13);

            ESN_L0->writeEndweightsToFile(21);
            ESN_L1->writeEndweightsToFile(22);
            ESN_L2->writeEndweightsToFile(23);

            // Write the input weights to file R0 -> 11, R1->12, R3->13, L0 ->21, ...
            ESN_R0->writeStartweightsToFile(11);
            ESN_R1->writeStartweightsToFile(12);
            ESN_R2->writeStartweightsToFile(13);

            ESN_L0->writeStartweightsToFile(21);
            ESN_L1->writeStartweightsToFile(22);
            ESN_L2->writeStartweightsToFile(23);

            // Write the inner RC weights to file R0 -> 11, R1->12, R3->13, L0 ->21, ...
            ESN_R0->writeInnerweightsToFile(11);
            ESN_R1->writeInnerweightsToFile(12);
            ESN_R2->writeInnerweightsToFile(13);

            ESN_L0->writeInnerweightsToFile(21);
            ESN_L1->writeInnerweightsToFile(22);
            ESN_L2->writeInnerweightsToFile(23);

            //noise
            ESN_R0->writeNoiseToFile(11);
            ESN_R1->writeNoiseToFile(12);
            ESN_R2->writeNoiseToFile(13);

            ESN_L0->writeNoiseToFile(21);
            ESN_L1->writeNoiseToFile(22);
            ESN_L2->writeNoiseToFile(23);
          }
          //To start capture output activation for LTM learning
          if(postcr.at(0)>-0.8 && postcr.at(0)>postcrold.at(0))
            ltm_start = true;

        }
        std::cout<<"learning_steps"<< ":"<<learning_steps<<std::endl;

        //        outFilenlc6<<m_pre.at(CR0_m/*6*/)<<' '<<
        //            reflex_R_fs.at(0)<<' '<<
        //            m_pre.at(CR1_m/*6*/)<<' '<<
        //            reflex_R_fs.at(1)<<' '<<
        //            m_pre.at(CR2_m/*6*/)<<' '<<
        //            reflex_R_fs.at(2)<<' '<<
        //            m_pre.at(CL0_m/*6*/)<<' '<<
        //            reflex_L_fs.at(0)<<' '<<
        //            m_pre.at(CL1_m/*6*/)<<' '<<
        //            reflex_L_fs.at(1)<<' '<<
        //            m_pre.at(CL2_m/*6*/)<<' '<<
        //            reflex_L_fs.at(2)
        //           <<' '<<endl;

        std::cout<<" N weight------"<<std::endl;
        ESN_R0->printMatrix(ESN_R0->outputs);

        if(global_count == t_change_gait1 && loadweight == false)
        {
          // control_input 0.04 corresponds to 1
          ESN_R0->writeEndweightsToFile(111);
          ESN_R1->writeEndweightsToFile(112);
          ESN_R2->writeEndweightsToFile(113);

          ESN_L0->writeEndweightsToFile(121);
          ESN_L1->writeEndweightsToFile(122);
          ESN_L2->writeEndweightsToFile(123);

          ESN_R0->writeStartweightsToFile(111);
          ESN_R1->writeStartweightsToFile(112);
          ESN_R2->writeStartweightsToFile(113);

          ESN_L0->writeStartweightsToFile(121);
          ESN_L1->writeStartweightsToFile(122);
          ESN_L2->writeStartweightsToFile(123);

          // Write the inner RC weights to file R0 -> 11, R1->12, R3->13, L0 ->21, ...
          ESN_R0->writeInnerweightsToFile(111);
          ESN_R1->writeInnerweightsToFile(112);
          ESN_R2->writeInnerweightsToFile(113);

          ESN_L0->writeInnerweightsToFile(121);
          ESN_L1->writeInnerweightsToFile(122);
          ESN_L2->writeInnerweightsToFile(123);

          //noise
          ESN_R0->writeNoiseToFile(111);
          ESN_R1->writeNoiseToFile(112);
          ESN_R2->writeNoiseToFile(113);

          ESN_L0->writeNoiseToFile(121);
          ESN_L1->writeNoiseToFile(122);
          ESN_L2->writeNoiseToFile(123);

        }

        if(global_count == t_change_gait2 && loadweight == false){

          // control_input 0.06 corresponds to 2
          ESN_R0->writeEndweightsToFile(211);
          ESN_R1->writeEndweightsToFile(212);
          ESN_R2->writeEndweightsToFile(213);

          ESN_L0->writeEndweightsToFile(221);
          ESN_L1->writeEndweightsToFile(222);
          ESN_L2->writeEndweightsToFile(223);

          ESN_R0->writeStartweightsToFile(211);
          ESN_R1->writeStartweightsToFile(212);
          ESN_R2->writeStartweightsToFile(213);

          ESN_L0->writeStartweightsToFile(221);
          ESN_L1->writeStartweightsToFile(222);
          ESN_L2->writeStartweightsToFile(223);

          // Write the inner RC weights to file R0 -> 11, R1->12, R3->13, L0 ->21, ...
          ESN_R0->writeInnerweightsToFile(211);
          ESN_R1->writeInnerweightsToFile(212);
          ESN_R2->writeInnerweightsToFile(213);

          ESN_L0->writeInnerweightsToFile(221);
          ESN_L1->writeInnerweightsToFile(222);
          ESN_L2->writeInnerweightsToFile(223);

          //noise
          ESN_R0->writeNoiseToFile(211);
          ESN_R1->writeNoiseToFile(212);
          ESN_R2->writeNoiseToFile(213);

          ESN_L0->writeNoiseToFile(221);
          ESN_L1->writeNoiseToFile(222);
          ESN_L2->writeNoiseToFile(223);

        }

        if(global_count == t_change_gait3 && loadweight == false){

          // control_input 0.09 corresponds to 3
          ESN_R0->writeEndweightsToFile(311);
          ESN_R1->writeEndweightsToFile(312);
          ESN_R2->writeEndweightsToFile(313);

          ESN_L0->writeEndweightsToFile(321);
          ESN_L1->writeEndweightsToFile(322);
          ESN_L2->writeEndweightsToFile(323);

          ESN_R0->writeStartweightsToFile(311);
          ESN_R1->writeStartweightsToFile(312);
          ESN_R2->writeStartweightsToFile(313);

          ESN_L0->writeStartweightsToFile(321);
          ESN_L1->writeStartweightsToFile(322);
          ESN_L2->writeStartweightsToFile(323);

          // Write the inner RC weights to file R0 -> 11, R1->12, R3->13, L0 ->21, ...
          ESN_R0->writeInnerweightsToFile(311);
          ESN_R1->writeInnerweightsToFile(312);
          ESN_R2->writeInnerweightsToFile(313);

          ESN_L0->writeInnerweightsToFile(321);
          ESN_L1->writeInnerweightsToFile(322);
          ESN_L2->writeInnerweightsToFile(323);

          //noise
          ESN_R0->writeNoiseToFile(311);
          ESN_R1->writeNoiseToFile(312);
          ESN_R2->writeNoiseToFile(313);

          ESN_L0->writeNoiseToFile(321);
          ESN_L1->writeNoiseToFile(322);
          ESN_L2->writeNoiseToFile(323);

          //To load weights from now after 9000 steps
          loadweight = true;
          learn = false;
        }


        //-----Module ESN 1
        if(Control_input == gait1)
        {
          ESTrainOutput_R0[0]= reflex_R_fs.at(0); //Training output (target function)
          ESTrainOutput_R0[1]= 0.0; //Training output (target function)
          ESTrainOutput_R0[2]= 0.0; //Training output (target function)


        }
        if(Control_input == gait2)
        {
          ESTrainOutput_R0[0]= 0.0; //Training output (target function)
          ESTrainOutput_R0[1]= reflex_R_fs.at(0); //Training output (target function)
          ESTrainOutput_R0[2]= 0.0; //Training output (target function)


        }
        if(Control_input == gait3)
        {
          ESTrainOutput_R0[0]= 0.0; //Training output (target function)
          ESTrainOutput_R0[1]= 0.0; //Training output (target function)
          ESTrainOutput_R0[2]= reflex_R_fs.at(0); //Training output (target function)

        }


        ESinput_R0[0] = m_pre.at(CR0_m/*6*/);// Input
        ESN_R0->setInput(ESinput_R0, 1/* no. input*/);
        ESN_R0->takeStep(ESTrainOutput_R0, learning_rate/*0.99 *RLS/ /*0.00055/*0.0005*/ /*0.0055*//*1.5*//*1.8*/, 1 /*no td = 1 else td_error*/, learn/* true= learn, false = not learning learn_critic*/, 0);

        //temp = ESN->outputs->val(0, 0);
        //fmodel_cmr_output_rc.at(0) = ESN_R0->outputs->val(0, 0); // first output
        fmodel_cmr0_output_rc.at(0) = ESN_R0->outputs->val(0, 0); // first output
        fmodel_cmr0_output_rc.at(1) = ESN_R0->outputs->val(1, 0); // second output
        fmodel_cmr0_output_rc.at(2) = ESN_R0->outputs->val(2, 0); // third output

        ESN_R0->printMatrix(ESN_R0->endweights);

        fmodel_cmr_output_rc.at(0) = fmodel_cmr0_output_rc.at(0)+fmodel_cmr0_output_rc.at(1)+fmodel_cmr0_output_rc.at(2);

        //         fmodel_cmr1_output_rc.resize(3);
        //         fmodel_cmr2_output_rc.resize(3);
        //         fmodel_cml0_output_rc.resize(3);
        //         fmodel_cml1_output_rc.resize(3);
        //         fmodel_cml2_output_rc.resize(3);

        //-----Transfer to LTM 1-------------//
        static int test = 0;

        //if(global_count>500 && countup.at(0)==6)//100)
        //          {
        //            ltm_start = true;
        //            test = 1;
        //            total_c = deltaxup.at(0)+deltaxdown.at(0); //34,35
        //          }

        //total_c = deltaxup.at(0)+deltaxdown.at(0); //34,35
        // std::cout<<"***************Total "<<total_c<<"::"<<deltaxup.at(0)+deltaxdown.at(0)<<std::endl;

        //        if(total_c != deltaxup.at(0)+deltaxdown.at(0))
        //        {
        //          ltm_start = false;
        //          count_neuron = 0;
        //        }

        //Learning pattern
        if(ltm_v1)
        {
          if(ltm_start == true)
          {
            std::cout<<"----------LTM start v2----------------"<<std::endl;
            if(count_neuron<NUM_LTM_R0)//total_c)//NUM_LTM_R0)
            {
              std::cout<<"----------count_neuron loop1----------------"<<count_neuron<<std::endl;

              //Convert to [0,...,1] & Capture signal

              //Input
              input_ltm.at(count_neuron) = (fmodel_cmr_output_rc.at(0)+1.0);
              count_neuron++;

            }
            else
            {

              std::cout<<"----------count_neuron loop2----------------"<<count_neuron<<std::endl;

              if(count_neuron_learning<NUM_LTM_R0)
              {
                //Output
                cmr0_ltm_neuron.at(count_neuron_learning) = cmr0_ltm_neuron_w.at(count_neuron_learning)*input_ltm.at(count_neuron_learning);

                //Learning // w = mu*(input*output - output*w)
                cmr0_ltm_neuron_w.at(count_neuron_learning) += 0.98*(cmr0_ltm_neuron.at(count_neuron_learning)*input_ltm.at(count_neuron_learning)-
                    cmr0_ltm_neuron.at(count_neuron_learning)*cmr0_ltm_neuron_w.at(count_neuron_learning));


                count_neuron_learning++;
              }
              else
                count_neuron_learning = 0;

            }
          }


          outFilenlc6<<fmodel_cmr_output_rc.at(0)+1.0<<' '<<
              input_ltm.at(0)<<' '<<
              input_ltm.at(1)<<' '<<
              input_ltm.at(2)<<' '<<
              input_ltm.at(3)<<' '<<
              input_ltm.at(4)<<' '<<
              input_ltm.at(5)<<' '<<
              input_ltm.at(6)<<' '<<
              input_ltm.at(7)<<' '<<
              input_ltm.at(8)<<' '<<
              input_ltm.at(9)<<' '<<
              input_ltm.at(10)<<' '<<
              input_ltm.at(11)<<' '<<
              input_ltm.at(12)<<' '<<
              input_ltm.at(13)<<' '<<
              input_ltm.at(14)<<' '<<
              input_ltm.at(15)<<' '<<
              input_ltm.at(16)<<' '<<
              input_ltm.at(17)<<' '<<
              input_ltm.at(18)<<' '<<
              input_ltm.at(19)<<' '<<
              input_ltm.at(20)<<' '<<
              input_ltm.at(21)<<' '<<
              input_ltm.at(22)<<' '<<
              input_ltm.at(23)<<' '<<
              input_ltm.at(24)<<' '<<
              input_ltm.at(25)<<' '<<
              input_ltm.at(26)<<' '<<
              input_ltm.at(27)<<' '<<
              input_ltm.at(28)<<' '<<
              input_ltm.at(29)<<' '<<
              input_ltm.at(30)<<' '<<
              input_ltm.at(31)<<' '<<
              input_ltm.at(32)<<' '<<
              input_ltm.at(33)<<' '<<endl;

          outFilenlc5<<
              cmr0_ltm_neuron_w.at(0)<<' '<<
              cmr0_ltm_neuron_w.at(1)<<' '<<
              cmr0_ltm_neuron_w.at(2)<<' '<<
              cmr0_ltm_neuron_w.at(3)<<' '<<
              cmr0_ltm_neuron_w.at(4)<<' '<<
              cmr0_ltm_neuron_w.at(5)<<' '<<
              cmr0_ltm_neuron_w.at(6)<<' '<<
              cmr0_ltm_neuron_w.at(7)<<' '<<
              cmr0_ltm_neuron_w.at(8)<<' '<<
              cmr0_ltm_neuron_w.at(9)<<' '<<
              cmr0_ltm_neuron_w.at(10)<<' '<<
              cmr0_ltm_neuron_w.at(11)<<' '<<
              cmr0_ltm_neuron_w.at(12)<<' '<<
              cmr0_ltm_neuron_w.at(13)<<' '<<
              cmr0_ltm_neuron_w.at(14)<<' '<<
              cmr0_ltm_neuron_w.at(15)<<' '<<
              cmr0_ltm_neuron_w.at(16)<<' '<<
              cmr0_ltm_neuron_w.at(17)<<' '<<
              cmr0_ltm_neuron_w.at(18)<<' '<<
              cmr0_ltm_neuron_w.at(19)<<' '<<
              cmr0_ltm_neuron_w.at(20)<<' '<<
              cmr0_ltm_neuron_w.at(21)<<' '<<
              cmr0_ltm_neuron_w.at(22)<<' '<<
              cmr0_ltm_neuron_w.at(23)<<' '<<
              cmr0_ltm_neuron_w.at(24)<<' '<<
              cmr0_ltm_neuron_w.at(25)<<' '<<
              cmr0_ltm_neuron_w.at(26)<<' '<<
              cmr0_ltm_neuron_w.at(27)<<' '<<
              cmr0_ltm_neuron_w.at(28)<<' '<<
              cmr0_ltm_neuron_w.at(29)<<' '<<
              cmr0_ltm_neuron_w.at(30)<<' '<<
              cmr0_ltm_neuron_w.at(31)<<' '<<
              cmr0_ltm_neuron_w.at(32)<<' '<<
              cmr0_ltm_neuron_w.at(33)<<' '<<endl;
        }

        //Learning frequency
        if(ltm_v2)
        {
          if(ltm_start == true)
          {

            std::cout<<"----------LTM start----------------"<<std::endl;

            //Convert to [0,...,1]
            //Input

            //Spiking input
            //            if (fmodel_cmr_output_rc.at(0)>-0.5)
            //              fmodel_cmr_output_ltm.at(0) = 0.5;//1.0 = high weight, 0.5 = low weight
            //            else
            //              fmodel_cmr_output_ltm.at(0) = 0.0;

            //Cont. input
            fmodel_cmr_output_ltm.at(0) = fmodel_cmr_output_rc.at(0)+1.0;

            //Output
            cmr0_ltm_neuron.at(0) = cmr0_ltm_neuron_w.at(0)*fmodel_cmr_output_ltm.at(0);

            //Learning // w = mu*(input*output - output*w*w) //0.98 = spiking input
            cmr0_ltm_neuron_w.at(0) += 0.05*(cmr0_ltm_neuron.at(0)*fmodel_cmr_output_ltm.at(0)-
                cmr0_ltm_neuron.at(0)*cmr0_ltm_neuron_w.at(0)*cmr0_ltm_neuron_w.at(0));

            outFilenlc6<<fmodel_cmr_output_ltm.at(0)<<' '<<
                cmr0_ltm_neuron.at(0)<<' '<<
                cmr0_ltm_neuron_w.at(0)<<' '<<endl;
          }
        }

        //Learning weights of RC
        if(ltm_v3)
        {
          for(int i = 0; i < ESN_R0->endweights->getM(); i++)
          {
            for(int j = 0; j < ESN_R0->endweights->getN(); j++)
            {

              //              std::cout<<"size M"<<ESN_R0->endweights->getM()<<std::endl;
              //              std::cout<<"size N"<<ESN_R0->endweights->getN()<<std::endl;

              //Output weights
              inputs_ltm_out->val(i,j) = ESN_R0->endweights->val(i,j)/10000;
              outputs_ltm_out->val(i,j) = weights_ltm_out->val(i,j)*inputs_ltm_out->val(i,j);

              //Learning // w = mu*(input*output - output*w*w)
              weights_ltm_out->val(i,j) += 0.1*(inputs_ltm_out->val(i,j)*outputs_ltm_out->val(i,j))-
                  0.1*(outputs_ltm_out->val(i,j)*weights_ltm_out->val(i,j)*weights_ltm_out->val(i,j));

              std::cout<<" out "<<weights_ltm_out->val(i,j)<<":";
              std::cout<<ESN_R0->endweights->val(i,j)<<" ";
            }
            std::cout<<std::endl;
          }

          for(int i = 0; i < ESN_R0->innerweights->getM(); i++)
          {
            for(int j = 0; j < ESN_R0->innerweights->getN(); j++)
            {

              //Hidden weights
              inputs_ltm_hid->val(i,j) = ESN_R0->innerweights->val(i,j);
              outputs_ltm_hid->val(i,j) = weights_ltm_hid->val(i,j)*inputs_ltm_hid->val(i,j);

              //Learning // w = mu*(input*output - output*w*w)
              weights_ltm_hid->val(i,j) += 0.1*(inputs_ltm_hid->val(i,j)*outputs_ltm_hid->val(i,j))-
                  0.1*(outputs_ltm_hid->val(i,j)*weights_ltm_hid->val(i,j)*weights_ltm_hid->val(i,j));

              std::cout<<" hid "<<weights_ltm_hid->val(i,j)<<":";
              std::cout<<ESN_R0->innerweights->val(i,j)<<" ";
            }
            std::cout<<std::endl;
          }

          for(int i = 0; i < ESN_R0->startweights->getM(); i++)
          {
            for(int j = 0; j < ESN_R0->startweights->getN(); j++)
            {

              //Input weights
              inputs_ltm_in->val(i,j) = ESN_R0->startweights->val(i,j);
              outputs_ltm_in->val(i,j) = weights_ltm_in->val(i,j)*inputs_ltm_in->val(i,j);

              //Learning // w = mu*(input*output - output*w*w)
              weights_ltm_in->val(i,j) += 0.1*(inputs_ltm_in->val(i,j)*outputs_ltm_in->val(i,j))-
                  0.1*(outputs_ltm_in->val(i,j)*weights_ltm_in->val(i,j)*weights_ltm_in->val(i,j));

              std::cout<<" in "<<weights_ltm_in->val(i,j)<<":";
              std::cout<<ESN_R0->startweights->val(i,j)<<" ";
            }
            std::cout<<std::endl;
          }


          for(int i = 0; i < ESN_R0->noise->getM(); i++)
          {
            for(int j = 0; j < ESN_R0->noise->getN(); j++)
            {

              //Bias weights
              inputs_ltm_bi->val(i,j) = ESN_R0->noise->val(i,j);
              outputs_ltm_bi->val(i,j) = weights_ltm_bi->val(i,j)*inputs_ltm_bi->val(i,j);

              //Learning // w = mu*(input*output - output*w*w)
              weights_ltm_bi->val(i,j) += 0.9*(inputs_ltm_bi->val(i,j)*outputs_ltm_bi->val(i,j))-
                  0.9*(outputs_ltm_bi->val(i,j)*weights_ltm_bi->val(i,j)*weights_ltm_bi->val(i,j));

              std::cout<<" noise "<<weights_ltm_bi->val(i,j)<<":";
              std::cout<<ESN_R0->noise->val(i,j)<<" ";
            }
            std::cout<<std::endl;
          }

        }


        //-----Module ESN 2
        if(Control_input == gait1)
        {
          ESTrainOutput_R1[0]= reflex_R_fs.at(1); //Training output (target function)
          ESTrainOutput_R1[1]= 00; //Training output (target function)
          ESTrainOutput_R1[2]= 0.0; //Training output (target function)
        }
        if(Control_input == gait2)
        {
          ESTrainOutput_R1[0]= 0.0; //Training output (target function)
          ESTrainOutput_R1[1]= reflex_R_fs.at(1); //Training output (target function)
          ESTrainOutput_R1[2]= 0.0; //Training output (target function)
        }
        if(Control_input == gait3)
        {
          ESTrainOutput_R1[0]= 0.0; //Training output (target function)
          ESTrainOutput_R1[1]= 0.0; //Training output (target function)
          ESTrainOutput_R1[2]= reflex_R_fs.at(1); //Training output (target function)
        }


        //ESTrainOutput_R1[0]= reflex_R_fs.at(1); //Training output (target function)
        ESinput_R1[0] = m_pre.at(CR1_m/*6*/);// Input
        ESN_R1->setInput(ESinput_R1, 1/* no. input*/);
        ESN_R1->takeStep(ESTrainOutput_R1, learning_rate/*0.99 *RLS/ /*0.00055/*0.0005*/ /*0.0055*//*1.5*//*1.8*/, 1 /*no td = 1 else td_error*/, learn/* true= learn, false = not learning learn_critic*/, 0);
        //fmodel_cmr_output_rc.at(1) = ESN_R1->outputs->val(0, 0);
        fmodel_cmr1_output_rc.at(0) = ESN_R1->outputs->val(0, 0); // first output
        fmodel_cmr1_output_rc.at(1) = ESN_R1->outputs->val(1, 0); // second output
        fmodel_cmr1_output_rc.at(2) = ESN_R1->outputs->val(2, 0); // third output

        fmodel_cmr_output_rc.at(1) = fmodel_cmr1_output_rc.at(0)+fmodel_cmr1_output_rc.at(1)+fmodel_cmr1_output_rc.at(2);


        //-----Module ESN 3

        if(Control_input == gait1)
        {
          ESTrainOutput_R2[0]= reflex_R_fs.at(2); //Training output (target function)
          ESTrainOutput_R2[1]= 0.0; //Training output (target function)
          ESTrainOutput_R2[2]= 0.0; //Training output (target function)
        }
        if(Control_input == gait2)
        {
          ESTrainOutput_R2[0]= 0.0; //Training output (target function)
          ESTrainOutput_R2[1]= reflex_R_fs.at(2); //Training output (target function)
          ESTrainOutput_R2[2]= 0.0; //Training output (target function)
        }
        if(Control_input == gait3)
        {
          ESTrainOutput_R2[0]= 0.0; //Training output (target function)
          ESTrainOutput_R2[1]= 0.0; //Training output (target function)
          ESTrainOutput_R2[2]= reflex_R_fs.at(2); //Training output (target function)
        }


        //ESTrainOutput_R2[0]= reflex_R_fs.at(2); //Training output (target function)
        ESinput_R2[0] = m_pre.at(CR2_m/*6*/);// Input
        ESN_R2->setInput(ESinput_R2, 1/* no. input*/);
        ESN_R2->takeStep(ESTrainOutput_R2, learning_rate/*0.99 *RLS/ /*0.00055/*0.0005*/ /*0.0055*//*1.5*//*1.8*/, 1 /*no td = 1 else td_error*/, learn/* true= learn, false = not learning learn_critic*/, 0);
        //fmodel_cmr_output_rc.at(2) = ESN_R2->outputs->val(0, 0);
        fmodel_cmr2_output_rc.at(0) = ESN_R2->outputs->val(0, 0); // first output
        fmodel_cmr2_output_rc.at(1) = ESN_R2->outputs->val(1, 0); // second output
        fmodel_cmr2_output_rc.at(2) = ESN_R2->outputs->val(2, 0); // third output

        fmodel_cmr_output_rc.at(2) = fmodel_cmr2_output_rc.at(0)+fmodel_cmr2_output_rc.at(1)+fmodel_cmr2_output_rc.at(2);


        //-----Module ESN 4

        if(Control_input == gait1)
        {
          ESTrainOutput_L0[0]= reflex_L_fs.at(0); //Training output (target function)
          ESTrainOutput_L0[1]= 0.0; //Training output (target function)
          ESTrainOutput_L0[2]= 0.0; //Training output (target function)
        }
        if(Control_input == gait2)
        {
          ESTrainOutput_L0[0]= 0.0; //Training output (target function)
          ESTrainOutput_L0[1]= reflex_L_fs.at(0); //Training output (target function)
          ESTrainOutput_L0[2]= 0.0; //Training output (target function)
        }
        if(Control_input == gait3)
        {
          ESTrainOutput_L0[0]= 0.0; //Training output (target function)
          ESTrainOutput_L0[1]= 0.0; //Training output (target function)
          ESTrainOutput_L0[2]= reflex_L_fs.at(0); //Training output (target function)
        }

        //ESTrainOutput_L0[0]= reflex_L_fs.at(0); //Training output (target function)
        ESinput_L0[0] = m_pre.at(CL0_m/*6*/);// Input
        ESN_L0->setInput(ESinput_L0, 1/* no. input*/);
        ESN_L0->takeStep(ESTrainOutput_L0, learning_rate/*0.99 *RLS/ /*0.00055/*0.0005*/ /*0.0055*//*1.5*//*1.8*/, 1 /*no td = 1 else td_error*/, learn/* true= learn, false = not learning learn_critic*/, 0);
        //fmodel_cml_output_rc.at(0) = ESN_L0->outputs->val(0, 0);
        fmodel_cml0_output_rc.at(0) = ESN_L0->outputs->val(0, 0); // first output
        fmodel_cml0_output_rc.at(1) = ESN_L0->outputs->val(1, 0); // second output
        fmodel_cml0_output_rc.at(2) = ESN_L0->outputs->val(2, 0); // third output

        fmodel_cml_output_rc.at(0) = fmodel_cml0_output_rc.at(0)+fmodel_cml0_output_rc.at(1)+fmodel_cml0_output_rc.at(2);


        //-----Module ESN 5

        if(Control_input == gait1)
        {
          ESTrainOutput_L1[0]= reflex_L_fs.at(1); //Training output (target function)
          ESTrainOutput_L1[1]= 0.0; //Training output (target function)
          ESTrainOutput_L1[2]= 0.0; //Training output (target function)
        }
        if(Control_input == gait2)
        {
          ESTrainOutput_L1[0]= 0.0; //Training output (target function)
          ESTrainOutput_L1[1]= reflex_L_fs.at(1); //Training output (target function)
          ESTrainOutput_L1[2]= 0.0; //Training output (target function)
        }
        if(Control_input == gait3)
        {
          ESTrainOutput_L1[0]= 0.0; //Training output (target function)
          ESTrainOutput_L1[1]= 0.0; //Training output (target function)
          ESTrainOutput_L1[2]= reflex_L_fs.at(1); //Training output (target function)
        }

        //ESTrainOutput_L1[0]= reflex_L_fs.at(1); //Training output (target function)
        ESinput_L1[0] = m_pre.at(CL1_m/*6*/);// Input
        ESN_L1->setInput(ESinput_L1, 1/* no. input*/);
        ESN_L1->takeStep(ESTrainOutput_L1, learning_rate/*0.99 *RLS/ /*0.00055/*0.0005*/ /*0.0055*//*1.5*//*1.8*/, 1 /*no td = 1 else td_error*/, learn/* true= learn, false = not learning learn_critic*/, 0);
        //fmodel_cml_output_rc.at(1) = ESN_L1->outputs->val(0, 0);
        fmodel_cml1_output_rc.at(0) = ESN_L1->outputs->val(0, 0); // first output
        fmodel_cml1_output_rc.at(1) = ESN_L1->outputs->val(1, 0); // second output
        fmodel_cml1_output_rc.at(2) = ESN_L1->outputs->val(2, 0); // third output

        fmodel_cml_output_rc.at(1) = fmodel_cml1_output_rc.at(0)+fmodel_cml1_output_rc.at(1)+fmodel_cml1_output_rc.at(2);


        //-----Module ESN 6
        if(Control_input == gait1)
        {
          ESTrainOutput_L2[0]= reflex_L_fs.at(2); //Training output (target function)
          ESTrainOutput_L2[1]= 0.0; //Training output (target function)
          ESTrainOutput_L2[2]= 0.0; //Training output (target function)
        }
        if(Control_input == gait2)
        {
          ESTrainOutput_L2[0]= 0.0; //Training output (target function)
          ESTrainOutput_L2[1]= reflex_L_fs.at(2); //Training output (target function)
          ESTrainOutput_L2[2]= 0.0; //Training output (target function)
        }
        if(Control_input == gait3)
        {
          ESTrainOutput_L2[0]= 0.0; //Training output (target function)
          ESTrainOutput_L2[1]= 0.0; //Training output (target function)
          ESTrainOutput_L2[2]= reflex_L_fs.at(2); //Training output (target function)
        }
        //ESTrainOutput_L2[0]= reflex_L_fs.at(2); //Training output (target function)
        ESinput_L2[0] = m_pre.at(CL2_m/*6*/);// Input
        ESN_L2->setInput(ESinput_L2, 1/* no. input*/);
        ESN_L2->takeStep(ESTrainOutput_L2, learning_rate/*0.99 *RLS/ /*0.00055/*0.0005*/ /*0.0055*//*1.5*//*1.8*/, 1 /*no td = 1 else td_error*/, learn/* true= learn, false = not learning learn_critic*/, 0);
        //fmodel_cml_output_rc.at(2) = ESN_L2->outputs->val(0, 0);
        fmodel_cml2_output_rc.at(0) = ESN_L2->outputs->val(0, 0); // first output
        fmodel_cml2_output_rc.at(1) = ESN_L2->outputs->val(1, 0); // second output
        fmodel_cml2_output_rc.at(2) = ESN_L2->outputs->val(2, 0); // third output

        fmodel_cml_output_rc.at(2) = fmodel_cml2_output_rc.at(0)+fmodel_cml2_output_rc.at(1)+fmodel_cml2_output_rc.at(2);


        //------------Add ESN training (3)----------------------------------//



        //-----------Calculating error--------------------------------------//

        for(unsigned int i=0; i<fmodel_cml_activity.size();i++)
        {
          double lowpass_error_gain;
          lowpass_error_gain = 0.95;

          low_pass_fmodel_cmr_error_old.at(i) = low_pass_fmodel_cmr_error.at(i);
          low_pass_fmodel_cml_error_old.at(i) = low_pass_fmodel_cml_error.at(i);

          //Calculate error
          fmodel_cmr_error.at(i) = reflex_R_fs.at(i)-fmodel_cmr_output_rc.at(i) /*regulate error*/; //target - output // only positive error
          low_pass_fmodel_cmr_error.at(i) = low_pass_fmodel_cmr_error_old.at(i)*lowpass_error_gain+(1-lowpass_error_gain)*fmodel_cmr_error.at(i);

          //Calculate error
          fmodel_cml_error.at(i) = reflex_L_fs.at(i)-fmodel_cml_output_rc.at(i) /*regulate error*/; //target - output // only positive error
          low_pass_fmodel_cml_error.at(i) = low_pass_fmodel_cml_error_old.at(i)*lowpass_error_gain+(1-lowpass_error_gain)*fmodel_cml_error.at(i);


          std::cout<<"pid_control"<<std::endl;

          //------------------Right legs------------------------------------//
          //-------STANCE--------------Searching control--------------------//
          if(postcr.at(i)<postcrold.at(i)) //stance
          {

            //Positive Error signal for controlling searching reflexes
            if(abs(low_pass_fmodel_cmr_error.at(i))>0.45)//0.15)
            {

              d_r.at(i) = low_pass_fmodel_cmr_error.at(i)-low_pass_fmodel_cmr_error_old.at(i);
              int_r.at(i) += abs(low_pass_fmodel_cmr_error.at(i));
              acc_cmr_error.at(i) = abs(low_pass_fmodel_cmr_error.at(i))*kp_r.at(i)+int_r.at(i)*ki_r.at(i)+d_r.at(i)*kd_r.at(i);

              if(acc_cmr_error.at(i)>500)
                acc_cmr_error.at(i) = 500;
            }
            else
            {
              acc_cmr_error.at(i) = 0.0;
            }

            //To store the last acc_error value of in stance phase
            acc_cmr_error_old.at(i) = acc_cmr_error.at(i);

            //std::cout<<"acc_error_control"<<i<<" "<<acc_cmr_error.at(i)<<std::endl;

            //Reset error signals of elevator
            acc_cmr_error_elev.at(i) = 0.0;
            error_cmr_elev.at(i) = 0.0;
          }
          else // swing phase
          {
            //-----SWING----------------Elevation control-----------------//

            //To store the last acc_error value of in stance phase-> for setting max or min in the next swing & stance phases
            max_error_cmr_pre_step.at(i) =  acc_cmr_error_old.at(i);

            //Negative Error signal for controlling elevator reflexes
            if(low_pass_fmodel_cmr_error.at(i)<0.35)//0.25)
            {
              acc_cmr_error_elev.at(i) += abs(low_pass_fmodel_cmr_error.at(i));
            }
            else
            {
              acc_cmr_error_elev.at(i) = 0.0;
            }

            //Reset error signals of searching
            acc_cmr_error.at(i) = 0;
            int_r.at(i) = 0.0;
            d_r.at(i) = 0.0;

            error_cmr_elev.at(i) = 1.0;
          }

          //------------------Left legs------------------------------------//
          //-------STANCE--------------Searching control--------------------//
          if(postcl.at(i)<postclold.at(i)) //stance
          {

            //Positive Error signal for controlling searching reflexes
            if(abs(low_pass_fmodel_cml_error.at(i))>0.45)//0.15)
            {

              d_l.at(i) = low_pass_fmodel_cml_error.at(i)-low_pass_fmodel_cml_error_old.at(i);
              int_l.at(i) += abs(low_pass_fmodel_cml_error.at(i));
              acc_cml_error.at(i) = abs(low_pass_fmodel_cml_error.at(i))*kp_l.at(i)+int_l.at(i)*ki_l.at(i)+d_l.at(i)*kd_l.at(i);

              if(acc_cml_error.at(i)>500)
                acc_cml_error.at(i) = 500;
            }
            else
            {
              acc_cml_error.at(i) = 0.0;
            }

            //To store the last acc_error value of in stance phase
            acc_cml_error_old.at(i) = acc_cml_error.at(i);

            //std::cout<<"acc_error_control"<<i<<" "<<acc_cml_error.at(i)<<std::endl;

            //Reset error signals of elevator
            acc_cml_error_elev.at(i) = 0.0;
            error_cml_elev.at(i) = 0.0;
          }
          else //swing phase
          {
            //-----SWING----------------Elevation control-----------------//
            //To store the last acc_error value of in stance phase-> for setting max or min in the next swing & stance phases
            max_error_cml_pre_step.at(i) =  acc_cml_error_old.at(i);

            //Negative Error signal for controlling elevator reflexes
            if(low_pass_fmodel_cml_error.at(i)<0.35)//0.25)
            {
              acc_cml_error_elev.at(i) += abs(low_pass_fmodel_cml_error.at(i));
            }
            else
            {
              acc_cml_error_elev.at(i) = 0.0;
            }

            //Reset error signals of searching
            //low_pass_fmodel_cml_error.at(i) = 0.0;
            acc_cml_error.at(i) = 0;
            int_l.at(i) = 0.0;
            d_l.at(i) = 0.0;

            error_cml_elev.at(i) = 1.0;
          }
        }


        break;

    }
  }
  // >> i/o operations here <<
  //  outFilenlc1<<m_pre.at(CR0_m)<<' '<<reflex_R_fs.at(0)<<' '<<fmodel_cmr_output.at(0)<<' '<<fmodel_cmr_errorW.at(0)<<' '<<fmodel_cmr_outputfinal.at(0)<<' '<<fmodel_cmr_error.at(0)<<' '<<low_pass_fmodel_cmr_error.at(0)<<' '<<acc_cmr_error_old.at(0)<<' '<<acc_cmr_error.at(0) /*searching reflex*/<<' '<<acc_cmr_error_elev.at(0) /*elevator reflexes*/<<' '<<fmodel_cmr_w.at(0)<<' '<<fmodel_fmodel_cmr_w.at(0)<<' '<<fmodel_cmr_bias.at(0)<<' '<<endl;
  //  outFilenlc2<<m_pre.at(CR1_m)<<' '<<reflex_R_fs.at(1)<<' '<<fmodel_cmr_output.at(1)<<' '<<fmodel_cmr_errorW.at(1)<<' '<<fmodel_cmr_outputfinal.at(1)<<' '<<fmodel_cmr_error.at(1)<<' '<<low_pass_fmodel_cmr_error.at(1)<<' '<<acc_cmr_error_old.at(1)<<' '<<acc_cmr_error.at(1) /*searching reflex*/<<' '<<acc_cmr_error_elev.at(1) /*elevator reflexes*/<<' '<<fmodel_cmr_w.at(1)<<' '<<fmodel_fmodel_cmr_w.at(1)<<' '<<fmodel_cmr_bias.at(1)<<' '<<endl;
  //  outFilenlc3<<m_pre.at(CR2_m)<<' '<<reflex_R_fs.at(2)<<' '<<fmodel_cmr_output.at(2)<<' '<<fmodel_cmr_errorW.at(2)<<' '<<fmodel_cmr_outputfinal.at(2)<<' '<<fmodel_cmr_error.at(2)<<' '<<low_pass_fmodel_cmr_error.at(2)<<' '<<acc_cmr_error_old.at(2)<<' '<<acc_cmr_error.at(2) /*searching reflex*/<<' '<<acc_cmr_error_elev.at(2) /*elevator reflexes*/<<' '<<fmodel_cmr_w.at(2)<<' '<<fmodel_fmodel_cmr_w.at(2)<<' '<<fmodel_cmr_bias.at(2)<<' '<<endl;
  //
  //  outFilenlc4<<m_pre.at(CL0_m)<<' '<<reflex_L_fs.at(0)<<' '<<fmodel_cml_output.at(0)<<' '<<fmodel_cml_errorW.at(0)<<' '<<fmodel_cml_outputfinal.at(0)<<' '<<fmodel_cml_error.at(0)<<' '<<low_pass_fmodel_cml_error.at(0)<<' '<<acc_cml_error_old.at(0)<<' '<<acc_cml_error.at(0) /*searching reflex*/<<' '<<acc_cml_error_elev.at(0) /*elevator reflexes*/<<' '<<fmodel_cml_w.at(0)<<' '<<fmodel_fmodel_cml_w.at(0)<<' '<<fmodel_cml_bias.at(0)<<' '<<endl;
  //  outFilenlc5<<m_pre.at(CL1_m)<<' '<<reflex_L_fs.at(1)<<' '<<fmodel_cml_output.at(1)<<' '<<fmodel_cml_errorW.at(1)<<' '<<fmodel_cml_outputfinal.at(1)<<' '<<fmodel_cml_error.at(1)<<' '<<low_pass_fmodel_cml_error.at(1)<<' '<<acc_cml_error_old.at(1)<<' '<<acc_cml_error.at(1) /*searching reflex*/<<' '<<acc_cml_error_elev.at(1) /*elevator reflexes*/<<' '<<fmodel_cml_w.at(1)<<' '<<fmodel_fmodel_cml_w.at(1)<<' '<<fmodel_cml_bias.at(1)<<' '<<endl;


  outFilepower1<<acc_cmr_error.at(0)<<' '<<acc_cmr_error_old.at(0)<<' '<<max_error_cmr_pre_step.at(0)<<' '<<m_reflex.at(TR0_m)<<' '<<m_reflex.at(CR0_m)<<' '<<m_reflex.at(FR0_m)<<' '<<postcr.at(0)<<' '<<acc_cmr_error.at(1)<<' '<<acc_cmr_error_old.at(1)<<' '<<max_error_cmr_pre_step.at(1)<<' '<<acc_cmr_error.at(2)<<' '<<acc_cmr_error_old.at(2)<<' '<<max_error_cmr_pre_step.at(2)<<' '<<endl;

  //******Reflex mechanisms end******

  //-----------------------------AMOSiiV2_Config------------------------------------------//
  /*******************************************************************************
   *  MODULE 7 REFLEX MECHANISMS
   *******************************************************************************/

  //******Motor mapping and searching reflexes*****

  //*****Motor mapping
  /*
		->TC range [MAX, MIN +70,...,-70 deg]
		->mapping: output-min_r/(max_r-min_r) = input-min/(max-min)
   */

  m_reflex_old.at(TR0_m) = m_reflex.at(TR0_m);
  m_reflex_old.at(TR1_m) = m_reflex.at(TR1_m);
  m_reflex_old.at(TR2_m) = m_reflex.at(TR2_m);
  m_reflex_old.at(TL0_m) = m_reflex.at(TL0_m);
  m_reflex_old.at(TL1_m) = m_reflex.at(TL1_m);
  m_reflex_old.at(TL2_m) = m_reflex.at(TL2_m);


  //1) TC_front//->normal walking range: front legs 60 deg Max = 0.858, -10 deg Min = -0.143

  min_tc_f_nwalking = 0.0143*min_tc_f_nwalking_deg;
  max_tc_f_nwalking = 0.0143*max_tc_f_nwalking_deg;

  m_reflex.at(TR0_m) = (((m_pre.at(TR0_m)-min_tc)/(max_tc-min_tc))*(max_tc_f_nwalking-min_tc_f_nwalking))+min_tc_f_nwalking;
  m_reflex.at(TL0_m) = (((m_pre.at(TL0_m)-min_tc)/(max_tc-min_tc))*(max_tc_f_nwalking-min_tc_f_nwalking))+min_tc_f_nwalking;

  //  //convert from activation to deg
  //  m_deg.at(TR0_m) = 70*m_reflex.at(TR0_m);
  //  m_deg.at(TL0_m) = 70*m_reflex.at(TL0_m);
  //
  //  std::cout<<"motor deg TR0"<<":"<<m_deg.at(TR0_m)<< "\n";
  //  std::cout<<"motor deg TL0"<<":"<<m_deg.at(TL0_m)<< "\n";

  //TC_middle//->normal walking range: middle legs 30 deg Max = 0.501, -40 deg Min = -0.668

  min_tc_m_nwalking = 0.0167*min_tc_m_nwalking_deg;
  max_tc_m_nwalking = 0.0167*max_tc_m_nwalking_deg;

  m_reflex.at(TR1_m) = (((m_pre.at(TR1_m)-min_tc)/(max_tc-min_tc))*(max_tc_m_nwalking-min_tc_m_nwalking))+min_tc_m_nwalking;
  m_reflex.at(TL1_m) = (((m_pre.at(TL1_m)-min_tc)/(max_tc-min_tc))*(max_tc_m_nwalking-min_tc_m_nwalking))+min_tc_m_nwalking;

  //convert from activation to deg
  //  m_deg.at(TR1_m) = 60*m_reflex.at(TR1_m);
  //  m_deg.at(TL1_m) = 60*m_reflex.at(TL1_m);
  //
  //  std::cout<<"motor deg TR1"<<":"<<m_deg.at(TR1_m)<< "\n";
  //  std::cout<<"motor deg TL1"<<":"<<m_deg.at(TL1_m)<< "\n";

  //TC_rear//->normal walking range: hind legs 10 deg Max = 0.143, -60 deg Min = -0.858

  min_tc_r_nwalking = 0.0143*min_tc_r_nwalking_deg;
  max_tc_r_nwalking = 0.0143*max_tc_r_nwalking_deg;

  m_reflex.at(TR2_m) = (((m_pre.at(TR2_m)-min_tc)/(max_tc-min_tc))*(max_tc_r_nwalking-min_tc_r_nwalking))+min_tc_r_nwalking;
  m_reflex.at(TL2_m) = (((m_pre.at(TL2_m)-min_tc)/(max_tc-min_tc))*(max_tc_r_nwalking-min_tc_r_nwalking))+min_tc_r_nwalking;

  /*Gap crossing*/
  if(crossing_gap && global_count>700)
  {

    for(unsigned int i=0; i<acc_cmr_error.size();i++)
     {

      //Max value for Control input 0.09 is 260
      if(max_error_cmr_pre_step.at(i)>260)
        max_error_cmr_pre_step.at(i) = 260;

      if(max_error_cml_pre_step.at(i)>260)
        max_error_cml_pre_step.at(i) = 260;

      offset_tcr.at(i) = (max_error_cmr_pre_step.at(i)/1000)*1.15; // (max_error_cmr_pre_step = 260, max offset_tcr = 0.3)
      offset_tcl.at(i) = (max_error_cml_pre_step.at(i)/1000)*1.15;//0.3;//acc_cml_error.at(0)/100;//(max_c/max_c_offset);//0.6216;/*Linear or Exponential function!!**/


      min_tcr_nwalking.at(i) = 0.0143*min_tc_f_nwalking_deg;//+offset_tcr.at(0);
      max_tcr_nwalking.at(i) = 0.0143*max_tc_f_nwalking_deg+offset_tcr.at(i);
      m_reflex.at(TR0_m+i) = (((m_pre.at(TR0_m+i)-min_tc)/(max_tc-min_tc))*(max_tcr_nwalking.at(i)-min_tcr_nwalking.at(i)))+min_tcr_nwalking.at(i);


      min_tcl_nwalking.at(i) = 0.0143*min_tc_f_nwalking_deg;//+offset_tcl.at(0);
      max_tcl_nwalking.at(i) = 0.0143*max_tc_f_nwalking_deg+offset_tcl.at(i);
      m_reflex.at(TL0_m+i) = (((m_pre.at(TL0_m+i)-min_tc)/(max_tc-min_tc))*(max_tcl_nwalking.at(i)-min_tcl_nwalking.at(i)))+min_tcl_nwalking.at(i);


      min_tcr_nwalking.at(1) = 0.0167*min_tc_m_nwalking_deg;//+offset_tcr.at(1);
      max_tcr_nwalking.at(1) = 0.0167*max_tc_m_nwalking_deg+offset_tcr.at(1);
      m_reflex.at(TR1_m) = (((m_pre.at(TR1_m)-min_tc)/(max_tc-min_tc))*(max_tcr_nwalking.at(1)-min_tcr_nwalking.at(1)))+min_tcr_nwalking.at(1);


      min_tcl_nwalking.at(1) = 0.0167*min_tc_m_nwalking_deg;//+offset_tcl.at(1);
      max_tcl_nwalking.at(1) = 0.0167*max_tc_m_nwalking_deg+offset_tcl.at(1);
      m_reflex.at(TL1_m) = (((m_pre.at(TL1_m)-min_tc)/(max_tc-min_tc))*(max_tcl_nwalking.at(1)-min_tcl_nwalking.at(1)))+min_tcl_nwalking.at(1);


      min_tcr_nwalking.at(2) = 0.0143*min_tc_r_nwalking_deg+offset_tcr.at(2);
      max_tcr_nwalking.at(2) = 0.0143*max_tc_r_nwalking_deg+offset_tcr.at(2);
      m_reflex.at(TR2_m) = (((m_pre.at(TR2_m)-min_tc)/(max_tc-min_tc))*(max_tcr_nwalking.at(2)-min_tcr_nwalking.at(2)))+min_tcr_nwalking.at(2);

      min_tcl_nwalking.at(2) = 0.0143*min_tc_r_nwalking_deg+offset_tcl.at(2);
      max_tcl_nwalking.at(2) = 0.0143*max_tc_r_nwalking_deg+offset_tcl.at(2);
      m_reflex.at(TL2_m) = (((m_pre.at(TL2_m)-min_tc)/(max_tc-min_tc))*(max_tcl_nwalking.at(2)-min_tcl_nwalking.at(2)))+min_tcl_nwalking.at(2);

     }


  }


  //convert from activation to deg
  //  m_deg.at(TR2_m) = 70*m_reflex.at(TR2_m);
  //  m_deg.at(TL2_m) = 70*m_reflex.at(TL2_m);
  //
  //  std::cout<<"motor deg TR2"<<":"<<m_deg.at(TR2_m)<< "\n";
  //  std::cout<<"motor deg TL2"<<":"<<m_deg.at(TL2_m)<< "\n";


  //*****Motor mapping & Searching reflexes

  //2) CTr range [+75,...,-75 deg]
  //normal walking range
  //up 75 deg Max = 1.0, 50 deg Min = 0.715

  for(unsigned int i=0; i<acc_cmr_error.size();i++)
  {
    //1) positive accumulated error = acc_cmr_error.at(i) -->for searching reflexes
    //2) negative accumulated error --> for elevating reflex


    offset_ctr.at(i) = 0.0;/*0...115 Linear or Exponential function!!**/
    offset_ctl.at(i) = 0.0;/*Linear or Exponential function!!**/

    if(switchon_reflexes)
    {
      /*2) TO DO NEED TO BE ADJUSTED THIS MAPPING FROM ACC_ERROR TO OFFSET*/

      if(use_pre_step_to_adjust_searching)
      {

      offset_ctr.at(i) = (max_error_cmr_pre_step.at(i)/max_scale)*acc_cmr_error.at(i)*(max_c/max_c_offset);//0.6216;/*0...115 Linear or Exponential function!!**/
      offset_ctl.at(i) = (max_error_cml_pre_step.at(i)/max_scale)*acc_cml_error.at(i)*(max_c/max_c_offset);//0.6216;/*Linear or Exponential function!!**/

      offset_ctr.at(2) = (max_error_cmr_pre_step.at(2)/max_scale)*acc_cmr_error.at(2)*(max_c/ (max_c_offset+40));//0.5946;/*0...110 Linear or Exponential function!!**/
      offset_ctl.at(2) = (max_error_cml_pre_step.at(2)/max_scale)*acc_cml_error.at(2)*(max_c/ (max_c_offset+40));//0.5946;/*0...110 Linear or Exponential function!!**/
      }
      else
      {

      offset_ctr.at(i) = acc_cmr_error.at(i)*(max_c/max_c_offset);//0.6216;/*0...115 Linear or Exponential function!!**/
      offset_ctl.at(i) = acc_cml_error.at(i)*(max_c/max_c_offset);//0.6216;/*Linear or Exponential function!!**/

      offset_ctr.at(2) = acc_cmr_error.at(2)*(max_c/ (max_c_offset+40));//0.5946;/*0...110 Linear or Exponential function!!**/
      offset_ctl.at(2) = acc_cml_error.at(2)*(max_c/ (max_c_offset+40));//0.5946;/*0...110 Linear or Exponential function!!**/
      }


      //      offset_ctr.at(i) = acc_cmr_error.at(i)/10;//*(max_c/max_c_offset);//0.6216;/*0...115 Linear or Exponential function!!**/
      //      offset_ctl.at(i) = acc_cml_error.at(i)/10;//*(max_c/max_c_offset);//0.6216;/*Linear or Exponential function!!**/
      //
      //      offset_ctr.at(2) = acc_cmr_error.at(i)/10;//*(max_c/ (max_c_offset+40));//0.5946;/*0...110 Linear or Exponential function!!**/
      //      offset_ctl.at(2) = acc_cml_error.at(i)/10;//*(max_c/ (max_c_offset+40));//0.5946;/*0...110 Linear or Exponential function!!**/

      //max_c_offset ; Adjust from 45,...185=> 185 (cin= 0.02, wave), 45 (cin= 0.18, tripod)
    }

    if(switchoff_searching_reflexes)
    {
      offset_ctr.at(i) = 0.0;/*0...115 Linear or Exponential function!!**/
      offset_ctl.at(i) = 0.0;/*Linear or Exponential function!!**/
    }

    //offset_ctr.at(i) = 150.0;// Change this parameter to extend leg
    //offset_ctl.at(i) = 150.0;// Change this parameter to extend leg

    //------Right CTR joints---------------------//
    //acc error upto 0...180

    //-----------Searching reflexes START---------------------------------------//
    //min_ctr_nwalking.at(i) = 0.0143*(min_ctr_nwalking_deg.at(i)-offset_ctr.at(i));//0.715;// MIN -70 deg
    //max_ctr_nwalking.at(i) = 0.0143*(max_ctr_nwalking_deg.at(i)-offset_ctr.at(i));//1; // MAX +70 deg


    //KOH-->Eduard
    min_ctr_nwalking.at(i) = 0.0133*(min_ctr_nwalking_deg.at(i)-offset_ctr.at(i)-offset_ctr_downward.at(i));//0.715;// 50 deg, MIN -75 deg
    max_ctr_nwalking.at(i) = 0.0133*(max_ctr_nwalking_deg.at(i)-offset_ctr.at(i)-offset_ctr_downward.at(i));//1; // 70 deg, MAX +75 deg

    m_reflex.at(i+CR0_m/*6*/) = (((m_pre.at(i+CR0_m/*6*/)-min_ctr)/(max_ctr-min_ctr))*(max_ctr_nwalking.at(i)-min_ctr_nwalking.at(i)))+min_ctr_nwalking.at(i);
    //m_reflex.at(i+CR0_m/*6*/) = 0.7865; //= +55 deg M shape with body touch ground
    //-----------Searching reflexes END----------------------------------------//

    //convert from activation to deg
    //m_deg.at(i+CR0_m/*6*/) = 75*m_reflex.at(i+CR0_m/*6*/);
    //std::cout<<"motor deg CR0"<<":"<<m_deg.at(CR0_m)<< "\n";


    //------Left CTR joints---------------------//
    //acc error upto 0...180

    //-----------Searching reflexes START----------------------------------------//

    //KOH-->Eduard
    min_ctl_nwalking.at(i) = 0.0133*(min_ctl_nwalking_deg.at(i)-offset_ctl.at(i)-offset_ctl_downward.at(i));//0.715;// 50 deg
    max_ctl_nwalking.at(i) = 0.0133*(max_ctl_nwalking_deg.at(i)-offset_ctl.at(i)-offset_ctl_downward.at(i));//1; // 70 deg

    m_reflex.at(i+CL0_m/*9*/) = (((m_pre.at(i+CL0_m/*9*/)-min_ctr)/(max_ctr-min_ctr))*(max_ctl_nwalking.at(i)-min_ctl_nwalking.at(i)))+min_ctl_nwalking.at(i);
    //-----------Searching reflexes END-----------------------------------------//


    //convert from activation to deg
    //m_deg.at(i+CL0_m/*9*/) = 75*m_reflex.at(i+CL0_m/*9*/);
    //std::cout<<"motor deg CL0"<<":"<<m_deg.at(CL0_m)<< "\n";

    //std::cout<<"motor deg CR"<<" "<<i<<":"<<m_deg.at(i+CR0_m/*6*/)<< "\n";
    //std::cout<<"motor deg CL"<<" "<<i<<":"<<m_deg.at(i+CL0_m/*6*/)<< "\n";

  }

  //3) FTi range [-20,...,-130 deg] neural activation = 0.0182*angle(deg)+1.3636
  //normal walking range
  //up -120 deg Max = -0.8204, -130 deg Min = -1.0024

  for(unsigned int i=0; i<acc_cmr_error.size();i++)
  {

    //------Right FTI joints---------------------//
    //acc error upto 0...180

    if(switchon_reflexes==true)
    {
      /*2) TO DO NEED TO BE ADJUSTED THIS MAPPING FROM ACC_ERROR TO OFFSET*/

      if(use_pre_step_to_adjust_searching)
      {
      offset_ftir.at(i) = (max_error_cmr_pre_step.at(i)/max_scale)*acc_cmr_error.at(i)*(max_f/ max_f_offset);//0.5946;/*0...110 Linear or Exponential function!!**/
      offset_ftil.at(i) = (max_error_cml_pre_step.at(i)/max_scale)*acc_cml_error.at(i)*(max_f/ max_f_offset);//0.5946;/*0...110 Linear or Exponential function!!**/

      //Too make FTI less extend for more stable walking
      offset_ftir.at(2) = (max_error_cmr_pre_step.at(2)/max_scale)*acc_cmr_error.at(2)*(max_f/ (max_f_offset+40));//0.5946;/*0...110 Linear or Exponential function!!**/
      offset_ftil.at(2) = (max_error_cmr_pre_step.at(2)/max_scale)*acc_cml_error.at(2)*(max_f/ (max_f_offset+40));//0.5946;/*0...110 Linear or Exponential function!!**/
      }
      else
      {
      offset_ftir.at(i) = acc_cmr_error.at(i)*(max_f/ max_f_offset);//0.5946;/*0...110 Linear or Exponential function!!**/
      offset_ftil.at(i) = acc_cml_error.at(i)*(max_f/ max_f_offset);//0.5946;/*0...110 Linear or Exponential function!!**/

      //Too make FTI less extend for more stable walking
      offset_ftir.at(2) = acc_cmr_error.at(2)*(max_f/ (max_f_offset+40));//0.5946;/*0...110 Linear or Exponential function!!**/
      offset_ftil.at(2) = acc_cml_error.at(2)*(max_f/ (max_f_offset+40));//0.5946;/*0...110 Linear or Exponential function!!**/
      }


      //      offset_ftir.at(i) = acc_cmr_error.at(i)/10;//*(max_f/ max_f_offset);//0.5946;/*0...110 Linear or Exponential function!!**/
      //      offset_ftil.at(i) = acc_cml_error.at(i)/10;//*(max_f/ max_f_offset);//0.5946;/*0...110 Linear or Exponential function!!**/
      //
      //      //Too make FTI less extend for more stable walking
      //      offset_ftir.at(2) = acc_cmr_error.at(i)/10;//*(max_f/ (max_f_offset));//+40));//0.5946;/*0...110 Linear or Exponential function!!**/
      //      offset_ftil.at(2) = acc_cml_error.at(i)/10;//*(max_f/ (max_f_offset));//+40));//0.5946;/*0...110 Linear or Exponential function!!**/

      //max_f_offset = 45.0; // Adjust from 45,...185=> 185 (cin= 0.02, wave), 45 (cin= 0.18, tripod)

    }

    if(switchon_reflexes==false)
    {
      offset_ftir.at(i) = 0.0;/*0...110 Linear or Exponential function!!**/
      offset_ftil.at(i) = 0.0;/*0...110 Linear or Exponential function!!**/
      acc_cmr_error_elev.at(i) = 0.0;
      acc_cml_error_elev.at(i) = 0.0;
    }


    if(switchoff_searching_reflexes)
    {
      offset_ftir.at(i) = 0.0;/*0...110 Linear or Exponential function!!**/
      offset_ftil.at(i) = 0.0;/*0...110 Linear or Exponential function!!**/
    }

    //offset_ftir.at(i) = 150.0;// Change this parameter to extend leg
    //offset_ftil.at(i) = 150.0;// Change this parameter to extend leg

    //-----------Searching reflexes START----------------------------------------//
    min_ftir_nwalking.at(i) = 0.0182*(min_ftir_nwalking_deg.at(i)+offset_ftir.at(i)+offset_ftir_downward.at(i))+1.3636;
    max_ftir_nwalking.at(i) = 0.0182*(max_ftir_nwalking_deg.at(i)+offset_ftir.at(i)+offset_ftir_downward.at(i))+1.3636;

    m_reflex.at(i+FR0_m/*12*/) = (((m_pre.at(i+FR0_m/*12*/)-min_fti)/(max_fti-min_fti))*(max_ftir_nwalking.at(i)-min_ftir_nwalking.at(i)))+min_ftir_nwalking.at(i);


    /*Gap crossing*/
    if(crossing_gap && global_count>700)
    {

      if(max_error_cmr_pre_step.at(i)>260)
        max_error_cmr_pre_step.at(i) = 260;

      offset_ftir.at(0) = max_error_cmr_pre_step.at(0)/15;//18;//15;//acc_cmr_error.at(0)/100;//(max_c/max_c_offset);//0.6216;/*0...115 Linear or Exponential function!!**/
      offset_ftir.at(1) = max_error_cmr_pre_step.at(1)/18;//15;//acc_cmr_error.at(0)/100;//(max_c/max_c_offset);//0.6216;/*0...115 Linear or Exponential function!!**/
      offset_ftir.at(2) = max_error_cmr_pre_step.at(2)/52;//5;//acc_cmr_error.at(0)/100;//(max_c/max_c_offset);//0.6216;/*0...115 Linear or Exponential function!!**/

      min_ftir_nwalking.at(i) = 0.0182*(min_ftir_nwalking_deg.at(i))+1.3636;
      max_ftir_nwalking.at(i) = 0.0182*(max_ftir_nwalking_deg.at(i)+offset_ftir.at(i))+1.3636;

      m_reflex.at(i+FR0_m/*12*/) = (((m_pre.at(i+FR0_m/*12*/)-min_fti)/(max_fti-min_fti))*(max_ftir_nwalking.at(i)-min_ftir_nwalking.at(i)))+min_ftir_nwalking.at(i);
    }

    //-----------Searching reflexes END-----------------------------------------//

    //-----------Elevator reflexes START----------------------------------------//

    if(elevator_reflexes==true)
    {

      if(i==0)
      {

        elevator_th = 0.7;/*simulation*///10;/*real robot*///15.0;//11.0;//20.0;// or smaller to on all the time
      }
      else if(i==1)
      {
        elevator_th = 0.7;/*simulation*///10;/*real robot*///15.0;//26.0;//20.0;// or smaller to on all the time
      }
      else if(i==2)
      {
        elevator_th = 0.7;/*simulation*///10;/*real robot*//15.0;//8.0;//20.0;// or smaller to on all the time
      }

      //      elevator_th = 20.0;//20.0;
      /*}
    else
    {
      elevator_th =900.0;
    }*/

      if(acc_cmr_error_elev.at(i)>elevator_th)//25.0)//20.0)--------------------------------------------------Dennis needs to change this 7.06.2012
        //if(acc_cmr_error_elev.at(i)>elevator_th || reflex_R_irs.at(i) > 0.9 && reflex_R_irs.at(i) < 1.1)//7.0
      {
        m_reflex.at(i+FR0_m/*12*/) = 0.8;//0.3;//0.1;//1.0;//0.6;//0.4; -0.4
        m_reflex.at(i+CR0_m) = 1.0;

        //For TR0
        if(i==0)
        {
          m_reflex.at(i+TR0_m) = m_reflex_old.at(i+TR0_m);//-0.1;//;//-0.2; // Dennis Change!!

          //                  for(int x=0; x<5; x++)
          //                  {
          //                    //m_reflex.at(i+TR0_m/*12*/) = -0.8+x*0.1;//-0.1+x*0.1;
          //                    m_reflex.at(i+TR0_m/*12*/) = m_reflex_old.at(i+TR0_m)-x*0.1;//-0.1+x*0.1;
          //                  }
        }

        //For TR1
        if(i==1)
        {
          m_reflex.at(i+TR0_m) = m_reflex_old.at(i+TR0_m);//-0.1;//;//-0.2; // Dennis Change!!

          //                  for(int x=0; x<5; x++)
          //                  {
          //                    //m_reflex.at(i+TR0_m/*12*/) = -0.8+x*0.1;
          //                    m_reflex.at(i+TR0_m/*12*/) = m_reflex_old.at(i+TR0_m)-x*0.1;//-0.1+x*0.1;
          //                  }
        }

        //For TR2
        if(i==2)
        {
          m_reflex.at(i+TR0_m) = m_reflex_old.at(i+TR0_m);//-0.1;//-0.2; // Dennis Change!!

          //                  for(int x=0; x<5; x++)
          //                  {
          //                    //m_reflex.at(i+TR0_m/*12*/) = -0.8+x*0.1;//0.0;//-0.95+x*0.1;
          //                    m_reflex.at(i+TR0_m/*12*/) = m_reflex_old.at(i+TR0_m)-x*0.1;//-0.1+x*0.1;
          //                  }
        }

      }
    }
    //convert from activation to deg
    //m_deg.at(i+FR0_m/*12*/) = 55*m_reflex.at(i+FR0_m/*12*/)-75;

    //-----------Elevator reflexes END----------------------------------------//

    //------Left FTI joints---------------------//
    //acc error upto 0...180

    //-----------Searching reflexes START----------------------------------------//
    min_ftil_nwalking.at(i) = 0.0182*(min_ftil_nwalking_deg.at(i)+offset_ftil.at(i)+offset_ftil_downward.at(i))+1.3636;
    max_ftil_nwalking.at(i) = 0.0182*(max_ftil_nwalking_deg.at(i)+offset_ftil.at(i)+offset_ftil_downward.at(i))+1.3636;

    m_reflex.at(i+FL0_m) = (((m_pre.at(i+FL0_m)-min_fti)/(max_fti-min_fti))*(max_ftil_nwalking.at(i)-min_ftil_nwalking.at(i)))+min_ftil_nwalking.at(i);

    //-----------Searching reflexes START----------------------------------------//

    //Gap crossing
    if(crossing_gap && global_count>700)
    {
      if(max_error_cml_pre_step.at(i)>260)
        max_error_cml_pre_step.at(i) = 260;

      offset_ftil.at(0) = max_error_cml_pre_step.at(0)/15;//18;//15;//acc_cmr_error.at(0)/100;//(max_c/max_c_offset);//0.6216;/*0...115 Linear or Exponential function!!**/
      offset_ftil.at(1) = max_error_cml_pre_step.at(1)/18;//15;//acc_cmr_error.at(0)/100;//(max_c/max_c_offset);//0.6216;/*0...115 Linear or Exponential function!!**/
      offset_ftil.at(2) = max_error_cml_pre_step.at(2)/52;//5;//acc_cmr_error.at(0)/100;//(max_c/max_c_offset);//0.6216;/*0...115 Linear or Exponential function!!**/

      min_ftil_nwalking.at(i) = 0.0182*(min_ftil_nwalking_deg.at(i))+1.3636;
      max_ftil_nwalking.at(i) = 0.0182*(max_ftil_nwalking_deg.at(i)+offset_ftil.at(i))+1.3636;

      m_reflex.at(i+FL0_m) = (((m_pre.at(i+FL0_m)-min_fti)/(max_fti-min_fti))*(max_ftil_nwalking.at(i)-min_ftil_nwalking.at(i)))+min_ftil_nwalking.at(i);
    }


    //-----------Elevator reflexes START----------------------------------------//

    if(elevator_reflexes==true)
    {

      if(i==0)
      {

        elevator_th = 0.7;/*simulation*///10;/*real robot*//15.0;//30.0;//20.0;// or smaller to on all the time
      }
      else if(i==1)
      {
        elevator_th = 0.7;/*simulation*///10;/*real robot*//15.0;//30.0;//20.0;// or smaller to on all the time
      }
      else if(i==2)
      {
        elevator_th = 0.7;/*simulation*///10;/*real robot*//15.0;//30.0;//20.0;// or smaller to on all the time
      }
      /*}
    else
    {
      elevator_th =900.0;
    }*/

      if(acc_cml_error_elev.at(i)>elevator_th)//25.0)//20.0)//900.0)//6.5)//7.0)
        //if(acc_cmr_error_elev.at(i)>elevator_th || reflex_R_irs.at(i) > 0.9 && reflex_R_irs.at(i) < 1.1)//7.0
      {
        //if(acc_cml_error_old.at(0)<-1)
        m_reflex.at(i+FL0_m/*12*/) = 0.8;//0.3;//-0.4;
        m_reflex.at(i+CL0_m) = 1.0;

        //For TL0
        if(i==0)
        {
          m_reflex.at(i+TL0_m) = m_reflex_old.at(i+TL0_m);//-0.1;//0.2; // Dennis Change!!

          //                  for(int x=0; x<5; x++)
          //                  {
          //                    //m_reflex.at(i+TL0_m/*12*/) = -0.8+x*0.1;//-0.1+x*0.1;
          //                    m_reflex.at(i+TL0_m/*12*/) = m_reflex_old.at(i+TL0_m)-x*0.1;//-0.1+x*0.1;
          //                  }
        }

        //For TL1
        if(i==1)
        {
          m_reflex.at(i+TL0_m) = m_reflex_old.at(i+TL0_m);//-0.1;//0.2; // Dennis Change!!

          //                  for(int x=0; x<5; x++)
          //                  {
          //                    //m_reflex.at(i+TL0_m/*12*/) = -0.8+x*0.1;
          //                    m_reflex.at(i+TL0_m/*12*/) = m_reflex_old.at(i+TL0_m)-x*0.1;
          //                  }
        }

        //For TL2
        if(i==2)
        {
          m_reflex.at(i+TL0_m) = m_reflex_old.at(i+TL0_m);//-0.1;//0.2;//-0.2 // Dennis Change!!


          //                  for(int x=0; x<5; x++)
          //                  {
          //                    //m_reflex.at(i+TL0_m/*12*/) = -0.8+x*0.1;//0.0;//-0.95+x*0.1;
          //                    m_reflex.at(i+TL0_m/*12*/) = m_reflex_old.at(i+TL0_m)-x*0.1;//0.0;//-0.95+x*0.1;
          //                  }
        }
      }
    }
    //-----------Elevator reflexes END----------------------------------------//

    //convert from activation to deg
    // m_deg.at(i+FL0_m) = 55*m_reflex.at(i+FL0_m)-75;

    //m_reflex.at(i+FR0_m/*12*/) = 1; //max = -20 deg
    //m_reflex.at(i+FL0_m/*15*/) = 1; //max = -20 deg
    //m_reflex.at(i+FR0_m/*12*/) = -1; //min = -130 deg //M shape with body touch ground
    //m_reflex.at(i+FL0_m/*15*/) = -1; //min = -130 deg //M shape with body touch ground

    //std::cout<<"motor deg FR"<<" "<<i<<":"<<m_deg.at(i+FR0_m/*6*/)<< "\n";
    //std::cout<<"motor deg FL"<<" "<<i<<":"<<m_deg.at(i+FL0_m/*6*/)<< "\n";
  }

  //4) BJ range [-45,...,45 deg] neural activation = 0.0222*angle(deg)
  //normal walking range
  //=0.0

  /*Gap crossing*/
  if(crossing_gap && global_count>700)
  {
    static int local_counter_bj = 0;

    if(max_error_cmr_pre_step.at(0)>260) // Max value for control input = 0.09 is 260
      max_error_cmr_pre_step.at(0) = 260;

    if(max_error_cml_pre_step.at(0)>260)
      max_error_cml_pre_step.at(0) = 260;


    offset_bj = ((max_error_cmr_pre_step.at(0)+max_error_cml_pre_step.at(0))/2)/400; //300 "850" can be optimize by learning

    if((max_error_cmr_pre_step.at(0)+max_error_cml_pre_step.at(0))/2>50)
      local_counter_bj++;


    if(local_counter_bj>80 /*"80" can be optimized*/)
    {
      //Move BJ to downward
      offset_bj = -0.1;

      //Move BJ to normal position for 200-80 = 120 time steps
      if(local_counter_bj>100)//200)
        offset_bj = 0.0;

      //Reset
      if(local_counter_bj>200 /*"200" can be optimized*/)
        local_counter_bj = 0;
    }
    std::cout<<"local_counter_bj"<<":"<<local_counter_bj<< " Value check:" <<(max_error_cmr_pre_step.at(0)+max_error_cml_pre_step.at(0))/2<<"offset_bj "<<offset_bj<<"\n";


    min_bj_fwalking = 0.0222*min_bj_fwalking_deg+offset_bj;
    max_bj_fwalking = 0.0222*max_bj_fwalking_deg+offset_bj;
    m_reflex.at(BJ_m) = (((bj_output.at(0)-min_bj)/(max_bj-min_bj))*(max_bj_fwalking-min_bj_fwalking))+min_bj_fwalking;
  }
  else{

    min_bj_fwalking = 0.0222*min_bj_fwalking_deg;
    max_bj_fwalking = 0.0222*max_bj_fwalking_deg;
    m_reflex.at(BJ_m) = (((bj_output.at(0)-min_bj)/(max_bj-min_bj))*(max_bj_fwalking-min_bj_fwalking))+min_bj_fwalking;
  }


  //convert from activation to deg
  m_deg.at(BJ_m) =  45*m_reflex.at(BJ_m);
  //m_reflex.at(BJ_m) = 1; // max 45 deg
  //m_reflex.at(BJ_m) = -1; // min -45 deg
  std::cout<<"motor deg BJ"<<":"<<m_deg.at(BJ_m)<< "\n";


  //convert from activation to deg
  m_deg.at(TR0_m) = 70*m_reflex.at(TR0_m);
  m_deg.at(TL0_m) = 70*m_reflex.at(TL0_m);

  std::cout<<"motor deg TR0"<<":"<<m_deg.at(TR0_m)<< "\n";
  std::cout<<"motor deg TL0"<<":"<<m_deg.at(TL0_m)<< "\n";


  m_deg.at(TR1_m) = 60*m_reflex.at(TR1_m);
  m_deg.at(TL1_m) = 60*m_reflex.at(TL1_m);

  std::cout<<"motor deg TR1"<<":"<<m_deg.at(TR1_m)<< "\n";
  std::cout<<"motor deg TL1"<<":"<<m_deg.at(TL1_m)<< "\n";


  m_deg.at(TR2_m) = 70*m_reflex.at(TR2_m);
  m_deg.at(TL2_m) = 70*m_reflex.at(TL2_m);

  std::cout<<"motor deg TR2"<<":"<<m_deg.at(TR2_m)<< "\n";
  std::cout<<"motor deg TL2"<<":"<<m_deg.at(TL2_m)<< "\n";



  m_deg.at(CR0_m/*9*/) = 75*m_reflex.at(CR0_m/*9*/);
  m_deg.at(CL0_m/*9*/) = 75*m_reflex.at(CL0_m/*9*/);

  std::cout<<"motor deg CR0"<<m_deg.at(CR0_m/*6*/)<< "\n";
  std::cout<<"motor deg CL0"<<m_deg.at(CL0_m/*6*/)<< "\n";

  m_deg.at(CR1_m/*9*/) = 75*m_reflex.at(CR1_m/*9*/);
  m_deg.at(CL1_m/*9*/) = 75*m_reflex.at(CL1_m/*9*/);

  std::cout<<"motor deg CR1"<<m_deg.at(CR1_m/*6*/)<< "\n";
  std::cout<<"motor deg CL1"<<m_deg.at(CL1_m/*6*/)<< "\n";

  m_deg.at(CR2_m/*9*/) = 75*m_reflex.at(CR2_m/*9*/);
  m_deg.at(CL2_m/*9*/) = 75*m_reflex.at(CL2_m/*9*/);

  std::cout<<"motor deg CR2"<<m_deg.at(CR2_m/*6*/)<< "\n";
  std::cout<<"motor deg CL2"<<m_deg.at(CL2_m/*6*/)<< "\n";

  m_deg.at(FR0_m/*9*/) = 55*m_reflex.at(FR0_m)-75;
  m_deg.at(FL0_m/*9*/) = 55*m_reflex.at(FL0_m)-75;

  std::cout<<"motor deg FR0"<<m_deg.at(FR0_m/*6*/)<< "\n";
  std::cout<<"motor deg FL0"<<m_deg.at(FL0_m/*6*/)<< "\n";

  m_deg.at(FR1_m/*9*/) = 55*m_reflex.at(FR1_m)-75;
  m_deg.at(FL1_m/*9*/) = 55*m_reflex.at(FL1_m)-75;

  std::cout<<"motor deg FR1"<<m_deg.at(FR1_m/*6*/)<< "\n";
  std::cout<<"motor deg FL1"<<m_deg.at(FL1_m/*6*/)<< "\n";

  m_deg.at(FR2_m/*9*/) = 55*m_reflex.at(FR2_m)-75;
  m_deg.at(FL2_m/*9*/) = 55*m_reflex.at(FL2_m)-75;

  std::cout<<"motor deg FR2"<<m_deg.at(FR2_m/*6*/)<< "\n";
  std::cout<<"motor deg FL2"<<m_deg.at(FL2_m/*6*/)<< "\n";

  //-----------------------------AMOSiiV2_Config------------------------------------------//

  /*******************************************************************************
   *  MODULE 8 MUSCLE MODELS
   *******************************************************************************/
  /*XIAOFENG*/

  /*******************************************************************************
   *  FINAL MOTOR OUTPUTS TO MOTOR NEURONS
   *******************************************************************************/

  //------------------Reading signals from Text file
  if(reading_text_testing)
  {
    //Test the reading function
    char str[10];
    //std::string str;

    //Opens for reading the file
    //ifstream b_file ("soinn/ManyLegs_Trans_0.11_by_AMMC_gain=0.5_trainon0.02.txt");//( "Test1.txt" );
    ifstream b_file ("soinn/ManyLegs_Tripod_0.18_by_AMMC_gain=0.5_trainon0.02.txt");//( "Test1.txt" );
    //ifstream b_file ("soinn/ManyLegs_0.05_by_AMMC_gain=0.5_trainon0.02.txt");//( "Test1.txt" );
    //ifstream b_file ("soinn/AMMC_Neurallocomotion_0.18_R1wL2wR3_L1wR2wL3.txt");//( "Test1.txt" );

    //Reads one string from the file

    m_r0_t_old = m.at(TR0_m);
    m_r1_t_old = m.at(TR1_m);
    m_r2_t_old = m.at(TR2_m);
    m_l0_t_old = m.at(TL0_m);
    m_l1_t_old = m.at(TL1_m);
    m_l2_t_old = m.at(TL2_m);

    if(!initialized)
    {	i_text_loop = 0;
    while(b_file>>str) //time first column
    {

      // b_file>> str;
      m_r0_text = atof(str);//input1
      m_r0_t.at(i_text_loop) = m_r0_text;

      b_file>> str;
      m_r1_text = atof(str);//input2
      m_r1_t.at(i_text_loop) = m_r1_text;

      b_file>> str;
      m_r2_text = atof(str);//input3
      m_r2_t.at(i_text_loop) = m_r2_text;

      b_file>> str;
      m_l0_text = atof(str);//input4
      m_l0_t.at(i_text_loop) = m_l0_text;

      b_file>> str;
      m_l1_text = atof(str);//input5
      m_l1_t.at(i_text_loop) = m_l1_text;

      b_file>> str;
      m_l2_text = atof(str);//input6
      m_l2_t.at(i_text_loop) = m_l2_text;


      i_text_loop++;

      std::cout<<"mR0 text "<<m_r0_text<<"mR1 "<<m_r1_text<<"mR2 "<<m_r2_text<<"mL0 "<<m_l0_text<<"mL1 "<<m_l1_text<<"mL2 "<<m_l2_text <<" "<<i_text_loop<<"\n"<<endl;

    }
    initialized = true;
    }

    ///Use motor signal from text

    if(ii<i_text_loop)
    {
      ii++;
    }
    else
    {
      ii=0;
    }

    std::cout<<"ii "<<ii<<"i "<<i_text_loop<<"\n"<<endl;
    std::cout<<"mR0 out "<<m_r0_t.at(ii)<<"mR1 "<<m_r1_t.at(ii)<<"mR2 "<<m_r2_t.at(ii)<<"mL0 "<<m_l0_t.at(ii)<<"mL1 "<<m_l1_t.at(ii)<<"mL2 "<<m_l2_t.at(ii) <<" "<<i_text_loop<<"\n"<<endl;


    m.at(TR0_m) = m_r0_t.at(ii);// m_reflex.at(TR0_m);
    m.at(TR1_m) = m_r1_t.at(ii);//m_reflex.at(TR1_m);
    m.at(TR2_m) = m_r2_t.at(ii);//m_reflex.at(TR2_m);

    m.at(TL0_m) = m_l0_t.at(ii);//m_reflex.at(TL0_m);
    m.at(TL1_m) = m_l1_t.at(ii);//m_reflex.at(TL1_m);
    m.at(TL2_m) = m_l2_t.at(ii);//m_reflex.at(TL2_m);

    double up = 0.6;

    if(m.at(TR0_m)>m_r0_t_old)
      m.at(CR0_m) = m.at(TR0_m)+0.5;
    else
      m.at(CR0_m) = up;

    if(m.at(TR1_m)>m_r1_t_old)
      m.at(CR1_m) = m.at(TR1_m)+1.5;
    else
      m.at(CR1_m) = up;

    if(m.at(TR2_m)>m_r2_t_old)
      m.at(CR2_m) = m.at(TR2_m)+1.5;
    else
      m.at(CR2_m) = up;

    if(m.at(TL0_m)>m_l0_t_old)
      m.at(CL0_m) = m.at(TL0_m)+1.5;
    else
      m.at(CL0_m) = up;

    if(m.at(TL1_m)>m_l1_t_old)
      m.at(CL1_m) = m.at(TL1_m)+1.5;
    else
      m.at(CL1_m) = up;

    if(m.at(TL2_m)>m_l2_t_old)
      m.at(CL2_m) = m.at(TL2_m)+1.5;
    else
      m.at(CL2_m) = up;

    m.at(FR0_m) = -1;//m_reflex.at(FR0_m);
    m.at(FR1_m) = -1;//m_reflex.at(FR1_m);
    m.at(FR2_m) = -1;//m_reflex.at(FR2_m);

    m.at(FL0_m) = -1;//m_reflex.at(FL0_m);
    m.at(FL1_m) = -1;//m_reflex.at(FL1_m);
    m.at(FL2_m) = -1;//m_reflex.at(FL2_m);

    m.at(BJ_m) =  m_reflex.at(BJ_m);//(m_reflex.at(TR0_m)-0.35)*5;//

    //------------------Reading signals from Text file
  }
  else // using normal control
  {


    m.at(TR0_m) = m_reflex.at(TR0_m);
    m.at(TR1_m) = m_reflex.at(TR1_m);
    m.at(TR2_m) = m_reflex.at(TR2_m);//-1.0

    m.at(TL0_m) = m_reflex.at(TL0_m);
    m.at(TL1_m) = m_reflex.at(TL1_m);
    m.at(TL2_m) = m_reflex.at(TL2_m);//-1.0

    //   m.at(CL0_m) = 1;//m_reflex.at(CL0_m);
    m.at(CL0_m) = m_reflex.at(CL0_m);
    m.at(CL1_m) = m_reflex.at(CL1_m);
    m.at(CL1_m) = m_reflex.at(CL1_m);
    m.at(CL2_m) = m_reflex.at(CL2_m);
    //m.at(CL2_m) = 1;//m_reflex.at(CL2_m);

    m.at(CR0_m) = m_reflex.at(CR0_m);
    m.at(CR1_m) = m_reflex.at(CR1_m);
    m.at(CR2_m) = m_reflex.at(CR2_m);

    //    m.at(CR2_m) = 1;//m_reflex.at(CR2_m);


    m.at(FR0_m) = m_reflex.at(FR0_m);
    m.at(FR1_m) = m_reflex.at(FR1_m);
    m.at(FR2_m) = m_reflex.at(FR2_m);

    //m.at(FR2_m) = 1;//m_reflex.at(FR2_m);

    m.at(FL0_m) = m_reflex.at(FL0_m);
    m.at(FL1_m) = m_reflex.at(FL1_m);
    m.at(FL2_m) = m_reflex.at(FL2_m);

    //m.at(FL2_m) =1;// m_reflex.at(FL2_m);

    m.at(BJ_m) =  m_reflex.at(BJ_m);

  }

  //recording duty factor//

  //************DUTY FACTOR****************************//

  //Duty factor = the time when leg supports the body normalized to gait period: D = stance/ swing + stance
  //[Gaits with duty factors smaller < 0.5 are = runs (as horses gallop, trot, pace)]
  //a duty factor of 0.52 ± 0.08 for desert ants and 0.58 ± 0.06 for wood ants. Speed is the major
  //influence on duty factor (a-0.8 per m/s), i.e., at higher speeds the relative ground contact time diminishes.
  //In wood ants, only the front leg shows a significantly higher duty factor than in desert ants (ß = 0.03 ± 0.01).
  //Individual body size and inclination have no influence on duty factor.
  //In conclusion, safety parameters such as gait and duty factor are not influenced by slope

  stance_time = deltaxdown.at(0);
  swing_time  = deltaxup.at(0);
  stride_period =  stance_time+swing_time;
  if(stride_period == 0)
    stride_period =1;
  duty_factor = stance_time/stride_period; //>0.5 =slow, <0.5 = running
  std::cout<<"Duty_factor"<<duty_factor<<"stance_time"<<deltaxdown.at(0)<<"swing_time"<< deltaxup.at(0)<<"stride_period"<<stride_period<<"\n"<<endl;

  //caluculate from foot signal
  reflex_R_fs.at(0) = in0.at(R0_fs); //R0_fs = 19
  reflex_R_fs.at(1) = in0.at(R1_fs); //R1_fs = 20
  reflex_R_fs.at(2) = in0.at(R2_fs); //R2_fs = 21
  reflex_L_fs.at(0) = in0.at(L0_fs); //L0_fs = 22
  reflex_L_fs.at(1) = in0.at(L1_fs); //L1_fs = 23
  reflex_L_fs.at(2) = in0.at(L2_fs); //L2_fs = 24

  //Gait period is the time of realization of one sequence of leg transfers.

  //Gait Cycle: The period of time from one event (usually initial contact) of one foot to following occurrence
  //of the same event with the same foot.

  //Stride period or Cycle time: The period of time from initial contact of one foot to the following
  //initial contact of the same foot, expressed in seconds (s)

  //Gait Stride: The distance from initial contact of one foot to the following initial contact of the same foot.

  //************Relative phase*************************//
  //Relative phase for chosen leg is time measured from beginning of gait period to the moment
  //when considered leg touches the ground normalized to gait period.
  //In some descriptions it is relative time between contacts with ground of two selected legs.
  //Relative phase  = swing / swing + stance

  //************CAL POWER CONSUMPTION******************//

  //ac_motor = in1.at(A_cs); //raw data from LTS 6-NP sensor
  //i_motor = (ac_motor*(5.0/256)-2.5)*6.0/0.625;

  ac_motor = in1.at(A_cs); //raw data from ZAP 25 sensor
  i_motor = (ac_motor*(5.0/256)-2.5)/0.037;
  motor_power_con = i_motor*5; //current*voltage (5v)


  //***********Inclinometer sensors*******************//
  incli_x = in1.at(In_x);
  incli_y = in1.at(In_y);

  //X= 131-incli_x;
  //Y=X*3.1416/180;
  //Z=asin(Y);
  //Final_degree = Z*180/3.1416; // +deg = til to the right, -deg = til to the left


  //***********Body position sensors****************//
  BX_pos_sensor = in1.at(BX_pos);
  BY_pos_sensor = in1.at(BY_pos);
  BZ_pos_sensor = in1.at(BZ_pos);

  //***********Body speed sensors*******************//
  BX_spd_sensor = in1.at(BX_spd);
  BY_spd_sensor = in1.at(BY_spd);
  BZ_spd_sensor = in1.at(BZ_spd);

  //outFilenlc1<<in0.at(R2_fs)<<' '<<in0.at(R1_fs)<<' '<<in0.at(R0_fs)<<' '<<in0.at(L2_fs)<<' '<<in0.at(L1_fs)<<' '<<in0.at(L0_fs)<<' '<<endl;
  //outFilenlc3<<relative_phase.at(CL0_m)<<' '<<relative_phase.at(CR0_m)<<' '<<relative_phase.at(CR1_m)<<' '<<relative_phase.at(CR2_m)<<' '<<relative_phase.at(CL1_m)<<' '<<relative_phase.at(CL2_m)<<' '<<endl;

  outFilegait<<Control_input<<' '<<cpg_output.at(0)<<' '<<cpg_output.at(1)<<' '<<pcpg_output.at(0)<<' '<<pcpg_output.at(1)<<' '<<in0.at(L1_fs)<<' '<<in0.at(L0_fs)<<' '<<input.at(3)<<' '<<input.at(4)<<' '<<endl;
  outFilegait2<<Control_input<<' '<<duty_factor<<' '<<stance_time<<' '<<swing_time<<' '<<stride_period<<' '<<in1.at(BX_spd)<<' '<<in1.at(BY_spd)<<' '<<endl;
  //outFilepower1<<ac_motor<<' '<<i_motor<<' '<<motor_power_con<<' '<<incli_x<<' '<<incli_y<<' '<<in0.at(R0_fs)<<' '<<in0.at(R1_fs)<<' '<<in0.at(R2_fs)<<' '<<in0.at(L0_fs)<<' '<<in0.at(L1_fs)<<' '<<in0.at(L2_fs)<<' '<<' '<<endl;

  outFilenlc_tc <<m.at(TR2_m)<<' '<<m_deg.at(TR2_m)<<' '<<m.at(TR1_m)<<' '<<m_deg.at(TR1_m)<<' '<<m.at(TR0_m)<<' '<<m_deg.at(TR0_m)<<' '<<m.at(TL2_m)<<' '<<m_deg.at(TL2_m)<<' '<<m.at(TL1_m)<<' '<<m_deg.at(TL1_m)<<' '<<m.at(TL0_m)<<' '<<m_deg.at(TL0_m)<<' '<<endl;
  outFilenlc_ctr<<m.at(CR2_m)<<' '<<m_deg.at(CR2_m)<<' '<<m.at(CR1_m)<<' '<<m_deg.at(CR1_m)<<' '<<m.at(CR0_m)<<' '<<m_deg.at(CR0_m)<<' '<<m.at(CL2_m)<<' '<<m_deg.at(CL2_m)<<' '<<m.at(CL1_m)<<' '<<m_deg.at(CL1_m)<<' '<<m.at(CL0_m)<<' '<<m_deg.at(CL0_m)<<' '<<endl;
  outFilenlF_fti<<m.at(FR2_m)<<' '<<m_deg.at(FR2_m)<<' '<<m.at(FR1_m)<<' '<<m_deg.at(FR1_m)<<' '<<m.at(FR0_m)<<' '<<m_deg.at(FR0_m)<<' '<<m.at(FL2_m)<<' '<<m_deg.at(FL2_m)<<' '<<m.at(FL1_m)<<' '<<m_deg.at(FL1_m)<<' '<<m.at(FL0_m)<<' '<<m_deg.at(FL0_m)<<' '<<endl;

  outFilenlc_tc_pre <<m_pre.at(TR2_m)<<' '<<m_pre.at(TR1_m)<<' '<<m_pre.at(TR0_m)<<' '<<m_pre.at(TL2_m)<<' '<<m_pre.at(TL1_m)<<' '<<m_pre.at(TL0_m)<<' '<<endl;
  outFilenlc_ctr_pre <<m_pre.at(CR2_m)<<' '<<m_pre.at(CR1_m)<<' '<<m_pre.at(CR0_m)<<' '<<m_pre.at(CL2_m)<<' '<<m_pre.at(CL1_m)<<' '<<m_pre.at(CL0_m)<<' '<<endl;
  outFilenlF_fti_pre <<m_pre.at(FR2_m)<<' '<<m_pre.at(FR1_m)<<' '<<m_pre.at(FR0_m)<<' '<<m_pre.at(FL2_m)<<' '<<m_pre.at(FL1_m)<<' '<<m_pre.at(FL0_m)<<' '<<endl;


  std::cout<<"i_motor: "<<i_motor<<"ac_motor : "<<ac_motor<<endl;

  int data = -1;

  //		m.at(TR0_m) = 0.44;//data;//m_reflex.at(TR0_m);
  //		m.at(TR1_m) = 0.0;//data;//m_reflex.at(TR1_m);
  //		m.at(TR2_m) = -0.44;//data//m_reflex.at(TR2_m);
  //
  //		m.at(TL0_m) = 0.44;//data;//m_reflex.at(TL0_m);
  //		m.at(TL1_m) = 0.0;//data;//m_reflex.at(TL1_m);
  //		m.at(TL2_m) = -0.44;//data;//m_reflex.at(TL2_m);
  //
  //		m.at(CL0_m) = 0.45;//data;//m_reflex.at(CL0_m);
  //		m.at(CL1_m) = 0.45;//data;//m_reflex.at(CL1_m);
  //		m.at(CL2_m) = 0.45;//data;//m_reflex.at(CL2_m);
  //
  //		m.at(CR0_m) = 0.45;//data;//m_reflex.at(CR0_m);
  //		m.at(CR1_m) = 0.45;//data;//m_reflex.at(CR1_m);
  //		m.at(CR2_m) = 0.45;//data;//m_reflex.at(CR2_m);
  //
  //
  //		m.at(FR0_m) = -0.65;//data;//m_reflex.at(FR0_m);
  //    m.at(FR1_m) = -0.65;//data;//m_reflex.at(FR1_m);
  //	  m.at(FR2_m) = -0.65;//data;//m_reflex.at(FR2_m);
  //
  //		m.at(FL0_m) = -0.65;//data;//m_reflex.at(FL0_m);
  //		m.at(FL1_m) = -0.65;//data;//m_reflex.at(FL1_m);
  //		m.at(FL2_m) = -0.65;//data;//m_reflex.at(FL2_m);

  //m.at(BJ_m) =  m_reflex.at(BJ_m);//(m_reflex.at(TR0_m)-0.35)*5;//


  global_count++;
  std::cout<<atan(40/116)*180/3.1416<<"COUNTER"<<global_count<<"allfoot_off_ground="<<allfoot_off_ground<< "\n";


  if(switchon_purefootsignal)
  {
    std::cout<<"Error = ONLY foot signal"<< "\n";
  }

  return m;

};





#include "muscleRunbotController.h"
using namespace matrix;
using namespace std;

#define STEPSIZE = 0.01
#define RADIUS = 1;

static double DEGTORAD=M_PI/180.0;

MuscleRunbotController::MuscleRunbotController(const std::string& name, const std::string& revision)
: AbstractController (name,revision){

  //Initialization//
  steps = 0;
  pos = 0;
  nSensors = 0;
  nMotors = 0;
  simulatedMass = 0.4;

  angle_hl_low_pass = 90;
  angle_hr_low_pass = 90;
  angle_kl_low_pass = 180;
  angle_kr_low_pass = 180;

  state_motorvolt_hl = 0;
  state_motorvolt_hr = 0;
  state_motorvolt_kl = 0;
  state_motorvolt_kr = 0;
  u_gl = 1;
  u_gr = 0;
  u_al = 0;
  u_ar = 0;


  //Plot data//
  addParameterDef("UBC",&ubc,-0.5,-1.0,1.0,"leaning forward and backwards [1 .. -1]");
  addParameterDef("Mass",&simulatedMass,0.4,0.0,3.0,"simulated mass for muscle model");

  addInspectableValue("speed",&speed,"the speed of runbot, measured in cm/sec");
  addInspectableValue("leftpiezo",&leftpiezo,"leftpiezo");
  addInspectableValue("rightpiezo",&rightpiezo,"rightpiezo");
  addInspectableValue("angle_hl",&angle_hl,"angle_hl");
  addInspectableValue("angle_hr",&angle_hr,"angle_hr");
  addInspectableValue("angle_kl",&angle_kl,"angle_kl");
  addInspectableValue("angle_kr",&angle_kr,"angle_kr");
  addInspectableValue("u_gl",&u_gl,"u_gl");
  addInspectableValue("u_gr",&u_gr,"u_gr");
  addInspectableValue("u_al",&u_al,"u_al");
  addInspectableValue("u_ar",&u_ar,"u_ar");


}


void MuscleRunbotController::init(int sensornumber, int motornumber, RandGen* randGen) {
  nSensors = sensornumber;
  nMotors =  motornumber;
  steps = 0;
  hipPlot.open("feedback.dat");//change this to your directory
  //actualAD.resize(8);

}

int MuscleRunbotController::getSensorNumber() const {
  return nSensors;
}

int MuscleRunbotController::getMotorNumber() const {
  return nMotors;
}


void MuscleRunbotController::step(const sensor* sensors, int sensornumber, motor* motors, int motornumber) {

  //---------------Sensory feedback--------------------------------//
  leftpiezo   = (int)(sensors[5]); // Foot sensor left
  rightpiezo  = (int)(sensors[6]); // Foot sensor right
  angle_hl = sensors[0]; // Joint angle of the hip left
  angle_hr = sensors[1]; // Joint angle of the hip right
  angle_kl = sensors[2]; // Joint angle of the knee left
  angle_kr = sensors[3]; // Joint angle of the knee right


  //speed measurement
  double stepsize = 0.01; //length of control intervalls (both should be obtained automatically somehow..)
  double radius = 1; //armlength in global coordinates
  if ((sensors[7]/10 - pos) < -M_PI)
    speed = speed*0.99+0.01*((sensors[7]/10 + 2*M_PI -pos)*radius/0.01);
  else
    speed = speed*0.99+0.01*((sensors[7]/10 - pos)*radius)/0.01;
  pos = sensors[7]/10;


  //---------------Sensory processing-----------------------------//

  threshold_al = 100; //degrees
  threshold_ar = 100; //degrees

  double gain_low_pass = 0.55;
  angle_hl_low_pass_pre = angle_hl_low_pass;
  angle_hl_low_pass = (1-gain_low_pass)*angle_hl+ angle_hl_low_pass_pre*gain_low_pass;

  angle_hr_low_pass_pre = angle_hr_low_pass;
  angle_hr_low_pass = (1-gain_low_pass)*angle_hr+ angle_hr_low_pass_pre*gain_low_pass;

  angle_kl_low_pass_pre = angle_kl_low_pass;
  angle_kl_low_pass = (1-gain_low_pass)*angle_kl+ angle_kl_low_pass_pre*gain_low_pass;

  angle_kr_low_pass_pre = angle_kr_low_pass;
  angle_kr_low_pass = (1-gain_low_pass)*angle_kr+ angle_kr_low_pass_pre*gain_low_pass;


  if (u_gl==1) {
    if(angle_hl_low_pass>threshold_al) // Stretch receptor sensor left
      u_al = 1;
    else
      u_al=0;
  } else {
    if (((int(angle_hl_low_pass*10))>(int(angle_hl_low_pass_pre*10))) &&
        ((int(angle_hl_low_pass*10))>(int(threshold_al*10)))) {
      u_al=1;
    }
  }
  if (u_gr==1) {
    if(angle_hr_low_pass>threshold_ar) // Stretch receptor sensor right
      u_ar = 1;
    else
      u_ar = 0;
  } else {
    if (((int(angle_hr_low_pass*10))>(int(angle_hr_low_pass_pre*10))) &&
        ((int(angle_hr_low_pass*10))>(int(threshold_ar*10)))) {
      u_ar=1;
    }
  }

    //State machine control///

    // Left leg at front
    if (u_al>0) {                               // Touch the ground of left leg
      if (int(angle_kl)>170) {                        // knee left has to be Straight
        if (abs(leftpiezo-2048)<300) {              // Foot signal ~ ground contact if it touch the ground then enter here!
          u_gl=1;
          u_gr=0;

        }
      }
    }


    // Right leg at front
    if (u_ar>0) {                               // Touch the ground of right leg
      //1)  Enter this loop
      if (int(angle_kr)>170) {                        // knee right has to be Straight
        //2)  Enter this loop
        if (abs(rightpiezo-2048)<300) {             // Foot signal ~ ground contact if it touch the ground then enter here!
          u_gl=0;
          u_gr=1;

        }
      }
    }



  //----------- Available sensors for controller-------------------//

  // leftpiezo; // Foot sensor left = 4096 or > 3000 (off the ground),  2048 or < 3000 (touch the ground)
  // rightpiezo; // Foot sensor right = 4096 or > 3000 (off the ground),  2048 or < 3000 (touch the ground)
  // angle_hl; // Joint angle of the hip left = 120 degree (forward), = 60 degree (backward)
  // angle_hr; // Joint angle of the hip right = 120 degree (forward), = 60 degree (backward)
  // angle_kl; // Joint angle of the knee left = 120 degree (forward), = 60 degree (backward)
  // angle_kr; // Joint angle of the knee right
  // u_gr; // Ground contact sensor of the right foot (preprocessed) = 1 (touch), 0 (off ground)
  // u_gl; // Ground contact sensor of the left foot (preprocessed)
  // u_al; // Stretch receptor sensor left = 1 (if the hip left moves beyond the threshold) otherwise = 0
  // u_ar; // Stretch receptor sensor right = 1 (if the hip right moves beyond the threshold) otherwise = 0


    std::cout<<"leftpiezo : "<<leftpiezo<<"\n"<<std::endl;
    std::cout<<"rightpiezo : "<<rightpiezo<<"\n"<<std::endl;



    //-----Student Modify the control parameters  Your task is here!!!----------------------------//
    //Set either 1.0 or 0.0 below
    // 0.0 = inactive
    // 1.0 = active

    //Hip joint control/////////////////
    if (u_gr==1){
      state_u_hr_em = 0.0;
      state_u_hr_fm = 0.0;

      state_u_hl_em = 0.0;
      state_u_hl_fm = 0.0;
    }

    if (u_gl==1){
      state_u_hr_em = 0.0;
      state_u_hr_fm = 0.0;

      state_u_hl_em = 0.0;
      state_u_hl_fm = 0.0;
    }

    //Knee joint control/////////////////

    // Hip Left move forward beyond the threshold then the knee left is extending
    if (angle_hl_low_pass>threshold_al){
      state_u_kl_em = 0.0;
      state_u_kl_fm = 0.0;
    }

    // Hip Left move backward and below the threshold then the knee left is hold
    if (angle_hl_low_pass<angle_hl_low_pass_pre){ // holding power
      state_u_kl_em = 0.0;
      state_u_kl_fm = 0.0;
    }

    // Hip Left move forward and below the threshold then the knee left is flexing
    if (angle_hl_low_pass>angle_hl_low_pass_pre&&angle_hl_low_pass<threshold_al&&(u_gr==1)) {
      state_u_kl_em = 0.0;
      state_u_kl_fm = 0.0;
    }

    // Hip Right move forward beyond the threshold then Extend the knee left
    if (angle_hr_low_pass>threshold_ar) {
      state_u_kr_em = 0.0;
      state_u_kr_fm = 0.0;
    }
    // Hip Right move backward and below the threshold then the knee left is off
    if (angle_hr_low_pass<angle_hr_low_pass_pre){ // holding power
      state_u_kr_em = 0.0;
      state_u_kr_fm = 0.0;
    }
    // Hip Right move forward and below the threshold then the knee right is flexing
    if (angle_hr_low_pass>angle_hr_low_pass_pre&&angle_hr_low_pass<threshold_ar&&(u_gl==1)) {
      state_u_kr_em = 0.0;
      state_u_kr_fm = 0.0;
    }

    //-----Student Modify the control parameters  ----------------------------//






    //Hip left
    state_motorvolt_hl = 0.25*(state_u_hl_em-state_u_hl_fm); // positive value = move forward, negative = move backward, 0 = center (not moving)

    //Hip right
    state_motorvolt_hr = 0.25*(state_u_hr_em-state_u_hr_fm); // positive value = move forward, negative = move backward, 0 = center (not moving)


    //Knee left
    state_motorvolt_kl = 0.25*(state_u_kl_em-state_u_kl_fm); // positive value = move forward, negative = move backward, 0 = center (not moving)

    //Knee right
    state_motorvolt_kr = 0.25*(state_u_kr_em-state_u_kr_fm); // positive value = move forward, negative = move backward, 0 = center (not moving)



  //write to file,
  hipPlot << state_motorvolt_hr<<" "<<state_motorvolt_hl<<std::endl ;


  motors[0] = state_motorvolt_hl;//Left hip;
  motors[1] = state_motorvolt_hr;//Right hip;
  motors[2] = state_motorvolt_kl;//Left knee;
  motors[3] = state_motorvolt_kr;//Right hip;
  motors[4] = 0.8;//ubc;//Upper body = +1 (lean forward), -1 (lean backward)


}



void MuscleRunbotController::stepNoLearning(const sensor* sensors, int number_sensors, motor* motors, int number_motors) {
}

bool MuscleRunbotController::store(FILE* f) const {
}

bool MuscleRunbotController::restore(FILE* f) {
}


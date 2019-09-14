#include "legWheelBot.h"

//ODE Relative position sensor
#include <ode_robots/relativepositionsensor.h>
//ODE Relative rotationsensor sensor
#include <ode_robots/rotationsensor.h>

// Using namespaces
using namespace osg;
using namespace std;

namespace lpzrobots {

  LegWheelBot::LegWheelBot(const OdeHandle& odeHandle, const OsgHandle& osgHandle,
                             const LegWheelBotConf& conf, const string& name)
    : OdeRobot(odeHandle, osgHandle, name, "2.0"), conf(conf){
  }

  LegWheelBot::~LegWheelBot(){
  }

  void LegWheelBot::placeIntern(const Matrix& pose){
    
    Matrix initialPose;
    
    // Moving robot upward such that the wheel are not stuck on the ground
    float height = max(conf.rightWheel.wheelRadius + conf.rightWheel.spokeLength, 
			    conf.leftWheel.wheelRadius + conf.leftWheel.spokeLength);
    
    initialPose = Matrix::translate(Vec3(0, 0, height) * pose);
    // Creating the robot
    create(initialPose);
  }

  void LegWheelBot::create(const Matrix& pose) {
    /* Creating body */
    // Cylinder geometry primitive as body
    auto body = new Cylinder(conf.bodyRadius, conf.bodyHeight);
    // Setting texture from Image library
    body->setTexture("Images/purple_velour.jpg");
    // Initializing the primitive
    body->init(odeHandle, conf.bodyMass, osgHandle);
    // Setting the pose of the primitive
    body->setPose(pose);
    // Adding the primitive to the list of objects
    objects.push_back(body);
    
    //Attach wheels
    //Adjust motor powers to allow asymetric wheel sizes
    if(conf.rightWheel.spokeLength > conf.leftWheel.spokeLength) {
      conf.rightWheel.wheelMotorMaxSpeed *= conf.leftWheel.spokeLength/(conf.rightWheel.spokeLength);
    } else {
      conf.leftWheel.wheelMotorMaxSpeed *= conf.rightWheel.spokeLength/(conf.leftWheel.spokeLength);
    }
    
    attachWheel(pose, -1, body, conf.rightWheel, "right motor");
    attachWheel(pose, 1, body, conf.leftWheel, "left motor");
    
    //Attach tail
    attachTail(pose, body, conf);
    
    //Add sensors
    auto myAxisSensor = std::make_shared<AxisOrientationSensor>(AxisOrientationSensor::Mode::Axis, Sensor::Dimensions::Y);
    addSensor(myAxisSensor);
    
    auto myPositionSensor = std::make_shared<RelativePositionSensor>(100,1);
    addSensor(myPositionSensor);
  }
  
  void LegWheelBot::attachWheel(const Matrix& bodyPose, 
				    int translationDirection,
				    Cylinder *body,
				    LegWheelBotWheelConf wheelConf,
				    string motorname
 				    ) {
    
    //Must recalculate wheel axis and spoke lengths to avoid segmentation faults
    float C = wheelConf.noOfSpokes * wheelConf.spokeRadius;
    float r = C / (2 * 3.14159265359);
    wheelConf.spokeLength = wheelConf.spokeLength - (r - wheelConf.wheelRadius); 
    wheelConf.wheelRadius = r;
    
    
    /* Creating wheel */
    Primitive* wheel = new Cylinder(wheelConf.wheelRadius, wheelConf.spokeRadius*2);
    
    // Setting texture from Images library
    //wheel->setTexture("Images/chess.rgb");
    wheel->init(odeHandle, wheelConf.wheelMass / (wheelConf.noOfSpokes + 1), osgHandle);
    
    Matrix wheelPose =
      Matrix::rotate(M_PI / 2.0, 0, 1, 0) *
      Matrix::translate(translationDirection * (conf.bodyRadius + wheelConf.spokeRadius*2 / 2.0), .0, .0) *
      bodyPose;
      
    // Setting the pose of the wheel
    wheel->setPose(wheelPose);
    objects.push_back(wheel);
    
    double rotation = (M_PI * 2) / wheelConf.noOfSpokes;
    
    for(int i= 0; i < wheelConf.noOfSpokes; i++) {
      
      //Primitive* spoke = new Cylinder(wheelConf.spokeRadius, wheelConf.spokeLength);
      Primitive* spoke = new Box(wheelConf.spokeRadius, wheelConf.spokeRadius, wheelConf.spokeLength);
      //spoke->setTexture("Images/chess.rgb");
      spoke->init(odeHandle, wheelConf.wheelMass / (wheelConf.noOfSpokes + 1), osgHandle);
      
      //Set the spokes position on the wheel
      Matrix spokePose =
      Matrix::translate(wheelConf.wheelRadius + wheelConf.spokeLength/2 + 0.01,0,0) *
      Matrix::rotate(rotation * i, 0,0,1) *
      wheelPose;
      spokePose = Matrix::rotate(M_PI/2, 0, 1, .0) * spokePose;
      spoke->setPose(spokePose);
      
      objects.push_back(spoke);
      
      FixedJoint* spokeJoint = new FixedJoint(wheel, spoke, Pos(0,0,0));
      spokeJoint->init(odeHandle, osgHandle);
      joints.push_back(spokeJoint);
    }

    // Joining the wheel to the body by a hingejoint
    // the anchor comes from the wheel and the axis of rotation
    auto bodyWheelJoint = new HingeJoint(body, wheel,
                                             wheel->getPosition(),
                                             Axis(0, 0, 1) * wheelPose);
    // Initializing the joint
    bodyWheelJoint->init(odeHandle, osgHandle);
    // Adding the joint to the list of joints
    joints.push_back(bodyWheelJoint);
    
    
    /* Motor */
    // Wheel motor, the OdeHandle, the joint and the maximun
    // power that motor will be used to achieve desired speed
    
    //Alter max motor speed to better do with different wheel sizes
    
    auto motor = std::make_shared<AngularMotor1Axis>(odeHandle, bodyWheelJoint,
                                                     wheelConf.wheelMotorPower);
    motor->setBaseName(motorname);
    motor->setVelovityFactor(wheelConf.wheelMotorMaxSpeed);
    addSensor(motor);
    addMotor(motor);
  }
  
  void LegWheelBot::attachTail(const osg::Matrix& bodyPose, Cylinder *body, LegWheelBotConf conf) { 
    float tailLength = max(conf.leftWheel.spokeLength + conf.leftWheel.wheelRadius, conf.rightWheel.spokeLength + conf.rightWheel.wheelRadius) * 1.5;
    
    Primitive* tail = new Cylinder(conf.bodyHeight/4., tailLength);
    tail -> setTexture("Images/chess.rgb");
    tail -> init(odeHandle, (conf.bodyMass/100.), osgHandle);
    tail->setPose(Matrix::rotate(M_PI / 2, 1,0,0) * 
    Matrix::translate(0,- conf.bodyRadius - 0.5*tailLength - 0.005, 0) * bodyPose);
    objects.push_back(tail);
    
    Primitive* tailBall = new Sphere(conf.bodyHeight/2.);
    tailBall -> setTexture("Images/chess.rgb");
    tailBall -> init(odeHandle, (conf.bodyMass/100.), osgHandle);
    tailBall -> setPose(
      Matrix::translate(0,- conf.bodyRadius -tailLength - conf.bodyHeight/2. - 0.005, 0) * bodyPose
    );
    objects.push_back(tailBall);
    
    FixedJoint* tailBodyJoint = new FixedJoint(body, tail, Pos(0,0,0));
    tailBodyJoint ->init(odeHandle, osgHandle);
    joints.push_back(tailBodyJoint);
    
    BallJoint* ballJoint = new BallJoint(tail, tailBall, tailBall->getPosition());
    ballJoint->init(odeHandle,osgHandle);
    joints.push_back(ballJoint);
  }
}
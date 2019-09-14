namespace lpzrobots
{
	/**
	 * forward declarations
	 */
	class HingeJoint;
	class IRSensor;
	class Joint;
	class OneAxisServo;
	class Primitive;
	class RaySensorBank;
	class SliderJoint;
	class Spring;
	class TwoAxisServo;

	typedef struct
	{
		//	Used for tests
		bool testFemurTibiaSensor;
		bool testTarsusSensor;
		bool testNo;
		bool testHead;
		bool testBody;
		bool testCoxa;
		bool testFemur;
		bool testTibia;
		bool testTarsus;


		//	Body
		double massFront;
		double massRear;
		double massHead;
		double size;
		double scale;
		std::vector<double> headDimension;
		std::vector<double> frontDimension;
		std::vector<double> rearDimension;

		//	Legs
		std::vector<double> coxaLength;
		std::vector<double> coxaRadius;
		std::vector<double> coxaMass;
		std::vector<double> femurLength;
		std::vector<double> femurRadius;
		std::vector<double> femurMass;
		std::vector<double> tibiaLength;
		std::vector<double> tibiaRadius;
		std::vector<double> tibiaMass;
		double tarsusMass;
		std::vector<double> tarsusLength;
		std::vector<double> tarsusRadius;

		//	Motor settings
		double head_Kp;
		double back_Kp;
		std::vector<double> coxa_Kp;
		std::vector<double> femur_Kp;
		std::vector<double> tibia_Kp;
		double tarsus_Kp;

		double head_Kd;
		double back_Kd;
		std::vector<double> coxa_Kd;
		std::vector<double> femur_Kd;
		std::vector<double> tibia_Kd;
		double tarsus_Kd;

		double head_Ki;
		double back_Ki;
		std::vector<double> coxa_Ki;
		std::vector<double> femur_Ki;
		std::vector<double> tibia_Ki;
		double tarsus_Ki;

		double headMaxVel;
		double backMaxVel;
		std::vector<double> coxaMaxVel;
		std::vector<double> femurMaxVel;
		std::vector<double> tibiaMaxVel;
		std::vector<double> footMaxVel;
		double tarsusMaxVel;

		//	Joint limits
		double headJointLimitD;
		double headJointLimitU;
		double backJointLimitD;
		double backJointLimitU;
		double fCoxaJointLimitF;
		double fCoxaJointLimitB;
		double mCoxaJointLimitF;
		double mCoxaJointLimitB;
		double rCoxaJointLimitF;
		double rCoxaJointLimitB;
		double fFemurJointLimitD;
		double fFemurJointLimitU;
		double mFemurJointLimitD;
		double mFemurJointLimitU;
		double rFemurJointLimitD;
		double rFemurJointLimitU;
		double fTibiaJointLimitD;
		double fTibiaJointLimitU;
		double mTibiaJointLimitD;
		double mTibiaJointLimitU;
		double rTibiaJointLimitD;
		double rTibiaJointLimitU;
	}DungBotConf;

}

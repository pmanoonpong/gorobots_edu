#ifndef DUNGBOTSENSORMOTORDEFINITION_H_
#define DUNGBOTSENSORMOTORDEFINITION_H_

namespace DungBotMotorSensor
{
	enum DungBotSensorNames
	{
		// Angle sensors (for actoric-sensor board (new board))
		TR0_as = 0, //Thoracic joint of right front leg
		TR1_as = 1, //Thoracic joint of right middle leg
		TR2_as = 2, //Thoracic joint of right hind leg

		TL0_as = 3, //Thoracic joint of left front leg
		TL1_as = 4, //Thoracic joint of left middle leg
		TL2_as = 5, //Thoracic joint of left hind leg

		CR0_as = 6, //Coxa joint of right front leg
		CR1_as = 7, //Coxa joint of right middle leg
		CR2_as = 8, //Coxa joint of right hind leg

		CL0_as = 9,  //Coxa joint of left front leg
		CL1_as = 10, //Coxa joint of left middle leg
		CL2_as = 11, //Coxa joint of left hind leg

		FR0_as = 12, //Fibula joint of right front leg
		FR1_as = 13, //Fibula joint of right middle leg
		FR2_as = 14, //Fibula joint of right hind leg

		FL0_as = 15, //Fibula joint of left front leg
		FL1_as = 16, //Fibula joint of left middle leg
		FL2_as = 17, //Fibula joint of left hind leg

		BJ_as = 18, //Backbone joint angle
		HJ_as = 19, //Head joint angle

		RPS_REARx = 61, RPS_REARy = 62, RPS_REARz = 63,
		RPS_FRONTx = 64, RPS_FRONTy = 65, RPS_FRONTz = 66,
		RPS_HEADx = 67,	RPS_HEADy = 68,	RPS_HEADz = 69,

		RPS_Leg1Cx = 70,RPS_Leg1Cy = 71,RPS_Leg1Cz = 72,
		RPS_Leg1Fx = 73,RPS_Leg1Fy = 74,RPS_Leg1Fz = 75,
		RPS_Leg1Tix= 76,RPS_Leg1Tiy= 77,RPS_Leg1Tiz= 78,
		RPS_Leg1Tax= 79,RPS_Leg1Tay= 80,RPS_Leg1Taz= 81,

		RPS_Leg2Cx = 82,RPS_Leg2Cy = 83,RPS_Leg2Cz = 84,
		RPS_Leg2Fx = 85,RPS_Leg2Fy = 86,RPS_Leg2Fz = 87,
		RPS_Leg2Tix= 88,RPS_Leg2Tiy= 89,RPS_Leg2Tiz= 90,
		RPS_Leg2Tax= 91,RPS_Leg2Tay= 92,RPS_Leg2Taz= 93,

		RPS_Leg3Cx = 94,RPS_Leg3Cy = 95,RPS_Leg3Cz = 96,
		RPS_Leg3Fx = 97,RPS_Leg3Fy = 98,RPS_Leg3Fz = 99,
		RPS_Leg3Tix= 100,RPS_Leg3Tiy= 101,RPS_Leg3Tiz= 102,
		RPS_Leg3Tax= 103,RPS_Leg3Tay= 104,RPS_Leg3Taz= 105,

		RPS_Leg4Cx = 106,RPS_Leg4Cy = 107,RPS_Leg4Cz = 108,
		RPS_Leg4Fx = 109,RPS_Leg4Fy = 110,RPS_Leg4Fz = 111,
		RPS_Leg4Tix= 112,RPS_Leg4Tiy= 113,RPS_Leg4Tiz= 114,
		RPS_Leg4Tax= 115,RPS_Leg4Tay= 116,RPS_Leg4Taz= 117,

		RPS_Leg5Cx = 118,RPS_Leg5Cy = 119,RPS_Leg5Cz = 120,
		RPS_Leg5Fx = 121,RPS_Leg5Fy = 122,RPS_Leg5Fz = 123,
		RPS_Leg5Tix= 124,RPS_Leg5Tiy= 125,RPS_Leg5Tiz= 126,
		RPS_Leg5Tax= 127,RPS_Leg5Tay= 128,RPS_Leg5Taz= 129,

		RPS_Leg6Cx = 130,RPS_Leg6Cy = 131,RPS_Leg6Cz = 132,
		RPS_Leg6Fx = 133,RPS_Leg6Fy = 134,RPS_Leg6Fz = 135,
		RPS_Leg6Tix= 136,RPS_Leg6Tiy= 137,RPS_Leg6Tiz= 138,
		RPS_Leg6Tax= 139,RPS_Leg6Tay= 140,RPS_Leg6Taz= 141,

		//	Contact sensors on the femur
		R0_CSF = 142,
		R1_CSF = 143,
		R2_CSF = 144,
		L0_CSF = 145,
		L1_CSF = 146,
		L2_CSF = 147,

		//	Contact sensors on the tibia
		R0_CST = 148,
		R1_CST = 149,
		R2_CST = 150,
		L0_CST = 151,
		L1_CST = 152,
		L2_CST = 153,

		//	Contact sensors on the tarsus.
		R0_s0 = 25,
		R0_s1 = 26,
		R0_s2 = 27,
		R0_s3 = 28,
		R0_s4 = 29,
		R0_s5 = 30,

		R1_s0 = 31,
		R1_s1 = 32,//middle right
		R1_s2 = 33,
		R1_s3 = 34,
		R1_s4 = 35,
		R1_s5 = 36,

		R2_s0 = 37,
		R2_s1 = 38,
		R2_s2 = 39,
		R2_s3 = 40,
		R2_s4 = 41,
		R2_s5 = 42,

		L0_s0 = 43,
		L0_s1 = 44,
		L0_s2 = 45,
		L0_s3 = 46,
		L0_s4 = 47,
		L0_s5 = 48,

		L1_s0 = 49,
		L1_s1 = 50,//middle left
		L1_s2 = 51,
		L1_s3 = 52,
		L1_s4 = 53,
		L1_s5 = 54,

		L2_s0 = 55,
		L2_s1 = 56,
		L2_s2 = 57,
		L2_s3 = 58,
		L2_s4 = 59,
		L2_s5 = 60,

        //Body orientation sensors
        BX_ori = 154, // around x axis
        BY_ori = 155, // around y axis
        BZ_ori = 156, // around z axis

		DUNGBOT_SENSOR_MAX = 157, //NEEDS TO BE ONE MORE THAN THE HIGHEST NUMBER
	};

	enum DungBotMotorNames
	{
		TR0_m = 0,	// Upward (+), Downward (-)
		TR1_m = 1,
		TR2_m = 2,
		TL0_m = 3,
		TL1_m = 4,
		TL2_m = 5,

		CR0_m = 6,	// Upward (+), Downward (-)
		CR1_m = 7,
		CR2_m = 8,
		CL0_m = 9,
		CL1_m = 10,
		CL2_m = 11,

		FR0_m = 12,	// Upward (+), Downward (-)
		FR1_m = 13,
		FR2_m = 14,
		FL0_m = 15,
		FL1_m = 16,
		FL2_m = 17,

		BJ_m = 18,  // Upward (+), Downward (-)
		HJ_m = 19,
		//Changing according to the maximum motor number
		DUNGBOT_MOTOR_MAX = 19,
	};
}//	Ending namespace DungBot
#endif

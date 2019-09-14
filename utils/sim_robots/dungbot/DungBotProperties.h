#ifndef DUNGBOTPROPERTIESDEFINITION_H_
#define DUNGBOTPROPERTIESDEFINITION_H_

#include <vector>

namespace Geotropes
{
	/********************************************
	 * Properties of the Geotropes
	 ********************************************/
/*
	double totalLength = 3.75+9.111+10.324;

	std::vector<double> coxaLength {
		2.46037/totalLength,
		2.11888/totalLength,
		4.05514/totalLength };

	std::vector<double> femurLength {
		3.24472/totalLength,
		4.23025/totalLength,
		4.63394/totalLength };

	std::vector<double> tibiaLength {
		4.68943/totalLength,
		3.72093/totalLength,
		5.54793/totalLength };

	double rotationScale = 0.9;
	double coxaFront = 95.7878*rotationScale;
	double coxaMiddle = 116.1153*rotationScale;
	double coxaHind = 160.8514*rotationScale;

	double Ac = 80;
	double Bc = 50;
	double Cc = 50;

	std::vector<double> tcRotation {
		-M_PI / 180.0 * Ac,				 // 70 deg; forward (-) MAX --> normal walking range 60 deg MAX
		 M_PI / 180.0 * (coxaFront-Ac),	 //-70 deg; backward (+) MIN --> normal walking range -10 deg MIN
		-M_PI / 180.0 * Bc,				 // 60 deg; forward (-) MAX --> normal walking range 30 deg MAX
		 M_PI / 180.0 * (coxaMiddle-Bc), // 60 deg; backward (+) MIN --> normal walking range -40 deg MIN
		-M_PI / 180.0 * Cc,				 // 70 deg; forward (-) MAX --> normal walking range 60 deg MAX
		 M_PI / 180.0 * (coxaHind-Cc)	 // 70 deg; backward (+) MIN --> normal walking range -10 deg MIN
	};

	double femur = 90*rotationScale;
	double Af = 30;
	double Bf = 30;
	double Cf = 30;

	std::vector<double> cfRotation {
		 M_PI / 180.0 * Af,
		-M_PI / 180.0 * ( femur-Af ),
		 M_PI / 180.0 * Bf,
		-M_PI / 180.0 * ( femur-Bf ),
		 M_PI / 180.0 * Cf,
		-M_PI / 180.0 * ( femur-Cf )
	};

	double tibia = 170*rotationScale;
	double At = 85;
	double Bt = 85;
	double Ct = 85;

	std::vector<double> ftRotation {
		 M_PI / 180.0 * At,
		-M_PI / 180.0 * ( tibia-At ),
		 M_PI / 180.0 * Bt,
		-M_PI / 180.0 * ( tibia-Bt ),
		 M_PI / 180.0 * Ct,
		-M_PI / 180.0 * ( tibia-Ct )
	};
*/
}//	Ending namespace Geotropes

namespace African
{
}// Ending namespace African

#endif

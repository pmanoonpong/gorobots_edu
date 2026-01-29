#ifndef SDHYS_H
#define SDHYS_H


class SDHys
{
protected:
	double ahys_;
	double whys_;
	double mtanh_;
	double inhys_;
	double ahysold_;

	double inverse_;
	double slopeUP_, slopeDOWN_;

	int update_;
	double output_;

	double tanh_(double x);

public:
	SDHys(double inverse=1., double up=1, double down=1);

	const double EPSILON;

	double step(double input);
	double getOutput();

	void setSlope(double up, double down);
	void setInverse(double inverse);
	void setParams(double w, double m, int update);
};


#endif /* SDHYS_H */

#include "Rotation3D.h"

Rotation3D::Rotation3D() {
	this->setQuaternion(1, 0, 0, 0);
}

Rotation3D::Rotation3D(double q0, double q1, double q2, double q3) {
	this->setQuaternion(q0, q1, q2, q3);
}	
		
Rotation3D::Rotation3D(double yaw, double pitch, double roll) {
	this->setEuler(yaw, pitch, roll);
}

Rotation3D::Rotation3D(const Rotation3D &r) {
	this->setQuaternion(r.q0, r.q1, r.q2, r.q3);
}

void Rotation3D::setQuaternion(double q0, double q1, double q2, double q3) {
	this->q0 = q0;
	this->q1 = q1;
	this->q2 = q2;
	this->q3 = q3;
	this->normalize(); // will fail on input 0,0,0,0
}

void Rotation3D::setRotationAxis(double theta, double axisX, double axisY, double axisZ) {
	double ctheta = cos(theta/2.);
	double stheta = sin(theta/2.);
	
	this->setQuaternion(ctheta, stheta * axisX, stheta * axisY, stheta * axisZ);
}

void Rotation3D::setEuler(double yaw, double pitch, double roll) {
	double cyaw = cos(yaw/2.);
	double syaw = sin(yaw/2.);
	double cpitch = cos(pitch/2.);
	double spitch = sin(pitch/2.);
	double croll = cos(roll/2.);
	double sroll = sin(roll/2.);
	
	this->q0 = croll * cpitch * cyaw + sroll * spitch * syaw;
	this->q1 = sroll * cpitch * cyaw - croll * spitch * syaw;
	this->q2 = croll * spitch * cyaw + sroll * cpitch * syaw;
	this->q3 = croll * cpitch * syaw - sroll * spitch * cyaw;
	this->normalize();
}
		
void Rotation3D::getQuaternion(double &q0, double &q1, double &q2, double &q3) {
	q0 = this->q0;
	q1 = this->q1;
	q2 = this->q2;
	q3 = this->q3;
}

void Rotation3D::getRotationAxis(double &theta, double &axisX, double &axisY, double &axisZ) {
	theta = acos(this->q0)*2.;
	double stheta = sin(theta/2.);
	axisX = this->q1 / stheta;
	axisY = this->q2 / stheta;
	axisZ = this->q3 / stheta;
}

void Rotation3D::getEuler(double &yaw, double &pitch, double &roll) {
	roll = atan2(2.*(this->q0*this->q1 + this->q2*this->q3), 1.-2.*(this->q1*this->q1 + this->q2*this->q2));
	pitch = asin(2.*(this->q0*this->q2 - this->q3*this->q1));
	yaw = atan2(2.*(this->q0*this->q3 + this->q1*this->q2), 1.-2.*(this->q2*this->q2 + this->q3*this->q3)); 
}

void Rotation3D::invert() {
	this->q1 = -this->q1;
	this->q2 = -this->q2;
	this->q3 = -this->q3;
}

Rotation3D Rotation3D::lerp(const Rotation3D &r1, const Rotation3D &r2, double t) {
	return Rotation3D(	(1-t)*r1.q0 + t*r2.q0,
						(1-t)*r1.q1 + t*r2.q1,
						(1-t)*r1.q2 + t*r2.q2,
						(1-t)*r1.q3 + t*r2.q3);
}

Rotation3D Rotation3D::slerp(const Rotation3D &r1, const Rotation3D &r2, double t) {
	double dot = r1.q0*r2.q0 + r1.q1*r2.q1 + r1.q2*r2.q2 + r1.q3*r2.q3;
	double f1;
	double f2;

	if (dot < 0) {
		dot = -dot;
		
		if(dot > 0.98) 
			return lerp(Rotation3D(-r1.q0, -r1.q1, -r1.q2, -r1.q3), r2, t);
		
		double omega = acos(dot);
		f1 = -sin(omega*(1-t))/sin(omega);
		f2 = sin(omega*t)/sin(omega);
	} else {
		if(dot > 0.98) 
			return lerp(r1, r2, t);
		
		double omega = acos(dot);
		f1 = sin(omega*(1-t))/sin(omega);
		f2 = sin(omega*t)/sin(omega);
	}
	return Rotation3D(	f1*r1.q0 + f2*r2.q0,
						f1*r1.q1 + f2*r2.q1,
						f1*r1.q2 + f2*r2.q2,
						f1*r1.q3 + f2*r2.q3);
}
		
void Rotation3D::normalize() {
	double length = sqrt(this->q0*this->q0 + this->q1*this->q1 + this->q2*this->q2 + this->q3*this->q3);
	this->q0 /= length;
	this->q1 /= length;
	this->q2 /= length;
	this->q3 /= length;
}

const Rotation3D Rotation3D::operator *(const Rotation3D &r) const
{
	return Rotation3D(	this->q0*r.q0 - this->q1*r.q1 - this->q2*r.q2 - this->q3*r.q3,
			this->q0*r.q1 + this->q1*r.q0 + this->q2*r.q3 - this->q3*r.q2,
			this->q0*r.q2 - this->q1*r.q3 + this->q2*r.q0 + this->q3*r.q1,
			this->q0*r.q3 + this->q1*r.q2 - this->q2*r.q1 + this->q3*r.q0);
}

void Rotation3D::apply(double &x, double &y, double &z) {
	double newx = (1.-2.*(q2*q2+q3*q3)) * x + 2.*(q1*q2-q0*q3) * y + 2.*(q0*q2+q1*q3) * z;
	double newy = 2.*(q1*q2+q0*q3) * x + (1.-2.*(q1*q1+q3*q3)) * y + 2.*(q2*q3-q0*q1) * z;
	double newz = 2.*(q1*q3-q0*q2) * x + 2.*(q0*q1+q2*q3) * y + (1.-2.*(q1*q1+q2*q2)) * z;
	
	x = newx;
	y = newy;
	z = newz;
}


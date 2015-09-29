#ifndef _ROTATION3D_H_
#define _ROTATION3D_H_

#include <cmath>
#include <string>

//Class to work with 3D rotations. The internal representation is done with quaternions, but there are conversion functions
//to Euler angles or angle-axis representation.
//All angles are right handed
class Rotation3D {
	
	public:
		//Init default quaternion (no rotation)
		Rotation3D();
		//Construct rotation from quaternion values
		//q0: Angle of rotation
		//q1: x coordinate of rotation axis
		//q2: y coordinate of rotation axis
		//q3: z coordinate of rotation axis
		Rotation3D(double q0, double q1, double q2, double q3);
		//Construct rotation from euler angles
		//yaw: Rotation around z-axis of object
		//pitch: Rotation around y axis of object
		//roll: Rotation around x axis of object
		Rotation3D(double yaw, double pitch, double roll);
		//Copy constructor
		Rotation3D(const Rotation3D &r);
		
		//Set rotation through quaternion values (see corresponding constructor)
		void setQuaternion(double q0, double q1, double q2, double q3);
		//Set rotation through angle axis representation (see corresponding constructor)
		void setRotationAxis(double theta, double axisX, double axisY, double axisZ);
		//Set rotation through Euler values (see corresponding constructor)
		void setEuler(double yaw, double pitch, double roll);
		
		//Return quaternion of rotation
		void getQuaternion(double &q0, double &q1, double &q2, double &q3);
		//Return angle-axis representation of rotation
		void getRotationAxis(double &theta, double &axisX, double &axisY, double &axisZ);
		//Return euler angles of rotation
		void getEuler(double &yaw, double &pitch, double &roll);
		//Invert rotation
		void invert();
		//Concatenate two rotations, that is multiply them. Use this as standard overload to '*' operator
		const Rotation3D operator *(const Rotation3D &r) const;
		//Apply rotation to a 3D vector, thus rotating the vector
		void apply(double &x, double &y, double &z);
		//Normalize quaternion
		void normalize();
		
	private:
		//Quaternion representation of rotation
		double q0;  //Angle of rotation
		double q1;  //x coordinate of rotation axis
		double q2;  //y coordinate of rotation axis
		double q3;  //z coordinate of rotation axis
		
		//Interpolation of rotations (not tested yet, thats why private)
		static Rotation3D lerp(const Rotation3D &r1, const Rotation3D &r2, double t);
		static Rotation3D slerp(const Rotation3D &r1, const Rotation3D &r2, double t);
};


#endif /* _ROTATION3D_H_ */

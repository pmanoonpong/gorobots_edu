#include <iostream>
#include <sstream>
#include "laserScanner.h"

using namespace std;
using namespace URG;

int main(int argc, char *argv[])
{
	//Init URG class and try to connect LRF
	URGLaser urg(true);
	//Check if connection attempt was successfull
	if(urg.connected){
		cout << "Acquiring data"<<endl;
		urg.updateData(); //Take measurement
		cout << "Write data to file"<<endl;
		urg.writeToFile("testLRF.dat");
		cout << "Printing measurement:"<<endl;
		vector<long> data = urg.data; //Store measurement
		for(int i = 0; i < (int)data.size(); i++)
			cout << urg.index2deg(i)<< " "<< data[i]<<endl;
	}
	else
		cout<<"Could not connect LRF. Did you run the program with sudo?"<<endl;
	
}

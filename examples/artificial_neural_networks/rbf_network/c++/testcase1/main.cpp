#include <iostream>
#include <vector>
#include "test_case_1_sinwave.h"



using namespace RBFnetwork;


int main() {
	

	TestCase1 tc1;
	int i;
    cin>>i;
	/*

    //set-ups for the configurations of the RBF network

    //set-up the configurations for each dimension
    std::vector<InputConfigs*> ICs;

    InputConfigs IC1(3.14, -3.14, 10);

    //InputConfigs IC2;
    //IC2.max_value = 1;
    //IC2.min_value = -1;
    //IC2.number_dim_centers = 3;

    ICs.push_back(&IC1);
    //ICs.push_back(&IC2);


    RBFNetwork RBFnet(1, 1, ICs);
    RBFnet.print_RBF_hidden_neurons_information();

	double error = RBFnet.learn(learn_units, 0.001, 1000, 0.5);
	cout<<"error"<<error;

	int i;
    cin>>i;

    /*std::vector<double> temp1;
    for (int i=0; i<10; i++)
        temp1.push_back(i);

    std::vector<double> temp2;
    temp2 = temp1;

    temp1[3] = 100;
    temp2[2] = 99999;

    cout<<"enter any number to exit..."<<endl;
    int i;
    cin>>i;

    for (int i=0; i<10; i++) {
        std::cout<<"temp1["<<i<<"] = "<<temp1[i]<<std::endl;
    }
    for (int i=0; i<10; i++) {
        std::cout<<"temp2["<<i<<"] = "<<temp2[i]<<std::endl;
    }*/

	
    return 0;
}

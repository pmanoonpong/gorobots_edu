#include "test_case_1_sinwave.h"
#include <iostream>
#include "math.h"


TestCase1::TestCase1() {
	
	std::vector<LearningUnit*> learn_units_train;
	std::vector<LearningUnit*> learn_units_test;

	//collect sin wave data
    //double dataset [number_of_data_points][2];
	LearningUnit* ln;
    for (int i=0; i<100; i++) {
        double in = (((double)rand()/RAND_MAX)*6) - 3; //-3 --> +3
        double out = sin(in); //-3 --> +3

		ln = new LearningUnit();
		ln->inputs.push_back(in);
		ln->outputs.push_back(out);

		if (i > 80)
			learn_units_test.push_back(ln);
		else
			learn_units_train.push_back(ln);
    }




	vector<InputConfigs*> inputs_confis;

	//set up configuration for each dimension
	InputConfigs ic1(3.14 //  max number
		, -3.14 //  min number
		, 10    // number of centers on this dimension
		);
	inputs_confis.push_back(&ic1); //add it to the list of dimensions

	//make an instance of the network and pass the configurations
	RBFNetwork RBFnet(1, 1, inputs_confis);
	RBFnet.print_RBF_hidden_neurons_information();
	cout<<"/////////////////////////////////////////////////"<<endl;
	cout<<"learning:"<<endl;
	cout<<"/////////////////////////////////////////////////"<<endl;
	double final_error = RBFnet.learn(learn_units_train, 0.05, 1000, 0.5, true);
	cout<<"/////////////////////////////////////////////////"<<endl;
	cout<<"testing:"<<endl;
	cout<<"/////////////////////////////////////////////////"<<endl;
	//////test results on the test data
	for (int i=0; i< learn_units_test.size(); i++)
		cout<<"Network output = "<<RBFnet.get_output(learn_units_test[i]->inputs)[0]
		<<", Desired output = "<<learn_units_test[i]->outputs[0]<<endl;
}

#include"test_case_2_multiplication.h"
#include<iostream>


TestCase2::TestCase2() {
	
	std::vector<LearningUnit*> learn_units_train;
	std::vector<LearningUnit*> learn_units_test;

	//collect sin wave data
    //double dataset [number_of_data_points][2];
	LearningUnit* ln;
    for (int i=0; i<200; i++) {
        double in1 = (((double)rand()/RAND_MAX)*20) - 10; //-3 --> +3
		double in2 = (((double)rand()/RAND_MAX)*20) - 10; //-3 --> +3
        double out = in1*in2; //-3 --> +3

		ln = new LearningUnit();
		ln->inputs.push_back(in1);
		ln->inputs.push_back(in2);
		ln->outputs.push_back(out);

		if (i > 160)
			learn_units_test.push_back(ln);
		else
			learn_units_train.push_back(ln);
    }




	vector<InputConfigs*> inputs_confis;

	//set up configuration for each dimension
	InputConfigs ic1(10 // max number
		, -10 // min number
		, 10    // number of centers on this dimension
		);
	InputConfigs ic2(10 // max number
		, -10 // min number
		, 10    // number of centers on this dimension
		);

	inputs_confis.push_back(&ic1); //add it to the list of dimensions
	inputs_confis.push_back(&ic2); //add it to the list of dimensions

	//make an instance of the network and pass the configurations
	RBFNetwork RBFnet(2, 1, inputs_confis);
	RBFnet.print_RBF_hidden_neurons_information();
	cout<<"/////////////////////////////////////////////////"<<endl;
	cout<<"learning:"<<endl;
	cout<<"/////////////////////////////////////////////////"<<endl;
	double final_error = RBFnet.learn(learn_units_train, 0.05, 400, 0.6, true);
	cout<<"/////////////////////////////////////////////////"<<endl;
	cout<<"testing:"<<endl;
	cout<<"/////////////////////////////////////////////////"<<endl;
	//////test results on the test data
	for (int i=0; i< learn_units_test.size(); i++)
		cout<<"Network output = "<<RBFnet.get_output(learn_units_test[i]->inputs)[0]
		<<", Desired output = "<<learn_units_test[i]->outputs[0]<<endl;
}
///Author: Bassel Zeidan

#include <iostream>
#include "rbf_network.h"
#include <math.h>

namespace RBFnetwork {

////////////////////////////////////////////////////////////////////////////////
//                            InputConfigs class                              //
////////////////////////////////////////////////////////////////////////////////
InputConfigs::InputConfigs(double dim_max_value
		, double dim_min_value
		, int number_of_dimension_centers) {
	max_value = dim_max_value;
	min_value = dim_min_value;
	number_dim_centers = number_of_dimension_centers;
}

void InputConfigs::calculate_dimension_data() {
	dim_variance = (max_value - min_value)
            / (2.0*number_dim_centers);
}

double InputConfigs::get_dimension_variance() {
	return dim_variance;
}

int InputConfigs::get_dimension_number_of_centers() {
	return number_dim_centers;
}

double InputConfigs::get_dimension_max_value() {
	return max_value;
}

double InputConfigs::get_dimension_min_value() {
	return min_value;
}
////////////////////////////////////////////////////////////////////////////////
//                            Neuron class                                        //
////////////////////////////////////////////////////////////////////////////////
double Neuron::get_output(std::vector<double> input) {return 0;}

////////////////////////////////////////////////////////////////////////////////
//                            RBFNeuron class                                        //
////////////////////////////////////////////////////////////////////////////////
RBFneuron::RBFneuron(std::vector<double> center_position, std::vector<double> variance_dim) {
    center = center_position;
    variance = variance_dim;
}

double RBFneuron::get_output(std::vector<double> input) {
    std::vector<double> distance;
    double ac = 0; //the activation of this neuron
    for (int i=0; i<center.size(); i++) {//loop over the number of dimensions
        double dis = input[i] - center[i]; //dis = input_i - center_i
        ac += (dis*dis)/(variance[i]); //variance = sigma^2
    }
    return (double)(exp(-ac/2.0));//the final activation
}

////////////////////////////////////////////////////////////////////////////////
//                            OutputNeuron class                                    //
////////////////////////////////////////////////////////////////////////////////
OutputNeuron::OutputNeuron(int number_of_inputs) {
    for (int i=0; i<number_of_inputs; i++)
        weights.push_back(0.0);
}

double OutputNeuron::get_output(std::vector<double> input) {
    double sum = 0;
    for (int i=0; i<input.size(); i++)
        sum += input[i]*weights[i];
    return sum;
}

int OutputNeuron::get_number_of_weights() {
	return weights.size();
}

////////////////////////////////////////////////////////////////////////////////
//                            Layer class                                            //
////////////////////////////////////////////////////////////////////////////////
void Layer::add_neuron(Neuron* neuron) {
    neurons.push_back(neuron);
}

std::vector<double> Layer::get_last_outputs() {
	return last_outputs;
}

void Layer::get_all_neurons(std::vector<Neuron*> all_neurons) {
	neurons = all_neurons;
}

std::vector<double> Layer::get_output(std::vector<double> inputs
	, bool normalized_output
	, bool save_layer_outputs) {
    std::vector<double> outputs;
	last_outputs.clear();

    //////////////////////////////////
    //collect neurons' output
    double sum=0;
    for (int i=0; i<neurons.size(); i++) {
        double neuron_output = neurons[i]->get_output(inputs);
        outputs.push_back(neuron_output);
        if (normalized_output)
            sum += neuron_output;
    }

    //////////////////////////////////
    //Normalize outputs if needed
    if (normalized_output) {
        for (int i=0; i<neurons.size(); i++)
            outputs[i] /= sum;
    }

	////////////////////////////
	//save outputs
	if (save_layer_outputs)
		last_outputs = outputs;

    return outputs;
}

int Layer::get_number_of_neurons() {
    return neurons.size();
}

////////////////////////////////////////////////////////////////////////////////
//                            RBF neuron class                                  //
////////////////////////////////////////////////////////////////////////////////
void RBFNetwork::add_further_neurons(int dim_index
        , std::vector<Neuron*> &final_array
        , std::vector<double> temp_centers_stack
		, std::vector<double> temp_variances_stack
        , std::vector<InputConfigs*> input_configs_array) {
    //this level handle one dimension

    //recursion stop condition
    if (dim_index > (input_configs_array.size() - 1)) {
		final_array.push_back(new RBFneuron(temp_centers_stack, temp_variances_stack));
        return;
	}

	for (int i=0; i<input_configs_array[dim_index]->get_dimension_number_of_centers(); i++) {
		double center  = 
			(input_configs_array[dim_index]->get_dimension_min_value()
			+ input_configs_array[dim_index]->get_dimension_variance())
                    +(i*2.0*input_configs_array[dim_index]->get_dimension_variance());
		//add a center
        std::vector<double> centers_level_stack = temp_centers_stack;
		centers_level_stack.push_back(center);
		//add a variance value related to that dimension
		std::vector<double> variances_level_stack = temp_variances_stack;
		variances_level_stack.push_back(input_configs_array[dim_index]->get_dimension_variance());
		//call again
        add_further_neurons(dim_index+1, final_array, centers_level_stack, variances_level_stack, input_configs_array);
    }
}

void RBFNetwork::initialize_hidden_layer(std::vector<InputConfigs*> input_configs_array) {
    //initialize hidden layer
    std::vector<Neuron*> final_hidden_neurons;
    std::vector<double> temp1, temp2;
    add_further_neurons(0, final_hidden_neurons, temp1, temp2, input_configs_array);
	hidden_layer.get_all_neurons(final_hidden_neurons);
}

void RBFNetwork::initialize_output_layer() {
    for (int i=0; i<number_outputs; i++)
        output_layer.add_neuron(new OutputNeuron(hidden_layer.get_number_of_neurons()));
}

RBFNetwork::RBFNetwork(int number_of_inputs, int number_of_outputs
		, std::vector<RBFneuron*> hidden_neurons) {
	std::vector<Neuron*> hidden_neurons_temp;
	for (int i=0; i < hidden_neurons.size(); i++) {
		hidden_neurons_temp.push_back((Neuron*) hidden_neurons[i]);
	}
	hidden_layer.get_all_neurons(hidden_neurons_temp);
    initialize_output_layer();
}

RBFNetwork::RBFNetwork(int number_of_inputs, int number_of_outputs
		, std::vector<InputConfigs*> input_configs_array) {
    number_inputs = number_of_inputs;
    number_outputs = number_of_outputs;
	//calculate variance for each dimension
	for (int i=0; i<input_configs_array.size(); i++)
		input_configs_array[i]->calculate_dimension_data();
	//initialization / hidden + output layers
    initialize_hidden_layer(input_configs_array);
    initialize_output_layer();
}

std::vector<double> RBFNetwork::get_output(std::vector<double> inputs) {
	//checking
	if (inputs.size() != number_inputs) {
		std::cout<<"Number of outputs doesn't match!!!";
		std::vector<double> temp;
		return temp;
	}
    //get the normalized output of the first layer (hidden layer) of the network
    std::vector<double> first_layer_outputs = hidden_layer.get_output(inputs, true, true);
    //get the output of the second layer (hidden layer) of the network
    std::vector<double> second_layer_outputs = output_layer.get_output(first_layer_outputs, false, false);
    return second_layer_outputs;
}



using namespace std;

void RBFNetwork::print_RBF_hidden_neurons_information() {
    for (int i=0; i<hidden_layer.neurons.size(); i++) {
		RBFneuron* RBF = dynamic_cast<RBFneuron*>(hidden_layer.neurons[i]);

		cout<<"RBF["<<i<<"]"<<endl<<"///////////////////////////////////"<<endl;
        for (int j=0; j<RBF->center.size(); j++) {
            cout<<"c["<<j<<"] = "<<RBF->center[j];
			cout<<std::endl;
        }
        
    }
}

std::vector<double> RBFNetwork::get_last_hidden_layer_outputs() {
	return hidden_layer.get_last_outputs();
}

//returns the overall squared error only when learning mode is 1
//0 otherwise
double RBFNetwork::learn_one_step(LearningUnit* learning_unit
		, double learning_rate
		, int learning_mode) {
	double final_error = 0;
	//get current output of the network according to the inputs
	std::vector<double> outputs = get_output(learning_unit->inputs);

	std::vector<double> errors;
	if (learning_mode == 1) {
		//calculate errors
		for (int i=0; i<output_layer.neurons.size(); i++) {
			double current_error = learning_unit->outputs[i] //the desired output
									- outputs[i];
			errors.push_back(current_error);
			current_error *= current_error;
			final_error += current_error;
		}
	} else if (learning_mode == 2) {
		errors = learning_unit->errors;
	}

	//update weights
	for (int i=0; i<output_layer.neurons.size(); i++) {
		OutputNeuron* o = dynamic_cast<OutputNeuron*>(output_layer.neurons[i]);
		for (int j=0; j<o->get_number_of_weights(); j++) {
			//apply update rule
			o->weights[j] += learning_rate*errors[i]*hidden_layer.get_last_outputs()[j];
		}
	}
	return final_error;
}

//returns the overall error (for all samples) only when learning mode is 1
//0 otherwise
double RBFNetwork::learn_one_epoch(std::vector<LearningUnit*> learning_units
				, double learning_rate
				, int learning_mode) {
	double final_error = 0;
	for (int i=0; i<learning_units.size(); i++) 
		final_error += learn_one_step(learning_units[i], learning_rate, learning_mode);
	return final_error/learning_units.size();
}

//this function is always take learning mode 1
double RBFNetwork::learn(std::vector<LearningUnit*> learning_units
	, double max_error
	, int max_number_of_iterations
	, double learning_rate
	, bool print_error_during_learning
	) {
	int iterations_counter = 0;
	double error = 999999999999.9;//big number
	while ((iterations_counter < max_number_of_iterations) && (error > max_error)) {
		error = learn_one_epoch(learning_units, learning_rate, 1);
		if (print_error_during_learning)
			std::cout<<"learning error="<<error<<", Iteration = "<<iterations_counter<<std::endl;
		iterations_counter++;
	}
	return error;
}

}

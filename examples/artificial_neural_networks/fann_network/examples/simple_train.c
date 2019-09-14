/*
Fast Artificial Neural Network Library (fann)
Copyright (C) 2003-2012 Steffen Nissen (sn@leenissen.dk)

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

//#include "fann.h"
//#include "fann_cpp.h"
#include "floatfann.h"

int main()
{
	//FANN::neural_net net;
	const unsigned int num_input = 2;
	const unsigned int num_output = 1;
	const unsigned int num_layers = 3;
	const unsigned int num_neurons_hidden = 3;
	const float desired_error = (const float) 0.0001;
	const unsigned int max_epochs = 1000000;
	const unsigned int epochs_between_reports = 1000;
	const float learning_rate =(const float) 0.5;

	struct fann *ann = fann_create_standard(num_layers, num_input, num_neurons_hidden, num_output);

	fann_set_activation_function_hidden(ann, FANN_SIGMOID_SYMMETRIC);
	fann_set_activation_function_output(ann, FANN_SIGMOID_SYMMETRIC);
	fann_set_training_algorithm (ann, FANN_TRAIN_BATCH);
	//fann_set_training_algorithm (ann, FANN_TRAIN_QUICKPROP);
	//fann_set_training_algorithm (ann, FANN_TRAIN_INCREMENTAL);
	fann_set_learning_rate(ann, learning_rate);

	fann_train_on_file(ann, "xor.data", max_epochs, epochs_between_reports, desired_error);

	fann_save(ann, "xor_float.net");

	fann_destroy(ann);

	return 0;
}

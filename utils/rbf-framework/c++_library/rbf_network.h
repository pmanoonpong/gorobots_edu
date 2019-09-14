/// Author: Bassel Zeidan
/// Extended by Timon Tomas (2016)

#ifndef RBF_NETWORK_
#define RBF_NETWORK_

#include <vector>
#include <string>
#include <sstream>
#include <fstream>

namespace RBFnetwork{

    class InputConfigs {

        private:
            double dim_variance;

            double max_value;
            double min_value;
            unsigned int number_dim_centers; //number of centers (basis) on this input dimension

        public:
            InputConfigs( 
                    double dim_max_value, 
                    double dim_min_value, 
                    int number_of_dimension_centers );
            void calculate_dimension_data();
            double get_dimension_variance();
            unsigned int get_dimension_number_of_centers();
            double get_dimension_max_value();
            double get_dimension_min_value();

    };

    class LearningUnit {

        public:
            std::vector<double> inputs;
            std::vector<double> outputs;
            std::vector<double> errors;

    };

    class Neuron {

        public:

            virtual double get_output( std::vector<double> input );

    };

    class RBFneuron : public Neuron {
    //private:

        public:
            //the center position in the input space
            std::vector<double> center;
            std::vector<double> variance;
            int i;

            RBFneuron( 
                    std::vector<double>& center_position, 
                    std::vector<double>& variance_dim );
            double get_output( std::vector<double> input );

    };

    class OutputNeuron : public Neuron {

        public :
            std::vector<double> weights;

            OutputNeuron( unsigned int number_of_inputs );
            OutputNeuron( std::vector<double>& w );
            double get_output( std::vector<double> input );
            unsigned int get_number_of_weights();

    };

    class Layer {

        private:
            std::vector<double> last_outputs;

        public:
            std::vector<Neuron*> neurons;

            void add_neuron( Neuron* neuron );
            void get_all_neurons( std::vector<Neuron*> all_neurons );
            std::vector<double> get_output(
                    std::vector<double> inputs, 
                    bool normalized_output, 
                    bool save_layer_outputs );
            int get_number_of_neurons();
            std::vector<double> get_last_outputs();

    };


    class RBFNetwork {

        private:
            unsigned int number_inputs;
            unsigned int number_outputs;
            Layer hidden_layer;
            Layer output_layer;

            void initialize_hidden_layer( std::vector<InputConfigs*> input_configs_array );
            void initialize_output_layer();
            void add_further_neurons( 
                    unsigned int dim_index, 
                    std::vector<Neuron*> &final_array, 
                    std::vector<double> temp_centers_stack ,
                    std::vector<double> temp_variances_stack, 
                    std::vector<InputConfigs*> input_configs_array );

        public:
            RBFNetwork(
                    unsigned int number_of_inputs, 
                    unsigned int number_of_outputs, 
                    std::vector<InputConfigs*> input_configs_array );

            RBFNetwork(
                    unsigned int number_of_inputs, 
                    unsigned int number_of_outputs, 
                    std::vector<RBFneuron*> hidden_neurons );

            RBFNetwork( std::string path );

            //learning input-output / one step
            double learn_one_step(
                    LearningUnit* learning_unit, 
                    double learning_rate, 
                    int learning_mode );

            double learn_one_epoch(
                    std::vector<LearningUnit*> learning_units,
                    double learning_rate,
                    int learning_mode );

            //this function always runs in learning mode 1
            double learn( 
                    std::vector<LearningUnit*> learning_units, 
                    double max_error, 
                    int max_number_of_iterations, 
                    double learning_rate, 
                    bool print_error_during_learning );
            
            int* getNeuronCount();

            std::vector<double> get_output( std::vector<double> inputs );
            std::vector<double> get_last_hidden_layer_outputs();
            void print_RBF_hidden_neurons_information();
            
            std::vector<double> split( std::string );

            void save( std::string path );
            void load( std::string path );

    };

}

#endif

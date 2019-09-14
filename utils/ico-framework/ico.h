/** \file ico.h
 * ICO class library containing the definitions.
 */

#ifndef ICO_H_
#define ICO_H_

/** \file ann.h
 * Artificial neural network class library used for implementing the ICO class.
 */
#include <utils/ann-framework/ann.h>

class ANN;

/**
 * Definition of the ICO class.
 *
 * This class is based on the artificial neural network class (ANN). It defines and modifies the parameters and methods needed for implementing an ICO learning class out of that one.
 */
class ICO : public ANN{
  public:
    /**
     * Class default constructor.
     */
    ICO();

    /**
     * Class basic constructor.
     *
     * This constructor creates a basic ICO network with 2 inputs as shown below. Being P the predictive input
     * , R the reflexive input and O the output.
     *
     * \dot
     * digraph A{
     *  rankdir=LR
     *  splines=line
     *
     *  node[fixedsize=true];
     *
     *  subgraph cluster0 {
     *    color=white;
     *    node [style=solid, color=blue4, shape=circle];
     *    P R;
     *    label = "Input layer";
     *  }
     *
     *  subgraph cluster1 {
     *    color=white;
     *    node [style=solid, color=red2, shape=circle];
     *    O;
     *    label = "Output layer";
     *  }
     *
     *  P -> O [label="0.0"];
     *  R -> O [label="1.0"];
     * }
     * \enddot
     *
     * @param rate The learning rate of the network.
     */
    ICO(const double& rate);

    /**
     * Class constructor for bigger networks.
     *
     * This constructor allows to create an ICO network with as many predictive inputs as desired
     * and a reflexive input as shown below. P1 to Pn are the predictive inputs, R the reflexive
     * input and O the output.
     *
     * \dot
     * digraph A{
     *  rankdir=LR
     *  splines=line
     *
     *  node[fixedsize=true];
     *
     *  subgraph cluster0 {
     *    color=white;
     *    node [style=solid, color=blue4, shape=circle];
     *    P1 ··· Pn R;
     *    label = "Input layer";
     *  }
     *
     *  subgraph cluster1 {
     *    color=white;
     *    node [style=solid, color=red2, shape=circle];
     *    O;
     *    label = "Output layer";
     *  }
     *
     *  P1 -> O [label="0.0"];
     *  Pn -> O [label="0.0"];
     *  R -> O [label="1.0"];
     * }
     * \enddot
     *
     * @param neuronNumber Number of predictive neurons in the network. It has to be equal or bigger than 1.
     * @param rate The learning rate of the network.
     */
    ICO(const int& neuronNumber, const double& rate);

    /**
     * Default class destructor.
     */
    virtual ~ICO();

    /**
     * Redefinition of the ANN::step method for the class.
     *
     * This method calls the methods ANN::updateActivities, ANN::updateOutputs and ICO::updateWeights in this order.
     */
    virtual void step();

    void stepNoLearning();

    /**
     * Function for setting the input of the reflexive neuron.
     *
     * This function has to be called before the ICO::step function to set the reflexive neuron value.
     *
     * @param value Double containing the input value for the reflexive neuron.
     */
    void setReflexiveNeuronInput(const double& value);

    /**
     * Function for setting the input of a single predictive neuron.
     *
     * This function sets the input value for an individual predictive neuron.
     * This function has to be called before the ICO::step function for every predictive neuron in the network.
     *
     * @param index Number of the predictive neuron whose input is set. Starts in 0.
     * @param value Double containing the input value for the predictive neuron.
     */
    void setPredictiveNeuronInput(const int& index, const double& value);

    /**
     * Function for setting the input of the predictive neurons of the network.
     *
     * This function sets the input value of all the predictive neurons.
     * This function has to be called before the ICO::step function.
     *
     * @param values Vector of doubles containing the inputs for all the predictive neurons. It has to be the same size as predictive neurons contains the network.
     */
    void setPredictiveNeuronInput(std::vector<double>& values);

    void setReflexiveNeuronWeight(const double& weight);

    void setPredictiveNeuronWeight(const int& index, const double& weight);

    void setPredictiveNeuronWeight(std::vector<double>& weights);

    /**
     * Function to get the output of the system.
     *
     * This function returns the output of the network.
     *
     * @return Double containing the output of the network.
     */
    const double& getOutputNeuronOutput();

    /**
     * Function to retrieve the synaptic weights of the neurons.
     * @return Vector of doubles containing the weights, starting with the reflexive one.
     */
    std::vector<double> getWeights();

    /**
     * Redefinition of the ANN:dumpWeights method for the class.
     *
     * This method returns a string containing the weights for the predictive signals. The reflexive one is removed to save space in terminal.
     * @return String with the predictive neuron weights.
     */
    std::string dumpWeights();

  protected:
    /**
     * Redefinition of the ANN::updateWeights method for the class.
     *
     * This method updates the weights of the network based on the correlation in between the reflexive input and the predictive inputs.
     * Only the weights of the predictive inputs are changed, while the reflexive one stays in 1.0 forever.
     */
    virtual void updateWeights();

    /**
     * Method that sets the input value of the neurons.
     *
     * Auxiliar method that sets the inputs of the neurons. If the index is 0 it sets the input of the reflexive neuron. Any other value sets the corresponding predictive neuron.
     *
     * @param index Index of the neuron whose input is set. 0 for the reflexive.
     * @param value Double containing the input value for the neuron.
     */
    void setInputNeuronInput(const int& index, const double& value);


  private:
    std::vector<Neuron*> inputNeurons; //!< Vector containing pointers to the input neurons of the network, both the predictives and the reflexive.
    typedef std::vector <Neuron*> NeuronList; //!< Redefinition of the type of vector of pointers to Neuron from ANN. It is needed since it is private.
    typedef std::vector <ANN*> AnnList; //!< Redefinition of the type of vector of pointers to ANN from ANN. It is needed since it is private.
    Neuron* outputNeuron; //!< Pointer to the output neuron of the network.
    double oldReflexiveInput; //!< Old input value of the reflexive neuron.
    double rate_ico; //!< Learning rate of the network.
};

#endif /* ICO_H_ */

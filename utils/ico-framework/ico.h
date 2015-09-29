#ifndef ICO_H_
#define ICO_H_

#include <utils/ann-framework/ann.h>

class ANN;

class ICO : public ANN{
  public:
    ICO();
    ICO(const double& value);
    ICO(const int& neuronNumber, const double& rate);
    virtual ~ICO();
    virtual void step();
    void setReflexiveNeuronInput(const double& value);
    void setPredictiveNeuronInput(const int& index, const double& value);
    void setPredictiveNeuronInput(std::vector<double>& values);
    const double& getOutputNeuronOutput();

  protected:
    virtual void updateWeights();
    void setInputNeuronInput(const int& index, const double& value);


  private:
    std::vector<Neuron*> inputNeurons;
    typedef std::vector <Neuron*> NeuronList;
    typedef std::vector <ANN*> AnnList;
    Neuron* outputNeuron;
    double oldReflexiveInput;
    double rate_ico;
};

#endif /* ICO_H_ */

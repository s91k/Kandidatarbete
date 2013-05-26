#pragma once

#include "Neuron.h"
#include <vector>

class NeuronLayer
{
private:
	std::vector<Neuron> m_neurons;
public:
	NeuronLayer();
	NeuronLayer(int nrOfNeurons, int nrOfInputs);
	NeuronLayer(std::vector<std::vector<float>> weights);
	~NeuronLayer();

	//Send the weighted output from the neurons in the layer to the neurons in the next layer
	void propagate(NeuronLayer &nextLayer);
	void backPropagate(NeuronLayer &nextLayer);

	//Set the outputs of the neurons in the layer
	void setOutput(std::vector<float> outputs);
	//Add the output values from the neurons to the vector
	void getOutput(std::vector<float> &outputs);
	//Get the number of neurons
	unsigned int size();
	//Calculate error, return the total error for the network
	float calculateError(std::vector<float> &expectedOutputs);
	//Adjust weights
	void adjustWeights(NeuronLayer &input, float momentum, float learningRate);
	//Reset the neuron weights
	void reset();
	//Get weights
	std::vector<std::vector<float>> getWeights() const;
};


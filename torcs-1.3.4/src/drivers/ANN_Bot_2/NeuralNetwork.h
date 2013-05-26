#pragma once

#include "NeuronLayer.h"
#include <vector>
#include <fstream>

class NeuralNetwork
{
private:
	NeuronLayer *m_inputLayer;
	NeuronLayer *m_outputLayer;

	std::vector<NeuronLayer> m_layers;
public:
	NeuralNetwork();
	NeuralNetwork(int nrOfInputs, int nrOfOutputs, int nrOfHiddenLayers, int nrOfHiddenNeurons);
	NeuralNetwork(std::string path);
	~NeuralNetwork();

	//Send the input values though the network and return the values from the output layer
	void use(std::vector<float> &input, std::vector<float> &output);
	//Compare the expected output with output that is calculated by the network and adjust the weights based on that
	void train(std::vector<float> &input, std::vector<float> &expectedOutput);
	//Reset the neuron weights
	void reset();
	//Get a vector of vectors describing the network
	std::vector<std::vector<std::vector<float>>> getNetworkWeights() const;
	//Export
	void exportNetwork(std::string path);
	//Import
	void importNetwork(std::string path);
};
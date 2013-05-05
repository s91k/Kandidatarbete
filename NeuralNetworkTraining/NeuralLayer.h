#pragma once

#include <vector>


struct Neuron
{
	std::vector<float> zWeights;	//Connection Strength
	std::vector<float> zLastDelta;	//Used for inertia in updating the weights while learning
	float zOutput;					//The fired potential of the neuron
	float zError;					//The error gradient of the potential from the expected 
	//potential; used when learning
};

class NeuralLayer
{
public:
	NeuralLayer(int nNeurons, int nInputs, int type);
	void Propagate(NeuralLayer& nextLayer);
	void BackPropagate(NeuralLayer& nextLayer);
	void AdjustWeights(NeuralLayer& nInputs, float lRate = 0.1f, float momentum = 0.5f);

	void SetInput(std::vector<float>& inputs);
	void GetOutput(std::vector<float> &outputs);
	inline unsigned int Size() {return this->zNeurons.size();}

	//Calculate error, return the total error for the network
	float CalculateError(std::vector<float> &expectedOutputs);

	//Data
	std::vector<Neuron*> zNeurons;

private:
	int zLayerType;
};
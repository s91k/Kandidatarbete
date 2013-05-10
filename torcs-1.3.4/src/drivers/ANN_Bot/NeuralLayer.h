#pragma once

#include <vector>

enum
{
	ACT_LOGISTIC,
	ACT_BIPOLAR,
	ACT_STEP,
	ACT_TANH,
	ACT_SOFTMAX,
	ACT_LINEAR
};

struct Neuron
{
	virtual ~Neuron() {}
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
	virtual ~NeuralLayer();
	void Propagate(int type, NeuralLayer& nextLayer);
	void BackPropagate(int type, NeuralLayer& nextLayer);
	void AdjustWeights(NeuralLayer& nInputs, float lRate = 0.1f, float momentum = 0.5f);

	void SetInput(std::vector<float>& inputs);
	void GetOutput(std::vector<float> &outputs);
	inline unsigned int Size() {return this->zNeurons.size();}

	//Calculate error, return the total error for the network
	float CalculateError(std::vector<float> &expectedOutputs);

	//activation functions
	float ActLogistic(float value);
	float ActStep(float value);
	float ActTanh(float value);
	float ActBipolarSigmoid(float value);

	//inverse functions for backprop
	float DerLogistic(float value);
	float DerTanh(float value);
	float DerBipolarSigmoid(float value);

	//Data
	std::vector<Neuron> zNeurons;
	int zLayerType;
	float zThreshold;
};
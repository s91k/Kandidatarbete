#pragma once

#include <vector>

enum
{
	// [0, 1]
	ACT_LOGISTIC,
	// [-1, 1]
	ACT_BIPOLAR, 
	//
	ACT_STEP,
	// [-1, 1]
	ACT_TANH,	 
	//
	ACT_SOFTMAX,
	//
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
	void Propagate(const int& type, NeuralLayer& nextLayer);
	void BackPropagate(const int& type, NeuralLayer& nextLayer);
	void AdjustWeights(NeuralLayer& nInputs, const float& lRate = 0.1f, const float& momentum = 0.5f);

	void SetInput(std::vector<float>& inputs);
	void GetOutput(std::vector<float> &outputs);
	inline unsigned int Size() {return this->zNeurons.size();}

	//Calculate error, return the total error for the network
	float CalculateError(int type, std::vector<float> &expectedOutputs);

	//activation functions
	float ActLogistic(const float& value);
	float ActStep(float value);
	float ActTanh(float value);
	float ActBipolarSigmoid(const float& value);

	//inverse functions for backprop
	float DerLogistic(const float& value);
	float DerTanh(float value);
	float DerBipolarSigmoid(const float& value);
	//Used to fix memory leaks since it tries to 
	//delete the neurons every time a new layer is created
	void Clear();

	//Data
	std::vector<Neuron*> zNeurons;
	int zLayerType;
	float zThreshold;
};
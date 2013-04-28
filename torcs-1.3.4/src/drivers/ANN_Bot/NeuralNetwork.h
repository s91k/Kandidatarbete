#pragma once

#include <vector>

enum NN_TYPE
{
	NN_INPUT,
	NN_HIDDEN,
	NN_OUTPUT
};

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
	void Propogate(NeuralLayer& nexLayer);
	void BackPropogate(NeuralLayer& nexLayer);
	void AdjustWeights(NeuralLayer& nInputs, float lRate = 0.1f, float momentum = 0.5f);

	//Data
	std::vector<Neuron*> zNeurons;
	int zLayerType;
};


class NeuralNetwork
{
private:
	int zNInputs;
	int zNOutputs;
	int zNLayers;
	int zNHiddenNodesPerLayer;

	std::vector<NeuralLayer> zLayers;
	NeuralLayer* zInputLayer;
	NeuralLayer* zOutputLayer;

	float zLearningRate;
	float zMomentum;
	float zError;

private:
	void AddLayer(int nNeurons, int nInputs, int type);
public:
	NeuralNetwork(int nInputs, int nOutputs, int nHiddenLayers, int nNodesInHiddenLayers);
	void Init();
	void Train(std::vector<float>& nInputs, std::vector<float>& nOutputs);
	
	void WriteWeights();
	void ReadWeights();
	
	float GetError() {return zError;}
};
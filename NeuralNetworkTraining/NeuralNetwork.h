#pragma once

#include "NeuralLayer.h"

enum NN_TYPE
{
	NN_INPUT,
	NN_HIDDEN,
	NN_OUTPUT
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

	std::string zWeightFile;

	int zActType;
	int zOutputActType;

private:
	void AddLayer(int nNeurons, int nInputs, int type);
public:
	NeuralNetwork(int nInputs, int nOutputs, int nHiddenLayers, int nNodesInHiddenLayers, const std::string& weightFile);
	virtual ~NeuralNetwork();
	void Init();
	void Train(std::vector<float>& nInputs, std::vector<float>& nExpectedOutput);
	void Use(std::vector<float>& nInputs, std::vector<float>& nOutputs);

	void Propagate();
	void BackPropagate();

	void WriteWeights();
	void ReadWeights();

	void SetInputs(std::vector<float>& inputs);
	void FindError(std::vector<float>& ouputs);

	float GetError() {return this->zError;}
};
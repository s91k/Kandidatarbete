#include "NeuralLayer.h"

NeuralLayer::NeuralLayer( int nNeurons, int nInputs, int type )
{
	this->zLayerType = type;
	for (int i = 0; i < nNeurons; i++)
	{
		Neuron newNeuron = Neuron();

		for (int j = 0; j < nInputs + 1; j++)
		{
			float valueOne = (float)rand() / (float)RAND_MAX;
			float valueTwo = (float)rand() / (float)RAND_MAX;
			
			//Random value between 1 & -1
			newNeuron.zWeights.push_back(valueOne - valueTwo);
			newNeuron.zLastDelta.push_back(0.0f);
		}

		//Initial Values
		newNeuron.zOutput = 0.0f;
		newNeuron.zError = 99999.9f;

		this->zNeurons.push_back(newNeuron);
	}
}

NeuralLayer::~NeuralLayer()
{

}

void NeuralLayer::Propagate( NeuralLayer& nextLayer )
{
	int numNeurons = nextLayer.zNeurons.size();

	for (int i = 0; i < numNeurons; i++)
	{
		float value = 0.0f;

		int numWeights = this->zNeurons.size();
		for (int j = 0; j < numWeights; j++)
		{
			value += nextLayer.zNeurons[i].zWeights[j] * this->zNeurons[j].zOutput;
		}

		//add in the bias (always has an input of -1)
		value += nextLayer.zNeurons[i].zWeights[numWeights] * -1.0f;

		nextLayer.zNeurons[i].zOutput = value;
	}
}

void NeuralLayer::BackPropagate( NeuralLayer& nextLayer )
{
	/*
	float outputValue;
	float error;

	int numNeurons = nextLayer.zNeurons.size();

	for (int i = 0; i < numNeurons; i++)
	{
		outputValue = nextLayer.zNeurons[i]->zOutput;
		error = 0;
		for (int j = 0; j < this->zNeurons.size(); j++)
		{
			error += this->zNeurons[j]->zWeights[i] * this->zNeurons[j]->zError;
		}
		nextLayer.zNeurons[i]->zError = outputValue * error;
	}
	*/
	for(unsigned int i = 0; i < this->zNeurons.size(); i++)
	{
		float error = 0.0f;

		for(unsigned int j = 0; j < nextLayer.zNeurons.size(); j++)
		{
			error += nextLayer.zNeurons[j].zWeights[i] * nextLayer.zNeurons[j].zError;
		}		

		this->zNeurons[i].zError = this->zNeurons[i].zOutput * error;
	}	
}

void NeuralLayer::AdjustWeights( NeuralLayer& nInputs, float lRate /*= 0.1f*/, float momentum /*= 0.5f*/ )
{
	for (unsigned int i = 0; i < this->zNeurons.size(); i++)
	{
		int numWeights = this->zNeurons[i].zWeights.size();
		for (int j = 0; j < numWeights; j++)
		{
			float output = (j == numWeights - 1) ? -1 : nInputs.zNeurons[j].zOutput;

			float error = this->zNeurons[i].zError;
			float delta = momentum * this->zNeurons[i].zLastDelta[j] + 
				(1.0f - momentum) * lRate * error * output; 

			this->zNeurons[i].zWeights[j] += delta;
			this->zNeurons[i].zLastDelta[j] = delta;
		}
	}
}

void NeuralLayer::SetInput( std::vector<float>& inputs )
{
	int numNeurons = this->zNeurons.size();
	for (int i = 0; i < numNeurons; i++)
	{
		this->zNeurons[i].zOutput = inputs[i];
	}
}

void NeuralLayer::GetOutput(std::vector<float> &outputs)
{
	outputs.clear();

	for(unsigned int i = 0; i < this->zNeurons.size(); i++)
	{
		outputs.push_back(this->zNeurons[i].zOutput);
	}
}

float NeuralLayer::CalculateError( std::vector<float> &expectedOutputs )
{
	float totalError = 0.0f;

	int numNeurons = this->zNeurons.size();

	for (int i = 0; i < numNeurons; i++)
	{
		float error = expectedOutputs[i] - this->zNeurons[i].zOutput;
		this->zNeurons[i].zError = error;

		totalError += error;
	}

	return totalError;
}
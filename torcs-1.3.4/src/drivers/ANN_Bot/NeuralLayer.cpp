#include "NeuralLayer.h"

NeuralLayer::NeuralLayer( int nNeurons, int nInputs, int type )
{
	this->zThreshold = 1.0f;
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

void NeuralLayer::Propagate(int type, NeuralLayer& nextLayer)
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

		switch (type)
		{
		case ACT_STEP:
			nextLayer.zNeurons[i].zOutput = this->ActStep(value);
			break;
		case ACT_TANH:
			nextLayer.zNeurons[i].zOutput = this->ActTanh(value);
			break;
		case ACT_LOGISTIC:
			nextLayer.zNeurons[i].zOutput = this->ActLogistic(value);
			break;
		case ACT_BIPOLAR:
			nextLayer.zNeurons[i].zOutput = this->ActBipolarSigmoid(value);
			break;
		case ACT_LINEAR:
		default:
			nextLayer.zNeurons[i].zOutput = value;
			break;
		}
		
	}
}

void NeuralLayer::BackPropagate(int type, NeuralLayer& nextLayer)
{

	/*float outputValue;
	float error;

	int numNeurons = nextLayer.zNeurons.size();

	for (int i = 0; i < numNeurons; i++)
	{
	outputValue = nextLayer.zNeurons[i].zOutput;
	error = 0;
	for (int j = 0; j < this->zNeurons.size(); j++)
	{
	error += this->zNeurons[j]->zWeights[i] * this->zNeurons[j].zError;
	}
	nextLayer.zNeurons[i].zError = outputValue * error;
	}*/

	for(unsigned int i = 0; i < this->zNeurons.size(); i++)
	{
		float error = 0.0f;

		for(unsigned int j = 0; j < nextLayer.zNeurons.size(); j++)
		{
			error += nextLayer.zNeurons[j].zWeights[i] * nextLayer.zNeurons[j].zError;
		}		

		switch (type)
		{
		case ACT_TANH:
			this->zNeurons[i].zError = this->DerTanh(this->zNeurons[i].zOutput) * error;
			break;
		case ACT_LOGISTIC:
			this->zNeurons[i].zError = this->DerLogistic(this->zNeurons[i].zOutput) * error;
			break;
		case ACT_BIPOLAR:
			this->zNeurons[i].zError = this->DerBipolarSigmoid(this->zNeurons[i].zOutput) * error;
			break;
		case ACT_LINEAR:
		default:
			this->zNeurons[i].zError = this->zNeurons[i].zOutput * error;
			break;
		}
	}	
}

void NeuralLayer::AdjustWeights(NeuralLayer& nInputs, float lRate /*= 0.1f*/, float momentum /*= 0.5f*/)
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

float NeuralLayer::ActLogistic( float value )
{
	return (1 / (1 + exp( -value * zThreshold)));
}

float NeuralLayer::ActStep( float value )
{
	return (value * zThreshold * (1.0f - value));
}

float NeuralLayer::ActTanh( float value )
{
	return (tanh(value * zThreshold));
}

float NeuralLayer::ActBipolarSigmoid( float value )
{
	 return ((2.0f / (1.0f + exp( -value * zThreshold))) - 1.0f);
}

float NeuralLayer::DerLogistic( float value )
{
	return (value * zThreshold * (1.0f - value));
}

float NeuralLayer::DerTanh( float value )
{
	return (1 - value * value);
}

float NeuralLayer::DerBipolarSigmoid( float value )
{
	return (0.5f * zThreshold * (1 + value) * (1 - value));
}

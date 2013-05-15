#include "NeuralLayer.h"

NeuralLayer::NeuralLayer( int nNeurons, int nInputs, int type )
{
	this->zThreshold = 1.0f;
	this->zLayerType = type;
	for (int i = 0; i < nNeurons; i++)
	{
		Neuron* newNeuron = new Neuron();

		for (int j = 0; j < nInputs + 1; j++)
		{
			float valueOne = (float)rand() / (float)RAND_MAX;
			float valueTwo = (float)rand() / (float)RAND_MAX;
			
			//Random value between 1 & -1
			newNeuron->zWeights.push_back(valueOne - valueTwo);
			newNeuron->zLastDelta.push_back(0.0f);
		}

		//Initial Values
		newNeuron->zOutput = 0.0f;
		newNeuron->zError = 99999.9f;

		this->zNeurons.push_back(newNeuron);
	}
}

NeuralLayer::~NeuralLayer()
{
	
}

void NeuralLayer::Clear()
{
	for (unsigned int i = 0; i < this->zNeurons.size(); i++)
	{
		if(this->zNeurons[i])
		{
			delete this->zNeurons[i];
			this->zNeurons[i] = NULL;
		}
	}
	this->zNeurons.clear();
}

void NeuralLayer::Propagate(int type, NeuralLayer& nextLayer)
{
	int numNeurons = nextLayer.zNeurons.size();

	for (int i = 0; i < numNeurons; i++)
	{
		float value = 0.0f;

		int numWeights = this->zNeurons.size();
		Neuron* neuron = nextLayer.zNeurons[i];
		for (int j = 0; j < numWeights; j++)
		{
			value += neuron->zWeights[j] * this->zNeurons[j]->zOutput;
		}

		//add in the bias (always has an input of -1)
		value += neuron->zWeights[numWeights] * -1.0f;

		switch (type)
		{
		case ACT_STEP:
			neuron->zOutput = this->ActStep(value);
			break;
		case ACT_TANH:
			neuron->zOutput = this->ActTanh(value);
			break;
		case ACT_LOGISTIC:
			neuron->zOutput = this->ActLogistic(value);
			break;
		case ACT_BIPOLAR:
			neuron->zOutput = this->ActBipolarSigmoid(value);
			break;
		case ACT_LINEAR:
		default:
			neuron->zOutput = value;
			break;
		}
	}
}

void NeuralLayer::BackPropagate(int type, NeuralLayer& nextLayer)
{

	//float outputValue;
	//float error;
	//
	//int numNeurons = nextLayer.zNeurons.size();
	//
	//for (int i = 0; i < numNeurons; i++)
	//{
	//	outputValue = nextLayer.zNeurons[i]->zOutput;
	//	error = 0;
	//	for (int j = 0; j < this->zNeurons.size(); j++)
	//	{
	//		error += this->zNeurons[j]->zWeights[i] * this->zNeurons[j]->zError;
	//	}
	//
	//	switch (type)
	//	{
	//	case ACT_TANH:
	//		nextLayer.zNeurons[i]->zError = this->DerTanh(outputValue) * error;
	//		break;
	//	case ACT_LOGISTIC:
	//		nextLayer.zNeurons[i]->zError = this->DerLogistic(outputValue) * error;
	//		break;
	//	case ACT_BIPOLAR:
	//		nextLayer.zNeurons[i]->zError = this->DerBipolarSigmoid(outputValue) * error;
	//		break;
	//	case ACT_LINEAR:
	//	default:
	//		nextLayer.zNeurons[i]->zError = outputValue * error;
	//		break;
	//	}
	//}

	unsigned int numNeurons = this->zNeurons.size();
	for(unsigned int i = 0; i < numNeurons; i++)
	{
		float error = 0.0f;
		unsigned int numNextLayerNeurons = nextLayer.zNeurons.size();
		for(unsigned int j = 0; j < numNextLayerNeurons; j++)
		{
			Neuron* NextLayerNeuron = nextLayer.zNeurons[j];
			error += NextLayerNeuron->zWeights[i] * NextLayerNeuron->zError;
		}		
		Neuron* neuron = this->zNeurons[i];
		switch (type)
		{
		case ACT_TANH:
			neuron->zError = this->DerTanh(neuron->zOutput) * error;
			break;
		case ACT_LOGISTIC:
			neuron->zError = this->DerLogistic(neuron->zOutput) * error;
			break;
		case ACT_BIPOLAR:
			neuron->zError = this->DerBipolarSigmoid(neuron->zOutput) * error;
			break;
		case ACT_LINEAR:
		default:
			neuron->zError = neuron->zOutput * error;
			break;
		}
	}	
}

void NeuralLayer::AdjustWeights(NeuralLayer& nInputs, float lRate /*= 0.1f*/, float momentum /*= 0.5f*/)
{
	unsigned int numNeurons = this->zNeurons.size();
	for (unsigned int i = 0; i < numNeurons; i++)
	{
		Neuron* neuron = this->zNeurons[i];
		int numWeights = neuron->zWeights.size();
		for (int j = 0; j < numWeights; j++)
		{
			float output = (j == numWeights - 1) ? -1 : nInputs.zNeurons[j]->zOutput;

			float error = neuron->zError;
			float delta = momentum * neuron->zLastDelta[j] + 
				(1.0f - momentum) * lRate * error * output; 

			neuron->zWeights[j] += delta;
			neuron->zLastDelta[j] = delta;
		}
	}
}

void NeuralLayer::SetInput( std::vector<float>& inputs )
{
	int numNeurons = this->zNeurons.size();

	for (int i = 0; i < numNeurons; i++)
	{
		this->zNeurons[i]->zOutput = inputs[i];
	}
}

void NeuralLayer::GetOutput(std::vector<float> &outputs)
{
	outputs.clear();

	unsigned int numNeurons = this->zNeurons.size();
	for(unsigned int i = 0; i < numNeurons; i++)
	{
		outputs.push_back(this->zNeurons[i]->zOutput);
	}
}

float NeuralLayer::CalculateError( std::vector<float> &expectedOutputs )
{
	float totalError = 0.0f;

	int numNeurons = this->zNeurons.size();

	for (int i = 0; i < numNeurons; i++)
	{
		Neuron* neuron = this->zNeurons[i];
		float error = expectedOutputs[i] - neuron->zOutput;
		neuron->zError = error;

		totalError += error;
	}

	return totalError;
}

float NeuralLayer::ActLogistic( float value )
{
	return (1 / (1 + exp( -value * this->zThreshold)));
}

float NeuralLayer::ActStep( float value )
{
	return (value * this->zThreshold * (1.0f - value));
}

float NeuralLayer::ActTanh( float value )
{
	return (tanh(value * this->zThreshold));
}

float NeuralLayer::ActBipolarSigmoid( float value )
{
	 return ((2.0f / (1.0f + exp( -value * this->zThreshold))) - 1.0f);
}

float NeuralLayer::DerLogistic( float value )
{
	return (value * this->zThreshold * (1.0f - value));
}

float NeuralLayer::DerTanh( float value )
{
	return (1 - value * value);
}

float NeuralLayer::DerBipolarSigmoid( float value )
{
	return (0.5f * this->zThreshold * (1.0f + value) * (1.0f - value));
}
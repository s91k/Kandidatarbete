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

void NeuralLayer::Propagate( const int& type, NeuralLayer& nextLayer )
{
	int numWeights;
	int numNeurons = nextLayer.zNeurons.size();

	for (int i = 0; i < numNeurons; i++)
	{
		float value = 0.0f;

		numWeights = this->zNeurons.size();
		Neuron* nextLayerNeuron = nextLayer.zNeurons[i];
		for (int j = 0; j < numWeights; j++)
		{
			value += nextLayerNeuron->zWeights[j] * this->zNeurons[j]->zOutput;
		}

		//add in the bias (always has an input of -1)
		value += nextLayer.zNeurons[i]->zWeights[numWeights] * -1.0f;

		switch (type)
		{
		case ACT_STEP:
			nextLayerNeuron->zOutput = this->ActStep(value);
			break;
		case ACT_TANH:
			nextLayerNeuron->zOutput = this->ActTanh(value);
			break;
		case ACT_LOGISTIC:
			nextLayerNeuron->zOutput = this->ActLogistic(value);
			break;
		case ACT_BIPOLAR:
			nextLayerNeuron->zOutput = this->ActBipolarSigmoid(value);
			break;
		case ACT_LINEAR:
		default:
			nextLayerNeuron->zOutput = value;
			break;
		}
	}
}

void NeuralLayer::BackPropagate( const int& type, NeuralLayer& nextLayer )
{
	float outputVal, error;
	unsigned int numNeurons;

	unsigned int numNextLayerNeurons = nextLayer.zNeurons.size();
	for(unsigned int i = 0; i < numNextLayerNeurons; i++)
	{
		error = 0.0f;
		outputVal = nextLayer.zNeurons[i]->zOutput;

		numNeurons = this->zNeurons.size();
		for(unsigned int j = 0; j < numNeurons; j++)
		{
			Neuron* neuron = this->zNeurons[j];
			error += neuron->zWeights[i] * neuron->zError;		
		}

		switch (type)
		{
		case ACT_TANH:
			nextLayer.zNeurons[i]->zError = this->DerTanh(outputVal) * error;
			break;
		case ACT_LOGISTIC:
			nextLayer.zNeurons[i]->zError = this->DerLogistic(outputVal) * error;
			break;
		case ACT_BIPOLAR:
			nextLayer.zNeurons[i]->zError = this->DerBipolarSigmoid(outputVal) * error;
			break;
		case ACT_LINEAR:
		default:
			nextLayer.zNeurons[i]->zError = outputVal * error;
			break;
		}
	}

	/*unsigned int numNeurons = this->zNeurons.size();
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
	}*/	

}

void NeuralLayer::AdjustWeights( NeuralLayer& nInputs, const float& lRate /*= 0.1f*/, const float& momentum /*= 0.5f*/ )
{
	unsigned int numNeurons = this->zNeurons.size();
	for (unsigned int i = 0; i < numNeurons; i++)
	{
		Neuron* neuron = this->zNeurons[i];
		int numWeights = neuron->zWeights.size();
		float multiplier = (1.0f - momentum) * lRate * neuron->zError;
		for (int j = 0; j < numWeights; j++)
		{
			float output = (j == numWeights - 1) ? -1 : nInputs.zNeurons[j]->zOutput;

			float delta = momentum * neuron->zLastDelta[j] + 
				multiplier * output; 

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

float NeuralLayer::CalculateError(int type, std::vector<float> &expectedOutputs)
{
	float totalError = 0.0f;

	int numNeurons = this->zNeurons.size();

	for (int i = 0; i < numNeurons; i++)
	{
		float outputVal = this->zNeurons[i]->zOutput;
		float error = expectedOutputs[i] - outputVal;
		switch (type)
		{
		case ACT_TANH:
			this->zNeurons[i]->zError = this->DerTanh(outputVal) * error;
			break;
		case ACT_LOGISTIC:
			this->zNeurons[i]->zError = this->DerLogistic(outputVal) * error;
			break;
		case ACT_BIPOLAR:
			this->zNeurons[i]->zError = this->DerBipolarSigmoid(outputVal) * error;
			break;
		case ACT_LINEAR:
		default:
			this->zNeurons[i]->zError = outputVal * error;
			break;
		}

		totalError += 0.5f * error * error;
	}

	return totalError;
}

float NeuralLayer::ActLogistic( const float& value )
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

float NeuralLayer::ActBipolarSigmoid( const float& value )
{
	 return ((2.0f / (1.0f + exp( -value * this->zThreshold))) - 1.0f);
}

float NeuralLayer::DerLogistic( const float& value )
{
	return (value * this->zThreshold * (1.0f - value));
}

float NeuralLayer::DerTanh( float value )
{
	return (1 - value * value);
}

float NeuralLayer::DerBipolarSigmoid( const float& value )
{
	return (0.5f * this->zThreshold * (1.0f + value) * (1.0f - value));
}
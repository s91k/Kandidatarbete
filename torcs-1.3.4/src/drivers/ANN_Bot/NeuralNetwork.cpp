#include "NeuralNetwork.h"

NeuralLayer::NeuralLayer( int nNeurons, int nInputs, int type )
{
	this->zLayerType = type;
	for (int i = 0; i < nNeurons; i++)
	{
		Neuron* newNeuron = new Neuron();

		for (int j = 0; j < nInputs; j++)
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

void NeuralLayer::Propagate( NeuralLayer& nextLayer )
{
	int weightIndex;
	int numNeurons = nextLayer.zNeurons.size();

	for (int i = 0; i < numNeurons; i++)
	{
		weightIndex = 0;
		float value = 0.0f;

		int numWeights = this->zNeurons.size();
		for (int j = 0; j < numWeights; j++)
		{
			value += nextLayer.zNeurons[i]->zWeights[j] * this->zNeurons[j]->zOutput;
		}

		//add in the bias (always has an input of -1)
		value += nextLayer.zNeurons[i]->zWeights[numWeights] * -1.0f;

		nextLayer.zNeurons[i]->zOutput = value;
	}
}

void NeuralLayer::BackPropagate( NeuralLayer& nextLayer )
{
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
}

void NeuralLayer::AdjustWeights( NeuralLayer& nInputs, float lRate /*= 0.1f*/, float momentum /*= 0.5f*/ )
{
	for (unsigned int i = 0; i < this->zNeurons.size(); i++)
	{
		int numWeights = this->zNeurons[i]->zWeights.size();
		for (unsigned int j = 0; j < numWeights; i++)
		{
			float output;
			if (j == numWeights - 1)
				output = -1;
			else
				nInputs.zNeurons[j]->zOutput;

			float error = this->zNeurons[i]->zError;
			float delta = momentum * this->zNeurons[i]->zLastDelta[j] + 
				(1 - momentum) * lRate * error * output; 

			this->zNeurons[i]->zWeights[j] += delta;
			this->zNeurons[i]->zLastDelta[j] = delta;
		}
	}
}


void NeuralNetwork::AddLayer( int nNeurons, int nInputs, int type )
{
	this->zLayers.push_back(NeuralLayer(nNeurons, nInputs, type));
}

NeuralNetwork::NeuralNetwork( int nInputs, int nOutputs, int nHiddenLayers, int nNodesInHiddenLayers )
{
	this->zNInputs = nInputs;
	this->zNOutputs = nOutputs;
	this->zNLayers = nHiddenLayers + 2;
	this->zNHiddenNodesPerLayer = nNodesInHiddenLayers;

	this->Init();
}

void NeuralNetwork::Train( std::vector<float>& nInputs, std::vector<float>& nExpectedOutput )
{
	this->SetInputs(nInputs);
	this->Propagate();
	this->FindError(nExpectedOutput);
	this->BackPropagate();
}

void NeuralNetwork::Use( std::vector<float>& nInputs, std::vector<float>& nOutputs )
{
	this->SetInputs(nInputs);
	this->Propagate();
	nOutputs.clear();

	//Return the net outputs
	for (unsigned int i = 0; i < this->zOutputLayer->zNeurons.size(); i++)
		nOutputs.push_back(this->zOutputLayer->zNeurons[i]->zOutput);
}

void NeuralNetwork::SetInputs( std::vector<float>& inputs )
{
	int numNeurons = this->zInputLayer->zNeurons.size();

	for (int i = 0; i < numNeurons; i++)
	{
		this->zInputLayer->zNeurons[i]->zOutput = inputs[i];
	}
}

void NeuralNetwork::FindError( std::vector<float>& outputs )
{
	this->zError = 0.0f;

	int numNeurons = this->zOutputLayer->zNeurons.size();

	for (int i = 0; i < numNeurons; i++)
	{
		float outputValue = this->zOutputLayer->zNeurons[i]->zOutput;
		float error = outputs[i] - outputValue;

		this->zOutputLayer->zNeurons[i]->zError = outputValue * error;

		//error calculation for the entire net
		this->zError += 0.5f * error * error;
	}

	

}

void NeuralNetwork::Propagate()
{
	for (int i = 0; i < this->zNLayers - 1; i++)
	{
		this->zLayers[i].Propagate(this->zLayers[i + 1]);
	}
}

void NeuralNetwork::BackPropagate()
{
	//BackPropagate the error
	for (int i = this->zNLayers - 1; i > 0; i--)
	{
		this->zLayers[i].BackPropagate(this->zLayers[i - 1]);
	}

	//Adjust the weights
	for (int i =  1; i < this->zNLayers; i++)
	{
		this->zLayers[i].AdjustWeights(this->zLayers[i - 1], zLearningRate, zMomentum);
	}
}

void NeuralNetwork::WriteWeights()
{

}

void NeuralNetwork::ReadWeights()
{

}

void NeuralNetwork::Init()
{
	this->zInputLayer = NULL;
	this->zOutputLayer = NULL;
	
	this->zLearningRate = 0.1f;
	this->zMomentum = 0.9f;

	//error check
	if(this->zNLayers < 2)
		return;

	//clear out the layers, in Case you're restarting the net
	zLayers.clear();

	//input layer
	AddLayer(this->zNInputs, 1, NN_INPUT);

	if (this->zNLayers > 2)
	{
		//First Hidden Layer connect back to inputs
		AddLayer(this->zNHiddenNodesPerLayer, this->zNInputs, NN_HIDDEN);

		//any other hidden layers connect to other hidden outputs
		//-3 since the first layer was the inputs,
		//the second (connected to inputs) was initialized above,
		//and the last one (connect to outputs) will be initialized below
		for (int i = 0; i < this->zNLayers - 3; i++)
			AddLayer(this->zNHiddenNodesPerLayer, this->zNHiddenNodesPerLayer, NN_HIDDEN);

		//Output layer connects to hidden
		AddLayer(this->zNOutputs, this->zNHiddenNodesPerLayer, NN_OUTPUT);
	}
	else
	{
		//output layer connects to inputs
		AddLayer(this->zNOutputs, this->zNInputs, NN_OUTPUT);
	}

	this->zInputLayer = &this->zLayers[0];
	this->zOutputLayer = &this->zLayers[this->zNLayers - 1];
}
#include "NeuralNetwork.h"

NeuralLayer::NeuralLayer( int nNeurons, int nInputs, int type )
{
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

void NeuralLayer::Propagate( NeuralLayer& nextLayer )
{
	int numNeurons = nextLayer.zNeurons.size();

	for (int i = 0; i < numNeurons; i++)
	{
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
			error += nextLayer.zNeurons[j]->zWeights[i] * nextLayer.zNeurons[j]->zError;
		}		

		this->zNeurons[i]->zError = this->zNeurons[i]->zOutput * error;
	}	
}

void NeuralLayer::AdjustWeights( NeuralLayer& nInputs, float lRate /*= 0.1f*/, float momentum /*= 0.5f*/ )
{
	for (unsigned int i = 0; i < this->zNeurons.size(); i++)
	{
		int numWeights = this->zNeurons[i]->zWeights.size();
		for (int j = 0; j < numWeights; j++)
		{
			float output = (j == numWeights - 1) ? -1 : nInputs.zNeurons[j]->zOutput;

			float error = this->zNeurons[i]->zError;
			float delta = momentum * this->zNeurons[i]->zLastDelta[j] + 
				(1 - momentum) * lRate * error * output; 

			this->zNeurons[i]->zWeights[j] += delta;
			this->zNeurons[i]->zLastDelta[j] = delta;
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

	for(unsigned int i = 0; i < this->zNeurons.size(); i++)
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
		float error = expectedOutputs[i] - this->zNeurons[i]->zOutput;
		this->zNeurons[i]->zError = error;

		totalError += error;
	}

	return totalError;
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

	this->zOutputLayer->GetOutput(nOutputs);
}

void NeuralNetwork::SetInputs( std::vector<float>& inputs )
{
	this->zInputLayer->SetInput(inputs);
}

void NeuralNetwork::FindError( std::vector<float>& expectedOutput )
{
	/*this->zError = 0.0f;

	int numNeurons = this->zOutputLayer->zNeurons.size();

	for (int i = 0; i < numNeurons; i++)
	{
		float outputValue = this->zOutputLayer->zNeurons[i]->zOutput;
		float error = expectedOutput[i] - outputValue;

		this->zOutputLayer->zNeurons[i]->zError = error;
	
		//error calculation for the entire net
		this->zError += 0.5f * error * error;
	}
	*/
	this->zError = this->zOutputLayer->CalculateError(expectedOutput);
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
		this->zLayers[i - 1].BackPropagate(this->zLayers[i]);
	}

	//Adjust the weights
	for (int i =  0; i < this->zNLayers - 1; i++)
	{
		this->zLayers[i + 1].AdjustWeights(this->zLayers[i], zLearningRate, zMomentum);
	}
}

void NeuralNetwork::WriteWeights()
{
	FILE* pFile;
	fopen_s(&pFile, "NNWeightData.txt", "w");
	if (pFile == NULL)
		return;

	for (int i = 0; i < this->zNLayers; i++)
	{
		int numNeurons = this->zLayers[i].zNeurons.size();

		for (int j = 0; j < numNeurons; j++)
		{
			int numWeights = this->zLayers[i].zNeurons[j]->zWeights.size();
			for (int k = 0; k < numWeights; k++)
				fprintf(pFile, "%f ", this->zLayers[i].zNeurons[j]->zWeights[k]);
		}
	}
	fclose(pFile);
}

void NeuralNetwork::ReadWeights()
{
	FILE* pFile;
	fopen_s(&pFile, "NNWeightData.txt", "r");
	if (pFile == NULL)
		return;

	for(int i = 0; i < this->zNLayers; i++)
	{
		int numNeurons = this->zLayers[i].zNeurons.size();
		for(int j = 0; j < numNeurons; j++)
		{
			int numWeights = this->zLayers[i].zNeurons[j]->zWeights.size();
			for(int k = 0; k < numWeights; k++)
				fscanf(pFile,"%f ",&this->zLayers[i].zNeurons[j]->zWeights[k]);
		}
	}
	fclose(pFile);
}

void NeuralNetwork::Init()
{
	this->zInputLayer = NULL;
	this->zOutputLayer = NULL;
	
	this->zLearningRate = 0.01f;
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
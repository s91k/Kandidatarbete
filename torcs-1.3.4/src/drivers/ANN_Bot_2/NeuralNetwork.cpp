#include "NeuralNetwork.h"

NeuralNetwork::NeuralNetwork()
{

}

NeuralNetwork::NeuralNetwork(int nrOfInputs, int nrOfOutputs, int nrOfHiddenLayers, int nrOfHiddenNeurons)
{
	this->m_layers.push_back(NeuronLayer(nrOfInputs, 0));	//Create the input layer

	for(int i = 0; i < nrOfHiddenLayers; i++)
	{
		this->m_layers.push_back(NeuronLayer(nrOfHiddenNeurons, this->m_layers.back().size()));
	}

	this->m_layers.push_back(NeuronLayer(nrOfOutputs, this->m_layers.back().size())); //Create the output layer

	//Assign the input and output layer variables for easy access
	this->m_inputLayer = &this->m_layers[0];
	this->m_outputLayer = &this->m_layers[this->m_layers.size() - 1];
}

NeuralNetwork::NeuralNetwork(std::string path)
{
	this->importNetwork(path);
}

NeuralNetwork::~NeuralNetwork()
{
}

void NeuralNetwork::use(std::vector<float> &input, std::vector<float> &output)
{
	this->m_inputLayer->setOutput(input);

	for(unsigned int i = 0; i < this->m_layers.size() - 1; i++)
	{
		this->m_layers[i].propagate(this->m_layers[i+1]);
	}

	this->m_outputLayer->getOutput(output);
}

void NeuralNetwork::train(std::vector<float> &input, std::vector<float> &expectedOutput)
{
	//Propagate the values
	this->m_inputLayer->setOutput(input);

	for(unsigned int i = 0; i < this->m_layers.size() - 1; i++)
	{
		this->m_layers[i].propagate(this->m_layers[i+1]);
	}

	//Find errors in the output layer
	float totalError = this->m_outputLayer->calculateError(expectedOutput);

	//Back propagate
	for(unsigned int i = this->m_layers.size() - 1; i > 0; i--)
	{
		this->m_layers[i - 1].backPropagate(this->m_layers[i]);
	}

	//Adjust the weights
	for(unsigned int i = 0; i < this->m_layers.size() - 1; i++)
	{
		this->m_layers[i + 1].adjustWeights(this->m_layers[i], 0.8f, 0.00000001f);
	}
}

void NeuralNetwork::reset()
{
	for(unsigned int i = 0; i < this->m_layers.size(); i++)
	{
		this->m_layers[i].reset();
	}
}

std::vector<std::vector<std::vector<float>>> NeuralNetwork::getNetworkWeights() const
{
	std::vector<std::vector<std::vector<float>>> networkWeights;

	for(int i = 0; i < this->m_layers.size(); i++)
	{
		networkWeights.push_back(this->m_inputLayer[i].getWeights());
	}

	return networkWeights;
}

void NeuralNetwork::exportNetwork(std::string path)
{
	std::vector<std::vector<std::vector<float>>> networkWeights = this->getNetworkWeights();
	std::ofstream output;

	output.open(path);

	if(output.is_open() == true)
	{
		for(int i = 0; i < networkWeights.size(); i++)
		{
			output << "LAYER_START" << std::endl;

			for(int j = 0; j < networkWeights[i].size(); j++)
			{
				output << "NEURON_START" << std::endl;

				for(int k = 0; k < networkWeights[i][j].size(); k++)
				{
					output << networkWeights[i][j][k] << std::endl;
				}

				output << "NEURON_END" << std::endl;
			}

			output << "LAYER_END" << std::endl;
		}

		output.close();
	}
}

void NeuralNetwork::importNetwork(std::string path)
{
	std::ifstream input;

	input.open(path);

	printf("Opening file.\n");

	if(input.is_open() == true)
	{
		std::vector<std::vector<std::vector<float>>> networkWeights;
		std::vector<std::vector<float>> layer;
		std::vector<float> neuron;

		printf("File opened.\n");

		while(input.eof() == false)
		{
			char buffer[1024];

			input.getline(buffer, 1024);

			if(strcmp(buffer, "LAYER_START") == 0)
			{
				layer.clear();
			}
			else if(strcmp(buffer, "LAYER_END") == 0)
			{
				networkWeights.push_back(layer);
			}
			else if(strcmp(buffer, "NEURON_START") == 0)
			{
				neuron.clear();
			}
			else if(strcmp(buffer, "NEURON_END") == 0)
			{
				layer.push_back(neuron);
			}
			else
			{
				float weight;
				sscanf(buffer, "%f", &weight);
				neuron.push_back(weight);
			}
		}

		input.close();

		for(int i = 0; i < networkWeights.size(); i++)
		{
			this->m_layers.push_back(NeuronLayer(networkWeights[i]));
		}

		this->m_inputLayer = &this->m_layers[0];
		this->m_outputLayer = &this->m_layers[this->m_layers.size() - 1];
	}
}
#include "NeuronLayer.h"


NeuronLayer::NeuronLayer()
{

}

NeuronLayer::NeuronLayer(int nrOfNeurons, int nrOfInputs)
{
	for(int i = 0; i < nrOfNeurons; i++)
	{
		this->m_neurons.push_back(Neuron(nrOfInputs));
	}
}

NeuronLayer::NeuronLayer(std::vector<std::vector<float>> weights)
{
	for(int i = 0; i < weights.size(); i++)
	{
		this->m_neurons.push_back(weights[i]);
	}
}

NeuronLayer::~NeuronLayer()
{

}

void NeuronLayer::propagate(NeuronLayer &nextLayer)
{
	//Send output values to each of the next layers' neurons
	for(unsigned int i = 0; i < nextLayer.m_neurons.size(); i++)
	{
		//Add the output value multiplied by the weight from each neuron in the layer to the neuron in the next layer
		float value = 0.0f;

		for(unsigned int j = 0; j < this->m_neurons.size(); j++)
		{
			value += this->m_neurons[j].m_output * nextLayer.m_neurons[i].m_weights[j];
		}

		nextLayer.m_neurons[i].m_output = value;	
	}
}

void NeuronLayer::backPropagate(NeuronLayer &nextLayer)
{
	for(unsigned int i = 0; i < this->m_neurons.size(); i++)
	{
		float error = 0.0f;

		for(unsigned int j = 0; j < nextLayer.m_neurons.size(); j++)
		{
			error += nextLayer.m_neurons[j].m_weights[i] * nextLayer.m_neurons[j].m_error;
		}		

		this->m_neurons[i].m_error = this->m_neurons[i].m_output * error;
	}	
}

void NeuronLayer::setOutput(std::vector<float> outputs)
{
	for(unsigned int i = 0; i < this->m_neurons.size(); i++)
	{
		this->m_neurons[i].m_output = outputs[i];
	}
}

void NeuronLayer::getOutput(std::vector<float> &outputs)
{
	outputs.clear();

	for(unsigned int i = 0; i < this->m_neurons.size(); i++)
	{
		outputs.push_back(this->m_neurons[i].m_output);
	}
}

unsigned int NeuronLayer::size()
{
	return this->m_neurons.size();
}

float NeuronLayer::calculateError(std::vector<float> &expectedOutputs)
{
	float totalError = 0.0f;

	for(unsigned int i = 0; i < this->m_neurons.size(); i++)
	{
		float error = expectedOutputs[i] - this->m_neurons[i].m_output;
		this->m_neurons[i].m_error = error;
		totalError += error;
	}

	return totalError;
}

void NeuronLayer::adjustWeights(NeuronLayer &input, float momentum, float learningRate)
{
	for(unsigned int i = 0; i < this->m_neurons.size(); i++)
	{
		for(unsigned int j = 0; j < this->m_neurons[i].m_weights.size(); j++)
		{
			float delta = momentum * this->m_neurons[i].m_lastDelta[j] + (1.0f - momentum) * learningRate * this->m_neurons[i].m_error * input.m_neurons[j].m_output;

			this->m_neurons[i].m_weights[j] += delta;
			this->m_neurons[i].m_lastDelta[j] = delta;

			this->m_neurons[i].m_weights[j] = std::max(this->m_neurons[i].m_weights[j], -100.0f);
			this->m_neurons[i].m_weights[j] = std::min(this->m_neurons[i].m_weights[j], 100.0f);

			this->m_neurons[i].m_lastDelta[j] = std::max(this->m_neurons[i].m_lastDelta[j], -100.0f);
			this->m_neurons[i].m_lastDelta[j] = std::min(this->m_neurons[i].m_lastDelta[j], 100.0f);
		}
	}
}

void NeuronLayer::reset()
{
	for(unsigned int i = 0; i < this->m_neurons.size(); i++)
	{
		this->m_neurons[i].m_error = 0.0f;
		this->m_neurons[i].m_output = 0.0f;

		for(unsigned int j = 0; j < this->m_neurons[i].m_weights.size(); j++)
		{
			this->m_neurons[i].m_weights[j] = 1.0f;
			this->m_neurons[i].m_lastDelta[j] = 0.0f;
		}
	}
}

std::vector<std::vector<float>> NeuronLayer::getWeights() const
{
	std::vector<std::vector<float>> weights;

	for(int i = 0; i < this->m_neurons.size(); i++)
	{
		weights.push_back(this->m_neurons[i].m_weights);
	}

	return weights;
}
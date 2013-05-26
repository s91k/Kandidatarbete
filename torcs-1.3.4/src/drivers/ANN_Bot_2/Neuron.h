#pragma once

#include <vector>

struct Neuron
{
	float m_output;
	float m_error;
	std::vector<float> m_weights;
	std::vector<float> m_lastDelta;

	Neuron(int nrOfInputs)
	{
		this->m_output = (float)(rand() % 10) / 10.0f - 0.5f;
		this->m_error = 0.0f;

		for(int i = 0; i < nrOfInputs; i++)
		{
			this->m_weights.push_back((float)(rand() % 10) / 10.0f - 0.5f);
			this->m_lastDelta.push_back(0.0f);
		}
	}

	Neuron(std::vector<float> weights)
	{
		this->m_output = (float)(rand() % 10) / 10.0f - 0.5f;
		this->m_error = 0.0f;

		for(int i = 0; i < weights.size(); i++)
		{
			this->m_weights.push_back(weights[i]);
			this->m_lastDelta.push_back(0.0f);
		}
	}
};
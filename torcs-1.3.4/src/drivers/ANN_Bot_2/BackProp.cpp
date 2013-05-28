#include "backprop.h"
#include <time.h>
#include <stdlib.h>


//	initializes and allocates memory on heap
CBackProp::CBackProp(int numLayers,int *sz,double b,double a):beta(b),alpha(a)
{
	//	set no of layers and their sizes
	this->zNumLayers = numLayers;
	layerSize = new int[zNumLayers];
	int i;

	for(i = 0; i < zNumLayers; i++)
	{
		layerSize[i] = sz[i];
	}

	//	allocate memory for output of each neuron
	out = new double*[zNumLayers];

	for( i = 0; i < zNumLayers; i++)
	{
		out[i] = new double[layerSize[i]];
	}

	//	allocate memory for delta
	delta = new double*[zNumLayers];

	for(i = 1; i < zNumLayers; i++)
	{
		delta[i] = new double[layerSize[i]];
	}

	//	allocate memory for weights
	weight = new double**[zNumLayers];

	for(i = 1; i < zNumLayers; i++)
	{
		weight[i] = new double*[layerSize[i]];
	}
	for(i = 1; i < zNumLayers; i++)
	{
		for(int j = 0; j < layerSize[i]; j++)
		{
			weight[i][j] = new double[layerSize[i-1] + 1];
		}
	}

	//	allocate memory for previous weights
	prevDwt = new double**[zNumLayers];

	for(i = 1; i < zNumLayers; i++)
	{
		prevDwt[i] = new double*[layerSize[i]];

	}
	for(i = 1; i < zNumLayers; i++)
	{
		for(int j = 0; j < layerSize[i]; j++)
		{
			prevDwt[i][j]=new double[layerSize[i-1]+1];
		}
	}

	//	seed and assign random weights
	srand((unsigned)(time(NULL)));

	for(i = 1; i < zNumLayers; i++)
		for(int j = 0; j < layerSize[i]; j++)
			for(int k = 0; k < layerSize[i-1] + 1; k++)
				weight[i][j][k] = (double)(rand()) / (RAND_MAX / 2) - 1;//32767

	//	initialize previous weights to 0 for first iteration
	for(i = 1; i < zNumLayers; i++)
		for(int j = 0; j < layerSize[i]; j++)
			for(int k = 0; k < layerSize[i-1] + 1; k++)
				prevDwt[i][j][k] = (double)0.0;

// Note that the following variables are unused,
//
// delta[0]
// weight[0]
// prevDwt[0]

//  I did this intentionaly to maintains consistancy in numbering the layers.
//  Since for a net having n layers, input layer is refered to as 0th layer,
//  first hidden layer as 1st layer and the nth layer as output layer. And 
//  first (0th) layer just stores the inputs hence there is no delta or weigth
//  values corresponding to it.
}



CBackProp::~CBackProp()
{
	int i;
	//	free out
	for(i = 0; i < zNumLayers; i++)
		delete[] out[i];
	delete[] out;

	//	free delta
	for(i=1;i<zNumLayers;i++)
		delete[] delta[i];
	delete[] delta;

	//	free weight
	for(i=1;i<zNumLayers;i++)
		for(int j=0;j<layerSize[i];j++)
			delete[] weight[i][j];
	for(i=1;i<zNumLayers;i++)
		delete[] weight[i];
	delete[] weight;

	//	free prevDwt
	for(i=1;i<zNumLayers;i++)
		for(int j=0;j<layerSize[i];j++)
			delete[] prevDwt[i][j];
	for(i=1;i<zNumLayers;i++)
		delete[] prevDwt[i];
	delete[] prevDwt;

	//	free layer info
	delete[] layerSize;
}

//	sigmoid function
double CBackProp::sigmoid(double in)
{
	return (double)(1 / (1 + exp(-in)));
}

//	mean square error
double CBackProp::mse(double *tgt) const
{
	double mse=0;
	for(int i = 0; i < layerSize[zNumLayers-1]; i++)
	{
		mse += (tgt[i] - out[zNumLayers-1][i]) * (tgt[i] - out[zNumLayers-1][i]);
	}
	return mse / 2;
}


//	returns i'th output of the net
double CBackProp::Out(int i) const
{
	return out[zNumLayers - 1][i];
}

// feed forward one set of input
void CBackProp::ffwd(double *in)
{
	double sum;
	int i;
	//	assign content to input layer
	for(i = 0; i < layerSize[0]; i++)
		out[0][i] = in[i];  // output_from_neuron(i,j) Jth neuron in Ith Layer

	//	assign output(activation) value 
	//	to each neuron usng sigmoid func
	for(i = 1; i < zNumLayers; i++)
	{				// For each layer
		for(int j = 0; j < layerSize[i]; j++)
		{		// For each neuron in current layer
			sum=0.0;
			for(int k = 0; k < layerSize[i - 1]; k++)
			{		// For input from each neuron in preceeding layer
				sum += out[i - 1][k] * weight[i][j][k];	// Apply weight to inputs and add to sum
			}
			sum += weight[i][j][layerSize[i - 1]];		// Apply bias
			out[i][j] = sigmoid(sum);				// Apply sigmoid function
		}
	}
}


//	backpropogate errors from output
//	layer uptill the first hidden layer
void CBackProp::bpgt(double *in,double *tgt)
{
	double sum;
	int i;

	//	update output values for each neuron
	ffwd(in);
	
	//	find delta for output layer
	for(i = 0; i < layerSize[zNumLayers - 1]; i++)
	{
		delta[zNumLayers - 1][i] = out[zNumLayers - 1][i] *
		(1 - out[zNumLayers - 1][i]) * (tgt[i] - out[zNumLayers - 1][i]);
	}

	//	find delta for hidden layers	
	for(i = zNumLayers - 2; i > 0; i--)
	{
		for(int j = 0; j < layerSize[i]; j++)
		{
			sum=0.0;
			int size = layerSize[i + 1];
			for(int k = 0; k < size;k++)
			{
				sum+=delta[i + 1][k] * weight[i + 1][k][j];
			}
			delta[i][j] = out[i][j] * (1 - out[i][j]) * sum;
		}
	}

	//	apply momentum ( does nothing if alpha=0 )
	for(i = 1; i < zNumLayers; i++)
	{
		for(int j = 0; j < layerSize[i]; j++)
		{
			for(int k = 0; k < layerSize[i - 1]; k++)
			{
				weight[i][j][k] += alpha * prevDwt[i][j][k];
			}
			weight[i][j][layerSize[i - 1]] += alpha * prevDwt[i][j][layerSize[i - 1]];
		}
	}

	//	adjust weights using steepest descent	
	for(i = 1; i < zNumLayers; i++)
	{
		for(int j = 0; j<layerSize[i]; j++)
		{
			for(int k = 0; k < layerSize[i - 1]; k++)
			{
				prevDwt[i][j][k] = beta * delta[i][j] * out[i - 1][k];
				weight[i][j][k] += prevDwt[i][j][k];
			}
			prevDwt[i][j][layerSize[i-1]] = beta * delta[i][j];
			weight[i][j][layerSize[i-1]] += prevDwt[i][j][layerSize[i - 1]];
		}
	}
}

void CBackProp::WriteWeights( const char* filename )
{
	FILE* pFile;
	fopen_s(&pFile, filename, "w");
	for (int i = 1; i < zNumLayers; i++)
	{
		for (int j = 0; j < layerSize[i]; j++)
		{
			for (int k = 0; k < layerSize[i - 1]; k++)
			{
				fprintf_s(pFile, "%f \n", weight[i][j][k]);
			}
		}
	}
	fclose(pFile);
}

void CBackProp::ReadWeights( const char* filename )
{
	FILE* pFile;
	int res = fopen_s(&pFile, filename, "r");
	if (res == 0)
		printf_s("Reading Weights. \n");
	else if(res == EINVAL || pFile == NULL)
	{
		printf_s("Failed Reading Weights. \n");
		return;
	}
	

	for (int i = 1; i < zNumLayers; i++)
	{
		for (int j = 0; j < layerSize[i]; j++)
		{
			for (int k = 0; k < layerSize[i - 1]; k++)
			{
				fscanf_s(pFile, "%f ", &weight[i][j][k]);
			}
		}
	}
	fclose(pFile);
}

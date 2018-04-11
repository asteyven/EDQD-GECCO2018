/**
 * @file
 * @author Leo Cazenille <leo.cazenille@upmc.fr>
 *
 *
 */

#include <neuralnetworks/LayeredNeuralNetwork.h>
#include <sstream>
#include <iostream>

using namespace Neural;


/* --------------------- LayeredNeuralNetwork --------------------- */

LayeredNeuralNetwork::~LayeredNeuralNetwork() {
	// ...
}

LayeredNeuralNetwork::LayeredNeuralNetwork(
		std::vector<double>& weights,
		unsigned int nbInputs,
		unsigned int nbOutputs,
		bool activeBias,
		bool onlyUseBiasForFirstHiddenLayer,
		double biasValue) :
		NeuralNetwork(weights, nbInputs, nbOutputs), _activeBias(activeBias),
		_onlyUseBiasForFirstHiddenLayer(onlyUseBiasForFirstHiddenLayer),
		_biasValue(biasValue)
{
    std::vector<unsigned int>* nbNeuronsPerHiddenLayers = new std::vector<unsigned int>(0);
	initNbNeuronsPerLayer(*nbNeuronsPerHiddenLayers);
    delete nbNeuronsPerHiddenLayers;
}

LayeredNeuralNetwork::LayeredNeuralNetwork(
		std::vector<double>& weights,
		unsigned int nbInputs,
		unsigned int nbOutputs,
		std::vector<unsigned int>& nbNeuronsPerHiddenLayer = *(new std::vector<unsigned int>(0)),
		bool activeBias,
		bool onlyUseBiasForFirstHiddenLayer,
		double biasValue) :
		NeuralNetwork(weights, nbInputs, nbOutputs), _activeBias(activeBias),
		_onlyUseBiasForFirstHiddenLayer(onlyUseBiasForFirstHiddenLayer),
		_biasValue(biasValue) {
	initNbNeuronsPerLayer(nbNeuronsPerHiddenLayer);
}


LayeredNeuralNetwork::LayeredNeuralNetwork(LayeredNeuralNetwork const& other) : NeuralNetwork(other) {
	_nbNeuronsPerLayer = other._nbNeuronsPerLayer;
	_activeBias = other._activeBias;
	_onlyUseBiasForFirstHiddenLayer = other._onlyUseBiasForFirstHiddenLayer;
	_biasValue = other._biasValue;
}

void LayeredNeuralNetwork::initNbNeuronsPerLayer(std::vector<unsigned int>& nbNeuronsPerHiddenLayer) {
	_nbNeuronsPerLayer = std::vector<unsigned int>(nbNeuronsPerHiddenLayer.size() + 2);
	_nbNeuronsPerLayer[0] = _nbInputs;
	for(size_t i = 0; i < nbNeuronsPerHiddenLayer.size(); i++)
		_nbNeuronsPerLayer[1 + i] = nbNeuronsPerHiddenLayer[i];
	_nbNeuronsPerLayer[nbNeuronsPerHiddenLayer.size() + 1] = _nbOutputs;
}


std::string LayeredNeuralNetwork::toString() const {
	std::stringstream ss;
	ss << "nn(";

	ss << _nbInputs;
	if(_activeBias)
		ss << "(+1)";

	for(size_t i = 1; i < _nbNeuronsPerLayer.size(); i++) {
		ss << ";";
		ss << _nbNeuronsPerLayer[i];
		ss << "[";
		int nbConnexions = _nbNeuronsPerLayer[i - 1];
		if(_activeBias && !_onlyUseBiasForFirstHiddenLayer)
			++nbConnexions;
		ss <<  nbConnexions;
		ss << "]";
	}

	ss << ")";
	return ss.str();
}

bool LayeredNeuralNetwork::getActiveBias() const {
	return _activeBias;
}

bool LayeredNeuralNetwork::getOnlyUseBiasForFirstHiddenLayer() const {
	return _onlyUseBiasForFirstHiddenLayer;
}

double LayeredNeuralNetwork::getBiasValue() const {
	return _biasValue;
}



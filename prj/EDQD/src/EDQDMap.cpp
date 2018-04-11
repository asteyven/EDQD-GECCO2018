/*
 * EDQDMap.cpp
 *
 *  Created on: 18 Dec 2017
 *      Author: andreas
 */

#include "EDQD/include/EDQDMap.h"
#include "RoboroboMain/roborobo.h"



EDQDMap::EDQDMap() : _map_dim(EDQDSharedData::gEDQDNbOfDimensions) {
//	_map.resize(boost::extents[EDQDSharedData::gEDQDNbOfIntervals][EDQDSharedData::gEDQDNbOfIntervals]);
	_mapHasEntry = false;
	_num_filled_cells = 0;

	for (size_t i = 0; i < _map_dim; ++i) {
		_map_shape[i] = EDQDSharedData::gEDQDNbOfIntervals;
	}
	_map.resize(_map_shape);
}

EDQDMap::~EDQDMap() {
	for(size_t i = 0; i < _map.num_elements(); i++){
		_map.data()[i].genome.clear();
		_map.data()[i].pos.clear();
	}
}



EDQD::behav_index_t EDQDMap::computeIndex(const std::map<int, int>& oc, double distance, double maxDistance, std::vector<double>* pos) {
	double _dim1 = 0.5, _dim2 = 0.0;

	int _oc0 = oc.at(0),
		_oc1 = oc.at(1);

	if ( oc.at(1) > 0.0 ) {
		_dim1 = _oc0 / ((double)(_oc0 + _oc1));
	} else if ( oc.at(0) > 0.0 ) {
		_dim1 = 1.0;
	}
	_dim1 *= ((double)EDQDSharedData::gEDQDNbOfIntervals - 0.5);

	if ( distance > 0 ) {
		_dim2 = ( (distance / ((double)maxDistance)) * ( (double)EDQDSharedData::gEDQDNbOfIntervals - 0.5 ) );
		if ( _dim2 < 0.0 ) { _dim2 = 0; }
		if ( _dim2 > EDQDSharedData::gEDQDNbOfIntervals - 1 ) { _dim2 = (double)EDQDSharedData::gEDQDNbOfIntervals - 0.5; }
	}

//	std::cout << _dim1 << " " << _dim2 << " "  << (long int)_dim1 << " " << (long int)_dim2 << std::endl; // DEBUG
	if (pos) {
		(*pos)[0] = _dim1;
		(*pos)[1] = _dim2;
	}

	EDQD::behav_index_t _index = {{(long int)_dim1,(long int)_dim2}};

	return _index;
}


bool EDQDMap::add( int id,
							EDQDController* ctrl,
							std::vector<double> genome,
							float sigma ) {
	std::vector<double>* pos = new std::vector<double>(2);
	EDQD::behav_index_t _index =
			EDQDMap::computeIndex(
					(*ctrl).getObjectCounters(),
					(*ctrl).getMaxTravelled(),
					(*ctrl).getPotMaxTravelled(),
					pos
			);

	if (_map(_index).fitness == -1
		|| (ctrl->getFitness() - _map(_index).fitness) > EDQDSharedData::gEDQDFitEpsilon
		|| (fabs(ctrl->getFitness() - _map(_index).fitness) <= EDQDSharedData::gEDQDFitEpsilon
				&& _dist_center((*pos)) < _dist_center(_map(_index).pos))
	) {
		if (_map(_index).fitness == -1) {
			_num_filled_cells++;
		}

		_map(_index).fitness = ctrl->getFitness();
		_map(_index).genome.clear();
		_map(_index).genome = genome;
		_map(_index).id = std::make_pair(id,ctrl->getBirthdate());
		_map(_index).sigma = sigma;
		_map(_index).pos.clear();
		_map(_index).pos = (*pos);

		setMapHasEntry(true);
		return true;
	}
	return false;
}

EDQD::behav_index_t EDQDMap::getRandomIndex() {
	EDQD::behav_index_t _index = {{(long int)(random() * EDQDSharedData::gEDQDNbOfIntervals),
									(long int)(random() * EDQDSharedData::gEDQDNbOfIntervals)}};
	return _index;
}

bool EDQDMap::mergeInto(EDQDMap &m) {
	bool __result = false;
	for (size_t i = 0; i < this->_map.num_elements(); i ++) {
		if ( m.getMap().data()[i].fitness == -1 ||
				m.getMap().data()[i] < this->_map.data()[i] )
		{
			m.getMap().data()[i] = this->_map.data()[i];
			__result = true;
		}
	}
	if (__result) {
		m.updateNumFilledCells();
	}
	return __result;
}

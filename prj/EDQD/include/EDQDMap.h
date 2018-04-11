/*
 * EDQDMap.h
 *
 *  Created on: 18 Dec 2017
 *      Author: andreas
 */

#ifndef EDMAPELITESMAP_H_
#define EDMAPELITESMAP_H_

#include "RoboroboMain/common.h"
//#include <boost/multi_array.hpp>
#include <boost/optional.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/utility.hpp>
#include <boost/serialization/array.hpp>
//#include <boost/serialization/base_object.hpp>
#include <boost/serialization/split_member.hpp>
//#include <boost/serialization/split_free.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
//#include <boost/archive/archive_exception.hpp>
#include "EDQD/include/EDQDController.h"
#include "EDQD/include/EDQDSharedData.h"
//#include "EDQD/include/EDQDRobotWorldModel.h"

class EDQDMap;
class EDQDController;

namespace EDQD {


	struct Entry {
	private:
	public:
		Entry(){}

		Entry(Entry* o) {
			this->fitness = o->fitness;
			this->genome.clear();
			this->genome = o->genome;
			this->id = o->id;
			this->sigma = o->sigma;
			this->pos.clear();
			this->pos = o->pos;
		}

		Entry(double fitness, std::vector<double> genome, std::pair<int, int> id, float sigma, std::vector<double> pos) {
			this->fitness = fitness;
			this->genome.clear();
			this->genome = genome;
			this->id = id;
			this->sigma = sigma;
			this->pos.clear();
			this->pos = pos;
		}

		double fitness = -1.0;
		std::vector<double> genome;
		std::pair<int, int> id;
		float sigma = EDQDSharedData::gSigmaRef;
		std::vector<double> pos;
		int counter = 0;

		inline bool operator< (const EDQD::Entry& rhs){
			return this->fitness < rhs.fitness ||
					( this->fitness == rhs.fitness && _dist_center(this->pos) > _dist_center(rhs.pos) ) ;
		}
		inline bool operator> (const EDQD::Entry& rhs){
			return rhs.fitness < this->fitness ||
					( this->fitness == rhs.fitness && _dist_center(rhs.pos) > _dist_center(this->pos) );
		}
		inline bool operator<=(const EDQD::Entry& rhs){
			return !(rhs.fitness < this->fitness ||
					( this->fitness == rhs.fitness && _dist_center(rhs.pos) > _dist_center(this->pos) ));
		}
		inline bool operator>=(const EDQD::Entry& rhs){
			return !(this->fitness < rhs.fitness ||
					( this->fitness == rhs.fitness && _dist_center(this->pos) > _dist_center(rhs.pos) ));
		}
		inline friend std::ostream& operator<<(std::ostream& os, const EDQD::Entry& obj){
			// write obj to stream
			os << obj.fitness;
			return os;
		};

		template <class Archive>
		void serialize(Archive& ar, const unsigned int version) {
			ar& BOOST_SERIALIZATION_NVP(fitness);
			ar& BOOST_SERIALIZATION_NVP(genome);
			ar& BOOST_SERIALIZATION_NVP(id);
			ar& BOOST_SERIALIZATION_NVP(sigma);
			ar& BOOST_SERIALIZATION_NVP(pos);
			ar& BOOST_SERIALIZATION_NVP(counter);
		}

		float _dist_center(const std::vector<double>& pos) {
			/* Returns distance to center of behavior descriptor cell */
			float dist = 0.0;
			for(size_t i = 0; i < pos.size(); ++i) {
				dist += pow(pos[i] - (float)round(pos[i] * (float)(EDQDSharedData::gEDQDNbOfIntervals - 1))/(float)(EDQDSharedData::gEDQDNbOfIntervals - 1), 2);
			}
			dist=sqrt(dist);
			return dist;
		}

		void incCounter() {
			this->counter++;
		}
	};

//	typedef typename std::map<std::pair<int,int>, EDQD::Entry> map_t;
	typedef boost::multi_array<EDQD::Entry, EDQDSharedData::gEDQDNbOfDimensions> map_t;
	typedef map_t::index map_index_t;
	typedef boost::array<map_index_t, EDQDSharedData::gEDQDNbOfDimensions> behav_index_t;



}  // namespace EDQD


class EDQDMap {

	friend class boost::serialization::access;

protected:
    EDQD::map_t _map;
    bool _mapHasEntry;
    size_t _map_dim;
	EDQD::behav_index_t _map_shape;

	int _num_filled_cells = 0;

//    std::pair<int,int> computeIndex(const std::map<int, int>& oc, double distance, int age) {

	void setMapHasEntry(bool mapHasEntry) {
		_mapHasEntry = mapHasEntry;
	}

	float _dist_center(const std::vector<double>& pos) {
		/* Returns distance to center of behavior descriptor cell */
		float dist = 0.0;
		for(size_t i = 0; i < _map_dim; ++i) {
			dist += pow(pos[i] - (float)round(pos[i] * (float)(_map.shape()[i] - 1))/(float)(_map.shape()[i] - 1), 2);
		}
		dist=sqrt(dist);
		return dist;
	}

public:

	EDQDMap();
	virtual ~EDQDMap();

	static EDQD::behav_index_t computeIndex(const std::map<int, int>& oc, double distance, double maxDistance, std::vector<double>* pos = NULL);

	//Setter
	EDQD::Entry& operator()(int i, int j) {
		return _map[i][j];
	}
	//getter
	EDQD::Entry operator()(int i, int j) const {
		return _map[i][j];
	}


	inline friend std::ostream& operator<<(std::ostream& os, const EDQDMap& obj) {
		// write obj to stream
		int __dim1 = obj._map.shape()[0];
		int __dim2 = obj._map.shape()[2];
		for (EDQD::map_index_t i = 0; i < __dim1; i++)
		{
			os << i;
			for (EDQD::map_index_t j = 0; j < __dim2; j++) {
				os << "," << (obj._map[i][j]);
			}
			if ( i < __dim1 - 1 ){ os << std::endl; }
		}

		return os;
	};

	friend class boost::serialization::access;
	BOOST_SERIALIZATION_SPLIT_MEMBER()

	template <class Archive>
	void load(Archive& ar, const int version) {
		ar& BOOST_SERIALIZATION_NVP(_mapHasEntry);
		ar& BOOST_SERIALIZATION_NVP(_map_dim);
//		ar& BOOST_SERIALIZATION_NVP(behav_shape);
//		boost::array<std::size_t, EDQDSharedData::gEDQDNbOfDimensions> shape;
//		ar & shape;
		for (size_t i = 0; i < _map_dim; i++) {
			ar& BOOST_SERIALIZATION_NVP(_map_shape[i]);
		}

		_map.resize(_map_shape);
		ar & boost::serialization::make_array(_map.data(), _map.num_elements());
	}

	template <class Archive>
	void save(Archive& ar, const int version) const {
		ar& BOOST_SERIALIZATION_NVP(_mapHasEntry);
		ar& BOOST_SERIALIZATION_NVP(_map_dim);
//		ar& BOOST_SERIALIZATION_NVP(behav_shape);
//		ar & boost::serialization::make_array(_map.shape(), _map.num_dimensions());
		for (size_t i = 0; i < _map_dim; i++) {
			ar& BOOST_SERIALIZATION_NVP(_map_shape[i]);
		}
		ar & boost::serialization::make_array(_map.data(), _map.num_elements());
	}



	bool add( int id,
				EDQDController* ctrl,
				std::vector<double> genome,
				float sigma );

	EDQD::Entry* get(EDQD::behav_index_t index) {
		return &_map(index);
	}

	EDQD::behav_index_t getRandomIndex();

	bool hasEntry() const {
		return _mapHasEntry;
	}

	EDQD::map_t& getMap() {
		return _map;
	}

	int getNumFilledCells() const
	{
		return _num_filled_cells;
	}

	bool mergeInto(EDQDMap& m);

	void updateNumFilledCells() {
		_num_filled_cells=0;
		for (size_t i = 0; i < _map.num_elements(); i++) {
			if (_map.data()[i].fitness > -1.0) {
				_num_filled_cells++;
				setMapHasEntry(true);
			}
		}
	}
};


namespace EDQD {


	static void writeMapBin(const EDQDMap& m, const std::string& filename) {
		std::ofstream ofs(filename.c_str());
		assert(ofs.good());
		std::cout << "writing : " << filename << std::endl;
		boost::archive::binary_oarchive oa(ofs);
		oa& BOOST_SERIALIZATION_NVP(m);
		std::cout << "done" << std::endl;
	};

	static EDQDMap loadMap(const std::string& filename) {
		std::ifstream ifs(filename.c_str());
		assert(ifs.good());
		boost::archive::binary_iarchive ia(ifs);
		EDQDMap data;
		ia& BOOST_SERIALIZATION_NVP(data);
		return data;
	}

	static std::string mapToString(const EDQD::map_t& m, const std::string& prefix = std::string("")) {
		std::stringstream ofs;

		size_t behav_dim = m.dimensionality;
		size_t offset = 0;
		for (const EDQD::Entry* i = m.data(); i < (m.data() + m.num_elements()); ++i) {
			if (i) {
				behav_index_t _index;
				for (unsigned int dim = 0; dim < behav_dim; dim++ ) {
				  _index[dim] = (offset / m.strides()[dim] % m.shape()[dim] +  m.index_bases()[dim]);
				}

				assert(m(_index).fitness == (i)->fitness);

				ofs << prefix << offset << ",";
				for (size_t dim = 0; dim < behav_dim; ++dim) {
					ofs << _index[dim] / (float)m.shape()[dim] << ",";
				}
				ofs << m(_index).fitness;
				ofs << std::endl;
			}
			++offset;
		}

		return ofs.str();
	}

	static void writeMapToFile(const EDQD::map_t& m,
		const std::string& filename,
		const std::string& note = "")
	{
		std::cout << "writing " << note << " " << filename << std::endl;

		std::ofstream ofs(filename.c_str());

		ofs << EDQD::mapToString(m);
	}



}  // namespace EDQD

#endif /* EDMAPELITESMAP_H_ */

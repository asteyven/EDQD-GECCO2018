/*
 * EDQDRobotWorldModel.h
 *
 *  Created on: 18 Dec 2017
 *      Author: andreas
 */

#ifndef EDMAPELITESROBOTWORLDMODEL_H_
#define EDMAPELITESROBOTWORLDMODEL_H_



#include "WorldModels/RobotWorldModel.h"
//#include "RoboroboMain/roborobo.h"
#include "EDQD/include/EDQDMap.h"



class EDQDRobotWorldModel: public RobotWorldModel {
//protected:

//    std::map<int,int> _objectCounters;

public:
	EDQDRobotWorldModel();
	virtual ~EDQDRobotWorldModel();

//	EDQDMap *_map;

//	const std::map<int,int>& getObjectCounters() const {
//		return _objectCounters;
//	}
//
//	void incObjectCounter(int group) {
//		_objectCounters[group]++;
//	}
//
//	void resetObjectCounter() {
//		for (auto &pog : gPhysicalObjectGroups)
//		{
//			_objectCounters[pog->getId()] = 0;
//		}
//	}
};


#endif /* EDMAPELITESROBOTWORLDMODEL_H_ */

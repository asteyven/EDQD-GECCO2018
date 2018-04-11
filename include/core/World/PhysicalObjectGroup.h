/*
 *  PhysicalObject.h
 *  roborobo
 *
 *  Created by Andreas Steyven on 07/12/2017.
 *
 */

#ifndef PHYSICALOBJECTGROUP_H
#define PHYSICALOBJECTGROUP_H

#include "RoboroboMain/common.h"
#include "Utilities/Geometry.h"
#include "World/PhysicalObject.h"

class PhysicalObjectGroup
{
protected :
    int _id;
    int _type;
    
    void init();  // called by constructor only

    int nbOfObjects;
	int initAreaWidth;
	int initAreaHeight;
	int initAreaX;
	int initAreaY;
	int initDistribution;
	int initAreaShape;

	std::vector<PhysicalObject*> groupObjects;

    void setType ( int __type ) { _type = __type; }

public :
    
    PhysicalObjectGroup( int __id ); // use PhysicalObjectFactory instead!
    ~PhysicalObjectGroup();

    void initObjects();


    int getId()
    {
        return _id;
    }

    int getType() const { return _type; }

	int getInitAreaHeight() const {
		return initAreaHeight;
	}

	int getInitAreaShape() const {
		return initAreaShape;
	}

	int getInitAreaWidth() const {
		return initAreaWidth;
	}

	int getInitAreaX() const {
		return initAreaX;
	}

	int getInitAreaY() const {
		return initAreaY;
	}

	int getInitDistribution() const {
		return initDistribution;
	}

	int getNbOfObjects() const {
		return nbOfObjects;
	}

	void setInitAreaHeight(int initAreaHeight) {
		this->initAreaHeight = initAreaHeight;
	}

	void setInitAreaShape(int initAreaShape) {
		this->initAreaShape = initAreaShape;
	}

	void setInitAreaWidth(int initAreaWidth) {
		this->initAreaWidth = initAreaWidth;
	}

	void setInitAreaX(int initAreaX) {
		this->initAreaX = initAreaX;
	}

	void setInitAreaY(int initAreaY) {
		this->initAreaY = initAreaY;
	}

	void setInitDistribution(int initDistribution) {
		this->initDistribution = initDistribution;
	}

	void setNbOfObjects(int nbOfObjects) {
		this->nbOfObjects = nbOfObjects;
	}

	const std::vector<PhysicalObject*>& getGroupObjects() const {
		return groupObjects;
	}

	void addPhysicalObject(PhysicalObject *object)
	{
		groupObjects.push_back(object);
	}

};

#endif

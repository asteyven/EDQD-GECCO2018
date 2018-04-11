#include "World/PhysicalObjectFactory.h"
#include "World/RoundObject.h"
#include "World/EnergyItem.h"
#include "World/GateObject.h"
#include "World/SwitchObject.h"
#include "World/MovableObject.h"
#include "RoboroboMain/roborobo.h"
#include "Utilities/Misc.h"


int PhysicalObjectFactory::_nextId = 0;

void PhysicalObjectFactory::makeObject( int group, int type )
{
    int id = PhysicalObjectFactory::getNextId();
    
    if ( group == -1 ) {
		std::string s = "";
		std::stringstream out;
		out << id;

		s = "physicalObject[";
		s += out.str();
		s += "].type";
		if ( gProperties.hasProperty( s ) )
		{
			convertFromString<int>(type, gProperties.getProperty( s ), std::dec);
		}
		else
		{
			if ( gVerbose )
				std::cerr << "[MISSING] PhysicalObjectFactory: object #" << id << ", type is missing. Assume type "<< gPhysicalObjectDefaultType << "." << std::endl;
			type = gPhysicalObjectDefaultType;
		}
    }
    else
    {
    	type = gPhysicalObjectGroups[group]->getType();
    }
    switch ( type )
    {
        case 0:
            if ( gVerbose )
                std::cout << "[INFO] Round Object created (type = " << type << ", group = " << group << ").\n";
            gPhysicalObjects.push_back( new RoundObject(id, group) );
            break;
        case 1:
            if ( gVerbose )
                std::cout << "[INFO] Energy Item created (type = " << type << ", group = " << group << ").\n";
            gPhysicalObjects.push_back( new EnergyItem(id, group) );
            break;
        case 2:
            if ( gVerbose )
                std::cout << "[INFO] Gate Object created (type = " << type << ", group = " << group << ").\n";
            gPhysicalObjects.push_back( new GateObject(id, group) );
            break;
        case 3:
            if ( gVerbose )
                std::cout << "[INFO] Switch Object created (type = " << type << ", group = " << group << ").\n";
            gPhysicalObjects.push_back( new SwitchObject(id, group) );
            break;
        case 4:
            if ( gVerbose )
                std::cout << "[INFO] Movable Object created (type = " << type << ", group = " << group << ").\n";
            gPhysicalObjects.push_back( new MovableObject(id, group) );
            break;
        // case ...: DO NOT FORGET TO UPDATE getNbOfTypes() method.
        default:
            std::cerr << "[CRITICAL] PhysicalObjectFactory: object #" << id << ", group " << group << ", type unknown (" << type << ")" << std::endl;
            exit(-1);
    }
}

int PhysicalObjectFactory::getNbOfTypes()
{
    return 5;
}


int PhysicalObjectFactory::getNextId()
{
    int retValue = _nextId;
    _nextId++;
    return retValue;
}

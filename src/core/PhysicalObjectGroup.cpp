#include "World/PhysicalObjectGroup.h"

#include "RoboroboMain/roborobo.h"
#include "Utilities/Misc.h"
#include "World/World.h"

#include <iomanip>


PhysicalObjectGroup::PhysicalObjectGroup( int __id ) // a unique and consistent __id should be given as argument
{
    _id = __id;
    init();
}

PhysicalObjectGroup::~PhysicalObjectGroup()
{
	for ( int i = 0 ; i < getNbOfObjects() ; i++ )
	{
		gPhysicalObjects.erase(
				std::remove(
						gPhysicalObjects.begin(),
						gPhysicalObjects.end(),
						groupObjects[i] ),
				gPhysicalObjects.end()
		);
	}
}

void PhysicalObjectGroup::init()
{
	std::string s = "";
	std::stringstream sId;
	sId << getId();
    
	s = "physicalObjectGroup[";
	s += sId.str();
	s += "].";


	if ( gProperties.hasProperty( s + "type" ) )
	{
		convertFromString<int>(_type, gProperties.getProperty( s + "type" ), std::dec);
	}
    else
    {
        _type = gPhysicalObjectDefaultType;
    }
    
	if ( gProperties.hasProperty( s + "count" ) )
	{
		convertFromString<int>(nbOfObjects, gProperties.getProperty( s + "count" ), std::dec);
	}
	else
	{
		nbOfObjects = gPhysicalObjectDefaultRegrowTimeMax;
	}

	if ( gProperties.hasProperty( s + "initAreaWidth" ) )
	{
		convertFromString<int>(initAreaWidth, gProperties.getProperty( s + "initAreaWidth" ), std::dec);
	}
	else
	{
		initAreaWidth = gPhysicalObjectDefaultInitAreaWidth;
	}

	if ( gProperties.hasProperty( s + "initAreaHeight" ) )
	{
		convertFromString<int>(initAreaHeight, gProperties.getProperty( s + "initAreaHeight" ), std::dec);
	}
	else
	{
		initAreaHeight = gPhysicalObjectDefaultInitAreaHeight;
	}

	if ( gProperties.hasProperty( s + "initAreaX" ) )
	{
		convertFromString<int>(initAreaX, gProperties.getProperty( s + "initAreaX" ), std::dec);
	}
	else
	{
		initAreaX = gPhysicalObjectDefaultInitAreaX;
	}

	if ( gProperties.hasProperty( s + "initAreaY" ) )
	{
		convertFromString<int>(initAreaY, gProperties.getProperty( s + "initAreaY" ), std::dec);
	}
	else
	{
		initAreaY = gPhysicalObjectDefaultInitAreaY;
	}

	if ( gProperties.hasProperty( s + "initDistribution" ) )
	{
		convertFromString<int>(initDistribution, gProperties.getProperty( s + "initDistribution" ), std::dec);
	}
	else
	{
		initDistribution = gPhysicalObjectDefaultInitDistribution;
	}

	if ( gProperties.hasProperty( s + "initAreaShape" ) )
	{
		convertFromString<int>(initAreaShape, gProperties.getProperty( s + "initAreaShape" ), std::dec);
	}
	else
	{
		initAreaShape = gPhysicalObjectDefaultInitAreaShape;
	}

	// adjust values
	if ( getInitAreaWidth() == -1 )	{ setInitAreaWidth( gAreaWidth - 20 ); }
	if ( getInitAreaHeight() == -1 ) { setInitAreaHeight( gAreaHeight - 20 ); }

	if (
		getInitAreaX() < 0 || getInitAreaX() > gAreaWidth ||
		getInitAreaY() < 0 || getInitAreaY() > gAreaHeight ||
		getInitAreaWidth() <= 0 || getInitAreaWidth() > getInitAreaX() + gAreaWidth ||
		getInitAreaHeight() <= 0 || getInitAreaHeight() > getInitAreaY() + gAreaHeight
		)
	{
		std::cerr << "[ERROR] Incorrect values for *InitArea* parameters for objects in group " << getId() << ".\n";
		exit(-1);
	}
}

void PhysicalObjectGroup::initObjects()
{
	for ( int i = 0 ; i < getNbOfObjects() ; i++ )
	{
		// call the factory and pass the group.
		PhysicalObjectFactory::makeObject( getId() );

		addPhysicalObject( gPhysicalObjects.back() );
	}
}


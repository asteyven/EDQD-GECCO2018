/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 *
 */

#include "World/PhysicalObject.h"
#include "RoboroboMain/roborobo.h"
#include "Utilities/Misc.h"
#include "World/World.h"
#include "World/PhysicalObjectGroup.h"

PhysicalObject::PhysicalObject( int __id, int __group ) // a unique and consistent __id should be given as argument
{
    _id = __id;
    _group = __group;
    if (__group > -1)
		_isPartOfGroup = true;
    else
    	_isPartOfGroup = false;

    init();

    gNbOfPhysicalObjects++;
}

PhysicalObject::~PhysicalObject()
{
	gNbOfPhysicalObjects--;
}

void PhysicalObject::init()
{
    double x = 0.0, y = 0.0;
    
	std::string s = "";
	std::stringstream out;
    
	if ( !_isPartOfGroup )
	{
		out << getId();
		s = "physicalObject[";
	}
	else
	{
		out << getGroup();
		s = "physicalObjectGroup[";
	}
	s += out.str();
	s += "].";

    regrowTime = 0;
    
    if ( gProperties.hasProperty( s + "regrowTimeMax" ) )
	{
		convertFromString<int>(regrowTimeMax, gProperties.getProperty( s + "regrowTimeMax" ), std::dec);
	}
    else
    {
        regrowTimeMax = gPhysicalObjectDefaultRegrowTimeMax;
    }
    
	if ( gProperties.hasProperty( s + "overwrite" ) )
        gProperties.checkAndGetPropertyValue(s + "overwrite",&overwrite,true);
    else
    {
        overwrite = gPhysicalObjectDefaultOverwrite;
    }
    
	if ( gProperties.hasProperty( s + "relocate" ) )
        gProperties.checkAndGetPropertyValue(s + "relocate",&relocate,true);
    else
    {
        relocate = gPhysicalObjectDefaultRelocate;
    }
    
	if ( gProperties.hasProperty( s + "visible" ) )
        gProperties.checkAndGetPropertyValue(s + "visible",&_visible,true);
	else
        gProperties.checkAndGetPropertyValue("gPhysicalObjectsVisible", &_visible, true);
    
    Uint32 colorValue;
    
	if ( gProperties.hasProperty( s + "displayColorRed" ) )
	{
		convertFromString<Uint32>(colorValue, gProperties.getProperty( s + "displayColorRed" ), std::dec);
        _displayColorRed = (colorValue & 0x000000FF);
	}
    else
    {
        _displayColorRed = gPhysicalObjectDefaultDisplayColorRed; // default
    }
    
	if ( gProperties.hasProperty( s + "displayColorGreen" ) )
	{
		convertFromString<Uint32>(colorValue, gProperties.getProperty( s + "displayColorGreen" ), std::dec);
        _displayColorGreen = (colorValue & 0x000000FF);
	}
    else
    {
        _displayColorGreen = gPhysicalObjectDefaultDisplayColorGreen; // default
    }
    
	if ( gProperties.hasProperty( s + "displayColorBlue" ) )
	{
		convertFromString<Uint32>(colorValue, gProperties.getProperty( s + "displayColorBlue" ), std::dec);
        _displayColorBlue = (colorValue & 0x000000FF);
	}
    else
    {
        _displayColorBlue = gPhysicalObjectDefaultDisplayColorBlue; // default
    }
    
	if ( ! _isPartOfGroup )
	{
		if ( gProperties.hasProperty( s + "x" ) )
		{
			convertFromString<double>(x, gProperties.getProperty( s + "x" ), std::dec);
		}
		else
		{
			x = -1.0;
		}

		if ( gProperties.hasProperty( s + "y" ) )
		{
			convertFromString<double>(y, gProperties.getProperty( s + "y" ), std::dec);
		}
		else
		{
			y = -1.0;
		}
	}
	else
	{
		x = -1.0, y = -1.0;
	}
	setCoordinates( x, y );

}

int PhysicalObject::findRandomLocation( )
{
    double x = 0.0, y = 0.0, _rndOne = 0.0, _rndTwo = 0.0, __rad = 0.0;
    int tries = 0;

    int __initAreaWidth = gPhysicalObjectsInitAreaWidth,
		__initAreaHeight = gPhysicalObjectsInitAreaHeight,
		__initAreaX = gPhysicalObjectsInitAreaX,
		__initAreaY = gPhysicalObjectsInitAreaY,
		__initDistribution = gPhysicalObjectsInitDistribution,
		__initAreaShape = gPhysicalObjectsInitAreaShape;

	if ( _isPartOfGroup ) {
		PhysicalObjectGroup *__group = gPhysicalObjectGroups[getGroup()];
		__initAreaWidth = __group->getInitAreaWidth();
		__initAreaHeight = __group->getInitAreaHeight();
		__initAreaX = __group->getInitAreaX();
		__initAreaY = __group->getInitAreaY();
		__initDistribution = __group->getInitDistribution();
		__initAreaShape = __group->getInitAreaShape();
	}
	do {
		switch (__initDistribution) {
		// produce random number between -0.5 and 0.5
			case 0: // uniform
				_rndOne = ((double)( randint() % gAreaWidth )) / ((double)gAreaWidth) - 0.5;
				_rndTwo = ((double)( randint() % gAreaHeight )) / ((double)gAreaHeight) - 0.5;
				break;
			case 1: // normal with mean 0 (and we assume that all numbers are within 3sd)
				_rndOne = randgaussian() / 3.0 ;
				_rndTwo = randgaussian() / 3.0 ;
				break;
			default:
				std::cerr << "[ERROR] no valid distribution for PhysicalObject (id:" << getId() << " group:" << getGroup() << ") placement" << std::endl;
				exit(1);
				break;
		}
		switch (__initAreaShape) {
			case 0: // square
				x = (int) ( __initAreaX + __initAreaWidth * ( _rndOne + 0.5 ) );
				y = (int) ( __initAreaY + __initAreaHeight * ( _rndTwo + 0.5 ) );
				break;
			case 1: // elliptic
				__rad = (randint() % 1000) / 1000.0 * 2 * M_PI;
				x = __initAreaX + abs(_rndOne) * (__initAreaWidth / 1.0) * cos( __rad );
				y = __initAreaY + abs(_rndTwo) * (__initAreaHeight / 1.0) * sin( __rad );
				break;
			default:
				std::cerr << "[ERROR] no valid shape for PhysicalObject (id:" << getId() << " group:" << getGroup() << ") placement" << std::endl;
				exit(1);
				break;
		}

		//x = (randint() % (gAreaWidth-20)) + 10;  // deprecated
		//y = (randint() % (gAreaHeight-20)) + 10; // deprecated

		setCoordinates( x, y );

		tries++;
	} while ( canRegister() == false && (tries < gLocationFinderMaxNbOfTrials || overwrite == false) );

	if ( tries == gLocationFinderMaxNbOfTrials && overwrite == false )
    {
        if ( gLocationFinderExitOnFail == true )
        {
            std::cerr << "[CRITICAL] Random initialization of initial position for physical object (id:" << getId() << " group:" << getGroup() << ") after trying " << gLocationFinderMaxNbOfTrials << " random picks (all failed). There may be too few (none?) possible locations (you may try to manually set initial positions). EXITING.\n";
            exit(-1);
        }
        else
        {
            std::cerr << "[WARNING] Random initialization of initial position for physical object (id:" << getId() << " group:" << getGroup() << ") after trying " << gLocationFinderMaxNbOfTrials << " random picks (all failed). Retry later.\n";
            regrowTime = 1;
            setCoordinates( -1, -1 );
        }
    }
    
    return tries;
}

void PhysicalObject::stepPhysicalObject()
{
    if ( registered == false && regrowTime != -1 )
    {
        if ( regrowTime > 0 )
        {
            regrowTime--;
        }
        else
        {
            if ( relocate == true )
            {
                findRandomLocation(); // fail: exit or return (x,y)=(-1,-1)
                if ( getXReal() != -1 ) // check if new location is possible
                {
                    _visible = true;
                    registered = true;
                    registerObject();
                }
            }
            else
            {
                if ( canRegister() == true || overwrite == true )
                {
                    _visible = true;
                    registered = true;
                    registerObject();
                }
            }
        }
    }
    else
    {
        if ( getXReal() != -1 ) // check if (new) location is possible
            if ( _visible )
            {
                if ( gPhysicalObjectsRedraw == true )
                {
                    registerObject();
                }
            }
    }
}

bool PhysicalObject::isInstanceOf ( int index ) // static
{
    if ( index >= gPhysicalObjectIndexStartOffset && index <  gPhysicalObjectIndexStartOffset + (int)gPhysicalObjects.size() )
        return true;
    else
        return false;
}


/*
 Given an object not registered in the environment, tries to re-register at once.
 Return false (and do nothing) if not possible at desired location.
 Exit if function is called when it should not -- calling this function can be critical, including registering multiple instances of the same object, thus breaking consistency.
*/
bool PhysicalObject::triggerRegrow()
{
    if ( registered == true )  // exit if object is already registered in the environment
    {
        std::cerr << "[CRITICAL] physical object #" << getId() << ": attempt to call triggerRegrow() method while object already exists in the environment. EXITING.\n";
        exit(-1);
    }
    
    if ( canRegister() == false ) // check is position is free.
    {
        return false;
    }
    else
    {
        regrowTime = gPhysicalObjectDefaultRegrowTimeMax;
        _visible = true;
        registered = true;
        registerObject();
    }
    
    return true;
}

std::string PhysicalObject::inspect( std::string prefix )
{
    return std::string(prefix + "PhysicalObject::inspect() not implemented.\n");
}

#include "World/EnergyItem.h"
#include "RoboroboMain/roborobo.h"
#include "World/World.h"
#include "WorldModels/RobotWorldModel.h"
#include "Utilities/Misc.h"

EnergyItem::EnergyItem( int __id, int __group ) : CircleObject( __id, __group ) // should only be called by PhysicalObjectFactory
{
    setType(1);

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


	if ( gProperties.hasProperty( s + "energy" ) )
		convertFromString<double>(maxEnergyLevel, gProperties.getProperty( s + "energy" ), std::dec);
	else
    {
        if ( gVerbose )
            std::cerr << "[MISSING] Physical Object #" << _id << " (EnergyItem) missing default energy initial level (integer, >=0). Assume default (" << gEnergyItemDefaultInit << ").\n";
        maxEnergyLevel = gEnergyItemDefaultInit;
    }
    
	if ( gProperties.hasProperty( s + "energyMode" ) )
		convertFromString<int>(energyMode, gProperties.getProperty( s + "energyMode" ), std::dec);
	else
    {
        if ( gVerbose )
            std::cerr << "[MISSING] Physical Object #" << _id << " (EnergyItem) missing default energy mode (integer, >=0). Assume default (" << gEnergyItemDefaultMode << ").\n";
        energyMode = gEnergyItemDefaultMode;
    }
}

void EnergyItem::step()
{
    stepPhysicalObject();
}

void EnergyItem::isTouched( int __idAgent )
{
//    if ( gVerbose && gDisplayMode <= 1)
//        std::cout << "[DEBUG] Physical object #" << this->getId() << " (energy item) touched by robot #" << __idAgent << std::endl;
}

void EnergyItem::isWalked( int __idAgent )
{
//    if ( gVerbose && gDisplayMode <= 1)
//        std::cout << "[DEBUG] Physical object #" << this->getId() << " (energy item) walked upon by robot #" << __idAgent << std::endl;

    double energyRequestedValueByRobot;
    double energyProvided;
    
    switch ( energyMode )
    {
        case 0: // give all
            gWorld->getRobot(__idAgent)->getWorldModel()->addEnergy( maxEnergyLevel );
            regrowTime = regrowTimeMax;
            break;
            
        case 1: // give what is asked, fixed respawn delay
            energyRequestedValueByRobot = gWorld->getRobot(__idAgent)->getWorldModel()->getEnergyRequestValue(); // in [0,1[ (guaranteed)
            energyProvided = energyRequestedValueByRobot * maxEnergyLevel;
            gWorld->getRobot(__idAgent)->getWorldModel()->addEnergy( energyProvided );
            assert ( energyProvided >= 0 );
            regrowTime = regrowTimeMax;
            break;
            
        case 2: // give what is asked, remaining sets respawn delay
            energyRequestedValueByRobot = gWorld->getRobot(__idAgent)->getWorldModel()->getEnergyRequestValue(); // in [0,1[ (guaranteed)
            energyProvided = energyRequestedValueByRobot * maxEnergyLevel;
            gWorld->getRobot(__idAgent)->getWorldModel()->addEnergy( energyProvided );
            assert ( energyProvided >= 0 );
            regrowTime = (int)( (double)regrowTimeMax * ( energyProvided/maxEnergyLevel ) ); // time to regrow is proportionate to energy taken
            break;

        case 3: // give nothing
            regrowTime = regrowTimeMax;
            break;
    }

    registered = false;

    unregisterObject();
    hide();
    _visible = false;
}

void EnergyItem::isPushed( int __id, std::tuple<double, double> __speed )
{
    //    if ( gVerbose && gDisplayMode <= 1)
    //        std::cout << "[DEBUG] Physical object #" << this->getId() << " (energy item) pushed by robot/object #" << __id << std::endl;
}


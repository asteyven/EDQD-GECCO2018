#include "World/SwitchObject.h"
#include "RoboroboMain/roborobo.h"
#include "Utilities/Misc.h"

SwitchObject::SwitchObject( int __id, int __group ) : CircleObject( __id, __group ) // should only be called by PhysicalObjectFactory
{
    setType(3);

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
	if ( gProperties.hasProperty( s + "sendMessageTo" ) )
		convertFromString<int>(sendMessageTo, gProperties.getProperty( s + "sendMessageTo" ), std::dec);
    else
    {
        std::cerr << "[CRITICAL] Physical object #" << _id << " (switch) missing sendMessageTo value (integer, >0).\n";
        exit(-1);
    }
}

void SwitchObject::step()
{
    stepPhysicalObject();
}

void SwitchObject::isTouched( int __idAgent )
{
//    if ( gVerbose && gDisplayMode <= 1)
//        std::cout << "[DEBUG] Physical object #" << this->getId() << " (switch) touched by robot #" << __idAgent << std::endl;
}

void SwitchObject::isWalked( int __idAgent )
{
//    if ( gVerbose && gDisplayMode <= 1)
//        std::cout << "[DEBUG] Physical object #" << this->getId() << " (switch) walked upon by robot #" << __idAgent << std::endl;

    if ( (size_t)sendMessageTo < gPhysicalObjects.size() )
        gPhysicalObjects[sendMessageTo]->isWalked(-1);

    unregisterObject();
    hide();
    _visible = false;

    registered = false;
    regrowTime = regrowTimeMax;
}

void SwitchObject::isPushed( int __id, std::tuple<double, double> __speed )
{
    //    if ( gVerbose && gDisplayMode <= 1)
    //        std::cout << "[DEBUG] Physical object #" << this->getId() << " (switch) pushed by robot/object #" << __idAgent << std::endl;
}

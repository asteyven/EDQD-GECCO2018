/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 *
 */


#include "RoboroboMain/roborobo.h"
#include "RoboroboMain/common.h"
#include "WorldModels/RobotWorldModel.h"
#include "EDQD/include/EDQDAgentObserver.h"
#include "EDQD/include/EDQDRobotWorldModel.h"
#include "EDQD/include/EDQDController.h"
#include "World/World.h"

EDQDAgentObserver::EDQDAgentObserver( EDQDRobotWorldModel *wm )
{
    _wm = wm;
    _r = NULL;
}

EDQDAgentObserver::~EDQDAgentObserver()
{
    // nothing to do.
}

void EDQDAgentObserver::init(Robot *me)
{
	_r = (EDQDRobot*) me;
}

void EDQDAgentObserver::reset()
{
    // nothing to do.
}

void EDQDAgentObserver::step()
{
    // * send callback messages to objects touched or walked upon.
    
    // through distance sensors
    for( int i = 0 ; i < _wm->_cameraSensorsNb; i++)
    {
        int targetIndex = _wm->getObjectIdFromCameraSensor(i);
        
        if ( PhysicalObject::isInstanceOf(targetIndex) )   // sensor ray bumped into a physical object
        {
            targetIndex = targetIndex - gPhysicalObjectIndexStartOffset;
            //std::cout << "[DEBUG] Robot #" << _wm->getId() << " touched " << targetIndex << "\n";
            gPhysicalObjects[targetIndex]->isTouched(_wm->getId());
        }
    }
    
    // through floor sensor
    int targetIndex = _wm->getGroundSensorValue();
    if ( PhysicalObject::isInstanceOf(targetIndex) ) // ground sensor is upon a physical object (OR: on a place marked with this physical object footprint, cf. groundsensorvalues image)
    {
        targetIndex = targetIndex - gPhysicalObjectIndexStartOffset;
        //std::cout << "[DEBUG] #" << _wm->getId() << " walked upon " << targetIndex << "\n";
        gPhysicalObjects[targetIndex]->isWalked(_wm->getId());
        _r->getController()->incObjectCounter(gPhysicalObjects[targetIndex]->getGroup());
    }

    // * update energy if needed
	if ( gEnergyLevel && _wm->isAlive() )
	{
		_wm->substractEnergy(1);
		assert( _wm->getEnergyLevel() >= 0 );
		if ( _wm->getEnergyLevel() == 0 )
			_wm->setAlive(false);
	}

	if( gWorld->getIterations() % EDQDSharedData::gEvaluationTime == 0 )
	{
		// call here: any constructor of AgentObserver, Controller or WorldModel is before robot is positioned which would lead to wrong value.
		_r->getController()->resetPotMaxTravelled();

		if (gWorld->getIterations() > 0) {
			// !!! add needs to happen before new genome is loaded to ensure the map has an entry.
			_r->getController()->getMap()->add(
					_wm->getId(),
					_r->getController(),
					_r->getController()->getCurrentGenome(),
					_r->getController()->getCurrentSigma()
					);
		}
	}

}

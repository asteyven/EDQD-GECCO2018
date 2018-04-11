#if defined PRJ_EDQD || !defined MODULAR

#include "Config/EDQDConfigurationLoader.h"

#include "EDQD/include/EDQDWorldObserver.h"
#include "EDQD/include/EDQDAgentObserver.h"
#include "EDQD/include/EDQDController.h"

#include "WorldModels/RobotWorldModel.h"

EDQDConfigurationLoader::EDQDConfigurationLoader()
{
}

EDQDConfigurationLoader::~EDQDConfigurationLoader()
{
	//nothing to do
}

WorldObserver* EDQDConfigurationLoader::make_WorldObserver(World* wm)
{
	return new EDQDWorldObserver(wm);
}

RobotWorldModel* EDQDConfigurationLoader::make_RobotWorldModel()
{
	return new EDQDRobotWorldModel();
}

AgentObserver* EDQDConfigurationLoader::make_AgentObserver(RobotWorldModel* wm)
{
	return new EDQDAgentObserver( (EDQDRobotWorldModel*) wm );
}

Controller* EDQDConfigurationLoader::make_Controller(RobotWorldModel* wm)
{
	return new EDQDController( (EDQDRobotWorldModel*) wm );
}

#endif

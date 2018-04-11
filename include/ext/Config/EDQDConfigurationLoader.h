/*
 * EDQDConfigurationLoader.h
 */

#ifndef EDMAPELITESCONFIGURATIONLOADER_H
#define EDMAPELITESCONFIGURATIONLOADER_H

#include "Config/ConfigurationLoader.h"

class EDQDConfigurationLoader : public ConfigurationLoader
{
	private:

	public:
		EDQDConfigurationLoader();
		~EDQDConfigurationLoader();

		WorldObserver *make_WorldObserver(World* wm) ;
		RobotWorldModel *make_RobotWorldModel();
		AgentObserver *make_AgentObserver(RobotWorldModel* wm) ;
		Controller *make_Controller(RobotWorldModel* wm) ;
};



#endif

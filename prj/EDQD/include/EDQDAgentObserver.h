/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 *
 */


#ifndef EDMAPELITESAGENTOBSERVER_H
#define EDMAPELITESAGENTOBSERVER_H

#include "RoboroboMain/roborobo.h"
#include "Observers/AgentObserver.h"
#include "EDQD/include/EDQDRobotWorldModel.h"
#include "EDQD/include/EDQDRobot.h"
#include "EDQD/include/EDQDController.h"

//class RobotWorldModel;
class EDQDRobot;
//class World;

class EDQDAgentObserver : public AgentObserver
{
	protected:
		EDQDRobotWorldModel *_wm;
		EDQDRobot *_r;

	public:
		EDQDAgentObserver(EDQDRobotWorldModel *wm);
		~EDQDAgentObserver();

		virtual void reset();
		virtual void step();

		virtual void init(Robot *me);

};

#endif


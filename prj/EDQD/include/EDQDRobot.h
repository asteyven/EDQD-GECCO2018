/*
 * EDQDRobot.h
 *
 *  Created on: 20 Dec 2017
 *      Author: andreas
 */

#ifndef PRJ_EDQD_SRC_EDMAPELITESROBOT_H_
#define PRJ_EDQD_SRC_EDMAPELITESROBOT_H_

#include "Agents/Robot.h"
#include "EDQD/include/EDQDController.h"
#include "EDQD/include/EDQDAgentObserver.h"
#include "EDQD/include/EDQDWorldObserver.h"
#include "EDQD/include/EDQDRobotWorldModel.h"

class EDQDAgentObserver;
//namespace EDQDRobot {

class EDQDRobot : public Robot {
public:
	EDQDRobot( World *__world );
	virtual ~EDQDRobot();

	EDQDController* getController() { return (EDQDController*)Robot::getController(); }
	EDQDAgentObserver* getObserver() { return (EDQDAgentObserver*)Robot::getObserver(); }
	EDQDWorldObserver* getWorldObserver() { return (EDQDWorldObserver*)Robot::getWorldObserver(); }
	EDQDRobotWorldModel* getWorldModel() { return (EDQDRobotWorldModel*)Robot::getWorldModel(); }
};

//} /* namespace EDQDRobot */

#endif /* PRJ_EDQD_SRC_EDMAPELITESROBOT_H_ */

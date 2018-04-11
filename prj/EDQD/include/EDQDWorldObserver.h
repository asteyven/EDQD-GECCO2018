/**
 * @author Nicolas Bredeche <nicolas.bredeche@upmc.fr>
 *
 */


#ifndef EDMAPELITESWORLDOBSERVER_H
#define EDMAPELITESWORLDOBSERVER_H

#include "Observers/WorldObserver.h"
#include "RoboroboMain/common.h"

#include "EDQD/include/EDQDMap.h"


class World;

class EDQDWorldObserver : public WorldObserver
{
protected:
    virtual void updateEnvironment();
    virtual void updateMonitoring();
    virtual void monitorPopulation( bool localVerbose = true );
    
    int _generationCount;
    int _generationItCount;
    
    EDQDMap _bestMap;

public:
    EDQDWorldObserver(World *world);
    ~EDQDWorldObserver();
    
    void initPre();
    void initPost();

    void stepPre();
    void stepPost();

    virtual  int getGenerationItCount() { return _generationItCount; }
};

#endif

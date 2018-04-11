/*
 *  SquareObject.h
 *  roborobo
 *
 *  Created by Nicolas on 26/4/2014.
 *
 */

#ifndef SQUAREOBJECT_H
#define SQUAREOBJECT_H

#include "World/PhysicalObject.h"

class SquareObject : public PhysicalObject
{

protected:

    
    int _solid_w;
    int _solid_h;
    int _soft_w;
    int _soft_h;
    
public :
    
    SquareObject( int __id, int __group ); // use PhysicalObjectFactory instead!
    ~SquareObject() { }
    
    bool canRegister(); // test if register object is possible (use both shape or footprints)
    void registerObject(); // register object in the world (write images)
    void unregisterObject(); // unregister object in the world (write blank pixels)
    void show(SDL_Surface *surface = gScreen);
    void hide();    // wrt. screen-rendering
    
};

#endif

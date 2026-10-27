/*************************************************************************
* Copyright (c) 2019-2021 Jonathan Peña
* Permission to use, copy, modify, distribute and sell this software
* and its documentation for any purpose is hereby granted without fee,
* provided that the above copyright notice appear in all copies.
* Jonathan Peña makes no representations about the suitability 
* of this software for any purpose.  
* It is provided "as is" without express or implied warranty. 
**************************************************************************/


#ifndef COLLISION_H
#define COLLISION_H

#include "shape.h"
#include "manifold.h"

typedef void(*CollisionCallBack)(Manifold&, Shape*, Shape*);

void CircleToCircle(Manifold&, Shape*, Shape*);

void CircleToOBB(Manifold&, Shape*, Shape*);

void OBBToCircle(Manifold&, Shape*, Shape*);

void OBBToOBB(Manifold&, Shape*, Shape*);

static CollisionCallBack Dispatcher[2][2] = {CircleToCircle, CircleToOBB, OBBToCircle, OBBToOBB};

#endif

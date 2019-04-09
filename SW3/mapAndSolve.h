#ifndef MS_H
#define MS_H
#include "Heap.h"
#include "Node.h"
#include "Arduino.h"

void moveOne();
void turnCW();
void turnCCW();
void setSpace(short,short);
void moveUp();
void moveRight();
void moveDown();
void moveLeft();
void inity();
void resett();
void dfsR(short,short);
Node astar();
void buildPath(Node);

#endif

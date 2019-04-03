#ifndef FLOOD_H
#define FLOOD_H

typedef struct _Node
{
	int north:1;
	int east:1;
	int south:1;
	int west:1;
	short value;
} Node;

void floodFill();

#endif

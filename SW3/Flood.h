#ifndef FLOOD_H
#define FLOOD_H

enum states {
  IDLE, //Waits for its next movement state
  MOVE_FORWARD, //Move forward one block and switch back to idle
  ROTATE_CW_FORWARD,  //Rotate 90 degrees Clockwise
  ROTATE_CCW_FORWARD,  //Rotate 90 degrees Counter Clock Wise
  TURN_FORWARD,  //Rotate 180 degrees and go forward
  GO
};

typedef struct _Node
{
  bool north;
  bool east;
  bool south;
  bool west;
  short value;
  short x;
  short y;
  bool visited;
} Node;

extern enum states robotState;
extern char facing;
extern Node grid[16][16];
extern Node curr;
extern char newFacing;

void initializeGrid();
int validCell(short,short);
void transitionCell(char);
void updateCell();
void floodFill();

#endif

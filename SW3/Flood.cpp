#include "Flood.h"
#include "Arduino.h"

Node grid[16][16]={};
Node curr={};

char facing='e';
char newFacing='e';

states robotState = IDLE;

void setSpace(short,short);

/*
 * Set values fo grid for fllod-fill
 * Each quarter of grid is calced separatly by Manhattan distance
 */
void initializeGrid()
{
  //top left
	for(int i=0;i<8;i++)
	{
		for(int j=0;j<8;j++)
		{
			grid[i][j].value=abs(7-i)+abs(7-j);
			grid[i][j].x=i;
			grid[i][j].y=j;
		}
	}
  //bottom left
	for(int i=8;i<16;i++)
	{
		for(int j=0;j<8;j++)
		{
			grid[i][j].value=abs(8-i)+abs(7-j);
			grid[i][j].x=i;
			grid[i][j].y=j;

		}
	}
  //top right
	for(int i=0;i<8;i++)
	{
		for(int j=8;j<16;j++)
		{
			grid[i][j].value=abs(7-i)+abs(8-j);
			grid[i][j].x=i;
			grid[i][j].y=j;

		}
	}
  //bottom right
	for(int i=8;i<16;i++)
	{
		for(int j=8;j<16;j++)
		{
			grid[i][j].value=abs(8-i)+abs(8-j);
			grid[i][j].x=i;
			grid[i][j].y=j;
		}
	}
  //initialize two given walls
  grid[0][0].north=true;
  grid[0][0].west=true;
}

/*
 * Return 1 if cell index is in the bounds of the grid
 * Return 0 otherwise
 */
int validCell(short x,short y)
{
	if(x<0||x>15||y<0||y>15)
	{
		return 0;
	}
	return 1;
}

/*
 * Determine movement to transition to next cell
 */
void transitionCell(char newFacing)
{
  //robot facing north
	if(facing=='n')
	{
    //robot needs to face north
		if(newFacing=='n')
		{
			robotState=MOVE_FORWARD;
		}
    //robot needs to face east
		else if(newFacing=='e')
		{
			robotState=ROTATE_CW_FORWARD;
		}
    //robot needs to face south
		else if(newFacing=='s')
		{
			robotState=TURN_FORWARD;
		}
    //robot needs to face west
		else //newFacing='w'
		{
			robotState=ROTATE_CCW_FORWARD;
		}

	}
	else if(facing=='e')
	{
		if(newFacing=='n')
		{
			robotState=ROTATE_CCW_FORWARD;
		}
		else if(newFacing=='e')
		{
			robotState=MOVE_FORWARD;
		}
		else if(newFacing=='s')
		{
			robotState=ROTATE_CW_FORWARD;
		}
		else //newFacing='w'
		{
			robotState=TURN_FORWARD;
		}

	}
	else if(facing=='s')
	{
		if(newFacing=='n')
		{
			robotState=TURN_FORWARD;
		}
		else if(newFacing=='e')
		{
			robotState=ROTATE_CCW_FORWARD;
		}
		else if(newFacing=='s')
		{
			robotState=MOVE_FORWARD;
		}
		else //newFacing='w'
		{
			robotState=ROTATE_CW_FORWARD;
		}

	}
	else //facing='w'
	{
		if(newFacing=='n')
		{
			robotState=ROTATE_CW_FORWARD;
		}
		else if(newFacing=='e')
		{
			robotState=TURN_FORWARD;
		}
		else if(newFacing=='s')
		{
			robotState=ROTATE_CCW_FORWARD;
		}
		else //newFacing='w'
		{
			robotState=MOVE_FORWARD;
		}

	}
}

/*
 * Preform the update step in flood-fill
 * Make sure all weights satisfy constraint to be min{all neighbors}+1
 */
void updateCell(Node curr) //update step in flood-fill
{
	short minVal=257;
	bool n=false,e=false,s=false,w=false;
	Node minNode={};
	//find the minimum neighbor
	if(validCell(curr.x-1,curr.y) && !curr.north) //up neighbor
	{
		n=true;
		if(grid[curr.x-1][curr.y].value<minVal)
		{
			minVal=grid[curr.x-1][curr.y].value;
			minNode=grid[curr.x-1][curr.y];
		}
	}
	if(validCell(curr.x+1,curr.y) && !curr.south) //down neighbor
	{
		s=true;
		if(grid[curr.x+1][curr.y].value<minVal)
		{
			minVal=grid[curr.x+1][curr.y].value;
			minNode=grid[curr.x+1][curr.y];
		}
	}
	if(validCell(curr.x,curr.y-1) && !curr.west) //west neighbor
	{
		w=true;
		if(grid[curr.x][curr.y-1].value<minVal)
		{
			minVal=grid[curr.x][curr.y-1].value;
			minNode=grid[curr.x][curr.y-1];
		}
	}
	if(validCell(curr.x,curr.y+1) && !curr.east) //east neighbor
	{
		e=true;
		if(grid[curr.x][curr.y+1].value<minVal)
		{
			minVal=grid[curr.x][curr.y+1].value;
			minNode=grid[curr.x][curr.y+1];
		}

	}
	if(grid[curr.x][curr.y].value!=minVal+1)
	{
    //if current cell was updated, need to check all of its neighbors
		grid[curr.x][curr.y].value=minVal+1;
		if(n)
		{
			updateCell(grid[curr.x-1][curr.y]);
		}
		if(s)
		{
			updateCell(grid[curr.x+1][curr.y]);
		}
		if(w)
		{
			updateCell(grid[curr.x][curr.y-1]);
		}
		if(e)
		{
			updateCell(grid[curr.x][curr.y+1]);
		}
	}
}

/*
 * Flood-fill algorithm 
 */
void floodFill()
{
	if((curr.x==7&&curr.y==7)||(curr.x==7&&curr.y==8)||(curr.x==8&&curr.y==7)||(curr.x==8&&curr.y==8))
	{
    //Serial1.printf("FINISHED\n");
		robotState=IDLE;
    exit(0);
	}
	if(!curr.visited)
	{
    //Serial1.printf("Updating walls of current node\n");
		setSpace(curr.x,curr.y); //marks all the walls at the current space
		updateCell(curr);
	}
	//find min neighbor to move to
	int minVal=257;
	Node minNode={};
	//char newFacing;
	if(validCell(curr.x-1,curr.y) && !curr.north) //up neighbor
	{
    //Serial1.printf("North cell is valid\n");
		if(grid[curr.x-1][curr.y].value<minVal)
		{
      //Serial1.printf("North cell is possible next choice\n");
			minVal=grid[curr.x-1][curr.y].value;
			minNode=grid[curr.x-1][curr.y];
			newFacing='n';
		}
	}
  //Serial1.printf("is south valid: %d\n", validCell(curr.x+1,curr.y));
  //Serial1.printf("is south wall there: %d\n", !curr.south);
  

	if(validCell(curr.x+1,curr.y) && !curr.south) //down neighbor
	{    
	  //Serial1.printf("south cell is valid\n");
		if(grid[curr.x+1][curr.y].value<minVal)
		{
      //Serial1.printf("south cell is possible next choice\n");
			minVal=grid[curr.x+1][curr.y].value;
			minNode=grid[curr.x+1][curr.y];
			newFacing='s';
		}
	}
	if(validCell(curr.x,curr.y-1) && !curr.west) //west neighbor
	{   
	  //Serial1.printf("west cell is valid\n");
		if(grid[curr.x][curr.y-1].value<minVal)
		{
      //Serial1.printf("west cell is possible next choice\n");
			minVal=grid[curr.x][curr.y-1].value;
			minNode=grid[curr.x][curr.y-1];
			newFacing='w';
		}
	}
	if(validCell(curr.x,curr.y+1) && !curr.east) //east neighbor
	{   
	  //Serial1.printf("east cell is valid\n");
		if(grid[curr.x][curr.y+1].value<minVal)
		{
      //Serial1.printf("east cell is possible next choice\n");
			minVal=grid[curr.x][curr.y+1].value;
			minNode=grid[curr.x][curr.y+1];
			newFacing='e';
		}
	}
  //Serial1.printf("next state is: %c\n", newFacing);

	//transition state
	transitionCell(newFacing);
  facing=newFacing;
	grid[curr.x][curr.y].visited=1;
	curr=grid[minNode.x][minNode.y];
}

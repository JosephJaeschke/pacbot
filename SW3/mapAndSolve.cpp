#include "Heap.h"
#include "Node.h"
#include "Arduino.h"

Node grid[16][16]={};
char facing='u'; //facing up default

void moveOne();
void turnCW();
void turnCCW();
void setSpace(short,short);

void moveUp()
{
	if(facing=='r')
	{
		turnCW();
	}
	else if(facing=='d')
	{
		turnCW();
		turnCW();
	}
	else if(facing=='l')
	{
		turnCCW();
	}
  facing='u';
	moveOne();
	return;
}

void moveRight()
{
	if(facing=='u')
	{
		turnCW();
	}
	else if(facing=='d')
	{
		turnCCW();
	}
	else if(facing=='l')
	{
		turnCW();
		turnCW();
	}
  facing='r';
	moveOne();
	return;
}

void moveDown()
{
	if(facing=='u')
	{
		turnCW();
		turnCW();
	}
	else if(facing=='r')
	{
		turnCW();
	}
	else if(facing=='l')
	{
		turnCCW();
	}
  facing='d';
	moveOne();
	return;
}

void moveLeft()
{
	if(facing=='u')
	{
		turnCCW();
	}
	else if(facing=='r')
	{
		turnCW();
		turnCW();
	}
	else if(facing=='d')
	{
		turnCW();
	}
  facing='l';
	moveOne();
	return;
}

/*
void printWalls(short r,short c)
{
	cout<<"---"<<endl;
	cout<<"up:"<<grid[r][c].up<<endl;
	cout<<"right:"<<grid[r][c].right<<endl;
	cout<<"down:"<<grid[r][c].down<<endl;
	cout<<"left:"<<grid[r][c].left<<endl;
}
*/

void initMaze()
{
	for(short i=0;i<16;i++)
	{
		for(short j=0;j<16;j++)
		{
			grid[i][j].x=i;
			grid[i][j].y=j;
		}
	}
/*
	grid[0][0].up=1;
	grid[0][0].left=1;
	grid[0][0].right=1;
	grid[0][1].left=1;
	grid[0][1].right=1;
	grid[0][2].left=1;
	grid[0][2].right=1;
	grid[0][3].left=1;
	grid[0][3].down=1;
	grid[1][3].right=1;
	grid[1][3].down=1;
	grid[1][2].left=1;
	grid[1][2].right=1;
	grid[1][1].left=1;
	grid[1][1].right=1;
	grid[1][0].left=1;
	grid[1][0].up=1;
	grid[1][0].right=1;
	
	grid[0][0].up=1;
	grid[0][0].right=1;
	grid[0][0].left=1;
	grid[0][1].left=1;
	grid[0][2].down=1;
	grid[0][2].left=1;
	grid[1][0].up=1;
	grid[1][0].right=1;
	grid[1][0].left=1;
	grid[1][1].down=1;
	grid[1][2].up=1;
	grid[1][2].down=1;
	grid[2][0].up=1;
	grid[2][0].right=1;
	grid[2][0].left=1;
	grid[2][1].right=1;
	grid[2][2].right=1;
	grid[2][2].down=1;
*/
}

void resetMaze()
{
	for(short i=0;i<16;i++)
	{
		for(short j=0;j<16;j++)
		{
			grid[i][j].visited=0;
			grid[i][j].px=0;
			grid[i][j].py=0;
		}
	}
}

void dfsR(short r,short c)
{
	grid[r][c].visited=1;
	//printWalls(r,c);
  if(facing=='u')
  {
	  if(grid[r][c].up==0&&grid[r][c-1].visited==0)
  	{
  		moveUp();
      setSpace(r,c-1);
  		dfsR(r,c-1);
  		moveDown();
  	}
  	if(grid[r][c].right==0&&grid[r+1][c].visited==0)
  	{
  		moveRight();
      setSpace(r+1,c);
  		dfsR(r+1,c);
  		moveLeft();
  	}
  	if(grid[r][c].left==0&&grid[r-1][c].visited==0)
  	{
  		moveLeft();
      setSpace(r-1,c);
  		dfsR(r-1,c);
  		moveRight();
  	}
    if(grid[r][c].down==0&&grid[r][c+1].visited==0)
    {
      moveDown();
      setSpace(r,c+1);
      dfsR(r,c+1);
      moveUp();
    }
  }

  if(facing=='r')
  {
    if(grid[r][c].right==0&&grid[r+1][c].visited==0)
    {
      moveRight();
      setSpace(r+1,c);
      dfsR(r+1,c);
      moveLeft();
    }
    if(grid[r][c].up==0&&grid[r][c-1].visited==0)
    {
      moveUp();
      setSpace(r,c-1);
      dfsR(r,c-1);
      moveDown();
    }
    if(grid[r][c].down==0&&grid[r][c+1].visited==0)
    {
      moveDown();
      setSpace(r,c+1);
      dfsR(r,c+1);
      moveUp();
    }
    if(grid[r][c].left==0&&grid[r-1][c].visited==0)
    {
      moveLeft();
      setSpace(r-1,c);
      dfsR(r-1,c);
      moveRight();
    }
  }

  if(facing=='d')
  {
    if(grid[r][c].down==0&&grid[r][c+1].visited==0)
    {
      moveDown();
      setSpace(r,c+1);
      dfsR(r,c+1);
      moveUp();
    }
    if(grid[r][c].right==0&&grid[r+1][c].visited==0)
    {
      moveRight();
      setSpace(r+1,c);
      dfsR(r+1,c);
      moveLeft();
    }
    if(grid[r][c].left==0&&grid[r-1][c].visited==0)
    {
      moveLeft();
      setSpace(r-1,c);
      dfsR(r-1,c);
      moveRight();
    }
    if(grid[r][c].up==0&&grid[r][c-1].visited==0)
    {
      moveUp();
      setSpace(r,c-1);
      dfsR(r,c-1);
      moveDown();
    }
  }

  if(facing=='l')
  {
    if(grid[r][c].down==0&&grid[r][c+1].visited==0)
    {
      moveDown();
      setSpace(r,c+1);
      dfsR(r,c+1);
      moveUp();
    }
    if(grid[r][c].right==0&&grid[r+1][c].visited==0)
    {
      moveRight();
      setSpace(r+1,c);
      dfsR(r+1,c);
      moveLeft();
    }
    if(grid[r][c].left==0&&grid[r-1][c].visited==0)
    {
      moveLeft();
      setSpace(r-1,c);
      dfsR(r-1,c);
      moveRight();
    }
    if(grid[r][c].up==0&&grid[r][c-1].visited==0)
    {
      moveUp();
      setSpace(r,c-1);
      dfsR(r,c-1);
      moveDown();
    }
  }
}

Node astar()
{
	Node endd;
	grid[0][0].g=0;
	Heap fringe;
	fringe.push(grid[0][0]);
	while(fringe.empty()!=true)
	{
		Node n=fringe.pop();
		if((n.x==7&&n.y==7)||(n.x==7&&n.y==8)||(n.x==8&&n.y==7)||(n.x==8&&n.y==8))
		{
			//found path
			return n;
		}
		grid[n.x][n.y].visited=1;
		for(short i=-1;i<2;i++)
		{
			for(short j=-1;j<2;j++)
			{
				if(i+j==-2||i+j==0||i+j==2||(i==0&&j==0))
				{
					continue;
				}
				if(n.x+i>=0&&n.y+j>=0&&n.x+i<16&&n.y+j<16&&grid[n.x+i][n.y+j].visited==0)
				{
					if(i==0&&j==-1&&grid[n.x][n.y].up==1)
					{
						continue;
					}
					else if(i==1&&j==0&&grid[n.x][n.y].right==1)
					{
						continue;
					}
					else if(i==0&&j==1&&grid[n.x][n.y].down==1)
					{
						continue;
					}
					else if(i==-1&&j==0&&grid[n.x][n.y].left==1)
					{
						continue;
					}
					if(!fringe.contains(n.x+i,n.y+j))
					{
						grid[n.x+i][n.y+j].g=32767;
						grid[n.x+i][n.y+j].py=-1;
						grid[n.x+i][n.y+j].px=-1;
					}
					//update vertex
					if(grid[n.x][n.y].g+1<grid[n.x+i][n.y+j].g)
					{
						grid[n.x+i][n.y+j].g=grid[n.x][n.y].g+1;
/*THIS LINE HAS HEURISTIC*/			grid[n.x+i][n.y+j].h=abs(n.x+i-7.5)+abs(n.y+j-7.5);
						grid[n.x+i][n.y+j].px=n.x;
						grid[n.x+i][n.y+j].py=n.y;
						if(fringe.contains(n.x+i,n.y+j))
						{
							fringe.remove(n.x+i,n.y+j);
						}
						fringe.push(grid[n.x+i][n.y+j]);
					}
				}
			}
		}
	}
	return endd;	
}

void buildPath(Node endd)
{
	Node n=endd;
	char path[100];
	short i;
	for(i=0;n.x!=0||n.y!=0;i++)
	{
		if(n.x-n.px==-1&&n.y-n.py==0)
		{
			path[i]='l';
		}
		else if(n.x-n.px==0&&n.y-n.py==-1)
		{
			path[i]='u';
		}
		else if(n.x-n.px==0&&n.y-n.py==1)
		{
			path[i]='d';
		}
		else if(n.x-n.px==1&&n.y-n.py==0)
		{
			path[i]='r';
		}
		n=grid[n.px][n.py];
	}
	short j;
	char temp;
	for (j=0;j<i/2;++j)
	{
		temp=path[i-j-1];
		path[i-j-1]=path[j];
		path[j]=temp;
	}
	for (j=0;j<i;++j)
	{
		if(path[j]=='u')
		{
			moveUp();
		}
		else if(path[j]=='r')
		{
			moveRight();
		}
		else if(path[j]=='d')
		{
			moveDown();
		}
		else if(path[j]=='l')
		{
			moveLeft();
		}
	}
}

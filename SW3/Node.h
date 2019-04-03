#ifndef NODE_H
#define NODE_H
/*
using namespace std;

std::ostream& operator<<(std::ostream& o, const Node& n)
{
	  return o << n.visited;
}
*/
typedef struct _Node
{
	short x;
	short y;
	short px;
	short py;
	short g;
	short h;
	bool up;
	bool right;
	bool down;
	bool left;
	bool visited;
	bool err;
}Node;

#endif

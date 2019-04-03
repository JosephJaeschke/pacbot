#include "Stack.h"
#include "Flood.h"

Stack::Stack()
{
	top=0;
	maxelem=100;
}

void Stack::push(Node t)
{
	if(top==maxelem)
	{
		return;
	}
	s[top]=t;
	top++;
}

Node Stack::pop()
{
	if (top==0)
	{
		Node a;
		return a;
	}
	return s[top];
}

Node Stack::see()
{
	if (top==0)
	{
		Node a;
		return a;
	}
	return s[top-1];
}

int Stack::tallness()
{
	return top;
}

bool Stack::nothing()
{
	return top==0;  
}


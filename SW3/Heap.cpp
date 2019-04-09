#include "Heap.h"

Heap::Heap()
{
	top=0;
	maxelem=100;
}

void Heap::push(Node t)
{
	if(top==maxelem)
	{
		return;
	}
	top++;
	short i=top-1;
	h[i]=t;
	while(i!=0&&(h[(i-1)/2].g+h[(i-1)/2].h)>(h[i].g+h[i].h))
	{
		Node temp=h[i];
		h[i]=h[(i-1)/2];
		h[(i-1)/2]=temp;
		i=(i-1)/2;
	}
}

Node Heap::pop()
{
	if(top==0)
	{
		Node a;
		a.err=1;
		return a;
	}
	else if(top==1)
	{
		return h[--top];
	}
	Node root=h[0];
	h[0]=h[top-1];
	top--;
	heapify(0);
	return root;
}

void Heap::heapify(short i)
{
	short s=i;
	if((2*i+1)<top&&(h[2*i+1].g+h[2*i+1].h)<(h[i].g+h[i].h))
	{
		s=2*i+1;
	}
	if((2*i+2)<top&&(h[2*i+2].g+h[2*i+2].h)<(h[s].g+h[s].h))
	{
		s=2*i+2;
	}
	if(s!=i)
	{
		Node temp=h[i];
		h[i]=h[s];
		h[s]=temp;
		heapify(s);
	}

}

bool Heap::contains(short x,short y)
{
	for(short i=0;i<top;i++)
	{
		Node n=h[i];
		if(n.x==x&&n.y==y)
		{
			return true;
		}
	}
	return false;
}

void Heap::remove(short x,short y)
{
	for(short i=0;i<top;i++)
	{
		if(h[i].x==x&&h[i].y==y)
		{
			h[i]=h[--top];
			heapify(0);
			return;
		}
	}
}

bool Heap::empty()
{
	return top==0;  
}

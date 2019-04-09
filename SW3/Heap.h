#ifndef HEAP_H
#define HEAP_H
#include "Node.h"

class Heap
{
	public:
	Heap();
	void push(Node);
	Node pop();
	void heapify(short);
	bool contains(short,short);
	void remove(short,short);
	bool empty();

	private:
	Node h[100];
	short top;
	short maxelem;
};

#endif

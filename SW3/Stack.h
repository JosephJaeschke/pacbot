#include "Flood.h"

class Stack
{
	public:
	Stack();
	void push(Node);
	Node pop();
	Node see();
	int tallness();
	bool nothing();

	private:
	Node s[100];
	short top;
	short maxelem;
};

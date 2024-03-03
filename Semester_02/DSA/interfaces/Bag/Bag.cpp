#include "Bag.h"
#include "BagIterator.h"
#include <exception>
#include <iostream>
using namespace std;

Bag::Bag()
{
	// TODO - Implementation
	this->elements = new TElem[0];
	this->nrElements = 0;
}

void Bag::add(TElem elem)
{
}

bool Bag::remove(TElem elem)
{
	// TODO - Implementation
	return false;
}

bool Bag::search(TElem elem) const
{
	// TODO - Implementation
	return false;
}

int Bag::nrOccurrences(TElem elem) const
{
	// TODO - Implementation
	return 0;
}

int Bag::size() const
{
	// TODO - Implementation
	return 0;
}

bool Bag::isEmpty() const
{
	// TODO - Implementation
	return 0;
}

BagIterator Bag::iterator() const
{
	return BagIterator(*this);
}

Bag::~Bag()
{
	// TODO - Implementation
}

#include "Bag.h"
#include "BagIterator.h"
#include <exception>
#include <iostream>
using namespace std;

Bag::Bag()
{
	this->elements = new TElem[1];
	this->frequencies = new int[1];
	this->length = 0;
	this->minimum = NULL_TELEM;
	this->maximum = NULL_TELEM;
}

void Bag::add(TElem elem)
{
	bool found = Bag::search(elem);
}

bool Bag::remove(TElem elem)
{
	// TODO - Implementation
	return false;
}

bool Bag::search(TElem elem) const
{
	for (int i = 0; i < this->length; i++)
	{
		if (this->elements[i] == elem)
		{
			return true;
		}
	}
	return false;
}

int Bag::nrOccurrences(TElem elem) const
{
	return this->frequencies[elem];
}

int Bag::size() const
{
	return this->length;
}

bool Bag::isEmpty() const
{
	return this->length == 0;
}

BagIterator Bag::iterator() const
{
	return BagIterator(*this);
}

Bag::~Bag()
{
	delete[] this->elements;
	delete[] this->frequencies;
}

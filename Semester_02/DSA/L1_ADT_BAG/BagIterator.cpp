#include <exception>
#include "BagIterator.h"
#include "Bag.h"

using namespace std;

BagIterator::BagIterator(const Bag &c) : bag(c)
{
	// TODO - Implementation
	this->position = 0;
}

/*
BC: Θ(1)
WC: Θ(1)
TC: Θ(1)
*/
void BagIterator::first()
{
	// TODO - Implementation
	this->position = 0;
}

void BagIterator::next()
{
	// TODO - Implementation
}

bool BagIterator::valid() const
{

	return this->bag.length != 0 &&
		   this->position != -1;
}

TElem BagIterator::getCurrent() const
{
	return NULL_TELEM
}

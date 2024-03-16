#include "Bag.h"
#include "BagIterator.h"
#include <exception>
#include <iostream>
#include <cmath>
using namespace std;

Bag::Bag()
{
	this->frequencies = new int[1];
	this->capacity = 1;
	this->length = 0;
	this->minimum = NULL_TELEM;
	this->maximum = NULL_TELEM;
}

/*
BC: Θ(1) - element is between minimum and maximum
WC: Θ(n) - element is not between minimum and maximum, so we need to resize the array
TC: O(n)
*/
void Bag::add(TElem elem)
{
	int oldMinimum = this->minimum, oldMaximum = this->maximum;
	if (Bag::size() == 0)
	{
		this->minimum = elem;
		this->maximum = elem;

		this->frequencies[0] = 1;
		this->length = this->length + 1;
		return;
	}

	bool resize = false;
	if (elem < this->minimum)
	{
		this->minimum = elem;
		resize = true;
	}
	if (elem > this->maximum)
	{
		this->maximum = elem;
		resize = true;
	}

	if (!resize)
	{
		this->frequencies[elem - this->minimum]++;
		this->length = this->length + 1;
		return;
	}

	int newCapacity = maximum - minimum + 1;

	int *newFrequencies = new int[newCapacity];

	for (int i = 0; i < newCapacity; i++)
	{
		newFrequencies[i] = 0;
	}

	if (oldMinimum > this->minimum)
	{
		int j = 0;
		for (int i = oldMinimum - this->minimum; i < newCapacity; i++)
		{
			newFrequencies[i] = this->frequencies[j++];
		}
		newFrequencies[0]++;
	}
	if (oldMaximum < this->maximum)
	{
		for (int i = 0; i < this->capacity; i++)
		{
			newFrequencies[i] = this->frequencies[i];
		}

		newFrequencies[newCapacity - 1]++;
	}

	delete[] this->frequencies;
	this->frequencies = newFrequencies;
	this->capacity = newCapacity;
	this->length = this->length + 1;
}

/*
BC: Θ(1) - element is between minimum and maximum, or does not belong to the bag
		 - or the element is the minimum or maximum, but it has more than one occurences
WC: Θ(n) - element is the minimum or maximum and has only one occurence, so we need to readjust the indexes again
TC: O(n)
*/

bool Bag::remove(TElem elem)
{
	if (this->length == 0 || search(elem) == false)
	{
		return false;
	}

	if (elem != this->minimum && elem != this->maximum)
	{
		this->frequencies[elem - this->minimum]--;
		this->length = this->length - 1;
		return true;
	}

	if (elem == this->minimum)
	{
		if (this->frequencies[elem - this->minimum] > 1)
		{
			this->frequencies[elem - this->minimum]--;
			this->length = this->length - 1;
			return true;
		}
		else
		{
			int oldMinimum = this->minimum;
			for (int i = 0; i < this->capacity; i++)
			{
				if (this->frequencies[i] > 0)
				{
					this->minimum = i + this->minimum;
					break;
				}
			}

			int newCapacity = this->maximum - this->minimum + 1;
			int *newFrequencies = new int[newCapacity];

			for (int i = 0; i < newCapacity; i++)
			{
				newFrequencies[i] = 0;
			}

			int j = 0;
			for (int i = this->minimum - oldMinimum; i < this->capacity; i++)
			{
				newFrequencies[j++] = this->frequencies[i];
			}

			delete[] this->frequencies;
			this->frequencies = newFrequencies;
			this->capacity = newCapacity;
			this->length = this->length - 1;
		}
	}
	if (elem == this->maximum)
	{
		if (this->frequencies[elem - this->minimum] > 1)
		{
			this->frequencies[elem - this->minimum]--;
			this->length = this->length - 1;
			return true;
		}
		else
		{
			for (int i = this->capacity - 1; i >= 0; i--)
			{
				if (this->frequencies[i] > 0)
				{
					this->maximum = i + this->minimum;
					break;
				}
			}

			int newCapacity = this->maximum - this->minimum + 1;
			int *newFrequencies = new int[newCapacity];

			for (int i = 0; i < newCapacity; i++)
			{
				newFrequencies[i] = this->frequencies[i];
			}

			delete[] this->frequencies;
			this->frequencies = newFrequencies;
			this->capacity = newCapacity;
			this->length = this->length - 1;
		}
	}

	return false;
}

/*
BC: Θ(1)
WC: Θ(1)
TC: Θ(1)
*/
bool Bag::search(TElem elem) const
{
	return elem >= this->minimum && elem <= this->maximum && this->frequencies[elem - this->minimum] > 0;
}

/*
BC: Θ(1)
WC: Θ(1)
TC: Θ(1)
*/
int Bag::nrOccurrences(TElem elem) const
{
	return this->frequencies[elem - this->minimum];
}

/*
BC: Θ(1)
WC: Θ(1)
TC: Θ(1)
*/
int Bag::size() const
{
	return this->length;
}

/*
BC: Θ(1)
WC: Θ(1)
TC: Θ(1)
*/
bool Bag::isEmpty() const
{
	return this->length == 0;
}

BagIterator Bag::iterator() const
{
	return BagIterator(*this);
}

void Bag::printBag()
{
	for (int i = 0; i < this->capacity; i++)
	{
		// if (this->frequencies[i] > 0)
		// cout << i + this->minimum << " ";
		cout << i + this->minimum << " ";
		// cout << i << " ";
	}
	cout << endl;
	for (int i = 0; i < this->capacity; i++)
	{
		// if (this->frequencies[i] > 0)
		// cout << this->frequencies[i] << " ";
		cout << this->frequencies[i] << " ";
	}
	cout << endl;
}

Bag::~Bag()
{
	delete[] this->frequencies;
}

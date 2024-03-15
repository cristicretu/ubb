#pragma once
// DO NOT INCLUDE BAGITERATOR

// DO NOT CHANGE THIS PART
#define NULL_TELEM -111111;
typedef int TElem;
class BagIterator;
class Bag
{

	/*
ADT Bag – represented as a dynamic array of frequencies.
For example, the bag [5, 10, -1, 2, 3, 10, 5, 5, -5] will be represented as [1, 0, 0, 0, 1, 0, 0, 1,
1, 0, 3, 0, 0, 0, 0, 2], built in the following way:
- the interval of values [-5, 10] is translated into the interval [0, 16] of positions
- at position 0 we have the frequency of -5 (minimum element), on position 1 we
have the frequency of -4, …, on position 15 we have the frequency of 10 (maximum
element).
	*/

private:
	TElem *elements;
	int *frequencies;

	int length;
	// capacity is given by (maximum + abs(minimum) + 1)
	TElem minimum;
	TElem maximum;

	// DO NOT CHANGE THIS PART
	friend class BagIterator;

public:
	// constructor
	Bag();

	// adds an element to the bag
	void add(TElem e);

	// removes one occurence of an element from a bag
	// returns true if an element was removed, false otherwise (if e was not part of the bag)
	bool remove(TElem e);

	// checks if an element appearch is the bag
	bool search(TElem e) const;

	// returns the number of occurrences for an element in the bag
	int nrOccurrences(TElem e) const;

	// returns the number of elements from the bag
	int size() const;

	// returns an iterator for this bag
	BagIterator iterator() const;

	// checks if the bag is empty
	bool isEmpty() const;

	// destructor
	~Bag();
};
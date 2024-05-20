#include "SMMIterator.h"
#include "SortedMultiMap.h"
#include <iostream>
#include <vector>
#include <exception>
using namespace std;

SortedMultiMap::SortedMultiMap(Relation r) {
	//TODO - Implementation
}

void SortedMultiMap::add(TKey c, TValue v) {
	//TODO - Implementation
}

vector<TValue> SortedMultiMap::search(TKey c) const {
	//TODO - Implementation
	return vector<TValue>();
}

bool SortedMultiMap::remove(TKey c, TValue v) {
	//TODO - Implementation
    return false;
}


int SortedMultiMap::size() const {
	//TODO - Implementation
	return 0;
}

bool SortedMultiMap::isEmpty() const {
	//TODO - Implementation
	return false;
}

SMMIterator SortedMultiMap::iterator() const {
	return SMMIterator(*this);
}

SortedMultiMap::~SortedMultiMap() {
	//TODO - Implementation
}

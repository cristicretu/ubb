#include "SMIterator.h"
#include "SortedMap.h"
#include <exception>
using namespace std;

SortedMap::SortedMap(Relation r) {
	//TODO - Implementation
}

TValue SortedMap::add(TKey k, TValue v) {
	//TODO - Implementation
	return NULL_TVALUE;
}

TValue SortedMap::search(TKey k) const {
	//TODO - Implementation
	return NULL_TVALUE;
}

TValue SortedMap::remove(TKey k) {
	//TODO - Implementation
	return NULL_TVALUE;
}

int SortedMap::size() const {
	//TODO - Implementation
	return 0;
}

bool SortedMap::isEmpty() const {
	//TODO - Implementation
	return false;
}

SMIterator SortedMap::iterator() const {
	return SMIterator(*this);
}

SortedMap::~SortedMap() {
	//TODO - Implementation
}

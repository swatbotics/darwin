#include "Index.h"
#include <iostream>
#include <cstdlib>
#include <ctime>
#include <assert.h>

void testReduceRestore(IndexSet& iset, IndexArray& work) {

  std::cout << "before messing around, iset = " << iset << "\n";

  IndexArray copy;
  for (size_t i=0; i<iset.size(); ++i) { copy.push_back(iset[i]); }

  if (iset.size() < 2) { return; }

  work.clear();
  for (size_t i=0; i<iset.size(); ++i) {
    if (rand() % 2) { work.push_back(i); }
  }

  size_t old_count = iset.reduce(work);
  
  testReduceRestore(iset, work);

  iset.restore(old_count, work);

  for (size_t i=0; i<iset.size(); ++i) { assert(iset[i] == copy[i]); }

  std::cout << " after messing around, iset = " << iset << "\n";

}

int main(int argc, char** argv) {
  
  srand(time(NULL));

  IndexSet iset(32);
  IndexArray work;

  testReduceRestore(iset, work);

  return 0;

}

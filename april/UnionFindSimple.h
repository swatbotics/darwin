#ifndef _UNIONFINDSIMPLE_H_
#define _UNIONFINDSIMPLE_H_

#include <vector>

class UnionFindSimple {
public:

  std::vector<int> data;

  enum { SZ = 2 };

  /** @param maxid The maximum node id that will be referenced. **/
  UnionFindSimple(int maxid);

  int getSetSize(int id);

  int getRepresentative(int id);

  int connectNodes(int aid, int bid);

};

#endif

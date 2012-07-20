#ifndef _INDEX_H_
#define _INDEX_H_

#include <vector>
#include <algorithm>
#include <iostream>

typedef std::vector<size_t> IndexArray;

class IndexSet {
public:

  IndexSet(): _size(0) {}

  IndexSet(const IndexSet& iset): _size(iset._size), _indices(iset._indices) {}

  explicit IndexSet(size_t count): _size(count), _indices(count) {
    for (size_t i=0; i<count; ++i) { _indices[i] = i; }
  }

  IndexSet(const IndexSet& iset, const IndexArray& subs) {
    _size = subs.size();
    _indices.resize(_size);
    for (size_t i=0; i<_size; ++i) { _indices[i] = iset[subs[i]]; }
  }

  const size_t operator[](size_t i) const { return _indices[i]; }

  size_t size() const { return _size; }

  size_t reduce(IndexArray& subs) {

    size_t orig_size = _size;
    size_t subs_size = subs.size();

    _size = subs.size();
    
    size_t next = 0;
    size_t i=0;

    while (subs.size() < orig_size) {
      if (i < subs_size && next == subs[i]) {
        ++i;
        ++next;
      } else {
        subs.push_back(next);
        ++next;
      }
    }

    for (size_t i=0; i<orig_size; ++i) {
      subs[i] = _indices[subs[i]];
    }
  
    for (size_t i=0; i<orig_size; ++i) {
      _indices[i] = subs[i];
    }

    return orig_size;

  }

  void restore(size_t prev_count, IndexArray& work) {

    IndexArray::const_iterator i0 = _indices.begin();
    IndexArray::const_iterator i1 = i0 + _size;
    IndexArray::const_iterator i2 = i1;
    IndexArray::const_iterator i3 = i0 + prev_count;

    work.resize(prev_count);
    std::merge( i0, i1, i2, i3, work.begin() );
    
    std::copy( work.begin(), work.end(), _indices.begin() );
    
    _size = prev_count;

  }


  void restore(size_t prev_count, IndexArray& minima, IndexArray& work) {

    work.clear();
    size_t i = 0;
    size_t j = _size;
    size_t k = 0;

    while (work.size() < prev_count) {
      if (i >= _size) {
        work.push_back(_indices[j++]);
      } else if (j >= prev_count || _indices[i] < _indices[j]) {
        while (k < minima.size() && minima[k] == i) {
          minima[k++] = work.size();
        }
        work.push_back(_indices[i++]);
      } else {
        work.push_back(_indices[j++]);
      }
    }

    std::copy( work.begin(), work.end(), _indices.begin() );

    _size = prev_count;

  }

private:

  size_t _size;
  IndexArray _indices;

};

inline std::ostream& operator<<(std::ostream& ostr, const IndexSet& iset) {
  ostr << "[ ";
  for (size_t i=0; i<iset.size(); ++i) { ostr << iset[i] << " "; }
  return ostr << "]";
}



#endif

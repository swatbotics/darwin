#include "timestamp.hpp"

#include <iostream>

int main(int argc, char* argv[]) {
  Timestamp ts(10, 100 * 1000 * 1000);
  Timestamp ts2(5, 500 * 1000 * 1000);
  std::cout << ts.ToString() << "\n";
  std::cout << ts << "\n";
  std::cout << (ts * 5) << "\n";
  std::cout << (ts - ts2) << "\n";
  std::cout << Timestamp::Now() << "\n";
  std::cout << Timestamp::Now().ToDouble() << "\n";
  return 0;
}

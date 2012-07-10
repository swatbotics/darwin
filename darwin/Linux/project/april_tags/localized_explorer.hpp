#ifndef LOCALIZED_EXPLORER_HPP
#define LOCALIZED_EXPLORER_HPP

#include "CM730.h"

#include "status_client.hpp"

namespace Robot {

class LocalizedExplorer {
 public:
  LocalizedExplorer();
  ~LocalizedExplorer();
  void Initialize();
  void Process();

 private:
  void InitializeMotionFramework();
  void InitializeMotionModules();

  CM730* cm730_;
  StatusClient client_;
};

}  // namespace Robot

#endif  // LOCALIZED_EXPLORER_HPP

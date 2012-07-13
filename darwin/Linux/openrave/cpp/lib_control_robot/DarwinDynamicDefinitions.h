/*
This file contains some information about the dynamics of the Darwin robot.
This includes the mass of each link, the transform of center of mass for
each link in the local frams, and the position of each of the links.
*/

#include <string>
#include "Transform3.h"

struct link;

struct link{
  std::string NAME;
  float MASS;              // Mass of link
  vec3f COM;            // Center of mass in the link frame
  int PREVIOUS;            // Index of previous link
  int NEXT;                // Index of next link (-1 for end links)
  Transform3f T_PREV2NEXT; // Transform from link frame to next joint
  vec3f AXIS;           // Axis of rotation for this link
};


struct chain{
  int FIRST;
  Transform3f T_FROM_BODY;
};


class Darwin{
  public:
    Darwin();
    struct chain Chains[5];
    struct link Links[21];
};

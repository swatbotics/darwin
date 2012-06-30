/*
This file contains some information about the dynamics of the Darwin robot.
This includes the mass of each link, the transform of center of mass for
each link in the local frams, and the position of each of the links.
*/

#include <string>

struct link;

struct link{
  std::string NAME;
  float MASS;            // Mass of link
  float COM[3];          // Center of mass in the link frame
  int PREVIOUS;          // Index of previous link
  int NEXT;              // Index of next link (-1 for end links)
  float T_PREV2NEXT[3][4]; // Transform from link frame to next joint
                         // as top 3 rows of transform matrix
  float AXIS[3];         // Axis of rotation for this link
};


struct chain{
  int FIRST;
  float T_FROM_BODY[3][4];
};


class Darwin{
  public:
    Darwin();
    struct chain Chains[5];
    struct link Links[21];
};

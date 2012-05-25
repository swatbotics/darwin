#ifndef _GRAYMODEL_H_
#define _GRAYMODEL_H_

#include "AprilTypes.h"

class GrayModel {
public:

  // we're solving Ax = b. For each observation, we add a row to
  // A of the form [x y xy 1] and to be of the form [gray]. x is
  // the vector [A B C D].
  //
  // The least-squares solution to the system is x = inv(A'A)A'b

  at::real A[4][4];
  at::real b[4];
  at::real X[4];

  int nobs; // how many observations?
  bool have_solution;

  GrayModel();

  void addObservation(at::real x, at::real y, at::real gray);

  int getNumObservations();

  void compute();

  at::real interpolate(at::real x, at::real y);

};

#endif

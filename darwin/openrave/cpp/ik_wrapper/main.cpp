#ifdef IKFAST_HEADER
#include IKFAST_HEADER
#endif
#include <cstdio>
#include "ik.cpp"

#define PI 3.1415926

int main(int argc, char** argv){
  std::vector<IKSolution> solutions;
  IKReal eerot[9] = {0,0,1,  1,0,0, 0,1,0};
  IKReal eetrans[3] = {0,-.3,0};
  if (argc==13){
    eerot[0] = atof(argv[1]); eerot[1] = atof(argv[2]); eerot[2] = atof(argv[3]); eetrans[0] = atof(argv[4]);
    eerot[3] = atof(argv[5]); eerot[4] = atof(argv[6]); eerot[5] = atof(argv[7]); eetrans[1] = atof(argv[8]);
    eerot[6] = atof(argv[9]); eerot[7] = atof(argv[10]); eerot[8] = atof(argv[11]); eetrans[2] = atof(argv[12]);
  } else if (argc==4){
    eetrans[0] = atof(argv[1]); eetrans[1] = atof(argv[2]); eetrans[2] = atof(argv[3]); 
  } else if (argc == 10){
    eerot[0] = atof(argv[1]); eerot[1] = atof(argv[2]); eerot[2] = atof(argv[3]); 
    eerot[3] = atof(argv[4]); eerot[4] = atof(argv[5]); eerot[5] = atof(argv[6]); 
    eerot[6] = atof(argv[7]); eerot[7] = atof(argv[8]); eerot[8] = atof(argv[9]); 
  }
  IKReal* pfree = NULL;
  IKSolver solver;
  bool success = solver.ik((IKReal*)eetrans,(IKReal*) eerot, pfree , solutions);
  if( !success ) {
      fprintf(stderr,"Failed to get ik solution\n");
      return -1;
  }

  // Display IK results
  printf("Found %d ik solutions:\n", (int)solutions.size());
  std::vector<IKReal> sol(getNumJoints());
  for(std::size_t i = 0; i < solutions.size(); ++i) {
      printf("sol%d (free=%d): ", (int)i, (int)solutions[i].GetFree().size());
      std::vector<IKReal> solfree(solutions[i].GetFree().size());
      solutions[i].GetSolution(&sol[0],solfree.size()>0?&solfree[0]:NULL);
      for( std::size_t j = 0; j < sol.size(); ++j)
          printf("%.2f, ", sol[j]);
      printf("\n");
  }

  // Move body to show result
  


  return 0;
}

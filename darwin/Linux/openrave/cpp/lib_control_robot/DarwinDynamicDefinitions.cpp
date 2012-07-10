
#include "DarwinDynamicDefinitions.h"

Darwin::Darwin(){
  Links[0].NAME = "DBODY_LINK";
  Links[0].MASS = 0.975599;
  Links[0].COM[0] = -0.003116;
  Links[0].COM[1] = -0.039444;
  Links[0].COM[2] = -0.019663;
  Links[0].NEXT = 1;
  Links[0].PREVIOUS = -1;
  Links[0].AXIS[0] = -0.707106;
  Links[0].AXIS[1] = 0.707108;
  Links[0].AXIS[2] = 0.000000;
  Links[0].T_PREV2NEXT[0][0] = 1.000000;
  Links[0].T_PREV2NEXT[0][1] = 0.000000;
  Links[0].T_PREV2NEXT[0][2] = 0.000000;
  Links[0].T_PREV2NEXT[0][3] = 0.000000;
  Links[0].T_PREV2NEXT[1][0] = 0.000000;
  Links[0].T_PREV2NEXT[1][1] = 1.000000;
  Links[0].T_PREV2NEXT[1][2] = 0.000000;
  Links[0].T_PREV2NEXT[1][3] = 0.000000;
  Links[0].T_PREV2NEXT[2][0] = 0.000000;
  Links[0].T_PREV2NEXT[2][1] = 0.000000;
  Links[0].T_PREV2NEXT[2][2] = 1.000000;
  Links[0].T_PREV2NEXT[2][3] = 0.000000;

  Links[1].NAME = "DNECK_LINK";
  Links[1].MASS = 0.024358;
  Links[1].COM[0] = 0.001424;
  Links[1].COM[1] = -0.016568;
  Links[1].COM[2] = -0.000713;
  Links[1].NEXT = 2;
  Links[1].PREVIOUS = 0;
  Links[1].AXIS[0] = 0.000000;
  Links[1].AXIS[1] = 1.000000;
  Links[1].AXIS[2] = 0.000000;
  Links[1].T_PREV2NEXT[0][0] = 1.000000;
  Links[1].T_PREV2NEXT[0][1] = -0.000000;
  Links[1].T_PREV2NEXT[0][2] = -0.000000;
  Links[1].T_PREV2NEXT[0][3] = 0.000000;
  Links[1].T_PREV2NEXT[1][0] = -0.000000;
  Links[1].T_PREV2NEXT[1][1] = 0.707106;
  Links[1].T_PREV2NEXT[1][2] = -0.707108;
  Links[1].T_PREV2NEXT[1][3] = 0.000000;
  Links[1].T_PREV2NEXT[2][0] = 0.000000;
  Links[1].T_PREV2NEXT[2][1] = 0.707108;
  Links[1].T_PREV2NEXT[2][2] = 0.707106;
  Links[1].T_PREV2NEXT[2][3] = 0.000000;

  Links[2].NAME = "DHEAD_LINK";
  Links[2].MASS = 0.158042;
  Links[2].COM[0] = 0.000064;
  Links[2].COM[1] = 0.018565;
  Links[2].COM[2] = 0.076666;
  Links[2].NEXT = -1;
  Links[2].PREVIOUS = 1;
  Links[2].AXIS[0] = -1.000000;
  Links[2].AXIS[1] = 0.000000;
  Links[2].AXIS[2] = 0.000000;
  Links[2].T_PREV2NEXT[0][0] = 1.000000;
  Links[2].T_PREV2NEXT[0][1] = 0.000000;
  Links[2].T_PREV2NEXT[0][2] = 0.000000;
  Links[2].T_PREV2NEXT[0][3] = 0.000000;
  Links[2].T_PREV2NEXT[1][0] = 0.000000;
  Links[2].T_PREV2NEXT[1][1] = 1.000000;
  Links[2].T_PREV2NEXT[1][2] = 0.000000;
  Links[2].T_PREV2NEXT[1][3] = 0.000000;
  Links[2].T_PREV2NEXT[2][0] = 0.000000;
  Links[2].T_PREV2NEXT[2][1] = 0.000000;
  Links[2].T_PREV2NEXT[2][2] = 1.000000;
  Links[2].T_PREV2NEXT[2][3] = 0.000000;

  Links[3].NAME = "DPELVYL_LINK";
  Links[3].MASS = 0.027069;
  Links[3].COM[0] = 0.000000;
  Links[3].COM[1] = 0.018437;
  Links[3].COM[2] = 0.000480;
  Links[3].NEXT = 4;
  Links[3].PREVIOUS = 0;
  Links[3].AXIS[0] = -0.000000;
  Links[3].AXIS[1] = -1.000000;
  Links[3].AXIS[2] = 0.000000;
  Links[3].T_PREV2NEXT[0][0] = 1.000000;
  Links[3].T_PREV2NEXT[0][1] = 0.000000;
  Links[3].T_PREV2NEXT[0][2] = -0.000000;
  Links[3].T_PREV2NEXT[0][3] = 0.000000;
  Links[3].T_PREV2NEXT[1][0] = -0.000000;
  Links[3].T_PREV2NEXT[1][1] = 1.000000;
  Links[3].T_PREV2NEXT[1][2] = 0.000000;
  Links[3].T_PREV2NEXT[1][3] = 0.000000;
  Links[3].T_PREV2NEXT[2][0] = 0.000000;
  Links[3].T_PREV2NEXT[2][1] = -0.000000;
  Links[3].T_PREV2NEXT[2][2] = 1.000000;
  Links[3].T_PREV2NEXT[2][3] = 0.000000;

  Links[4].NAME = "DPELVL_LINK";
  Links[4].MASS = 0.167108;
  Links[4].COM[0] = 0.000080;
  Links[4].COM[1] = -0.013873;
  Links[4].COM[2] = -0.018242;
  Links[4].NEXT = 5;
  Links[4].PREVIOUS = 3;
  Links[4].AXIS[0] = -0.000000;
  Links[4].AXIS[1] = -0.000000;
  Links[4].AXIS[2] = -1.000000;
  Links[4].T_PREV2NEXT[0][0] = 1.000000;
  Links[4].T_PREV2NEXT[0][1] = 0.000000;
  Links[4].T_PREV2NEXT[0][2] = 0.000000;
  Links[4].T_PREV2NEXT[0][3] = 0.000000;
  Links[4].T_PREV2NEXT[1][0] = 0.000000;
  Links[4].T_PREV2NEXT[1][1] = 1.000000;
  Links[4].T_PREV2NEXT[1][2] = 0.000000;
  Links[4].T_PREV2NEXT[1][3] = 0.000000;
  Links[4].T_PREV2NEXT[2][0] = 0.000000;
  Links[4].T_PREV2NEXT[2][1] = 0.000000;
  Links[4].T_PREV2NEXT[2][2] = 1.000000;
  Links[4].T_PREV2NEXT[2][3] = 0.000000;

  Links[5].NAME = "DLEGUPPERL_LINK";
  Links[5].MASS = 0.119043;
  Links[5].COM[0] = -0.000323;
  Links[5].COM[1] = -0.062966;
  Links[5].COM[2] = 0.000692;
  Links[5].NEXT = 6;
  Links[5].PREVIOUS = 4;
  Links[5].AXIS[0] = -1.000000;
  Links[5].AXIS[1] = -0.000000;
  Links[5].AXIS[2] = 0.000000;
  Links[5].T_PREV2NEXT[0][0] = 1.000000;
  Links[5].T_PREV2NEXT[0][1] = 0.000000;
  Links[5].T_PREV2NEXT[0][2] = 0.000000;
  Links[5].T_PREV2NEXT[0][3] = -0.000000;
  Links[5].T_PREV2NEXT[1][0] = 0.000000;
  Links[5].T_PREV2NEXT[1][1] = 1.000000;
  Links[5].T_PREV2NEXT[1][2] = 0.000000;
  Links[5].T_PREV2NEXT[1][3] = -0.093000;
  Links[5].T_PREV2NEXT[2][0] = 0.000000;
  Links[5].T_PREV2NEXT[2][1] = 0.000000;
  Links[5].T_PREV2NEXT[2][2] = 1.000000;
  Links[5].T_PREV2NEXT[2][3] = -0.000000;

  Links[6].NAME = "DLEGLOWERL_LINK";
  Links[6].MASS = 0.07031;
  Links[6].COM[0] = -0.000592;
  Links[6].COM[1] = 0.053955;
  Links[6].COM[2] = 0.006547;
  Links[6].NEXT = 7;
  Links[6].PREVIOUS = 5;
  Links[6].AXIS[0] = -1.000000;
  Links[6].AXIS[1] = -0.000000;
  Links[6].AXIS[2] = 0.000000;
  Links[6].T_PREV2NEXT[0][0] = 1.000000;
  Links[6].T_PREV2NEXT[0][1] = 0.000000;
  Links[6].T_PREV2NEXT[0][2] = 0.000000;
  Links[6].T_PREV2NEXT[0][3] = -0.000000;
  Links[6].T_PREV2NEXT[1][0] = 0.000000;
  Links[6].T_PREV2NEXT[1][1] = 1.000000;
  Links[6].T_PREV2NEXT[1][2] = 0.000000;
  Links[6].T_PREV2NEXT[1][3] = -0.093000;
  Links[6].T_PREV2NEXT[2][0] = 0.000000;
  Links[6].T_PREV2NEXT[2][1] = 0.000000;
  Links[6].T_PREV2NEXT[2][2] = 1.000000;
  Links[6].T_PREV2NEXT[2][3] = -0.000000;

  Links[7].NAME = "DANKLEL_LINK";
  Links[7].MASS = 0.167108;
  Links[7].COM[0] = -0.000214;
  Links[7].COM[1] = 0.013873;
  Links[7].COM[2] = -0.018536;
  Links[7].NEXT = 8;
  Links[7].PREVIOUS = 6;
  Links[7].AXIS[0] = 1.000000;
  Links[7].AXIS[1] = 0.000000;
  Links[7].AXIS[2] = 0.000000;
  Links[7].T_PREV2NEXT[0][0] = 1.000000;
  Links[7].T_PREV2NEXT[0][1] = -0.000000;
  Links[7].T_PREV2NEXT[0][2] = 0.000000;
  Links[7].T_PREV2NEXT[0][3] = 0.000000;
  Links[7].T_PREV2NEXT[1][0] = 0.000000;
  Links[7].T_PREV2NEXT[1][1] = 1.000000;
  Links[7].T_PREV2NEXT[1][2] = -0.000000;
  Links[7].T_PREV2NEXT[1][3] = 0.000000;
  Links[7].T_PREV2NEXT[2][0] = -0.000000;
  Links[7].T_PREV2NEXT[2][1] = 0.000000;
  Links[7].T_PREV2NEXT[2][2] = 1.000000;
  Links[7].T_PREV2NEXT[2][3] = 0.000000;

  Links[8].NAME = "DFOOTL_LINK";
  Links[8].MASS = 0.079446;
  Links[8].COM[0] = 0.009506;
  Links[8].COM[1] = -0.025995;
  Links[8].COM[2] = -0.000503;
  Links[8].NEXT = -1;
  Links[8].PREVIOUS = 7;
  Links[8].AXIS[0] = -0.000000;
  Links[8].AXIS[1] = -0.000000;
  Links[8].AXIS[2] = 1.000000;
  Links[8].T_PREV2NEXT[0][0] = 1.000000;
  Links[8].T_PREV2NEXT[0][1] = 0.000000;
  Links[8].T_PREV2NEXT[0][2] = 0.000000;
  Links[8].T_PREV2NEXT[0][3] = 0.000000;
  Links[8].T_PREV2NEXT[1][0] = 0.000000;
  Links[8].T_PREV2NEXT[1][1] = 1.000000;
  Links[8].T_PREV2NEXT[1][2] = 0.000000;
  Links[8].T_PREV2NEXT[1][3] = 0.000000;
  Links[8].T_PREV2NEXT[2][0] = 0.000000;
  Links[8].T_PREV2NEXT[2][1] = 0.000000;
  Links[8].T_PREV2NEXT[2][2] = 1.000000;
  Links[8].T_PREV2NEXT[2][3] = 0.000000;

  Links[9].NAME = "DPELVYR_LINK";
  Links[9].MASS = 0.027069;
  Links[9].COM[0] = -0.000000;
  Links[9].COM[1] = 0.018437;
  Links[9].COM[2] = 0.000480;
  Links[9].NEXT = 10;
  Links[9].PREVIOUS = 0;
  Links[9].AXIS[0] = -0.000000;
  Links[9].AXIS[1] = -1.000000;
  Links[9].AXIS[2] = 0.000000;
  Links[9].T_PREV2NEXT[0][0] = 1.000000;
  Links[9].T_PREV2NEXT[0][1] = 0.000000;
  Links[9].T_PREV2NEXT[0][2] = -0.000000;
  Links[9].T_PREV2NEXT[0][3] = 0.000000;
  Links[9].T_PREV2NEXT[1][0] = -0.000000;
  Links[9].T_PREV2NEXT[1][1] = 1.000000;
  Links[9].T_PREV2NEXT[1][2] = 0.000000;
  Links[9].T_PREV2NEXT[1][3] = 0.000000;
  Links[9].T_PREV2NEXT[2][0] = 0.000000;
  Links[9].T_PREV2NEXT[2][1] = -0.000000;
  Links[9].T_PREV2NEXT[2][2] = 1.000000;
  Links[9].T_PREV2NEXT[2][3] = 0.000000;

  Links[10].NAME = "DPELVR_LINK";
  Links[10].MASS = 0.167108;
  Links[10].COM[0] = -0.000080;
  Links[10].COM[1] = -0.013873;
  Links[10].COM[2] = -0.018242;
  Links[10].NEXT = 11;
  Links[10].PREVIOUS = 9;
  Links[10].AXIS[0] = -0.000000;
  Links[10].AXIS[1] = -0.000000;
  Links[10].AXIS[2] = -1.000000;
  Links[10].T_PREV2NEXT[0][0] = 1.000000;
  Links[10].T_PREV2NEXT[0][1] = 0.000000;
  Links[10].T_PREV2NEXT[0][2] = 0.000000;
  Links[10].T_PREV2NEXT[0][3] = 0.000000;
  Links[10].T_PREV2NEXT[1][0] = 0.000000;
  Links[10].T_PREV2NEXT[1][1] = 1.000000;
  Links[10].T_PREV2NEXT[1][2] = 0.000000;
  Links[10].T_PREV2NEXT[1][3] = 0.000000;
  Links[10].T_PREV2NEXT[2][0] = 0.000000;
  Links[10].T_PREV2NEXT[2][1] = 0.000000;
  Links[10].T_PREV2NEXT[2][2] = 1.000000;
  Links[10].T_PREV2NEXT[2][3] = 0.000000;

  Links[11].NAME = "DLEGUPPERR_LINK";
  Links[11].MASS = 0.119043;
  Links[11].COM[0] = 0.000323;
  Links[11].COM[1] = -0.062966;
  Links[11].COM[2] = 0.000692;
  Links[11].NEXT = 12;
  Links[11].PREVIOUS = 10;
  Links[11].AXIS[0] = 1.000000;
  Links[11].AXIS[1] = 0.000000;
  Links[11].AXIS[2] = 0.000000;
  Links[11].T_PREV2NEXT[0][0] = 1.000000;
  Links[11].T_PREV2NEXT[0][1] = 0.000000;
  Links[11].T_PREV2NEXT[0][2] = 0.000000;
  Links[11].T_PREV2NEXT[0][3] = -0.000000;
  Links[11].T_PREV2NEXT[1][0] = 0.000000;
  Links[11].T_PREV2NEXT[1][1] = 1.000000;
  Links[11].T_PREV2NEXT[1][2] = 0.000000;
  Links[11].T_PREV2NEXT[1][3] = -0.093000;
  Links[11].T_PREV2NEXT[2][0] = 0.000000;
  Links[11].T_PREV2NEXT[2][1] = 0.000000;
  Links[11].T_PREV2NEXT[2][2] = 1.000000;
  Links[11].T_PREV2NEXT[2][3] = -0.000000;

  Links[12].NAME = "DLEGLOWERR_LINK";
  Links[12].MASS = 0.07031;
  Links[12].COM[0] = 0.000592;
  Links[12].COM[1] = 0.053955;
  Links[12].COM[2] = 0.006547;
  Links[12].NEXT = 13;
  Links[12].PREVIOUS = 11;
  Links[12].AXIS[0] = 1.000000;
  Links[12].AXIS[1] = 0.000000;
  Links[12].AXIS[2] = 0.000000;
  Links[12].T_PREV2NEXT[0][0] = 1.000000;
  Links[12].T_PREV2NEXT[0][1] = 0.000000;
  Links[12].T_PREV2NEXT[0][2] = 0.000000;
  Links[12].T_PREV2NEXT[0][3] = -0.000000;
  Links[12].T_PREV2NEXT[1][0] = 0.000000;
  Links[12].T_PREV2NEXT[1][1] = 1.000000;
  Links[12].T_PREV2NEXT[1][2] = 0.000000;
  Links[12].T_PREV2NEXT[1][3] = -0.093000;
  Links[12].T_PREV2NEXT[2][0] = 0.000000;
  Links[12].T_PREV2NEXT[2][1] = 0.000000;
  Links[12].T_PREV2NEXT[2][2] = 1.000000;
  Links[12].T_PREV2NEXT[2][3] = -0.000000;

  Links[13].NAME = "DANKLER_LINK";
  Links[13].MASS = 0.167108;
  Links[13].COM[0] = 0.000214;
  Links[13].COM[1] = 0.013873;
  Links[13].COM[2] = -0.018536;
  Links[13].NEXT = 14;
  Links[13].PREVIOUS = 12;
  Links[13].AXIS[0] = -1.000000;
  Links[13].AXIS[1] = -0.000000;
  Links[13].AXIS[2] = 0.000000;
  Links[13].T_PREV2NEXT[0][0] = 1.000000;
  Links[13].T_PREV2NEXT[0][1] = -0.000000;
  Links[13].T_PREV2NEXT[0][2] = 0.000000;
  Links[13].T_PREV2NEXT[0][3] = 0.000000;
  Links[13].T_PREV2NEXT[1][0] = 0.000000;
  Links[13].T_PREV2NEXT[1][1] = 1.000000;
  Links[13].T_PREV2NEXT[1][2] = -0.000000;
  Links[13].T_PREV2NEXT[1][3] = 0.000000;
  Links[13].T_PREV2NEXT[2][0] = -0.000000;
  Links[13].T_PREV2NEXT[2][1] = 0.000000;
  Links[13].T_PREV2NEXT[2][2] = 1.000000;
  Links[13].T_PREV2NEXT[2][3] = 0.000000;

  Links[14].NAME = "DFOOTR_LINK";
  Links[14].MASS = 0.079446;
  Links[14].COM[0] = -0.009506;
  Links[14].COM[1] = -0.025995;
  Links[14].COM[2] = -0.000503;
  Links[14].NEXT = -1;
  Links[14].PREVIOUS = 13;
  Links[14].AXIS[0] = -0.000000;
  Links[14].AXIS[1] = -0.000000;
  Links[14].AXIS[2] = 1.000000;
  Links[14].T_PREV2NEXT[0][0] = 1.000000;
  Links[14].T_PREV2NEXT[0][1] = 0.000000;
  Links[14].T_PREV2NEXT[0][2] = 0.000000;
  Links[14].T_PREV2NEXT[0][3] = 0.000000;
  Links[14].T_PREV2NEXT[1][0] = 0.000000;
  Links[14].T_PREV2NEXT[1][1] = 1.000000;
  Links[14].T_PREV2NEXT[1][2] = 0.000000;
  Links[14].T_PREV2NEXT[1][3] = 0.000000;
  Links[14].T_PREV2NEXT[2][0] = 0.000000;
  Links[14].T_PREV2NEXT[2][1] = 0.000000;
  Links[14].T_PREV2NEXT[2][2] = 1.000000;
  Links[14].T_PREV2NEXT[2][3] = 0.000000;

  Links[15].NAME = "DSHOULDERL_LINK";
  Links[15].MASS = 0.025913;
  Links[15].COM[0] = -0.013523;
  Links[15].COM[1] = 0.010264;
  Links[15].COM[2] = 0.001394;
  Links[15].NEXT = 16;
  Links[15].PREVIOUS = 0;
  Links[15].AXIS[0] = 1.000000;
  Links[15].AXIS[1] = -0.000000;
  Links[15].AXIS[2] = 0.000000;
  Links[15].T_PREV2NEXT[0][0] = 0.707106;
  Links[15].T_PREV2NEXT[0][1] = -0.707108;
  Links[15].T_PREV2NEXT[0][2] = -0.000000;
  Links[15].T_PREV2NEXT[0][3] = -0.000000;
  Links[15].T_PREV2NEXT[1][0] = 0.707108;
  Links[15].T_PREV2NEXT[1][1] = 0.707106;
  Links[15].T_PREV2NEXT[1][2] = 0.000000;
  Links[15].T_PREV2NEXT[1][3] = -0.016000;
  Links[15].T_PREV2NEXT[2][0] = -0.000000;
  Links[15].T_PREV2NEXT[2][1] = -0.000000;
  Links[15].T_PREV2NEXT[2][2] = 1.000000;
  Links[15].T_PREV2NEXT[2][3] = -0.000000;

  Links[16].NAME = "DARMUPPERL_LINK";
  Links[16].MASS = 0.168377;
  Links[16].COM[0] = 0.000660;
  Links[16].COM[1] = -0.036239;
  Links[16].COM[2] = 0.000734;
  Links[16].NEXT = 17;
  Links[16].PREVIOUS = 15;
  Links[16].AXIS[0] = 0.000000;
  Links[16].AXIS[1] = 0.000000;
  Links[16].AXIS[2] = -1.000000;
  Links[16].T_PREV2NEXT[0][0] = 1.000000;
  Links[16].T_PREV2NEXT[0][1] = 0.000000;
  Links[16].T_PREV2NEXT[0][2] = -0.000000;
  Links[16].T_PREV2NEXT[0][3] = 0.000000;
  Links[16].T_PREV2NEXT[1][0] = 0.000000;
  Links[16].T_PREV2NEXT[1][1] = 0.000000;
  Links[16].T_PREV2NEXT[1][2] = 1.000000;
  Links[16].T_PREV2NEXT[1][3] = -0.060000;
  Links[16].T_PREV2NEXT[2][0] = -0.000000;
  Links[16].T_PREV2NEXT[2][1] = -1.000000;
  Links[16].T_PREV2NEXT[2][2] = 0.000000;
  Links[16].T_PREV2NEXT[2][3] = 0.016000;

  Links[17].NAME = "DARMLOWERL_LINK";
  Links[17].MASS = 0.059288;
  Links[17].COM[0] = 0.006666;
  Links[17].COM[1] = -0.045838;
  Links[17].COM[2] = -0.013490;
  Links[17].NEXT = -1;
  Links[17].PREVIOUS = 16;
  Links[17].AXIS[0] = 1.000000;
  Links[17].AXIS[1] = -0.000000;
  Links[17].AXIS[2] = -0.000000;
  Links[17].T_PREV2NEXT[0][0] = 1.000000;
  Links[17].T_PREV2NEXT[0][1] = 0.000000;
  Links[17].T_PREV2NEXT[0][2] = 0.000000;
  Links[17].T_PREV2NEXT[0][3] = 0.000000;
  Links[17].T_PREV2NEXT[1][0] = 0.000000;
  Links[17].T_PREV2NEXT[1][1] = 1.000000;
  Links[17].T_PREV2NEXT[1][2] = 0.000000;
  Links[17].T_PREV2NEXT[1][3] = 0.000000;
  Links[17].T_PREV2NEXT[2][0] = 0.000000;
  Links[17].T_PREV2NEXT[2][1] = 0.000000;
  Links[17].T_PREV2NEXT[2][2] = 1.000000;
  Links[17].T_PREV2NEXT[2][3] = 0.000000;

  Links[18].NAME = "DSHOULDERR_LINK";
  Links[18].MASS = 0.025913;
  Links[18].COM[0] = 0.013523;
  Links[18].COM[1] = 0.010264;
  Links[18].COM[2] = 0.001394;
  Links[18].NEXT = 19;
  Links[18].PREVIOUS = 0;
  Links[18].AXIS[0] = -1.000000;
  Links[18].AXIS[1] = 0.000000;
  Links[18].AXIS[2] = 0.000000;
  Links[18].T_PREV2NEXT[0][0] = 0.707106;
  Links[18].T_PREV2NEXT[0][1] = 0.707108;
  Links[18].T_PREV2NEXT[0][2] = -0.000000;
  Links[18].T_PREV2NEXT[0][3] = -0.000000;
  Links[18].T_PREV2NEXT[1][0] = -0.707108;
  Links[18].T_PREV2NEXT[1][1] = 0.707106;
  Links[18].T_PREV2NEXT[1][2] = -0.000000;
  Links[18].T_PREV2NEXT[1][3] = -0.016000;
  Links[18].T_PREV2NEXT[2][0] = 0.000000;
  Links[18].T_PREV2NEXT[2][1] = -0.000000;
  Links[18].T_PREV2NEXT[2][2] = 1.000000;
  Links[18].T_PREV2NEXT[2][3] = 0.000000;

  Links[19].NAME = "DARMUPPERR_LINK";
  Links[19].MASS = 0.168377;
  Links[19].COM[0] = -0.000660;
  Links[19].COM[1] = -0.036239;
  Links[19].COM[2] = 0.000734;
  Links[19].NEXT = 20;
  Links[19].PREVIOUS = 18;
  Links[19].AXIS[0] = -0.000000;
  Links[19].AXIS[1] = 0.000000;
  Links[19].AXIS[2] = -1.000000;
  Links[19].T_PREV2NEXT[0][0] = 1.000000;
  Links[19].T_PREV2NEXT[0][1] = -0.000000;
  Links[19].T_PREV2NEXT[0][2] = 0.000000;
  Links[19].T_PREV2NEXT[0][3] = -0.000000;
  Links[19].T_PREV2NEXT[1][0] = -0.000000;
  Links[19].T_PREV2NEXT[1][1] = 0.000000;
  Links[19].T_PREV2NEXT[1][2] = 1.000000;
  Links[19].T_PREV2NEXT[1][3] = -0.060000;
  Links[19].T_PREV2NEXT[2][0] = -0.000000;
  Links[19].T_PREV2NEXT[2][1] = -1.000000;
  Links[19].T_PREV2NEXT[2][2] = 0.000000;
  Links[19].T_PREV2NEXT[2][3] = 0.016000;

  Links[20].NAME = "DARMLOWERR_LINK";
  Links[20].MASS = 0.059288;
  Links[20].COM[0] = -0.006666;
  Links[20].COM[1] = -0.045838;
  Links[20].COM[2] = -0.013490;
  Links[20].NEXT = -1;
  Links[20].PREVIOUS = 19;
  Links[20].AXIS[0] = -1.000000;
  Links[20].AXIS[1] = 0.000000;
  Links[20].AXIS[2] = -0.000000;
  Links[20].T_PREV2NEXT[0][0] = 1.000000;
  Links[20].T_PREV2NEXT[0][1] = 0.000000;
  Links[20].T_PREV2NEXT[0][2] = 0.000000;
  Links[20].T_PREV2NEXT[0][3] = 0.000000;
  Links[20].T_PREV2NEXT[1][0] = 0.000000;
  Links[20].T_PREV2NEXT[1][1] = 1.000000;
  Links[20].T_PREV2NEXT[1][2] = 0.000000;
  Links[20].T_PREV2NEXT[1][3] = 0.000000;
  Links[20].T_PREV2NEXT[2][0] = 0.000000;
  Links[20].T_PREV2NEXT[2][1] = 0.000000;
  Links[20].T_PREV2NEXT[2][2] = 1.000000;
  Links[20].T_PREV2NEXT[2][3] = 0.000000;

  Chains[0].FIRST = 1;
  Chains[0].T_FROM_BODY[0][0] = 1.000000;
  Chains[0].T_FROM_BODY[0][1] = -0.000000;
  Chains[0].T_FROM_BODY[0][2] = -0.000000;
  Chains[0].T_FROM_BODY[0][3] = 0.000000;
  Chains[0].T_FROM_BODY[1][0] = -0.000000;
  Chains[0].T_FROM_BODY[1][1] = 1.000000;
  Chains[0].T_FROM_BODY[1][2] = -0.000000;
  Chains[0].T_FROM_BODY[1][3] = 0.051000;
  Chains[0].T_FROM_BODY[2][0] = -0.000000;
  Chains[0].T_FROM_BODY[2][1] = -0.000000;
  Chains[0].T_FROM_BODY[2][2] = 1.000000;
  Chains[0].T_FROM_BODY[2][3] = -0.000000;

  Chains[1].FIRST = 3;
  Chains[1].T_FROM_BODY[0][0] = 1.000000;
  Chains[1].T_FROM_BODY[0][1] = -0.000000;
  Chains[1].T_FROM_BODY[0][2] = -0.000000;
  Chains[1].T_FROM_BODY[0][3] = 0.037000;
  Chains[1].T_FROM_BODY[1][0] = -0.000000;
  Chains[1].T_FROM_BODY[1][1] = 1.000000;
  Chains[1].T_FROM_BODY[1][2] = -0.000000;
  Chains[1].T_FROM_BODY[1][3] = -0.122200;
  Chains[1].T_FROM_BODY[2][0] = -0.000000;
  Chains[1].T_FROM_BODY[2][1] = -0.000000;
  Chains[1].T_FROM_BODY[2][2] = 1.000000;
  Chains[1].T_FROM_BODY[2][3] = -0.005000;

  Chains[2].FIRST = 9;
  Chains[2].T_FROM_BODY[0][0] = 1.000000;
  Chains[2].T_FROM_BODY[0][1] = -0.000000;
  Chains[2].T_FROM_BODY[0][2] = -0.000000;
  Chains[2].T_FROM_BODY[0][3] = -0.037000;
  Chains[2].T_FROM_BODY[1][0] = -0.000000;
  Chains[2].T_FROM_BODY[1][1] = 1.000000;
  Chains[2].T_FROM_BODY[1][2] = -0.000000;
  Chains[2].T_FROM_BODY[1][3] = -0.122200;
  Chains[2].T_FROM_BODY[2][0] = -0.000000;
  Chains[2].T_FROM_BODY[2][1] = -0.000000;
  Chains[2].T_FROM_BODY[2][2] = 1.000000;
  Chains[2].T_FROM_BODY[2][3] = -0.005000;

  Chains[3].FIRST = 15;
  Chains[3].T_FROM_BODY[0][0] = 1.000000;
  Chains[3].T_FROM_BODY[0][1] = -0.000000;
  Chains[3].T_FROM_BODY[0][2] = -0.000000;
  Chains[3].T_FROM_BODY[0][3] = 0.082000;
  Chains[3].T_FROM_BODY[1][0] = -0.000000;
  Chains[3].T_FROM_BODY[1][1] = 1.000000;
  Chains[3].T_FROM_BODY[1][2] = -0.000000;
  Chains[3].T_FROM_BODY[1][3] = 0.000000;
  Chains[3].T_FROM_BODY[2][0] = -0.000000;
  Chains[3].T_FROM_BODY[2][1] = -0.000000;
  Chains[3].T_FROM_BODY[2][2] = 1.000000;
  Chains[3].T_FROM_BODY[2][3] = 0.000000;

  Chains[4].FIRST = 18;
  Chains[4].T_FROM_BODY[0][0] = 1.000000;
  Chains[4].T_FROM_BODY[0][1] = -0.000000;
  Chains[4].T_FROM_BODY[0][2] = -0.000000;
  Chains[4].T_FROM_BODY[0][3] = -0.082000;
  Chains[4].T_FROM_BODY[1][0] = -0.000000;
  Chains[4].T_FROM_BODY[1][1] = 1.000000;
  Chains[4].T_FROM_BODY[1][2] = -0.000000;
  Chains[4].T_FROM_BODY[1][3] = -0.000000;
  Chains[4].T_FROM_BODY[2][0] = -0.000000;
  Chains[4].T_FROM_BODY[2][1] = -0.000000;
  Chains[4].T_FROM_BODY[2][2] = 1.000000;
  Chains[4].T_FROM_BODY[2][3] = -0.000000;

}
from openravepy import *

#--------------------------------------------------------
def getAxis(axis, trans):
    newAxis = numpy.zeros(3)
    for i in range(3):
        newAxis[i] = sum(trans[j][i] * axis[j] for j in range(3))
    return newAxis

def getP2N(prev, next):

    nextR = next[0:3,0:3]
    nextT = next[0:3,3]
    prevR = prev[0:3,0:3]
    prevT = prev[0:3,3]
    P2N = numpy.eye(4)
    # Rotation is nextR*prevR^T
    for i in range(3):
        for j in range(3):
            P2N[i][j] = sum(prevR[k][i]*nextR[k][j] for k in range(3))
        P2N[i][3] = sum(prevR[k][i]*(nextT[k]-prevT[k]) for k in range(3))
    return P2N

#--------------------------------------------------------
# These numbers are taken from SourceForge
# darwinop/Hardware/Mechanics/Phisical%20Information/DARwIn-OP_Dynamics.zip
COM = numpy.zeros((21,3))
COM[0] = [-.003116, -.039444, -.019663]
COM[1] = [ .001424, -.016568, -.000713]
COM[2] = [ .000064,  .018565,  .076666]
COM[3] = [ .000000,  .018437,  .000480]
COM[4] = [ .000080, -.013873, -.018242]
COM[5] = [-.000323, -.062966,  .000692]
COM[6] = [-.000592,  .053955,  .006547]
COM[7] = [-.000214,  .013873, -.018536]
COM[8] = [ .009506, -.025995, -.000503]
COM[9] = [-.000000,  .018437,  .000480]
COM[10] = [-.000080, -.013873, -.018242]
COM[11] = [ .000323, -.062966,  .000692]
COM[12] = [ .000592,  .053955,  .006547]
COM[13] = [ .000214,  .013873, -.018536]
COM[14] = [-.009506, -.025995, -.000503]
COM[15] = [-.013523,  .010264,  .001394]
COM[16] = [ .000660, -.036239,  .000734]
COM[17] = [ .006666, -.045838, -.013490]
COM[18] = [ .013523,  .010264,  .001394]
COM[19] = [-.000660, -.036239,  .000734]
COM[20] = [-.006666, -.045838, -.013490]

#--------------------------------------------------------
env = Environment()
env.Load('robots/romela-darwin-op.dae')
robot = env.GetRobots()[0]
links = robot.GetLinks()
joints = robot.GetJoints()
chainnames = ['Head', 'LeftLeg', 'RightLeg', 'LeftArm', 'RightArm']
lasts = [2,8,14,17,20] # index of links at the end of chains
firsts = [1,3,9,15,18] # begin of chains
body = [0] # The body link
env.SetViewer('qtcoin')
h = env.plot3(links[2].GetGlobalCOM(),10)

with open('DarwinDynamicDefinitions.h','w') as output:
    # Head of the file
    output.write("""\
/*
This file contains some information about the dynamics of the Darwin robot.
This includes the mass of each link, the transform of center of mass for
each link in the local frams, and the position of each of the links.
*/

#include <string>
#include "Transform3.h"

struct link;
""")
    
    # Data structures to hold the links and chains
    output.write(
"""
struct link{
  std::string NAME;
  float MASS;              // Mass of link
  vec3f COM;            // Center of mass in the link frame
  int PREVIOUS;            // Index of previous link
  int NEXT;                // Index of next link (-1 for end links)
  Transform3f T_PREV2NEXT; // Transform from link frame to next joint
  vec3f AXIS;           // Axis of rotation for this link
};

""")
    output.write(
"""
struct chain{
  int FIRST;
  Transform3f T_FROM_BODY;
};

""")

    output.write(
"""
class Darwin{{
  public:
    Darwin();
    struct chain Chains[{numchains}];
    struct link Links[{numlinks}];
}};
""".format(numchains=str(len(lasts)), numlinks=str(len(links))))

with open('DarwinDynamicDefinitions.cpp','w') as output:

    output.write(
"""
#include "DarwinDynamicDefinitions.h"

Darwin::Darwin(){
  float t_array[16];
  mat4f t;

""")

    # Information about each link
    for i in range(len(links)):
        if i in lasts + body:
            transform = numpy.zeros((4,4))
            transform[0:3,0:3] = numpy.eye(3)
        else:
            transform = getP2N(links[i].GetTransform(), 
                               links[i+1].GetTransform())
        output.write(
"""\
  Links[{index}].NAME = \"{name}\";
  Links[{index}].MASS = {mass};
  Links[{index}].COM = vec3f({com});
  Links[{index}].NEXT = {nextlink};
  Links[{index}].PREVIOUS = {prevlink};
  Links[{index}].AXIS = vec3f({axis});
{p2n}
  t = mat4f(t_array);
  Links[{index}].T_PREV2NEXT = Transform3f(t);

""".format(index=str(i), 
           name = links[i].GetName(),
           mass=str(links[i].GetMass()),

           com=','.join(('%f' % COM[i][j]) for j in range(3)),

           nextlink="-1" if i in lasts else 
           "{nextindex}".format(nextindex=str(i+1)),
           prevlink="0" if i in firsts else "-1" if i in body else 
           "{previndex}".format(previndex=str(i-1)),

           axis=','.join(
                    ('%f' % getAxis(joints[i-1].GetAxis(), 
                                    links[i].GetTransform())[j]) 
                    for j in range(3)),

           p2n='\n'.join(
                    ('  t_array[{index}] = {transvalue};'
                     .format(index=j*4+k, transvalue='%f' 
                            % transform[j][k] ))
                    for j in range(4) for k in range(4)),

)
)

    for i in range(len(firsts)):
        
        transform_first = getP2N(links[body[0]].GetTransform(),
                                 links[firsts[i]].GetTransform())
        output.write(
"""\
  Chains[{index}].FIRST = {first};
{transform}
  t = mat4f(t_array);
  Chains[{index}].T_FROM_BODY = Transform3f(t);

""".format(index=i,first=firsts[i], 
           transform='\n'.join(
                    ('  t_array[{index}] = {transvalue};'
                     .format(index=j*4+k, transvalue='%f'
                             % transform_first[j][k] )) 
                    for j in range(4) for k in range(4))))

    output.write('}')


print links[19].GetTransform()
print links[20].GetTransform()
print getP2N(links[19].GetTransform(), links[20].GetTransform())
raw_input()

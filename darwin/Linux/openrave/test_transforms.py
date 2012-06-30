from openravepy import *

def getP2N(prev, next):

    nextR = next[0:3,0:3]
    nextT = next[0:3,3]
    prevR = prev[0:3,0:3]
    prevT = prev[0:3,3]
    P2N = numpy.zeros((3,4))
    # Rotation is nextR*prevR^T
    for i in range(3):
        for j in range(3):
            P2N[i][j] = sum(prevR[k][i]*nextR[k][j] for k in range(3))
        P2N[i][3] = sum(prevR[k][i]*(nextT[k]-prevT[k]) for k in range(3))
    return P2N



env = Environment()
env.Load('robots/romela-darwin-op.dae')
robot = env.GetRobots()[0]
links = robot.GetLinks()
joints = robot.GetJoints()

robot.SetDOFValues([.045662, -.037544, .008118],[4,5,6])
robot.SetDOFValues([-.045662, .037544, -.008118],[10,11,12])

for i in range(20):
    print ''
    print i+1
    print getP2N(links[0].GetTransform(), links[i+1].GetTransform())

env.SetViewer('qtcoin')
raw_input('')

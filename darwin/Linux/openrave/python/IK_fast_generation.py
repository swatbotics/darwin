from openravepy import *
from openravepy import ikfast
env = Environment()
kinbody = env.ReadRobotXMLFile('xml/romela-darwin-op.dae')
env.AddRobot(kinbody)
solver = ikfast.IKFastSolver(kinbody=kinbody)
manip = kinbody.SetActiveManipulator(2)
chaintree = solver.generateIkSolver(baselink=manip.GetBase(),eelink=8,freeindices=manip.GetArmIndices(),solvefn=ikfast.IKFastSolver.solveFullIK_6D)
code = solver.writeIkSolver(chaintree)
open('ik.cpp','w').write(code)

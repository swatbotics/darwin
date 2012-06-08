from openravepy import *
env = Environment()
kinbody = env.ReadRobotXMLFile('robots/romela-darwin-op.dae')
env.AddRobot(kinbody)
solver = ikfast.IKFastSolver(kinbody=kinbody)
chaintree = solver.generateIkSolver(baselink=0,eelink=8,freeindices=None,solvefn=ikfast.IKFastSolver.solveFullIK_6D)
code = solver.writeIkSolver(chaintree)
open('ik.cpp','w').write(code)

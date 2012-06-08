from openravepy import *
env = Environment() # create the environment
env.SetViewer('qtcoin') # start the viewer
env.Load('robots/romela-darwin-op.dae') # load a scene
robot = env.GetRobots()[0] # get the first robot

manip = robot.SetActiveManipulator(2)
print manip
ikmodel = databases.inversekinematics.InverseKinematicsModel(robot,iktype=IkParameterization.Type.Transform6D)
if not ikmodel.load():
    ikmodel.autogenerate()

with robot: # lock environment and save robot state
   # robot.SetDOFValues([2.58, 0.547, 1.5, -0.7],[0,1,2,3]) # set the first 4 dof values
    Tee = manip.GetEndEffectorTransform() # get end effector
    Tee[2,3] = Tee[2,3]+.02
    print Tee
   # ikparam = IkParameterization(Tee[0:3,3],ikmodel.iktype) # build up the translation3d ik query
    sols = manip.FindIKSolutions(Tee, 0)# IkFilterOptions.CheckEnvCollisions) # get all solutions

print sols
h = env.plot3(Tee[0:3,3],10) # plot one point
raw_input('press any key')
with robot: # save robot state
    for sol in sols: # go through every 10th solution
        robot.SetDOFValues(sol,manip.GetArmIndices()) # set the current solution
        env.UpdatePublishedBodies() # allow viewer to update new robot
        raw_input('press any key')

# raveLogInfo('restored dof values: '+repr(robot.GetDOFValues())) # robot state is restored to original

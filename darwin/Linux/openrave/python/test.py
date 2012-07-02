from openravepy import *
import time


env = Environment()

env.Reset()
env.Load('robots/romela-darwin-op.dae')
    
robot = env.GetRobots()[0]
robot.SetDOFValues([0.5],[0])


env.SetViewer('qtcoin')

while 1:
    time.sleep(1)





#try:
#    result = RaveInitialize(load_all_plugins=False)
#    print result
#finally:
#    RaveDestroy() # destroy the runtime

    

import pybullet as p
import time
import math
from datetime import datetime

#clid = p.connect(p.SHARED_MEMORY)
import pybullet_data

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)
p.loadURDF("data/plane.urdf", [0, 0, 0.0], useFixedBase=True)
kukaId = p.loadURDF("data/sia5/model.urdf", [0, 0, 0], useFixedBase=True)
p.resetBasePositionAndOrientation(kukaId, [0, 0, 0], [0, 0, 0, 1])
kukaEndEffectorIndex = 8
numJoints = p.getNumJoints(kukaId)
print(numJoints)
if (numJoints != 9):
  exit()

p.loadURDF("data/cube.urdf", [2, 2, 5])
p.loadURDF("data/cube.urdf", [-2, -2, 5])
p.loadURDF("data/cube.urdf", [2, -2, 5])
#ballId = p.loadSoftBody("mininoodle.obj", simFileName = "dmininoodle.vtk", basePosition = [-2,2,5], scale = 0.5, mass = 4, useNeoHookean = 1, NeoHookeanMu = 400, NeoHookeanLambda = 600, NeoHookeanDamping = 0.001, useSelfCollision = 1, frictionCoeff = .5, collisionMargin = 0.001)
ballId = p.loadSoftBody("worldorigin.obj", simFileName = "worldorigin.vtk", basePosition = [-2,2,3], scale = 0.5, mass = 4, useNeoHookean = 1, NeoHookeanMu = 400, NeoHookeanLambda = 600, NeoHookeanDamping = 0.001, useSelfCollision = 1, frictionCoeff = .5, collisionMargin = 0.001)
p.setGravity(0, 0, -10)
t = 0.
prevPose = [0, 0, 0]
prevPose1 = [0, 0, 0]
hasPrevPose = 0
useNullSpace = 0

count = 0
useOrientation = 1
useSimulation = 1
useRealTimeSimulation = 1
p.setRealTimeSimulation(useRealTimeSimulation)
#trailDuration is duration (in seconds) after debug lines will be removed automatically
#use 0 for no-removal
trailDuration = 15

logId1 = p.startStateLogging(p.STATE_LOGGING_GENERIC_ROBOT, "data/LOG0001.txt", [0, 1, 2])
logId2 = p.startStateLogging(p.STATE_LOGGING_CONTACT_POINTS, "data/LOG0002.txt", bodyUniqueIdA=2)

while 1:
  if (useRealTimeSimulation):
    dt = datetime.now()
    t = (dt.second / 60.) * 2. * math.pi
  else:
    t = t + 0.1##

  if (useSimulation and useRealTimeSimulation == 0):
    p.stepSimulation()
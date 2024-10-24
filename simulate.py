import pybullet as p
import pybullet_data
import pyrosim.pyrosim as ps
import numpy as np
import time 
import matplotlib.pyplot as plt

physicsClient = p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.setGravity(0,0,-9.8)
planeId = p.loadURDF("plane.urdf")
#p.loadSDF("box.sdf")
robotId = p.loadURDF("phil_the_robot.urdf")

time.sleep(1)

duration = 10000

ps.Prepare_To_Simulate(robotId)

x = np.linspace(0,10*np.pi, duration)
y = np.sin(x)*np.pi/2*6

leftWheelPos = np.zeros(duration)
leftWheelVel = np.zeros(duration)

rightWheelPos = np.zeros(duration)
rightWheelVel = np.zeros(duration)

for i in range(duration):
    ps.Set_Motor_For_Joint(bodyIndex = robotId, 
                           jointName = b'axle_left_joint',
                           controlMode = p.POSITION_CONTROL,
                           targetPosition = y[i],
                           maxForce = 500)
    ps.Set_Motor_For_Joint(bodyIndex = robotId, 
                           jointName = b'axle_right_joint',
                           controlMode = p.POSITION_CONTROL,
                           targetPosition = y[i],
                           maxForce = 500)
    
    leftWheelState = p.getJointState(robotId, 1)
    leftWheelPos[i] = leftWheelState[0]
    leftWheelVel[i] = leftWheelState[1]

    rightWheelState = p.getJointState(robotId, 2)
    rightWheelPos[i] = rightWheelState[0]
    rightWheelVel[i] = rightWheelState[1]

    p.stepSimulation()
    time.sleep(1/500)



xcoords = np.arange(duration)
fig, axs = plt.subplots(2, 2)

axs[0, 0].plot(xcoords, leftWheelPos)
axs[0, 0].set_title('Left Wheel Position')

axs[0, 1].plot(xcoords, rightWheelPos)
axs[0, 1].set_title('Right Wheel Position')

axs[1, 0].plot(xcoords, leftWheelVel)
axs[1, 0].set_title('Left Wheel Velocity')

axs[1, 1].plot(xcoords, rightWheelVel)
axs[1, 1].set_title('Right Wheel Velocity')

for ax in axs.flat:
    ax.set(xlabel='Steps', ylabel='Units')

for ax in axs.flat:
    ax.label_outer()

plt.show()

p.disconnect()
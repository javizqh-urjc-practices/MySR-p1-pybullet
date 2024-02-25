import pybullet as p
import pybullet_data
import time
import csv

physicsClient = p.connect (p.GUI)
p.setAdditionalSearchPath (pybullet_data.getDataPath())
p.setRealTimeSimulation(1)
p.setGravity (0, 0, -9.81)

planeId = p.loadURDF ("plane.urdf")

euler_angles = [0, 0, 0]
startOrientation = p.getQuaternionFromEuler(euler_angles)
startPosition = [0, 0, 0.1]

robotId = p.loadURDF ("husky/husky.urdf", startPosition, startOrientation)

startPosition = [10, 0, 0.1]
slopeId = p.loadURDF ("models/rampa.urdf", startPosition, startOrientation)

startPosition = [17, 1.5, 0.1]
barrierId = p.loadURDF ("models/barrera.urdf", startPosition, startOrientation)

startPosition = [20, 0, 0.1]
goalId = p.loadURDF ("models/goal.urdf", startPosition, startOrientation)


# Only to know what is the number of the wheels
numJoints = p.getNumJoints(barrierId)
# print("NumJoints: " + str(numJoints))

for j in range (numJoints):
    print("%d - %s" % (p.getJointInfo(barrierId,j)[0], p.getJointInfo(barrierId,j)[1].decode("utf-8")))

joints = [2,3,4,5]

speedId = p.addUserDebugParameter("HUSKY_speed", -20, 20, 0)
forceId = p.addUserDebugParameter("HUSKY_force", 0, 40, 5)

lastDistance = -1
origTime = time.time()

p.changeDynamics(barrierId, 0, localInertiaDiagonal=[20/3,0.0,20/3])

with open('data/Fase3.csv', 'w', newline='', encoding='utf-8') as csvfile:
    csv_writer = csv.writer(csvfile, delimiter=',')
    csv_writer.writerow(['Time', 'Position_Robot', 'Speed_Robot', 'Speed_Wheel', 'Force_Wheel'])

    while p.getBasePositionAndOrientation(robotId)[0][0] <= 20.0 :
        speed = p.readUserDebugParameter(speedId)
        torque = p.readUserDebugParameter(forceId)
        speed = 11.5
        torque = 25.0

        p.setJointMotorControlArray(robotId,
                                    joints,
                                    p.VELOCITY_CONTROL,
                                    targetVelocities=[speed,speed,speed,speed],
                                    forces=[torque,torque,torque,torque])
        
        for wheel in joints:
            p.changeDynamics(robotId, wheel, lateralFriction=0.93)
            p.changeDynamics(robotId, wheel, spinningFriction=0.05)
            p.changeDynamics(robotId, wheel, rollingFriction=0.003)
                
        distance = p.getBasePositionAndOrientation(robotId)[0][0]
        if (distance != lastDistance):
            csv_writer.writerow([time.time() - origTime, distance, p.getBaseVelocity(robotId)[0][0],
                                 speed, torque])
            lastDistance = distance

p.disconnect()
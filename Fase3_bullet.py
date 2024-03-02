import pybullet as p
import pybullet_data
import time
import csv

physics_client = p.connect (p.GUI)
p.setAdditionalSearchPath (pybullet_data.getDataPath())
p.setRealTimeSimulation(1)
p.setGravity (0, 0, -9.81)

plane_id = p.loadURDF ("plane.urdf")

euler_angles = [0, 0, 0]
start_orientation = p.getQuaternionFromEuler(euler_angles)
start_position = [0, 0, 0.1]

robot_id = p.loadURDF ("husky/husky.urdf", start_position, start_orientation)

start_position = [10, 0, 0.1]
slope_id = p.loadURDF ("models/rampa.urdf", start_position, start_orientation)

start_position = [17, 1.5, 0.1]
barrier_id = p.loadURDF ("models/barrera.urdf", start_position, start_orientation)

start_position = [20, 0, 0.1]
goal_id = p.loadURDF ("models/goal.urdf", start_position, start_orientation)


# Only to know what is the number of the wheels
# numJoints = p.getNumJoints(barrier_id)
# print("NumJoints: " + str(numJoints))

#for j in range (numJoints):
#    print("%d - %s" % (p.getJointInfo(barrier_id,j)[0], p.getJointInfo(barrier_id,j)[1].decode("utf-8")))

joints = [2,3,4,5]

last_distance = -1
start_time = time.time()

p.changeDynamics(barrier_id, 0, localInertiaDiagonal=[20/3,0.0,20/3])
csv_values = []

while p.getBasePositionAndOrientation(robot_id)[0][0] <= 20.0 :
    speed = 11.5
    torque = 25.0

    p.setJointMotorControlArray(robot_id,
                                joints,
                                p.VELOCITY_CONTROL,
                                targetVelocities=[speed, speed, speed, speed],
                                forces=[torque, torque, torque, torque])
    
    p.resetDebugVisualizerCamera( cameraDistance=6, cameraYaw=30, cameraPitch=-52, cameraTargetPosition=p.getBasePositionAndOrientation(robot_id)[0])
    
    for wheel in joints:
        p.changeDynamics(robot_id, wheel, lateralFriction=0.93)
        p.changeDynamics(robot_id, wheel, spinningFriction=0.05)
        p.changeDynamics(robot_id, wheel, rollingFriction=0.003)
            
    distance = p.getBasePositionAndOrientation(robot_id)[0][0]
    if (distance != last_distance):
        csv_values.append([time.time() - start_time, distance, p.getBaseVelocity(robot_id)[0][0], speed, torque])
        last_distance = distance

p.disconnect()

with open('data/Fase3.csv', 'w', newline='', encoding='utf-8') as csvfile:
    csv_writer = csv.writer(csvfile, delimiter=',')
    csv_writer.writerow(['Time', 'Position_Robot', 'Speed_Robot', 'Speed_Wheel', 'Force_Wheel'])
    for i in csv_values:
    	csv_writer.writerow([i[0],i[1],i[2],i[3],i[4]])

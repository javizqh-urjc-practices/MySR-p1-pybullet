import pybullet as p
import pybullet_data
import time
import csv
import pandas as pd
from scipy.spatial.transform import Rotation

def clamp(n, min, max):
    if n < min:
        return min
    elif n > max:
        return max
    else:
        return n

class PID:
  def __init__(self, min_ref, max_ref, min_output, max_output):
      self.min_ref_ = min_ref
      self.max_ref_ = max_ref
      self.min_output_ = min_output
      self.max_output_ = max_output
      self.prev_error_ = self.int_error_ = 0.0
    
      self.KP_ = 0.41
      self.KI_ = 0.06
      self.KD_ = 0.53
  
  def setPid(self, n_KP, n_KI, n_KD):
      self.KP_ = n_KP
      self.KI_ = n_KI
      self.KD_ = n_KD
  
  def getOutput(self, new_reference):
    ref = new_reference
    output = 0.0
  
    # Proportional Error
    direction = 0.0
    if ref != 0.0:
      direction = ref / abs(ref)
  
    if abs(ref) < self.min_ref_ :
      output = 0.0
      # output = direction * self.min_output_
    elif abs(ref) > self.max_ref_:
      output = direction * self.max_output_ * self.max_ref_
      # output = 0.0
    else:
      output = direction * self.min_output_ + ref * (self.max_output_ - self.min_output_)
  
    # Integral Error
    self.int_error_ = (self.int_error_ + output) * 2.0 / 3.0
  
    # Derivative Error
    deriv_error = output - self.prev_error_
    self.prev_error_ = output
  
    output = self.KP_ * output + self.KI_ * self.int_error_ + self.KD_ * deriv_error
  
    return clamp(output / self.max_ref_, self.min_output_, self.max_output_)

# For speed based on the robot speed
pidLin = PID(0,3,5,47)
pidLin.setPid(0.3,0.2,1)

# For speed based on the inclination
pidLin2 = PID(0,50,-5,35)
pidLin2.setPid(0.62,0.02,3)

# For torque based on the inclination
pidTorq = PID(0,90,50,400)
pidTorq.setPid(0.6,1,2)


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
numJoints = p.getNumJoints(robot_id)
# print("NumJoints: " + str(numJoints))

for j in range (numJoints):
    print("%d - %s" % (p.getJointInfo(robot_id,j)[0], p.getJointInfo(robot_id,j)[1].decode("utf-8")))

joints = [2,3,4,5]

last_distance = -1
start_time = time.time()

p.changeDynamics(barrier_id, 0, localInertiaDiagonal=[20/3,0.0,20/3])
csv_values = []

while p.getBasePositionAndOrientation(robot_id)[0][0] <= 20.0 :
	carVel = p.getBaseVelocity(robot_id)[0][0]

	rot = Rotation.from_quat(p.getBasePositionAndOrientation(robot_id)[1])
	rot_euler = rot.as_euler('xyz', degrees=True)
	   
	torque  = pidTorq.getOutput(abs(rot_euler[1]))
	speed = pidLin.getOutput(3-carVel)	
	speed2  = pidLin2.getOutput(-rot_euler[1])

  # In order to be able to see the robot while moving 
	p.resetDebugVisualizerCamera( cameraDistance=6, cameraYaw=30, cameraPitch=-52, cameraTargetPosition=p.getBasePositionAndOrientation(robot_id)[0])

	for wheel in joints:
	    p.changeDynamics(robot_id, wheel, lateralFriction=0.93)
	    p.changeDynamics(robot_id, wheel, spinningFriction=0.05)
	    p.changeDynamics(robot_id, wheel, rollingFriction=0.003)

	p.setJointMotorControlArray(robot_id,
		                    joints,
		                    p.VELOCITY_CONTROL,
		                    targetVelocities=[speed, speed, speed + speed2, speed + speed2],
		                    forces=[torque, torque, torque, torque])
	    
	distance = p.getBasePositionAndOrientation(robot_id)[0][0]
	if (distance != last_distance):
		csv_values.append([time.time() - start_time, distance, carVel, speed, torque])
		last_distance = distance

p.disconnect()

with open('data/Fase4.csv', 'w', newline='', encoding='utf-8') as csvfile:
    csv_writer = csv.writer(csvfile, delimiter=',')
    csv_writer.writerow(['Time', 'Position_Robot', 'Speed_Robot', 'Speed_Wheel', 'Force_Wheel'])
    for i in csv_values:
    	csv_writer.writerow([i[0],i[1],i[2],i[3],i[4]])

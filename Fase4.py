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

pidLin = PID(0,3,5,47)
pidLin.setPid(0.3,0.2,1)

pidLin2 = PID(0,50,-5,35)
pidLin2.setPid(0.6,0.008,3)

pidTorq = PID(0,90,50,400)
pidTorq.setPid(0.6,1,2)


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
numJoints = p.getNumJoints(robotId)
# print("NumJoints: " + str(numJoints))

for j in range (numJoints):
    print("%d - %s" % (p.getJointInfo(robotId,j)[0], p.getJointInfo(robotId,j)[1].decode("utf-8")))

joints = [2,3,4,5]

lastDistance = -1
origTime = time.time()

p.changeDynamics(barrierId, 0, localInertiaDiagonal=[20/3,0.0,20/3])

with open('data/Fase4.csv', 'w', newline='', encoding='utf-8') as csvfile:
    csv_writer = csv.writer(csvfile, delimiter=',')
    csv_writer.writerow(['Time', 'Position_Robot', 'Speed_Robot', 'Speed_Wheel', 'Force_Wheel'])

    while p.getBasePositionAndOrientation(robotId)[0][0] <= 20.0 :
        carVel = p.getBaseVelocity(robotId)[0][0]

        rot = Rotation.from_quat(p.getBasePositionAndOrientation(robotId)[1])
        rot_euler = rot.as_euler('xyz', degrees=True)
           
        torque  = pidTorq.getOutput(abs(rot_euler[1]))
        speed   = pidLin.getOutput(3-carVel)
        speed2  = pidLin2.getOutput(-rot_euler[1])

        for wheel in joints:
            p.changeDynamics(robotId, wheel, lateralFriction=0.93)
            p.changeDynamics(robotId, wheel, spinningFriction=0.05)
            p.changeDynamics(robotId, wheel, rollingFriction=0.003)

        p.setJointMotorControlArray(robotId,
                                    joints,
                                    p.VELOCITY_CONTROL,
                                    targetVelocities=[speed,speed,speed+speed2,speed+speed2],
                                    forces=[torque,torque,torque,torque])
            
        distance = p.getBasePositionAndOrientation(robotId)[0][0]
        if (distance != lastDistance):
            csv_writer.writerow([time.time() - origTime, distance, carVel,
                                 speed, torque])
            lastDistance = distance

p.disconnect()

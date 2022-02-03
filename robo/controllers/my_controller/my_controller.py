from controller import Robot, DistanceSensor, Motor, Camera
from scipy.optimize import fsolve
from matplotlib import pyplot as plt
import math

# create the Robot instance.
robot = Robot()
numOfObjectsOnTheTable = 0
pi = 3.14159
MAX_SPEED = pi/2

def getAnglesFormPos(x, y, z):
    if x == 0:
     x = 1
    A = 300   #mm
    D = 1095
    F = 1270
    def equations(p):
        x1, x2, x3 = p
        return (-z + 495 + D * math.cos(x2) + 175 * math.cos(x2+x3) - F * math.sin(x2+x3), -math.sqrt((x-A*math.sin(x1))**2 + (y + A * math.cos(x1))**2) + 175 + D * math.sin(x2) + 175 * math.sin(x2+x3) + F * math.cos(x2+x3), x1 - math.atan(y/x)-math.atan(A/math.sqrt((x-A*math.sin(x1))**2 + (y + A * math.cos(x1))**2)))

    tetha1, tetha2, tetha3 = fsolve(equations, (0.5, 0.5, 0.5))
    res1, res2, res3 = equations((tetha1, tetha2, tetha3))
    return tetha1, tetha2, tetha3

def gripperTake():
    ang1 = 0.5
    ang2 = 0.5
    ang3 = 0.5
    robot.getMotor("finger_1_joint_1").setPosition(ang1)
    robot.getMotor("finger_2_joint_1").setPosition(ang1)
    robot.getMotor("finger_middle_joint_1").setPosition(ang1)
    robot.getMotor("finger_1_joint_2").setPosition(ang2)
    robot.getMotor("finger_2_joint_2").setPosition(ang2)
    robot.getMotor("finger_middle_joint_2").setPosition(ang2)
    robot.getMotor("finger_1_joint_3").setPosition(ang3)
    robot.getMotor("finger_2_joint_3").setPosition(ang3)
    robot.getMotor("finger_middle_joint_3").setPosition(ang3)
    
        
def gripperRelease():
    robot.getMotor("finger_1_joint_1").setPosition(0)
    robot.getMotor("finger_2_joint_1").setPosition(0)
    robot.getMotor("finger_middle_joint_1").setPosition(0)
    robot.getMotor("finger_1_joint_3").setPosition(0)
    robot.getMotor("finger_2_joint_3").setPosition(0)
    robot.getMotor("finger_middle_joint_2").setPosition(0)
    robot.getMotor("finger_1_joint_2").setPosition(0)
    robot.getMotor("finger_2_joint_2").setPosition(0)
    robot.getMotor("finger_middle_joint_3").setPosition(0)
    robot.getMotor("palm_finger_2_joint").setPosition(0)
    robot.getMotor("palm_finger_1_joint").setPosition(0)
    
def updateRobotPosition(x,y,z):
    tetha1,tetha2,tetha3 = getAnglesFormPos(x, y, z)
    motorA.setPosition(tetha1)
    motorB.setPosition(tetha2)
    motorC.setPosition(tetha3)
    motorD.setPosition(pi/2)
    motorE.setPosition(-pi/2)
    motorF.setPosition(pi/2+tetha2+tetha3)
    
def isNearAndBlue(camera):
    img = camera.getImage()
    near = False
    isBlue = False
    color= "None"
    blueSum = 0
    redSum = 0
    greenSum = 0
    for i in range(64*64):
            blueSum += img[4 * i] * img[4 * i + 3] / 255
            greenSum += img[4 * i + 1] * img[4 * i + 3] / 255
            redSum += img[4 * i + 2] * img[4 * i + 3] / 255
    if blueSum > 400000 and redSum < 40000 and greenSum < 40000:
        near = True
        color = "b"
        isBlue = True
    elif redSum > 300000 and blueSum < 40000 and greenSum < 40000:
        near = True
        color = "r"
    elif greenSum > 350000 and blueSum < 40000 and redSum < 40000:
        near = True
        color = "g"
    else:
        near = False
        color = "None"
    return near, isBlue


# initialize devices
motorA = robot.getMotor("A motor")
motorB = robot.getMotor("B motor")
motorC = robot.getMotor("C motor")
motorD = robot.getMotor("D motor")
motorE = robot.getMotor("E motor")
motorF = robot.getMotor("F motor")
motorA.setVelocity(MAX_SPEED);
motorB.setVelocity(MAX_SPEED);
motorC.setVelocity(MAX_SPEED);
motorD.setVelocity(MAX_SPEED);
motorE.setVelocity(MAX_SPEED);
motorF.setVelocity(MAX_SPEED);
camera = robot.getCamera("camera")
camera.enable(1)

x_base = 1200
y_base = -300
z_base = 520
x = x_base
y = y_base
z = z_base
updateRobotPosition(x,y,z)
gripperRelease()


state = 1
TIME_STEP = 100
timeRatio = pi/2/MAX_SPEED
# feedback loop: step simulation until receiving an exit event
while robot.step(100) != -1:

   if state == 1:
        near, isBlue = isNearAndBlue(camera)
        print('near: ', near, ' isBlue: ', isBlue);
        if near:
            if isBlue:
                state = 2
            else:
                state = 15
   elif state == 2:
        gripperTake()
        state = 3
        robot.step(100)
   elif state == 3:
        z = 1000
        updateRobotPosition(x,y,z)
        state = 4
   elif state == 4:
        x = 900 - 200 * numOfObjectsOnTheTable
        y = 1500
        updateRobotPosition(x,y,z)
        state = 5
        robot.step(int(900 * timeRatio))
   elif state == 5:
        z = 540
        updateRobotPosition(x,y,z)
        state = 6
        robot.step(int(200 * timeRatio))
   elif state == 6:
        numOfObjectsOnTheTable += 1
        gripperRelease()
        state = 7
        robot.step(200)
   elif state == 7:
        z = 1000
        updateRobotPosition(x,y,z)
        state = 8
        robot.step(int(200 * timeRatio))
   elif state == 8:
        x = x_base
        y = y_base
        updateRobotPosition(x,y,z)
        state = 9       
   elif state == 9:
        z = z_base
        updateRobotPosition(x,y,z)
        state = 1       
   elif state == 15:
        gripperTake()
        state = 16
        robot.step(100)
   elif state == 16:
        z = 1000
        updateRobotPosition(x,y,z)
        state = 17
        robot.step(int(200 * timeRatio))
   elif state == 17:
        x = 1500
        y = 1500
        z = 750
        updateRobotPosition(x,y,z)
        state = 18
        robot.step(int(450 * timeRatio))
   elif state == 18:
        gripperRelease()
        state = 7
        robot.step(200)
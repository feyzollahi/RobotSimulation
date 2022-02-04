from controller import Robot, DistanceSensor, Motor, Camera
from scipy.optimize import fsolve
import math
import threading
from time import sleep, perf_counter
from matplotlib import pyplot as plt


# create the Robot instance.
robot = Robot()
pi = 3.14159
# from datasheet
A = 300   #mm
D = 1095
F = 1270
# functions:
def IKP(x, y, z):
    if x == 0:
     x = 1
    def equations(p):
        x1, x2, x3 = p
        return (-z + 495 + D * math.cos(x2) + 175 * math.cos(x2+x3) - F * math.sin(x2+x3), -math.sqrt((x-A*math.sin(x1))**2 + (y + A * math.cos(x1))**2) + 175 + D * math.sin(x2) + 175 * math.sin(x2+x3) + F * math.cos(x2+x3), x1 - math.atan2(y,x)-math.atan2(A,math.sqrt((x-A*math.sin(x1))**2 + (y + A * math.cos(x1))**2)))

    tetha1, tetha2, tetha3 = fsolve(equations, (0.5, 0.5, 0.5))
    res1, res2, res3 = equations((tetha1, tetha2, tetha3))
    return tetha1, tetha2, tetha3


def FKP(tetha1, tetha2, tetha3):
    z = 495 + D * math.cos(tetha2) + 175 * math.cos(tetha2+tetha3) - F * math.sin(tetha2+tetha3)
    r = 175 + D * math.sin(tetha2) + 175 * math.sin(tetha2+tetha3) + F * math.cos(tetha2+tetha3)
    x = r * math.cos(tetha1) + A * math.sin(tetha1)
    y = r * math.sin(tetha1) - A * math.cos(tetha1)
    return x,y,z

def gripperTake():
    ang1 = 0.4
    ang2 = 0.5
    ang3 = -0.06
    robot.getDevice("finger_1_joint_1").setPosition(ang1)
    robot.getDevice("finger_2_joint_1").setPosition(ang1)
    robot.getDevice("finger_middle_joint_1").setPosition(ang1)
    robot.getDevice("finger_1_joint_2").setPosition(ang2)
    robot.getDevice("finger_2_joint_2").setPosition(ang2)
    robot.getDevice("finger_middle_joint_2").setPosition(ang2)
    robot.getDevice("finger_1_joint_3").setPosition(ang3)
    robot.getDevice("finger_2_joint_3").setPosition(ang3)
    robot.getDevice("finger_middle_joint_3").setPosition(ang3)
    
        
def gripperRelease():
    ang1 = 0.05
    ang2 = 0.001
    ang3 = -0.06
    robot.getDevice("finger_1_joint_1").setPosition(ang1)
    robot.getDevice("finger_2_joint_1").setPosition(ang1)
    robot.getDevice("finger_middle_joint_1").setPosition(ang1)
    robot.getDevice("finger_1_joint_2").setPosition(ang2)
    robot.getDevice("finger_2_joint_2").setPosition(ang2)
    robot.getDevice("finger_middle_joint_2").setPosition(ang2)
    robot.getDevice("finger_1_joint_3").setPosition(ang3)
    robot.getDevice("finger_2_joint_3").setPosition(ang3)
    robot.getDevice("finger_middle_joint_3").setPosition(ang3)
    robot.getDevice("palm_finger_2_joint").setPosition(0)
    robot.getDevice("palm_finger_1_joint").setPosition(0)
    
tetha1_F,tetha2_F,tetha3_F = [0,0,0]
tetha1_I,tetha2_I,tetha3_I = [0,0,0]
time_I = 0.0
T = 0.8
def robotPositionUpdated(x,y,z,positionChanged):
    global time_I,tetha1_F,tetha2_F,tetha3_F,tetha1_I,tetha2_I,tetha3_I
    if positionChanged:
        time_I = robot.getTime()
        tetha1_F,tetha2_F,tetha3_F = IKP(x, y, z)
        tetha1_I,tetha2_I,tetha3_I = [sensorA.getValue(),sensorB.getValue(),sensorC.getValue()]
        # print(max(abs(tetha1_F-tetha1_I),abs(tetha2_F-tetha2_I),abs(tetha3_F-tetha3_I)))
    thaw = (robot.getTime()-time_I)/T
    if thaw > 1:
        xx,yy,zz = FKP(sensorA.getValue(),sensorB.getValue(),sensorC.getValue())
        print('x err:',xx - x,'y err:',yy-y,'z err:',zz-z)
        return True

    s_t = 6*thaw**5 - 15*thaw**4+10*thaw**3
    tetha1 = tetha1_I + (tetha1_F - tetha1_I) * s_t 
    tetha2 = tetha2_I + (tetha2_F - tetha2_I) * s_t 
    tetha3 = tetha3_I + (tetha3_F - tetha3_I) * s_t 
    motorA.setPosition(tetha1)
    motorB.setPosition(tetha2)
    motorC.setPosition(tetha3)
    motorD.setPosition(pi/2)
    motorE.setPosition(-pi/2)
    motorF.setPosition(pi/2+tetha2+tetha3)
    return False
    
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
motorA = robot.getDevice("A motor")
motorB = robot.getDevice("B motor")
motorC = robot.getDevice("C motor")
motorD = robot.getDevice("D motor")
motorE = robot.getDevice("E motor")
motorF = robot.getDevice("F motor")
motorA.setVelocity(175/180*pi);
motorB.setVelocity(175/180*pi);
motorC.setVelocity(175/180*pi);
motorD.setVelocity(250/180*pi);
motorE.setVelocity(250/180*pi);
motorF.setVelocity(360/180*pi);
camera = robot.getDevice("camera")
camera.enable(1)
x_base = 1200
y_base = -300
z_base = 520
gripperRelease()
sensorA = motorA.getPositionSensor()
sensorB = motorB.getPositionSensor()
sensorC = motorC.getPositionSensor()
sensorF = motorF.getPositionSensor()
sensorA.enable(1)
sensorB.enable(1)
sensorC.enable(1)
sensorF.enable(1)


# test IKP:
testNum = 1
x = -1000
y = 1000
z = 1500
positionUpdated = False
positionChanged = True

while robot.step(10) != -1:
    if not positionUpdated or positionChanged:
        positionUpdated = robotPositionUpdated(x,y,z,positionChanged)
        if positionChanged:
            positionChanged = False
    else:
        if testNum == 1:
            testNum = 2
            x = 0
            y = 1000
            positionChanged = True
        elif testNum == 2:
            testNum = 3
            x = 1000
            y = 1000
            positionChanged = True
        elif testNum == 3:
            testNum = 4
            x = 1000
            y = 0
            positionChanged = True
        elif testNum == 4:
            testNum = 5
            x = 0
            y = 0
            positionChanged = True
        elif testNum == 5:
            testNum = 6
            x = 1000
            y = -1000
            positionChanged = True
        elif testNum == 6:
            testNum = 7
            x = 0
            y = -1000
            positionChanged = True
        elif testNum == 7:
            testNum = 8
            x = -1000
            y = -1000
            positionChanged = True
        elif testNum == 8:
            break
            

x = x_base
y = y_base
z = z_base
state = 1
TIME_STEP = 10
waitingTime = 0
numOfObjectsOnTheTable = 0
positionChanged = True
positionUpdated = False

psensA= []
psensB= []
psensC= []
psensF= []
time = []

while robot.step(TIME_STEP) != -1:
   if len(time) < 500 and robot.getTime() > 19:
        time.append(robot.getTime())
        psensA.append(sensorA.getValue())
        psensB.append(sensorB.getValue())
        psensC.append(sensorC.getValue())
        psensF.append(sensorF.getValue())
        if len(time) == 500:
            fig, axs = plt.subplots(2, 2)
            axs[0, 0].plot(time, psensA)
            axs[0, 0].set_title('Axis 1')
            axs[0, 1].plot(time, psensB, 'tab:orange')
            axs[0, 1].set_title('Axis 2')
            axs[1, 0].plot(time, psensC, 'tab:green')
            axs[1, 0].set_title('Axis 3')
            axs[1, 1].plot(time, psensF, 'tab:red')
            axs[1, 1].set_title('Axis 6')
            plt.show()
                    
   if not positionUpdated or positionChanged:
       positionUpdated = robotPositionUpdated(x,y,z,positionChanged)
       if positionChanged:
           positionChanged = False
           
   elif waitingTime != 0:
       waitingTime -= TIME_STEP
   elif state == 1:
        near, isBlue = isNearAndBlue(camera)
        if near:
            print('near: ', near, ' isBlue: ', isBlue);
            if isBlue:
                state = 2
            else:
                state = 15
   elif state == 2:
        gripperTake()
        state = 3
        waitingTime = 100
   elif state == 3:
        z = 1000
        positionChanged = True
        state = 4
   elif state == 4:
        x = 900 - 200 * numOfObjectsOnTheTable
        y = 1500
        positionChanged = True
        state = 5
   elif state == 5:
        z = 540
        positionChanged = True
        state = 6
   elif state == 6:
        numOfObjectsOnTheTable += 1
        gripperRelease()
        waitingTime = 100
        state = 7
   elif state == 7:
        z = 1000
        positionChanged = True
        state = 8
   elif state == 8:
        x = x_base
        y = y_base
        positionChanged = True
        state = 9       
   elif state == 9:
        z = z_base
        positionChanged = True
        state = 1       
   elif state == 15:
        gripperTake()
        waitingTime = 100
        state = 16
   elif state == 16:
        z = 1000
        positionChanged = True
        state = 17
   elif state == 17:
        x = 1500
        y = 1500
        z = 750
        positionChanged = True
        state = 18
   elif state == 18:
        gripperRelease()
        waitingTime = 100
        state = 7
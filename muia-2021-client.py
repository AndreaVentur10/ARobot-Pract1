#!/usr/bin/python3

# --------------------------------------------------------------------------

print('### Script:', __file__)

# --------------------------------------------------------------------------

import math
import sys
import time

# import cv2 as cv
# import numpy as np
import sim

# --------------------------------------------------------------------------

def getRobotHandles(clientID):
    # Motor handles
    _,lmh = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor',
                                     sim.simx_opmode_blocking)
    _,rmh = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor',
                                     sim.simx_opmode_blocking)

    # Sonar handles
    str = 'Pioneer_p3dx_ultrasonicSensor%d'
    sonar = [0] * 16
    for i in range(16):
        _,h = sim.simxGetObjectHandle(clientID, str % (i+1),
                                       sim.simx_opmode_blocking)
        sonar[i] = h
        sim.simxReadProximitySensor(clientID, h, sim.simx_opmode_streaming)

    # Camera handles
    _,cam = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_camera',
                                        sim.simx_opmode_oneshot_wait)
    sim.simxGetVisionSensorImage(clientID, cam, 0, sim.simx_opmode_streaming)
    sim.simxReadVisionSensor(clientID, cam, sim.simx_opmode_streaming)

    return [lmh, rmh], sonar, cam

# --------------------------------------------------------------------------

def setSpeed(clientID, hRobot, lspeed, rspeed):
    sim.simxSetJointTargetVelocity(clientID, hRobot[0][0], lspeed,
                                    sim.simx_opmode_oneshot)
    sim.simxSetJointTargetVelocity(clientID, hRobot[0][1], rspeed,
                                    sim.simx_opmode_oneshot)

# --------------------------------------------------------------------------

def getSonar(clientID, hRobot):
    r = [1.0] * 16
    for i in range(16):
        handle = hRobot[1][i]
        e,s,p,_,_ = sim.simxReadProximitySensor(clientID, handle,
                                                 sim.simx_opmode_buffer)
        if e == sim.simx_return_ok and s:
            r[i] = math.sqrt(p[0]*p[0]+p[1]*p[1]+p[2]*p[2])

    return r

# --------------------------------------------------------------------------

# def getImage(clientID, hRobot):
#     img = []
#     err,r,i = sim.simxGetVisionSensorImage(clientID, hRobot[2], 0,
#                                             sim.simx_opmode_buffer)

#     if err == sim.simx_return_ok:
#         img = np.array(i, dtype=np.uint8)
#         img.resize([r[1],r[0],3])
#         img = np.flipud(img)
#         img = cv.cvtColor(img, cv.COLOR_RGB2BGR)

#     return err, img

# --------------------------------------------------------------------------

def getImageBlob(clientID, hRobot):
    rc,ds,pk = sim.simxReadVisionSensor(clientID, hRobot[2],
                                         sim.simx_opmode_buffer)
    blobs = 0
    coord = []
    if rc == sim.simx_return_ok and pk[1][0]:
        blobs = int(pk[1][0])
        offset = int(pk[1][1])
        for i in range(blobs):
            coord.append(pk[1][4+offset*i])
            coord.append(pk[1][5+offset*i])

    return blobs, coord

# --------------------------------------------------------------------------

def avoid(sonar, blobs, coord):
    
    lspeed, rspeed = +1.0, +1.0
    auxl, auxr = +1.0, +1.0
    print(coord)
    if blobs == 1:
        if coord[0]<= 0.25:
            rspeed += 1.5
        elif coord[0]>0.25 and coord[0]<= 0.5:
            rspeed += 1.0
        elif coord[0]>0.5 and coord[0]<= 0.75:
            lspeed += 1.0
        else : #coord[1]> 0.5:
            lspeed += 1.5

            
        if coord[1]< 0.15:
            lspeed -= 1.5
            rspeed -= 1.5
        elif coord[1]>=0.15 and coord[1]< 0.25:
            lspeed -= 1.0
            rspeed -= 1.0
        elif coord[1]< 0.5 and coord[1]>= 0.25:
            lspeed += 0.25      ##0.5
            rspeed += 0.25       ##0.5
        elif coord[1]>= 0.5 and coord[1]< 0.7:
            lspeed += 0.7
            rspeed += 0.7 
        else:
            lspeed += 1.5       ##1.1
            rspeed += 1.5       ##1.1
         
        #### obstacles ###############
        print('centrales:', sonar[3], sonar[4])
        print('Lado Izq:', sonar[0], sonar[1], sonar[2])
        print('Lado Der:', sonar[5], sonar[6], sonar[7])
        print('Traseros:', sonar[8], sonar[9],sonar[10], sonar[11],sonar[12], sonar[13],sonar[14], sonar[15])
        
        if sonar[3]< 0.15 or sonar[4]< 0.15:
            lspeed -= 2.0
            rspeed -= 2.0
        if sonar[3] < 0.5 and sonar[4] < 0.5 and coord[0]< 0.25:
            lspeed -= 1.0
            rspeed -= 1.0
        if sonar[3] < 0.5 and sonar[4] < 0.5:
            lspeed -= 0.5
            rspeed -= 0.5
        elif sonar[0]<0.7 and sonar[1] < 0.7:
            lspeed += 2.0
        elif sonar[6]<0.7 and sonar[7] < 0.7:
            rspeed += 2.0
        elif sonar[2]<0.7 and sonar[3] < 0.7:
            lspeed += 1.5    
        elif sonar[4]<0.7 and sonar[5] < 0.7:
            rspeed += 1.5   
            
    else: #blobs != 1
        lspeed, rspeed = +0.0, +2.0
        
        
    #if (sonar[3] < 0.1) or (sonar[4] < 0.1):
        #lspeed, rspeed = +0.3, -0.5
    #elif sonar[1] < 0.3:
        #lspeed, rspeed = +1.0, +0.3
    #elif sonar[5] < 0.2:
        #lspeed, rspeed = +0.2, +0.7
    #else:
        #lspeed, rspeed = +2.0, +2.0
    #print(sonar)
    
    #lspeed, rspeed = +2.0, +2.0
        
    print("Ahora hace: ")    
    print(lspeed,rspeed)
    return lspeed, rspeed

# --------------------------------------------------------------------------

def main():
    print('### Program started')

    print('### Number of arguments:', len(sys.argv), 'arguments.')
    print('### Argument List:', str(sys.argv))

    sim.simxFinish(-1) # just in case, close all opened connections

    port = int(sys.argv[1])
    clientID = sim.simxStart('127.0.0.1', port, True, True, 2000, 5)

    if clientID == -1:
        print('### Failed connecting to remote API server')

    else:
        print('### Connected to remote API server')
        hRobot = getRobotHandles(clientID)

        while sim.simxGetConnectionId(clientID) != -1:
            # Perception
            sonar = getSonar(clientID, hRobot)
            print('getSonar')
            # print '### s', sonar
           # print(hRobot)

            blobs, coord = getImageBlob(clientID, hRobot)
            print('###  ', blobs, coord)

            # Planning
            lspeed, rspeed = avoid(sonar, blobs, coord)

            # Action
            setSpeed(clientID, hRobot, lspeed, rspeed)
            time.sleep(0.1)

        print('### Finishing...')
        sim.simxFinish(clientID)

    print('### Program ended')

# --------------------------------------------------------------------------

if __name__ == '__main__':
    main()

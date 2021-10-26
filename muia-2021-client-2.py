#!/usr/bin/python3

# --------------------------------------------------------------------------

print('### Script:', __file__)

# --------------------------------------------------------------------------

import math
import sys
import time

# import cv2 as cv
#import numpy as np
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
            coord.append(pk[1][6+offset*i])

    return blobs, coord

# --------------------------------------------------------------------------

def avoid(sonar, blobs, coord):

    lspeed, rspeed = +0.7, +0.7
    auxl, auxr = +1.0, +1.0
    print(coord)
    if blobs == 1:
        if coord[0]<= 0.25:
            rspeed += 1.2       #1.5
            print("1")
        elif coord[0]>0.25 and coord[0]<= 0.5:
            rspeed += 0.7       #1.0
            print("12")
        elif coord[0]>0.5 and coord[0]<= 0.75:
            lspeed += 0.7       #1.0
            print("13")
        else : #coord[1]> 0.5:
            lspeed += 1.2
            print("14")

        if coord[1]< 0.15:
            #lspeed -= 1.2
            lspeed = lspeed/3
            #rspeed -= 1.2
            rspeed = rspeed/3
            print("25")
        elif coord[1]>=0.15 and coord[1]< 0.25:
            #lspeed -= 1.0
            lspeed = lspeed/2
            #rspeed -= 1.0
            rspeed = rspeed/2
            print("26")
        elif coord[1]< 0.5 and coord[1]>= 0.25:
            lspeed += 0.25      ##0.5
            rspeed += 0.25       ##0.5
            print("27")
        elif coord[1]>= 0.5 and coord[1]< 0.8:
            lspeed += 0.5  # 0.7
            rspeed += 0.5
            print("28")
        else:
            lspeed += 0.8       ##1.1
            rspeed += 0.8       ##1.1
            print("29")

        #### obstacles ###############
        print('ImageBlob:', coord[0],coord[1])
        print('centrales:', sonar[3], sonar[4])
        print('Lado Izq:', sonar[0], sonar[1], sonar[2])
        print('Lado Der:', sonar[5], sonar[6], sonar[7])
        print('Traseros:', sonar[8], sonar[9],sonar[10], sonar[11],sonar[12], sonar[13],sonar[14], sonar[15])

        if sonar[3]< 0.18 or sonar[4]< 0.18: ## ~0.15  ###AQUIII HAYYYY MOVIDAAAAAA
            #lspeed -= 3.0 ##2.0
            #rspeed -= 3.0
            if  coord[0] < 0.4:
                lspeed = -0.2
                rspeed = 1.2
                print("30")
            elif coord[0] > 0.6:
                lspeed = 1.2
                rspeed = -0.2
                print("31")
            else:
                lspeed = 0.0
                rspeed = 0.0
                print("32")
        elif sonar[3] <0.25 and sonar[4] < 0.25 and coord[1]< 0.25:
            lspeed -= 1.4 ##2.0
            rspeed -= 1.4
            print("43")
        elif sonar[3] < 0.5 and sonar[4] < 0.5 and coord[1]< 0.35: ##0.25
            lspeed -= 1.2 ##1.5
            rspeed -= 1.2
            print("44")
        elif sonar[3] < 0.5 and sonar[4] < 0.5:
            lspeed -= 0.5
            rspeed -= 0.5
            print("45")
        elif sonar[0]<0.7 and sonar[1] < 0.7:
            lspeed += 1.4   ##1.5
            print("46")
        elif sonar[6]<0.7 and sonar[7] < 0.7:
            rspeed += 1.4   ##1.5
            print("47")
        elif sonar[2]<0.7 and sonar[3] < 0.7:
            lspeed += 1.4   ##1.5
            print("48")
        elif sonar[4]<0.7 and sonar[5] < 0.7:
            rspeed += 1.4   ##1.5
            print("49")

    else: #blobs != 1
        lspeed, rspeed = +1.4, +0.0 #+0.0, +2.0
        print("60")

        #### obstacles ###############
        print('centrales:', sonar[3], sonar[4])
        print('Lado Izq:', sonar[0], sonar[1], sonar[2])
        print('Lado Der:', sonar[5], sonar[6], sonar[7])
        print('Traseros:', sonar[8], sonar[9],sonar[10], sonar[11],sonar[12], sonar[13],sonar[14], sonar[15])


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
def centerBall(coord):
    xtarget = 0.5                   # desirable speed
    Kp = 3                         # constant 3
    error = xtarget - coord[0]      # positive is on the left, negative is on the right
    Px = Kp * error

    return Px

# --------------------------------------------------------------------------
def fixDistance(coord):
    dist_target = 0.6              # less is more 0.6 0.65
    Kp = 3 #5 #4
    dist_current = coord[2]         #( sonar[3] + sonar[4] )/ 2
    #print('ESTA ES LA Z:', coord[2])
    error = dist_target - dist_current
    Pdepth = Kp * error

    return Pdepth
# --------------------------------------------------------------------------
def avoidCollisionFront(sonar):
    #print('CENTRAL [3]    : {}      CENTRAL [4]    : {}'.format(sonar[3], sonar[4]))
    f_right = sonar[4]
    f_left = sonar[3]
    dist_target = 0.5
    Kp = 1.2 #1.5 1.4
    Signal = 'OK'
    error = 0.0

    if f_right < 0.3 or f_left < 0.3: #0.3
        if f_right < f_left:
            error = dist_target - f_right
            Signal = 'CD'
        else:
            error = dist_target - f_left
            Signal = 'CI'
        return Signal, error * Kp
    else:
        return Signal, error

# --------------------------------------------------------------------------
def avoidCollisionLat(sonar):
    #print('IZQ [1]    : {}   '.format(sonar[1]))
    #print('IZQ [2]    : {}   '.format(sonar[2]))
    #print('IZQ [0]    : {}   '.format(sonar[0]))

    #print('____________________________________________________________')

   # right = sonar[6]            #sonar[4] + sonar[5] + sonar[7]
   # left = sonar[1]             #sonar[3] + sonar[2] + sonar[0]

    # sep_target = 0.5            #separation desired
    #---------------------------------------------------------------
    right = sonar[6]   # (sonar[6] + sonar[7])/2
    left = sonar[1]   # (sonar[1] + sonar[0])/2

    sep_target = right + left / 2
    #---------------------------------------------------------------

    Kp = 1.2 # 1
    Signal = 'OK'
    if left < 0.24 or right< 0.24:  #0.24
        print('IZQ [1]    : {}      DER [6]    : {}'.format(sonar[1], sonar[6]))

        if right < left:
            Signal = 'D'
            error = sep_target - right
        else:
            Signal = 'I'
            error = sep_target - left
    else:
        error = 0.0

    Psep = Kp * error

    return Signal, Psep
# --------------------------------------------------------------------------
def avoidCollisionLatNoBall(sonar):

    right = sonar[6] + sonar[7]          #sonar[4] + sonar[5]
    left = sonar[1] + sonar[0]          #sonar[3] + sonar[2]

    sep_target = right + left / 4

    Kp = 1.0 # 1
    Signal = 'OK'

    if right < left:

        Signal = 'D'
        error = sep_target - right
    else:
        Signal = 'I'
        error = sep_target - left

    Psep = Kp * error

    return Signal, Psep
# --------------------------------------------------------------------------
def avoidCollisionFrontNoBall(sonar):
    #print('CENTRAL [3]    : {}      CENTRAL [4]    : {}'.format(sonar[3], sonar[4]))
    print('CENTRAL [8]    : {}      CENTRAL [15]    : {}'.format(sonar[8], sonar[15]))
    f_right = sonar[4] + sonar[5]
    f_left = sonar[3] + sonar[2]
    back_free = sonar[12] ==1.0 and sonar[11] == 1.0
    back_sides = sonar[8] < 0.5 and sonar[15] <0.5
    dist_target = 0.5
    Kp = 1.2
    Signal = 'OK'
    error = 0.0
    """if sonar[4] < 0.3 or sonar[3] < 0.3 and sonar[7] <0.40 and sonar[0]< 0.40 and back_free:
        Signal = 'BA'
        print('BACKWARDS')
        return Signal, error"""

    if f_right < 0.6 or f_left < 0.6: #0.3
        if f_right < f_left:
            error = dist_target - f_right
            Signal = 'CD'
        else:
            error = dist_target - f_left
            Signal = 'CI'
        return Signal, error * Kp

    else:
        return Signal, error

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
            #print('getSonar')

            blobs, coord = getImageBlob(clientID, hRobot)
            #print('###  ', blobs, coord)
            #print(coord)

            # Planning
            vel_base = 0.7 #1.4 0.85

            """# Sin bola --------------------------------------------
            lspeed, rspeed = 0.0, 0.0
            Psep, W = avoidCollisionLat(sonar)
            if W == 'I':
                lspeed, rspeed = vel_base - Psep, vel_base + Psep
            else:
                lspeed, rspeed = vel_base + Psep, vel_base - Psep
            # Sin bola --------------------------------------------"""


            if blobs == 1:
                Px = centerBall(coord)
                Pdepth = fixDistance(coord)
                lspeed, rspeed = vel_base - Px + Pdepth,  vel_base + Px + Pdepth
                sep_max = 0.55
                W, PsepF  = avoidCollisionFront(sonar)
                WL, PsepL = avoidCollisionLat(sonar)
                #vel_base = 1
                if W != 'OK':

                    if W == 'CD':
                        lspeed, rspeed = vel_base + Px + Pdepth + PsepF, vel_base + Px + Pdepth - PsepF#vel_base + Px + Pdepth - PsepF #
                        print('01')
                    elif W == 'CI':
                        lspeed, rspeed = vel_base - Px + Pdepth - PsepF, vel_base - Px + Pdepth + PsepF# #
                        print('09')
                    #print('esto que es {} {}'.format(lspeed, rspeed))
                    print('CHOQUE FRONTAL 1')

                else:
                    if WL != 'OK':  # si hay choque lateral
                        if WL == 'D':
                            lspeed, rspeed = vel_base + Px + Pdepth - PsepL, vel_base + Px + Pdepth + PsepL  # vel_base + Px + Pdepth + Psep
                        elif WL == 'I':
                            lspeed, rspeed = vel_base - Px + Pdepth + PsepL, vel_base + Px + Pdepth - PsepL  # vel_base - Px + Pdepth + Psep
                        print('CHOQUE LATERAL 1')

            else:
                W, PsepF = avoidCollisionFrontNoBall(sonar)
                WL, PsepL = avoidCollisionLatNoBall(sonar)
                lspeed, rspeed = +1.4, -1.4
                vel_base = 1.0 #1.0

                if W != 'OK':

                    if W == 'CD':
                        lspeed, rspeed = vel_base - PsepF, vel_base + PsepF  # vel_base + Px + Pdepth - PsepF #
                    elif W == 'CI':
                        lspeed, rspeed = vel_base + PsepF, vel_base - PsepF  # #
                    """elif W =='BA':
                        lspeed, rspeed = -1.4, +0.0"""
                    # print('esto que es {} {}'.format(lspeed, rspeed))
                    print('CHOQUE FRONTAL 2')

                else:
                    if WL != 'OK':  # si hay choque lateral
                        if WL == 'D':
                            lspeed, rspeed = vel_base - PsepL, vel_base + PsepL  # vel_base + Px + Pdepth + Psep
                        elif WL == 'I':
                            lspeed, rspeed = vel_base + PsepL, vel_base - PsepL  # vel_base - Px + Pdepth + Psep
                        print('CHOQUE LATERAL 2 ')


            # Action
            setSpeed(clientID, hRobot, lspeed, rspeed)
            time.sleep(0.1)

        print('### Finishing...')
        sim.simxFinish(clientID)

    print('### Program ended')

# --------------------------------------------------------------------------

if __name__ == '__main__':
    main()

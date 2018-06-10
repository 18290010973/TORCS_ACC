'''
Main scripts which calls the controller and the TorcsEnv scripts
Invokes the drive_example function which has both the leader car and the follower car functionalities
Includes random generation of leader car speeds and its position on the track
'''

import Controller
import TorcsEnv
import random #for generating the random traget speed and position 
import csv
import numpy as np
from keras.models import model_from_json 

#Variables for the random behavior of the leader car
accumulateSpeedTime = 0.0
accumulateStrTime =0.0
targetSpeed = 40.0
targetStrE = 0.0
timeIntervalSpeed = 10.0
timeIntervalStr = 10.0
trackPoslast = 0.0

Vl = 0.0 # leader car velocity 

#csvfile = open("fulldata2.csv", "w")

def drive_example(c, num):
    global accumulateSpeedTime, accumulateStrTime, targetSpeed, targetStrE, timeIntervalSpeed, timeIntervalStr
    global Vl
    global trackPoslast
    S, R = c.S.d, c.R.d

    if (num==0):
        # The first car controller
        # random the target speed and the target position of the first car
        if (S['curLapTime'] < .20):
            accumulateSpeedTime = 0.0
            accumulateStrTime = 0.0
            
            #Random generation of the leader car speed 
            targetSpeed = random.gauss(70.0, 30.0)
            speedMin = 0.0
            speedMax = 70.0
            targetSpeed = min(speedMax, targetSpeed)
            targetSpeed = max(speedMin, targetSpeed)

            #Random generation of the leader car track position 
            targetStrE = random.gauss(0.0, 0.5)
            strMin = -0.7
            strMax = 0.7
            targetStrE = min(strMax, targetStrE)
            targetStrE = max(strMin, targetStrE)

            #Random time interval generator within the current lap
            timeIntervalSpeed = float(random.randint(10, 25))
            timeIntervalStr = float(random.randint(10, 25))
            #print('New Lap is Starting....', temp_speed, temp_strE)
          
        #Random generation of the leader car speed  
        if (S['curLapTime'] > accumulateSpeedTime + timeIntervalSpeed):
            accumulateSpeedTime += timeIntervalSpeed
            timeIntervalSpeed = float(random.randint(10, 25))
            targetSpeed = random.gauss(70.0, 30.0)
            speedMin = 0.0
            speedMax = 70.0
            targetSpeed = min(speedMax, targetSpeed)
            targetSpeed = max(speedMin, targetSpeed)
            #print('update the target speed to', temp_speed)
      
        #Random generation of the leader car track position
        if (S['curLapTime'] > accumulateStrTime + timeIntervalStr):
            accumulateStrTime += timeIntervalStr
            timeIntervalStr = float(random.randint(10, 25))
            targetStrE = random.gauss(0.0, 0.5)
            strMin = -0.9
            strMax = 0.9
            targetStrE = min(strMax, targetStrE)
            targetStrE = max(strMin, targetStrE)
 
        #Acctuators of the first car
        R['steer'] = Controller.steeringControl(S, targetStrE)
        R['accel'] = Controller.speedControl(S, R, targetSpeed)
        R['gear'] = Controller.automaticGear(S)
        
        Vl = S['speedX'] # transfer the leader car speed to the second car controller
        
    if (num==1):
        # The second car controller 
        #R['steer'] = Controller.ACCSteeringController(S)
        [acc, brake, Xr] = Controller.ACCVelocityController(Vl, S)
        #R['accel'] = acc
        #R['brake'] = brake
        

        # nn controller
        trackPos = Controller.trackPosCalc(S['track'])
        distance = min(S['opponents'])
        row = [S['speedX'], S['speedY'], trackPos, trackPoslast, distance, Vl, Xr]
        row = (row - dataNormalization['X_mean'])/dataNormalization['X_std']
        trackPoslast = trackPos
        Y = Controller.nncontroller(row, model)
        #Y = Y * dataNormalization['Y_std']
        R['steer'] = Y[0]
        R['accel'] = Y[1]
        R['brake'] = Y[2]
        R['gear'] = Controller.automaticGear(S)
        #collectData(S, R, Xr, Vl)# Data collection in csv
        #x=min(S['opponents'])
        #print(x)

def collectData(S, R, Xr, Vl):
        #Data being collected in CSV
        car1 = []
        car1.append(Xr)
        car1.append(Vl)
        car1.append(S['speedX'])
        car1.append(S['speedY'])
        car1.append(S['speedZ'])
        car1.append(S['opponents'])
        car1.append(S['track'])
        car1.append(S['trackPos'])
        car1.append(S['wheelSpinVel'])
        car1.append(S['angle'])
        car1.append(S['gear'])
        car1.append(S['focus'])
        car1.append(S['distRaced'])
        car1.append(S['distFromStart'])
        car1.append(S['rpm'])
        car1.append(S['damage'])
        car1.append(S['z'])
        car1.append(R['steer'])
        car1.append(R['accel'])
        car1.append(R['brake'])

        writer = csv.writer(csvfile)
        writer.writerow(car1)
        

if __name__ == "__main__":
    #Loading the nn model
    with open('model.json', 'r') as jfile:
        model = model_from_json(jfile.read())
    model.load_weights('weights.best.hdf5')

    #Loading the normalized parameters
    dataNormalization = np.load('normalizeParameters.npz')
 
    #Adding different clients to the simulation
    CS=[TorcsEnv.Client(p=P) for P in [3001, 3002]]
    for step in range(CS[0].maxSteps, 0, -1):#The simulation steps
        num = 0
        for C in CS:
            C.get_servers_input()#Get sensor values from the simulator 
            drive_example(C, num)#Invoke the python client for the car controller
            C.respond_to_server()#Send the actuator control signals to the simulator
            num+=1
    C.shutdown()

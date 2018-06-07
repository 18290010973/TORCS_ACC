import Controller
import TorcsEnv

import random # this is for generating the random traget speed and position
import csv


accumulateSpeedTime = 0.0
accumulateStrTime =0.0
targetSpeed = 40.0
targetStrE = 0.0
timeIntervalSpeed = 10.0
timeIntervalStr = 10.0

Vl = 0.0 # leader car velocity 

csvfile = open("fulldata1.csv", "a")

def drive_example(c, num):
    global accumulateSpeedTime, accumulateStrTime, targetSpeed, targetStrE, timeIntervalSpeed, timeIntervalStr
    global Vl
    S, R = c.S.d, c.R.d

    if (num==0):
        # this is the first car controller

        # random the target speed and the target position of the first car
        if (S['curLapTime'] < .20):
            accumulateSpeedTime = 0.0
            accumulateStrTime = 0.0
            targetSpeed = random.gauss(70.0, 30.0)

            speedMin = 0.0
            speedMax = 70.0
            targetSpeed = min(speedMax, targetSpeed)
            targetSpeed = max(speedMin, targetSpeed)

            targetStrE = random.gauss(0.0, 0.5)
            strMin = -0.7
            strMax = 0.7
            targetStrE = min(strMax, targetStrE)
            targetStrE = max(strMin, targetStrE)

            timeIntervalSpeed = float(random.randint(10, 25))
            timeIntervalStr = float(random.randint(10, 25))
            #print('New Lap is Starting....', temp_speed, temp_strE)
            
        if (S['curLapTime'] > accumulateSpeedTime + timeIntervalSpeed):
            accumulateSpeedTime += timeIntervalSpeed
            timeIntervalSpeed = float(random.randint(10, 25))
            targetSpeed = random.gauss(70.0, 30.0)
            speedMin = 0.0
            speedMax = 70.0
            targetSpeed = min(speedMax, targetSpeed)
            targetSpeed = max(speedMin, targetSpeed)
            #print('update the target speed to', temp_speed)

        if (S['curLapTime'] > accumulateStrTime + timeIntervalStr):
            accumulateStrTime += timeIntervalStr
            timeIntervalStr = float(random.randint(10, 25))
            targetStrE = random.gauss(0.0, 0.5)
            strMin = -0.9
            strMax = 0.9
            targetStrE = min(strMax, targetStrE)
            targetStrE = max(strMin, targetStrE) 
    
        R['steer'] = Controller.steeringControl(S, targetStrE)
        R['accel'] = Controller.speedControl(S, R, targetSpeed)
        R['gear'] = Controller.automaticGear(S)
        
        Vl = S['speedX'] # transfer the leader car speed to the second car controller
        
    if (num==1):
        # this is the second car controller 
        R['steer'] = Controller.ACCSteeringController(S)
        [acc, brake] = Controller.ACCVelocityController(Vl, S)
        R['accel'] = acc
        R['brake'] = brake
        R['gear'] = Controller.automaticGear(S)
        collectData(S, R)
    

def collectData(S, R):
        car1 = []
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
    CS=[TorcsEnv.Client(p=P) for P in [3001, 3002]]
    for step in range(CS[0].maxSteps, 0, -1):
        num = 0
        for C in CS:
            C.get_servers_input()
            drive_example(C, num)
            C.respond_to_server()
            num+=1
    C.shutdown()

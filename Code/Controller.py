import math

PI = 3.14159265359
k_siP = 0.4
k_siD = 5.0
k_sd = 0.0001
M_c2 = 0.0001

Xr = 0.0
errorL = 0.0

# steering control from snakeoil
def steeringControl(S, targetPos):
    steering = S['angle']*10 / PI
    steering -= (S['trackPos']-targetPos) *.10
    return steering

# speed control from snakeoil
def speedControl(S, R, targetSpeed):
    if S['speedX'] < targetSpeed - (R['steer']*50):
        acc = R['accel'] + .01# Add the accelarction code here
    else:
        acc = R['accel'] - .01 #Brake code goes here
    if S['speedX']<10:
        acc = R['accel'] + 1/(S['speedX']+.1)
    return acc


def ACCVelocityController(Vl, S):
    global Xr
    distance = min(S['opponents'])
    V = S['speedX']
    Xmin = 20.0
    Vd = Vl + (distance - Xmin - 0.5*V*1000/3600)*0.3*3600.0/1000.0
    Vr = Vd - V
    Xr += Vr*1000/3600*0.02
    action = 0.04*Xr + 0.04*Vr
    #print("rawAction:", action, "Xr:", Xr, "Vd:", Vd, "Distance", distance)
    actionMin = -1.0
    actionMax = 1.0
    action = max(actionMin, action)
    action = min(actionMax, action)
    if (action >= 0.0):
        return [action, 0.0]
    else:
        return [0.0, -1.0*action] 


#Function which computes the steering value for the second car controller
def ACCSteeringController(S):
    global M_c2, errorL
    trackPos = trackPosCalc(S['track'])
    vx = S['speedX']
    vy = S['speedY']
    if (trackPos >=0):
        sign = -1.0
    else:
        sign = 1.0
    steering = sign * (math.fabs(k_siP*(trackPos)+k_siD*(trackPos-errorL)) -k_sd*math.fabs(vy) - M_c2*vx)
    errorL = trackPos
    if (steering > 1):
       steering = 1.0
    elif (steering < -1):
       steering = -1.0
    return steering

def trackPosCalc(trackLidar):
    edges=trackLidar
    rightc=edges[15]
    rightp=edges[18]

    #Calculating the Lateral_Distance from the sidewall, using the LIDAR2 (track) values
    rightalpha=math.atan((1.0*rightc*math.cos(30.0*PI/180) - rightp)/(1.0*rightc*math.sin(30.0*PI/180)))
    rightd=rightp * math.cos(rightalpha)
    trackPos = (rightd - 5.0)/5.0

    return trackPos

def automaticGear(S):
    gear=1
    if S['speedX']>50:
        gear=2
    if S['speedX']>80:
        gear=3
    if S['speedX']>110:
        gear=4
    if S['speedX']>140:
        gear=5
    if S['speedX']>170:
        gear=6
    return gear

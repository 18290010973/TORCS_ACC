#!/usr/bin/python
#Client side python script for both leader car and the follower car
#The number of cars that can be added into the simulation can be varied. For now we have 2 cars.
#The leader car has a simple controller which takes in sensor values (TrackPos, angle and SpeedX) and produces control of (steering, acceleration)
#The follower controller uses Siyuan controller which takes in sensor values (LIDAr1, LIDAR2, SpeedX and SpeedY) and produces control of (Steering, acceleration and brakes)
import csv
import socket
import sys
import getopt
import os
import time
from datetime import datetime
from influxdb.client import InfluxDBClientError
from influxdb import client as influxdb
import time
import datetime
import math
from threading import Thread
from time import sleep
from threading import Condition

condition = Condition()

PI= 3.14159265359
lasterror=0
lasterror2=0
car0=[0.0,0.0,0.0,0.0,0.0,0.0]
car1=[0.0,0.0,0.0,0.0,0.0,0.0,0.0]

#Controller constants to be tuned
k_ti = 0.04 #0.1
k_td = 0.001#0.001
M_c1 = -0.1 #-0.05
M_c2 = 0.0001 #0.0001#-0.0001
k_bi = 0.05#0.1
k_bd = 0.001 # 0.001
k_si = 0.08#0.04
k_sd = 0.0001 #0.0001
d_d = 10.0
d_ld = 5
v_d = 70
errorL = 0
data_size = 2**17
buffer1=[]
buffer2=[]
flag=0
csvfile = open("dataset2.csv", "a")

# Initialize help messages
ophelp=  'Options:\n'
ophelp+= ' --host, -H <host>    TORCS server host. [localhost]\n'
ophelp+= ' --port, -p <port>    TORCS port. [3001]\n'
ophelp+= ' --id, -i <id>        ID for server. [SCR]\n'
ophelp+= ' --steps, -m <#>      Maximum simulation steps. 1 sec ~ 50 steps. [100000]\n'
ophelp+= ' --episodes, -e <#>   Maximum learning episodes. [1]\n'
ophelp+= ' --track, -t <track>  Your name for this track. Used for learning. [unknown]\n'
ophelp+= ' --stage, -s <#>      0=warm up, 1=qualifying, 2=race, 3=unknown. [3]\n'
ophelp+= ' --debug, -d          Output full telemetry.\n'
ophelp+= ' --help, -h           Show this help.\n'
ophelp+= ' --version, -v        Show current version.'
usage= 'Usage: %s [ophelp [optargs]] \n' % sys.argv[0]
usage= usage + ophelp
version= "20130505-2"

def clip(v,lo,hi):
    if v<lo: return lo
    elif v>hi: return hi
    else: return v

def bargraph(x,mn,mx,w,c='X'):
    '''Draws a simple asciiart bar graph. Very handy for
    visualizing what's going on with the data.
    x= Value from sensor, mn= minimum plottable value,
    mx= maximum plottable value, w= width of plot in chars,
    c= the character to plot with.'''
    if not w: return '' # No width!
    if x<mn: x= mn      # Clip to bounds.
    if x>mx: x= mx      # Clip to bounds.
    tx= mx-mn # Total real units possible to show on graph.
    if tx<=0: return 'backwards' # Stupid bounds.
    upw= tx/float(w) # X Units per output char width.
    if upw<=0: return 'what?' # Don't let this happen.
    negpu, pospu, negnonpu, posnonpu= 0,0,0,0
    if mn < 0: # Then there is a negative part to graph.
        if x < 0: # And the plot is on the negative side.
            negpu= -x + min(0,mx)
            negnonpu= -mn + x
        else: # Plot is on pos. Neg side is empty.
            negnonpu= -mn + min(0,mx) # But still show some empty neg.
    if mx > 0: # There is a positive part to the graph
        if x > 0: # And the plot is on the positive side.
            pospu= x - max(0,mn)
            posnonpu= mx - x
        else: # Plot is on neg. Pos side is empty.
            posnonpu= mx - max(0,mn) # But still show some empty pos.
    nnc= int(negnonpu/upw)*'-'
    npc= int(negpu/upw)*c
    ppc= int(pospu/upw)*c
    pnc= int(posnonpu/upw)*'_'
    return '[%s]' % (nnc+npc+ppc+pnc)

class Client():
    def __init__(self,H=None,p=None,i=None,e=None,t=None,s=None,d=None,vision=False):
        # If you don't like the option defaults,  change them here.
        self.vision = vision

        self.host= 'localhost'
        self.port= 3001
        self.sid= 'SCR'
        self.maxEpisodes=1 # "Maximum number of learning episodes to perform"
        self.trackname= 'unknown'
        self.stage= 3 # 0=Warm-up, 1=Qualifying 2=Race, 3=unknown <Default=3>
        self.debug= False
        self.maxSteps= 200000  # 50steps/second
        self.parse_the_command_line()
        if H: self.host= H
        if p: self.port= p
        if i: self.sid= i
        if e: self.maxEpisodes= e
        if t: self.trackname= t
        if s: self.stage= s
        if d: self.debug= d
        self.S= ServerState()
        self.R= DriverAction()
        self.setup_connection()

    def setup_connection(self):
        # == Set Up UDP Socket ==
        try:
            self.so= socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        except socket.error as emsg:
            print('Error: Could not create socket...')
            sys.exit(-1)
        # == Initialize Connection To Server ==
        self.so.settimeout(1)

        n_fail = 5
        while True:
            a= "-45 -19 -12 -7 -4 -2.5 -1.7 -1 -.5 0 .5 1 1.7 2.5 4 7 12 19 45"
            initmsg='%s(init %s)' % (self.sid,a)

            try:
                self.so.sendto(initmsg.encode(), (self.host, self.port))
            except socket.error as emsg:
                sys.exit(-1)
            sockdata= str()
            try:
                sockdata,addr= self.so.recvfrom(data_size)
                sockdata = sockdata.decode('utf-8')
            except socket.error as emsg:
                print("Waiting for server on %d............" % self.port)
                print("Count Down : " + str(n_fail))
                if n_fail < 0:
                    print("relaunch torcs")
                    os.system('pkill torcs')
                    time.sleep(1.0)
                    if self.vision is False:
                        os.system('torcs -nofuel -nodamage -nolaptime &')
                    else:
                        os.system('torcs -nofuel -nodamage -nolaptime -vision &')

                    time.sleep(1.0)
                    os.system('sh autostart.sh')
                    n_fail = 5
                n_fail -= 1

            identify = '***identified***'
            if identify in sockdata:
                print("Client connected on %d.............." % self.port)
                break

#Function to parse the command line arguments
    def parse_the_command_line(self):
        try:
            (opts, args) = getopt.getopt(sys.argv[1:], 'H:p:i:m:e:t:s:dhv',
                       ['host=','port=','id=','steps=',
                        'episodes=','track=','stage=',
                        'debug','help','version'])
        except getopt.error as why:
            print('getopt error: %s\n%s' % (why, usage))
            sys.exit(-1)
        try:
            for opt in opts:
                if opt[0] == '-h' or opt[0] == '--help':
                    print(usage)
                    sys.exit(0)
                if opt[0] == '-d' or opt[0] == '--debug':
                    self.debug= True
                if opt[0] == '-H' or opt[0] == '--host':
                    self.host= opt[1]
                if opt[0] == '-i' or opt[0] == '--id':
                    self.sid= opt[1]
                if opt[0] == '-t' or opt[0] == '--track':
                    self.trackname= opt[1]
                if opt[0] == '-s' or opt[0] == '--stage':
                    self.stage= int(opt[1])
                if opt[0] == '-p' or opt[0] == '--port':
                    self.port= int(opt[1])
                if opt[0] == '-e' or opt[0] == '--episodes':
                    self.maxEpisodes= int(opt[1])
                if opt[0] == '-m' or opt[0] == '--steps':
                    self.maxSteps= int(opt[1])
                if opt[0] == '-v' or opt[0] == '--version':
                    print('%s %s' % (sys.argv[0], version))
                    sys.exit(0)
        except ValueError as why:
            print('Bad parameter \'%s\' for option %s: %s\n%s' % (
                                       opt[1], opt[0], why, usage))
            sys.exit(-1)
        if len(args) > 0:
            print('Superflous input? %s\n%s' % (', '.join(args), usage))
            sys.exit(-1)

#Function to get data from server
    def get_servers_input(self):
        '''Server's input is stored in a ServerState object'''
        if not self.so: return
        sockdata= str()

        while True:
            try:
                # Receive server data
                sockdata,addr= self.so.recvfrom(data_size)
                sockdata = sockdata.decode('utf-8')
            except socket.error as emsg:
                print('.', end=' ')
                #print "Waiting for data on %d.............." % self.port
            if '***identified***' in sockdata:
                print("Client connected on %d.............." % self.port)
                continue
            elif '***shutdown***' in sockdata:
                print((("Server has stopped the race on %d. "+
                        "You were in %d place.") %
                        (self.port,self.S.d['racePos'])))
                self.shutdown()
                return
            elif '***restart***' in sockdata:
                # What do I do here?
                print("Server has restarted the race on %d." % self.port)
                # I haven't actually caught the server doing this.
                self.shutdown()
                return
            elif not sockdata: # Empty?
                continue       # Try again.
            else:
                self.S.parse_server_str(sockdata)
                if self.debug:
                    sys.stderr.write("\x1b[2J\x1b[H") # Clear for steady output.
                    print(self.S)
                break # Can now return from this function.


    def respond_to_server(self):
        if not self.so: return
        try:
            message = repr(self.R)
            self.so.sendto(message.encode(), (self.host, self.port))
        except socket.error as emsg:
            print("Error sending to server: %s Message %s" % (emsg[1],str(emsg[0])))
            sys.exit(-1)
        if self.debug: print(self.R.fancyout())

    def shutdown(self):
        if not self.so: return
        print(("Race terminated or %d steps elapsed. Shutting down %d."
               % (self.maxSteps,self.port)))
        self.so.close()
        self.so = None
        #sys.exit() # No need for this really.

class ServerState():
    '''What the server is reporting right now.'''
    def __init__(self):
        self.servstr= str()
        self.d= dict()

    def parse_server_str(self, server_string):
        '''Parse the server string.'''
        self.servstr= server_string.strip()[:-1]
        sslisted= self.servstr.strip().lstrip('(').rstrip(')').split(')(')
        for i in sslisted:
            w= i.split(' ')
            self.d[w[0]]= destringify(w[1:])

    def __repr__(self):
        # Comment the next line for raw output:
        return self.fancyout()
        # -------------------------------------
        out= str()
        for k in sorted(self.d):
            strout= str(self.d[k])
            if type(self.d[k]) is list:
                strlist= [str(i) for i in self.d[k]]
                strout= ', '.join(strlist)
            out+= "%s: %s\n" % (k,strout)
        return out

#Different sensors to be added onto our car
    def fancyout(self):
        '''Specialty output for useful ServerState monitoring.'''
        out= str()
        sensors= [ # Select the ones you want in the order you want them.
        'curLapTime',
        'lastLapTime',
        'stucktimer',
        'damage',
        'focus',
        'fuel',
        'gear',
        'distRaced',
        'distFromStart',
        'racePos',
        'opponents',
        'wheelSpinVel',
        'z',
        'speedZ',
        'speedY',
        'speedX',
        'targetSpeed',
        'rpm',
        'skid',
        'slip',
        'track',
        'trackPos',
        'angle',
        ]

        #for k in sorted(self.d): # Use this to get all sensors.
        for k in sensors:
            if type(self.d.get(k)) is list: # Handle list type data.
                if k == 'track': # Nice display for track sensors.
                    strout= str()
                    raw_tsens= ['%.1f'%x for x in self.d['track']]
                    strout+= ' '.join(raw_tsens[:9])+'_'+raw_tsens[9]+'_'+' '.join(raw_tsens[10:])
                elif k == 'opponents': # Nice display for opponent sensors.
                    strout= str()
                    for osensor in self.d['opponents']:
                        if   osensor >190: oc= '_'
                        elif osensor > 90: oc= '.'
                        elif osensor > 39: oc= chr(int(osensor/2)+97-19)
                        elif osensor > 13: oc= chr(int(osensor)+65-13)
                        elif osensor >  3: oc= chr(int(osensor)+48-3)
                        else: oc= '?'
                        strout+= oc
                    strout= ' -> '+strout[:18] + ' ' + strout[18:]+' <-'
                else:
                    strlist= [str(i) for i in self.d[k]]
                    strout= ', '.join(strlist)
            else: # Not a list type of value.
                if k == 'gear': # This is redundant now since it's part of RPM.
                    gs= '_._._._._._._._._'
                    p= int(self.d['gear']) * 2 + 2  # Position
                    l= '%d'%self.d['gear'] # Label
                    if l=='-1': l= 'R'
                    if l=='0':  l= 'N'
                    strout= gs[:p]+ '(%s)'%l + gs[p+3:]
                elif k == 'damage':
                    strout= '%6.0f %s' % (self.d[k], bargraph(self.d[k],0,10000,50,'~'))
                elif k == 'fuel':
                    strout= '%6.0f %s' % (self.d[k], bargraph(self.d[k],0,100,50,'f'))
                elif k == 'speedX':
                    cx= 'X'
                    if self.d[k]<0: cx= 'R'
                    strout= '%6.1f %s' % (self.d[k], bargraph(self.d[k],-30,300,50,cx))
                elif k == 'speedY': # This gets reversed for display to make sense.
                    strout= '%6.1f %s' % (self.d[k], bargraph(self.d[k]*-1,-25,25,50,'Y'))
                elif k == 'speedZ':
                    strout= '%6.1f %s' % (self.d[k], bargraph(self.d[k],-13,13,50,'Z'))
                elif k == 'z':
                    strout= '%6.3f %s' % (self.d[k], bargraph(self.d[k],.3,.5,50,'z'))
                elif k == 'trackPos': # This gets reversed for display to make sense.
                    cx='<'
                    if self.d[k]<0: cx= '>'
                    strout= '%6.3f %s' % (self.d[k], bargraph(self.d[k]*-1,-1,1,50,cx))
                elif k == 'stucktimer':
                    if self.d[k]:
                        strout= '%3d %s' % (self.d[k], bargraph(self.d[k],0,300,50,"'"))
                    else: strout= 'Not stuck!'
                elif k == 'rpm':
                    g= self.d['gear']
                    if g < 0:
                        g= 'R'
                    else:
                        g= '%1d'% g
                    strout= bargraph(self.d[k],0,10000,50,g)
                elif k == 'angle':
                    asyms= [
                          "  !  ", ".|'  ", "./'  ", "_.-  ", ".--  ", "..-  ",
                          "---  ", ".__  ", "-._  ", "'-.  ", "'\.  ", "'|.  ",
                          "  |  ", "  .|'", "  ./'", "  .-'", "  _.-", "  __.",
                          "  ---", "  --.", "  -._", "  -..", "  '\.", "  '|."  ]
                    rad= self.d[k]
                    deg= int(rad*180/PI)
                    symno= int(.5+ (rad+PI) / (PI/12) )
                    symno= symno % (len(asyms)-1)
                    strout= '%5.2f %3d (%s)' % (rad,deg,asyms[symno])
                elif k == 'skid': # A sensible interpretation of wheel spin.
                    frontwheelradpersec= self.d['wheelSpinVel'][0]
                    skid= 0
                    if frontwheelradpersec:
                        skid= .5555555555*self.d['speedX']/frontwheelradpersec - .66124
                    strout= bargraph(skid,-.05,.4,50,'*')
                elif k == 'slip': # A sensible interpretation of wheel spin.
                    frontwheelradpersec= self.d['wheelSpinVel'][0]
                    slip= 0
                    if frontwheelradpersec:
                        slip= ((self.d['wheelSpinVel'][2]+self.d['wheelSpinVel'][3]) -
                              (self.d['wheelSpinVel'][0]+self.d['wheelSpinVel'][1]))
                    strout= bargraph(slip,-5,150,50,'@')
                else:
                    strout= str(self.d[k])
            out+= "%s: %s\n" % (k,strout)
        return out

class DriverAction():
    '''What the driver is intending to do (i.e. send to the server).
    Composes something like this for the server:
    (accel 1)(brake 0)(gear 1)(steer 0)(clutch 0)(focus 0)(meta 0) or
    (accel 1)(brake 0)(gear 1)(steer 0)(clutch 0)(focus -90 -45 0 45 90)(meta 0)'''
    def __init__(self):
       self.actionstr= str()
       # "d" is for data dictionary.
       self.d= { 'accel':0.2,
                   'brake':0,
                  'clutch':0,
                    'gear':1,
                   'steer':0,
                   'focus':[-90,-45,0,45,90],
                    'meta':0
                    }
#Function to keep the control signals within limits. (used only in the first car controller)
    def clip_to_limits(self):

        """There pretty much is never a reason to send the server
        something like (steer 9483.323). This comes up all the time
        and it's probably just more sensible to always clip it than to
        worry about when to.(not actually required)"""

        self.d['steer']= clip(self.d['steer'], -1, 1)
        self.d['brake']= clip(self.d['brake'], 0, 1)
        self.d['accel']= clip(self.d['accel'], 0, 1)
        self.d['clutch']= clip(self.d['clutch'], 0, 1)
        if self.d['gear'] not in [-1, 0, 1, 2, 3, 4, 5, 6]:
            self.d['gear']= 0
        if self.d['meta'] not in [0,1]:
            self.d['meta']= 0
        if type(self.d['focus']) is not list or min(self.d['focus'])<-180 or max(self.d['focus'])>180:
            self.d['focus']= 0

    def __repr__(self):
        self.clip_to_limits()
        out= str()
        for k in self.d:
            out+= '('+k+' '
            v= self.d[k]
            if not type(v) is list:
                out+= '%.3f' % v
            else:
                out+= ' '.join([str(x) for x in v])
            out+= ')'
        return out
        return out+'\n'

    def fancyout(self):
        '''Specialty output for useful monitoring of bot's effectors.'''
        out= str()
        od= self.d.copy()
        od.pop('gear','')
        od.pop('meta','')
        od.pop('focus','')
        for k in sorted(od):
            if k == 'clutch' or k == 'brake' or k == 'accel':
                strout=''
                strout= '%6.3f %s' % (od[k], bargraph(od[k],0,1,50,k[0].upper()))
            elif k == 'steer': # Reverse the graph to make sense.
                strout= '%6.3f %s' % (od[k], bargraph(od[k]*-1,-1,1,50,'S'))
            else:
                strout= str(od[k])
            out+= "%s: %s\n" % (k,strout)
        return out

# == Misc Utility Functions
def destringify(s):
    '''makes a string into a value or a list of strings into a list of
    values (if possible)'''
    if not s: return s
    if type(s) is str:
        try:
            return float(s)
        except ValueError:
            print("Could not find a value in %s" % s)
            return s
    elif type(s) is list:
        if len(s) < 2:
            return destringify(s[0])
        else:
            return [destringify(i) for i in s]

#Function that performs the acceleration for the second car controller
def follower_acceleration(rightd, distance, S):
    global k_ti, k_td, M_c1, v_d, d_d
    vx = S['speedX']
    vy = S['speedY']
    h_plus = 1.5
    Dd = d_d + 0.5*vx*1000.0/3600
    if ( (1 - 0.2*math.fabs(rightd - d_ld)/d_ld)*v_d - vx >= 0 and (distance >= 1.1 * Dd)):
        s_t = 1
    else:
        s_t = 0

    acceleration = s_t*k_ti*(distance - d_d) - s_t*k_td*vx + s_t*M_c1*math.fabs(vy)

    return acceleration

#Function which calculates the braking for the second car controller
def follower_braking(rightd, distance, S):
    global k_bi, k_bd, v_d, d_d
    vx = S['speedX']
    vy = S['speedY']
    h_minus = 0.5
    Dd = d_d + 0.5*vx*1000.0/3600
    if ((1-0.2*math.fabs(rightd - d_ld)/d_ld)*v_d - vx < 0.0 or distance < 1.0 * Dd):
        s_b = 1
    else:
        s_b = 0

    brake = s_b*k_bi*(d_d - distance) + s_b*k_bd*vx

    return brake

#Function which computes the steering value for the second car controller
def follower_steering(rightd,S):
    global k_si, k_sd, M_c2, d_ld,errorL
    vx = S['speedX']
    vy = S['speedY']

    error = rightd - d_ld
    if (error >=0):
        sign = -1.0
    else:
        sign = 1.0
    steering = sign * (math.fabs(k_si*(error)+1*(error-errorL)) - k_sd*math.fabs(vy) - M_c2*vx)
    errorL = error
    #steering=0.000035
    return steering

#Function that receives the data from the server and does the required control calculation
#The controller for both the car's are invoked here
def drive_example(c,num):
    global lasterror
    global lasterror2
    global start_time
    global flag
    global valuelist1
    #global csvfile
    '''This is only an example. It will get around the track but the
    correct thing to do is write your own `drive()` function.'''
    S,R= c.S.d,c.R.d
    target_speed=90
    # find angle of oponnent vehicle
    lidarvalues=S['opponents']
    distance=min(lidarvalues)
    ind=lidarvalues.index(distance)
    angle=-180.0+(10*ind)

#The leader car controller
    if (num==0):
        car0[0]=S['speedX']
        car0[1]=angle
        car0[2]=distance
        car0[3]=S['trackPos']
        current_time = time.time()
        if ((current_time - start_time) >= 20 and (current_time - start_time) <= 30):
            target_speed = 40
        elif ((current_time - start_time) >= 40 and (current_time - start_time) <= 50):
            target_speed = 65
        elif ((current_time - start_time) >= 70 and (current_time - start_time) <= 100):
            target_speed = 50
        else:
            target_speed=60
        #The steering value calculation
        R['steer']= S['angle']*10 / PI
        # Steer To Center
        R['steer']-= S['trackPos']*.10
        car0[4]=R['steer']
        #The acceleration value calculation
        if S['speedX'] < target_speed - (R['steer']*50):
            R['accel']+= .01# Add the accelarction code here
            car0[5]=R['accel']
        else:
            R['accel']-= .01 #Brake code goes here
            car0[5]=R['accel']
        if S['speedX']<10:
            R['accel']+= 1/(S['speedX']+.1)
            car0[5]=R['accel']
        #valuelist.append(car0)
        #print(valuelist)

#The Follower car controller with Siyuan's controller
    if (num==1):
        starttime = time.time()
        edges=S['track']
        leftp=edges[0]
        leftc=edges[3]
        rightc=edges[15]
        rightp=edges[18]

        #Calculating the Lateral_Distance from the sidewall, using the LIDAR2 (track) values
        rightalpha=math.atan((1.0*rightc*math.cos(30.0*PI/180) - rightp)/(1.0*rightc*math.sin(30.0*PI/180)))
        rightd=rightp * math.cos(rightalpha)
        #Steering, acceleration and Brake values calculated by the above defined functions
        R['steer'] = follower_steering(rightd, S)
        R['accel'] = follower_acceleration(rightd, distance, S)
        R['brake'] = follower_braking(rightd, distance, S)

        car1[0]=S['speedX']
        car1[1]=S['speedY']
        car1[2]=lidarvalues
        car1[3]=edges
        car1[4]=R['steer']
        car1[5]=R['accel']
        car1[6]=R['brake']

#Writing data into CSV
        writer = csv.writer(csvfile)
        writer.writerow(car1)

        print(R['accel'], ':', R['brake'], ':', distance, ':', S['speedX'], ':', S['speedY'])
        print('Lateral_Distance', rightd, 'Steering_value', R['steer'])


    if ((S['wheelSpinVel'][2]+S['wheelSpinVel'][3]) -(S['wheelSpinVel'][0]+S['wheelSpinVel'][1]) > 5):
        R['accel']-= .2

    # Automatic Transmission
    R['gear']=1
    if S['speedX']>50:
        R['gear']=2
    if S['speedX']>80:
        R['gear']=3
    if S['speedX']>110:
        R['gear']=4
    if S['speedX']>140:
        R['gear']=5
    if S['speedX']>170:
        R['gear']=6


    return num,S['speedX'],angle,distance

'''
#Function to write data into influxdb
def threading_function():
    global car0
    global car1
    global valuelist1
    global flag
    global csvfile
    row=[]
    print("Thread Started")
    mydb= influxdb.InfluxDBClient('localhost',8086,'torc','trial','torcs')
    mydb.create_database('torcs')
    retention_policy='awesome_policy'
    mydb.create_retention_policy('retention_policy','10d',10,default=True)
    while(1):

               writer = csv.writer(csvfile)
               writer.writerow(car1)
               lists1 = [{
                            'measurement':'car_speed',
                            'tags':{"car_id":1},
                            'time':time.strftime('%Y-%m-%d %H:%M:%S'),
                            'fields':{
                                'speedX':car1[0],
                                'speedY':car1[1],
                                'LIDAR1':car1[2],
                                'LIDAR2':ca1[3],
                                'Accleration':car1[4],
                                'Brake':car1[5],
                                'Steering':car1[6],
                                },
                                }]
               print(lists1)
               res=mydb.write_points(lists1)
               sleep(0.01)
'''
# ================ MAIN ================
if __name__ == "__main__":

    global start_time
    #thread = Thread(target = threading_function)
    #thread.start()
    start_time = time.time()
    CS=[Client(p=P) for P in [3001, 3002]]
    for step in range(CS[0].maxSteps,0,-1):
        num=0
        for C in CS:
            C.get_servers_input()
            a,b,angle,dist=drive_example(C,num)
            C.respond_to_server()
            num+=1
    C.shutdown()

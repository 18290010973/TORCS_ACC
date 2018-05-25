import csv
from keras.models import Sequential, Model
from keras.layers import Dense
from keras.layers import Input
from keras.layers import concatenate
from sklearn.model_selection import train_test_split
from sklearn.utils import shuffle
from keras.optimizers import Adam
import numpy as np
from keras import optimizers
from keras.callbacks import ModelCheckpoint
from keras.models import model_from_json
#from keras.layers import Dense, Flatten, Input, merge, Lambda
#from keras.initializations import normal, identity
import sklearn
import math

PI = 3.1415926
X=[]
Y=[]
pre_rightd = 0.0
with open('/home/ubuntu/torcs-code/2carsnn/dataset2.csv') as File:
    reader=csv.reader(File)
    for row in reader:
        lidar1 = row[2].strip().split(',')
        lidar1[0]=lidar1[0][1:]
        lidar1[len(lidar1)-1]=lidar1[len(lidar1)-1][:-1]
        lidar1 = [float(i) for i in lidar1]

        lidar2 = row[3].strip().split(',')
        lidar2[0]=lidar2[0][1:]
        lidar2[len(lidar2)-1]=lidar2[len(lidar2)-1][:-1]
        lidar2 = [float(i) for i in lidar2]

        distance = min(lidar1)
        rightc = lidar2[15]
        rightp = lidar2[18]
        rightalpha=math.atan((1.0*rightc*math.cos(30.0*PI/180) - rightp)/(1.0*rightc*math.sin(30.0*PI/180)))
        rightd=rightp * math.cos(rightalpha)

        X.append([float(row[0])/100.0, float(row[1])/20.0, float(distance)/50.0, float(rightd)/10.0, float(pre_rightd)/10.0])
        Y.append([float(row[4]), float(row[5]), float(row[6])])
        pre_rightd = rightd
print(X[0], Y[0])
#X = sklearn.preprocessing.normalize(X)
#Y = sklearn.preprocessing.normalize(Y)
print(X[0], Y[0])

inp=Input(shape=(5,))
layer1=Dense(40,activation='relu', kernel_initializer='glorot_normal', bias_initializer='zeros')(inp)
layer2=Dense(40,activation='relu', kernel_initializer='glorot_normal', bias_initializer='zeros')(layer1)
#layer3=Dense(200,activation='relu')(layer2)
layer3=Dense(40,activation='relu', kernel_initializer='glorot_normal', bias_initializer='zeros')(layer2)
#layer3=Dense(200,activation='relu')(layer2)
#layer4=Dense(200,activation='relu')(layer3)
#layer5=Dense(300,activation='relu')(layer4)
layer5a=Dense(1,activation='tanh')(layer3)
layer5b=Dense(1,activation='tanh')(layer3)
layer5c=Dense(1,activation='tanh')(layer3)
out=concatenate([layer5a, layer5b, layer5c])
model=Model(inputs=inp, outputs=out)

adam = optimizers.Adam(lr=0.1)
#sgd = optimizers.SGD(lr=1)
model.compile(loss='mse', optimizer=adam,metrics=['accuracy', 'mae'])

model.fit(np.array(X),np.array(Y),nb_epoch=10,batch_size=256)
model_json = model.to_json()
with open("model.json", "w") as json_file:
    json_file.write(model_json)

model.save_weights("model.h5")
print("Saved model to disk")

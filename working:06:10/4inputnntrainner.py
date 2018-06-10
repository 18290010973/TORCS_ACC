import csv
import keras
import Controller
from keras.models import Sequential, Model
from keras.layers import Dense
from keras.layers import Input
from keras.layers import concatenate
from keras.optimizers import Adam
import numpy as np
from keras import optimizers
from keras.callbacks import ModelCheckpoint
from keras.models import model_from_json, load_model
#from keras.layers import Dense, Flatten, Input, merge, Lambda
#from keras.initializations import normal, identity
from pathlib import Path

PI = 3.1415926
def readData():
    X = []
    Y = []
    trackPoslast=0.0
    with open('fulldata2.csv') as File:
        reader=csv.reader(File)
        for row in reader:
            rowFlatten = []

            #steering control
            rowFlatten.append(float(row[2]))
            rowFlatten.append(float(row[3]))
            lidar2 = row[6].strip().split(',')
            lidar2[0] = lidar2[0][1:]
            lidar2[len(lidar2)-1]=lidar2[len(lidar2)-1][:-1]
            lidar2 = [float(i) for i in lidar2]
            trackPos = Controller.trackPosCalc(lidar2)
            rowFlatten.append(float(trackPos))
            rowFlatten.append(float(trackPoslast))
            trackPoslast = trackPos

            #Speed control
            lidar1 = row[5].strip().split(',')
            lidar1[0] = lidar1[0][1:]
            lidar1[len(lidar1)-1]=lidar1[len(lidar1)-1][:-1]
            lidar1 = [float(i) for i in lidar1]
            distance=min(lidar1)
            rowFlatten.append(float(distance))
            rowFlatten.append(float(row[0]))
            rowFlatten.append(float(row[1]))
            
            #Appending inputs
            X.append(rowFlatten)
           
            #Appending outputs
            Y.append([float(row[17]), float(row[18]),float(row[19])])
  
        #Converting to numpy array
        X = np.array(X)
        Y = np.array(Y)
        index = np.arange(len(X))
        index = np.random.permutation(index)#Shuffle the array
	
        #selecting training dataset		
        X = X[index[:200000]]
        Y = Y[index[:200000]]
        print(X.shape)
        return X,Y

def normalizeData(X, Y):
    X_mean = np.mean(X, axis=0)
    X_std = np.std(X, axis=0)
    X = (X - X_mean)/X_std
    Y_mean = np.mean(Y, axis=0)
    Y_std = np.std(Y, axis=0)
    #print(X_mean, X_std,':', Y_mean, Y_std)
    np.savez('./normalizeParameters', X_mean=X_mean, X_std=X_std, Y_mean=Y_mean, Y_std=Y_std)
    return X, Y

def createModel():
    '''
    model = Sequential()
    model.add(Dense(40, input_dim=7, kernel_initializer='glorot_normal', bias_initializer='zeros', activation='relu'))
    model.add(Dense(40, kernel_initializer='glorot_normal', bias_initializer='zeros', activation='relu'))
    model.add(Dense(20, kernel_initializer='glorot_normal', bias_initializer='zeros', activation='relu'))
    #model.add(Dense(20, kernel_initializer='glorot_normal', bias_initializer='zeros', activation='relu'))
    model.add(Dense(3, kernel_initializer='glorot_normal', bias_initializer='zeros', activation='tanh'))
    '''
    inp=Input(shape=(7,))
    layer1=Dense(20,activation='relu', kernel_initializer='glorot_normal')(inp)
    layer2=Dense(40,activation='relu', kernel_initializer='glorot_normal')(layer1)
    layer3=Dense(40,activation='relu', kernel_initializer='glorot_normal')(layer2)
    layer4=Dense(20,activation='relu', kernel_initializer='glorot_normal')(layer3)
    layer4a=Dense(1,activation='tanh', kernel_initializer='glorot_normal')(layer4)
    layer4b=Dense(1,activation='sigmoid', kernel_initializer='glorot_normal')(layer4)
    layer4c=Dense(1,activation='sigmoid', kernel_initializer='glorot_normal')(layer4)
    out=concatenate([layer4a, layer4b, layer4c])
    model=Model(inputs=inp, outputs=out)
    return model

def trainModel(model, X, Y):
    #RMSprop=keras.optimizers.RMSprop(lr=0.01, rho=0.9, epsilon=None, decay=0.0)
    #sgd = keras.optimizers.SGD(lr=0.01, momentum = 0.9)
    adam = keras.optimizers.Adam(lr=0.001, beta_1=0.9, beta_2=0.999, epsilon=None, decay=0.0, amsgrad=False)
    model.compile(loss='mse', optimizer=adam)	
    
    # checkpoint
    filePath = "weights.best.hdf5"
    checkpoint = ModelCheckpoint(filePath, monitor='loss', verbose=1, save_best_only=True, mode='min')
    callbacks_list = [checkpoint]
    model.fit(X, Y,  epochs=50, batch_size=32, callbacks=callbacks_list, verbose=1)

def saveModel(model):
	model_json = model.to_json()
	with open("model.json", "w") as json_file:
		json_file.write(model_json)
		
	model.save_weights("model.h5")
	print("Saved model to disk")


if __name__ == "__main__":
    X, Y = readData()
    X, Y = normalizeData(X, Y)
    if (Path("model.json").is_file() and Path("weights.best.hdf5").is_file()):
        with open('model.json', 'r') as jfile:
            model = model_from_json(jfile.read())
        model.load_weights("weights.best.hdf5")
        print("load from the existing model...")
    else:
        model = createModel()
        print("create a new model")
            
    trainModel(model, X, Y)
    saveModel(model)

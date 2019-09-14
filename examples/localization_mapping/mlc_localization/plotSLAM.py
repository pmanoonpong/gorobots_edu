import numpy as np
import matplotlib.pyplot as plt
import matplotlib
import sys

def plotMap(ax):
    #Load map
    mapName = "circle.map"
    index, posX, posY, sizeX, sizeY, occupied = np.loadtxt(mapName, unpack=True)
    index = index.astype(int)
    #Plot map
    for i in range(index[-1]+1):
        box = matplotlib.patches.Rectangle((posX[i],posY[i]), sizeX[i], sizeY[i], color = str(1 - occupied[i]), zorder = 0)
        ax.add_patch(box)
        ax.set_xlim([np.min(posX)-1,np.max(posX)+1])
        ax.set_ylim([np.min(posY)-1,np.max(posY)+1])
    

dataDirectory = "data/"
#Get particles
indexParticle, posXParticle, posYParticle, truePosX, truePosY, yaw, yawTrue = np.loadtxt(dataDirectory + "particles.dat", unpack=True)
dataSim = np.loadtxt("sensorData/sensorData.dat")
truePosX = dataSim[:,1]
truePosY = dataSim[:,2]

#Stepnumber
stepNumber = int(sys.argv[2])

#Init plot
fig = plt.figure()
ax = fig.add_subplot(111)
#Check options
#Plot particles with highest weight for each step
if sys.argv[1] == "path":
    #Configure plot
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.plot(posXParticle[:stepNumber],posYParticle[:stepNumber],'.-',color='red',label='Path Particle')
    
#Plot all particles for each step
if sys.argv[1] == "cloud":
    #Load particles of step
    particleName = dataDirectory + "particles_" + str(stepNumber) + ".dat"
    indexParticle, posXParticle, posYParticle, orientationParticles, weightParticle = np.loadtxt(particleName, unpack = True)
    ax.scatter(posXParticle, posYParticle, marker='o',color='red',label='Particles')

#Plot true path    
ax.scatter(truePosX[:stepNumber],truePosY[:stepNumber],marker='o',color='green',label='Path True')
#Plot map
plotMap(ax)
    
ax.legend()
plt.show()
            
            
            
            
            
            
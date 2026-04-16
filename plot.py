import numpy as np
import matplotlib.pyplot as plt

def transform(position,angle,key):
    x = position[0]
    y = position[1]
    z = position[2]
    if key == 1:
        angle = angle[0]
        return np.array([[1,0,0,x],[0,np.cos(angle),np.sin(angle),y],[0,-np.sin(angle),np.cos(angle),z],[0,0,0,1]])
    elif key == 2:
        angle = angle[1]
        return np.array([[np.cos(angle),0,-np.sin(angle),x],[0,1,0,y],[np.sin(angle),0,np.cos(angle),z],[0,0,0,1]])  
    elif key == 3:
        angle = angle[2]
        return np.array([[np.cos(angle),np.sin(angle),0,x],[-np.sin(angle),np.cos(angle),0,y],[0,0,1,z],[0,0,0,1]])


wTc = transform([0,0,1.6],[0,-np.pi/2,0],2)

cTo = transform([0.9750000581145322, 0.17825690623723656, -0.06684633983896371],[np.deg2rad(117),0,0],1)

print(wTc@cTo)

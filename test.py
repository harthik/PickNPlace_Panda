import pybullet as p
import pybullet_data
import numpy as np
import cv2 as cv
from numpy import random
import time

class sim:
    def __init__(self):
        p.connect(p.GUI)
        p.setGravity(0,0,-9.81)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.loadURDF("plane.urdf")
        p.loadURDF("table/table.urdf")
    def spawn_jenga(self, position, orientation):
        jengaId = p.loadURDF("jenga/jenga.urdf", basePosition=position, baseOrientation=orientation)
        return jengaId

class robot:
    def __init__(self):
        self.robotId = p.loadURDF("franka_panda/panda.urdf", basePosition=[-0.5,0,0.6], baseOrientation=p.getQuaternionFromEuler([0, 0, 0]), useFixedBase=True)
        for i in range(7):
            p.resetJointState(self.robotId, i, [0, -0.785, 0, -2.356, 0, 1.571, 0.785][i])
    def get_end_effector_state(self):
        end_effector_state = p.getLinkState(self.robotId, 11)
        self.current_pos = end_effector_state[0]
        self.current_orn = end_effector_state[1]
        return self.current_pos , self.current_orn
    def move(self, target_pos, target_orn):
        self.get_end_effector_state()
        joint_angles = p.calculateInverseKinematics(self.robotId, 11, target_pos, target_orn, maxNumIterations=500, residualThreshold=0.001)
        p.setJointMotorControlArray(self.robotId, 
            range(7), 
            p.POSITION_CONTROL, 
            targetPositions=joint_angles[:7],
            forces=[500] * 7,
            positionGains=[0.01] * 7,
            velocityGains=[0.5] * 7)
    def ef_control(self,state):
        if state == False:
            ef_pos = [0.15] * 2
        elif state == True:
            ef_pos = [0.01] * 2
        p.setJointMotorControlArray(self.robotId, 
                [9,10], 
                p.POSITION_CONTROL, 
                targetPositions=ef_pos,
                forces=[200] * 2,
                positionGains=[0.1] * 2)



class camera:
    def __init__(self, position,orientation):
        self.position = position
        self.orientation = orientation
        self.W = 320
        self.H = 240
        self.fov = 60
        self.aspect = self.W / self.H
        self.fy = self.H / (2 * np.tan(np.deg2rad(self.fov) / 2))
        self.fx = self.fy
        self.cx = (self.W - 1) / 2
        self.cy = (self.H - 1) / 2
        self.near = 0.5
        self.far = 1.0
        self.wTc = self.transform(position,np.deg2rad(orientation),2)
        self.view_matrix = p.computeViewMatrixFromYawPitchRoll(cameraTargetPosition=[0, 0, 0.6], 
                                                         distance=1, 
                                                         yaw=self.orientation[0], 
                                                         pitch=self.orientation[1], 
                                                         roll=self.orientation[2], 
                                                         upAxisIndex=2)
        self.projection_matrix = p.computeProjectionMatrixFOV(fov=self.fov, aspect=self.aspect, nearVal=self.near, farVal=self.far)
    def get_image(self):
        width, height, self.rgbImg, self.depthImg, self.segImg = p.getCameraImage(width=self.W, 
                                                                                  height=self.H, 
                                                                                  viewMatrix=self.view_matrix, 
                                                                                  projectionMatrix=self.projection_matrix, 
                                                                                  flags=p.ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX)
    def get_object_orn(self,object_id):
        seg = np.array(self.segImg)

        body_ids = seg & ((1 << 24) - 1)
        mask = (body_ids == object_id)

        depth_mask = np.argwhere(mask)
        depth_img = self.depthImg[depth_mask[:,0],depth_mask[:,1]]
        mask = mask.astype(np.uint8)
        max_depth = depth_img.min()

        mask *= 255
        contours, _  = cv.findContours(mask,cv.RETR_TREE,cv.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            rect = cv.minAreaRect(contour)
            u = round(rect[0][0])
            v = round(rect[0][1])+1
            print(u,v)
            depth = self.depthImg[u,v]
            x = self.far * self.near / (self.far - (self.far - self.near) * depth)
            z = (u - self.cx) * x / self.fx
            y = (self.cy - v) * x / self.fy
            M = cv.moments(contour)
            if M["m00"] != 0:
                angle = 0.5 * np.arctan2(2 * M["mu11"], M["mu20"] - M["mu02"])
        wTc = self.wTc
        rot = wTc@[[x],[y],[z],[1]]
        return angle, rot, max_depth
    def transform(self,position,angle,key):
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
    def workspace(self, target_ids):
        c1 = 100
        c2 = 219
        r1 = 120
        r2 = 239
        seg = np.array(self.segImg)
        obj_ids = seg & ((1 << 24) - 1)

        mask = np.isin(obj_ids, target_ids)
        result = np.any(mask[r1:r2, c1:c2])
        return result
 

if __name__ == "__main__":
    # Setup Simulation
    sim = sim()
    panda = robot()
    camera = camera(position=[0, 0, 1.6], orientation=[0, -90, 0])
    camera.get_image()

    id = sim.spawn_jenga(position=[random.uniform(-0.1, 0.1), random.uniform(-0.1,0.1), random.uniform(0.6,0.7)], 
                            orientation=p.getQuaternionFromEuler([0, 0, random.uniform(0, np.pi)]))
    
    # Simulation update to let everything spawn and settle.
    for _ in range(480):
        p.stepSimulation()
        time.sleep(1/240)
    camera.get_image()
    status = camera.workspace(id)

    # Get object positions and orientations of jenga blocks
    angle,pos,max_depth = camera.get_object_orn(id)
    print("angle",np.rad2deg(angle))
    print("pos",pos)

    target_pos = [pos[0],pos[1],pos[2]+0.1]
    target_orn = p.getQuaternionFromEuler([np.pi,0,-angle])
    panda.move(target_pos,target_orn)
    for _ in range(480):
        p.stepSimulation()
        time.sleep(1/240)
    camera.get_image()
    #while True:
    #    p.stepSimulation()
    #    time.sleep(1./240.)
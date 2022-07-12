#!/usr/bin/env python3

import numpy as np
import math as m

def Rx(theta):
  return np.matrix([[ 1, 0           , 0           ],
                   [ 0, m.cos(theta),-m.sin(theta)],
                   [ 0, m.sin(theta), m.cos(theta)]])

def Ry(theta):
  return np.matrix([[ m.cos(theta), 0, m.sin(theta)],
                   [ 0           , 1, 0           ],
                   [-m.sin(theta), 0, m.cos(theta)]])

def Rz(theta):
  return np.matrix([[ m.cos(theta), -m.sin(theta), 0 ],
                   [ m.sin(theta), m.cos(theta) , 0 ],
                   [ 0           , 0            , 1 ]])

def rectangle_spheres(origin, w, h, r, extra_s = 0):
    # Method to determine a rectangle of sphere with a certain radius qaually distributes on the perimeter
    perimeter = 2*h+2*w
    # n_spheres = int((perimeter + 2*r)/(2*r) - 1 + extra_s)
    n_spheres = 10
    rot_matrix = Rz(m.pi/2)

    # Initialize matrix containing sphere points
    sphere_loc = np.zeros((3, n_spheres))
    dist = perimeter/float(n_spheres)
    for i in range(n_spheres):
        # x location of the sphere, unwrapped rectangle into a line
        sphere_loc[0,i] = dist*i

    for i in range(n_spheres):
        if sphere_loc[0,i] > w:
            # Translate to origin
            sphere_loc[0,i] -= w
            # Rotate
            sphere_loc[:,i] =  np.matmul(rot_matrix, sphere_loc[:,i])
            # Translate back
            sphere_loc[0,i] += w

    for i in range(n_spheres):
        if sphere_loc[1,i] > h:
            sphere_loc[0,i] -= w
            sphere_loc[1,i] -= h
            sphere_loc[:,i] =  np.matmul(rot_matrix, sphere_loc[:,i])
            sphere_loc[0,i] += w
            sphere_loc[1,i] += h

    for i in range(n_spheres):
        if sphere_loc[0,i] < 0:
            sphere_loc[1,i] -= h
            # Rotate
            sphere_loc[:,i] =  np.matmul(rot_matrix, sphere_loc[:,i])
            # Translate back
            sphere_loc[1,i] += h

    # Translate origin in the center of rectangle
    sphere_loc[0,:] = sphere_loc[0,:] - w/2 + origin[0]
    sphere_loc[1,:] = sphere_loc[1,:] - h/2 + origin[1]
    sphere_loc[2,:] = sphere_loc[2,:] + origin[2]

    return sphere_loc.T

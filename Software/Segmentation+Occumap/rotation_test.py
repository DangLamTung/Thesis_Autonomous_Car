
#!/usr/bin/env python3
import numpy as np
import math
'''If you know the three rotations a, b and g, about x, y, z respectively, using the 
right-hand rule.
The x0, y0, z0 are the translations between the origins of the two coordinate 
systems.'''
#https://stackoverflow.com/questions/22175385/3d-matrix-perspective-transform/22311973#22311973

def make_matrix(roll,pitch,heading,x0,y0,z0):
    a = math.radians(roll)
    b = math.radians(pitch)
    g = math.radians(heading)

    T = np.array([[ math.cos(b)*math.cos(g), (math.sin(a)*math.sin(b)*math.cos(g) + 
    math.cos(a)*math.sin(g)), (math.sin(a)*math.sin(g) - 
    math.cos(a)*math.sin(b)*math.cos(g)), x0],
    [-math.cos(b)*math.sin(g), (math.cos(a)*math.cos(g) - 
    math.sin(a)*math.sin(b)*math.sin(g)), (math.sin(a)*math.cos(g) + 
    math.cos(a)*math.sin(b)*math.sin(g)), y0],
    [        math.sin(b), -math.sin(a)*math.cos(b), math.cos(a)*math.cos(b), z0],
    [ 0, 0, 0, 1]])
    return T

def Rz(theta):
    return np.matrix([[ np.cos(theta), -np.sin(theta), 0 ],
                   [ np.sin(theta), np.cos(theta) , 0 ],
                   [ 0           , 0            , 1 ]])
if __name__ == '__main__':

    # T = make_matrix(0,0,45,-296,-360,0)
    # orig = load_orig()
    pos = np.array([296 + 10,360+10,0])
    pos =pos - np.array([296,360,0])
    Rot = Rz(np.radians(45))
    pos = Rot.dot(pos)
    # new = T.dot(orig)
    print(pos)
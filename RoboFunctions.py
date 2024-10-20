import numpy as np

# coding: utf-8
def find_t1(x, y, l1=27.3, l2=32.7):
    d = np.sqrt(x**2+y**2)
    return 180/np.pi * np.arccos((l1**2 + d**2-l2**2)/(2*l1*d))
    
def find_t2(x, y, l1=27.3, l2=32.7):
    d = np.sqrt(x**2+y**2)
    return 180/np.pi * np.arccos(-(l1**2+l2**2-d**2)/(2*l1*l2))
    
def get_x(t1, t2, l1=27.3, l2=32.7):
    t1 = np.pi/180*t1
    t2 = np.pi/180*t2
    return np.cos(t2 - t1) * l2 + l1 * np.cos(t1)
    
def get_y(t1, t2, l1=27.3, l2=32.7):
    t1 = np.pi/180*t1
    t2 = np.pi/180*t2
    return -np.sin(t2 - t1) * l2 + l1 * np.sin(t1)
    

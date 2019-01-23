import csv
import math
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

csvfile = open("../data/gts_hemi_icosa_4.csv", "rb")
reader = csv.reader(csvfile, delimiter = " ")
x = []
y = []
z = []

for row in reader:
    x.append(float(row[0]))
    y.append(float(row[1]))
    z.append(float(row[2]))

phi = []
theta = []
pose = []
data = []
rad = np.pi/180
yaw = [0, 15*rad, 30*rad, 45*rad ,60*rad ,75*rad ,90*rad ,105*rad ,120*rad ,135*rad ,150*rad ,165*rad ,180*rad]

print len(x)

for j in range(len(yaw)):
    for i in range(0, len(x)):
        pose = [x[i], y[i], z[i]]

        try:
            theta = np.arccos(z[i]/math.sqrt(x[i]**2 + y[i]**2 + z[i]**2))
            phi = np.arctan2(y[i], x[i])

        except:
            theta = 0

        if(theta <= 90*np.pi/180):
            pose.append(phi)
            pose.append(theta)
            pose.append(yaw[j])
            data.append(pose)


# x, y, z, phi, theta, yaw

# f = open('../data/gazebo_hemi_icosa_div4.csv', 'w')
# for i in range(len(data)):
    # f.write(str(data[i][0]) + " " +
            # str(data[i][1]) + " " +
            # str(data[i][2]) + " " +
            # str(data[i][3]) + " " +
            # str(data[i][4]) + " " +
            # str(data[i][5]) + " " + "\n")

# f.close()




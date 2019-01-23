import csv
import numpy as np
import matplotlib
print(matplotlib.__version__)
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

csvfile = open("../data/gts_sphere_4.csv", "rb")
# csvfile = open("../data/gazebo_hemi_icosa_div4.csv", "rb")
reader = csv.reader(csvfile, delimiter = " ")
# theta, phi, x, y, z
# theta = []
# phi = []
x = []
y = []
z = []

for row in reader:
    # theta.append(float(row[0]))
    # phi.append(float(row[1]))
    x.append(float(row[0]))
    y.append(float(row[1]))
    z.append(float(row[2]))

# for i in range(0, len(theta)):
    # print ("%d %f %f %f %f %f" % (i, theta[i], phi[i], x[i], y[i], z[i]))
hemi_x = []
hemi_y = []
hemi_z = []

# f = open('../data/gts_hemi_icosa_4.csv', 'w')
# for i in range(len(x)):
    # if z[i] >= 0:
        # f.write(str(x[i]) + " " +
                # str(y[i]) + " " +
                # str(z[i]) + " " + "\n")
        # hemi_x.append(x[i])
        # hemi_y.append(y[i])
        # hemi_z.append(z[i])
# f.close()

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter3D(x, y, z)
ax.set_xlabel("x")
ax.set_ylabel("y")
ax.set_zlabel("z")
ax.set_title("icosahedron(m)")
plt.show()

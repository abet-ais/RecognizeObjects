import csv
import numpy as np
import matplotlib.pyplot as plt
from sklearn.decomposition import PCA
from scipy.stats import multivariate_normal

in_path = "/home/ais/ais-project/catkin_ws/src/devel/abe/openpose_data/proceeding0228_1/"
name = "output"
out_path = in_path + name
input_file = out_path + "/rhand"

# ignore above outlier_x
# outlier_x = 0.18
outlier_x = 100
# ignore under outlier_z
# outlier_z = -0.45
outlier_z = -100
# outlier = 100

start_frame = 70
end_frame = 400

reliable_index = [9, 13, 14, 12, 11,  6, 16, 15,  5, 17, 10,  7, 18, 19,  8, 20,  1,  2,  0,  3,  4]

small_val = 0.00000001

# red: joint9, blue:joint13, yellow:joint14
color = ["red", "blue", "yellow"]

def getJointData(num, rhand, start, end):
    global outlier_x
    global outlier_z
    rhand_x = []
    rhand_y = []
    rhand_z = []

    for j in range(len(rhand[num])-7):
        if(float(rhand[num][j][1]) < outlier_x and float(rhand[num][j][3]) > outlier_z):
            rhand_x.append(float(rhand[num][j][1]))
            rhand_y.append(float(rhand[num][j][2]))
            rhand_z.append(float(rhand[num][j][3]))

    return rhand_x, rhand_y, rhand_z

def readCsv(start, end):
    global input_file
    global reliable_index
    global color

    filename = input_file
    rhand = []
    confidence_list = []

    for num in range(21):
        lst = []
        try:
            with open(filename+str(num)+".csv") as f:
                reader = csv.reader(f)
                for row in reader:
                    lst.append([row[0], row[1], row[2], row[3], row[4]]) 

                splited_lst = lst[start:end]
                rhand.append(splited_lst)
        except:
            print ("no file: %d" % (num))
            confidence_list.append(0)
            import traceback
            traceback.print_exc()
            continue

    rhand_x = []
    rhand_y = []
    rhand_z = []
    for i in range(3):
        tmp_x, tmp_y, tmp_z = getJointData(reliable_index[i], rhand, start, end)
        rhand_x.append(tmp_x)
        rhand_y.append(tmp_y)
        rhand_z.append(tmp_z)

    x_ave = []
    y_ave = []
    z_ave = []
    
    n = min([len(rhand_x[0]), len(rhand_x[1]), len(rhand_x[2])])
    for i in range(n):
        x_ave.append((rhand_x[0][i] + rhand_x[1][i] + rhand_x[2][i])/3)
        y_ave.append((rhand_y[0][i] + rhand_y[1][i] + rhand_y[2][i])/3)
        z_ave.append((rhand_z[0][i] + rhand_z[1][i] + rhand_z[2][i])/3)
        


    for i in range(3):
        lab = "joint"+str(reliable_index[i])
        plt.subplot(1,3,1)
        plt.title(name+"_x")
        plt.ylim(ymin=-0.4, ymax=0.6)
        plt.xlabel("frame (frame)")
        plt.ylabel("x joint movement (m)")
        # plt.scatter([j+start for j in range(len(x[i]))], x[i], c=color[i], label=lab)
        plt.plot([j+start for j in range(len(rhand_x[i]))], rhand_x[i], c=color[i], label=lab, linewidth=2, marker="o", markeredgewidth=0)
        plt.legend(bbox_to_anchor=(1, 1), loc='upper right', borderaxespad=0, fontsize=13)

        plt.subplot(1,3,2)
        plt.title(name+"_y")
        plt.ylim(ymin=-0.4, ymax=0.6)
        plt.xlabel("frame (frame)")
        plt.ylabel("y joint movement (m)")
        # plt.scatter([j+start for j in range(len(y[i]))], y[i], c=color[i], label=lab)
        plt.plot([j+start for j in range(len(rhand_y[i]))], rhand_y[i], c=color[i], label=lab, linewidth=2, marker="o", markeredgewidth=0)
        plt.legend(bbox_to_anchor=(1, 1), loc='upper right', borderaxespad=0, fontsize=13)

        plt.subplot(1,3,3)
        plt.title(name+"_z")
        plt.ylim(ymin=-0.4, ymax=0.6)
        plt.xlabel("frame (frame)")
        plt.ylabel("z joint movement (m)")
        # plt.scatter([j+start for j in range(len(z[i]))], z[i], c=color[i], label=lab)
        plt.plot([j+start for j in range(len(rhand_z[i]))], rhand_z[i], c=color[i], label=lab, linewidth=2, marker="o", markeredgewidth=0)
        plt.legend(bbox_to_anchor=(1, 1), loc='upper right', borderaxespad=0, fontsize=13)

        plt.savefig("/home/ais/ais-project/catkin_ws/src/devel/abe/openpose_data/proceeding0228_1/graph_feature/"+name+"_"+str(start)+"_"+str(end)+".png")
    plt.show()


def main():
    global start_frame
    global end_frame
    global chunk # for create histogram
    readCsv(0, end_frame-start_frame)

if __name__=='__main__':
    main()


import csv
import numpy as np
import matplotlib.pyplot as plt

# input_file = "/home/ais/ais-project/catkin_ws/src/devel/abe/openpose_data/hand_motion_0201_2_08_rhand_vertical_210_250_insert/rhand"
# output_file = "/home/ais/ais-project/catkin_ws/src/devel/abe/openpose_data/histogram/hist_0201_2_08_rhand_vertical_210_250_insert"
# in_path = "/home/ais/ais-project/catkin_ws/src/devel/abe/openpose_data/0203_proceeding/"
in_path = "/home/ais/ais-project/catkin_ws/src/devel/abe/openpose_data/proceeding0217/"

# name = "0130_3_08_rhand_250_450_insert"
# name = "0131_2_08_rhand_vertical_750_1025_insert"
# name = "0130_3_08_rhand_480_800_screw"
# name = "0131_2_08_rhand_vertical_480_645_screw"
# name = "0201_1_08_rhand_185_310_screw"
# name = "0201_2_08_rhand_vertical_330_421_screw"
# name = "0201_1_08_rhand_60_150_insert"
name = "output"
outlier = 0.3
# outlier = 100
# start = 40
# end = 60

out_path = in_path + name
flow_path = in_path+"flow/"

input_file = out_path + "/rhand"
output_file = in_path+"/hist/"+"/hist_"+ "rhand"
ave_output_file = in_path+"/hist/"+"/hist_"+ "rhand" +"_ave"


small_val = 0.00000001

def writeCsv(csv_list, num, start, end):
    global output_file
    if(num == 74):
        f = open(ave_output_file+"_"+str(start)+"_"+str(end)+".csv", "w")
        for i in range(len(csv_list)):
            csv_contents = str(csv_list[i]) + "\n"
            f.write(csv_contents)
        f.close()

        x_axis = []
        for k in range(-30, 40, 2):
            k = k*0.01
            x_axis.append(k)

        plt.subplot(1,3,1)
        plt.xlabel("bin")
        plt.title("position_histogram_x")
        plt.xlim(xmin=-0.3, xmax=0.4)
        plt.bar(x_axis, csv_list[0], width=0.02)

        plt.subplot(1,3,2)
        plt.xlabel("bin")
        plt.title("position_histogram_y")
        plt.xlim(xmin=-0.3, xmax=0.4)
        plt.bar(x_axis, csv_list[1], width=0.02)

        plt.subplot(1,3,3)
        plt.xlabel("bin")
        plt.title("position_histogram_z")
        plt.xlim(xmin=-0.3, xmax=0.4)
        plt.bar(x_axis, csv_list[2], width=0.02)

    else:
        f = open(output_file+str(num)+".csv", "w")
        for i in range(len(csv_list)):
            csv_contents = str(csv_list[i][0]) + ", " + str(csv_list[i][1]) + "\n"
            f.write(csv_contents)
        f.close()

def decideReliableData(conf_list):
    lst = np.array(conf_list)
    tmp = np.sort(lst)
    sorted_lst = tmp[-1::-1]
    tmp_index = np.argsort(lst)
    index = tmp_index[-1::-1]
    return index

def distinctData(num, rhand, start, end):
    global outlier
    # global start, end
    score = []
    no_data_count = []
    for i in range(len(rhand[num])):
        if(float(rhand[num][i][1]) < outlier):
            score.append(float(rhand[num][i][4]))
        else:
            no_data_count.append(1)

    try:
        score_ave = sum(score)/len(score)
        total_frame = end - start
        data_score = float(len(score))/total_frame
        confidence_factor = (8*score_ave + 2*data_score)/10

    except:
        data_score = score_ave = confidence_factor = 0

    return data_score, score_ave, confidence_factor

def extractFeature(num, rhand, start, end):
    global small_val
    global outlier

    x = []
    y = []
    z = []
    rhand_x = []
    rhand_y = []
    rhand_z = []

    for j in range(len(rhand[num])-1):
        if(float(rhand[num][j][1]) < outlier):
            rhand_x.append(float(rhand[num][j][1]))
            rhand_y.append(float(rhand[num][j][2]))
            rhand_z.append(float(rhand[num][j][3]))

    x.append(rhand_x)
    y.append(rhand_y)
    z.append(rhand_z)


    rhand = [rhand_x, rhand_y, rhand_z]
    rhand_hist = []
    
    for axis in range(len(rhand)):
        bin = [[] for i in range(35)]
        for j in range(len(rhand[axis])):
            index = 0
            for k in range(-30, 40, 2):
                k = k*0.01
                if(rhand[axis][j] > k and rhand[axis][j] < k+0.02):
                    bin[index].append(1)
                index = index + 1

        position_hist=[]
        for i in range(len(bin)):
            position_hist.append(len(bin[i]))

        mx = max(position_hist)
        normalized_position_hist = []
        for i in range(len(position_hist)):
            tmp_position_hist = position_hist[i]/float(mx)
            normalized_position_hist.append(tmp_position_hist)
    
        csv_list = []
        for i in range(len(normalized_position_hist)):
            tmp_list = [i, normalized_position_hist[i]]
            csv_list.append(tmp_list)

        # writeCsv(csv_list, num, start, end)
        rhand_hist.append(normalized_position_hist)

    return rhand_hist, x, y, z

def main(start, end):
    # global start, end
    global input_file
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
                # rhand.append(lst)
                rhand.append(splited_lst)
        except:
            print ("no file: %d" % (num))
            confidence_list.append(0)
            import traceback
            traceback.print_exc()
            continue

        try:
            data_score, score_ave, confidence_factor = distinctData(num, rhand, start, end)
            confidence_list.append(confidence_factor)
        except:
            print "distinctData error"
            import traceback
            traceback.print_exc()
            continue

    reliable_index = decideReliableData(confidence_list)

    # plt.figure(figsize=(15,15))

    hist = []
    x_lst = []
    y_lst = []
    z_lst = []
    for i in range(3):
        tmp_hist, tmp_x, tmp_y, tmp_z = extractFeature(reliable_index[i], rhand, start, end)
        hist.append(tmp_hist)
        x_lst.append(tmp_x)
        y_lst.append(tmp_y)
        z_lst.append(tmp_z)
        # print hist[0][i]

    hist_ave = [[],[],[]]
    for j in range(len(hist_ave)):
        for i in range(len(hist[0][0])):
            ave = (4*hist[0][j][i] + 3*hist[1][j][i] + 3*hist[2][j][i])/10
            # ave = hist[0][j][i]
            hist_ave[j].append(ave)

    histogram = [[],[],[]]
    for j in range(len(histogram)):
        for i in range(len(hist_ave[j])):
            tmp = float(hist_ave[j][i] / max(hist_ave[j]))
            histogram[j].append(tmp)

    writeCsv(histogram, 74, start, end)

    color = ["red", "green", "blue"]

    # for i in range(3):
    #     lab = "joint"+str(reliable_index[i])
    #     plt.subplot(2,3,1)
    #     plt.title(name+"_x")
    #     plt.xlim(xmin=0, xmax=220)
    #     plt.ylim(ymin=-0.3, ymax=0.6)
    #     plt.xlabel("frame (frame)")
    #     plt.ylabel("x joint movement (m)")
    #     # plt.scatter([j+start for j in range(len(x[i]))], x[i], c=color[i], label=lab)
        # plt.plot([j+start for j in range(len(x_lst[i]))], x_lst[i], c=color[i], label=lab)
    #     plt.legend(bbox_to_anchor=(1, 1), loc='upper right', borderaxespad=0, fontsize=13)

    #     plt.subplot(2,3,2)
    #     plt.title(name+"_y")
    #     plt.xlim(xmin=0, xmax=220)
    #     plt.ylim(ymin=-0.3, ymax=0.6)
    #     plt.xlabel("frame (frame)")
    #     plt.ylabel("y joint movement (m)")
    #     # plt.scatter([j+start for j in range(len(y[i]))], y[i], c=color[i], label=lab)
    #     plt.plot([j+start for j in range(len(y_lst[i]))], y_lst[i], c=color[i], label=lab)
    #     plt.legend(bbox_to_anchor=(1, 1), loc='upper right', borderaxespad=0, fontsize=13)

    #     plt.subplot(2,3,3)
    #     plt.title(name+"_z")
    #     plt.xlim(xmin=0, xmax=220)
    #     plt.ylim(ymin=-0.3, ymax=0.6)
    #     plt.xlabel("frame (frame)")
    #     plt.ylabel("z joint movement (m)")
    #     # plt.scatter([j+start for j in range(len(z[i]))], z[i], c=color[i], label=lab)
    #     plt.plot([j+start for j in range(len(z_lst[i]))], z_lst[i], c=color[i], label=lab)
    #     plt.legend(bbox_to_anchor=(1, 1), loc='upper right', borderaxespad=0, fontsize=13)

    plt.savefig("/home/abe/ais-project/catkin_ws/src/devel/abe/openpose_data/proceeding0217/graph_feature_posi2/"+name+"_"+str(start)+"_"+str(end)+".png")
    # plt.show()
    # plt.pause(.01)


if __name__=='__main__':
    for i in range(0,215,2):
        print i
        plt.figure(figsize=(15,5))
        main(i, i+10)
    # main(0,225)


import csv
import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import KMeans

in_path = "/home/ais/ais-project/catkin_ws/src/devel/abe/openpose_data/0203_proceeding/"
name = "output_0203_2"
input_file = in_path + name

matching_threshold = [80, 90]
outlier = 10
data_size = 1527
fill_frame = []

def readLinemodData():
    global input_file
    filename = input_file + "/linemod_info"
    data = []

    for num in range(2):
        lst = []
        try:
            with open(filename+str(num)+".csv") as f:
                reader = csv.reader(f)
                for row in reader:
                    lst.append([int(row[0]), float(row[1]), float(row[2]), float(row[6])])
                data.append(lst)
        except:
            print ("no file: %d" % (num))
            import traceback
            traceback.print_exc()
            continue
    
    linemod_frame = [[],[]]
    u = [[],[]]
    v = [[],[]]
    match = [[],[]]

    for i in range(len(data)):
        for j in range(len(data[i])):
            linemod_frame[i].append(data[i][j][0])
            u[i].append(data[i][j][1])
            v[i].append(data[i][j][2])
            match[i].append(data[i][j][3])

    linemod_data = [linemod_frame, u, v, match]
    return linemod_data

def readOpenposeData():
    global input_file
    filename = input_file + "/rhand"
    rhand = []
    confidence_list = []
    for i in range(21):
        lst = []
        try:
            with open(filename+str(i)+".csv") as f:
                reader = csv.reader(f)
                for row in reader:
                    lst.append([int(row[0]), float(row[1]), float(row[2]), float(row[3]), float(row[4])])
                rhand.append(lst)
        except:
            print ("no file: %d" % (i))
            confidence_list.append(0)
            import traceback
            traceback.print_exc()
            continue

    op_frame = [[] for i in range(21)]
    op_x = [[] for i in range(21)]
    op_y = [[] for i in range(21)]
    op_z = [[] for i in range(21)]
    op_score = [[] for i in range(21)]

    for i in range(len(op_frame)):
        for j in range(len(rhand[0])):
            op_frame[i].append(rhand[0][j][0])
            op_x[i].append(rhand[0][j][1])
            op_y[i].append(rhand[0][j][2])
            op_z[i].append(rhand[0][j][3])
            op_score[i].append(rhand[0][j][4])

    openpose_data = [op_frame, op_x, op_y, op_z, op_score]
    return openpose_data

def getFlowData(openpose_data):
    global outlier
    flow_norm = [[] for i in range(21)]
    flow_frame = [[] for i in range(21)]

    for i in range(len(openpose_data[0])):
        for j in range(len(openpose_data[0][i])-1): 
            # if(float(openpose_data[1][i][j+1]) < outlier and float(openpose_data[1][i][j]) < outlier):
            if(1 == 1):
                dx = openpose_data[1][i][j+1] - openpose_data[1][i][j]
                dy = openpose_data[2][i][j+1] - openpose_data[2][i][j]
                dz = openpose_data[3][i][j+1] - openpose_data[3][i][j]
                vec = np.array([dx, dy, dz])
                norm = np.linalg.norm(vec)
                flow_norm[i].append(norm)
                flow_frame[i].append(openpose_data[0][i][j])

    flow = [flow_frame, flow_norm]
    return flow

def inspectFlowMovement(flow):
    flow_norm = [[] for i in range(len(flow[0]))]
    flow_frame = [[] for i in range(len(flow[0]))]
    for i in range(len(flow[0])):
        for j in range(len(flow[0][i])):
            if(flow[1][i][j] < 0.2 and flow[1][i][j] > 0.002):
                flow_norm[i].append(flow[0][i][j])
                flow_frame[i].append(flow[1][i][j])

    return flow_frame, flow_norm

def inspectLinemodData(linemod_data):
    linemod_nondetect_frame = [[], []]
    linemod_detect_frame = [[], []]
    linemod_u = [[],[]]
    linemod_v = [[],[]]
    linemod_match = [[],[]]

    frame = linemod_data[0]
    u = linemod_data[1]
    v = linemod_data[2]
    match = linemod_data[3]
    # 0: 4parts, 1: lgear

    for i in range(len(linemod_nondetect_frame)):
        for j in range(len(frame[0])-2):
            du = u[i][j+2] - u[i][j]
            dv = v[i][j+2] - v[i][j]
            linemod_vec = np.sqrt(du**2 + dv**2)
            if(match[i][j] == 0 or linemod_vec > 20):
                linemod_nondetect_frame[i].append(frame[0][j])
            else:
                linemod_detect_frame[i].append(frame[0][j])
                linemod_u[i].append(u[i][j])
                linemod_v[i].append(v[i][j])
                linemod_match[i].append(match[i][j])

    return linemod_nondetect_frame, linemod_detect_frame, linemod_u, linemod_v, linemod_match

def getMeaningfulMotion(frame):
    num = len(frame[0])
    # nonparts4: 0, nonlgear: 1, parts4: 2, lgear: 3, flow: 5

    parts4_non_detect = []
    parts4_detect = []
    lgear_non_detect = []
    lgear_detect = []
    move = []

    for i in range(len(frame[0])):
        if(frame[0][i] != 0):
            parts4_non_detect.append(1)
        else:
            parts4_non_detect.append(0)

    for i in range(len(frame[0])):
        if(frame[2][i] != 0):
            parts4_detect.append(1)
        else:
            parts4_detect.append(0)

    for i in range(len(frame[0])):
        if(frame[1][i] != 0):
            lgear_non_detect.append(1)
        else:
            lgear_non_detect.append(0)

    for i in range(len(frame[0])):
        if(frame[3][i] != 0):
            lgear_detect.append(1)
        else:
            lgear_detect.append(0)

    for i in range(len(frame[0])):
        if(frame[4][i] != 0):
            move.append(1)
        else:
            move.append(0)

    binary_feature = []
    for i in range(len(frame[0])):
        tmp = str(lgear_detect[i])+str(lgear_non_detect[i])+str(parts4_detect[i])+str(parts4_non_detect[i])+str(move[i])
        binary_feature.append(tmp)

    return parts4_non_detect, parts4_detect, lgear_non_detect, lgear_detect, binary_feature

def prepareFrameData(flow_frame, flow_norm, nondetect, detect, u, v, match):
    global data_size
    # add end frame for recursive call
    for i in range(len(flow_frame)):
        flow_frame[i].append(data_size)
    for i in range(len(nondetect)):
        nondetect[i].append(data_size)
    for i in range(len(detect)):
        detect[i].append(data_size)
        
    ori_lst = [nondetect[0], nondetect[1], detect[0], detect[1], flow_frame[7]]
    ori_lst_buf = ori_lst

    for j in range(len(ori_lst)):
        fillList(ori_lst[j], 0)
    
    frame = fill_frame
    return frame

def fillList(lst, n):
    global data_size
    pre_end = len(lst)
    for i in range(n, len(lst)):
        if(lst[i] != i):
            lst.insert(i, 0)
    if(len(lst) <= data_size):
        fillList(lst, pre_end)
    else:
        fill_frame.append(lst)

def getEachMotionFrame(binary_feature):
    meaningful_frame = []
    for i in range(len(binary_feature)):
        if(binary_feature[i] == "01011"):
            meaningful_frame.append(i)
    binary_meaningful_frame = []
    return meaningful_frame

def clusterData(meaningful_frame):
    lst = [[] for i in range(len(meaningful_frame))]
    for i in range(len(meaningful_frame)):
        lst[i].append(meaningful_frame[i])

    index = []

    for i in range(len(meaningful_frame)):
        index.append(i)

    features = np.array(lst)
    kmeans_model = KMeans(n_clusters=9, max_iter=1000, n_init=100).fit(features)
    labels = kmeans_model.labels_

    return labels

def splitFrame(frame, label):
    print len(frame), len(label)
    
    start_frame = []
    end_frame = []
    for i in range(len(frame)-1):
        if(label[i] != label[i+1]):
            end_frame.append(frame[i])
            start_frame.append(frame[i+1])

    start_frame.insert(0,frame[0])
    end_frame.append(frame[len(frame)-1])
    print start_frame
    print end_frame
    return start_frame, end_frame

def main():
    linemod_data = readLinemodData()
    openpose_data = readOpenposeData()
    flow = getFlowData(openpose_data)
    flow_norm, flow_frame = inspectFlowMovement(flow)
    nondetect_frame, detect_frame, u, v, match = inspectLinemodData(linemod_data)
    frame = prepareFrameData(flow_frame, flow_norm, nondetect_frame, detect_frame, u, v, match)
    parts4_non_detect, parts4_detect, lgear_non_detect, lgear_detect, binary_feature = getMeaningfulMotion(frame)    
    meaningful_frame = getEachMotionFrame(binary_feature)
    labels = clusterData(meaningful_frame)
    label_lst = list(labels)
    start_frame, end_frame = splitFrame(meaningful_frame, label_lst)

    color = ["red", "yellow", "blue", "pink", "black", "orange", "gray"]
    plt.figure(figsize=(15,15))
    plt.subplot(2,1,1)
    plt.title("meaningful_frame")
    plt.scatter(meaningful_frame, [1 for i in range(len(meaningful_frame))])
    plt.subplot(2,1,2)
    plt.title("clustering")
    plt.scatter(meaningful_frame, label_lst)

    plt.show()

    # plt.figure(figsize=(15,15))
    # plt.subplot(2,2,1)
    # plt.title("parts4_non_detect")
    # plt.bar([i for i in range(len(parts4_non_detect))], parts4_non_detect)
    # plt.subplot(2,2,2)
    # plt.title("parts4_detect")
    # plt.bar([i for i in range(len(parts4_detect))], parts4_detect)
    # plt.subplot(2,2,3)
    # plt.title("lgear_non_detect")
    # plt.bar([i for i in range(len(lgear_non_detect))], lgear_non_detect)
    # plt.subplot(2,2,4)
    # plt.title("lgear_detect")
    # plt.bar([i for i in range(len(lgear_detect))], lgear_detect)
    # plt.show()

if __name__=='__main__':
    main()


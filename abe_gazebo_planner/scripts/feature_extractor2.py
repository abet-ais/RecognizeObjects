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
chunk = 30 # for create histogram

sigmoid = [90, 80, 80]
sigmoid_a = 5

# for proceeding0228_1
# screw, insert, other
sigma_x = [0.33,0.55,0.32]
sigma_y = [1, 0.5, 0.4, 0.4, 0.4]

# screw, insert, other
correct_linemod_state = np.array([[0, 0, 1], [0, 0, 0], [1, 1, 1], [1, 0, 1], [0, 0, 1]])
correct_flow_hist = np.array([[0.07203389830508473, 0.14406779661016947, 0.5550847457627118, 0.5762711864406779, 0.4576271186440678, 0.28813559322033894, 0.4576271186440678, 0.21610169491525422, 0.41101694915254233, 0.3855932203389831, 0.07203389830508473, 0.28813559322033894, 0.5296610169491525, 0.8220338983050847, 1.0, 0.5550847457627118, 0.28813559322033894, 0.0], [0.0303030303030303, 0.0, 0.0, 0.1414141414141414, 0.19528619528619526, 0.0909090909090909, 0.24915824915824913, 0.569023569023569, 1.0, 0.5925925925925927, 0.468013468013468, 0.13468013468013465, 0.2154882154882155, 0.0909090909090909, 0.09764309764309763, 0.06734006734006732, 0.0606060606060606, 0.09764309764309763],[0.5 for i in range(18)]])

joints_num = 21
reliable_index = [9, 13, 14, 12, 11,  6, 16, 15,  5, 17, 10,  7, 18, 19,  8, 20,  1,  2,  0,  3,  4]
base_vec = np.array([0, 1, 0])
pca_frame = 7

x = []
y = []
z = []
small_val = 0.00000001

def writeHistogramCsv(flow_histogram, end):
    f = open(in_path + "./histogram/histogram" + str(start_frame+end) + ".csv", "w")
    for i in range(len(flow_histogram)):
        f.write(str(flow_histogram[i]) + "\n")
    f.close()

def writeProbabilityCsv(probability_str):
    f = open(in_path + "./probability.csv", "a")
    f.write(probability_str + "\n")
    f.close()

def computeGaussian(correct, current, D, sigma_cof):
    sigma = sigma_cof*np.eye(D)
    x = current
    mean = correct
    N = multivariate_normal.pdf(x, mean=mean, cov=sigma)
    return N

def computeGaussianNaiveBayes(flow_histogram, linemod_data, start, end):
    global start_frame
    global correct_linemod_state
    global correct_flow_hist
    global sigma_x
    global sigma_y

    writeHistogramCsv(flow_histogram, end)

    x_c = np.zeros(3)
    y_c = np.zeros(3)
    current_linemod_state = np.array(linemod_data)
    current_flow_hist = np.array(flow_histogram)

    before_y_c = 0
    for i in range(3):
        x_c[i] = computeGaussian(correct_flow_hist[i], current_flow_hist, correct_flow_hist[i].shape[0], sigma_x[i])
    for i in range(5):
        if i < 2:
            y_c[i] = computeGaussian(correct_linemod_state[i], current_linemod_state, correct_linemod_state.shape[1], sigma_y[i])
        elif i > 2:
            y_c[2] = computeGaussian(correct_linemod_state[i], current_linemod_state, correct_linemod_state.shape[1], sigma_y[i])
            if y_c[2] < before_y_c:
                y_c[2] = before_y_c
        else:
            y_c[2] = computeGaussian(correct_linemod_state[i], current_linemod_state, correct_linemod_state.shape[1], sigma_y[i])
            before_y_c = y_c[2]

    x_c = x_c / x_c.sum()
    y_c = y_c / y_c.sum()
    p_c = (1/3.0)
    p_x_y = ((x_c * y_c)* p_c).sum()
    p_c_xy = (p_c * (x_c * y_c)) / p_x_y

    probability_str = str(end+start_frame) + ", " + str(p_c_xy[0]) + ", " + str(p_c_xy[1]) + ", " + str(p_c_xy[2])
    # print probability_str
    writeProbabilityCsv(probability_str)

    return end+start_frame, x_c, y_c, p_c_xy

def extractFeature(num, rhand, start, end):
    global small_val
    global outlier_x
    global outlier_z
    global base_vec
    global pca_frame

    rad = np.pi/180
    feature = []
    pc_vec_lst = []    

    for j in range(len(rhand[num])-pca_frame):
        flow_chunk = []
        for k in range(pca_frame):
            if(float(rhand[num][j+k][1]) < outlier_x and float(rhand[num][j][3]) > outlier_z):
                tmp_lst = [rhand[num][j+k][1], rhand[num][j+k][2], rhand[num][j+k][3]]
                flow_chunk.append(tmp_lst)

        if len(flow_chunk) != 0:
            pca_data = np.array(flow_chunk)
            pca = PCA()
            pca.fit(pca_data)
            pc_vec = pca.components_[0]
            pc_vec_lst.append(pc_vec)
            pc_len = np.linalg.norm(pc_vec)
        else:
            continue

        feature.append(pc_vec)

    cross = []
    cross_length = []
    for i in range(len(feature)-1):
        tmp_cross = np.cross(feature[i], feature[i+1])
        cross.append(tmp_cross)
        tmp_cross_length = np.linalg.norm(tmp_cross)
        cross_length.append(tmp_cross_length)

    deg_base_cross = []
    for i in range(len(cross)):
        base_vec_len = np.linalg.norm(base_vec)
        base_cross_dot = np.dot(base_vec, cross[i])
        if(cross_length[i] > small_val):
            theta = np.arccos(base_cross_dot/(base_vec_len*cross_length[i]))
            deg_base_cross.append(theta)
        else:
            continue

    bin = [[] for i in range(18)]
    bin_value = 10
    for i in range(len(deg_base_cross)):
        for j in range(0, 180, bin_value):
            index = j/bin_value
            if(deg_base_cross[i] > j*rad and deg_base_cross[i] < (j+bin_value)*rad):
                bin[index].append(1)

    deg_hist=[]
    for i in range(len(bin)):
       deg_hist.append(len(bin[i]))

    mx = max(deg_hist)
    normalized_deg_hist = []

    for i in range(len(deg_hist)):
        try:
            tmp_deg_hist = deg_hist[i]/float(mx)
            normalized_deg_hist.append(tmp_deg_hist)
        except:
            print("miss devide")
    
    csv_list = []
    for i in range(len(normalized_deg_hist)):
        tmp_list = [i, normalized_deg_hist[i]]
        csv_list.append(tmp_list)

    return normalized_deg_hist

def createHistogram(start, end, linemod_data):
    global input_file
    global reliable_index
    global joints_num

    filename = input_file
    rhand = []

    for num in range(joints_num):
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

    hist = []
    for i in range(3):
        tmp_hist = extractFeature(reliable_index[i], rhand, start, end)
        hist.append(tmp_hist)
    
    # histogram that average of top3 joints
    hist_ave = []
    for i in range(len(hist[0])):
        ave = (3*hist[0][i] + 3*hist[1][i] + 3*hist[2][i])/9
        hist_ave.append(ave)

    # normalize histogram
    flow_histogram = []
    for i in range(len(hist_ave)):
        tmp = float(hist_ave[i] / max(hist_ave))
        flow_histogram.append(tmp)

    # comupute likelihood of each motion
    frame, x_c, y_c, p_c_xy = computeGaussianNaiveBayes(flow_histogram, linemod_data, start, end)

    return frame, x_c, y_c, p_c_xy

def readLinemod(start, end):
    global out_path
    global sigmoid
    global sigmoid_a
    filename = out_path + "/linemod_info"
    data = []

    # 1: shaft, 2: base, 3: lgear2
    for num in range(3):
        lst = []
        try:
            with open(filename+str(num)+".csv") as f:
                reader = csv.reader(f)
                for row in reader:
                    lst.append([int(row[0]), float(row[6])])
                splited_lst = lst[start:end]
                data.append(splited_lst)
        except:
            print ("no file: %d" % (num))
            import traceback
            traceback.print_exc()
            continue
    
    # shat, base, lgear
    linemod_frame = [[], [], []]
    match = [[], [], []]
    sig_match = [[], [], []]

    for i in range(len(data)):
        for j in range(len(data[i])):
            linemod_frame[i].append(data[i][j][0])
            match[i].append(data[i][j][1])
            sig = np.exp((-sigmoid_a)*(data[i][j][1] - sigmoid[i]))
            tmp_sig_match = 1 / (1+sig)
            sig_match[i].append(tmp_sig_match)

    shaft_sig_match = sum(sig_match[0]) / len(sig_match[0])
    base_sig_match = sum(sig_match[1]) / len(sig_match[1])
    gear_sig_match = sum(sig_match[2]) / len(sig_match[2])
    linemod_data = [shaft_sig_match, base_sig_match, gear_sig_match]
    return linemod_data

def main():
    global start_frame
    global end_frame
    global chunk # for create histogram

    frame_lst = []
    x_c_screw = []
    x_c_insert = []
    x_c_other = []
    y_c_screw = []
    y_c_insert = []
    y_c_other = []
    p_c_xy_screw = []
    p_c_xy_insert = []
    p_c_xy_other = []

    for i in range(0, end_frame-(start_frame+chunk), 1):
        linemod_data = readLinemod(i, i+chunk)
        frame, x_c, y_c, p_c_xy = createHistogram(i, i+chunk, linemod_data)

        frame_lst.append(frame)
        p_c_xy_screw.append(p_c_xy[0])
        p_c_xy_insert.append(p_c_xy[1])
        p_c_xy_other.append(p_c_xy[2])
        x_c_screw.append(x_c[0])
        x_c_insert.append(x_c[1])
        x_c_other.append(x_c[2])
        y_c_screw.append(y_c[0])
        y_c_insert.append(y_c[1])
        y_c_other.append(y_c[2])

    # visualize motion estimate result
    color = ["red", "blue", "yellow"]
    plt.subplot(3,1,1)
    plt.rcParams["font.size"] = 16
    plt.title("Motion Lkelihood estimated by Combined Feature: p(c|x,y)")
    plt.xlabel("frame")
    plt.ylabel("likelihood")
    plt.xlim(xmin=100, xmax=400)
    plt.ylim(ymin=0, ymax=1)
    plt.scatter(frame_lst, p_c_xy_screw, c=color[0], s=80, label="screw")
    plt.scatter(frame_lst, p_c_xy_insert, c=color[1], s=80, label="insert")
    plt.scatter(frame_lst, p_c_xy_other, c=color[2], s=80, label="other")
    plt.legend(bbox_to_anchor=(1, 1), loc='upper right', borderaxespad=0, fontsize=13)
    plt.grid()

    plt.subplot(3,1,2)
    plt.title("Motion Lkelihood estimated by Hand Motion Feature: p(x|c)")
    plt.xlabel("frame")
    plt.ylabel("likelihood")
    plt.xlim(xmin=100, xmax=400)
    plt.ylim(ymin=0, ymax=1)
    plt.scatter(frame_lst, x_c_screw, c=color[0], s=80, label="screw")
    plt.scatter(frame_lst, x_c_insert, c=color[1], s=80, label="insert")
    plt.scatter(frame_lst, x_c_other, c=color[2], s=80, label="other")
    plt.legend(bbox_to_anchor=(1, 1), loc='upper right', borderaxespad=0, fontsize=13)
    plt.grid()

    plt.subplot(3,1,3)
    plt.title("Motion Lkelihood estimated by Object Recognition State: p(y|c)")
    plt.xlabel("frame")
    plt.ylabel("likelihood")
    plt.xlim(xmin=100, xmax=400)
    plt.ylim(ymin=0, ymax=1)
    plt.scatter(frame_lst, y_c_screw, c=color[0], s=80, label="screw")
    plt.scatter(frame_lst, y_c_insert, c=color[1], s=80, label="insert")
    plt.scatter(frame_lst, y_c_other, c=color[2], s=80, label="other")
    plt.legend(bbox_to_anchor=(1, 1), loc='upper right', borderaxespad=0, fontsize=13)
    plt.grid()

    plt.show()

if __name__=='__main__':
    main()


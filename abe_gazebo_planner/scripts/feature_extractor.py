import csv
import numpy as np

def main():
    frame = []
    x = []
    y = []
    z = []

    with open("/home/ais/ais-project/catkin_ws/src/devel/abe/abe_sandbox/hand_motion_insert_0131_2_730_1100_vertical.csv") as f:
        reader = csv.reader(f)
        for row in reader:
            frame.append(row[0])
            x.append(row[1])
            y.append(row[2])
            z.append(row[3])

    base_vec = np.array([1, 0, 0])

    feature = []
    for i in range(len(frame)-1):
        diff_x = float(x[i+1]) - float(x[i])
        diff_y = float(y[i+1]) - float(y[i])
        diff_z = float(z[i+1]) - float(z[i])
        vec = np.array([diff_x, diff_y, diff_z])
        length = np.linalg.norm(vec)
        normalized_vec = vec/length
        feature.append(normalized_vec)

    cross = []
    cross_length = []
    for i in range(len(feature)-1):
        tmp_cross = np.cross(feature[i], feature[i+1])
        cross.append(tmp_cross)
        tmp_cross_length = np.linalg.norm(tmp_cross)
        cross_length.append(tmp_cross_length)
        # print tmp_cross_length

    deg_base_cross = []
    for i in range(len(cross)):
        base_vec_len = np.linalg.norm(base_vec)
        base_cross_dot = np.dot(base_vec, cross[i])
        theta = np.arccos(base_cross_dot/(base_vec_len*cross_length[i]))
        deg_base_cross.append(theta)

    rad = np.pi/180
    bin10=[]
    bin20=[]
    bin30=[]
    bin40=[]
    bin50=[]
    bin60=[]
    bin70=[]
    bin80=[]
    bin90=[] 
    bin100=[]
    bin110=[]
    bin120=[]
    bin130=[]
    bin140=[]
    bin150=[]
    bin160=[]
    bin170=[]
    bin180=[]

    for i in range(len(deg_base_cross)):
        if(deg_base_cross[i] > 0*rad and deg_base_cross[i] < 10*rad):
            bin10.append(1)
        elif(deg_base_cross[i] > 10*rad and deg_base_cross[i] < 20*rad):
            bin20.append(1)
        elif(deg_base_cross[i] > 20*rad and deg_base_cross[i] < 30*rad):
            bin30.append(1)
        elif(deg_base_cross[i] > 30*rad and deg_base_cross[i] < 40*rad):
            bin40.append(1)
        elif(deg_base_cross[i] > 40*rad and deg_base_cross[i] < 50*rad):
            bin50.append(1)
        elif(deg_base_cross[i] > 50*rad and deg_base_cross[i] < 60*rad):
            bin60.append(1)
        elif(deg_base_cross[i] > 60*rad and deg_base_cross[i] < 70*rad):
            bin70.append(1)
        elif(deg_base_cross[i] > 70*rad and deg_base_cross[i] < 80*rad):
            bin80.append(1)
        elif(deg_base_cross[i] > 80*rad and deg_base_cross[i] < 90*rad):
            bin90.append(1)
        elif(deg_base_cross[i] > 90*rad and deg_base_cross[i] < 100*rad):
            bin100.append(1)
        elif(deg_base_cross[i] > 100*rad and deg_base_cross[i] < 110*rad):
            bin110.append(1)
        elif(deg_base_cross[i] > 110*rad and deg_base_cross[i] < 120*rad):
            bin120.append(1)
        elif(deg_base_cross[i] > 120*rad and deg_base_cross[i] < 130*rad):
            bin130.append(1)
        elif(deg_base_cross[i] > 130*rad and deg_base_cross[i] < 140*rad):
            bin140.append(1)
        elif(deg_base_cross[i] > 140*rad and deg_base_cross[i] < 150*rad):
            bin150.append(1)
        elif(deg_base_cross[i] > 150*rad and deg_base_cross[i] < 160*rad):
            bin160.append(1)
        elif(deg_base_cross[i] > 160*rad and deg_base_cross[i] < 170*rad):
            bin170.append(1)
        elif(deg_base_cross[i] > 170*rad and deg_base_cross[i] < 180*rad):
            bin180.append(1)

    deg_hist = [len(bin10), len(bin20), len(bin30), len(bin40), len(bin50), len(bin60), len(bin70), len(bin80), len(bin90), len(bin100), len(bin110), len(bin120), len(bin130), len(bin140), len(bin150), len(bin160), len(bin170), len(bin180)]
    print deg_hist

    csv_list = []
    # for i in range(len(cross_length)):
        # tmp_list = [i, cross_length[i]]
        # csv_list.append(tmp_list)

    # for i in range(len(deg_base_cross)):
        # tmp_list = [i, deg_base_cross[i]]
        # csv_list.append(tmp_list)

    for i in range(len(deg_hist)):
        tmp_list = [i, deg_hist[i]]
        csv_list.append(tmp_list)

    f = open("./hist_insert_0131_2_730_1100_vertical.csv", "w")
    for i in range(len(csv_list)):
        # print csv_list[i]
        f.write(str(csv_list[i][0]) + ", " +
                str(csv_list[i][1]) + "\n")
    
    f.close()

if __name__=='__main__':
    main()


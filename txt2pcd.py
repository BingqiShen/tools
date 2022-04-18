import math
import os
def txt2pcd(filename):
    xlist = []
    ylist = []
    zlist = []
    intensitylist = []

    # read txt file
    txt_file = filename + '.txt'
    with open(txt_file, 'r') as file_to_read:
        while True:
            lines = file_to_read.readline()
            if not lines:
                break
                pass
            x, y, z, intensity = [(float(i)) for i in lines.split(' ')]
            # x, y, z = [(float(i)) for i in lines.split(' ')]
            if z > 2.1:
                continue
            xlist.append(x)
            ylist.append(y)
            zlist.append(z)
            intensitylist.append(intensity)
 
    # save pcd file
    savefilename = filename + '.pcd'
    if not os.path.exists(savefilename):
        f = open(savefilename, 'w')
        f.close()
    with open(savefilename, 'w') as file_to_write:
        file_to_write.writelines("# .PCD v0.7 - Point Cloud Data file format\n")
        file_to_write.writelines("VERSION 0.7\n")
        file_to_write.writelines("FIELDS x y z intensity\n")
        file_to_write.writelines("SIZE 4 4 4 4\n")
        file_to_write.writelines("TYPE F F F F\n")
        file_to_write.writelines("COUNT 1 1 1 1\n")
        file_to_write.writelines("WIDTH " + str(len(xlist)) + "\n")
        file_to_write.writelines("HEIGHT 1\n")
        file_to_write.writelines("VIEWPOINT 0 0 0 1 0 0 0\n")
        file_to_write.writelines("POINTS " + str(len(xlist)) + "\n")
        file_to_write.writelines("DATA ascii\n")
        for i in range(len(xlist)):
            file_to_write.writelines(str(xlist[i]) + " " + str(ylist[i]) + " " + str(zlist[i]) + " " + str(intensitylist[i]) + "\n")


txt2pcd('/home/nuc/visualization_cloud/new_pcd/global')
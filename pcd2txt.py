import os
# 定义一个三维点类
class Point(object):
    def __init__(self,x,y,z,intensity):
        self.x = x
        self.y = y
        self.z = z
        self.intensity = intensity

points = []
filename = '/home/nuc/visualization_cloud/cloudGlobal'

# 读取pcd文件,从pcd的第12行开始是三维点
with open(filename+'.pcd') as f:
    for line in  f.readlines()[11:len(f.readlines())-1]:
        strs = line.split(' ')
        points.append(Point(strs[0], strs[1], strs[2], strs[3].strip()))
# strip()是用来去除换行符

# 把三维点写入txt文件
fw = open(filename+'.txt','w')
for i in range(len(points)):
     linev = points[i].x + " " + points[i].y + " " + points[i].z + " " + points[i].intensity + "\n"
     fw.writelines(linev)
fw.close()

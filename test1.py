
import numpy as np
import open3d as o3d
import json
import pandas as pd
from time import *

begin_time = time()

x1 = pd.read_table('to_yunqi_offline_data/info_room_fpcd/7_room_1.txt', sep="\t", header=None)

num1 = int(x1[0][0]) # num is the number of points

feature1 = np.zeros([20,20], dtype = int)
feature = np.zeros(20,dtype = int)

for i in range(1,num1+1):
    temp = x1[0][i].split(' ') # each point data, for each row
    y = int(temp[6]) # the instance label
    if y % 10000 == 9999 or y % 10000 == 0:
        continue
    feature1[int(y/10000)][y%10000] = 1

for i in range(1,20):
    feature[i] = np.linalg.norm(feature1[i],ord=1)

print(feature)





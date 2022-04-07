
import numpy as np
import pandas as pd
from time import *

begin_time = time()

x1 = pd.read_table('to_yunqi_offline_data/ipark_room_fpcd/3_room_0.txt', sep="\t", header=None)
x2 = pd.read_table('to_yunqi_offline_data/ipark_room_fpcd/1_room_0.txt', sep="\t", header=None)
num1 = int(x1[0][0]) # num is the number of points
num2 = int(x2[0][0]) # num is the number of points

feature1 = np.zeros([20,20], dtype = int)
feature2 = np.zeros([20,20], dtype = int)

real_feature1 = np.zeros(20, dtype = int)
real_feature2 = np.zeros(20, dtype = int)

for i in range(1,num1+1):
    temp = x1[0][i].split(' ') # each point data, for each row
    y = int(temp[6]) # the instance label
    if y % 10000 >= 20 or y % 10000 <= 0:
        continue
    feature1[int(y/10000)][y%10000] = 1

for i in range(1,20):
    real_feature1[i] = np.linalg.norm(feature1[i],ord=1)

for i in range(1,num2+1):
    temp = x2[0][i].split(' ') # each point data, for each row
    y = int(temp[6]) # the instance label
    if y % 10000 >= 20 or y % 10000 <= 0:
        continue
    feature2[int(y/10000)][y%10000] = 1

for i in range(1,20):
    real_feature2[i] = np.linalg.norm(feature2[i],ord=1)

end_time = time()

print(end_time - begin_time)

h1 = real_feature1/np.linalg.norm(real_feature1)
h2 = real_feature2/np.linalg.norm(real_feature2)

print(real_feature1)
print(real_feature2)
print(np.matmul(h1.T,h2))



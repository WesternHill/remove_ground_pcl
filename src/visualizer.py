import numpy as np
import matplotlib.pyplot as plt

data_set = np.loadtxt(
    fname="./build/FpsVsPointnum.csv", # File to read
    dtype="float", # Value type to read
    delimiter=",", # Csv has delimiter ","
)

for data in data_set:
    plt.scatter(data[0], 1000/data[1] ,s=0.2) # expect column 1.points, 2.fps(1000/(milliseconds to segmentation))

plt.title("Fps vs Point_num")
plt.xlabel("Point num")
plt.ylabel("fps")
plt.grid()

plt.savefig("./FpsVsPointnum.png")

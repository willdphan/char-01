import math
import matplotlib.pyplot as plt
import numpy as np

# constant based on lidar resolution
LIDAR_RESOLUTION = 240
# lidar resolution divided by 4 to simplify the visualization
VISUALIZATION_RESOLUTION = 240


def GetDataFromArduino(line):
    # [:-3] get rid of end of line sign and additional comma separator that is sent from arduino
    data = line[:-3]
    print(data)
    d_list = data.split(",")
    return d_list


def GenerateLinePositions(numberOfLines):
    angle = 360/numberOfLines
    lines = []
    for x in range(numberOfLines):
        lines.append([300 * math.cos((x+1)*angle/180 * math.pi),
                     300 * math.sin((x+1)*angle/180 * math.pi)])
    return lines


line_positions = GenerateLinePositions(VISUALIZATION_RESOLUTION)

file1 = open('/media/psf/Developer/Robotics/char-01/ros2_pkg/data/all.txt', 'r')
Lines = file1.readlines()

for line in Lines:
    distances = GetDataFromArduino(line)
    print(len(distances))
    if (len(distances) == LIDAR_RESOLUTION):
        # Create a new figure for each line
        plt.figure()

        for x in range(VISUALIZATION_RESOLUTION):
            if distances[x] == 0:
                distances[x] = 20
            a = int(float(distances[x]))/2000
            if x in [136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 176, 184, 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 210, 211, 212, 213, 214, 215, 216, 217, 218, 219, 220, 221, 222, 223]:
                plt.plot(line_positions[x][0]*a+400,
                         line_positions[x][1]*a+400, 'ro')
            else:
                plt.plot(line_positions[x][0]*a+400,
                         line_positions[x][1]*a+400, 'ko')

        plt.plot(400, 400, 'o', color=(252/255, 132/255, 3/255))
        plt.show()

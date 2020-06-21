import numpy as np
import matplotlib.pyplot as plt
import random
import csv

from mpl_toolkits.mplot3d import Axes3D

#Variables
theta = []
z = []
x= []
y = []
pos_x = []
pos_y = []

def parse_flight_path(path):
    # <summary>
    #Parse the flight data in csv format (order is Y and X)
    # </summary>
    # <param> 
    # path -> The csv file path in directory 
    # </param>
    # <returns> The amount of sweep</returns>

    length = 0
    sweep = 0
    with open(path) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        for row in csv_reader:
            if(length ==0):
                length = int(row[1])
                sweep = int(row[0])
                pos_x.append([])
                pos_y.append([])
            else:
                pos_y[sweep].append(float(row[0]))
                pos_x[sweep].append(float(row[1]))
                length-=1
    return sweep

def prase_LIDAR(path):
    # <summary>
    #Parse the 2D LIDAR data in csv format (order is theta and distance)
    # The data is converted into x & y coordinate for visualization purposes
    # </summary>
    # <param> 
    # path -> The csv file path in directory 
    # </param>
    # <returns> The amount of sweep</returns>
    length = 0
    sweep = 0
    with open(path) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        for row in csv_reader:
            theta_ = float(row[0])
            z_ = float(row[1])
            if(length == 0):
                length = int(z_)
                sweep = int(theta_)
                theta.append([])
                z.append([])
                x.append([])
                y.append([])
            else:
                #Compute the x and y of the point detected by LIDAR data
                #sin theta = x/z | cos theta = y/z
                z_/=1000 # to meter
                theta[sweep].append(float(theta_))
                z[sweep].append(float(z_))
                #The final position is x + position of the dron in current sweep            
                x[sweep].append(np.sin( float(theta_) * np.pi/180) * float(z_) - pos_x[sweep][0])
                y[sweep].append(np.cos( float(theta_) * np.pi/180) * float(z_) + pos_y[sweep][0])                
                length-=1
    return sweep

def visualize_data_2D(VISUALIZE_SWEEP, sweep):
    # <summary>
    # Visualize the drone position and the point cloud obtained from LIDAR in 2D
    # </summary>
    # <param> 
    # VISUALIZE_SWEEP -> Visualize each sweep in a different window 
    # sweep -> How many sweep to visualize
    # </param>
    # <returns> The amount of sweep</returns>
    
    fig = plt.figure('combined')
    ax = fig.add_subplot(111)

    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
        
    for i in range(sweep):
        if(VISUALIZE_SWEEP):
            fig = plt.figure(i)
            ax = fig.add_subplot(111)
        colors=(random.uniform(0, 1), random.uniform(0, 1), random.uniform(0, 1))
        ax.scatter(x[i][:], y[i][:], s=5, c=np.array([colors]), marker='o') #LIDAR
        ax.scatter(-pos_x[i][0], pos_y[i][0], s=100, c=np.array([colors]), marker='P') #DRONE
    plt.show()

def visualize_data_3D(VISUALIZE_SWEEP, sweep):
    # <summary>
    # Visualize the drone position and the point cloud obtained from LIDAR in 3D
    # </summary>
    # <param> 
    # VISUALIZE_SWEEP -> Visualize each sweep in a different window 
    # sweep -> How many sweep to visualize
    # </param>
    # <returns> The amount of sweep</returns>

    fig = plt.figure('combined')

    ax = fig.add_subplot(111, projection='3d')
    ax.set_zlabel('Z axis')
    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
        
    for i in range(sweep):
        if(VISUALIZE_SWEEP):
            fig = plt.figure(i)
            ax = fig.add_subplot(111, projection='3d')
        colors=(random.uniform(0, 1), random.uniform(0, 1), random.uniform(0, 1))
        ax.scatter(x[i][:], y[i][:], s=5, c=np.array([colors]), marker='o') #LIDAR
        ax.scatter(-pos_x[i][0], pos_y[i][0], s=100, c=np.array([colors]), marker='P') #DRONE
    plt.show()



if __name__ == "__main__":
    #TODO get this from the user input not static initialization
    VISUALIZE_3D = True
    VISUALIZE_SWEEP = False #TODO Too many window replace with interactive UI
    flight_path_dir = 'data/FlightPath.csv'
    LIDAR_dir = 'data/LIDARPoints.csv'

    path_sweep = parse_flight_path(flight_path_dir)
    LIDAR_sweep = prase_LIDAR(LIDAR_dir)
    if(path_sweep != LIDAR_sweep):
        print("Inconsistent data")
        exit
    if(VISUALIZE_3D):
        visualize_data_3D(VISUALIZE_SWEEP, LIDAR_sweep)
    else:
        visualize_data_2D(VISUALIZE_SWEEP, LIDAR_sweep)
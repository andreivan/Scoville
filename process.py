import numpy as np
import matplotlib.pyplot as plt
import random
import csv

from mpl_toolkits.mplot3d import Axes3D

#Variables
theta = []
z = []
lidar_x = []
lidar_y = []
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
                lidar_x.append([])
                lidar_y.append([])
            else:
                #Compute the x and y of the point detected by LIDAR data
                #sin theta = x/z | cos theta = y/z
                z_/=1000 # to meter
                theta[sweep].append(float(theta_))
                z[sweep].append(float(z_))
                #The final position is x + position of the dron in current sweep            
                lidar_x[sweep].append(np.sin( float(theta_) * np.pi/180) * float(z_) - pos_x[sweep][0])
                lidar_y[sweep].append(np.cos( float(theta_) * np.pi/180) * float(z_) + pos_y[sweep][0])                
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
        ax.scatter(lidar_x[i][:], lidar_y[i][:], s=5, c=np.array([colors]), marker='o') #LIDAR
        ax.scatter(-pos_x[i][0], pos_y[i][0], s=100, c=np.array([colors]), marker='P') #DRONE

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
        ax.scatter(lidar_x[i][:], lidar_y[i][:], s=5, c=np.array([colors]), marker='o') #LIDAR
        ax.scatter(-pos_x[i][0], pos_y[i][0], s=100, c=np.array([colors]), marker='P') #DRONE


def visualize_trajectory(sweep):
    # <summary>
    # Visualize the tracjectory of drone
    # </summary>
    # <param> 
    # The number of sweep to visualize
    # </param>
    fig = plt.figure('trajectory')

    ax = fig.add_subplot(111)
    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
        
    for i in range(sweep):
        colors=(random.uniform(0, 1), random.uniform(0, 1), random.uniform(0, 1))
        ax.scatter(lidar_x[i][:], lidar_y[i][:], s=5, c='b', marker='o') #LIDAR
        ax.scatter(-pos_x[i][0], pos_y[i][0], s=100, c=np.array([colors]), marker='P') #DRONE
        ax.text(-pos_x[i][0], pos_y[i][0], s=i, fontsize=12) #SWEEP
        if(i<sweep-1):
            plt.plot([-pos_x[i][0], -pos_x[i+1][0]], [pos_y[i][0], pos_y[i+1][0]], 'rx-')

    
def visualize_grid(sweep):
    # <summary>
    # Visualize the LIDAR data as a grid to do path planning or reroute
    # </summary>
    # <param> 
    # The number of sweep to visualize
    # </param>
    x_ = lidar_x.copy()
    y_ = lidar_y.copy()

    pos_x_ = pos_x.copy()
    pos_y_ = pos_y.copy()

    max_x = np.amax(np.amax(x_))
    min_x = np.amin(np.amin(x_))
    
    max_y = np.amax(np.amax(y_))
    min_y = np.amin(np.amin(y_))
    
    width = int(np.round(np.abs(max_x) + np.abs(min_x)))
    height = int(np.round(np.abs(max_y) + np.abs(min_y))) 

    #[meter]
    sx = -pos_x_[0][0] 
    sy = pos_y_[0][0]
    gx = -pos_x_[sweep-1][0]  
    gy = pos_y_[sweep-1][0]  
    grid_size = 2.0  
    robot_radius = 1.0  

    # set obstacle positions
    ox, oy = [], []
    for s in range (sweep):
        for x, y in zip(x_[s], y_[s]):
            ox.append(x)
            oy.append(y)

    fig = plt.figure('planning')

    ax = fig.add_subplot(111)

    plt.plot(ox, oy, ".k")
    plt.plot(sx, sy, "og")
    plt.plot(gx, gy, "xb")
    plt.grid(True)
    plt.axis("equal")



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

    visualize_trajectory(LIDAR_sweep)

    visualize_grid(LIDAR_sweep)
    plt.show()
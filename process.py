import numpy as np
import matplotlib.pyplot as plt
import random
import csv

from a_star import AStarPlanner

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
                pos_y[sweep].append(float(row[0])*10)
                pos_x[sweep].append(float(row[1])*10)
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
                z_/=100 # to decimeter
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
    ax_c = fig.add_subplot(111)

    ax_c.set_xlabel('X axis')
    ax_c.set_ylabel('Y axis')
        
    for i in range(sweep):
        colors=(random.uniform(0, 1), random.uniform(0, 1), random.uniform(0, 1))
        if(VISUALIZE_SWEEP):
            fig = plt.figure(i)
            ax = fig.add_subplot(111)
            ax.set_xlabel('X axis')
            ax.set_ylabel('Y axis')
            ax.scatter(lidar_x[i][:], lidar_y[i][:], s=5, c=np.array([colors]), marker='o') #LIDAR
            ax.scatter(-pos_x[i][0], pos_y[i][0], s=100, c=np.array([colors]), marker='P') #DRONE
        ax_c.scatter(lidar_x[i][:], lidar_y[i][:], s=5, c=np.array([colors]), marker='o') #LIDAR
        ax_c.scatter(-pos_x[i][0], pos_y[i][0], s=100, c=np.array([colors]), marker='P') #DRONE

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

    ax_c = fig.add_subplot(111, projection='3d')
    ax_c.set_zlabel('Z axis')
    ax_c.set_xlabel('X axis')
    ax_c.set_ylabel('Y axis')
        
    for i in range(sweep):
        colors=(random.uniform(0, 1), random.uniform(0, 1), random.uniform(0, 1))
        if(VISUALIZE_SWEEP):
            fig = plt.figure(i)
            ax = fig.add_subplot(111, projection='3d')
            ax.set_zlabel('Z axis')
            ax.set_xlabel('X axis')
            ax.set_ylabel('Y axis')
            ax.scatter(lidar_x[i][:], lidar_y[i][:], s=5, c=np.array([colors]), marker='o') #LIDAR
            ax.scatter(-pos_x[i][0], pos_y[i][0], s=100, c=np.array([colors]), marker='P') #DRONE
        ax_c.scatter(lidar_x[i][:], lidar_y[i][:], s=5, c=np.array([colors]), marker='o') #LIDAR
        ax_c.scatter(-pos_x[i][0], pos_y[i][0], s=100, c=np.array([colors]), marker='P') #DRONE


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
        plt.plot(lidar_x[i][:], lidar_y[i][:], 'bo') #LIDAR
        ax.scatter(-pos_x[i][0], pos_y[i][0], s=100, c=np.array([colors]), marker='P') #DRONE
        ax.text(-pos_x[i][0], pos_y[i][0], s=i, fontsize=12) #SWEEP
        if(i<sweep-1):
            plt.plot([-pos_x[i][0], -pos_x[i+1][0]], [pos_y[i][0], pos_y[i+1][0]], 'rx-')

def find_min_max(x, y):
    max_x = np.amax(np.amax(x))
    min_x = np.amin(np.amin(x))
    max_y = np.amax(np.amax(y))
    min_y = np.amin(np.amin(y))

    return min_x, max_x, min_y, max_y
    
def flight_reroute(start, goal, sweep):
    # <summary>
    # Reroute the trajectory based on start and goal (currently assumed as the first and the sweep parameter trajectory)
    # </summary>
    # <param> 
    # The goal of flight reroute
    # </param>
    x_ = lidar_x.copy()
    y_ = lidar_y.copy()

    pos_x_ = pos_x.copy()
    pos_y_ = pos_y.copy()

    min_x, max_x, min_y, max_y = find_min_max(x_, y_)
    
    width = int(np.round(np.abs(max_x) + np.abs(min_x))) 
    height = int(np.round(np.abs(max_y) + np.abs(min_y))) 

    #[decimeter]
    sx = int(-pos_x_[start][0]) 
    sy = int(pos_y_[start][0]) 
    gx = int(-pos_x_[goal-1][0]) 
    gy = int(pos_y_[goal-1][0]) 
    grid_size = 8.0 
    robot_radius = 1.0  

    # set obstacle positions
    ox, oy = [], []
    for s in range (sweep):
        for x, y in zip(x_[s], y_[s]):
            ox.append(int(np.round(x)))
            oy.append(int(np.round(y)))

    fig = plt.figure('reroute')
    ax = fig.add_subplot(111)
    plt.plot(ox, oy, ".k")
    plt.plot(sx, sy, "og")
    plt.plot(gx, gy, "xb")
    major_ticks = np.arange(-200, 301, 20)
    ax.set_xticks(major_ticks)
    ax.set_yticks(major_ticks)
    plt.grid(True)
    plt.axis("equal")

    a_star = AStarPlanner(ox, oy, grid_size, robot_radius, True)
    rx, ry = a_star.planning(sx, sy, gx, gy)
    plt.plot(rx, ry, "-r")
    
def flight_optimization(start, goal, sweep):
    # <summary>
    # Optimize the trajectory based on start and goal set
    # To optimize the trajectory we use the middle trajectory as reference and perform rerouting twice
    # </summary>
    # <param> 
    # The goal of flight reroute
    # </param>
    x_ = lidar_x.copy()
    y_ = lidar_y.copy()

    pos_x_ = pos_x.copy()
    pos_y_ = pos_y.copy()

    min_x, max_x, min_y, max_y = find_min_max(x_, y_)
    
    width = int(np.round(np.abs(max_x) + np.abs(min_x))) 
    height = int(np.round(np.abs(max_y) + np.abs(min_y))) 

    #[decimeter]
    grid_size = 9.0 
    robot_radius = 1.0  

    # set obstacle positions
    ox, oy = [], []
    for s in range (sweep):
        for x, y in zip(x_[s], y_[s]):
            ox.append(int(np.round(x)) )
            oy.append(int(np.round(y)) )

    fig = plt.figure('optimize')
    ax = fig.add_subplot(111)
    plt.plot(ox, oy, ".k")
    major_ticks = np.arange(-200, 301, 20)
    ax.set_xticks(major_ticks)
    ax.set_yticks(major_ticks)
    plt.grid(True)
    plt.axis("equal")

    #optimization
    gx = []
    gy = [] 

    for i in range (start, goal, 3):
        print(i)
        gx.append(int(-pos_x_[i][0])) 
        gy.append(int(pos_y_[i][0])) 
    if(goal%3!=0):
        gx.append(int(-pos_x_[goal-1][0]))
        gy.append(int(pos_y_[goal-1][0])) 
    
    a_star = AStarPlanner(ox, oy, grid_size, robot_radius, True)
    rx = []
    ry = []
    for i in range(len(gx) - 1):
        rx_, ry_ = a_star.planning(gx[i], gy[i], gx[i+1], gy[i+1])
        rx.append(rx_)
        ry.append(ry_)
        plt.plot(gx[i], gy[i], "xg")
        plt.plot(rx[i], ry[i], "-r")
    
    #plot original trajectory
    fig = plt.figure('original')
    ax = fig.add_subplot(111)
    plt.plot(ox, oy, ".k")
    major_ticks = np.arange(-200, 301, 20)
    ax.set_xticks(major_ticks)
    ax.set_yticks(major_ticks)
    plt.grid(True)
    plt.axis("equal")

    for i in range(sweep):
        plt.plot(-pos_x_[i][0], pos_y_[i][0], 'xg')
        plt.plot([-pos_x_[i][0], -pos_x_[i+1][0]], [pos_y_[i][0], pos_y_[i+1][0]], '-r')

    


if __name__ == "__main__":
    #TODO get this from the user input not static initialization
    VISUALIZE_3D = False
    #TODO Too many window replace with interactive UI
    VISUALIZE_SWEEP = False #Visualize each sweep in a different window
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
    flight_reroute(0, LIDAR_sweep, LIDAR_sweep)
    flight_optimization(0, LIDAR_sweep, LIDAR_sweep)
    plt.show()
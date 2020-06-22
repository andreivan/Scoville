# Scoville
Software Engineering Assignment from Scoville

# Dependencies
  - Python
  - Matplotlib
   
# How to install
Make sure Python is installed
```sh
python ––version
```

If not install any version of Python 3 and up
```sh
sudo apt install python3.6
```

Install matplotlib
```sh
$ pip install matplotlib
```

# How to run the program

```sh
$ python3 process.py
```
To visualize each sweep of the LIDAR in different window change the flag to True
```sh
VISUALIZE_SWEEP = False #Visualize each sweep in a different window
```

# Discussion
- The data is parsed into a 2D list separated by each sweep so data from each sweep can be accessed easily.
- The format or coordinate system of the file is between LIDAR and drone is not well defined.
- The reroute and optimization function is implemented to take any start and goal position instead of the first and last sweep. 
- To search the best route a* algorithm is choosen knowing its efficiency.
- Collision between the wall and drone is not considered. This can be implemented by dilating the wall by at least 1X of the drone size. The altitude of the drone is also not considered since no data is provided.
- The flight optimization is designed by naively jumping over 3 paths and optimize it. This might not be efficient if the trajectory of the drone went back and forth creating unnecesary trajectory which should be optimized.
- To save the development time all visualization is done through matplotlib library. Also considering the data from LIDAR is a 2D plane this visualization should be sufficient.
- Some problem in visualization especially the spacing between grid. Hyperparameter tuning can be done later to make the visualization more accurate and pretty.

# Results
![Data visualization](https://github.com/andreivan/Scoville/blob/master/results/trajectory_and_lidar.png?raw=true)
*Data visualization*
Plus &#8594; Drone position with the number of sweep in order
Blue dots &#8594; The combined LIDAR point cloud mapped into x and y
Red line &#8594; The predicted path by drawing line between two drone position

![Sweep #0](https://github.com/andreivan/Scoville/blob/master/results/combined.png?raw=true)
*Combined map*

Plus &#8594; Drone position with the color representing point cloud belong to which sweep.

![Sweep #0](https://github.com/andreivan/Scoville/blob/master/results/sweep_4.png?raw=true)
*Individual sweep #5*

Plus &#8594; Drone position with the color representing point cloud belong to which sweep.

![Reroute](https://github.com/andreivan/Scoville/blob/master/results/reroute_traj.png?raw=true)
*Path reroute*

Blue cross &#8594; Starting position.
Green circle &#8594; Goal position.
Teal cross &#8594; The searched position by A* algorithm.
Red line &#8594; The predicted reroute path by A* algorithm.

![Flight optimization](https://github.com/andreivan/Scoville/blob/master/results/optimized_traj.png?raw=true)
*Path optimization*

Green cross &#8594; Trajectories to be optimized.
Teal cross &#8594; The searched position by A* algorithm.
Red line &#8594; The predicted optimized path by A* algorithm.

### Todos

 - Remove static variables (directory and flags). Get the input from user by parsing arguements.
 - Fancier visualization instead of matplot

License
----

MIT

# Scoville
Software Engineering Assignment from Scoville

# Dependencies
  - Python
  - Matplotlib
   
# How to install
```sh
$ pip install matplotlib
```

# How to run the program

```sh
$ python3 process.py
```

# Discussion
- The data is parsed into a 2D list separated by each sweep so data from each sweep can be accessed easily.
- The format or coordinate system of the file is not well defined. Example the X axis between LIDAR and drone position is reversed.
- The reroute and optimization function is implemented to take any start and goal position instead of the first and last sweep. 
- To search the best route a* algorithm is choosen knowing its efficiency.
- Collision between the wall and drone is not considered. This can be implemented by dilating the wall by at least 1X of the drone size. The altitude of the drone is also not considered since no data is provided.
- The flight optimization is designed by naively jumping over 3 paths and optimize it. This might not be efficient if the trajectory of the drone went back and forth creating unnecesary trajectory which should be optimized.
- To save the development time all visualization is done through matplotlib library. Also considering the data from LIDAR is a 2D plane this visualization should be sufficient.
- Some problem in visualization especially the spacing between grid. Hyperparameter tuning can be done later to make the visualization more accurate and pretty.

# Results
-Data visualization
![Data visualization](https://github.com/andreivan/Scoville/blob/master/results/trajectory_and_lidar.png?raw=true)

-First sweep
![Sweep #0](https://github.com/andreivan/Scoville/blob/master/results/sweep_0.png?raw=true)

-Last sweep
![Sweep #16](https://github.com/andreivan/Scoville/blob/master/results/sweep_16.png?raw=true)

-Path reroute
![Reroute](https://github.com/andreivan/Scoville/blob/master/results/reroute_traj.png?raw=true)

-Path optimization
![Flight optimization](https://github.com/andreivan/Scoville/blob/master/results/optimized_traj.png?raw=true)


### Todos

 - Remove static variables (directory and flags)
 - Fancier visualization instead of matplot

License
----

MIT

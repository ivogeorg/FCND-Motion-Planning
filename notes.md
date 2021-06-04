# Notes on FCND P2 3D Motion Planning

## Lesson 3: From Grids to Graphs

### Medial-Axis Exercise

1. In the task for finding the points on the `skeleton` (the set of medial-axis ridges) which are closest to the `start_ne` and `goal_ne`, my solution did not use the optimization features of `numpy`.  
2. My [solution](notebooks/Medial-Axis.ipynb) was a progressive expansion of the radial "squares" around a point, checking if any of the points at the current "radius" belonged to the skeleton. I defined 3 local (i.e. nested) functions (`on_skel`, `neighbors`, `find_closest`) to achieve a short and self-explanatory solution. It works fine but may be: (i) inefficient, and (ii) less than elegant.    
3. The [official](notebooks/Medial-Axis-Solution.ipynb) elegantly employs the _vectorization and broadcasting_ features of `numpy` (`np.nonzero`, `np.transpose`, `np.linalg.norm`, and `np.argmin`).  
4. A series of three articles on the main optimization features of `numpy` is:
   1. [Part 1: Vectorization and Broadcasting](https://blog.paperspace.com/numpy-optimization-vectorization-and-broadcasting/).
   2. [Part 2: Profiling and Vectorization](https://blog.paperspace.com/numpy-optimization-vectorization-and-broadcasting/).
   3. [Part 3: Internals, Strides, Reshaping, and Transpose](https://blog.paperspace.com/numpy-optimization-vectorization-and-broadcasting/).


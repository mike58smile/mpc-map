<p align="left">
  <img src="images/logo.png" height="80">
  <img src="images/fekt.png" height="80">
</p>

# MPC-MAP Assignment No. X - Report

**Author:** Michal Miškolci  
**Date:** 27. 4. 2026  
## Task 1

I implemented a simple A* search on the occupancy grid stored in `read_only_vars.discrete_map.map`. The open set is ordered by $f(x)=g(x)+h(x)$, where $g(x)$ accumulates the step cost and $h(x)$ is the Euclidean distance to the goal. The algorithm expands 8-connected neighbors, skips occupied cells, and reconstructs the path from the goal back to the start to ensure a collision-free route.

![](images/plannedPath.png)


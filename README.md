# A* Pathfinding in MATLAB

## Overview
This project implements the A* pathfinding algorithm in 4 differeent environments, which are 40x40 map1, 40x40 map2, 500x500, 500x500 complex grid in MATLAB. The project aim to show the difference on results when implementing Euclidean, Diagonal, and Manhattan Distance heuristics. It is designed to navigate through a predefined set of obstacles to find the shortest path from a start to a goal node. 

## Features
- Customizable grid and obstacle setup
- A* pathfinding algorithm with different distance heuristic
- Calculation of path length with distinct costs for diagonal and orthogonal moves
- Visualization of the path and obstacles within the MATLAB environment

## Requirements
- MATLAB (This code was tested on MATLAB R202X)

## Installation
No specific installation is needed, simply clone the repository or copy the MATLAB files to your local machine.

## Usage
To run the A* simulation, execute the `A_star_pathfinding` function within MATLAB. This function initializes the simulation, performs the A* search, and plots the resulting path.

## Configuration
- Modify `startX`, `startY`, `goalX`, and `goalY` to set the starting and goal positions.
- Edit `obstaclesrec` array to configure obstacles within the grid.
- The `objectSize` variable defines the size of the object moving through the grid, affecting obstacle avoidance.

## Contributing
Contributions to improve the simulation or adapt it for different scenarios are welcome. Please ensure you test any changes in your local environment before submitting a pull request.

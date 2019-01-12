# Portfolio
A collection of projects/assignments I had an opportunity to be a part of during my Bachelor's/Master's years in UCF.

## Robocopters
- A system within ROS to control a quadcopter to identify and attack prey drones
- My contribution of the project consists of navigation subsystem:
-- The line of sight pursuit guidance is implemented in C++
-- Communications with the flight controller are done through the Python interface
-- Contains scripts for the initialization of the environment as well
-- Tested in SITL environment

## Pacman Berkley Project
- Solutions to the ai.berkley pacman project. All the source code is taken directly from the ai.berkley website, and the blanks were
  filled in by me. To run a particular part, just run the autograder.py within the folder.
- Part 1 : DFS, BFS, UCS, A* and heuristics
- Part 2 : Reflex Agents, MiniMax, Alpha-Beta Pruning, Expectimax
- Part 3 : Value Iteration and Q-learning
- Still working on parts 4,5,6

## Unbounded Nonblocking Dequeu
- Unbounded Concurrent Nonblocking container written in C++
- Based on the code from "An Unbounded Nonblocking Double-ended Queue" by Matthew Graichen, Joseph Izraelevitz, Michael L. Scott 2016 45th International Conference on Parallel Processing
- Includes alternative transactional designs with different granularity
- Includes a test case for performance comparisons 

## Minor Problems:
- Concurrent stacks: Lock-free, Descriptor-based in Java; and Elimination-backoff in C++.
- K nearest neigbour classifier on MNIST. k-fold validation for optimal k, sliding window. Python
- SVM for glass dataset, k-fold validation, hyperparameter testing, kernel selection. Python

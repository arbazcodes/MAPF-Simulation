# Multi-Robot Pathfinding and Simulation Framework  

This project is a **simulation framework** designed for **multi-agent pathfinding** and real-time visual simulation. It provides a robust platform for testing and visualizing pathfinding algorithms, with a primary focus on **Conflict-Based Search (CBS)** and **Priority Inheritance with Backtracking for Iterative Multi-agent Path Finding (PIBT)**. This project is built for applications in fields like warehouse automation, transport automation, robotics, gaming, and autonomous vehicle navigation, where real-time coordination of multiple agents is essential.  

The framework is modular, resource-efficient, and built for **extensibility**, allowing easy integration of new algorithms and simulation components. This makes it suitable for academic research, industry testing, and real-world applications.

## Key Features  
- **Pathfinding Algorithms:** The framework provides implementations for **Conflict-Based Search (CBS)** and **Priority Inheritance with Backtracking for Iterative Multi-agent Path Finding (PIBT)**, two powerful algorithms for multi-agent pathfinding.
- **Real-Time Simulation:** Visualizes robot movements, rotations, and goal-reaching behaviors in a 2D grid environment.
- **Collision-Free Path Planning:** CBS and PIBT ensure optimal or near-optimal collision-free paths for multiple robots in a shared space.
- **Dynamic Reset on Timeouts:** The simulation automatically resets if pathfinding times out, ensuring robustness in challenging environments.
- **Modular Design for Extensibility:** The framework is designed to easily integrate new pathfinding algorithms, visual components, or features.
- **Resource Management:** Efficient handling of textures, shaders, and grid layouts to ensure smooth rendering even with complex simulations.

## Pathfinding Algorithms  

### Conflict-Based Search (CBS)  
CBS is a **hierarchical** algorithm that decomposes the pathfinding problem into a high-level search for conflicting agent paths, followed by resolving conflicts at the low level. It’s known for delivering **optimal solutions** while efficiently handling complex environments with multiple agents.  

- **Performance Stats:**
  - **Optimal for Up to 16% Density:** CBS works effectively with **up to 16%** density, $\left( \frac{\text{Number of agents}}{\text{Total available locations}} \right) \times 100$, ensuring optimal solutions.
  

### Prioritized Informed-Subset Pathfinding (PIBT)  
PIBT is a **hierarchical** pathfinding algorithm designed to improve upon traditional prioritized planning methods by incorporating **informed search** techniques. PIBT prioritizes agents based on their difficulty of planning and allocates resources accordingly, improving scalability and efficiency when solving complex multi-agent pathfinding problems. It is particularly effective in large-scale environments where agents need to navigate with a minimal number of conflicts.  

- **Performance Stats:**
  - **High Density, Low Computational Overhead:** PIBT can handle **high-density environments** (approaching 99% occupancy) while maintaining **low computational overhead**, allowing for fast solutions without significant resource demands.
  - **Optimal Solutions in Lower Density:** For environments with fewer obstacles, PIBT can also provide near-optimal solutions, although it does not guarantee optimality in all cases.
  - **Suboptimal Solutions for High Density:** PIBT still performs well even at **99%** obstacle density, with an average runtime of **0.0057443 seconds**. However, the solutions may be **bounded suboptimal** at 
 these higher densities, making it ideal for real-time applications where slight suboptimality can be tolerated for faster results.

## Assumptions and Constraints  
Both CBS and PIBT operate under the following **assumptions and constraints**:
- **Avoidance of Vertex and Edge Conflicts:** Agents must avoid both vertex and edge conflicts to prevent any collisions during movement.
- **Stopping Constraint:** An agent cannot move to a spot occupied by another agent that has stopped, ensuring no overlap between agents at specific positions.
- **Following Constraint:** An agent cannot immediately move to a spot vacated by another agent only one timestep ago, reducing the risk of race conditions and deadlocks.

## Technology Stack  
- **Programming Language:** C++  
- **Graphics and Rendering:** OpenGL, GLFW, GLAD, glm  
- **Build System:** CMake  
- **Pathfinding Algorithms:** Conflict-Based Search (CBS), Prioritized Informed-Subset Pathfinding (PIBT)  

## Installation  

### Prerequisites  
Before building the project, ensure that the following dependencies are installed:
- **C++17 or newer**
- **CMake 3.10+**
- **OpenGL 3.3+**
- **GLFW** (for window management and user input handling)
- **GLAD** (for OpenGL function loading)
- **GLM** (for linear algebra operations)

### Build Instructions  
1. Clone the repository:  
   ```bash  
   git clone git@github.com:arbazcodes/MAPF-Simulation.git 
   cd MAPF-Simulation
2. Create and navigate to the build directory:  
   ```bash  
   mkdir build  
   cd build    
3. Configure and build the project using CMake:  
   ```bash  
   cmake ..  
   make

### Running the Engines
This project includes two engines: CBS Engine and PIBT Engine. You can run them as follows:

  * CBS Engine:
    ```bash
    ./cbs_engine

  * PIBT Engine:
    ```bash
    ./pibt_engine

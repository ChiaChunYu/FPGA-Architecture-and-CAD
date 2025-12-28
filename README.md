# FPGA Architecture & CAD

This is the programming assignment for the NTHU CS 516000 FPGA Architecture & CAD course (2025 Fall).

## Congestion Driven FPGA Placement

### Overview
The primary objective of this assignment is to implement an optimized, congestion-driven placer for an island-style FPGA. The system produces a legal placement on an $R \times C$ grid while minimizing both the Half-Perimeter Wire Length (HPWL) and the Congestion Coefficient (CC).

This project implements:
* **Legal Placement**: Ensures every logic block is assigned to a unique integer coordinate without overlapping or moving fixed I/O pins.
* **Congestion Optimization**: Utilizes a bounding-box overlap model to calculate and minimize uneven routing demand.

### Methodology
The implementation follows a structured heuristic approach to handle the complexity of the placement problem.

#### 1. Initial Placement
The placer starts by generating a **random legal initial placement**. It collects all movable logic blocks and assigns them to unique grid sites using a random shuffling mechanism. 

#### 2. Simulated Annealing (SA) Framework
The core optimization engine is based on a **Simulated Annealing (SA)** framework. Starting from a high initial temperature, the algorithm iteratively explores the solution space. As the temperature cools down, the probability of accepting "bad moves" decreases, allowing the placer to escape local optima and eventually converge on a high-quality global solution.

#### 3. Cost Function
To balance wirelength and routing congestion, the optimization is guided by a multiplicative cost function that combines the HPWL and congestion metrics:
$$Cost = HPWL \times (Congestion\_Factor)$$

#### 4. Move Operators (Swap Methods)
The placer explores new configurations by proposing swaps between logic blocks. The movement strategy includes:
* **Random Swapping**: Randomly selecting two logic blocks (or a block and an empty site) to exchange positions.
* **Region-Based Moves**: Implementing a range limiter that restricts the swap distance as the temperature decreases, focusing on local refinements during the later stages of annealing.

### How to Run

#### 1. Requirements
Before running the application, ensure you have a C++ compiler supporting C++17 and a Linux environment:
```bash
# Verify g++ installation
g++ --version
```

#### 2. Compiling the Code
The source code is written in C++ and utilizes a Makefile for automated building. To compile the program, execute the following command in the `src/` directory:
```bash
# Compile the placer executable
make
```
The executable `placer` will be generated in the `bin/` directory.

#### 3. Running the Program
To execute the placer, provide the input testcase file and the desired output file path:
```bash
# Run the placement tool
./placer <input_file> <output_file>
```

#### 4.Testing
```bash
# Run test for alu4 benchmark
make test_alu4

# Run all public testcases
make test
```

### Constraints & Limits
* **Runtime**: Each testcase must complete within 240 seconds.
* **Memory**: The program is limited to 8 GB of memory usage.

### Performance
**98.6/100**

### References
* [1] "A Congestion Driven Placement Algorithm for FPGA Synthesis", In Proceedings of the International Conference on Field Programmable Logic and Applications (FPL 2006).
* [2] V. Betz and J. Rose, "VPR: A New Packing, Placement and Routing Tool for FPGA Research," 1997.
* [3] R. Zh. Chochaev et al., "Simulated Annealing Based Placement in FPGAs with a Modified Permutation Operator and Congestion Optimization", PIERE, 2024.

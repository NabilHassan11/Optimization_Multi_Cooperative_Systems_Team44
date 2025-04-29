# Multi-UAV Cooperative Target Exploration Optimization

**Course**: MCTR 1021 - Optimization Techniques for Multi Cooperative Systems  
**Institution**: German University in Cairo, Faculty of Engineering and Materials Science  
**Team 44**: Ahmed Daw, Amr Mahgoub, Nabil Hassan, Mohamed Ashraf, Mohamed Yasser, Abdulrahman Manea  
**Supervisor**: Assoc. Prof. Dr. Omar Shehata  

---

## 📖 Overview
This repository contains the code, simulations, and analysis for optimizing cooperative path planning in **multi-UAV systems**. The goal is to minimize exploration time, maximize coverage efficiency, and ensure collision-free navigation while adhering to constraints (e.g., restricted zones, communication range). Four metaheuristic algorithms are implemented and compared:
1. **Simulated Annealing (SA)**
2. **Genetic Algorithm (GA)**
3. **Particle Swarm Optimization (PSO)**
4. **Teaching-Learning Based Optimization (TLBO)**

---

## 🚀 Key Features
- **Algorithm Implementations**: Code for SA, GA, PSO, and TLBO with customizable parameters.
- **Constraint Handling**: Avoid restricted zones, maintain safe inter-UAV distances, and enforce velocity/angle limits.
- **Dynamic Visualization**: Plot UAV paths, convergence curves, and fitness metrics.
- **Performance Comparison**: Standard deviation, mean fitness, and best fitness across 15 runs.

---

## 📂 Repository Structure
├── algorithms/ # Algorithm implementations (SA, GA, PSO, TLBO)
│ ├── sa/
│ ├── ga/
│ ├── pso/
│ └── tlbo/
├── config/ # Parameter files (YAML/JSON)
├── data/ # Inputs (map dimensions, target/obstacle coordinates)
├── results/ # Generated plots and convergence graphs
├── report/ # Final report (PDF) and LaTeX sources
├── tests/ # Test cases and validation scripts
├── LICENSE
└── requirements.txt # Python dependencies

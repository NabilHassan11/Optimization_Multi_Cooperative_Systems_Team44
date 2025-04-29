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

## 🦀 Key Features
- **Algorithm Implementations**: Code for SA, GA, PSO, and TLBO with customizable parameters.
- **Constraint Handling**: Avoid restricted zones, maintain safe inter-UAV distances, and enforce velocity/angle limits.
- **Dynamic Visualization**: Plot UAV paths, convergence curves, and fitness metrics.
- **Performance Comparison**: Standard deviation, mean fitness, and best fitness across 15 runs.

---

## 📂 Repository Structure
```text
├── algorithms/           # Algorithm implementations (SA, GA, PSO, TLBO)
│   ├── sa/               # Simulated Annealing
│   ├── ga/               # Genetic Algorithm
│   ├── pso/              # Particle Swarm Optimization
│   └── tlbo/             # Teaching-Learning-Based Optimization
├── config/               # Parameter configuration files (YAML/JSON)
├── data/                 # Input data (e.g., map dimensions, target/obstacle coordinates)
├── results/              # Generated plots and convergence graphs
├── report/               # Final report (PDF) and LaTeX sources
├── tests/                # Test cases and validation scripts
├── LICENSE               # Project license
└── requirements.txt      # Python dependencies
```
---
## 📊 Results

### Algorithm Comparison (15 Runs)

| Algorithm | Std. Deviation | Mean Fitness | Best Fitness |
|-----------|----------------|--------------|--------------|
| SA        | 26.94          | 111.58       | 85.43        |
| GA        | 6.75           | 76.63        | 72.56        |
| PSO       | 4.38           | 89.11        | 81.87        |
| TLBO      | 12.26          | 137.70       | 122.13       |

### 🔍 Key Insights

- **GA** achieved the lowest best fitness (**72.56**) but requires careful tuning to avoid local minima.
- **PSO** was the most consistent algorithm with the **lowest standard deviation**.
- **SA** converged the fastest but often settled for suboptimal solutions.

---

## ⚙️ Installation

### Prerequisites
- Python 3.8+  (optional for alternative implementations)
- MATLAB

1. **Clone the repository**:
   ```bash
   git clone https://github.com/Team44-GUC/Multi-UAV-Optimization.git
   cd Multi-UAV-Optimization

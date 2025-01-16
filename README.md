# Contributors 

- Bruno Fernandes - up202108871
- Hugo Abelheira - up202409899
- Tiago Coelho - up202105004

# Aircraft Landing Scheduling

The Aircraft Landing Scheduling (ALS) project addresses the problem of coordinating the landing of multiple aircraft on one or more runways while respecting critical constraints. These constraints include separation times between planes, runway availability, and penalties for early or late landings. The goal is to determine the optimal landing schedule while minimizing total penalties and ensuring safe operations.

This project uses both Constraint Programming (CP) and Mixed-Integer Programming (MIP) techniques to solve the problem under various configurations. Different strategies for branching, variable selection, and value selection are tested to evaluate their impact on solution quality and performance. Results are analyzed for both single and multiple runway scenarios, and key metrics such as execution time and success rates are recorded.

The repository is structured to include the implementation of solvers, utility functions for data preprocessing, performance evaluation scripts, and visualization tools to interpret the results. Input datasets, located in the `data` folder, define the aircraft and runway configurations for each test case.

```shell
.
├── README.md
├── Presentation.pdf
├── Report.pdf
├── requirements.txt
└── src
    ├── ALS
    │   ├── __init__.py
    │   ├── CP.py
    │   ├── MIP.py
    │   ├── performanceCP.py
    │   ├── performanceMIP.py
    │   ├── utils.py
    │   └── visualization.py
    ├── data
    ├── notebook.ipynb
```

To run the project, Python 3.8 or higher is required, along with the dependencies listed in `requirements.txt`. After installing the dependencies, the solvers can be executed to test either single or multiple runway scenarios using different adjustable parameters. Results can be analyzed or visualized using the pre-built Jupyter notebooks provided.

The repository also includes batch processing capabilities to test multiple datasets and strategy combinations automatically. Each run outputs performance metrics and detailed results for further analysis.

This project serves as a foundation for exploring optimization techniques in complex scheduling problems, particularly in the aviation domain. It is open for contributions to enhance its functionality, add new features, or improve performance.

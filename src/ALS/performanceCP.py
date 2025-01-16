from ortools.sat.python import cp_model

class PerformanceTracker(cp_model.CpSolverSolutionCallback):
    def __init__(self, solver, model, planes_data):
        super().__init__()

        # Store solver reference to access performance metrics
        self.solver = solver
        self.model = model
        self.planes_data = planes_data

    def getStatus(self):
        return self.solver.StatusName()

    def getWallTime(self):
        return self.solver.WallTime()

    def getNumConflicts(self):
        return self.solver.NumConflicts()

    def getNumBranches(self):
        return self.solver.NumBranches()

    def getBestObjectiveBound(self):
        return self.solver.BestObjectiveBound()

    def getNumVariables(self):
        return len(self.model.Proto().variables)

    def getNumConstraints(self):
        return len(self.model.Proto().constraints)

    def get_performance_metrics(self):
        """Returns solver performance metrics after solving as a dictionary."""

        # Coletar as métricas
        exec_time = self.getWallTime()
        num_variables = self.getNumVariables()
        num_constraints = self.getNumConstraints()
        status = self.getStatus()
        conflicts = self.getNumConflicts()
        branches = self.getNumBranches()
        best_objective = self.getBestObjectiveBound()

        # Criar o dicionário com as métricas
        metrics = {
            "exec_time": exec_time,
            "num_variables": num_variables,
            "num_constraints": num_constraints,
            "status": status,
            "conflicts": conflicts,
            "branches": branches,
            "best_objective": best_objective
        }

        return metrics

    def print_performance_metrics(self):  
        """Prints solver performance metrics after solving."""

        metrics = self.get_performance_metrics()

        exec_time = metrics["exec_time"]
        num_variables = metrics["num_variables"]
        num_constraints = metrics["num_constraints"]
        status = metrics["status"]
        conflicts = metrics["conflicts"]
        branches = metrics["branches"]
        best_objective = metrics["best_objective"]

        print()
        print("=" * 60)
        print("\t\tPerformance Metrics for CP")
        print("=" * 60, "\n")
        print(f"-> Execution time (s): {exec_time:.2f}")
        print(f"-> Number of variables: {num_variables}")
        print(f"-> Number of constraints: {num_constraints}")
        print(f"-> Solution Status: {status}")
        print(f"-> Number of Conflicts: {conflicts}")
        print(f"-> Number of Branches: {branches}")
        print(f"-> Best objective bound: {best_objective:.1f}")
        print("\n" + "=" * 60)

        return metrics 

def performance_CP(solver, model, planes_data):        
    # Create a PerformanceTracker instance that will track solver performance
    tracker = PerformanceTracker(solver, model, planes_data)

    # Print performance metrics after solving
    return tracker.print_performance_metrics()

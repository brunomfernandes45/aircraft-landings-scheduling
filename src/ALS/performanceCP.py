from ortools.sat.python import cp_model

class PerformanceTracker(cp_model.CpSolverSolutionCallback):
    def __init__(self, solver, planes_data, mem_before, mem_after):
        super().__init__()

        # Store solver reference to access performance metrics
        self.solver = solver
        self.planes_data = planes_data
        
        # Initialize performance metrics
        self.memory_before = mem_before 
        self.memory_after = mem_after
        
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

    def getMemoryUsage(self):
        memory_usage = (self.memory_after - self.memory_before) / (1024 * 1024)  # MB
        return memory_usage

    def get_performance_metrics(self):
        """Returns solver performance metrics after solving as a dictionary."""
        
        # Coletar as métricas
        exec_time = self.getWallTime()
        status = self.getStatus()
        mem_usage = self.getMemoryUsage()
        conflicts = self.getNumConflicts()
        branches = self.getNumBranches()
        best_objective = self.getBestObjectiveBound()
        
        # Criar o dicionário com as métricas
        metrics = {
            "exec_time": exec_time,
            "status": status,
            "mem_usage": mem_usage,
            "conflicts": conflicts,
            "branches": branches,
            "best_objective": best_objective
        }
        
        return metrics


    def print_performance_metrics(self):  
        """Prints solver performance metrics after solving."""
        
        metrics = self.get_performance_metrics()
        
        print()
        print("=" * 60)
        print("\t\tPerformance Metrics for CP")
        print("=" * 60, "\n")
        print(f"-> Execution time (s): {metrics.exec_time}")
        print(f"-> Solution Status: {metrics.status}")
        print(f"-> Memory usage (MB): {metrics.mem_usage}")
        print(f"-> Number of Conflicts: {metrics.conflicts}")
        print(f"-> Number of Branches: {metrics.branches}")
        print(f"-> Best objective bound: {metrics.best_objective}")
        print("\n" + "=" * 60)
        
        return metrics 
     
def performance_CP(solver, planes_data, mem_before, mem_after):        
    # Create a PerformanceTracker instance that will track solver performance
    tracker = PerformanceTracker(solver, planes_data, mem_before, mem_after)

    # Print performance metrics after solving
    return tracker.print_performance_metrics()
     

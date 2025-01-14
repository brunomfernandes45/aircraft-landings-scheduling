from ortools.sat.python import cp_model
import psutil, time

class PerformanceTracker(cp_model.CpSolverSolutionCallback):
    def __init__(self, solver, planes_data, mem_before, mem_after):
        super().__init__()

        # Store solver reference to access performance metrics
        self.solver = solver
        self.planes_data = planes_data
        
        # Initialize performance metrics
        self.start_time = time.time()
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

    def print_performance_metrics(self):
        """Prints solver performance metrics after solving."""
        print()
        print("=" * 60)
        print("Performance Metrics for CP")
        print("=" * 60)
        print(f"-> Execution time (s): {self.getWallTime()}")
        print(f"-> Solution Status: {self.getStatus()}")
        print(f"-> Memory usage (MB): {self.getMemoryUsage()}")
        print(f"-> Number of Conflicts: {self.getNumConflicts()}")
        print(f"-> Number of Branches: {self.getNumBranches()}")
        print(f"-> Best objective bound:{self.getBestObjectiveBound()}")
        print("=" * 60)
     
def performance_CP(solver, planes_data, mem_before, mem_after):        
    # Create a PerformanceTracker instance that will track solver performance
    tracker = PerformanceTracker(solver, planes_data, mem_before, mem_after)

    # Print performance metrics after solving
    tracker.print_performance_metrics()

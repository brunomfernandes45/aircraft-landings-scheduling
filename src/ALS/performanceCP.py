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
        exec_time = self.getWallTime()
        status = self.getStatus()
        mem_usage = self.getMemoryUsage()
        conflicts = self.getNumConflicts()
        branches = self.getNumBranches()
        best_objective = self.getBestObjectiveBound()
        print()
        print("=" * 60)
        print("Performance Metrics for CP")
        print("=" * 60)
        print(f"-> Execution time (s): {exec_time}")
        print(f"-> Solution Status: {status}")
        print(f"-> Memory usage (MB): {mem_usage}")
        print(f"-> Number of Conflicts: {conflicts}")
        print(f"-> Number of Branches: {branches}")
        print(f"-> Best objective bound: {best_objective}")
        print("=" * 60)
        
        return exec_time, status, mem_usage, conflicts, branches, best_objective
     
def performance_CP(solver, planes_data, mem_before, mem_after):        
    # Create a PerformanceTracker instance that will track solver performance
    tracker = PerformanceTracker(solver, planes_data, mem_before, mem_after)

    # Print performance metrics after solving
    return tracker.print_performance_metrics()
     

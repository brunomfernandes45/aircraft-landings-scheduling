from ortools.sat.python import cp_model
import psutil, time

class PerformanceTracker(cp_model.CpSolverSolutionCallback):
    def __init__(self, solver, planes_data):
        super().__init__()

        # Store solver reference to access performance metrics
        self.solver = solver
        self.planes_data = planes_data
        
        # Initialize performance metrics
        self.start_time = time.time()
        self.memory_before = psutil.Process().memory_info().rss  # Memory in bytes
        self.num_solutions = 0

    def on_solution_callback(self):
        """Called when a solution is found."""
        self.num_solutions += 1

    def print_performance_metrics(self):
        """Prints solver performance metrics after solving."""
        # Compute execution time and memory usage
        execution_time = time.time() - self.start_time
        memory_after = psutil.Process().memory_info().rss
        memory_usage = (memory_after - self.memory_before) / (1024 * 1024)  # MB

        
        print("=" * 60)
        print("Performance Metrics for CP")
        print("=" * 60)
        print(f"Execution time (s): {execution_time:.4f}")
        print(f"Memory usage (MB): {memory_usage:.4f}")
        print(f"Number of solutions found: {self.num_solutions}")

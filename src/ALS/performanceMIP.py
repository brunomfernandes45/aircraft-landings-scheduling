from ortools.linear_solver import pywraplp
import psutil

def calculate_execution_time(solver):
    """
    Calculates the total execution time of the solver.
    
    Args:
        solver: The OR-Tools solver instance.
        
    Returns:
        float: Execution time in seconds.
    """
    return solver.WallTime() / 1000  # Convert milliseconds to seconds

def calculate_num_variables(solver):
    """
    Calculates the total number of variables in the model.
    
    Args:
        solver: The OR-Tools solver instance.
    
    Returns:
        int: The total number of variables.
    """
    return solver.NumVariables()

def calculate_num_constraints(solver):
    """
    Calculates the total number of constraints in the model.
    
    Args:
        solver: The OR-Tools solver instance.
    
    Returns:
        int: The total number of constraints.
    """
    return solver.NumConstraints()

def calculate_total_penalty(variables, planes_data):
    """
    Calculates the total penalty of the solution based on early and late deviations.

    Args:
        variables: Dictionary containing model variables.
        planes_data: List of dictionaries with data for each plane.

    Returns:
        float: Total penalty of the solution.
    """
    total_penalty = 0
    for i, plane in enumerate(planes_data):
        total_penalty += (
            variables["early_deviation"][i].solution_value() * plane["penalty_early"]
            + variables["late_deviation"][i].solution_value() * plane["penalty_late"]
        )
    return total_penalty

def calculate_memory_usage(memory_before, memory_after):
    """
    Measures the memory usage of the solver process during execution.

    Returns:
        float: Memory usage in megabytes (MB).
    """
    memory_usage = (memory_after - memory_before) / (1024 * 1024)  # MB
    return memory_usage


def performance_MIP(solver, variables, num_planes, mem_before, mem_after, num_runways=None, planes_data=None):
    """
    Calculates and prints metrics specific to MIP problems.

    Args:
        solver: Instance of the OR-Tools solver.
        variables: Dictionary containing the model variables.
        num_planes: Number of planes in the problem.
        num_runways: (Optional) Number of runways.
        planes_data: (Optional) Data of planes used to calculate penalties.

    Returns:
        dict: Dictionary containing all calculated metrics.
    """
    print()
    print("=" * 60)
    print("Performance Metrics for MIP")
    print("=" * 60)

    # Execution time
    exec_time = calculate_execution_time(solver)
    print(f"-> Execution time: {exec_time:.2f} seconds")

    # Number of variables
    num_variables = calculate_num_variables(solver)
    print(f"-> Number of variables in the model: {num_variables}")

    # Number of constraints
    num_constraints = calculate_num_constraints(solver)
    print(f"-> Number of constraints in the model: {num_constraints}")

    # Total penalty (if applicable)
    total_penalty = 0
    if planes_data is not None:
        total_penalty = calculate_total_penalty(variables, planes_data)
        print(f"-> Total penalty: {total_penalty:.2f}")
    else:
        print("-> Total penalty not calculated (plane data not provided).")
    
    # Memory usage
    mem_usage = calculate_memory_usage(mem_before, mem_after)
    print(f"-> Memory usage: {mem_usage:.2f} MB")
    
    print("=" * 60)
    return exec_time, num_variables, num_constraints, total_penalty, mem_usage
    
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

def calculate_runway_workload(variables, num_runways, num_planes):
    """
    Calculates the workload (number of landings) for each runway.

    Args:
        variables: Dictionary containing model variables.
        num_runways: Number of runways.
        num_planes: Number of planes.

    Returns:
        list: Workload for each runway.
    """
    workloads = [0] * num_runways
    for r in range(num_runways):
        workloads[r] = sum(
            variables["landing_runway"][(i, r)].solution_value() for i in range(num_planes)
        )
    return workloads

def calculate_workload_imbalance(workloads):
    """
    Calculates the imbalance in workload across runways.

    Args:
        workloads: List of workloads for each runway.

    Returns:
        int: Difference between the maximum and minimum workload.
    """
    return max(workloads) - min(workloads)

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

def calculate_memory_usage():
    """
    Measures the memory usage of the solver process during execution.

    Returns:
        float: Memory usage in megabytes (MB).
    """
    process = psutil.Process()
    memory_in_bytes = process.memory_info().rss
    return memory_in_bytes / (1024 * 1024)  # Convert to MB



def summarize_metrics_MIP(solver, variables, num_planes, num_runways=None, planes_data=None):
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
    print("=" * 60)
    print("Performance Metrics for MIP")
    print("=" * 60)

    metrics = {}

    # Execution time
    execution_time = calculate_execution_time(solver)
    metrics['execution_time'] = execution_time
    print(f"-> Execution time: {execution_time:.2f} seconds")

    # Number of variables
    num_variables = calculate_num_variables(solver)
    metrics['num_variables'] = num_variables
    print(f"-> Number of variables in the model: {num_variables}")

    # Number of constraints
    num_constraints = calculate_num_constraints(solver)
    metrics['num_constraints'] = num_constraints
    print(f"-> Number of constraints in the model: {num_constraints}")

    # Workload per runway (if applicable)
    if num_runways is not None:
        workloads = calculate_runway_workload(variables, num_runways, num_planes)
        workload_imbalance = calculate_workload_imbalance(workloads)
        metrics['workloads'] = workloads
        metrics['workload_imbalance'] = workload_imbalance
        print(f"-> Workload per runway: {workloads}")
        print(f"-> Workload imbalance: {workload_imbalance}")
    else:
        print("-> Workload per runway not calculated (number of runways not provided).")

    # Total penalty (if applicable)
    if planes_data is not None:
        total_penalty = calculate_total_penalty(variables, planes_data)
        metrics['total_penalty'] = total_penalty
        print(f"-> Total penalty: {total_penalty:.2f}")
    else:
        print("-> Total penalty not calculated (plane data not provided).")
    
    # Memory usage
    memory_usage = calculate_memory_usage()
    metrics['memory_usage'] = memory_usage
    print(f"-> Memory usage: {memory_usage:.2f} MB")
    
    print("=" * 60)
    return metrics

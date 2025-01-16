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

def performance_MIP(solver):
    """
    Calculates and prints metrics specific to MIP problems.

    Args:
        solver: Instance of the OR-Tools solver.

    Returns:
        dict: Dictionary containing all calculated metrics.
    """
    print()
    print("=" * 60)
    print("\t\tPerformance Metrics for MIP")
    print("=" * 60, "\n")
    
    # Execution time
    exec_time = calculate_execution_time(solver)
    print(f"-> Execution time: {exec_time:.2f} seconds")

    # Number of variables
    num_variables = calculate_num_variables(solver)
    print(f"-> Number of variables in the model: {num_variables}")

    # Number of constraints
    num_constraints = calculate_num_constraints(solver)
    print(f"-> Number of constraints in the model: {num_constraints}")

    # Total penalty
    total_penalty = solver.Objective().Value()
    print(f"-> Total penalty: {total_penalty:.1f}")

    print("\n" + "=" * 60)
    
    # Create and return a dictionary with the metrics
    metrics = {
        "exec_time": exec_time,
        "num_variables": num_variables,
        "num_constraints": num_constraints,
        "total_penalty": total_penalty,
    }
    
    return metrics


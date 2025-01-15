def read_data(filename):
    """
    Reads data from a file with the specified format for an air traffic scheduling problem.

    Args:
        filename (str): The path to the data file.

    Returns:
        tuple: A tuple containing the parsed data:
            - num_planes (int): The number of planes.
            - planes_data (list): A list of dictionaries, where each dictionary contains
              the data for a plane.
            - separation_times (list of lists): A 2D list of separation times.
    """
    print("=" * 60)
    print("\t       Reading data from", filename.split('/')[-1])
    print("=" * 60, "\n")
    
    try:
        with open(filename, "r") as f:
            # Read the first line: number of planes and freeze time
            first_line = f.readline().strip().split()
            num_planes = int(first_line[0])
            _ = int(first_line[1])

            planes_data = []
            separation_times = []

            for _ in range(num_planes):
                line = f.readline().strip().split()
                appearance_time = int(line[0])
                earliest_landing_time = int(line[1])
                target_landing_time = int(line[2])
                latest_landing_time = int(line[3])
                penalty_early = float(line[4])
                penalty_late = float(line[5])
                planes_data.append(
                    {
                        "appearance_time": appearance_time,
                        "earliest_landing_time": earliest_landing_time,
                        "target_landing_time": target_landing_time,
                        "latest_landing_time": latest_landing_time,
                        "penalty_early": penalty_early,
                        "penalty_late": penalty_late,
                    }
                )

                separation_row = []
                while len(separation_row) < num_planes:
                    line = f.readline().strip().split()
                    separation_row.extend([int(x) for x in line])

                separation_times.append(separation_row)

        print("-> Number of planes:", num_planes, "\n")
        
        return num_planes, planes_data, separation_times

    except FileNotFoundError:
        print(f"-> Error: File '{filename}' not found.")
        return None, None, None, None
    except ValueError:
        print(f"-> Error: Error reading data in file '{filename}'.")
        return None, None, None, None

def get_value(variable, approach, solver):
    """
    Retrieves the value of a variable from a dictionary.

    Args:
        dictionary (dict): The dictionary containing the variables.
        variable_name (str): The name of the variable.
        index (int): The index of the variable.
        approach (str): The approach used to solve the problem.

    Returns:
        float: The value of the variable.
    """
    if approach == "CP":
        return solver.Value(variable)
    elif approach == "MIP":
        return variable.solution_value()

    return None

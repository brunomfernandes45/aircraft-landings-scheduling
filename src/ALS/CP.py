from ortools.sat.python import cp_model

def create_or_tools_cp_model_with_permutation(num_planes, num_runways, freeze_time, planes_data, separation_times):
    # Create the CP-SAT model
    model = cp_model.CpModel()

    # ---------------------
    # DATA EXTRACTION
    # ---------------------
    E = [p["earliest_landing_time"] for p in planes_data]  # Earliest possible landing time for each plane
    T = [p["target_landing_time"] for p in planes_data]    # Target landing time for each plane
    L = [p["latest_landing_time"] for p in planes_data]    # Latest possible landing time for each plane
    cost_e = [p["penalty_early"] for p in planes_data]     # Penalty cost for early landing
    cost_l = [p["penalty_late"] for p in planes_data]      # Penalty cost for late landing

    # ---------------------
    # VARIABLE CREATION
    # ---------------------
    # 'position[i]' is the landing order of plane i (0 means lands first, etc.)
    position = [model.NewIntVar(0, num_planes - 1, f"position_{i}") for i in range(num_planes)]
    
    # 'landing_time[i]' is the integer variable indicating the time plane i lands
    landing_time = [model.NewIntVar(0, 10000000, f"landing_time_{i}") for i in range(num_planes)]
    
    # 'earliness[i]' is the non-negative amount of time plane i lands before its target
    earliness = [model.NewIntVar(0, max(T[i] - E[i], 0), f"earliness_{i}") for i in range(num_planes)]
    
    # 'lateness[i]' is the non-negative amount of time plane i lands after its target
    lateness = [model.NewIntVar(0, max(L[i] - T[i], 0), f"lateness_{i}") for i in range(num_planes)]
    
    # 'runway[i]' is the index of the runway on which plane i lands
    runway = [model.NewIntVar(0, num_runways - 1, f"runway_{i}") for i in range(num_planes)]

    # ---------------------
    # BOOLEAN VARIABLE CREATION
    # ---------------------
    # We create 2D lists for the boolean variables iBeforeJ and same_runway
    # iBeforeJ[i][j] = True if plane i is before plane j in the order
    # same_runway[i][j] = True if plane i and plane j land on the same runway
    iBeforeJ = []
    same_runway = []

    for i in range(num_planes):
        iBeforeJ.append([])
        same_runway.append([])
        for j in range(num_planes):
            if j > i:
                iBeforeJ[i].append(model.NewBoolVar(f"iBeforeJ_{i}_{j}"))
                same_runway[i].append(model.NewBoolVar(f"same_runway_{i}_{j}"))
            else:
                # To keep indices consistent, you might store None or a dummy variable for j <= i
                iBeforeJ[i].append(None)
                same_runway[i].append(None)

    # ---------------------
    # CONSTRAINTS
    # ---------------------

    # All planes must have different 'position' (each plane has a unique landing order).
    model.AddAllDifferent(position)

    # Time window constraints: each plane i must land between its earliest and latest times
    # Also, define earliness and lateness relative to the target time
    for i in range(num_planes):
        # landing_time[i] >= earliest landing time
        model.Add(landing_time[i] >= E[i])
        # landing_time[i] <= latest landing time
        model.Add(landing_time[i] <= L[i])
        # earliness[i] >= T[i] - landing_time[i] (early if we land before target)
        model.Add(earliness[i] >= T[i] - landing_time[i])
        # earliness[i] >= 0
        model.Add(earliness[i] >= 0)
        # lateness[i] >= landing_time[i] - T[i] (late if we land after target)
        model.Add(lateness[i] >= landing_time[i] - T[i])
        # lateness[i] >= 0
        model.Add(lateness[i] >= 0)

    # Separation constraints: if plane i lands before j on the same runway,
    # landing_time[j] must be at least landing_time[i] + separation_times[i][j], and vice-versa
    for i in range(num_planes):
        for j in range(i + 1, num_planes):
            # iBeforeJ[i][j] = True if plane i is before j
            model.Add(position[i] < position[j]).OnlyEnforceIf(iBeforeJ[i][j])
            model.Add(position[i] >= position[j]).OnlyEnforceIf(iBeforeJ[i][j].Not())

            # same_runway[i][j] = True if plane i and j use the same runway
            model.Add(runway[i] == runway[j]).OnlyEnforceIf(same_runway[i][j])
            model.Add(runway[i] != runway[j]).OnlyEnforceIf(same_runway[i][j].Not())

            # If plane i is before j and they share the same runway, impose separation times
            model.Add(landing_time[j] >= landing_time[i] + separation_times[i][j])\
                 .OnlyEnforceIf(iBeforeJ[i][j])\
                 .OnlyEnforceIf(same_runway[i][j])

            # If plane j is before i and they share the same runway, impose separation times
            model.Add(landing_time[i] >= landing_time[j] + separation_times[j][i])\
                 .OnlyEnforceIf(iBeforeJ[i][j].Not())\
                 .OnlyEnforceIf(same_runway[i][j])

    # ---------------------
    # OBJECTIVE FUNCTION
    # ---------------------
    # Minimize the total cost of earliness and lateness
    cost_terms = []
    for i in range(num_planes):
        cost_terms.append(cost_e[i] * earliness[i])
        cost_terms.append(cost_l[i] * lateness[i])
    model.Minimize(sum(cost_terms))

    # ---------------------
    # RETURN MODEL & VARS
    # ---------------------
    variables = {
        "position": position,
        "landing_time": landing_time,
        "earliness": earliness,
        "lateness": lateness,
        "runway": runway,
        "iBeforeJ": iBeforeJ,
        "same_runway": same_runway
    }
    return model, variables


def solve_cp_model_with_permutation(num_planes, num_runways, freeze_time, planes_data, separation_times, search_strategy):
    # Create the model and variables
    model, vars_ = create_or_tools_cp_model_with_permutation(
        num_planes,
        num_runways,
        freeze_time,
        planes_data,
        separation_times
    )

    # Create the solver
    solver = cp_model.CpSolver()

    # Example of applying different search strategies:
    # if search_strategy == "default":
    #     pass  # Use default search strategy
    # elif search_strategy == "min_conflicts":
    #     solver.parameters.search_branching = cp_model.VarBranchingPolicy.MIN_CONFLICTS
    # elif search_strategy == "first_fail":
    #     solver.parameters.search_branching = cp_model.VarBranchingPolicy.FIRST_FAIL
    # elif search_strategy == "automatic":
    #     solver.parameters.search_branching = cp_model.VarBranchingPolicy.AUTOMATIC
    # else:
    #     raise ValueError(f"Unknown search strategy: {search_strategy}")

    # Solve the model
    status = solver.Solve(model)

    # If a solution is found, print results
    if status in [cp_model.OPTIMAL, cp_model.FEASIBLE]:
        print("Status:", solver.StatusName(status))
        print("Objective (total earliness + lateness):", solver.ObjectiveValue())
        print()

        position = vars_["position"]
        landing_time = vars_["landing_time"]
        earliness = vars_["earliness"]
        lateness = vars_["lateness"]
        runway = vars_["runway"]

        # Display each plane's assigned values
        for i in range(num_planes):
            pos_val = solver.Value(position[i])
            t_val = solver.Value(landing_time[i])
            e_ = solver.Value(earliness[i])
            L_ = solver.Value(lateness[i])
            T_ = planes_data[i]["target_landing_time"]
            r_ = solver.Value(runway[i])
            print(f"Plane {i}: position={pos_val}, landing={t_val}, runway={r_}, E'={e_}, L'={L_}, target={T_}")

        # Display the schedule order based on 'position'
        print("\nSchedule order (by position):")
        schedule = sorted(range(num_planes), key=lambda i: solver.Value(position[i]))
        for k in schedule:
            print(f" -> Plane {k} (pos={solver.Value(position[k])}, "
                  f"land={solver.Value(landing_time[k])}, runway={solver.Value(runway[k])})")

    else:
        print("No feasible/optimal solution found. Status:", solver.StatusName(status))

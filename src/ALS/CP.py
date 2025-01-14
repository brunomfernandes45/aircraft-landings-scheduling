from ortools.sat.python import cp_model
import psutil

#----------------------------
# SINGLE_RUNWAY
#----------------------------

def create_cp_model_single_runways(num_planes, planes_data, separation_times):
    # Create the CP-SAT model
    model = cp_model.CpModel()

    # ------------------------------------------------------------------
    # 1) EXTRACT RELEVANT DATA INTO ARRAYS (for convenience)
    # ------------------------------------------------------------------
    E = [p["earliest_landing_time"] for p in planes_data]  # Earliest landing times
    T = [p["target_landing_time"]   for p in planes_data]  # Target landing times
    L = [p["latest_landing_time"]   for p in planes_data]  # Latest landing times
    cost_e = [p["penalty_early"]    for p in planes_data]  # Penalty for earliness
    cost_l = [p["penalty_late"]     for p in planes_data]  # Penalty for lateness

    # ------------------------------------------------------------------
    # 2) VARIABLE CREATION
    # ------------------------------------------------------------------

    # 'position[i]' determines the landing order of plane i
    position = [
        model.NewIntVar(0, num_planes - 1, f"position_{i}")
        for i in range(num_planes)
    ]

    # 'landing_time[i]' is the time plane i actually lands
    landing_time = [
        model.NewIntVar(0, 10_000_000, f"landing_time_{i}")
        for i in range(num_planes)
    ]

    # 'earliness[i]' measures how many time units plane i lands before target
    earliness = [
        model.NewIntVar(
            0,
            max(T[i] - E[i], 0),  # Max possible earliness
            f"earliness_{i}")
        for i in range(num_planes)]

    # 'lateness[i]' measures how many time units plane i lands after target
    lateness = [
        model.NewIntVar(
            0,
            max(L[i] - T[i], 0),  # Max possible lateness
            f"lateness_{i}")
        for i in range(num_planes)]

    # Boolean variables: iBeforeJ[i][j] = True if plane i lands before plane j (i < j).
    # We'll store these in a 2D list for convenience.
    iBeforeJ = []
    for i in range(num_planes):
        row = []
        for j in range(num_planes):
            if j > i:
                # Only define it for j > i to avoid duplication
                row.append(model.NewBoolVar(f"iBeforeJ_{i}_{j}"))
            else:
                # For j <= i, we can store None (or a dummy variable)
                row.append(None)
        iBeforeJ.append(row)

    # ------------------------------------------------------------------
    # 3) CONSTRAINTS
    # ------------------------------------------------------------------

    # (3.1) All-Different for positions to enforce a permutation
    model.AddAllDifferent(position)

    # (3.2) Earliest/latest landing time constraints
    for i in range(num_planes):
        model.Add(landing_time[i] >= E[i])  # no earlier than E[i]
        model.Add(landing_time[i] <= L[i])  # no later than L[i]

    # (3.3) Earliness / lateness definitions
    for i in range(num_planes):
        model.Add(earliness[i] >= T[i] - landing_time[i])
        model.Add(earliness[i] >= 0)
        model.Add(lateness[i]  >= landing_time[i] - T[i])
        model.Add(lateness[i]  >= 0)

    # (3.4) Separation constraints using the boolean iBeforeJ
    # For each pair (i, j) with i < j, if plane i lands before j, then
    # landing_time[j] >= landing_time[i] + separation_times[i][j].
    # Otherwise, landing_time[i] >= landing_time[j] + separation_times[j][i].
    for i in range(num_planes):
        for j in range(i + 1, num_planes):
            # iBeforeJ[i][j] <-> (position[i] < position[j])
            model.Add(position[i] < position[j]).OnlyEnforceIf(iBeforeJ[i][j])
            model.Add(position[i] >= position[j]).OnlyEnforceIf(iBeforeJ[i][j].Not())

            # If i lands before j:
            model.Add(
                landing_time[j] >= landing_time[i] + separation_times[i][j]
            ).OnlyEnforceIf(iBeforeJ[i][j])

            # If j lands before i:
            model.Add(
                landing_time[i] >= landing_time[j] + separation_times[j][i]
            ).OnlyEnforceIf(iBeforeJ[i][j].Not())

    # ------------------------------------------------------------------
    # 4) OBJECTIVE FUNCTION: MINIMIZE TOTAL EARLINESS + LATENESS COST
    # ------------------------------------------------------------------
    cost_terms = []
    for i in range(num_planes):
        cost_terms.append(cost_e[i] * earliness[i])
        cost_terms.append(cost_l[i] * lateness[i])

    model.Minimize(sum(cost_terms))

    # ------------------------------------------------------------------
    # 5) RETURN MODEL AND VARIABLES
    # ------------------------------------------------------------------
    variables = {
        "position": position,
        "landing_time": landing_time,
        "earliness": earliness,
        "lateness": lateness,
        "iBeforeJ": iBeforeJ
    }

    return model, variables


def solve_single_runway_cp(num_planes, planes_data, separation_times):
    """Builds and solves the single-runway CP model with a permutation approach."""
    model, vars_ = create_cp_model_single_runways(
        num_planes, planes_data, separation_times
    )

    # Create solver instance
    solver = cp_model.CpSolver()

    # Memory Usage before the Solver
    memory_before = psutil.Process().memory_info().rss  # Memory in bytes

    # Solve the model with performance tracking
    status = solver.Solve(model)
    
    # Memory Usage after the Solver
    memory_after = psutil.Process().memory_info().rss 
    
    if status in [cp_model.OPTIMAL, cp_model.FEASIBLE]:
        print("Status:", solver.StatusName(status))
        print("Objective:", solver.ObjectiveValue())
        print()

        position = vars_["position"]
        landing_time = vars_["landing_time"]
        earliness = vars_["earliness"]
        lateness = vars_["lateness"]
        iBeforeJ = vars_["iBeforeJ"]

        for i in range(num_planes):
            pos_val = solver.Value(position[i])
            t_val = solver.Value(landing_time[i])
            e_ = solver.Value(earliness[i])
            L_ = solver.Value(lateness[i])
            T_ = planes_data[i]["target_landing_time"]
            print(f"Plane {i}: position={pos_val}, landing={t_val}, E'={e_}, L'={L_}, target={T_}")

        print("\nSchedule order (by position):")
        schedule = sorted(range(num_planes), key=lambda i: solver.Value(position[i]))
        for k in schedule:
            print(f" -> Plane {k} (pos={solver.Value(position[k])}, land={solver.Value(landing_time[k])})")

    else:
        print("No feasible/optimal solution found. Status:", solver.StatusName(status))

    return solver, memory_before, memory_after

#----------------------------
# MULTIPLE RUNWAYS
#----------------------------


def create_cp_model_multiple_runways(num_planes, num_runways, planes_data, separation_times):
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


def solve_multiple_runways_cp(num_planes, num_runways, planes_data, separation_times):
    # Create the model and variables
    model, vars_ = create_cp_model_multiple_runways(
        num_planes,
        num_runways,
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

    # Memory Usage before the Solver
    memory_before = psutil.Process().memory_info().rss  # Memory in bytes

    # Solve the model with performance tracking
    status = solver.Solve(model)
    
    # Memory Usage after the Solver
    memory_after = psutil.Process().memory_info().rss 
    
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
        
    return solver, memory_before, memory_after

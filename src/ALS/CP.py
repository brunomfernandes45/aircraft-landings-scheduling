from ortools.sat.python import cp_model
import psutil

# ----------------------------
# SINGLE_RUNWAY
# ----------------------------

def create_cp_model_single_runway(num_planes, planes_data, separation_times):
    print("=" * 60)
    print("\t\t     Creating CP model") 
    print("=" * 60, "\n")
    
    # Create the CP-SAT model
    model = cp_model.CpModel()

    # ------------------------------------------------------------------
    # 1) EXTRACT RELEVANT DATA INTO ARRAYS (for convenience)
    # ------------------------------------------------------------------
    E = [p["earliest_landing_time"] for p in planes_data]  # Earliest landing times
    T = [p["target_landing_time"]   for p in planes_data]  # Target landing times
    L = [p["latest_landing_time"]   for p in planes_data]  # Latest landing times
    cost_e = [p["penalty_early"]    for p in planes_data]  # Penalty for early_deviation
    cost_l = [p["penalty_late"]     for p in planes_data]  # Penalty for late_deviation

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

    # 'early_deviation[i]' measures how many time units plane i lands before target
    early_deviation = [
        model.NewIntVar(
            0,
            max(T[i] - E[i], 0),  # Max possible early_deviation
            f"early_deviation_{i}")
        for i in range(num_planes)]

    # 'late_deviation[i]' measures how many time units plane i lands after target
    late_deviation = [
        model.NewIntVar(
            0,
            max(L[i] - T[i], 0),  # Max possible late_deviation
            f"late_deviation_{i}")
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

    # (3.3) early_deviation / late_deviation definitions
    for i in range(num_planes):
        model.Add(early_deviation[i] >= T[i] - landing_time[i])
        model.Add(early_deviation[i] >= 0)
        model.Add(late_deviation[i]  >= landing_time[i] - T[i])
        model.Add(late_deviation[i]  >= 0)

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
    # 4) OBJECTIVE FUNCTION: MINIMIZE TOTAL early_deviation + late_deviation COST
    # ------------------------------------------------------------------
    cost_terms = []
    for i in range(num_planes):
        cost_terms.append(cost_e[i] * early_deviation[i])
        cost_terms.append(cost_l[i] * late_deviation[i])

    model.Minimize(sum(cost_terms))

    # ------------------------------------------------------------------
    # 5) RETURN MODEL AND VARIABLES
    # ------------------------------------------------------------------
    variables = {
        "position": position,
        "landing_time": landing_time,
        "early_deviation": early_deviation,
        "late_deviation": late_deviation,
        "iBeforeJ": iBeforeJ
    }

    return model, variables


def solve_single_runway_cp(num_planes, planes_data, separation_times, decision_strategies=None,hint=False, search_strategy=cp_model.AUTOMATIC_SEARCH):
    """Builds and solves the single-runway CP model with a permutation approach."""
    model, vars_ = create_cp_model_single_runway(
        num_planes, planes_data, separation_times
    )

    if hint:
        for i in range(num_planes):
            model.AddHint(vars_["landing_time"][i], planes_data[i]["target_landing_time"])

    # Create solver instance
    solver = cp_model.CpSolver()

    # Set search strategy
    solver.parameters.search_branching = search_strategy

    print("-> Number of decision variables created:", len(model.Proto().variables))
    print("-> Number of constraints:", len(model.Proto().constraints))

    print("\n" + "=" * 60)
    print("\t\t\tSolving CP")
    print("=" * 60, "\n")


    if decision_strategies:
        for strategy in decision_strategies:
            # Obter as variáveis a partir do nome fornecido
            var_names = strategy["variables"]
            if isinstance(var_names, str):
                var_list = vars_.get(var_names, [])
            elif isinstance(var_names, list):
                var_list = []
                for var_name in var_names:
                    var_list.extend(vars_.get(var_name, []))
            else:
                raise ValueError("The 'variables' field must be a string or list of strings.")
            
            # Aplicar a estratégia ao conjunto de variáveis
            model.AddDecisionStrategy(
                var_list,
                strategy["variable_strategy"],
                strategy["value_strategy"]
            )

    # Solve the model with performance tracking
    status = solver.Solve(model)

    position = vars_["position"]
    landing_time = vars_["landing_time"]
    early_deviation = vars_["early_deviation"]
    late_deviation = vars_["late_deviation"]
    
    if status == cp_model.OPTIMAL:
        # -------------------------------
        # Print as if it were MIP's OPTIMAL
        # -------------------------------
        print(f"-> Optimal Cost: {solver.ObjectiveValue()}")

        print("\n-> Planes that did not land on the target time:")
        for i in range(num_planes):
            e_ = solver.Value(early_deviation[i])
            L_ = solver.Value(late_deviation[i])
            # If early_deviation or late_deviation > 0, plane missed its target
            if e_ > 0 or L_ > 0:
                # Calculate penalty
                penalty = (
                    e_ * planes_data[i]["penalty_early"]
                    + L_ * planes_data[i]["penalty_late"]
                )
                landing_t = solver.Value(landing_time[i])
                target_t = planes_data[i]["target_landing_time"]
                print(
                    f"  -> Plane {i}: {landing_t} | Target Time: {target_t} | Penalty: {penalty}"
                )

    elif status == cp_model.FEASIBLE:
        # -------------------------------
        # No optimal solution
        # -------------------------------
        print("-> Best feasible solution found:", round(solver.ObjectiveValue(), 2))

        # You can optionally also list planes that missed their target
        print("\n-> Planes that did not land on the target time:")
        for i in range(num_planes):
            e_ = solver.Value(early_deviation[i])
            L_ = solver.Value(late_deviation[i])
            if e_ > 0 or L_ > 0:
                penalty = (
                    e_ * planes_data[i]["penalty_early"]
                    + L_ * planes_data[i]["penalty_late"]
                )
                landing_t = solver.Value(landing_time[i])
                target_t = planes_data[i]["target_landing_time"]
                print(
                    f"  -> Plane {i}: {landing_t} | Target Time: {target_t} | Penalty: {penalty}"
                )

    else:
        print("-> No feasible/optimal solution found. Status:", solver.StatusName(status))

    return solver, model, vars_

# ----------------------------
# MULTIPLE RUNWAYS
# ----------------------------


def create_cp_model_multiple_runways(num_planes, num_runways, planes_data, separation_times):
    print("=" * 60)
    print("\t\t     Creating CP model")
    print("=" * 60, "\n")

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
    
    # 'early_deviation[i]' is the non-negative amount of time plane i lands before its target
    early_deviation = [model.NewIntVar(0, max(T[i] - E[i], 0), f"early_deviation_{i}") for i in range(num_planes)]
    
    # 'late_deviation[i]' is the non-negative amount of time plane i lands after its target
    late_deviation = [model.NewIntVar(0, max(L[i] - T[i], 0), f"late_deviation_{i}") for i in range(num_planes)]
    
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
    # Also, define early_deviation and late_deviation relative to the target time
    for i in range(num_planes):
        # landing_time[i] >= earliest landing time
        model.Add(landing_time[i] >= E[i])
        # landing_time[i] <= latest landing time
        model.Add(landing_time[i] <= L[i])
        # early_deviation[i] >= T[i] - landing_time[i] (early if we land before target)
        model.Add(early_deviation[i] >= T[i] - landing_time[i])
        # early_deviation[i] >= 0
        model.Add(early_deviation[i] >= 0)
        # late_deviation[i] >= landing_time[i] - T[i] (late if we land after target)
        model.Add(late_deviation[i] >= landing_time[i] - T[i])
        # late_deviation[i] >= 0
        model.Add(late_deviation[i] >= 0)

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
    # Minimize the total cost of early_deviation and late_deviation
    cost_terms = []
    for i in range(num_planes):
        cost_terms.append(cost_e[i] * early_deviation[i])
        cost_terms.append(cost_l[i] * late_deviation[i])
    model.Minimize(sum(cost_terms))

    # ---------------------
    # RETURN MODEL & VARS
    # ---------------------
    variables = {
        "position": position,
        "landing_time": landing_time,
        "early_deviation": early_deviation,
        "late_deviation": late_deviation,
        "runway": runway,
        "iBeforeJ": iBeforeJ,
        "same_runway": same_runway
    }
    


    return model, variables


def solve_multiple_runways_cp(num_planes, num_runways, planes_data, separation_times, decision_strategies=None, hint=False,search_strategy=cp_model.AUTOMATIC_SEARCH):
    # Create the model and variables
    model, vars_ = create_cp_model_multiple_runways(
        num_planes,
        num_runways,
        planes_data,
        separation_times
    )

    if hint:
        for i in range(num_planes):
            model.AddHint(vars_["landing_time"][i], planes_data[i]["target_landing_time"])

    # Create the solver
    solver = cp_model.CpSolver()
    
    # Set search strategy
    solver.parameters.search_branching = search_strategy


    print("-> Number of decision variables created:", len(model.Proto().variables))
    print("-> Number of constraints:", len(model.Proto().constraints))

    print("\n" + "=" * 60)
    print("\t\t\tSolving CP")
    print("=" * 60, "\n")

    if decision_strategies:
        for strategy in decision_strategies:
            # Obter as variáveis a partir do nome fornecido
            var_names = strategy["variables"]
            if isinstance(var_names, str):
                var_list = vars_.get(var_names, [])
            elif isinstance(var_names, list):
                var_list = []
                for var_name in var_names:
                    var_list.extend(vars_.get(var_name, []))
            else:
                raise ValueError("The 'variables' field must be a string or list of strings.")

            # Aplicar a estratégia ao conjunto de variáveis
            model.AddDecisionStrategy(
                var_list,
                strategy["variable_strategy"],
                strategy["value_strategy"]
            )

    # Solve the model
    status = solver.Solve(model)

    # Unpack variables for easy reference
    position = vars_["position"]
    landing_time = vars_["landing_time"]
    early_deviation = vars_["early_deviation"]
    late_deviation = vars_["late_deviation"]
    runway = vars_["runway"]

    # 1) Check solver status for CP-SAT
    if status == cp_model.OPTIMAL:
        # -------------------------------
        # Print as if it were MIP's OPTIMAL
        # -------------------------------
        print(f"-> Optimal Cost: {solver.ObjectiveValue()}")

        print("\n-> Planes that did not land on the target time:")
        for i in range(num_planes):
            e_ = solver.Value(early_deviation[i])
            L_ = solver.Value(late_deviation[i])
            # If early_deviation or late_deviation > 0, plane missed its target
            if e_ > 0 or L_ > 0:
                # Calculate penalty
                penalty = (
                    e_ * planes_data[i]["penalty_early"]
                    + L_ * planes_data[i]["penalty_late"]
                )
                landing_t = solver.Value(landing_time[i])
                target_t = planes_data[i]["target_landing_time"]
                print(
                    f"  -> Plane {i}: {landing_t} | Target Time: {target_t} | Penalty: {penalty}"
                )

    elif status == cp_model.FEASIBLE:
        # -------------------------------
        # No optimal solution
        # -------------------------------
        print("-> No optimal solution found.")
        print("-> Best feasible solution found:", round(solver.ObjectiveValue(), 2))

        # You can optionally also list planes that missed their target
        print("\n-> Planes that did not land on the target time:")
        for i in range(num_planes):
            e_ = solver.Value(early_deviation[i])
            L_ = solver.Value(late_deviation[i])
            if e_ > 0 or L_ > 0:
                penalty = (
                    e_ * planes_data[i]["penalty_early"]
                    + L_ * planes_data[i]["penalty_late"]
                )
                landing_t = solver.Value(landing_time[i])
                target_t = planes_data[i]["target_landing_time"]
                print(
                    f"  -> Plane {i}: {landing_t} | Target Time: {target_t} | Penalty: {penalty}"
                )
    else:
        print("-> No feasible/optimal solution found. Status:", solver.StatusName(status))

    # Return solver
    return solver, model


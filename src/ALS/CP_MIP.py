from ortools.sat.python import cp_model


def create_cp_model_multiple_runways(
    num_planes,
    planes_data,
    separation_times,
    num_runways,
):
    print("=" * 60)
    print("\t\t    Creating CP-SAT Model")
    print("=" * 60, "\n")

    model = cp_model.CpModel()
    variables = {}

    # Decision Variables
    # x_i: Landing time for plane i
    landing_times = [
        model.NewIntVar(
            planes_data[i]["earliest_landing_time"],
            planes_data[i]["latest_landing_time"],
            f"LandingTime_{i}",
        )
        for i in range(num_planes)
    ]
    variables["landing_time"] = landing_times

    # delta_ij:  Binary variable representing if plane i lands before plane j
    landing_order = {}
    for i in range(num_planes):
        for j in range(num_planes):
            if i != j:
                landing_order[(i, j)] = model.NewBoolVar(f"LandingOrder_{i}_{j}")
    variables["landing_order"] = landing_order

    # alpha_i: Time by which plane i lands before its target time
    early_deviation = [
        model.NewIntVar(
            0,
            max(
                planes_data[i]["target_landing_time"]
                - planes_data[i]["earliest_landing_time"],
                0,
            ),
            f"EarlyDeviation_{i}",
        )
        for i in range(num_planes)
    ]
    variables["early_deviation"] = early_deviation

    # beta_i: Time by which plane i lands after its target time
    late_deviation = [
        model.NewIntVar(
            0,
            max(
                planes_data[i]["latest_landing_time"]
                - planes_data[i]["target_landing_time"],
                0,
            ),
            f"LateDeviation_{i}",
        )
        for i in range(num_planes)
    ]
    variables["late_deviation"] = late_deviation

    # z_ij:  Binary variable indicating if plane i and plane j land on the same runway
    same_runway = {}
    for i in range(num_planes):
        for j in range(num_planes):
            if i != j:
                same_runway[(i, j)] = model.NewBoolVar(f"SameRunway_{i}_{j}")

    variables["same_runway"] = same_runway

    # y_ir:  Binary variable indicating if plane i lands on runway r
    landing_runway = {}
    for i in range(num_planes):
        for r in range(num_runways):
            landing_runway[(i, r)] = model.NewBoolVar(f"LandingRunway_{i}_{r}")

    variables["landing_runway"] = landing_runway

    # Constraints
    # (2) Each pair of planes must have an order
    for i in range(num_planes):
        for j in range(i + 1, num_planes):
            model.Add(landing_order[(i, j)] + landing_order[(j, i)] == 1)

    # Set W: Pairs with certain separation
    certain_with_separation_pairs = []
    for i in range(num_planes):
        for j in range(num_planes):
            if i != j:
                latest_i = planes_data[i]["latest_landing_time"]
                earliest_j = planes_data[j]["earliest_landing_time"]
                separation_ij = separation_times[i][j]
                if latest_i < earliest_j and latest_i + separation_ij <= earliest_j:
                    certain_with_separation_pairs.append((i, j))

    # Set V: Pairs with certain order but separation not automatic
    certain_with_no_separation_pairs = []
    for i in range(num_planes):
        for j in range(num_planes):
            if i != j:
                latest_i = planes_data[i]["latest_landing_time"]
                earliest_j = planes_data[j]["earliest_landing_time"]
                separation_ij = separation_times[i][j]
                if latest_i < earliest_j and latest_i + separation_ij > earliest_j:
                    certain_with_no_separation_pairs.append((i, j))

    # Set U: Uncertain pairs
    uncertain_pairs = []
    for i in range(num_planes):
        for j in range(num_planes):
            if i != j:
                earliest_i = planes_data[i]["earliest_landing_time"]
                latest_i = planes_data[i]["latest_landing_time"]
                earliest_j = planes_data[j]["earliest_landing_time"]
                latest_j = planes_data[j]["latest_landing_time"]

                if (
                    (earliest_j <= earliest_i <= latest_j)
                    or (earliest_j <= latest_i <= latest_j)
                ) or (
                    (earliest_i <= earliest_j <= latest_i)
                    or (earliest_i <= latest_j <= latest_i)
                ):
                    uncertain_pairs.append((i, j))

    # Enforce separation for pairs where order is determined (Set V)
    for i, j in certain_with_no_separation_pairs:
        model.Add(landing_order[(i, j)] == 1)  # (6)
        model.Add(
            landing_times[j]
            >= landing_times[i] + separation_times[i][j] * same_runway[(i, j)]
        )  # (7)

    # Enforce order for pairs where order is determined and separation is automatic (Set W)
    for i, j in certain_with_separation_pairs:
        model.Add(landing_order[(i, j)] == 1)  # (6)

    # Enforce separation for uncertain pairs
    for i, j in uncertain_pairs:
        # solver.Add(landing_times[j] >= landing_times[i] + separation_times[i][j] * same_runway[(i, j)] + separation_times_between_runways[(i, j)] * (1 - same_runway[(i, j)]) - M * (landing_order[(j, i)])) # (8)
        latest_i = planes_data[i]["latest_landing_time"]
        earliest_j = planes_data[j]["earliest_landing_time"]

        model.Add(
            landing_times[j]
            >= landing_times[i]
            + separation_times[i][j] * same_runway[(i, j)]
            - (latest_i + separation_times[i][j] - earliest_j) * (landing_order[(j, i)])
        )  # (8)

    # Relating Deviation Variables to Landing Times
    for i in range(num_planes):
        earliest_i = planes_data[i]["earliest_landing_time"]
        latest_i = planes_data[i]["latest_landing_time"]
        target_i = planes_data[i]["target_landing_time"]

        # (14)
        model.Add(early_deviation[i] >= target_i - landing_times[i])

        # (15)
        model.Add(early_deviation[i] >= 0)
        model.Add(early_deviation[i] <= target_i - earliest_i)

        # (16)
        model.Add(late_deviation[i] >= landing_times[i] - target_i)

        # (17)
        model.Add(late_deviation[i] >= 0)
        model.Add(late_deviation[i] <= latest_i - target_i)

        # (18)
        model.Add(
            landing_times[i] == target_i - early_deviation[i] + late_deviation[i]
        )

    # New constraints for multiple runways
    # (28) Each plane must land on exactly one runway
    for i in range(num_planes):
        model.Add(
            sum(landing_runway[(i, r)] for r in range(num_runways)) == 1
        )

    # (29) same_runway is symmetric
    for i in range(num_planes):
        for j in range(i + 1, num_planes):
            model.Add(same_runway[(i, j)] == same_runway[(j, i)])

    # (30) same_runway is true if both planes are assigned to the same runway
    for i in range(num_planes):
        for j in range(i + 1, num_planes):
            for r in range(num_runways):
                model.Add(
                    same_runway[(i, j)]
                    >= landing_runway[(i, r)] + landing_runway[(j, r)] - 1
                )

    # Objective: Minimize total penalties
    objective_terms = []
    for i in range(num_planes):
        objective_terms.append(
            planes_data[i]["penalty_early"] * early_deviation[i]
        )
        objective_terms.append(
            planes_data[i]["penalty_late"] * late_deviation[i]
        )

    model.Minimize(sum(objective_terms))

    # Display the number of variables and constraints
    print("-> Number of decision variables created:", model.Proto().variables.__len__())
    print("-> Number of constraints:", model.Proto().constraints.__len__())

    return model, variables


def solve_cp_sat_model_multiple_runways(num_planes, num_runways, planes_data, separation_times, hint=False):
    model, variables = create_cp_model_multiple_runways(
        num_planes, planes_data, separation_times, num_runways
    )

    if hint:
        for i in range(num_planes):
            model.AddHint(variables["landing_time"][i], planes_data[i]["target_landing_time"])

    print("\n" + "=" * 60)
    print("\t\t\tSolving CP-SAT Model")
    print("=" * 60, "\n")

    # Create solver and solve
    solver = cp_model.CpSolver()
    status = solver.Solve(model)

    landing_time = variables["landing_time"]
    early_deviation = variables["early_deviation"]
    late_deviation = variables["late_deviation"]

    if status == cp_model.OPTIMAL or status == cp_model.FEASIBLE:
        if status == cp_model.OPTIMAL:
            print(f"-> Optimal Cost: {solver.ObjectiveValue()}")
        else:
            print("-> Best feasible solution found.")

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
    else:
        print(
            "-> No feasible/optimal solution found. Status:", solver.StatusName(status)
        )

    # Optionally, you can return the solver for further inspection
    return solver, model


from ortools.sat.python import cp_model


def create_cp_model_single_runway(num_planes, planes_data, separation_times):
    print("=" * 60)
    print("\t\t    Creating CP-SAT Model")
    print("=" * 60, "\n")

    model = cp_model.CpModel()
    variables = {}

    # Decision Variables
    # x_i: Landing time for plane i
    landing_times = [
        model.NewIntVar(
            planes_data[i]["earliest_landing_time"],
            planes_data[i]["latest_landing_time"],
            f"LandingTime_{i}",
        )
        for i in range(num_planes)
    ]
    variables["landing_time"] = landing_times

    # delta_ij:  Binary variable representing if plane i lands before plane j
    landing_order = {}
    for i in range(num_planes):
        for j in range(num_planes):
            if i != j:
                landing_order[(i, j)] = model.NewBoolVar(f"LandingOrder_{i}_{j}")
    variables["landing_order"] = landing_order

    # alpha_i: Time by which plane i lands before its target time
    early_deviation = [
        model.NewIntVar(
            0,
            max(
                planes_data[i]["target_landing_time"]
                - planes_data[i]["earliest_landing_time"],
                0,
            ),
            f"EarlyDeviation_{i}",
        )
        for i in range(num_planes)
    ]
    variables["early_deviation"] = early_deviation

    # beta_i: Time by which plane i lands after its target time
    late_deviation = [
        model.NewIntVar(
            0,
            max(
                planes_data[i]["latest_landing_time"]
                - planes_data[i]["target_landing_time"],
                0,
            ),
            f"LateDeviation_{i}",
        )
        for i in range(num_planes)
    ]
    variables["late_deviation"] = late_deviation

    # Constraints
    # (2) Each pair of planes must have an order
    for i in range(num_planes):
        for j in range(i + 1, num_planes):
            model.Add(landing_order[(i, j)] + landing_order[(j, i)] == 1)

    # Set V: Pairs with certain order but separation not automatic
    certain_with_no_separation_pairs = []
    for i in range(num_planes):
        for j in range(num_planes):
            if i != j:
                latest_i = planes_data[i]["latest_landing_time"]
                earliest_j = planes_data[j]["earliest_landing_time"]
                separation_ij = separation_times[i][j]
                if latest_i < earliest_j and latest_i + separation_ij > earliest_j:
                    certain_with_no_separation_pairs.append((i, j))

    # Set W: Pairs with certain separation
    certain_with_separation_pairs = []
    for i in range(num_planes):
        for j in range(num_planes):
            if i != j:
                latest_i = planes_data[i]["latest_landing_time"]
                earliest_j = planes_data[j]["earliest_landing_time"]
                separation_ij = separation_times[i][j]
                if latest_i < earliest_j and latest_i + separation_ij <= earliest_j:
                    certain_with_separation_pairs.append((i, j))

    # Set U: Uncertain pairs
    uncertain_pairs = []
    for i in range(num_planes):
        for j in range(num_planes):
            if i != j:
                earliest_i = planes_data[i]["earliest_landing_time"]
                latest_i = planes_data[i]["latest_landing_time"]
                earliest_j = planes_data[j]["earliest_landing_time"]
                latest_j = planes_data[j]["latest_landing_time"]

                if (
                    (earliest_j <= earliest_i <= latest_j)
                    or (earliest_j <= latest_i <= latest_j)
                ) or (
                    (earliest_i <= earliest_j <= latest_i)
                    or (earliest_i <= latest_j <= latest_i)
                ):
                    uncertain_pairs.append((i, j))

    # Enforce separation for pairs where order is determined (Set V)
    for i, j in certain_with_no_separation_pairs:
        model.Add(landing_order[(i, j)] == 1)
        model.Add(landing_times[j] >= landing_times[i] + separation_times[i][j])

    # Enforce order for pairs where order is determined and separation is automatic (Set W)
    for i, j in certain_with_separation_pairs:
        model.Add(landing_order[(i, j)] == 1)

    # Enforce separation for uncertain pairs
    for i, j in uncertain_pairs:
        latest_i = planes_data[i]["latest_landing_time"]
        earliest_j = planes_data[j]["earliest_landing_time"]
        separation_ij = separation_times[i][j]
        delta_ij = landing_order[(i, j)]
        delta_ji = landing_order[(j, i)]

        # Constraint (11)
        model.Add(
            landing_times[j]
            >= landing_times[i]
            + separation_ij * delta_ij
            - (latest_i - earliest_j) * delta_ji
        )

    # Relating Deviation Variables to Landing Times
    for i in range(num_planes):
        earliest_i = planes_data[i]["earliest_landing_time"]
        latest_i = planes_data[i]["latest_landing_time"]
        target_i = planes_data[i]["target_landing_time"]

        # (14)
        model.Add(early_deviation[i] >= target_i - landing_times[i])

        # (15)
        model.Add(early_deviation[i] >= 0)
        model.Add(early_deviation[i] <= target_i - earliest_i)

        # (16)
        model.Add(late_deviation[i] >= landing_times[i] - target_i)

        # (17)
        model.Add(late_deviation[i] >= 0)
        model.Add(late_deviation[i] <= latest_i - target_i)

        # (18)
        model.Add(
            landing_times[i] == target_i - early_deviation[i] + late_deviation[i]
        )

    # Objective: Minimize total penalties
    objective_terms = []
    for i in range(num_planes):
        objective_terms.append(
            planes_data[i]["penalty_early"] * early_deviation[i]
        )
        objective_terms.append(
            planes_data[i]["penalty_late"] * late_deviation[i]
        )

    model.Minimize(sum(objective_terms))

    # Display the number of variables and constraints
    print("-> Number of decision variables created:", len(model.Proto().variables))
    print("-> Number of constraints:", len(model.Proto().constraints))

    return model, variables


def solve_cp_sat_model_single_runway(num_planes, planes_data, separation_times, hint=False):
    model, variables = create_cp_model_single_runway(
        num_planes, planes_data, separation_times
    )

    if hint:
        for i in range(num_planes):
            model.AddHint(variables["landing_time"][i], planes_data[i]["target_landing_time"])

    print("\n" + "=" * 60)
    print("\t\t\tSolving CP-SAT Model")
    print("=" * 60, "\n")

    # Create solver and solve
    solver = cp_model.CpSolver()
    status = solver.Solve(model)

    landing_time = variables["landing_time"]
    early_deviation = variables["early_deviation"]
    late_deviation = variables["late_deviation"]

    if status == cp_model.OPTIMAL or status == cp_model.FEASIBLE:
        if status == cp_model.OPTIMAL:
            print(f"-> Optimal Cost: {solver.ObjectiveValue()}")
        else:
            print("-> Best feasible solution found.")

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
    else:
        print(
            "-> No feasible/optimal solution found. Status:", solver.StatusName(status)
        )

    # Optionally, you can return the solver for further inspection
    return solver, model, variables

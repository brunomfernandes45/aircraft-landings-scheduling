from ortools.linear_solver import pywraplp
import psutil


def create_mip_model_multiple_runways(
    num_planes,
    planes_data,
    separation_times,
    num_runways,
):
    print("=" * 60)
    print("\t\t    Creating MIP Solver")
    print("=" * 60, "\n")

    # Create the LP solver
    solver = pywraplp.Solver.CreateSolver("SAT")  # Using the SAT solver
    variables = {}

    # Decision Variables
    # x_i: Landing time for plane i
    # (1)
    landing_times = [
        solver.NumVar(
            planes_data[i]["earliest_landing_time"],
            planes_data[i]["latest_landing_time"],
            f"LandingTime_{i}",
        )
        for i in range(num_planes)
    ]
    variables["landing_times"] = landing_times

    # delta_ij:  Fraction representing if plane i lands before plane j (0 to 1)
    # Note: In a pure LP model, we relax the integrality constraint.
    landing_order = {}
    for i in range(num_planes):
        for j in range(num_planes):
            if i != j:
                landing_order[(i, j)] = solver.NumVar(0, 1, f"LandingOrder_{i}_{j}")
    variables["landing_order"] = landing_order

    # alpha_i: Time by which plane i lands before its target time
    early_deviation = [
        solver.NumVar(
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
        solver.NumVar(
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

    # z_ij: 1 if plane i and plane j land on the same runway, 0 otherwise
    same_runway = {}
    for i in range(num_planes):
        for j in range(num_planes):
            if i != j:
                same_runway[(i, j)] = solver.NumVar(0, 1, f"SameRunway_{i}_{j}")

    variables["same_runway"] = same_runway

    # y_ir: 1 if plane i lands on runway r, 0 otherwise
    landing_runway = {}
    for i in range(num_planes):
        for r in range(num_runways):
            landing_runway[(i, r)] = solver.NumVar(0, 1, f"LandingRunway_{i}_{r}")

    variables["landing_runway"] = landing_runway

    # Constraints
    # (2)
    for i in range(num_planes):
        for j in range(i + 1, num_planes):
            solver.Add(landing_order[(i, j)] + landing_order[(j, i)] == 1)

    # Set W
    # (3)
    certain_with_separation_pairs = []
    for i in range(num_planes):
        for j in range(num_planes):
            if i != j:
                earliest_i, latest_i = (
                    planes_data[i]["earliest_landing_time"],
                    planes_data[i]["latest_landing_time"],
                )
                earliest_j = planes_data[j]["earliest_landing_time"]
                separation_ij = separation_times[i][j]
                if latest_i < earliest_j and latest_i + separation_ij <= earliest_j:
                    certain_with_separation_pairs.append((i, j))

    # Set V
    # (4)
    certain_with_no_separation_pairs = []
    for i in range(num_planes):
        for j in range(num_planes):
            if i != j:
                earliest_i, latest_i = (
                    planes_data[i]["earliest_landing_time"],
                    planes_data[i]["latest_landing_time"],
                )
                earliest_j = planes_data[j]["earliest_landing_time"]
                separation_ij = separation_times[i][j]
                if latest_i < earliest_j and latest_i + separation_ij > earliest_j:
                    certain_with_no_separation_pairs.append((i, j))

    # Set U
    # (5)
    uncertain_pairs = []
    for i in range(num_planes):
        for j in range(num_planes):
            if i != j:
                earliest_i, latest_i = (
                    planes_data[i]["earliest_landing_time"],
                    planes_data[i]["latest_landing_time"],
                )
                earliest_j, latest_j = (
                    planes_data[j]["earliest_landing_time"],
                    planes_data[j]["latest_landing_time"],
                )

                if (
                    (earliest_j <= earliest_i <= latest_j)
                    or (earliest_j <= latest_i <= latest_j)
                ) or (
                    (earliest_i <= earliest_j <= latest_i)
                    or (earliest_i <= latest_j <= latest_i)
                ):
                    uncertain_pairs.append((i, j))

    # Enforce separation for pairs where order is determined (Set V)
    # (6) and (7)
    for i, j in certain_with_no_separation_pairs:
        solver.Add(landing_order[(i, j)] == 1)  # (6)
        solver.Add(
            landing_times[j]
            >= landing_times[i] + separation_times[i][j] * same_runway[(i, j)]
        )  # (7)

    # Enforce order for pairs where order is determined and separation is automatic (Set W)
    # (6)
    for i, j in certain_with_separation_pairs:
        solver.Add(landing_order[(i, j)] == 1)  # (6)

    for i, j in uncertain_pairs:
        # solver.Add(landing_times[j] >= landing_times[i] + separation_times[i][j] * same_runway[(i, j)] + separation_times_between_runways[(i, j)] * (1 - same_runway[(i, j)]) - M * (landing_order[(j, i)])) # (8)
        latest_i = planes_data[i]["latest_landing_time"]
        earliest_j = planes_data[j]["earliest_landing_time"]

        solver.Add(
            landing_times[j]
            >= landing_times[i]
            + separation_times[i][j] * same_runway[(i, j)]
            - (latest_i + separation_times[i][j] - earliest_j) * (landing_order[(j, i)])
        )  # (8)

    # 4. Relating Deviation Variables to Landing Times
    for i in range(num_planes):
        earliest_i = planes_data[i]["earliest_landing_time"]
        latest_i = planes_data[i]["latest_landing_time"]
        target_i = planes_data[i]["target_landing_time"]

        # (14)
        solver.Add(early_deviation[i] >= target_i - landing_times[i])

        # (15)
        solver.Add(early_deviation[i] >= 0)
        solver.Add(early_deviation[i] <= target_i - earliest_i)

        # (16)
        solver.Add(late_deviation[i] >= landing_times[i] - target_i)

        # (17)
        solver.Add(late_deviation[i] >= 0)
        solver.Add(late_deviation[i] <= latest_i - target_i)

        # (18)
        solver.Add(
            landing_times[i] == target_i - early_deviation[i] + late_deviation[i]
        )

    # New constraints for multiple runways
    # (28)
    for i in range(num_planes):
        solver.Add(solver.Sum(landing_runway[(i, r)] for r in range(num_runways)) == 1)

    # (29)
    for i in range(num_planes):
        for j in range(i + 1, num_planes):
            solver.Add(same_runway[(i, j)] == same_runway[(j, i)])

    # (30)
    for i in range(num_planes):
        for j in range(i + 1, num_planes):
            for r in range(num_runways):
                solver.Add(
                    same_runway[(i, j)]
                    >= landing_runway[(i, r)] + landing_runway[(j, r)] - 1
                )

    objective = solver.Objective()

    for i in range(num_planes):
        objective.SetCoefficient(early_deviation[i], planes_data[i]["penalty_early"])
        objective.SetCoefficient(late_deviation[i], planes_data[i]["penalty_late"])

    objective.SetMinimization()

    print("-> Number of decision variables created:", solver.NumVariables())
    print("-> Number of constraints:", solver.NumConstraints())

    return solver, variables


def solve_multiple_runways_mip(num_planes, num_runways, planes_data, separation_times):
    solver, variables = create_mip_model_multiple_runways(
        num_planes, planes_data, separation_times, num_runways
    )
    print("\n" + "=" * 60)
    print("\t\t\tSolving MIP")
    print("=" * 60, "\n")

    # Memory Usage before the Solver
    memory_before = psutil.Process().memory_info().rss  # Memory in bytes

    # Solve the model with performance tracking
    status = solver.Solve()

    # Memory Usage after the Solver
    memory_after = psutil.Process().memory_info().rss

    landing_time = variables["landing_times"]
    earliness = variables["early_deviation"]
    lateness = variables["late_deviation"]

    if status == pywraplp.Solver.OPTIMAL:
        print(f"-> Optimal Cost: {solver.Objective().Value()}")

        print("\n-> Planes that did not land on the target time:")
        for i in range(num_planes):
            e_ = variables["early_deviation"][i].solution_value()
            L_ = variables["late_deviation"][i].solution_value()
            # If earliness or lateness > 0, plane missed its target
            if e_ > 0 or L_ > 0:
                # Calculate penalty
                penalty = (
                    e_ * planes_data[i]["penalty_early"]
                    + L_ * planes_data[i]["penalty_late"]
                )
                landing_t = variables["landing_times"][i]
                target_t = planes_data[i]["target_landing_time"]
                print(
                    f"  -> Plane {i}: {landing_t} | Target Time: {target_t} | Penalty: {penalty}"
                )
    elif status == pywraplp.Solver.FEASIBLE:
        # -------------------------------
        # No optimal solution
        # -------------------------------
        print("-> No optimal solution found.")

        print("-> Best feasible solution found:", round(solver.ObjectiveValue(), 2))

        # You can optionally also list planes that missed their target
        print("\n-> Planes that did not land on the target time:")
        for i in range(num_planes):
            e_ = solver.Value(earliness[i])
            L_ = solver.Value(lateness[i])
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
        print(
            "-> No feasible/optimal solution found. Status:", solver.StatusName(status)
        )

    # Return the solver, variables, number of planes, memory usage info and the number of runways
    return solver, variables, num_planes, memory_before, memory_after


def create_mip_model_single_runway(num_planes, planes_data, separation_times):
    print("=" * 60)
    print("\t\t    Creating MIP Solver")
    print("=" * 60, "\n")

    # Create the LP solver
    solver = pywraplp.Solver.CreateSolver("SAT")
    variables = {}

    # Decision Variables
    # x_i: Landing time for plane i
    # (1)
    landing_times = [
        solver.NumVar(
            planes_data[i]["earliest_landing_time"],
            planes_data[i]["latest_landing_time"],
            f"LandingTime_{i}",
        )
        for i in range(num_planes)
    ]
    variables["landing_times"] = landing_times

    # delta_ij:  Fraction representing if plane i lands before plane j (0 to 1)
    # Note: In a pure LP model, we relax the integrality constraint.
    landing_order = {}
    for i in range(num_planes):
        for j in range(num_planes):
            if i != j:
                landing_order[(i, j)] = solver.NumVar(0, 1, f"LandingOrder_{i}_{j}")
    variables["landing_order"] = landing_order

    # alpha_i: Time by which plane i lands before its target time
    early_deviation = [
        solver.NumVar(
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
        solver.NumVar(
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
    # (2)
    for i in range(num_planes):
        for j in range(i + 1, num_planes):
            solver.Add(landing_order[(i, j)] + landing_order[(j, i)] == 1)

    # Set V
    certain_with_no_separation_pairs = []
    for i in range(num_planes):
        for j in range(num_planes):
            if i != j:
                earliest_i, latest_i = (
                    planes_data[i]["earliest_landing_time"],
                    planes_data[i]["latest_landing_time"],
                )
                earliest_j = planes_data[j]["earliest_landing_time"]
                separation_ij = separation_times[i][j]
                if latest_i < earliest_j and latest_i + separation_ij > earliest_j:
                    certain_with_no_separation_pairs.append((i, j))

    # Set W
    certain_with_separation_pairs = []
    for i in range(num_planes):
        for j in range(num_planes):
            if i != j:
                earliest_i, latest_i = (
                    planes_data[i]["earliest_landing_time"],
                    planes_data[i]["latest_landing_time"],
                )
                earliest_j = planes_data[j]["earliest_landing_time"]
                separation_ij = separation_times[i][j]
                if latest_i < earliest_j and latest_i + separation_ij <= earliest_j:
                    certain_with_separation_pairs.append((i, j))

    uncertain_pairs = []

    # Set U
    for i in range(num_planes):
        for j in range(num_planes):
            if i != j:
                earliest_i, latest_i = (
                    planes_data[i]["earliest_landing_time"],
                    planes_data[i]["latest_landing_time"],
                )
                earliest_j, latest_j = (
                    planes_data[j]["earliest_landing_time"],
                    planes_data[j]["latest_landing_time"],
                )

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
        solver.Add(landing_order[(i, j)] == 1)
        solver.Add(landing_times[j] >= landing_times[i] + separation_times[i][j])

    # Enforce order for pairs where order is determined and separation is automatic (Set W)
    for i, j in certain_with_separation_pairs:
        solver.Add(landing_order[(i, j)] == 1)

    for i, j in uncertain_pairs:
        latest_i = planes_data[i]["latest_landing_time"]
        earliest_j = planes_data[j]["earliest_landing_time"]
        separation_ij = separation_times[i][j]
        delta_ij = landing_order[(i, j)]
        delta_ji = landing_order[(j, i)]

        solver.Add(
            landing_times[j]
            >= landing_times[i]
            + separation_ij * delta_ij
            - (latest_i - earliest_j) * delta_ji
        )  # (11)

    # 4. Relating Deviation Variables to Landing Times
    for i in range(num_planes):
        earliest_i = planes_data[i]["earliest_landing_time"]
        latest_i = planes_data[i]["latest_landing_time"]
        target_i = planes_data[i]["target_landing_time"]

        # (14)
        solver.Add(early_deviation[i] >= target_i - landing_times[i])

        # (15)
        solver.Add(early_deviation[i] >= 0)
        solver.Add(early_deviation[i] <= target_i - earliest_i)

        # (16)
        solver.Add(late_deviation[i] >= landing_times[i] - target_i)

        # (17)
        solver.Add(late_deviation[i] >= 0)
        solver.Add(late_deviation[i] <= latest_i - target_i)

        # (18)
        solver.Add(
            landing_times[i] == target_i - early_deviation[i] + late_deviation[i]
        )

    objective = solver.Objective()

    for i in range(num_planes):
        objective.SetCoefficient(early_deviation[i], planes_data[i]["penalty_early"])
        objective.SetCoefficient(late_deviation[i], planes_data[i]["penalty_late"])

    objective.SetMinimization()

    print("-> Number of decision variables created:", solver.NumVariables())
    print("-> Number of constraints:", solver.NumConstraints())

    return solver, variables


def solve_single_runway_mip(num_planes, planes_data, separation_times):
    solver, variables = create_mip_model_single_runway(
        num_planes, planes_data, separation_times
    )
    print("\n" + "=" * 60)
    print("\t\t\tSolving MIP")
    print("=" * 60, "\n")

    # Memory Usage before the Solver
    memory_before = psutil.Process().memory_info().rss  # Memory in bytes

    # Solve the model with performance tracking
    status = solver.Solve()

    # Memory Usage after the Solver
    memory_after = psutil.Process().memory_info().rss

    landing_time = variables["landing_times"]
    earliness = variables["early_deviation"]
    lateness = variables["late_deviation"]

    if status == pywraplp.Solver.OPTIMAL:
        print(f"-> Optimal Cost: {solver.Objective().Value()}")

        print("\n-> Planes that did not land on the target time:")
        for i in range(num_planes):
            e_ = variables["early_deviation"][i].solution_value()
            L_ = variables["late_deviation"][i].solution_value()
            # If earliness or lateness > 0, plane missed its target
            if e_ > 0 or L_ > 0:
                # Calculate penalty
                penalty = (
                    e_ * planes_data[i]["penalty_early"]
                    + L_ * planes_data[i]["penalty_late"]
                )
                landing_t = variables["landing_times"][i]
                target_t = planes_data[i]["target_landing_time"]
                print(
                    f"  -> Plane {i}: {landing_t} | Target Time: {target_t} | Penalty: {penalty}"
                )
    elif status == pywraplp.Solver.FEASIBLE:
        # -------------------------------
        # No optimal solution
        # -------------------------------
        print("-> No optimal solution found.")

        print("-> Best feasible solution found:", round(solver.ObjectiveValue(), 2))

        # You can optionally also list planes that missed their target
        print("\n-> Planes that did not land on the target time:")
        for i in range(num_planes):
            e_ = solver.Value(earliness[i])
            L_ = solver.Value(lateness[i])
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
        print(
            "-> No feasible/optimal solution found. Status:", solver.StatusName(status)
        )

    return solver, variables, num_planes, memory_before, memory_after

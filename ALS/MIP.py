from ortools.linear_solver import pywraplp

def create_mip_solver(
    num_planes,
    planes_data,
    separation_times,
    num_runways,
):
    print("---------- Creating MIP solver ----------\n")

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

    M = sum(plane["latest_landing_time"] for plane in planes_data)

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

    print("Number of decision variables created:", solver.NumVariables())
    print("Number of constraints:", solver.NumConstraints())

    return solver, variables


def solve_multiple_runways_mip(num_planes, planes_data, separation_times, num_runways):
    solver, variables = create_mip_solver(
        num_planes, planes_data, separation_times, num_runways
    )

    print("\n---------- Solving MIP ----------\n")

    status = solver.Solve()

    if status == pywraplp.Solver.OPTIMAL:
        print(f"Optimal Cost: {solver.Objective().Value()}")

        # print the landing times of the planes that did not land on the target time
        print("\nPlanes that did not land on the target time:")
        for i in range(num_planes):
            if (
                variables["early_deviation"][i].solution_value() > 0
                or variables["late_deviation"][i].solution_value() > 0
            ):
                # calculate associated penalties
                penalty = (
                    variables["early_deviation"][i].solution_value()
                    * planes_data[i]["penalty_early"]
                    + variables["late_deviation"][i].solution_value()
                    * planes_data[i]["penalty_late"]
                )

                print(
                    f"Plane {i}: {variables['landing_times'][i].solution_value()} | Target Time: {planes_data[i]['target_landing_time']} | Penalty: {penalty}"
                )
    else:
        print("No optimal solution found.")

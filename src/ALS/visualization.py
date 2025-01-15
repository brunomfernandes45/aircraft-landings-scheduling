import matplotlib.lines as mlines
import matplotlib.pyplot as plt
from .utils import get_value

def visualize_solution(solver, num_planes, planes_data, variables, approach):
    """
    Visualizes the landing times for each plane with labels, with improved aesthetics.

    Args:
        solver (CP-SAT solver): The CP-SAT solver instance.
        num_planes (int): The number of planes.
        planes_data (list): A list of dictionaries containing plane data.
        variables (dict): A dictionary containing the decision variables.
        approach (str): The approach used to solve the problem.
    """

    LABEL_FONT_SIZE = 12
    TITLE_FONT_SIZE = 16
    LEGEND_FONT_SIZE = 10

    # Define a harmonious color palette
    COLOR_TARGET = "#4c72b0"
    COLOR_OPTIMAL = "#55a868"
    COLOR_EARLY_DEV = "#c44e52"
    COLOR_LATE_DEV = "#dd8452"
    COLOR_OPTIMAL_EQUAL_TARGET = "#8172b3"
    COLOR_EARLIEST_LATEST = "#646464"

    plt.figure(figsize=(17, num_planes * 0.65))
    ax = plt.gca()
    plt.xlabel("Time", fontsize=LABEL_FONT_SIZE)
    plt.ylabel("Planes", fontsize=LABEL_FONT_SIZE)
    plt.title("Plane Landing Schedule", fontsize=TITLE_FONT_SIZE, fontweight="bold")

    # Remove plot frame
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    ax.spines["bottom"].set_visible(False)
    ax.spines["left"].set_visible(False)

    # Sort planes by optimal landing time
    plane_order = sorted(
        range(num_planes), key=lambda i: get_value(variables["landing_time"][i], approach, solver)
    )

    # Set y-axis ticks and labels based on the sorted order
    plt.yticks(
        range(num_planes), [f"Plane {plane_order[i]}" for i in range(num_planes)]
    )

    ax.tick_params(axis="y", length=0)

    plt.xticks([])

    # Adjust plot margins
    plt.subplots_adjust(left=0.1, right=0.8, top=0.9, bottom=0.1)

    max_optimal_time = 0
    min_earliest_time = float("inf")

    # Define legend handles
    legend_handles = [
        mlines.Line2D(
            [], [], color="lightgray", linewidth=5, label="Available Time Range"
        ),
        plt.Line2D(
            [],
            [],
            color=COLOR_EARLIEST_LATEST,
            marker="^",
            linestyle="None",
            markersize=8,
            label=r"Earliest Time ($E_i$)",
        ),
        plt.Line2D(
            [],
            [],
            color=COLOR_EARLIEST_LATEST,
            marker="v",
            linestyle="None",
            markersize=8,
            label=r"Latest Time ($L_i$)",
        ),
        plt.Line2D(
            [],
            [],
            color=COLOR_TARGET,
            marker="o",
            linestyle="None",
            markersize=4,
            label=r"Target Landing Time ($T_i$)",
        ),
        plt.Line2D(
            [],
            [],
            color=COLOR_OPTIMAL,
            marker="x",
            linestyle="None",
            markersize=8,
            label=r"Optimal Landing Time ($x_i$)",
        ),
        plt.Line2D(
            [],
            [],
            color=COLOR_OPTIMAL_EQUAL_TARGET,
            marker="*",
            linestyle="None",
            markersize=8,
            label=r"Optimal = Target ($x_i=T_i$)",
        ),
        mlines.Line2D(
            [],
            [],
            color=COLOR_EARLY_DEV,
            linewidth=8,
            label=r"Early Deviation ($\alpha_i$)",
        ),
        mlines.Line2D(
            [],
            [],
            color=COLOR_LATE_DEV,
            linewidth=8,
            label=r"Late Deviation ($\beta_i$)",
        ),
        mlines.Line2D(
            [], [], color="none", label=r"Penalty values are shown below the deviation."
        ),
    ]

    # for i in range(num_planes):
    for index, i in enumerate(plane_order):
        earliest = planes_data[i]["earliest_landing_time"]
        latest = planes_data[i]["latest_landing_time"]
        target = planes_data[i]["target_landing_time"]
        # optimal = variables["landing_time"][i].solution_value()
        optimal = get_value(variables["landing_time"][i], approach, solver)
        max_optimal_time = max(max_optimal_time, optimal)
        min_earliest_time = min(min_earliest_time, earliest)
        # early_dev = variables["early_deviation"][i].solution_value()
        early_dev = get_value(variables["early_deviation"][i], approach, solver)
        # late_dev = variables["late_deviation"][i].solution_value()
        late_dev = get_value(variables["late_deviation"][i], approach, solver)
        penalty = (
            early_dev * planes_data[i]["penalty_early"]
            + late_dev * planes_data[i]["penalty_late"]
        )

        # y_coord = num_planes - i - 1
        y_coord = num_planes - 1 - index

        # Plotting the time ranges as a background bar
        plt.hlines(
            y=y_coord,
            xmin=earliest,
            xmax=latest,
            color="lightgray",
            linewidth=5,
            label="_nolegend_",
            zorder=1,
        )

        # Plot the deviation lines
        if abs(target - optimal) >= 1e-6:
            if optimal < target:  # Early deviation
                plt.hlines(
                    y=y_coord,
                    xmin=optimal,
                    xmax=target,
                    color=COLOR_EARLY_DEV,
                    linewidth=8,
                    zorder=2,
                    label="_nolegend_",
                )
            elif optimal > target:  # Late deviation
                plt.hlines(
                    y=y_coord,
                    xmin=target,
                    xmax=optimal,
                    color=COLOR_LATE_DEV,
                    linewidth=8,
                    zorder=2,
                    label="_nolegend_",
                )

        if abs(target - optimal) < 1e-6:  # Check if target and optimal are very close
            # Special symbol for landing on target
            plt.scatter(
                optimal,
                y_coord,
                color=COLOR_OPTIMAL_EQUAL_TARGET,
                marker="*",
                s=150,
                zorder=3,
            )
            plt.text(
                optimal,
                y_coord + 0.15,
                r"$x_{" + str(i) + "}$",
                ha="center",
                va="bottom",
                fontsize=LABEL_FONT_SIZE,
                color=COLOR_OPTIMAL_EQUAL_TARGET,
                zorder=3,
            )
        else:
            # Plotting and labeling the times if not on target
            plt.scatter(target, y_coord, color=COLOR_TARGET, marker="o", s=50, zorder=3)
            plt.text(
                target,
                y_coord + 0.15,
                r"$T_{" + str(i) + "}$",
                ha="center",
                va="bottom",
                fontsize=LABEL_FONT_SIZE,
                color=COLOR_TARGET,
                zorder=3,
            )

            plt.scatter(
                optimal, y_coord, color=COLOR_OPTIMAL, marker="x", s=100, zorder=3
            )
            plt.text(
                optimal,
                y_coord + 0.15,
                r"$x_{" + str(i) + "}$",
                ha="center",
                va="bottom",
                fontsize=LABEL_FONT_SIZE,
                color=COLOR_OPTIMAL,
                zorder=3,
            )

            # Add penalty labels
            if penalty > 1e-6:  # Prevent displaying penalty of 0.0
                mid_x = (optimal + target) / 2
                plt.text(
                    mid_x,
                    y_coord - 0.25,
                    f"{penalty:.1f}",
                    ha="center",
                    va="center",
                    fontsize=LABEL_FONT_SIZE - 2,
                    color="black",
                    zorder=3,
                )

        plt.text(
            earliest,
            y_coord - 0.15,
            r"$E_{" + str(i) + "}$",
            ha="center",
            va="top",
            fontsize=LABEL_FONT_SIZE,
            color=COLOR_EARLIEST_LATEST,
            zorder=3,
        )
        plt.scatter(
            earliest, y_coord, color=COLOR_EARLIEST_LATEST, marker="^", s=100, zorder=3
        )
        # Conditionally plot the latest time
        if latest <= max_optimal_time + 0.05 * max_optimal_time:
            plt.scatter(
                latest,
                y_coord,
                color=COLOR_EARLIEST_LATEST,
                marker="v",
                s=100,
                zorder=3,
            )
            plt.text(
                latest,
                y_coord - 0.15,
                r"$L_{" + str(i) + "}$",
                ha="center",
                va="top",
                fontsize=LABEL_FONT_SIZE,
                color=COLOR_EARLIEST_LATEST,
                zorder=3,
            )

    # Adjusting the x-axis limits
    plt.xlim(
        left=min_earliest_time - 0.1 * min_earliest_time,
        right=max_optimal_time + 0.05 * max_optimal_time,
    )

    plt.legend(
        handles=legend_handles,
        loc="center left",
        bbox_to_anchor=(1.05, 0.5),
        fontsize=LEGEND_FONT_SIZE,
        title="Legend",
    )

    plt.tight_layout(pad=2)
    plt.show()

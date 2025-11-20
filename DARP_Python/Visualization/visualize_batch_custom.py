import os

import numpy as np
from matplotlib import pyplot as plt
from matplotlib.ticker import MultipleLocator

from Simulation.utilities import read_csv_with_encoding, darken_color
from Visualization.plot_config import PlotConfig
import seaborn as sns


def create_truncate_objective_scatter(data_path: str, config: PlotConfig):
    # Read data
    df = read_csv_with_encoding(data_path)

    # Group by 'MaxLabel' and 'sortPath' and calculate the mean for '(Lim)wait/req' and '#IMP Iter'
    df = df[df['Ride_W2'] == 0.5]
    grouped = df.groupby(['MaxLabel', 'sortPath']).agg({
        'Objective': 'mean',
        '#IMP Iter': 'mean'
    }).reset_index()

    max_label_40 = grouped[grouped['MaxLabel'] == 40]['Objective'].mean()
    # Remove MaxLabel = 40 from the main dataset for plotting
    grouped = grouped[grouped['MaxLabel'] != 40]

    # Pivot the table to get 'MaxLabel' on the index and the methods from 'sortPath' on the columns
    pivot_df = grouped.pivot(index='MaxLabel', columns='sortPath')

    # Create color palettes
    palette = sns.color_palette("gist_earth", n_colors=4)[0:3][::-1]
    line_colors = palette  # Colors for the lines

    sort_paths = pivot_df['Objective'].columns.tolist()

    # Create combined plot
    xtick_labels = [5, 10, 15, 20, 25, 30, 35]  # Replace 40 with 'N/A'

    # Save the line plot separately
    fig_line, ax_line = plt.subplots(figsize=config.fig_size)
    ax_line.spines['top'].set_visible(False)
    ax_line.spines['right'].set_visible(False)

    for i, sort_path in enumerate(sort_paths):
        if sort_path == 'PATH_SCORE  ':
            marker = 's'
            label = 'Normalized Reduced Cost'
        elif sort_path == 'LAMBDA_SCORE':
            marker = '^'
            label = 'Lambda Score'
        else:
            marker = 'o'
            label = 'Reduced Cost'

        ax_line.plot(pivot_df.index, pivot_df[('Objective', sort_path)],
                     label=label,
                     color=line_colors[i % len(line_colors)], marker=marker, zorder=3)

  #  ax_line.axhline(y=max_label_40, color='red', label=f'No truncation ({max_label_40:.2f}s)')
    ax_line.plot([5, 35], [max_label_40, max_label_40], color='darkred', linestyle='--',
             label=f'No truncation ({max_label_40:.0f}s)')
    ax_line.set_xlabel('$M^{Label}$', fontweight='bold', fontsize=config.axis_label_fsize)
    ax_line.set_ylabel('Objective (s)', fontweight='bold', fontsize=config.axis_label_fsize)
    ax_line.tick_params(axis='both', which='major', labelsize=config.tick_label_fsize)

    ax_line.set_xticks(pivot_df.index)  # For the line plot
    ax_line.set_xticklabels(xtick_labels)  # Replace the label for 40 with 'N/A'

 #   ax_line.set_ylim(162, 168)
 #   ax_line.grid(True, linestyle='-', linewidth=0.5, color='darkgray')
    ax_line.legend(loc='upper right', fontsize=config.legend_fsize, title='Sort Options',
                   edgecolor=config.legend_edgecolor, title_fontproperties={'weight': 'bold', 'size': config.legend_title_fsize})

    # Add a zoomed inset graph
    axins = ax_line.inset_axes([0.28, 0.55, 0.33, 0.45])  # [x_position, y_position, width, height]

    for i, sort_path in enumerate(sort_paths):
        if sort_path == 'PATH_SCORE  ':
            marker = 's'
            label = 'Normalized Reduced Cost'
        elif sort_path == 'LAMBDA_SCORE':
            marker = '^'
            label = 'Lambda Score'
        else:
            marker = 'o'
            label = 'Reduced Cost'

        axins.plot(pivot_df.index, pivot_df[('Objective', sort_path)],
                     label=label,
                     color=line_colors[i % len(line_colors)], marker=marker, zorder=3)

        axins.plot([5, 35], [max_label_40, max_label_40], color='darkred', linestyle='--',
                     label=f'No truncation ({max_label_40:.2f}s)')


    # Set limits for zoomed-in view (adjust these based on the area you want to zoom)
    x1, x2, y1, y2 = 4, 16.5, 184.7,185.4  # These values can be adjusted as needed
    axins.set_xlim(x1, x2)
   # axins.set_ylim(158380, 158750)
    axins.set_ylim(276450, 277100)
    axins.set_facecolor('#f7f7f7')

 #   axins.set_yticks([64000, 64100])  # Only show tick at 185
 #   axins.set_yticklabels(['158500', '158600'])
    axins.tick_params(axis='y', which='major', labelsize=config.tick_label_fsize)

    # Indicate zoomed-in area
    ax_line.indicate_inset_zoom(axins)

    fig_line.tight_layout()
    line_figure_path = os.path.join(os.path.dirname(data_path), 'truncate_plot_line_05.png')
    fig_line.savefig(line_figure_path, dpi=300, bbox_inches="tight")
    plt.close()


def create_dynamic_iteration_bubble_plot_double(data_path: str, config: PlotConfig):
    # Read data
    df = read_csv_with_encoding(data_path)

    isDropPickPossible = True
    df = df[df['isDropPickPossible'] == isDropPickPossible]

    df['Dynamic_Pricing'] = df['Dynamic_Pricing'].astype(str)

    # Color palette
    palette = sns.color_palette("gist_earth", n_colors=2)
    darkened_palette = [darken_color(color, factor=0.4) for color in palette]
    light_palette = [(r, g, b, 0.5) for r, g, b in palette]

    # Initialize the matplotlib figure with two subplots
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=config.fig_size)

    # Extract and sort unique labels for Instance
    df['Instance'] = df['Instance'].astype(str)
    instances = np.sort(df['Instance'].unique())
    drop_options = ['False', 'True']

    # Function to plot on a given axis
    def plot_on_axis(ax, df_filtered, ride_w2_value):
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)

        # Determine positions for each instance
        positions = np.array(range(len(instances)))
        width = 0.35  # Width offset for the two categories

        # Get all unique iteration values to determine y-axis range
        all_iterations = df_filtered['#SP Iter '].values
        unique_iterations = np.sort(df_filtered['#SP Iter '].unique())

        # Calculate frequency of each (instance, drop_option, iteration_value) combination
        max_count = 0

        # First pass: collect all bubbles and find max_count
        bubbles_to_plot = []

        for idx, ml in enumerate(instances):
            for i, sp in enumerate(drop_options):
                # Get data for this combination
                data = df_filtered[
                    (df_filtered['Instance'] == ml) &
                    (df_filtered['Dynamic_Pricing'].str.strip() == sp)
                    ]['#SP Iter ']

                if len(data) > 0:
                    # Count frequency of each iteration value
                    value_counts = data.value_counts()
                    max_count = max(max_count, value_counts.max())

                    # X position: offset based on drop_option
                    x_pos = positions[idx]

                    # Store bubble info for later plotting
                    for iter_value, count in value_counts.items():
                        bubbles_to_plot.append({
                            'x': x_pos,
                            'y': iter_value,
                            'count': count,
                            'color': light_palette[i],
                            'edge_color': darkened_palette[i]
                        })

        # Second pass: plot bubbles sorted by size (largest first, so smallest appear on top)
        bubbles_to_plot.sort(key=lambda b: b['count'], reverse=True)

        for bubble in bubbles_to_plot:
            # Bubble size proportional to frequency
            bubble_size = (bubble['count'] / max_count) * 800  # Scale factor for visibility

            ax.scatter(bubble['x'], bubble['y'], s=bubble_size,
                       c=[bubble['color']], edgecolors=bubble['edge_color'],
                       linewidths=1.2, alpha=0.5, zorder=3)

        # Set the labels
        ax.set_xlabel(f"$W_{{delay}}$ = {ride_w2_value}",
                      fontweight='bold', fontsize=config.tick_label_fsize)

        # Set x-axis limits with padding to avoid overlap with y-axis
        ax.set_xlim(positions[0] - 0.4, positions[-1] + 0.4)

        # Set x-ticks and labels
        ax.set_xticks(positions)
        ax.set_xticklabels(instances, rotation=15, fontsize=config.tick_label_fsize)

        # Set y-axis
        ax.set_ylim(0.3,5.5)
        ax.yaxis.set_major_locator(MultipleLocator(1))

        # Grid
        ax.grid(color="lightgray", linestyle="--", axis='x', alpha=0.5, zorder=0)
        ax.tick_params(axis='both', which='major', labelsize=config.tick_label_fsize, length=3.8)

    # Plot for Ride_W2 = 0 (left subplot)
    df_ride_0 = df[df['Ride_W2'] == 0]
    plot_on_axis(ax1, df_ride_0, 0)
    ax1.set_ylabel('# SP Iterations', fontsize=config.axis_label_fsize, fontweight='bold')

    # Plot for Ride_W2 = 0.5 (right subplot)
    df_ride_05 = df[df['Ride_W2'] == 0.5]
    plot_on_axis(ax2, df_ride_05, 0.5)

    # Create custom legend handles for drop-pick options
    drop_pick_legend_handles = [
        plt.Line2D([0], [0], marker='o', color='w',
                   markerfacecolor=palette[0], markeredgecolor=darkened_palette[0],
                   markersize=10, markeredgewidth=1.5, alpha=0.7),
        plt.Line2D([0], [0], marker='o', color='w',
                   markerfacecolor=palette[1], markeredgecolor=darkened_palette[1],
                   markersize=10, markeredgewidth=1.5, alpha=0.7)
    ]
    drop_pick_labels = ['2 Pickups', 'Dynamic Pickups Limit']

    # Shared legend
    legend1 = fig.legend(
        handles=drop_pick_legend_handles,
        labels=drop_pick_labels,
        loc='upper center',
        bbox_to_anchor=(0.55, 0.99),
        ncol=2,
        fontsize=config.legend_fsize,
        labelspacing=0.45,
        edgecolor=config.legend_edgecolor,
        title='Pickup Limits',
        title_fontproperties={'weight': 'bold', 'size': config.legend_title_fsize},
        framealpha=1.0,
        facecolor='white'
    )

    # Adjust layout and save the figure
    fig.tight_layout(rect=[0, 0, 1, 0.85])
    figure_path = os.path.join(os.path.dirname(data_path),
                               f'dynamic-iterations-bubble_{isDropPickPossible}.png')
    fig.savefig(figure_path, dpi=300, bbox_inches="tight")
    plt.close(fig)

    return figure_path

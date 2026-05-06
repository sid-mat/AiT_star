"""
ENPM661 Project 5 - Benchmark & Visualization
Compares RRT*, Informed RRT*, and AIT* on two environments.

Run:
    python main.py

Outputs:
    results/comparison_cluttered.png
    results/comparison_corridor.png
    results/convergence.png
"""

import os
import sys
import time
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyArrowPatch
from matplotlib.lines import Line2D

# Make imports work from project root
sys.path.insert(0, os.path.dirname(__file__))

from utils.environment import Environment
from planners.rrt_star import RRTStar
from planners.informed_rrt_star import InformedRRTStar
from planners.ait_star import AITStar

os.makedirs('results', exist_ok=True)

# ── Color palette ─────────────────────────────────────────────────────────────
C = {
    'bg':       '#0f1117',
    'grid':     '#1e2130',
    'obs':      '#c0392b',
    'obs_edge': '#e74c3c',
    'start':    '#2ecc71',
    'goal':     '#e74c3c',
    'rrt':      '#3498db',
    'inf':      '#f39c12',
    'ait':      '#9b59b6',
    'tree':     '#ffffff',
    'path':     '#ffffff',
    'text':     '#ecf0f1',
}

# ── Drawing helpers ───────────────────────────────────────────────────────────

def draw_env(ax, env, title=''):
    ax.set_facecolor(C['bg'])
    ax.set_xlim(env.bounds[0], env.bounds[1])
    ax.set_ylim(env.bounds[2], env.bounds[3])
    ax.set_aspect('equal')
    ax.grid(True, color=C['grid'], linewidth=0.4, alpha=0.5)
    ax.tick_params(colors=C['text'], labelsize=7)
    for spine in ax.spines.values():
        spine.set_edgecolor('#2c3e50')
    if title:
        ax.set_title(title, color=C['text'], fontsize=10, fontweight='bold', pad=6)
    for obs in env.obstacles:
        circ = plt.Circle((obs.cx, obs.cy), obs.radius,
                          color=C['obs'], alpha=0.75, zorder=3)
        edge = plt.Circle((obs.cx, obs.cy), obs.radius,
                          fill=False, edgecolor=C['obs_edge'], linewidth=0.8, zorder=4)
        ax.add_patch(circ)
        ax.add_patch(edge)


def draw_tree_rrt(ax, nodes, color, alpha=0.3, lw=0.5):
    for node in nodes:
        if node.parent:
            ax.plot([node.x, node.parent.x], [node.y, node.parent.y],
                    color=color, alpha=alpha, linewidth=lw, zorder=2)


def draw_tree_ait(ax, tree_nodes, color, alpha=0.25, lw=0.5):
    for node in tree_nodes:
        if node.parent:
            ax.plot([node.x, node.parent.x], [node.y, node.parent.y],
                    color=color, alpha=alpha, linewidth=lw, zorder=2)


def draw_samples(ax, samples, color, alpha=0.15, s=4):
    xs = [v.x for v in samples]
    ys = [v.y for v in samples]
    ax.scatter(xs, ys, s=s, color=color, alpha=alpha, zorder=1)


def draw_path(ax, path, color, lw=2.5, zorder=8):
    if path is None or len(path) < 2:
        return
    xs, ys = zip(*path)
    ax.plot(xs, ys, color=color, linewidth=lw, zorder=zorder, solid_capstyle='round')


def draw_endpoints(ax, start, goal):
    ax.scatter(*start, s=120, color=C['start'], zorder=10,
               marker='*', edgecolors='white', linewidths=0.5)
    ax.scatter(*goal,  s=120, color=C['goal'],  zorder=10,
               marker='X', edgecolors='white', linewidths=0.5)
    ax.annotate('S', start, textcoords='offset points', xytext=(5, 5),
                color=C['start'], fontsize=7, fontweight='bold')
    ax.annotate('G', goal,  textcoords='offset points', xytext=(5, 5),
                color=C['goal'],  fontsize=7, fontweight='bold')


# ── Run a planner and collect convergence data ────────────────────────────────

def run_rrt_variant(PlannerClass, env, start, goal, max_iter, **kwargs):
    planner = PlannerClass(env, start, goal, **kwargs)
    costs, times = [], []
    t0 = time.time()
    for _, cost, path in planner.plan(max_iter=max_iter):
        costs.append(cost if cost < float('inf') else float('nan'))
        times.append(time.time() - t0)
    return planner, costs, times


def run_ait(env, start, goal, max_iter, batch_size=150):
    planner = AITStar(env, start, goal, batch_size=batch_size)
    costs, times, snapshots = [], [], []
    t0 = time.time()
    for it, cost, path, tree, samples in planner.plan(max_iter=max_iter):
        costs.append(cost if cost < float('inf') else float('nan'))
        times.append(time.time() - t0)
        snapshots.append((it, tree[:], samples[:], path))
    return planner, costs, times, snapshots


# ── Plot 1: Side-by-side final plans ─────────────────────────────────────────

def plot_comparison(env, start, goal, name,
                    rrt_planner, inf_planner, ait_planner, ait_snapshots):
    fig, axes = plt.subplots(1, 3, figsize=(15, 5.5))
    fig.patch.set_facecolor('#0a0c12')
    fig.suptitle(f'Planning Comparison — {name}',
                 color=C['text'], fontsize=14, fontweight='bold', y=1.01)

    titles = ['RRT*', 'Informed RRT*', 'AIT*']
    colors = [C['rrt'], C['inf'], C['ait']]
    planners = [rrt_planner, inf_planner, ait_planner]
    paths = [rrt_planner.best_path, inf_planner.best_path, ait_planner.best_path]

    for ax, title, color, planner, path in zip(axes, titles, colors, planners, paths):
        draw_env(ax, env, title)

        if title in ('RRT*', 'Informed RRT*'):
            draw_tree_rrt(ax, planner.nodes, color=color)
        else:
            _, tree, samples, _ = ait_snapshots[-1]
            draw_tree_ait(ax, tree, color=color)
            draw_samples(ax, samples, color=color)

        draw_path(ax, path, color='#ffffff', lw=2.8)
        draw_endpoints(ax, start, goal)

        cost_str = f'{planner.best_cost:.3f}' if planner.best_cost < float('inf') else 'No path'
        if title == 'AIT*':
            cost_str = f'{ait_planner.best_cost:.3f}' if ait_planner.best_cost < float('inf') else 'No path'

        ax.set_xlabel(f'Path cost: {cost_str}', color=color,
                      fontsize=9, fontweight='bold', labelpad=4)

    plt.tight_layout()
    out = f'results/comparison_{name.lower().replace(" ", "_")}.png'
    plt.savefig(out, dpi=180, bbox_inches='tight', facecolor=fig.get_facecolor())
    plt.close()
    print(f'  Saved: {out}')
    return out


# ── Plot 2: Convergence curves ────────────────────────────────────────────────

def plot_convergence(rrt_costs, inf_costs, ait_costs,
                     rrt_times, inf_times, ait_times, name):
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5))
    fig.patch.set_facecolor('#0a0c12')
    fig.suptitle(f'Convergence Analysis — {name}',
                 color=C['text'], fontsize=13, fontweight='bold')

    def finite(lst):
        return [x if not (x != x) else None for x in lst]  # NaN -> None for gap

    def plot_curve(ax, xdata, ydata, color, label, lw=2):
        # Plot only finite segments
        xs, ys = [], []
        for x, y in zip(xdata, ydata):
            if y is not None:
                xs.append(x)
                ys.append(y)
        if xs:
            ax.plot(xs, ys, color=color, linewidth=lw, label=label,
                    solid_capstyle='round')

    for ax in (ax1, ax2):
        ax.set_facecolor(C['bg'])
        ax.grid(True, color=C['grid'], linewidth=0.5, alpha=0.6)
        ax.tick_params(colors=C['text'], labelsize=8)
        for spine in ax.spines.values():
            spine.set_edgecolor('#2c3e50')

    # Iteration-based
    iters_rrt = list(range(len(rrt_costs)))
    iters_inf = list(range(len(inf_costs)))
    iters_ait = list(range(0, len(ait_costs) * 150, 150))  # batch_size=150

    plot_curve(ax1, iters_rrt, finite(rrt_costs), C['rrt'], 'RRT*')
    plot_curve(ax1, iters_inf, finite(inf_costs), C['inf'], 'Informed RRT*')
    plot_curve(ax1, iters_ait, finite(ait_costs), C['ait'], 'AIT*', lw=2.5)

    ax1.set_xlabel('Iterations / Samples', color=C['text'], fontsize=9)
    ax1.set_ylabel('Solution cost', color=C['text'], fontsize=9)
    ax1.set_title('Cost vs Samples', color=C['text'], fontsize=10, fontweight='bold')
    ax1.legend(facecolor='#1e2130', edgecolor='#2c3e50',
               labelcolor=C['text'], fontsize=9)

    # Time-based
    plot_curve(ax2, rrt_times, finite(rrt_costs), C['rrt'], 'RRT*')
    plot_curve(ax2, inf_times, finite(inf_costs), C['inf'], 'Informed RRT*')
    plot_curve(ax2, ait_times, finite(ait_costs), C['ait'], 'AIT*', lw=2.5)

    ax2.set_xlabel('Wall-clock time (s)', color=C['text'], fontsize=9)
    ax2.set_ylabel('Solution cost', color=C['text'], fontsize=9)
    ax2.set_title('Cost vs Time', color=C['text'], fontsize=10, fontweight='bold')
    ax2.legend(facecolor='#1e2130', edgecolor='#2c3e50',
               labelcolor=C['text'], fontsize=9)

    plt.tight_layout()
    out = f'results/convergence_{name.lower().replace(" ", "_")}.png'
    plt.savefig(out, dpi=180, bbox_inches='tight', facecolor=fig.get_facecolor())
    plt.close()
    print(f'  Saved: {out}')
    return out


# ── Plot 3: AIT* batch progression (for presentation) ────────────────────────

def plot_ait_progression(env, start, goal, snapshots, name):
    snap_indices = [0, len(snapshots)//4, len(snapshots)//2, -1]
    labels = ['Batch 1', f'Batch {len(snapshots)//4}',
              f'Batch {len(snapshots)//2}', f'Final (Batch {len(snapshots)})']

    fig, axes = plt.subplots(1, 4, figsize=(18, 4.5))
    fig.patch.set_facecolor('#0a0c12')
    fig.suptitle(f'AIT* Batch Progression — {name}',
                 color=C['text'], fontsize=13, fontweight='bold', y=1.01)

    for ax, si, label in zip(axes, snap_indices, labels):
        _, tree, samples, path = snapshots[si]
        draw_env(ax, env, label)
        draw_samples(ax, samples, C['ait'], alpha=0.2, s=5)
        draw_tree_ait(ax, tree, C['ait'], alpha=0.4)
        draw_path(ax, path, '#ffffff', lw=2.5)
        draw_endpoints(ax, start, goal)
        ax.set_xlabel(f'{len(samples)} samples | {len(tree)} tree nodes',
                      color=C['text'], fontsize=7, labelpad=3)

    plt.tight_layout()
    out = f'results/ait_progression_{name.lower().replace(" ", "_")}.png'
    plt.savefig(out, dpi=180, bbox_inches='tight', facecolor=fig.get_facecolor())
    plt.close()
    print(f'  Saved: {out}')
    return out


# ── Main ──────────────────────────────────────────────────────────────────────

def run_scenario(name, env, start, goal,
                 rrt_iters=2000, ait_batches=40, ait_batch_size=150):
    print(f'\n{"="*55}')
    print(f'  Scenario: {name}')
    print(f'{"="*55}')

    np.random.seed(42)
    print('  Running RRT*...')
    rrt, rrt_costs, rrt_times = run_rrt_variant(
        RRTStar, env, start, goal, rrt_iters)
    print(f'    Final cost: {rrt.best_cost:.3f}  |  nodes: {len(rrt.nodes)}')

    np.random.seed(42)
    print('  Running Informed RRT*...')
    inf, inf_costs, inf_times = run_rrt_variant(
        InformedRRTStar, env, start, goal, rrt_iters)
    print(f'    Final cost: {inf.best_cost:.3f}  |  nodes: {len(inf.nodes)}')

    np.random.seed(42)
    print('  Running AIT*...')
    ait, ait_costs, ait_times, ait_snaps = run_ait(
        env, start, goal, ait_batches, ait_batch_size)
    print(f'    Final cost: {ait.best_cost:.3f}  |  tree: {len(ait.T)}  |  samples: {len(ait.V)}')

    print('  Generating figures...')
    plot_comparison(env, start, goal, name, rrt, inf, ait, ait_snaps)
    plot_convergence(rrt_costs, inf_costs, ait_costs,
                     rrt_times, inf_times, ait_times, name)
    plot_ait_progression(env, start, goal, ait_snaps, name)

    # Summary stats
    def first_solution_iter(costs):
        for i, c in enumerate(costs):
            if c == c and c < float('inf'):
                return i
        return None

    print('\n  --- Summary ---')
    print(f'  {"Planner":<20} {"Final Cost":>12}  {"1st Solution Iter":>18}')
    print(f'  {"-"*52}')
    for label, costs in [('RRT*', rrt_costs), ('Informed RRT*', inf_costs), ('AIT*', ait_costs)]:
        fc = [c for c in costs if c == c and c < float('inf')]
        final = f'{fc[-1]:.3f}' if fc else 'None'
        first = first_solution_iter(costs)
        print(f'  {label:<20} {final:>12}  {str(first):>18}')


if __name__ == '__main__':
    scenarios = [
        {
            'name': 'Cluttered Room',
            'env':   Environment.cluttered_room(),
            'start': (0.5, 0.5),
            'goal':  (9.5, 9.5),
        },
        {
            'name': 'Narrow Corridor',
            'env':   Environment.narrow_corridor(),
            'start': (1.0, 5.0),
            'goal':  (9.0, 5.0),
        },
    ]

    for s in scenarios:
        run_scenario(**s, rrt_iters=1500, ait_batches=15, ait_batch_size=50)

    print('\nAll done. Results in ./results/')

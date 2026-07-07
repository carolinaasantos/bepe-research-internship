# ------------------------------------------------------------------
# ICBS PATH ANIMATION AND VISUALIZATION TOOL

# Adapted from MAPF-ICBS.
#
# Original source:
# https://github.com/gloriyo/MAPF-ICBS
#
# This script visualizes ICBS-generated MAPF solutions together
# with VRPPD pickup and delivery goals.
#
# Modifications in this file:
# - Added support for multiple goals in the visualization layer.
# - Introduced dynamic objects, where gray boxes (representing obstacles)
#   appear and disappear depending on collection or delivery events.
# - Agents are allowed to traverse obstacle cells if the cell corresponds
#   to their pickup point and if it matches exactly their next goal.
# ------------------------------------------------------------------

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib import animation
from matplotlib.patches import Circle, Rectangle

PROJECT_ROOT = Path(__file__).resolve().parent
WORKSPACE_ROOT = PROJECT_ROOT.parent
SOLVER_ROOT = WORKSPACE_ROOT / "solver"

Colors = ["yellow", "blue", "green", "purple", "gold", "cyan", "magenta", "brown"]

# --------------------------------------------------------------
# ICBS animation engine
#
# Responsible for rendering and updating the complete MAPF
# simulation.
#
# Visualized elements:
# - Static obstacles
# - Dynamic obstacle cells
# - Agent trajectories
# - Pickup and delivery goals
# - Collision events
#
# Agent positions are interpolated between timesteps to
# provide smooth movement during playback.
# --------------------------------------------------------------

class Animation:
    def __init__(self, my_map, starts, paths, goal_sequences=None):

        # ----------------------------------------------------------
        # Coordinate conversion
        #
        # The original ICBS representation uses (row, col) indexing,
        # while matplotlib rendering operates in Cartesian space.
        # All coordinates are transformed into plotting coordinates.
        # ----------------------------------------------------------

        self.map_rc = [row[:] for row in my_map]
        self.my_map = np.flip(np.transpose(my_map), 1)
        self.rows = len(my_map)
        self.cols = len(my_map[0])

        self.starts = [self.to_plot(loc) for loc in starts]
        self.paths = []
        for path in paths:
            self.paths.append([self.to_plot(loc) for loc in path])

        # goal_sequences: list per agent of original-map coordinates
        self.goal_sequences = []
        if goal_sequences is not None:
            for goals in goal_sequences:
                self.goal_sequences.append([self.to_plot(g) for g in goals])
        self.goal_sequences_rc = goal_sequences if goal_sequences is not None else None

        # ----------------------------------------------------------
        # Dynamic obstacle generation
        #
        # Pickup (odd) goals open obstacle cells.
        # Delivery (even) goals close obstacle cells.
        #
        # Events are reconstructed automatically from the executed
        # agent paths and goal sequences.
        # ----------------------------------------------------------
        self.dynamic_events = []
        if self.goal_sequences_rc is not None:
            self.dynamic_events = build_dynamic_obstacle_events(paths, self.goal_sequences_rc)
        self.pickup_goal_patches = {}
        self.pickup_event_time = {}
        for t, loc, action in self.dynamic_events:
            if action == "open" and loc not in self.pickup_event_time:
                self.pickup_event_time[loc] = t

        # ----------------------------------------------------------
        # Plot configuration
        #
        # Creates the visualization canvas and adjusts plotting
        # limits according to map dimensions.
        # ----------------------------------------------------------

        aspect = len(self.my_map) / len(self.my_map[0])
        self.fig = plt.figure(frameon=False, figsize=(4 * aspect, 4))
        self.ax = self.fig.add_subplot(111, aspect="equal")
        self.fig.subplots_adjust(left=0, right=1, bottom=0, top=1, wspace=None, hspace=None)

        self.patches = []
        self.artists = []
        self.agents = {}
        self.agent_names = {}

        x_min = -0.5
        y_min = -0.5
        x_max = len(self.my_map) - 0.5
        y_max = len(self.my_map[0]) - 0.5
        plt.xlim(x_min, x_max)
        plt.ylim(y_min, y_max)

        # ----------------------------------------------------------
        # Static obstacle rendering
        #
        # Every blocked cell is represented as a gray square.
        # These cells form the initial obstacle configuration
        # before any pickup or delivery event occurs.
        # ----------------------------------------------------------

        self.patches.append(
            Rectangle((x_min, y_min), x_max - x_min, y_max - y_min, facecolor="none", edgecolor="gray")
        )
        self.obstacle_patches = {}
        self.initial_blocked = set()
        for r in range(self.rows):
            for c in range(self.cols):
                plot = self.to_plot((r, c))
                patch = Rectangle((plot[0] - 0.5, plot[1] - 0.5), 1, 1, facecolor="gray", edgecolor="gray")
                blocked = bool(self.map_rc[r][c])
                patch.set_visible(blocked)
                self.obstacle_patches[(r, c)] = patch
                if blocked:
                    self.initial_blocked.add((r, c))
                self.patches.append(patch)

        # ----------------------------------------------------------
        # Goal visualization
        #
        # Pickup and delivery goals are rendered as colored markers
        # associated with each agent.
        #
        # Pickup goals are tracked separately so their visual state
        # can be updated after collection.
        # ----------------------------------------------------------

        if self.goal_sequences:
            for a, goals in enumerate(self.goal_sequences):
                color = Colors[a % len(Colors)]
                for idx, g in enumerate(goals):
                    patch = Rectangle((g[0] - 0.25, g[1] - 0.25), 0.5, 0.5,
                                      facecolor=color, edgecolor="black", alpha=0.7)
                    self.patches.append(patch)
                    # Goal index 0,2,4... = pickup
                    if idx % 2 == 0:
                        rc_loc = self.goal_sequences_rc[a][idx]
                        self.pickup_goal_patches.setdefault(rc_loc, []).append((patch, color))

        # ----------------------------------------------------------
        # Agent initialization
        #
        # Creates graphical representations and labels for every
        # agent participating in the simulation.
        # ----------------------------------------------------------

        self.T = 0
        for i in range(len(self.paths)):
            name = str(i)
            self.agents[i] = Circle((self.starts[i][0], self.starts[i][1]), 0.3,
                                    facecolor=Colors[i % len(Colors)], edgecolor="black")
            self.agents[i].original_face_color = Colors[i % len(Colors)]
            self.patches.append(self.agents[i])
            self.T = max(self.T, len(self.paths[i]) - 1)

            self.agent_names[i] = self.ax.text(self.starts[i][0], self.starts[i][1] + 0.25, name)
            self.agent_names[i].set_horizontalalignment("center")
            self.agent_names[i].set_verticalalignment("center")
            self.artists.append(self.agent_names[i])

        # ----------------------------------------------------------
        # Animation controller
        #
        # Executes frame updates and drives the simulation playback.
        # ----------------------------------------------------------

        self.animation = animation.FuncAnimation(
            self.fig,
            self.animate_func,
            init_func=self.init_func,
            frames=int(self.T + 1) * 10,
            interval=110,
            blit=True,
        )

    # --------------------------------------------------------------
    # Coordinate transformation
    #
    # Converts map coordinates (row, col) into plotting-space
    # coordinates used by matplotlib.
    # --------------------------------------------------------------

    def to_plot(self, loc):
        # (row, col) -> plotting coords
        return (loc[1], self.cols - 1 - loc[0])

    # --------------------------------------------------------------
    # Animation exporter
    #
    # Saves the generated animation to a video file.
    # --------------------------------------------------------------

    def save(self, file_name, speed):
        self.animation.save(
            file_name,
            fps=10 * speed,
            dpi=200,
            savefig_kwargs={"pad_inches": 0, "bbox_inches": "tight"},
        )

    # --------------------------------------------------------------
    # Interactive viewer
    #
    # Displays the animation in a matplotlib window.
    # --------------------------------------------------------------

    @staticmethod
    def show():
        plt.show()

    # --------------------------------------------------------------
    # Initial rendering callback
    #
    # Registers all graphical elements with the plotting canvas.
    # --------------------------------------------------------------

    def init_func(self):
        for p in self.patches:
            self.ax.add_patch(p)
        for a in self.artists:
            self.ax.add_artist(a)
        return self.patches + self.artists

    # --------------------------------------------------------------
    # Frame update callback
    #
    # Updates:
    # - Dynamic obstacle states
    # - Goal visualization
    # - Agent positions
    # - Collision indicators
    #
    # Executed once per rendered frame.
    # --------------------------------------------------------------

    def animate_func(self, t):
        sim_t = int(t / 10)
        blocked_now = self.get_blocked_cells_at_time(sim_t)
        for loc, patch in self.obstacle_patches.items():
            patch.set_visible(loc in blocked_now)
        for loc, patch_list in self.pickup_goal_patches.items():
            picked = sim_t >= self.pickup_event_time.get(loc, float("inf"))
            for patch, color in patch_list:
                if picked:
                    patch.set_facecolor("none")
                    patch.set_edgecolor(color)
                    patch.set_linestyle(":")
                    patch.set_linewidth(2.0)
                    patch.set_alpha(1.0)
                else:
                    patch.set_facecolor(color)
                    patch.set_edgecolor("black")
                    patch.set_linestyle("-")
                    patch.set_linewidth(1.0)
                    patch.set_alpha(0.7)

        for k in range(len(self.paths)):
            pos = self.get_state(t / 10, self.paths[k])
            self.agents[k].center = (pos[0], pos[1])
            self.agent_names[k].set_position((pos[0], pos[1] + 0.5))

        for _, agent in self.agents.items():
            agent.set_facecolor(agent.original_face_color)

        agents_array = [agent for _, agent in self.agents.items()]
        for i in range(len(agents_array)):
            for j in range(i + 1, len(agents_array)):
                d1 = agents_array[i]
                d2 = agents_array[j]
                pos1 = np.array(d1.center)
                pos2 = np.array(d2.center)

                # ----------------------------------------------------------
                # Collision detection
                #
                # Agents occupying overlapping positions are highlighted
                # in red to indicate a collision event.
                # ----------------------------------------------------------

                if np.linalg.norm(pos1 - pos2) < 0.7:
                    d1.set_facecolor("red")
                    d2.set_facecolor("red")
                    print(f"COLLISION! (agent-agent) ({i}, {j}) at time {t / 10}")

        return self.patches + self.artists

    # --------------------------------------------------------------
    # Dynamic obstacle reconstruction
    #
    # Computes the set of blocked cells for a specific timestep
    # by replaying pickup and delivery events.
    # --------------------------------------------------------------

    def get_blocked_cells_at_time(self, timestep):
        blocked = set(self.initial_blocked)
        for t, loc, action in self.dynamic_events:
            if t > timestep:
                break
            if action == "open":
                blocked.discard(loc)
            else:
                blocked.add(loc)
        return blocked

    # --------------------------------------------------------------
    # Agent state interpolation
    #
    # Computes the agent position at an arbitrary simulation
    # timestamp using linear interpolation.
    # --------------------------------------------------------------

    @staticmethod
    def get_state(t, path):
        if int(t) <= 0:
            return np.array(path[0])
        if int(t) >= len(path):
            return np.array(path[-1])
        pos_last = np.array(path[int(t) - 1])
        pos_next = np.array(path[int(t)])
        return (pos_next - pos_last) * (t - int(t)) + pos_last

# --------------------------------------------------------------
# ICBS instance loader
#
# Reads:
# - Grid map
# - Agent start positions
#
# Expected format:
# 1. Map dimensions
# 2. Grid layout
# 3. Number of agents
# 4. Agent start coordinates
# --------------------------------------------------------------

def import_mapf_instance(filename):
    p = Path(filename)
    if not p.is_file():
        raise BaseException(filename + " does not exist.")

    with open(filename, "r") as f:
        rows, cols = [int(x) for x in f.readline().split()]

        my_map = []
        for _ in range(rows):
            line = f.readline()
            row = []
            for cell in line:
                if cell == "@":
                    row.append(True)
                elif cell == ".":
                    row.append(False)
            my_map.append(row)

        num_agents = int(f.readline())
        starts = []
        for _ in range(num_agents):
            vals = [int(x) for x in f.readline().split()]
            starts.append((vals[0], vals[1]))

    return my_map, starts

# --------------------------------------------------------------
# Concatenated path parser
#
# Reads complete MAPF trajectories represented as:
#
# x1 y1 x2 y2 x3 y3 ...
#
# One line per agent.
#
# Returns:
# - Ordered list of agent trajectories
# --------------------------------------------------------------

def import_concatenated_paths(filename):
    p = Path(filename)
    if not p.is_file():
        raise BaseException(filename + " does not exist.")

    paths = []
    with open(filename, "r") as f:
        for raw in f:
            line = raw.split("#", 1)[0].strip()
            if not line:
                continue
            vals = [int(x) for x in line.split()]
            if len(vals) < 2 or len(vals) % 2 != 0:
                raise BaseException("Invalid concatenated path line: " + line)
            path = []
            for i in range(0, len(vals), 2):
                path.append((vals[i], vals[i + 1]))
            paths.append(path)
    return paths

# --------------------------------------------------------------
# VRPPD goal parser
#
# Reads VRPPD pickup and delivery sequences and extracts
# only the goal locations.
#
# The initial agent position is discarded because it is
# already provided by the MAPF instance file.
# --------------------------------------------------------------

def import_vrppd_goals(filename):
    p = Path(filename)
    if not p.is_file():
        raise BaseException(filename + " does not exist.")

    goals = []
    with open(filename, "r") as f:
        lines = [raw.split("#", 1)[0].strip() for raw in f if raw.strip()]

    i = 0
    while i < len(lines):
        vals = [int(x) for x in lines[i].split()]

        # Se for linha com ID do agente (ex: "0", "1", ...)
        if len(vals) == 1:
            i += 1
            if i >= len(lines):
                raise BaseException("Expected goals after agent index")
            vals = [int(x) for x in lines[i].split()]

        # Agora deve ser linha com coordenadas
        if len(vals) < 4 or len(vals) % 2 != 0:
            raise BaseException("Invalid vrppd_paths line: " + lines[i])

        coords = []
        for j in range(0, len(vals), 2):
            coords.append((vals[j], vals[j + 1]))

        # mantém só os goals (remove o start)
        goals.append(coords[1:])

        i += 1

    return goals

# --------------------------------------------------------------
# Dynamic obstacle event generator
#
# Reconstructs obstacle state transitions from executed
# agent trajectories.
#
# Rules:
# - Pickup goals open obstacle cells.
# - Delivery goals close obstacle cells.
#
# Returns a chronological list of obstacle updates.
# --------------------------------------------------------------

def build_dynamic_obstacle_events(paths, goal_sequences):
    events = []
    for agent, goals in enumerate(goal_sequences):
        g_idx = 0
        for t, loc in enumerate(paths[agent]):
            if g_idx >= len(goals):
                break
            if loc == goals[g_idx]:
                action = "open" if (g_idx % 2 == 0) else "close"
                events.append((t, loc, action))
                g_idx += 1
    events.sort(key=lambda x: x[0])
    return events

# --------------------------------------------------------------
# Example directory resolver
#
# Locates the directory containing the selected benchmark.
#
# Search order:
# 1. solver/instances/<boxes_dir>/example<number>
# 2. solver/instances/example<number> (legacy layout)
# --------------------------------------------------------------

def resolve_example_dir(project_root: Path, number: str, boxes_dir: str) -> Path:
    base = project_root.parent / "solver" / "instances"

    with_boxes = base / boxes_dir / f"example{number}"
    legacy = base / f"example{number}"

    if with_boxes.exists():
        return with_boxes

    if legacy.exists():
        return legacy

    return with_boxes

# --------------------------------------------------------------
# Shortcut argument expansion
#
# Automatically derives all required file paths from the
# selected benchmark instance.
#
# This allows execution using only:
# - Example number
# - Box configuration
# --------------------------------------------------------------

def apply_shortcut_mode(args):
    if args.num_example is None:
        return

    boxes_dir = args.boxes_dir or f"{args.num_boxes}_boxes"
    project_root = PROJECT_ROOT
    example_dir = resolve_example_dir(project_root, args.num_example, boxes_dir)

    if args.instance is None:
        args.instance = str(example_dir / f"example{args.num_example}_icbs_map.txt")
    if args.vrppd_concatenated_paths is None:
        args.vrppd_concatenated_paths = str(
            example_dir / f"example{args.num_example}_icbs_visualize.txt"
        )
    if args.vrppd_paths is None:
        args.vrppd_paths = str(example_dir / f"example{args.num_example}_vrppd_paths.txt")

# --------------------------------------------------------------
# Main visualization pipeline
#
# Workflow:
# 1. Parse CLI arguments
# 2. Resolve benchmark files
# 3. Load ICBS instance
# 4. Load agent trajectories
# 5. Load VRPPD goals
# 6. Build dynamic obstacle events
# 7. Create animation
# 8. Compute makespan
# 9. Save makespan statistics
# 10. Display or export animation
# --------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Visualize VRPPD concatenated MAPF paths")
    parser.add_argument("num_example", nargs="?",
                        help="Example number (ex: 1 for example1)")
    parser.add_argument("num_boxes", nargs="?", type=int, default=9,
                        help="Number of boxes (defines the folder <num boxes>boxes by default)")
    parser.add_argument("--boxes-dir", type=str, default=None,
                        help="Folder inside de solver/instances (ex.: 8_boxes, 10_boxes)")
    parser.add_argument("--instance", type=str, default=None,
                        help="Instance file with map and number of agents")
    parser.add_argument("--vrppd_concatenated_paths", type=str, default=None,
                        help="Concatenated paths file (x y x y ...) per agent")
    parser.add_argument("--vrppd_paths", type=str, default=None,
                        help="VRPPD goals file (sx sy gx1 gy1 gx2 gy2 ...) per agent")
    parser.add_argument("--save", type=str, default=None,
                        help="Optional output video path (e.g., anim.mp4)")
    parser.add_argument("--speed", type=int, default=1,
                        help="Speed multiplier when saving")
    parser.add_argument("--not-show", action="store_true", help="Do not show the animation")
    args = parser.parse_args()
    apply_shortcut_mode(args)

    if args.instance is None or args.vrppd_concatenated_paths is None or args.vrppd_paths is None:
        parser.error(
            "Provide positional args <num_example> <num_boxes> or all flags "
            "--instance, --vrppd_concatenated_paths and --vrppd_paths."
        )

    my_map, starts = import_mapf_instance(args.instance)
    full_paths = import_concatenated_paths(args.vrppd_concatenated_paths)
    all_goals = import_vrppd_goals(args.vrppd_paths)

    if len(full_paths) != len(starts):
        raise BaseException(
            f"concatenated paths has {len(full_paths)} agents, but instance has {len(starts)}"
        )
    if len(all_goals) != len(starts):
        raise BaseException(
            f"vrppd_paths has {len(all_goals)} agents, but instance has {len(starts)}"
        )

    anim = Animation(my_map, starts, full_paths, goal_sequences=all_goals)

    # Makespan equals the total animation time (paths already include pauses)
    makespan = anim.T
    print(f"Makespan: {makespan:.5f} s")  # 5 decimal place precision

    boxes_dir = args.boxes_dir or f"{args.num_boxes}_boxes"

    example_dir = (
        SOLVER_ROOT
        / "instances"
        / boxes_dir
        / f"example{args.num_example}"
    )

    output_file = example_dir / "ICBS_makespan.txt"

    output_file.parent.mkdir(parents=True, exist_ok=True)

    with open(output_file, "w") as f:
        f.write(f"{makespan:.5f}\n")

    if args.save:
        anim.save(args.save, args.speed)
    if args.not_show:
        print("")
    else:
        anim.show()
        print("")

if __name__ == "__main__":
    main()

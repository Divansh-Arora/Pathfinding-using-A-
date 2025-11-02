import tkinter as tk
import heapq
import time

ROWS, COLS = 10, 10
CELL_SIZE = 50
grid = [[0 for _ in range(COLS)] for _ in range(ROWS)]

# Two agents: A (green) and B (blue)
agents = {
    "A": {"start": None, "goal": None, "color": "green"},
    "B": {"start": None, "goal": None, "color": "blue"},
}

placing_obstacles = True
active_agent = "A"  # Default agent for setting points


# ---------------------- A* Algorithm ---------------------- #
def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def astar(grid, start, goal, occupied):
    rows, cols = len(grid), len(grid[0])
    open_set = []
    heapq.heappush(open_set, (0 + heuristic(start, goal), 0, start))
    came_from = {}
    g_score = {start: 0}
    visited = set()

    while open_set:
        _, current_g, current = heapq.heappop(open_set)
        if current in visited:
            continue
        visited.add(current)

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1], visited

        for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]:
            neighbor = (current[0]+dx, current[1]+dy)
            x, y = neighbor
            if (
                0 <= x < rows
                and 0 <= y < cols
                and grid[x][y] == 0
                and neighbor not in occupied
            ):
                tentative_g = current_g + 1
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score, tentative_g, neighbor))
                    came_from[neighbor] = current
    return None, visited


# ---------------------- UI and Visualization ---------------------- #
window = tk.Tk()
window.title("Multi-Agent A* Pathfinding")
canvas = tk.Canvas(window, width=COLS * CELL_SIZE, height=ROWS * CELL_SIZE)
canvas.pack()


def draw_grid(paths={}, visited_sets={}):
    canvas.delete("all")
    for r in range(ROWS):
        for c in range(COLS):
            x1, y1 = c * CELL_SIZE, r * CELL_SIZE
            x2, y2 = x1 + CELL_SIZE, y1 + CELL_SIZE
            cell = (r, c)

            # Base color
            if grid[r][c] == 1:
                color = "black"
            else:
                color = "white"

            # Agent paths and visited nodes
            for name, path in paths.items():
                if cell in visited_sets.get(name, set()):
                    color = "lightgray"
                if cell in path:
                    color = agents[name]["color"]

            # Start and goal points
            for name, data in agents.items():
                if data["start"] == cell:
                    color = agents[name]["color"]
                elif data["goal"] == cell:
                    color = "red"

            canvas.create_rectangle(x1, y1, x2, y2, fill=color, outline="gray")


def on_click(event):
    global placing_obstacles, active_agent
    row, col = event.y // CELL_SIZE, event.x // CELL_SIZE

    if placing_obstacles:
        grid[row][col] = 1 - grid[row][col]
    else:
        agent = agents[active_agent]
        if agent["start"] is None:
            agent["start"] = (row, col)
        elif agent["goal"] is None:
            agent["goal"] = (row, col)
        else:
            agent["start"] = (row, col)
            agent["goal"] = None
    draw_grid()


canvas.bind("<Button-1>", on_click)


def toggle_obstacles():
    global placing_obstacles
    placing_obstacles = not placing_obstacles
    btn_obstacles.config(
        text="Placing Obstacles" if placing_obstacles else "Setting Start/Goal"
    )


def switch_agent():
    global active_agent
    active_agent = "B" if active_agent == "A" else "A"
    btn_agent.config(text=f"Active Agent: {active_agent}")


def start_visualization():
    a, b = agents["A"], agents["B"]
    if not (a["start"] and a["goal"] and b["start"] and b["goal"]):
        print("Please set both start and goal for each agent.")
        return

    visualize_multi_agent()


btn_obstacles = tk.Button(window, text="Placing Obstacles", command=toggle_obstacles)
btn_obstacles.pack(side="left", padx=10, pady=10)

btn_agent = tk.Button(window, text="Active Agent: A", command=switch_agent)
btn_agent.pack(side="left", padx=10, pady=10)

btn_start = tk.Button(window, text="Start Multi-Agent A*", command=start_visualization)
btn_start.pack(side="right", padx=10, pady=10)


# ---------------------- Multi-Agent Simulation ---------------------- #
def visualize_multi_agent():
    occupied = set()
    paths, visited_sets = {}, {}

    for name, data in agents.items():
        path, visited = astar(grid, data["start"], data["goal"], occupied)
        if not path:
            print(f"No path found for Agent {name}")
            return
        paths[name] = path
        visited_sets[name] = visited
        occupied.update(path)  # reserve path to prevent collision

    draw_grid(paths, visited_sets)

    # Animate movement
    max_len = max(len(p) for p in paths.values())

    def animate(step=0):
        if step < max_len:
            occupied.clear()
            for name, path in paths.items():
                if step < len(path):
                    agents[name]["start"] = path[step]
                    occupied.add(path[step])
            draw_grid(paths={n: [p[min(step, len(p)-1)] for p in [paths[n]]] for n in paths})
            window.after(300, animate, step + 1)
        else:
            print("All agents reached their goals!")

    animate()


window.mainloop()

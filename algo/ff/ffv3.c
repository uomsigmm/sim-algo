#include "api.h"
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <limits.h> // For INT_MAX

// --- Constants ---
static const int MAZE_WIDTH = 16;
static const int MAZE_HEIGHT = 16;
#define MAX_CELLS (MAZE_WIDTH * MAZE_HEIGHT)
#define INVALID_DISTANCE (MAX_CELLS) // Represents infinity

// Goal cells (0-indexed coordinates, center 4 cells)
static const int GOAL_X1 = 7;
static const int GOAL_Y1 = 7;
static const int GOAL_X2 = 8;
static const int GOAL_Y2 = 8;

// --- Enums ---
typedef enum {
    NORTH = 0,
    EAST = 1,
    SOUTH = 2,
    WEST = 3,
    DIRECTION_COUNT = 4 // Helper for loops/arrays
} Direction;

typedef enum {
    SEARCH_MODE, // Explore to find the goal
    RETURN_MODE, // Return to start after finding the goal
    SPEED_MODE   // Fast run from start to goal using known path
} RunMode;

// --- Structs ---

// Represents a coordinate point
typedef struct {
    int x;
    int y;
} Point;

// Holds the mouse's current state
typedef struct {
    Point pos;
    Direction orientation;
    RunMode mode;
    bool goal_found;
    Point shortest_path[MAX_CELLS]; // Stores the computed shortest path
    int path_length;
} MouseState;

// Holds the maze's discovered state
typedef struct {
    int distances[MAZE_WIDTH][MAZE_HEIGHT];     // Distance values for flood fill
    bool walls[MAZE_WIDTH][MAZE_HEIGHT][DIRECTION_COUNT]; // Wall information
    bool visited[MAZE_WIDTH][MAZE_HEIGHT];    // Visited cells during search
} Maze;

// --- Global State (Encapsulated in Structs) ---
MouseState mouse;
Maze maze;

// --- Direction Deltas (Consistent Order with Direction Enum) ---
// Indexed by Direction enum: NORTH, EAST, SOUTH, WEST
const Point direction_delta[DIRECTION_COUNT] = {
    {0, 1},  // NORTH
    {1, 0},  // EAST
    {0, -1}, // SOUTH
    {-1, 0}  // WEST
};

// --- Function Prototypes ---
void init_simulation(void);
void init_maze(Maze *m);
void init_mouse(MouseState *ms);

bool is_within_bounds(Point p);
bool is_at_goal(Point p);
bool is_at_start(Point p);
Direction get_opposite_direction(Direction dir);

void update_walls_current_cell(MouseState *ms, Maze *m);
void set_wall(Maze *m, Point p, Direction dir);
bool has_wall(const Maze *m, Point p, Direction dir);

void flood_fill(Maze *m, Point target);
void flood_fill_goal(Maze *m);
void flood_fill_start(Maze *m);

Direction choose_next_direction(const MouseState *ms, const Maze *m);
void turn_to_direction(MouseState *ms, Direction target_dir);
void move_forward_update_state(MouseState *ms, Maze *m);

void compute_shortest_path(MouseState *ms, Maze *m);
bool verify_path_exploration(const MouseState *ms, const Maze *m);
void follow_shortest_path(MouseState *ms, Maze *m);

void update_display(const MouseState *ms, const Maze *m);
void log_message(const char *msg);

// --- Main Function ---
int main(int argc, char *argv[]) {
    log_message("Starting maze solver");
    init_simulation();

    while (true) {

        if (API_wasReset()) {
            log_message("Simulator reset detected!");
            API_ackReset();
            init_simulation(); // Reset everything
        }

        // 1. Sense Walls & Update Map
        update_walls_current_cell(&mouse, &maze);

        // 2. Mark current cell as visited
        maze.visited[mouse.pos.x][mouse.pos.y] = true;

        // 3. Update Display
        update_display(&mouse, &maze);

        // 4. State Machine Logic
        switch (mouse.mode) {
            case SEARCH_MODE:
                if (is_at_goal(mouse.pos)) {
                    log_message("=== Goal reached! Switching to RETURN_MODE ===");
                    mouse.goal_found = true;
                    mouse.mode = RETURN_MODE;
                    flood_fill_start(&maze); // Recalculate distances for return trip
                } else {
                    flood_fill_goal(&maze); // Ensure distances point towards goal
                    move_forward_update_state(&mouse, &maze); // Decide and move
                }
                break;

                case RETURN_MODE:
                    if (is_at_start(mouse.pos)) {
                        log_message("=== Back at start! Preparing for speed run ===");
                        // Optional: Final wall update at start
                        update_walls_current_cell(&mouse, &maze);

                        // Compute the shortest path based on current knowledge
                        compute_shortest_path(&mouse, &maze);

                        if (mouse.path_length > 0) { // Only verify if a path was actually found
                            // Verify if the computed path is safe (only uses explored cells)
                            if (verify_path_exploration(&mouse, &maze)) {
                                // Path is safe, proceed to speed run
                                log_message("=== Path verified! Switching to SPEED_MODE ===");
                                mouse.mode = SPEED_MODE;
                            } else {
                                // Path is unsafe, needs more exploration along the computed path
                                log_message("=== Path requires exploration! Returning to SEARCH_MODE ===");

                                // Find the first unvisited cell on the path to target
                                Point target_unvisited = mouse.pos; // Default to current if error
                                for (int i = 0; i < mouse.path_length; ++i) {
                                    Point p = mouse.shortest_path[i];
                                    if (!maze.visited[p.x][p.y]) {
                                        target_unvisited = p;
                                        char buffer[100];
                                        sprintf(buffer, "Targeting first unvisited cell on path: (%d,%d)", target_unvisited.x, target_unvisited.y);
                                        log_message(buffer);
                                        break;
                                    }
                                }

                                // Flood fill towards the target unvisited cell to guide exploration
                                flood_fill(&maze, target_unvisited);
                                mouse.mode = SEARCH_MODE; // Go back to exploration mode
                                // The main loop will now use the new distances to explore towards the target
                            }
                        } else {
                             log_message("ERROR: No path computed after returning to start. Cannot proceed.");
                             // Handle error: maybe try flooding to goal and searching again? Or stop.
                             // For now, just stay put or enter a safe stop state.
                             // Consider adding a specific ERROR_MODE?
                             // As a fallback, maybe just try searching again:
                             log_message("Attempting to re-initiate search from start.");
                             flood_fill_goal(&maze);
                             mouse.mode = SEARCH_MODE;
                        }

                    } else {
                        // Still returning to start
                        flood_fill_start(&maze); // Ensure distances point towards start
                        move_forward_update_state(&mouse, &maze); // Decide and move
                    }
                    break;

                case SPEED_MODE:
                     // Speed mode logic remains the same as before
                     log_message("=== Beginning speed run ===");
                     follow_shortest_path(&mouse, &maze);
                     log_message("=== Speed run finished (check log for success/failure) ===");
                     // Optionally add a loop here to wait for reset, or just exit.
                     // API_setColor(mouse.pos.x, mouse.pos.y, 'F'); // Mark final spot
                     // while(!API_wasReset()) { /* wait */ }
                     return 0; // End program after speed run attempt
        }

        // Simple debug message
        char buffer[100];
        sprintf(buffer, "State: Pos=(%d,%d) Orient=%d Mode=%d GoalFound=%d",
                mouse.pos.x, mouse.pos.y, mouse.orientation, mouse.mode, mouse.goal_found);
        log_message(buffer);
    }

    return 0; // Should not be reached in normal operation
}

// --- Initialization Functions ---

void init_simulation(void) {
    log_message("Initializing simulation state...");
    init_maze(&maze);
    init_mouse(&mouse);
    // Initial flood fill towards goal for the first search phase
    flood_fill_goal(&maze);
}

void init_maze(Maze *m) {
    for (int x = 0; x < MAZE_WIDTH; x++) {
        for (int y = 0; y < MAZE_HEIGHT; y++) {
            m->distances[x][y] = INVALID_DISTANCE;
            m->visited[x][y] = false;
            for (Direction dir = 0; dir < DIRECTION_COUNT; dir++) {
                m->walls[x][y][dir] = false; // Assume no walls initially (except boundaries)
            }
        }
    }

    // Set outer boundary walls
    for (int i = 0; i < MAZE_WIDTH; i++) {
        set_wall(m, (Point){i, 0}, SOUTH);
        set_wall(m, (Point){i, MAZE_HEIGHT - 1}, NORTH);
    }
    for (int i = 0; i < MAZE_HEIGHT; i++) {
        set_wall(m, (Point){0, i}, WEST);
        set_wall(m, (Point){MAZE_WIDTH - 1, i}, EAST);
    }
}

void init_mouse(MouseState *ms) {
    ms->pos = (Point){0, 0};          // Start at (0,0)
    ms->orientation = NORTH;          // Facing North initially
    ms->mode = SEARCH_MODE;
    ms->goal_found = false;
    ms->path_length = 0;
    maze.visited[0][0] = true; // Mark starting cell visited
}

// --- Coordinate and Boundary Checks ---

bool is_within_bounds(Point p) {
    return p.x >= 0 && p.x < MAZE_WIDTH && p.y >= 0 && p.y < MAZE_HEIGHT;
}

bool is_at_goal(Point p) {
    return (p.x == GOAL_X1 || p.x == GOAL_X2) && (p.y == GOAL_Y1 || p.y == GOAL_Y2);
}

bool is_at_start(Point p) {
    return p.x == 0 && p.y == 0;
}

Direction get_opposite_direction(Direction dir) {
    return (Direction)((dir + 2) % DIRECTION_COUNT);
}

// --- Wall Management ---

// Updates walls for the current cell based on sensor readings
void update_walls_current_cell(MouseState *ms, Maze *m) {
    Point current_pos = ms->pos;
    Direction current_orient = ms->orientation;

    // Directions relative to the mouse's orientation
    Direction front_dir = current_orient;
    Direction right_dir = (Direction)((current_orient + 1) % DIRECTION_COUNT);
    Direction left_dir = (Direction)((current_orient + 3) % DIRECTION_COUNT);

    if (API_wallFront()) {
        set_wall(m, current_pos, front_dir);
    }
    if (API_wallRight()) {
        set_wall(m, current_pos, right_dir);
    }
    if (API_wallLeft()) {
        set_wall(m, current_pos, left_dir);
    }

    // Log detected walls (optional)
    // char buffer[80];
    // sprintf(buffer, "Walls at (%d,%d): N=%d E=%d S=%d W=%d",
    //         current_pos.x, current_pos.y,
    //         m->walls[current_pos.x][current_pos.y][NORTH],
    //         m->walls[current_pos.x][current_pos.y][EAST],
    //         m->walls[current_pos.x][current_pos.y][SOUTH],
    //         m->walls[current_pos.x][current_pos.y][WEST]);
    // log_message(buffer);
}

// Sets a wall and its corresponding neighbor's wall
void set_wall(Maze *m, Point p, Direction dir) {
    if (!is_within_bounds(p)) return;

    m->walls[p.x][p.y][dir] = true;

    // Update the neighboring cell's perspective
    Point neighbor_pos = {p.x + direction_delta[dir].x, p.y + direction_delta[dir].y};
    if (is_within_bounds(neighbor_pos)) {
        Direction opposite_dir = get_opposite_direction(dir);
        m->walls[neighbor_pos.x][neighbor_pos.y][opposite_dir] = true;
    }
}

// Checks if a wall exists from the maze's perspective
bool has_wall(const Maze *m, Point p, Direction dir) {
    if (!is_within_bounds(p)) return true; // Treat out of bounds as walls
    return m->walls[p.x][p.y][dir];
}


// --- Flood Fill Algorithm (Manhattan Distance) ---

// General flood fill from a target point
void flood_fill(Maze *m, Point target) {
    Point queue[MAX_CELLS];
    int q_head = 0;
    int q_tail = 0;

    // Reset all distances
    for (int x = 0; x < MAZE_WIDTH; x++) {
        for (int y = 0; y < MAZE_HEIGHT; y++) {
            m->distances[x][y] = INVALID_DISTANCE;
        }
    }

    // Add target to queue and set its distance to 0
    if (is_within_bounds(target)) {
        m->distances[target.x][target.y] = 0;
        queue[q_tail++] = target;
    } else {
        log_message("ERROR: Flood fill target out of bounds!");
        return;
    }

    // Breadth-First Search
    while (q_head < q_tail) {
        Point current = queue[q_head++];
        int current_dist = m->distances[current.x][current.y];

        // Explore neighbors
        for (Direction dir = 0; dir < DIRECTION_COUNT; dir++) {
            // If there's a wall in this direction, skip
            if (has_wall(m, current, dir)) {
                continue;
            }

            Point neighbor = {current.x + direction_delta[dir].x, current.y + direction_delta[dir].y};

            // If neighbor is valid and has a higher distance, update it
            if (is_within_bounds(neighbor) && m->distances[neighbor.x][neighbor.y] > current_dist + 1) {
                m->distances[neighbor.x][neighbor.y] = current_dist + 1;
                queue[q_tail++] = neighbor; // Add neighbor to queue
            }
        }
    }
}

// Flood fill targeting the center goal area
// Improvement: Flood fill from *all* goal cells simultaneously
void flood_fill_goal(Maze *m) {
    Point queue[MAX_CELLS];
    int q_head = 0;
    int q_tail = 0;

    // Reset all distances
    for (int x = 0; x < MAZE_WIDTH; x++) {
        for (int y = 0; y < MAZE_HEIGHT; y++) {
            m->distances[x][y] = INVALID_DISTANCE;
        }
    }

    // Add all goal cells to the queue with distance 0
    for (int x = GOAL_X1; x <= GOAL_X2; ++x) {
        for (int y = GOAL_Y1; y <= GOAL_Y2; ++y) {
            Point goal_cell = {x,y};
             if (is_within_bounds(goal_cell)) {
                m->distances[goal_cell.x][goal_cell.y] = 0;
                queue[q_tail++] = goal_cell;
            }
        }
    }

     if (q_tail == 0) {
        log_message("ERROR: No valid goal cells found for flood fill!");
        return;
    }

    // Breadth-First Search (same as general flood_fill from here)
    while (q_head < q_tail) {
        Point current = queue[q_head++];
        int current_dist = m->distances[current.x][current.y];

        for (Direction dir = 0; dir < DIRECTION_COUNT; dir++) {
            if (has_wall(m, current, dir)) continue;

            Point neighbor = {current.x + direction_delta[dir].x, current.y + direction_delta[dir].y};

            if (is_within_bounds(neighbor) && m->distances[neighbor.x][neighbor.y] > current_dist + 1) {
                m->distances[neighbor.x][neighbor.y] = current_dist + 1;
                queue[q_tail++] = neighbor;
            }
        }
    }
}


// Flood fill targeting the start cell (0,0)
void flood_fill_start(Maze *m) {
    flood_fill(m, (Point){0, 0});
}


// --- Movement Logic ---

// Decides the best direction to move next based on flood fill distances
// Prefers lower distance values. In SEARCH_MODE, adds a small bias towards unvisited cells.
Direction choose_next_direction(const MouseState *ms, const Maze *m) {
    Point current_pos = ms->pos;
    int min_dist = INVALID_DISTANCE + 10; // Initialize higher than max possible distance + bonus
    Direction best_dir = NORTH; // Default, should be overridden
    bool found_move = false;

    // Check all four directions
    for (Direction dir = 0; dir < DIRECTION_COUNT; dir++) {
        // Skip if there's a wall
        if (has_wall(m, current_pos, dir)) {
            continue;
        }

        Point neighbor = {current_pos.x + direction_delta[dir].x, current_pos.y + direction_delta[dir].y};

        // Skip if neighbor is out of bounds (shouldn't happen if wall checks work)
        if (!is_within_bounds(neighbor)) {
            continue;
        }

        int neighbor_dist = m->distances[neighbor.x][neighbor.y];

        // Add exploration bonus in SEARCH_MODE to prefer unvisited cells slightly
        int adjusted_dist = neighbor_dist;
        if (ms->mode == SEARCH_MODE && !m->visited[neighbor.x][neighbor.y]) {
             // Make unvisited significantly more attractive than visited cells with the *same* base distance.
             // If an unvisited cell has a higher base distance, we still prefer lower distance overall.
             adjusted_dist -= 1; // Simple bonus - adjust magnitude as needed
        }


        // If this neighbor has a lower (potentially adjusted) distance, it's the new best
        if (adjusted_dist < min_dist) {
            min_dist = adjusted_dist;
            best_dir = dir;
            found_move = true;
        }
    }

    if (!found_move) {
        // This should ideally not happen if flood fill is correct and there's a path
        log_message("ERROR: No valid move found! Stuck?");
        // If stuck, maybe turn around as a fallback?
        best_dir = get_opposite_direction(ms->orientation);
    }

    return best_dir;
}

// Turns the mouse to face the target direction using minimal turns
void turn_to_direction(MouseState *ms, Direction target_dir) {
    if (ms->orientation == target_dir) {
        return; // Already facing the right way
    }

    int diff = (target_dir - ms->orientation + DIRECTION_COUNT) % DIRECTION_COUNT;

    if (diff == 1) { // 90 degrees right
        log_message("Turning right");
        API_turnRight();
        ms->orientation = (Direction)((ms->orientation + 1) % DIRECTION_COUNT);
    } else if (diff == 3) { // 90 degrees left (270 right)
        log_message("Turning left");
        API_turnLeft();
        ms->orientation = (Direction)((ms->orientation + 3) % DIRECTION_COUNT);
    } else { // 180 degrees
        log_message("Turning around (two rights)");
        API_turnRight();
        ms->orientation = (Direction)((ms->orientation + 1) % DIRECTION_COUNT);
        API_turnRight();
        ms->orientation = (Direction)((ms->orientation + 1) % DIRECTION_COUNT);
    }
}

// Chooses direction, turns, moves forward, and updates state.
// Handles unexpected walls discovered during movement.
void move_forward_update_state(MouseState *ms, Maze *m) {
    // 1. Decide where to go
    Direction next_dir = choose_next_direction(ms, m);

    // 2. Turn to face that direction
    turn_to_direction(ms, next_dir);

    // 3. Attempt to move forward
    log_message("Moving forward");
    if (API_moveForward()) {
        // 4a. Move successful: Update mouse position
        ms->pos.x += direction_delta[ms->orientation].x;
        ms->pos.y += direction_delta[ms->orientation].y;
    } else {
        // 4b. Move failed: Hit an unexpected wall
        log_message("WARN: Move failed - unexpected wall detected!");
        set_wall(m, ms->pos, ms->orientation); // Update wall map

        // Re-run flood fill as the distances are now potentially incorrect
        if (ms->mode == SEARCH_MODE) {
             log_message("Recalculating distances to goal due to new wall.");
             flood_fill_goal(m);
        } else if (ms->mode == RETURN_MODE) {
             log_message("Recalculating distances to start due to new wall.");
             flood_fill_start(m);
        }
         // We don't move, stay in the same cell for the next iteration
    }
}


// --- Pathfinding and Following ---

// Computes the shortest path from start (0,0) to the goal area using the current maze map
void compute_shortest_path(MouseState *ms, Maze *m) {
    log_message("Computing shortest path from start to goal...");

    // Ensure distances are calculated relative to the goal
    flood_fill_goal(m);

    Point current_pos = {0, 0};
    ms->path_length = 0;

    // Check if start cell is reachable
    if (m->distances[0][0] == INVALID_DISTANCE) {
         log_message("ERROR: Start cell is unreachable from goal!");
         ms->path_length = 0;
         return;
    }

    // Store start position
    ms->shortest_path[ms->path_length++] = current_pos;

    // Trace path back from start using lowest distance neighbors
    while (!is_at_goal(current_pos)) {
        int min_dist = m->distances[current_pos.x][current_pos.y];
        Direction best_dir = NORTH; // Placeholder
        bool found_next = false;

        for (Direction dir = 0; dir < DIRECTION_COUNT; dir++) {
            if (has_wall(m, current_pos, dir)) {
                continue;
            }

            Point neighbor = {current_pos.x + direction_delta[dir].x, current_pos.y + direction_delta[dir].y};

            if (is_within_bounds(neighbor) && m->distances[neighbor.x][neighbor.y] < min_dist) {
                min_dist = m->distances[neighbor.x][neighbor.y];
                best_dir = dir;
                found_next = true;
            }
        }

        if (!found_next) {
            log_message("ERROR: Could not find next step while computing shortest path! Path broken?");
            // Mark path as invalid?
            ms->path_length = 0;
            return;
        }

        // Move to the best neighbor
        current_pos.x += direction_delta[best_dir].x;
        current_pos.y += direction_delta[best_dir].y;

        // Store the new position in the path
        if (ms->path_length < MAX_CELLS) {
             ms->shortest_path[ms->path_length++] = current_pos;
        } else {
             log_message("ERROR: Shortest path exceeds maximum length!");
             ms->path_length = 0; // Indicate error
             return;
        }
         // Log path points (optional)
        // char buffer[50];
        // sprintf(buffer, "Path point %d: (%d,%d)", ms->path_length - 1, current_pos.x, current_pos.y);
        // log_message(buffer);
    }

    char buffer[80];
    sprintf(buffer, "Shortest path computed with %d steps (length %d including start).", ms->path_length -1, ms->path_length);
    log_message(buffer);
}

// Checks if the current computed shortest path goes only through visited cells.
bool verify_path_exploration(const MouseState *ms, const Maze *m) {
    if (ms->path_length <= 1) {
        log_message("Path verification: Path is too short or invalid.");
        return false; // Cannot run speed mode on an empty/single-cell path
    }

    log_message("Verifying path exploration...");
    for (int i = 0; i < ms->path_length; ++i) {
        Point p = ms->shortest_path[i];
        if (!is_within_bounds(p)) {
             char buffer[100];
             sprintf(buffer, "Path verification FAILED: Point %d (%d,%d) is out of bounds.", i, p.x, p.y);
             log_message(buffer);
             return false; // Should not happen if compute_shortest_path is correct
        }
        if (!m->visited[p.x][p.y]) {
            char buffer[100];
            sprintf(buffer, "Path verification FAILED: Point %d (%d,%d) on path was not visited.", i, p.x, p.y);
            log_message(buffer);
            return false; // Found an unvisited cell on the path
        }
         // Optional stricter check: Ensure the transition between path[i-1] and path[i] is known open.
         // compute_shortest_path should already handle this via has_wall, but double-checking adds robustness.
         if (i > 0) {
             Point prev_p = ms->shortest_path[i-1];
             Direction move_dir = NORTH; // Find direction from prev_p to p
             bool dir_found = false;
             for(Direction d = 0; d < DIRECTION_COUNT; ++d) {
                 if (prev_p.x + direction_delta[d].x == p.x && prev_p.y + direction_delta[d].y == p.y) {
                     move_dir = d;
                     dir_found = true;
                     break;
                 }
             }
             if (!dir_found || has_wall(m, prev_p, move_dir)) {
                  char buffer[120];
                  sprintf(buffer, "Path verification FAILED: Transition from (%d,%d) to (%d,%d) uses unknown/walled path segment.", prev_p.x, prev_p.y, p.x, p.y);
                  log_message(buffer);
                  return false;
             }
         }
    }

    log_message("Path verification PASSED: Path is fully explored.");
    return true; // All cells on the path are visited
}

// Executes the speed run along the pre-computed shortest path
void follow_shortest_path(MouseState *ms, Maze *m) {
    log_message("Starting speed run execution...");

     // Ensure mouse is at start and facing North (or a default direction)
     if (!is_at_start(ms->pos)) {
        log_message("ERROR: Cannot start speed run, mouse not at (0,0)!");
        return; // Or potentially navigate back to start first
     }
     turn_to_direction(ms, NORTH); // Ensure consistent starting orientation


    if (ms->path_length <= 1) {
        log_message("WARN: No valid path computed for speed run.");
        return;
    }

    // Follow the path from the second point (index 1) onwards
    for (int i = 1; i < ms->path_length; i++) {
        Point target_pos = ms->shortest_path[i];

        // Determine direction needed to move from current to target
        Direction move_dir = NORTH; // Placeholder
        bool dir_found = false;
        for (Direction dir = 0; dir < DIRECTION_COUNT; ++dir) {
            if (ms->pos.x + direction_delta[dir].x == target_pos.x &&
                ms->pos.y + direction_delta[dir].y == target_pos.y)
            {
                move_dir = dir;
                dir_found = true;
                break;
            }
        }

        if (!dir_found) {
            char buffer[100];
            sprintf(buffer, "ERROR: Speed run path invalid. Cannot determine move direction from (%d,%d) to (%d,%d)",
                    ms->pos.x, ms->pos.y, target_pos.x, target_pos.y);
            log_message(buffer);
            return; // Path is broken
        }

        // Turn to face the required direction
        turn_to_direction(ms, move_dir);

        // Move forward (expecting no walls along the computed path)
        log_message("Speed run: Moving forward");
        if (API_moveForward()) {
            // Update position successfully
            ms->pos = target_pos;
            update_display(ms, m); // Update display after each move
        } else {
            // This indicates a major inconsistency between the computed path and reality
            char buffer[120];
            sprintf(buffer, "FATAL ERROR: Speed run failed! Hit unexpected wall moving from (%d,%d) towards (%d,%d) facing %d. Map is wrong!",
                    ms->shortest_path[i-1].x, ms->shortest_path[i-1].y, // Log previous position
                    target_pos.x, target_pos.y, ms->orientation);
            log_message(buffer);
            // Update the wall map based on this new information
            set_wall(m, ms->pos, ms->orientation);
            update_display(ms, m);
            // Abort speed run? Or try to recompute? For now, abort.
            return;
        }

         // Log progress
        char buffer[80];
        sprintf(buffer, "Speed run: Reached (%d,%d)", ms->pos.x, ms->pos.y);
        log_message(buffer);
    }

    // Check if we actually ended up in a goal cell
    if (is_at_goal(ms->pos)) {
         log_message("=== Speed run complete! Goal successfully reached! ===");
         API_setColor(ms->pos.x, ms->pos.y, 'G'); // Final confirmation color
    } else {
         log_message("ERROR: Speed run finished, but not at a goal cell!");
    }
}


// --- Utility Functions ---

// Updates the simulator display
void update_display(const MouseState *ms, const Maze *m) {
    // Clear previous colors/text? API might handle this. Assume it does.

    for (int x = 0; x < MAZE_WIDTH; x++) {
        for (int y = 0; y < MAZE_HEIGHT; y++) {
            Point p = {x, y};
            // Set text (distance value)
            if (m->distances[x][y] == INVALID_DISTANCE) {
                API_setText(x, y, "-");
            } else {
                char buffer[8];
                sprintf(buffer, "%d", m->distances[x][y]);
                API_setText(x, y, buffer);
            }

            // Set cell color based on state
            if (p.x == ms->pos.x && p.y == ms->pos.y) {
                API_setColor(x, y, 'R'); // Current mouse position: Red
            } else if (is_at_goal(p)) {
                API_setColor(x, y, 'G'); // Goal cells: Green
            } else if (m->visited[x][y]) {
                API_setColor(x, y, 'B'); // Visited cells: Blue
            } else {
                API_setColor(x, y, 'Y'); // Unvisited cells: Yellow
            }

             // Highlight the shortest path during speed mode or after computation
            if (ms->mode == SPEED_MODE && ms->path_length > 0) {
                bool on_path = false;
                for(int i = 0; i < ms->path_length; ++i) {
                    if (ms->shortest_path[i].x == x && ms->shortest_path[i].y == y) {
                        on_path = true;
                        break;
                    }
                }
                 // Don't overwrite current pos or goal color
                if (on_path && !(p.x == ms->pos.x && p.y == ms->pos.y) && !is_at_goal(p)) {
                    API_setColor(x, y, 'C'); // Path cells: Cyan
                }
            }


            // Draw known walls
            if (m->walls[x][y][NORTH]) API_setWall(x, y, 'n');
            if (m->walls[x][y][EAST]) API_setWall(x, y, 'e');
            if (m->walls[x][y][SOUTH]) API_setWall(x, y, 's');
            if (m->walls[x][y][WEST]) API_setWall(x, y, 'w');
        }
    }
}

// Logs a message to the simulator console (stderr)
void log_message(const char *msg) {
    fprintf(stderr, "%s\n", msg);
    fflush(stderr); // Ensure message is displayed immediately
}

/// ffv0.c -- clean implementation based on ffv3 with enhanced logging
/// demonstrates best practices for space-efficient debug output
/// - uses LOG_PHASE for phase transitions (high visibility)
/// - uses LOG_STEP for position traces (compact format)
/// - uses LOG for algorithm events (path computation, verification)
/// - uses LOG_ERROR for failures
///
/// algorithm logic is identical to ffv3.c; only logging changed.
/// see tmp/log_output_analysis.md for logging strategy rationale.

#include "api.h"
#include "log.h"
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <limits.h>

// --- Constants ---
#define MAZE_WIDTH 16
#define MAZE_HEIGHT 16
#define MAX_CELLS (MAZE_WIDTH * MAZE_HEIGHT)
#define INVALID_DISTANCE (MAX_CELLS)

// Goal cells (0-indexed, center 4 cells)
#define GOAL_X1 7
#define GOAL_Y1 7
#define GOAL_X2 8
#define GOAL_Y2 8

// --- Enums ---
typedef enum {
  NORTH = 0,
  EAST = 1,
  SOUTH = 2,
  WEST = 3,
  DIRECTION_COUNT = 4
} Direction;

typedef enum {
  SEARCH_MODE,
  RETURN_MODE,
  SPEED_MODE
} RunMode;

// --- Structs ---

typedef struct {
  int x;
  int y;
} Point;

typedef struct {
  Point     pos;
  Direction orientation;
  RunMode   mode;
  bool      goal_found;
  Point     shortest_path[MAX_CELLS];
  int       path_length;
} MouseState;

typedef struct {
  int  distances[MAZE_WIDTH][MAZE_HEIGHT];
  bool walls[MAZE_WIDTH][MAZE_HEIGHT][DIRECTION_COUNT];
  bool visited[MAZE_WIDTH][MAZE_HEIGHT];
} Maze;

// --- Logging Support ---
static int g_step = 0;
static const char DIR_CHAR[4] = {'N', 'E', 'S', 'W'};

// --- Global State ---
MouseState mouse;
Maze       maze;

// --- Direction Deltas (indexed by Direction enum) ---
const Point direction_delta[DIRECTION_COUNT] = {
  {0, 1},   // NORTH
  {1, 0},   // EAST
  {0, -1},  // SOUTH
  {-1, 0}   // WEST
};

// --- Function Prototypes ---
void init_simulation(void);
void init_maze(Maze *m);
void init_mouse(MouseState *ms, Maze *m);

bool      is_within_bounds(Point p);
bool      is_at_goal(Point p);
bool      is_at_start(Point p);
Direction get_opposite_direction(Direction dir);

void update_walls_current_cell(MouseState *ms, Maze *m);
void set_wall(Maze *m, Point p, Direction dir);
bool has_wall(const Maze *m, Point p, Direction dir);

void flood_fill(Maze *m, Point target);
void flood_fill_goal(Maze *m);
void flood_fill_start(Maze *m);

Direction choose_next_direction(const MouseState *ms,
                                const Maze *m);
void      turn_to_direction(MouseState *ms, Direction target);
void      move_forward_update_state(MouseState *ms, Maze *m);

void compute_shortest_path(MouseState *ms, Maze *m);
bool verify_path_exploration(const MouseState *ms,
                             const Maze *m);
void follow_shortest_path(MouseState *ms, Maze *m);

void update_display(const MouseState *ms, const Maze *m);

// --- Main Function ---
int main(void) {
  LOG("starting maze solver");
  init_simulation();
  LOG("init complete");

  while (true) {
    if (API_wasReset()) {
      LOG("simulator reset detected");
      API_ackReset();
      g_step = 0;
      init_simulation();
    }

    // 1. sense walls and update map
    update_walls_current_cell(&mouse, &maze);

    // 2. mark current cell as visited
    maze.visited[mouse.pos.x][mouse.pos.y] = true;

    // 3. update simulator display
    update_display(&mouse, &maze);

    // 4. state machine
    switch (mouse.mode) {
    case SEARCH_MODE:
      if (is_at_goal(mouse.pos)) {
        LOG_PHASE("SEARCH -> RETURN [step #%d, pos (%d,%d)]",
                  g_step, mouse.pos.x, mouse.pos.y);
        mouse.goal_found = true;
        mouse.mode = RETURN_MODE;
        flood_fill_start(&maze);
      } else {
        flood_fill_goal(&maze);
        move_forward_update_state(&mouse, &maze);
      }
      break;

    case RETURN_MODE:
      if (is_at_start(mouse.pos)) {
        update_walls_current_cell(&mouse, &maze);
        compute_shortest_path(&mouse, &maze);

        if (mouse.path_length > 0) {
          if (verify_path_exploration(&mouse, &maze)) {
            LOG_PHASE(
              "RETURN -> SPEED [step #%d, path: %d steps]",
              g_step, mouse.path_length - 1);
            mouse.mode = SPEED_MODE;
          } else {
            // find first unvisited cell on path
            Point target = mouse.pos;
            for (int i = 0; i < mouse.path_length; ++i) {
              Point p = mouse.shortest_path[i];
              if (!maze.visited[p.x][p.y]) {
                target = p;
                break;
              }
            }
            LOG_PHASE(
              "RETURN -> SEARCH [step #%d, "
              "unvisited: (%d,%d)]",
              g_step, target.x, target.y);
            flood_fill(&maze, target);
            mouse.mode = SEARCH_MODE;
          }
        } else {
          LOG_ERROR("no path computed after return to start");
          LOG("re-initiating search from start");
          flood_fill_goal(&maze);
          mouse.mode = SEARCH_MODE;
        }
      } else {
        flood_fill_start(&maze);
        move_forward_update_state(&mouse, &maze);
      }
      break;

    case SPEED_MODE:
      LOG_PHASE("SPEED RUN START [step #%d]", g_step);
      follow_shortest_path(&mouse, &maze);
      LOG_PHASE("SPEED RUN DONE [step #%d, pos (%d,%d)]",
                g_step, mouse.pos.x, mouse.pos.y);
      return 0;
    }
  }

  return 0;
}

// --- Initialization ---

void init_simulation(void) {
  init_maze(&maze);
  init_mouse(&mouse, &maze);
  flood_fill_goal(&maze);
}

void init_maze(Maze *m) {
  for (int x = 0; x < MAZE_WIDTH; x++) {
    for (int y = 0; y < MAZE_HEIGHT; y++) {
      m->distances[x][y] = INVALID_DISTANCE;
      m->visited[x][y] = false;
      for (Direction dir = 0; dir < DIRECTION_COUNT; dir++) {
        m->walls[x][y][dir] = false;
      }
    }
  }

  // set outer boundary walls
  for (int i = 0; i < MAZE_WIDTH; i++) {
    set_wall(m, (Point){i, 0}, SOUTH);
    set_wall(m, (Point){i, MAZE_HEIGHT - 1}, NORTH);
  }
  for (int i = 0; i < MAZE_HEIGHT; i++) {
    set_wall(m, (Point){0, i}, WEST);
    set_wall(m, (Point){MAZE_WIDTH - 1, i}, EAST);
  }
}

void init_mouse(MouseState *ms, Maze *m) {
  ms->pos = (Point){0, 0};
  ms->orientation = NORTH;
  ms->mode = SEARCH_MODE;
  ms->goal_found = false;
  ms->path_length = 0;
  m->visited[0][0] = true;
}

// --- Coordinate and Boundary Checks ---

bool is_within_bounds(Point p) {
  return p.x >= 0 && p.x < MAZE_WIDTH
      && p.y >= 0 && p.y < MAZE_HEIGHT;
}

bool is_at_goal(Point p) {
  return (p.x == GOAL_X1 || p.x == GOAL_X2)
      && (p.y == GOAL_Y1 || p.y == GOAL_Y2);
}

bool is_at_start(Point p) {
  return p.x == 0 && p.y == 0;
}

Direction get_opposite_direction(Direction dir) {
  return (Direction)((dir + 2) % DIRECTION_COUNT);
}

// --- Wall Management ---

void update_walls_current_cell(MouseState *ms, Maze *m) {
  Point     pos = ms->pos;
  Direction orient = ms->orientation;

  Direction front = orient;
  Direction right =
    (Direction)((orient + 1) % DIRECTION_COUNT);
  Direction left =
    (Direction)((orient + 3) % DIRECTION_COUNT);

  if (API_wallFront()) set_wall(m, pos, front);
  if (API_wallRight()) set_wall(m, pos, right);
  if (API_wallLeft())  set_wall(m, pos, left);
}

void set_wall(Maze *m, Point p, Direction dir) {
  if (!is_within_bounds(p))
    return;

  m->walls[p.x][p.y][dir] = true;

  Point neighbor = {p.x + direction_delta[dir].x,
                    p.y + direction_delta[dir].y};
  if (is_within_bounds(neighbor)) {
    Direction opp = get_opposite_direction(dir);
    m->walls[neighbor.x][neighbor.y][opp] = true;
  }
}

bool has_wall(const Maze *m, Point p, Direction dir) {
  if (!is_within_bounds(p))
    return true;
  return m->walls[p.x][p.y][dir];
}

// --- Flood Fill Algorithm ---

void flood_fill(Maze *m, Point target) {
  Point queue[MAX_CELLS];
  int   q_head = 0;
  int   q_tail = 0;

  for (int x = 0; x < MAZE_WIDTH; x++)
    for (int y = 0; y < MAZE_HEIGHT; y++)
      m->distances[x][y] = INVALID_DISTANCE;

  if (is_within_bounds(target)) {
    m->distances[target.x][target.y] = 0;
    queue[q_tail++] = target;
  } else {
    LOG_ERROR("flood fill target out of bounds");
    return;
  }

  while (q_head < q_tail) {
    Point cur = queue[q_head++];
    int   d = m->distances[cur.x][cur.y];

    for (Direction dir = 0; dir < DIRECTION_COUNT; dir++) {
      if (has_wall(m, cur, dir))
        continue;

      Point nb = {cur.x + direction_delta[dir].x,
                  cur.y + direction_delta[dir].y};

      if (is_within_bounds(nb) &&
          m->distances[nb.x][nb.y] > d + 1) {
        m->distances[nb.x][nb.y] = d + 1;
        queue[q_tail++] = nb;
      }
    }
  }
}

void flood_fill_goal(Maze *m) {
  Point queue[MAX_CELLS];
  int   q_head = 0;
  int   q_tail = 0;

  for (int x = 0; x < MAZE_WIDTH; x++)
    for (int y = 0; y < MAZE_HEIGHT; y++)
      m->distances[x][y] = INVALID_DISTANCE;

  // seed all 4 goal cells
  for (int x = GOAL_X1; x <= GOAL_X2; ++x) {
    for (int y = GOAL_Y1; y <= GOAL_Y2; ++y) {
      Point g = {x, y};
      if (is_within_bounds(g)) {
        m->distances[g.x][g.y] = 0;
        queue[q_tail++] = g;
      }
    }
  }

  if (q_tail == 0) {
    LOG_ERROR("no valid goal cells for flood fill");
    return;
  }

  while (q_head < q_tail) {
    Point cur = queue[q_head++];
    int   d = m->distances[cur.x][cur.y];

    for (Direction dir = 0; dir < DIRECTION_COUNT; dir++) {
      if (has_wall(m, cur, dir))
        continue;

      Point nb = {cur.x + direction_delta[dir].x,
                  cur.y + direction_delta[dir].y};

      if (is_within_bounds(nb) &&
          m->distances[nb.x][nb.y] > d + 1) {
        m->distances[nb.x][nb.y] = d + 1;
        queue[q_tail++] = nb;
      }
    }
  }
}

void flood_fill_start(Maze *m) {
  flood_fill(m, (Point){0, 0});
}

// --- Movement Logic ---

Direction choose_next_direction(const MouseState *ms,
                                const Maze *m) {
  Point     pos = ms->pos;
  int       min_dist = INVALID_DISTANCE + 10;
  Direction best_dir = NORTH;
  bool      found = false;

  for (Direction dir = 0; dir < DIRECTION_COUNT; dir++) {
    if (has_wall(m, pos, dir))
      continue;

    Point nb = {pos.x + direction_delta[dir].x,
                pos.y + direction_delta[dir].y};
    if (!is_within_bounds(nb))
      continue;

    int dist = m->distances[nb.x][nb.y];

    // exploration bonus: prefer unvisited in search mode
    if (ms->mode == SEARCH_MODE &&
        !m->visited[nb.x][nb.y]) {
      dist -= 1;
    }

    if (dist < min_dist) {
      min_dist = dist;
      best_dir = dir;
      found = true;
    }
  }

  if (!found) {
    LOG_ERROR("no valid move found - stuck at (%d,%d)",
              pos.x, pos.y);
    best_dir = get_opposite_direction(ms->orientation);
  }

  return best_dir;
}

void turn_to_direction(MouseState *ms, Direction target) {
  if (ms->orientation == target)
    return;

  int diff =
    (target - ms->orientation + DIRECTION_COUNT)
    % DIRECTION_COUNT;

  if (diff == 1) {
    API_turnRight();
    ms->orientation =
      (Direction)((ms->orientation + 1) % DIRECTION_COUNT);
  } else if (diff == 3) {
    API_turnLeft();
    ms->orientation =
      (Direction)((ms->orientation + 3) % DIRECTION_COUNT);
  } else {
    API_turnRight();
    ms->orientation =
      (Direction)((ms->orientation + 1) % DIRECTION_COUNT);
    API_turnRight();
    ms->orientation =
      (Direction)((ms->orientation + 1) % DIRECTION_COUNT);
  }
}

void move_forward_update_state(MouseState *ms, Maze *m) {
  Direction next_dir = choose_next_direction(ms, m);
  turn_to_direction(ms, next_dir);

  if (API_moveForward()) {
    ms->pos.x += direction_delta[ms->orientation].x;
    ms->pos.y += direction_delta[ms->orientation].y;
    LOG_STEP(++g_step, ms->pos.x, ms->pos.y,
             DIR_CHAR[ms->orientation]);
  } else {
    LOG_ERROR("move failed - wall at (%d,%d) %c",
              ms->pos.x, ms->pos.y,
              DIR_CHAR[ms->orientation]);
    set_wall(m, ms->pos, ms->orientation);

    if (ms->mode == SEARCH_MODE) {
      flood_fill_goal(m);
    } else if (ms->mode == RETURN_MODE) {
      flood_fill_start(m);
    }
  }
}

// --- Pathfinding ---

void compute_shortest_path(MouseState *ms, Maze *m) {
  flood_fill_goal(m);

  Point current = {0, 0};
  ms->path_length = 0;

  if (m->distances[0][0] == INVALID_DISTANCE) {
    LOG_ERROR("start unreachable from goal");
    ms->path_length = 0;
    return;
  }

  ms->shortest_path[ms->path_length++] = current;

  while (!is_at_goal(current)) {
    int       min_d = m->distances[current.x][current.y];
    Direction best = NORTH;
    bool      found = false;

    for (Direction dir = 0; dir < DIRECTION_COUNT; dir++) {
      if (has_wall(m, current, dir))
        continue;

      Point nb = {current.x + direction_delta[dir].x,
                  current.y + direction_delta[dir].y};

      if (is_within_bounds(nb) &&
          m->distances[nb.x][nb.y] < min_d) {
        min_d = m->distances[nb.x][nb.y];
        best = dir;
        found = true;
      }
    }

    if (!found) {
      LOG_ERROR("path broken at (%d,%d)",
                current.x, current.y);
      ms->path_length = 0;
      return;
    }

    current.x += direction_delta[best].x;
    current.y += direction_delta[best].y;

    if (ms->path_length < MAX_CELLS) {
      ms->shortest_path[ms->path_length++] = current;
    } else {
      LOG_ERROR("path exceeds maximum length");
      ms->path_length = 0;
      return;
    }
  }

  LOG("computing shortest path... %d steps",
      ms->path_length - 1);
}

// --- Path Verification ---

bool verify_path_exploration(const MouseState *ms,
                             const Maze *m) {
  if (ms->path_length <= 1) {
    LOG_ERROR("path too short or invalid for verification");
    return false;
  }

  for (int i = 0; i < ms->path_length; ++i) {
    Point p = ms->shortest_path[i];

    if (!is_within_bounds(p)) {
      LOG_ERROR("path point %d (%d,%d) out of bounds",
                i, p.x, p.y);
      return false;
    }

    if (!m->visited[p.x][p.y]) {
      LOG("verifying path: FAILED at (%d,%d) - "
          "not visited", p.x, p.y);
      return false;
    }

    // verify transition between consecutive points
    if (i > 0) {
      Point     prev = ms->shortest_path[i - 1];
      Direction move_dir = NORTH;
      bool      dir_found = false;
      for (Direction d = 0; d < DIRECTION_COUNT; ++d) {
        if (prev.x + direction_delta[d].x == p.x &&
            prev.y + direction_delta[d].y == p.y) {
          move_dir = d;
          dir_found = true;
          break;
        }
      }
      if (!dir_found || has_wall(m, prev, move_dir)) {
        LOG("verifying path: FAILED at (%d,%d)->(%d,%d)"
            " - walled segment",
            prev.x, prev.y, p.x, p.y);
        return false;
      }
    }
  }

  LOG("verifying path: PASSED");
  return true;
}

// --- Speed Run ---

void follow_shortest_path(MouseState *ms, Maze *m) {
  if (!is_at_start(ms->pos)) {
    LOG_ERROR("speed run aborted - not at start");
    return;
  }

  turn_to_direction(ms, NORTH);

  if (ms->path_length <= 1) {
    LOG_ERROR("no valid path for speed run");
    return;
  }

  int start_step = g_step + 1;
  LOG("#%d..%d following %d-step path",
      start_step, start_step + ms->path_length - 2,
      ms->path_length - 1);

  for (int i = 1; i < ms->path_length; i++) {
    Point target_pos = ms->shortest_path[i];

    // determine direction from current to target
    Direction move_dir = NORTH;
    bool      dir_found = false;
    for (Direction dir = 0; dir < DIRECTION_COUNT; ++dir) {
      if (ms->pos.x + direction_delta[dir].x ==
              target_pos.x &&
          ms->pos.y + direction_delta[dir].y ==
              target_pos.y) {
        move_dir = dir;
        dir_found = true;
        break;
      }
    }

    if (!dir_found) {
      LOG_ERROR("speed run path invalid: "
                "(%d,%d)->(%d,%d) no direction",
                ms->pos.x, ms->pos.y,
                target_pos.x, target_pos.y);
      return;
    }

    turn_to_direction(ms, move_dir);

    if (API_moveForward()) {
      ms->pos = target_pos;
      ++g_step;
      update_display(ms, m);
    } else {
      LOG_ERROR("speed run wall at (%d,%d) %c",
                ms->pos.x, ms->pos.y,
                DIR_CHAR[ms->orientation]);
      set_wall(m, ms->pos, ms->orientation);
      update_display(ms, m);
      return;
    }
  }

  if (is_at_goal(ms->pos)) {
    LOG("speed run complete - goal reached");
    API_setColor(ms->pos.x, ms->pos.y, 'G');
  } else {
    LOG_ERROR("speed run ended at (%d,%d) - not goal",
              ms->pos.x, ms->pos.y);
  }
}

// --- Display ---

void update_display(const MouseState *ms, const Maze *m) {
  for (int x = 0; x < MAZE_WIDTH; x++) {
    for (int y = 0; y < MAZE_HEIGHT; y++) {
      Point p = {x, y};

      // set distance text
      if (m->distances[x][y] == INVALID_DISTANCE) {
        API_setText(x, y, "-");
      } else {
        char buf[8];
        snprintf(buf, sizeof(buf), "%d",
                 m->distances[x][y]);
        API_setText(x, y, buf);
      }

      // set cell color
      if (p.x == ms->pos.x && p.y == ms->pos.y) {
        API_setColor(x, y, 'R');
      } else if (is_at_goal(p)) {
        API_setColor(x, y, 'G');
      } else if (m->visited[x][y]) {
        API_setColor(x, y, 'B');
      } else {
        API_setColor(x, y, 'Y');
      }

      // highlight shortest path in speed mode
      if (ms->mode == SPEED_MODE &&
          ms->path_length > 0) {
        bool on_path = false;
        for (int i = 0; i < ms->path_length; ++i) {
          if (ms->shortest_path[i].x == x &&
              ms->shortest_path[i].y == y) {
            on_path = true;
            break;
          }
        }
        if (on_path &&
            !(p.x == ms->pos.x &&
              p.y == ms->pos.y) &&
            !is_at_goal(p)) {
          API_setColor(x, y, 'C');
        }
      }

      // draw known walls
      if (m->walls[x][y][NORTH])
        API_setWall(x, y, 'n');
      if (m->walls[x][y][EAST])
        API_setWall(x, y, 'e');
      if (m->walls[x][y][SOUTH])
        API_setWall(x, y, 's');
      if (m->walls[x][y][WEST])
        API_setWall(x, y, 'w');
    }
  }
}

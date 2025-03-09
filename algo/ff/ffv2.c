#include "api.h"
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#define MAZE_WIDTH 16
#define MAZE_HEIGHT 16

// goal cells in 0-indexed coordinates (center 4 cells)
#define GOAL_X1 7
#define GOAL_Y1 7
#define GOAL_X2 8
#define GOAL_Y2 8

// directions
#define NORTH 0
#define EAST 1
#define SOUTH 2
#define WEST 3

// run modes
#define SEARCH_MODE 0
#define RETURN_MODE 1
#define SPEED_MODE 2

// current position and orientation
int posX = 0;
int posY = 0;
int direction = NORTH;
int currentMode = SEARCH_MODE;
bool goalFound = false;

// maze state
int distances[MAZE_WIDTH][MAZE_HEIGHT]; // distance values
bool walls[MAZE_WIDTH][MAZE_HEIGHT][4]; // wall information: [x][y][direction]
bool visited[MAZE_WIDTH][MAZE_HEIGHT];  // visited cells
int fastestPath[MAZE_WIDTH * MAZE_HEIGHT]
               [2]; // stores the shortest path coordinates
int pathLength = 0;

// direction deltas
const int dx[4] = {0, 1, 0, -1}; // NORTH, EAST, SOUTH, WEST
const int dy[4] = {1, 0, -1, 0};

// ===== function prototypes =====
void initializeMaze(void);
void updateWalls(void);
void floodFill(int targetX, int targetY);
void floodFillToGoal(void);
void floodFillToStart(void);
bool isAtGoal(void);
bool isAtStart(void);
void moveToNextCell(void);
void updateDisplay(void);
void logMessage(const char *msg);
void computeShortestPath(void);
void verifyShortestPath(void);
void followShortestPath(void);
void recomputePathIfNeeded(void);

int main(int argc, char *argv[]) {
  logMessage("Starting maze solver");

  // initialize maze data
  initializeMaze();

  // main navigation loop
  while (1) {

    if (API_wasReset()) {
      logMessage("Simulator reset detected!");
      API_ackReset();

      // Reset our state
      posX = 0;
      posY = 0;
      direction = NORTH;
      currentMode = SEARCH_MODE;
      goalFound = false;
      pathLength = 0;

      // Reinitialize the maze
      initializeMaze();
    }
    // update our knowledge of the maze
    updateWalls();

    // mark current cell as visited
    visited[posX][posY] = true;

    // update display with current state
    updateDisplay();

    switch (currentMode) {
    case SEARCH_MODE:
      if (isAtGoal()) {
        logMessage("=== Goal reached! Switching to return mode ===");
        goalFound = true;
        currentMode = RETURN_MODE;
        floodFillToStart(); // prepare for journey back to start
      } else {
        floodFillToGoal(); // calculate distances to goal
        moveToNextCell();  // move toward goal
      }
      break;

    case RETURN_MODE:
      if (isAtStart()) {
        logMessage("=== Back at start! Computing shortest path ===");
        updateWalls();
        if (posX != 0 || posY != 0) {
          logMessage(
              "ERROR: Expected to be at (0,0) but coordinates don't match!");
        }
        computeShortestPath();
        verifyShortestPath();
        recomputePathIfNeeded();
        currentMode = SPEED_MODE;
      } else {
        floodFillToStart(); // calculate distances to start
        moveToNextCell();   // move toward start
      }
      break;

    case SPEED_MODE:
      logMessage("=== Beginning speed run ===");
      followShortestPath();
      logMessage("=== Speed run complete! ===");
      return 0; // end program after speed run
    }

    // simple debug message to track progress
    char buffer[80];
    sprintf(buffer, "now at (%d,%d) facing %d, mode: %d", posX, posY, direction,
            currentMode);
    logMessage(buffer);
  }

  return 0;
}

void initializeMaze(void) {
  // initialize all arrays
  for (int x = 0; x < MAZE_WIDTH; x++) {
    for (int y = 0; y < MAZE_HEIGHT; y++) {
      // set initial distances (we'll recalculate with floodFill)
      distances[x][y] = MAZE_WIDTH * MAZE_HEIGHT; // effectively infinity

      // clear visited flags and walls
      visited[x][y] = false;
      for (int dir = 0; dir < 4; dir++) {
        walls[x][y][dir] = false;
      }
    }
  }

  // set boundary walls for the outer cells
  for (int x = 0; x < MAZE_WIDTH; x++) {
    walls[x][0][SOUTH] = true;               // bottom edge
    walls[x][MAZE_HEIGHT - 1][NORTH] = true; // top edge
  }
  for (int y = 0; y < MAZE_HEIGHT; y++) {
    walls[0][y][WEST] = true;              // left edge
    walls[MAZE_WIDTH - 1][y][EAST] = true; // right edge
  }

  // initialize first cell as visited
  visited[0][0] = true;

  // initial floodFill to goal
  floodFillToGoal();
}

void updateWalls(void) {
  // detect walls from api and update our wall map
  bool frontWall = API_wallFront();
  bool rightWall = API_wallRight();
  bool leftWall = API_wallLeft();

  // Log detected walls for debugging
  char buffer[80];
  sprintf(buffer, "Detecting walls at (%d,%d): front=%d, right=%d, left=%d",
          posX, posY, frontWall, rightWall, leftWall);
  logMessage(buffer);

  // update walls for current cell
  if (frontWall) {
    walls[posX][posY][direction] = true;

    // update the neighboring cell's wall too
    int nx = posX + dx[direction];
    int ny = posY + dy[direction];
    if (nx >= 0 && nx < MAZE_WIDTH && ny >= 0 && ny < MAZE_HEIGHT) {
      walls[nx][ny][(direction + 2) % 4] = true;
    }
  } else {
    // Explicitly mark that there is no wall
    walls[posX][posY][direction] = false;

    int nx = posX + dx[direction];
    int ny = posY + dy[direction];
    if (nx >= 0 && nx < MAZE_WIDTH && ny >= 0 && ny < MAZE_HEIGHT) {
      walls[nx][ny][(direction + 2) % 4] = false;
    }
  }

  if (rightWall) {
    int rightDir = (direction + 1) % 4;
    walls[posX][posY][rightDir] = true;

    int nx = posX + dx[rightDir];
    int ny = posY + dy[rightDir];
    if (nx >= 0 && nx < MAZE_WIDTH && ny >= 0 && ny < MAZE_HEIGHT) {
      walls[nx][ny][(rightDir + 2) % 4] = true;
    }
  } else {
    // Explicitly mark that there is no wall
    int rightDir = (direction + 1) % 4;
    walls[posX][posY][rightDir] = false;

    int nx = posX + dx[rightDir];
    int ny = posY + dy[rightDir];
    if (nx >= 0 && nx < MAZE_WIDTH && ny >= 0 && ny < MAZE_HEIGHT) {
      walls[nx][ny][(rightDir + 2) % 4] = false;
    }
  }

  if (leftWall) {
    int leftDir = (direction + 3) % 4;
    walls[posX][posY][leftDir] = true;

    int nx = posX + dx[leftDir];
    int ny = posY + dy[leftDir];
    if (nx >= 0 && nx < MAZE_WIDTH && ny >= 0 && ny < MAZE_HEIGHT) {
      walls[nx][ny][(leftDir + 2) % 4] = true;
    }
  } else {
    // Explicitly mark that there is no wall
    int leftDir = (direction + 3) % 4;
    walls[posX][posY][leftDir] = false;

    int nx = posX + dx[leftDir];
    int ny = posY + dy[leftDir];
    if (nx >= 0 && nx < MAZE_WIDTH && ny >= 0 && ny < MAZE_HEIGHT) {
      walls[nx][ny][(leftDir + 2) % 4] = false;
    }
  }
}

// Generalized flood fill to any target
void floodFill(int targetX, int targetY) {
  // create queue for BFS
  typedef struct {
    int x, y;
  } Cell;

  Cell queue[MAZE_WIDTH * MAZE_HEIGHT];
  int qFront = 0;
  int qBack = 0;

  // reset distances to a high value
  for (int x = 0; x < MAZE_WIDTH; x++) {
    for (int y = 0; y < MAZE_HEIGHT; y++) {
      distances[x][y] = MAZE_WIDTH * MAZE_HEIGHT; // effectively infinity
    }
  }

  // start with target
  distances[targetX][targetY] = 0;
  queue[qBack].x = targetX;
  queue[qBack].y = targetY;
  qBack++;

  // BFS to calculate distances
  while (qFront < qBack) {
    Cell current = queue[qFront++];
    int x = current.x;
    int y = current.y;
    int d = distances[x][y];

    // try all four neighboring cells
    for (int dir = 0; dir < 4; dir++) {
      // skip if there's a wall
      if (walls[x][y][dir])
        continue;

      int nx = x + dx[dir];
      int ny = y + dy[dir];

      // check bounds
      if (nx < 0 || nx >= MAZE_WIDTH || ny < 0 || ny >= MAZE_HEIGHT)
        continue;

      // update neighbor's distance if it's better
      if (distances[nx][ny] > d + 1) {
        distances[nx][ny] = d + 1;
        queue[qBack].x = nx;
        queue[qBack].y = ny;
        qBack++;
      }
    }
  }
}

// Flood fill with goal as the target
void floodFillToGoal(void) {
  // Use the closest goal cell as the target
  int goalDistances[4];
  goalDistances[0] = abs(posX - GOAL_X1) + abs(posY - GOAL_Y1);
  goalDistances[1] = abs(posX - GOAL_X1) + abs(posY - GOAL_Y2);
  goalDistances[2] = abs(posX - GOAL_X2) + abs(posY - GOAL_Y1);
  goalDistances[3] = abs(posX - GOAL_X2) + abs(posY - GOAL_Y2);

  int minDist = goalDistances[0];
  int minIdx = 0;
  for (int i = 1; i < 4; i++) {
    if (goalDistances[i] < minDist) {
      minDist = goalDistances[i];
      minIdx = i;
    }
  }

  int targetX, targetY;
  switch (minIdx) {
  case 0:
    targetX = GOAL_X1;
    targetY = GOAL_Y1;
    break;
  case 1:
    targetX = GOAL_X1;
    targetY = GOAL_Y2;
    break;
  case 2:
    targetX = GOAL_X2;
    targetY = GOAL_Y1;
    break;
  case 3:
    targetX = GOAL_X2;
    targetY = GOAL_Y2;
    break;
  }

  floodFill(targetX, targetY);
}

// Flood fill with start as the target
void floodFillToStart(void) {
  floodFill(0, 0); // start position is (0,0)
}

bool isAtGoal(void) {
  return (posX == GOAL_X1 || posX == GOAL_X2) &&
         (posY == GOAL_Y1 || posY == GOAL_Y2);
}

bool isAtStart(void) { return posX == 0 && posY == 0; }

void moveToNextCell(void) {
  // find the direction with the minimum distance
  int minDir = -1;
  int minDist = MAZE_WIDTH * MAZE_HEIGHT;

  // exploration bonus - prefer unvisited cells
  int bonus[4] = {0, 0, 0, 0};

  // During search mode, give a small preference to unexplored cells
  if (currentMode == SEARCH_MODE) {
    for (int dir = 0; dir < 4; dir++) {
      if (walls[posX][posY][dir])
        continue;

      int nx = posX + dx[dir];
      int ny = posY + dy[dir];

      if (nx < 0 || nx >= MAZE_WIDTH || ny < 0 || ny >= MAZE_HEIGHT)
        continue;

      if (!visited[nx][ny]) {
        bonus[dir] = 1; // Small bonus for unvisited cells
      }
    }
  }

  for (int dir = 0; dir < 4; dir++) {
    // skip directions with walls
    if (walls[posX][posY][dir])
      continue;

    int nx = posX + dx[dir];
    int ny = posY + dy[dir];

    // check bounds
    if (nx < 0 || nx >= MAZE_WIDTH || ny < 0 || ny >= MAZE_HEIGHT)
      continue;

    // Adjust the distance with the exploration bonus
    int adjustedDist = distances[nx][ny] - bonus[dir];

    // check if this is a better direction
    if (adjustedDist < minDist) {
      minDist = adjustedDist;
      minDir = dir;
    }
  }

  // if no valid direction, something's wrong
  if (minDir == -1) {
    logMessage("=== ERROR: no valid direction found! ===");
    return;
  }

  // turn to face the best direction
  while (direction != minDir) {
    // find the shortest way to turn
    int diff = (minDir - direction + 4) % 4;

    if (diff == 1) {
      API_turnRight();
      direction = (direction + 1) % 4;
    } else if (diff == 3) {
      API_turnLeft();
      direction = (direction + 3) % 4;
    } else {
      // turn around
      API_turnRight();
      direction = (direction + 1) % 4;
      API_turnRight();
      direction = (direction + 1) % 4;
    }
  }

  // move forward
  if (API_moveForward()) {
    // update position if move was successful
    posX += dx[direction];
    posY += dy[direction];
  } else {
    // if we couldn't move, there's a wall we didn't know about
    walls[posX][posY][direction] = true;

    // update the neighboring cell's wall
    int nx = posX + dx[direction];
    int ny = posY + dy[direction];
    if (nx >= 0 && nx < MAZE_WIDTH && ny >= 0 && ny < MAZE_HEIGHT) {
      walls[nx][ny][(direction + 2) % 4] = true;
    }

    // re-run flood fill after discovering a new wall
    if (currentMode == SEARCH_MODE) {
      floodFillToGoal();
    } else if (currentMode == RETURN_MODE) {
      floodFillToStart();
    }
  }
}

void computeShortestPath(void) {
  logMessage("Computing shortest path from start to goal");

  // Start at the beginning
  int x = 0, y = 0;

  // Store the start as first point
  fastestPath[0][0] = x;
  fastestPath[0][1] = y;
  pathLength = 1;

  // Calculate distances from goal to start
  floodFillToGoal();

  char buffer[50];
  // Follow the shortest path by always picking the neighbor
  // with the lowest distance value
  while (!((x == GOAL_X1 || x == GOAL_X2) && (y == GOAL_Y1 || y == GOAL_Y2))) {
    int minDist = MAZE_WIDTH * MAZE_HEIGHT;
    int nextX = -1, nextY = -1;

    // Check all 4 directions
    for (int dir = 0; dir < 4; dir++) {
      // Skip if there's a wall
      if (walls[x][y][dir])
        continue;

      int nx = x + dx[dir];
      int ny = y + dy[dir];

      // Check bounds
      if (nx < 0 || nx >= MAZE_WIDTH || ny < 0 || ny >= MAZE_HEIGHT)
        continue;

      // Find the neighbor with the lowest distance
      if (distances[nx][ny] < minDist) {
        minDist = distances[nx][ny];
        nextX = nx;
        nextY = ny;
      }
    }

    if (nextX == -1) {
      logMessage("ERROR: Failed to compute shortest path!");
      return;
    }

    // Add this cell to the path
    x = nextX;
    y = nextY;
    fastestPath[pathLength][0] = x;
    fastestPath[pathLength][1] = y;
    pathLength++;

    sprintf(buffer, "Path point %d: (%d,%d)", pathLength - 1, x, y);
    logMessage(buffer);
  }

  sprintf(buffer, "Shortest path length: %d steps", pathLength - 1);
  logMessage(buffer);
}

void verifyShortestPath() {
  logMessage("Verifying shortest path for consistency...");

  int x = 0, y = 0;
  for (int i = 1; i < pathLength; i++) {
    int targetX = fastestPath[i][0];
    int targetY = fastestPath[i][1];

    // Find the direction of movement
    int moveDir = -1;
    for (int dir = 0; dir < 4; dir++) {
      int nx = x + dx[dir];
      int ny = y + dy[dir];
      if (nx == targetX && ny == targetY) {
        moveDir = dir;
        break;
      }
    }

    if (moveDir == -1) {
      char buffer[80];
      sprintf(buffer,
              "ERROR: Path verification failed! Can't find direction from "
              "(%d,%d) to (%d,%d)",
              x, y, targetX, targetY);
      logMessage(buffer);
      return;
    }

    // Check if there's a wall in this direction
    if (walls[x][y][moveDir]) {
      char buffer[80];
      sprintf(
          buffer,
          "ERROR: Wall detected in path! Cannot move from (%d,%d) to (%d,%d)",
          x, y, targetX, targetY);
      logMessage(buffer);
      return;
    }

    // Move to next cell in path
    x = targetX;
    y = targetY;
  }

  logMessage("Path verification successful. No obstacles detected.");
}

void recomputePathIfNeeded(void) {
  // Re-detect walls at the current position
  updateWalls();

  // Check if any detected walls conflict with our path
  bool needsRecompute = false;

  if (pathLength > 1) {
    int nextX = fastestPath[1][0];
    int nextY = fastestPath[1][1];

    // Find direction to first move
    int moveDir = -1;
    for (int dir = 0; dir < 4; dir++) {
      int nx = posX + dx[dir];
      int ny = posY + dy[dir];
      if (nx == nextX && ny == nextY) {
        moveDir = dir;
        break;
      }
    }

    if (moveDir != -1 && walls[posX][posY][moveDir]) {
      logMessage("Path conflicted with detected walls - recomputing path");
      needsRecompute = true;
    }
  }

  if (needsRecompute) {
    // Rerun flood fill and compute a new path
    floodFillToGoal();
    computeShortestPath();
    verifyShortestPath();
  }
}

void followShortestPath(void) {
  // // Reset to starting position for the speed run
  // posX = 0;
  // posY = 0;
  // direction = NORTH;

  while (direction != NORTH) {
    API_turnRight();
    direction = (direction + 1) % 4;
  }

  updateWalls();

  char buffer[80];
  sprintf(buffer, "Real wall state at start: N=%d, E=%d, S=%d, W=%d",
          walls[0][0][NORTH], walls[0][0][EAST], walls[0][0][SOUTH],
          walls[0][0][WEST]);
  logMessage(buffer);

  // Log the start of the speed run
  logMessage("Starting speed run from (0,0)");

  // Follow the precomputed path
  for (int i = 1; i < pathLength; i++) {
    int targetX = fastestPath[i][0];
    int targetY = fastestPath[i][1];

    // Determine which direction to move
    int moveDir = -1;
    for (int dir = 0; dir < 4; dir++) {
      int nx = posX + dx[dir];
      int ny = posY + dy[dir];
      if (nx == targetX && ny == targetY) {
        moveDir = dir;
        break;
      }
    }

    if (moveDir == -1) {
      char buffer[80];
      sprintf(buffer,
              "ERROR: Invalid path point! Cannot find direction from (%d,%d) "
              "to (%d,%d)",
              posX, posY, targetX, targetY);
      logMessage(buffer);
      return;
    }

    // Debug the planned move
    char buffer[80];
    sprintf(buffer,
            "Moving from (%d,%d) to (%d,%d), need to face direction: %d, "
            "currently facing: %d",
            posX, posY, targetX, targetY, moveDir, direction);
    logMessage(buffer);

    // Turn to face the right direction
    while (direction != moveDir) {
      int diff = (moveDir - direction + 4) % 4;
      if (diff == 1) {
        logMessage("Turning right");
        API_turnRight();
        direction = (direction + 1) % 4;
      } else if (diff == 3) {
        logMessage("Turning left");
        API_turnLeft();
        direction = (direction + 3) % 4;
      } else {
        // Turn around
        logMessage("Turning around (two rights)");
        API_turnRight();
        direction = (direction + 1) % 4;
        API_turnRight();
        direction = (direction + 1) % 4;
      }
    }

    // Move forward
    logMessage("Moving forward");
    if (!API_moveForward()) {
      // Add more debug info
      sprintf(buffer,
              "ERROR: Failed to move during speed run! At (%d,%d), facing %d",
              posX, posY, direction);
      logMessage(buffer);

      // Check if there's a wall in our way
      logMessage("Checking for walls in front:");
      if (API_wallFront()) {
        logMessage("Wall detected in front - map data is inconsistent with "
                   "actual maze!");
      } else {
        logMessage("No wall in front - this is unexpected!");
      }
      return;
    }

    // Update position
    posX = targetX;
    posY = targetY;

    // Update display
    updateDisplay();

    // Debug message
    sprintf(buffer, "Speed run: now at (%d,%d) facing %d", posX, posY,
            direction);
    logMessage(buffer);
  }

  // Mark the goal as reached
  API_setColor(posX, posY, 'G');
  logMessage("=== Speed run complete! Goal reached! ===");
}

void updateDisplay(void) {
  // color the entire maze
  for (int x = 0; x < MAZE_WIDTH; x++) {
    for (int y = 0; y < MAZE_HEIGHT; y++) {
      // set the text to show distance
      char buffer[8];
      sprintf(buffer, "%d", distances[x][y]);
      API_setText(x, y, buffer);

      // set cell color
      if (x == posX && y == posY) {
        API_setColor(x, y, 'r'); // current cell: red
      } else if ((x == GOAL_X1 || x == GOAL_X2) &&
                 (y == GOAL_Y1 || y == GOAL_Y2)) {
        API_setColor(x, y, 'G'); // goal cells: green
      } else if (visited[x][y]) {
        API_setColor(x, y, 'B'); // visited cells: blue
      } else {
        API_setColor(x, y, 'Y'); // unvisited cells: yellow
      }

      // Highlight the shortest path in speed mode
      if (currentMode == SPEED_MODE) {
        for (int i = 0; i < pathLength; i++) {
          if (x == fastestPath[i][0] && y == fastestPath[i][1]) {
            API_setColor(x, y, 'C'); // path: cyan
            break;
          }
        }
      }
    }
  }

  // draw walls
  for (int x = 0; x < MAZE_WIDTH; x++) {
    for (int y = 0; y < MAZE_HEIGHT; y++) {
      // mark walls on the api map
      if (walls[x][y][NORTH]) {
        API_setWall(x, y, 'n');
      }
      if (walls[x][y][EAST]) {
        API_setWall(x, y, 'e');
      }
      if (walls[x][y][SOUTH]) {
        API_setWall(x, y, 's');
      }
      if (walls[x][y][WEST]) {
        API_setWall(x, y, 'w');
      }
    }
  }
}

void logMessage(const char *msg) {
  fprintf(stderr, "%s\n", msg);
  fflush(stderr);
}

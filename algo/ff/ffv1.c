#include "api.h"
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

// TODO: Search Run

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

// current position and orientation
int posX = 0;
int posY = 0;
int direction = NORTH;

// maze state
int distances[MAZE_WIDTH][MAZE_HEIGHT]; // distance values
bool walls[MAZE_WIDTH][MAZE_HEIGHT][4]; // wall information: [x][y][direction]
bool visited[MAZE_WIDTH][MAZE_HEIGHT];  // visited cells

// direction deltas
const int dx[4] = {0, 1, 0, -1}; // NORTH, EAST, SOUTH, WEST
const int dy[4] = {1, 0, -1, 0};

// ===== function prototypes =====
void initializeMaze(void);
void updateWalls(void);
void floodFill(void);
bool isAtGoal(void);
void moveToNextCell(void);
void updateDisplay(void);
void logMessage(const char *msg);

int main(int argc, char *argv[]) {
  logMessage("Starting maze solver");

  // initialize maze data
  initializeMaze();

  // main navigation loop
  while (1) {
    // update our knowledge of the maze
    updateWalls();

    // mark current cell as visited
    visited[posX][posY] = true;

    // update display
    updateDisplay();

    // check if we've reached the goal
    if (isAtGoal()) {
      logMessage("=== goal reached! ===");
      API_setColor(posX, posY, 'G');
      break;
    }

    // update distances
    floodFill();

    // move to the next cell
    moveToNextCell();

    // simple debug message to track progress
    char buffer[50];
    sprintf(buffer, "now at (%d,%d) facing %d", posX, posY, direction);
    logMessage(buffer);
  }

  return 0;
}

void initializeMaze(void) {
  // initialize all arrays
  for (int x = 0; x < MAZE_WIDTH; x++) {
    for (int y = 0; y < MAZE_HEIGHT; y++) {
      // set initial distances
      if ((x == GOAL_X1 || x == GOAL_X2) && (y == GOAL_Y1 || y == GOAL_Y2)) {
        distances[x][y] = 0; // goal cells
      } else {
        // manhattan distance to the nearest goal cell
        int d1 = abs(x - GOAL_X1) + abs(y - GOAL_Y1);
        int d2 = abs(x - GOAL_X1) + abs(y - GOAL_Y2);
        int d3 = abs(x - GOAL_X2) + abs(y - GOAL_Y1);
        int d4 = abs(x - GOAL_X2) + abs(y - GOAL_Y2);

        distances[x][y] = d1;
        if (d2 < distances[x][y])
          distances[x][y] = d2;
        if (d3 < distances[x][y])
          distances[x][y] = d3;
        if (d4 < distances[x][y])
          distances[x][y] = d4;
      }

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
}

void updateWalls(void) {
  // detect walls from api and update our wall map
  bool frontWall = API_wallFront();
  bool rightWall = API_wallRight();
  bool leftWall = API_wallLeft();

  // update walls for current cell
  if (frontWall) {
    walls[posX][posY][direction] = true;

    // update the neighboring cell's wall too
    int nx = posX + dx[direction];
    int ny = posY + dy[direction];
    if (nx >= 0 && nx < MAZE_WIDTH && ny >= 0 && ny < MAZE_HEIGHT) {
      walls[nx][ny][(direction + 2) % 4] = true;
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
  }

  if (leftWall) {
    int leftDir = (direction + 3) % 4;
    walls[posX][posY][leftDir] = true;

    int nx = posX + dx[leftDir];
    int ny = posY + dy[leftDir];
    if (nx >= 0 && nx < MAZE_WIDTH && ny >= 0 && ny < MAZE_HEIGHT) {
      walls[nx][ny][(leftDir + 2) % 4] = true;
    }
  }
}

void floodFill(void) {
  // create queue for bfs
  typedef struct {
    int x, y;
  } Cell;

  Cell queue[MAZE_WIDTH * MAZE_HEIGHT];
  int qFront = 0;
  int qBack = 0;

  // reset distances to a high value except for goal
  for (int x = 0; x < MAZE_WIDTH; x++) {
    for (int y = 0; y < MAZE_HEIGHT; y++) {
      if ((x == GOAL_X1 || x == GOAL_X2) && (y == GOAL_Y1 || y == GOAL_Y2)) {
        distances[x][y] = 0;
        // add goal cells to the queue
        queue[qBack].x = x;
        queue[qBack].y = y;
        qBack++;
      } else {
        distances[x][y] = MAZE_WIDTH * MAZE_HEIGHT; // effectively infinity
      }
    }
  }

  // bfs to calculate distances
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

bool isAtGoal(void) {
  return (posX == GOAL_X1 || posX == GOAL_X2) &&
         (posY == GOAL_Y1 || posY == GOAL_Y2);
}

void moveToNextCell(void) {
  // find the direction with the minimum distance
  int minDir = -1;
  int minDist = MAZE_WIDTH * MAZE_HEIGHT;

  for (int dir = 0; dir < 4; dir++) {
    // skip directions with walls
    if (walls[posX][posY][dir])
      continue;

    int nx = posX + dx[dir];
    int ny = posY + dy[dir];

    // check bounds
    if (nx < 0 || nx >= MAZE_WIDTH || ny < 0 || ny >= MAZE_HEIGHT)
      continue;

    // check if this is a better direction
    if (distances[nx][ny] < minDist) {
      minDist = distances[nx][ny];
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
      // turn around (2 rights is often more reliable than 2 lefts)
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
    floodFill();
  }
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

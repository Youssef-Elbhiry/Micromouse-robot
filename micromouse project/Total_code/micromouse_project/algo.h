#ifndef ALGORITHM_FUNCTIONS_H
#define ALGORITHM_FUNCTIONS_H

#define NORTH 0
#define SOUTH 1
#define EAST 2
#define WEST 3

#define dis 20.0

#define MAZE_SIZE 16
#define WALL_ARRAY_SIZE 256

// Global variables initializatio
int maze[MAZE_SIZE][MAZE_SIZE]; 
bool walls[WALL_ARRAY_SIZE][WALL_ARRAY_SIZE] = {false};

// Function prototypes
int cellNumber(int x, int y);
bool hasWall(int x1, int y1, int x2, int y2);
void addWall(int x1, int y1, int x2, int y2);
bool isValidCell(int x, int y);
void updateWalls(short robot_x, short robot_y, short direction);
void bfs(std::vector<std::pair<int, int>>& startPoints);
void findBestCell(short robot_x, short robot_y, short direction, short& best_x, short& best_y, int& min_value);
void moveToBestCell(short& robot_x, short& robot_y, short& direction);

#endif
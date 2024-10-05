#include <queue>
#include <iostream>
#include "algo.h"
#include "walls.h"
#include "Movement.h"



// Convert cell coordinates to a cell number
int cellNumber(int x, int y) {
    return y * MAZE_SIZE + x; // Assuming MAZE_SIZE is the width of the grid
}
/*
// Check if there is a wall between two adjacent cells
bool hasWall(int x1, int y1, int x2, int y2) {
    if (isValidCell(x1, y1) && isValidCell(x2, y2)) {
        if (x1 == x2 && abs(y1 - y2) == 1) {
            return walls[cellNumber(x1, y1)][cellNumber(x2, y2)];
        } else if (y1 == y2 && abs(x1 - x2) == 1) {
            return walls[cellNumber(x1, y1)][cellNumber(x2, y2)];
        }
    }
    return false;
}
*/
// Check if there is a wall between two adjacent cells
bool hasWall(int x1, int y1, int x2, int y2) {
    if (isValidCell(x1, y1) && isValidCell(x2, y2)) {
        if (x1 == x2 && abs(y1 - y2) == 1) {
            return walls[cellNumber(x1, y1)][cellNumber(x2, y2)];
        } else if (y1 == y2 && abs(x1 - x2) == 1) {
            return walls[cellNumber(x1, y1)][cellNumber(x2, y2)];
        }
		 else 
		 {
            return false;
        }
    }
    else {
        return false;
    }
}

// Add a wall between two cells
void addWall(int x1, int y1, int x2, int y2) {
    int cell1 = cellNumber(x1, y1);
    int cell2 = cellNumber(x2, y2);

    if (cell1 < 0 || cell1 >= WALL_ARRAY_SIZE || cell2 < 0 || cell2 >= WALL_ARRAY_SIZE) {
        return;
    }

    walls[cell1][cell2] = true;
    walls[cell2][cell1] = true; // Bidirectional wall
}

// Check if the cell is within valid bounds
bool isValidCell(int x, int y) {
    return (x >= 0 && x < MAZE_SIZE && y >= 0 && y < MAZE_SIZE);
}

// Update walls based on robot's surroundings
void updateWalls(short robot_x, short robot_y, short direction) {
    if (wallFront()) {
        if (direction == NORTH && isValidCell(robot_x - 1, robot_y)) {
            addWall(robot_x, robot_y, robot_x - 1, robot_y);
        } else if (direction == SOUTH && isValidCell(robot_x + 1, robot_y)) {
            addWall(robot_x, robot_y, robot_x + 1, robot_y);
        } else if (direction == EAST && isValidCell(robot_x, robot_y + 1)) {
            addWall(robot_x, robot_y, robot_x, robot_y + 1);
        } else if (direction == WEST && isValidCell(robot_x, robot_y - 1)) {
            addWall(robot_x, robot_y, robot_x, robot_y - 1);
        }
        Serial.println("added wall front ");
    }

    if (wallRight()) {
        if (direction == NORTH && isValidCell(robot_x, robot_y + 1)) {
            addWall(robot_x, robot_y, robot_x, robot_y + 1);
        } else if (direction == SOUTH && isValidCell(robot_x, robot_y - 1)) {
            addWall(robot_x, robot_y, robot_x, robot_y - 1);
        } else if (direction == EAST && isValidCell(robot_x + 1, robot_y)) {
            addWall(robot_x, robot_y, robot_x + 1, robot_y);
        } else if (direction == WEST && isValidCell(robot_x - 1, robot_y)) {
            addWall(robot_x, robot_y, robot_x - 1, robot_y);
        }
        Serial.println("added wall right ");

    }

    if (wallLeft()) {
        if (direction == NORTH && isValidCell(robot_x, robot_y - 1)) {
            addWall(robot_x, robot_y, robot_x, robot_y - 1);
        } else if (direction == SOUTH && isValidCell(robot_x, robot_y + 1)) {
            addWall(robot_x, robot_y, robot_x, robot_y + 1);
        } else if (direction == EAST && isValidCell(robot_x - 1, robot_y)) {
            addWall(robot_x, robot_y, robot_x - 1, robot_y);
                     Serial.println("added wall left");
        } else if (direction == WEST && isValidCell(robot_x + 1, robot_y)) {
            addWall(robot_x, robot_y, robot_x + 1, robot_y);
        }
    }
}

// Breadth-first search to populate the maze array
void bfs(std::vector<std::pair<int, int>>& startPoints) {
    std::queue<std::pair<int, int>> q;

    // Initialize the maze with a high value
    for (int i = 0; i < MAZE_SIZE; i++) {
        for (int j = 0; j < MAZE_SIZE; j++) {
            maze[i][j] = 9999; 
        }
    }

    // Start BFS from the initial points
    for (const auto& point : startPoints) {
        q.push(point);
        maze[point.first][point.second] = 0;
    }

    int dx[] = {0, 0, -1, 1};  // Directions: Left, Right
    int dy[] = {-1, 1, 0, 0};  // Directions: Up, Down

    while (!q.empty()) {
        std::pair<int, int> current = q.front();
        int x = current.first;
        int y = current.second;
        q.pop();

        for (int i = 0; i < 4; i++) {
            int nx = x + dx[i];
            int ny = y + dy[i];

            if (isValidCell(nx, ny) && !hasWall(x, y, nx, ny) && maze[nx][ny] > maze[x][y] + 1) {
                maze[nx][ny] = maze[x][y] + 1;
                q.push({nx, ny});
            }
        }
    }

}

// Find the best cell to move to based on current position
void findBestCell(short robot_x, short robot_y, short direction, short& best_x, short& best_y, int& min_value) {
    best_x = robot_x;
    best_y = robot_y;
    min_value = 9999;  // Initialize to a large value

    const int dx[] = {0, -1, 0, 1}; // Directions: Left, Right
    const int dy[] = {-1, 0, 1, 0}; // Directions: Up, Down
    // const short directions[] = {WEST, EAST, NORTH, SOUTH};
   // const short directions[] = {NORTH, SOUTH, EAST, WEST};


    for (int i = 0; i < 4; i++) {
        int nx = robot_x + dx[i];
        int ny = robot_y + dy[i];
     //   short move_direction = directions[i];

        if (isValidCell(nx, ny) && maze[nx][ny] < min_value) {
          if(robot_x==12&&robot_y==1)
          {Serial.println("check the wall!!");
            Serial.println(hasWall(robot_x, robot_y, nx, ny));
          }
            if (!hasWall(robot_x, robot_y, nx, ny)) {
               
               Serial.print("i=");
               Serial.print(i);
                min_value = maze[nx][ny];
                best_x = nx;
                best_y = ny;
            }
        } 
      
    }

}

// Move the robot to the best cell found
void moveToBestCell(short& robot_x, short& robot_y, short& direction) {
    short best_x, best_y;
    int min_value = 9999;

    findBestCell(robot_x, robot_y, direction, best_x, best_y, min_value);
    Serial.print("best_x");
    Serial.println(best_x);
        Serial.print("best_y");
        Serial.println(best_y);





    // Move robot based on the best cell's position
    if (best_x < robot_x) {
        // Move North
        if (direction == NORTH) {
            moveForward(dis);
            Serial.println("move forward directily");
        } else if (direction == WEST) {
            Serial.println("turn right then move forward ");

            turnRight();
            moveForward(dis);
        } else if (direction == SOUTH) {
            Serial.println("turn left twice  then move forward ");

            turnLeft();
            turnLeft();
           moveForward(dis);
        } else if (direction == EAST) {
             Serial.println("turn left one  then move forward ");

            turnLeft();
            moveForward(dis);
        }
        direction = NORTH;
        robot_x--;
    } else if (best_x > robot_x) {
        // Move South
        if (direction == NORTH) {
                      Serial.println("turn left twice  then move forward ");

            turnLeft();
            turnLeft();
            moveForward(dis);
        } else if (direction == WEST) {
            Serial.println("turn left one  then move forward ");
            turnLeft();
            moveForward(dis);
        } else if (direction == SOUTH) {
            Serial.println("move forward directily");

            moveForward(dis);
        } else if (direction == EAST) {
            Serial.println("turn right then move forward ");

            turnRight();
            moveForward(dis);
        }
        robot_x++;
        direction = SOUTH;
        Serial.println("move backward");
    } else if (best_y > robot_y) {
        // Move East
        if (direction == NORTH) {
            Serial.println("turn right then move forward ");
            turnRight();
            Serial.println("turn right DONE ");
            moveForward(dis);
        } else if (direction == WEST) {
          Serial.println("turn right twice  then move forward ");
            turnRight();
            turnRight();
            moveForward(dis);
        } else if (direction == SOUTH) {
          Serial.println("turn left   then move forward ");
            turnLeft();
            moveForward(dis);
        } else if (direction == EAST) {
        Serial.println(" move forward ");
            moveForward(dis);
        }
        robot_y++;
        direction = EAST;
    } else if (best_y < robot_y) {
        // Move West
        if (direction == NORTH) {
          Serial.println("turn left   then move forward ");
            turnLeft();
            moveForward(dis);
        } else if (direction == WEST) {
          Serial.println(" move forward ");
            moveForward(dis);
        } else if (direction == SOUTH) {
          Serial.println("turn riht   then move forward ");
            turnRight();
            moveForward(dis);
        } else if (direction == EAST) {
          Serial.println("turn riht twice  then move forward ");
            turnRight();
            turnRight();
            moveForward(dis);
        }
        robot_y--;
        direction=WEST;
}
}
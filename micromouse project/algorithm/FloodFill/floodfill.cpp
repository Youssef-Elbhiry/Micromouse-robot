#include <queue>
#include <iostream>
#include "floodfill.h"
#include "API.h"

// Define maze size
#define MAZE_SIZE 16
#define WALL_ARRAY_SIZE 256

void log(const std::string& text) {
    std::cerr << text << std::endl;
    std::cerr.flush();  // Flush the stream to ensure output is displayed immediately
}

// Initialize visited array
int visited[MAZE_SIZE][MAZE_SIZE] = {0};

// Initialize maze with values (example values, adjust as needed)
int maze[MAZE_SIZE][MAZE_SIZE];

bool walls[WALL_ARRAY_SIZE][WALL_ARRAY_SIZE] = {false};
std::vector<std::pair<int, int>> centerPoints = {
        {MAZE_SIZE / 2 - 1, MAZE_SIZE / 2 - 1},
        {MAZE_SIZE / 2 - 1, MAZE_SIZE / 2},
        {MAZE_SIZE / 2, MAZE_SIZE / 2 - 1},
        {MAZE_SIZE / 2, MAZE_SIZE / 2}
    };
// Convert cell coordinates to cell number
int cellNumber(int x, int y) {
    return y * MAZE_SIZE + x; // Assuming MAZE_SIZE is the width of the grid
}

// Check if there is a wall between two adjacent cells
bool hasWall(int x1, int y1, int x2, int y2) {
    // Ensure the cells are adjacent
    if (isValidCell(x1, y1) && isValidCell(x2, y2)) {
        if (x1 == x2 && abs(y1 - y2) == 1) {  // Same row, adjacent columns
            return walls[cellNumber(x1, y1)][cellNumber(x2, y2)];
        } else if (y1 == y2 && abs(x1 - x2) == 1) {  // Same column, adjacent rows
            return walls[cellNumber(x1, y1)][cellNumber(x2, y2)];
        } else {
            std::cerr << "Cells are not adjacent." << std::endl;
            return false;
        }
    } else {
        std::cerr << "Invalid cell coordinates." << std::endl;
        return false;
    }
}
// Add a wall between two cells
void addWall(int x1, int y1, int x2, int y2) {
    int cell1 = cellNumber(x1, y1);
    int cell2 = cellNumber(x2, y2);

    if (cell1 < 0 || cell1 >= WALL_ARRAY_SIZE || cell2 < 0 || cell2 >= WALL_ARRAY_SIZE) {
        std::cerr << "Invalid cell numbers." << std::endl;
        return;
    }

    walls[cell1][cell2] = true;
    walls[cell2][cell1] = true; // The wall between the two cells is bidirectional
}

// Check if a cell is within maze boundaries
bool isValidCell(int x, int y) {
    return (x >= 0 && x < MAZE_SIZE && y >= 0 && y < MAZE_SIZE);
}
void findBestCell(short robot_x, short robot_y, short direction,
                  short& best_x, short& best_y, int& min_value) {
    best_x = robot_x;
    best_y = robot_y;

    // Array to store directions and their corresponding movements
    const int dx[] = {0, 0, -1, 1}; // Left, Right
    const int dy[] = {-1, 1, 0, 0}; // Up, Down
    const short directions[] = {WEST, EAST, NORTH, SOUTH};

    // Loop through all possible moves
    for (int i = 0; i < 4; i++) {
        int nx = robot_x + dx[i];
        int ny = robot_y + dy[i];
        short move_direction = directions[i];

        // Check if the new cell is valid and has a lower value
        if (isValidCell(nx, ny) && maze[nx][ny] < min_value) {
            // Check if there's no wall blocking the move
            if (!hasWall(robot_x, robot_y, nx, ny)) {
                min_value = maze[nx][ny];
                best_x = nx;
                best_y = ny;
            }
        }
}
				  }
// خوارزمية BFS لحساب المسافات بناءً على موقع البداية
void bfs(std::vector<std::pair<int, int>>& startPoints) {
    std::queue<std::pair<int, int>> q;

    // تهيئة مصفوفة المسافات بقيم عالية (مثل قيمة لا نهائية)
    for (int i = 0; i < MAZE_SIZE; i++) {
        for (int j = 0; j < MAZE_SIZE; j++) {
            maze[i][j] = 9999; // قيمة أولية عالية
        }
    }
	for(int i =0 ; i<MAZE_SIZE;i++){
	for(int j =0 ; j<MAZE_SIZE;j++){
		API::setText(j, MAZE_SIZE - 1 - i, std::to_string(maze[i][j]));
	}
	}
	

    // بدء BFS من الخلايا المحددة
    for (const auto& point : startPoints) {
        q.push(point);
        maze[point.first][point.second] = 0;
    }

    // المتجهات للحركة في 4 اتجاهات
    int dx[] = {0, 0, -1, 1};  // Left, Right
    int dy[] = {-1, 1, 0, 0};  // Up, Down

    while (!q.empty()) {
        std::pair<int, int> current = q.front();
        int x = current.first;
        int y = current.second;
        q.pop();

        // فحص جميع الاتجاهات الأربعة
        for (int i = 0; i < 4; i++) {
            int nx = x + dx[i];
            int ny = y + dy[i];

            // التأكد من أن الخلية الجديدة صالحة ولا يوجد جدار
            if (isValidCell(nx, ny) && !hasWall(x, y, nx,ny) && maze[nx][ny] > maze[x][y] + 1) {
                maze[nx][ny] = maze[x][y] + 1;
                q.push({nx, ny});
            }
        }
    }
}


void moveToBestCell( short& robot_x, short& robot_y, short& direction) {
    short best_x, best_y;
    int min_value=9999;
    // العثور على أفضل خلية حول الروبوت
    findBestCell(robot_x, robot_y, direction, best_x, best_y, min_value);


    // التحرك بناءً على أفضل خلية تم العثور عليها
    if (best_x < robot_x) {
        // Move forward
        if (direction == NORTH) {
            API::moveForward();
        } else if (direction == WEST) {
            API::turnRight();
            API::moveForward();
        } else if (direction == SOUTH) {
            API::turnLeft();
            API::turnLeft();
            API::moveForward();
        } else if (direction == EAST) {
            API::turnLeft();
            API::moveForward();
        }
        direction = NORTH;
        robot_x--;
		log("moveForward");
    } else if (best_x > robot_x) {
        // Move backward
        if (direction == NORTH) {
            API::turnLeft();
            API::turnLeft();
            API::moveForward();
        } else if (direction == WEST) {
            API::turnLeft();
            API::moveForward();
        } else if (direction == SOUTH) {
            API::moveForward();
        } else if (direction == EAST) {
            API::turnRight();
            API::moveForward();
        }
        robot_x++;
        direction = SOUTH;
		log("backward");
    } else if (best_y > robot_y) {
        // Move right
        if (direction == NORTH) {
            API::turnRight();
            API::moveForward();
        } else if (direction == WEST) {
            API::turnRight();
            API::turnRight();
            API::moveForward();
        } else if (direction == SOUTH) {
            API::turnLeft();
            API::moveForward();
        } else if (direction == EAST) {
            API::moveForward();
        }
        robot_y++;
        direction = EAST;  
		log("moveright");
    } else if (best_y < robot_y) {
        // Move left
        if (direction == NORTH) {
            API::turnLeft();
            API::moveForward();
        } else if (direction == WEST) {
            API::moveForward();
        } else if (direction == SOUTH) {
            API::turnRight();
            API::moveForward();
        } else if (direction == EAST) {
            API::turnRight();
            API::turnRight();
            API::moveForward();
        }
        robot_y--;
        direction = WEST; 
		// Update direction after moving left
		log("moveleft");
    }
	else {
		 
	       bfs(centerPoints);

        	}

	// تحويل قيمة الخلية إلى نص
    std::string cellValue = std::to_string(maze[robot_x][robot_y]);
    
    // إظهار قيمة الخلية على الـ simulator
	API::setText(robot_y,MAZE_SIZE -1-robot_x ,cellValue);
	log("Position after move: (" + std::to_string(robot_x) + ", " + std::to_string(robot_y) + ") Value: " + std::to_string(maze[robot_x][robot_y]));

}
// تحديث الجدران بناءً على اتجاه الروبوت الحالي
void updateWalls(short robot_x, short robot_y, short direction) {
    // إضافة جدار أمام الروبوت بناءً على الاتجاه
    if (API::wallFront()) {
        if (direction == NORTH && isValidCell(robot_x - 1, robot_y)) {
            addWall(robot_x, robot_y, robot_x - 1, robot_y);
        } else if (direction == SOUTH && isValidCell(robot_x + 1, robot_y)) {
            addWall(robot_x, robot_y, robot_x + 1, robot_y);
        } else if (direction == EAST && isValidCell(robot_x, robot_y + 1)) {
            addWall(robot_x, robot_y, robot_x, robot_y + 1);
        } else if (direction == WEST && isValidCell(robot_x, robot_y - 1)) {
            addWall(robot_x, robot_y, robot_x, robot_y - 1);
        }
		log("wallFront");
    }


    // إضافة جدار على يمين الروبوت بناءً على الاتجاه
    if (API::wallRight()) {
        if (direction == NORTH && isValidCell(robot_x, robot_y + 1)) {
            addWall(robot_x, robot_y, robot_x, robot_y + 1);
        } else if (direction == SOUTH && isValidCell(robot_x, robot_y - 1)) {
            addWall(robot_x, robot_y, robot_x, robot_y - 1);
        } else if (direction == EAST && isValidCell(robot_x + 1, robot_y)) {
            addWall(robot_x, robot_y, robot_x + 1, robot_y);
        } else if (direction == WEST && isValidCell(robot_x - 1, robot_y)) {
            addWall(robot_x, robot_y, robot_x - 1, robot_y);
        }
		log("wallright");
    }

    // إضافة جدار على يسار الروبوت بناءً على الاتجاه
    if (API::wallLeft()) {
        if (direction == NORTH && isValidCell(robot_x, robot_y - 1)) {
            addWall(robot_x, robot_y, robot_x, robot_y - 1);
        } else if (direction == SOUTH && isValidCell(robot_x, robot_y + 1)) {
            addWall(robot_x, robot_y, robot_x, robot_y + 1);
        } else if (direction == EAST && isValidCell(robot_x - 1, robot_y)) {
            addWall(robot_x, robot_y, robot_x - 1, robot_y);
        } else if (direction == WEST && isValidCell(robot_x + 1, robot_y)) {
            addWall(robot_x, robot_y, robot_x + 1, robot_y);
        }
		log("wallleft");
    }
	
}

int main() {
    short direction = NORTH;
    short robot_x = 15; // Starting X coordinate
    short robot_y = 0;  // Starting Y coordinate


	       bfs(centerPoints);
    while ( maze[robot_x][robot_y]!=0) {
        // Update walls based on the current robot position
         updateWalls(robot_x, robot_y, direction);
		 
		   bfs(centerPoints);
        // Move robot to the best cell
        moveToBestCell( robot_x, robot_y, direction);
	
}
}
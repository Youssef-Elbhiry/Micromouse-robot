#ifndef FLOODFILL_H
#define FLOODFILL_H

#define NORTH 0
#define SOUTH 1
#define EAST 2
#define WEST 3

#include <vector>
#include <queue>
#include <stack>

// تحديث إحداثيات الروبوت بناءً على الاتجاه والجدار
void updateCoordinates(short direction, short wall, short x, short y, short &toX, short &toY);

// تحريك الروبوت من موقع إلى آخر
void move(short &direction, short x, short y, short toX, short toY);

// خوارزمية Flood Fill لحل المتاهة
void floodFill(short x, short y, short &direction, bool visited[][17], bool matrix[][257]);

// خوارزمية BFS (البحث بالعرض) لتحديد المسار الأقصر
void BFS(short source, short target, short direction, bool matrix[][257]);
bool hasWall(int x1, int y1, int x2, int y2);
bool isValidCell(int x, int y);
// تنفيذ المنطق الكامل (هذه يمكن أن تكون نقطة البداية)
void doIt();

#endif

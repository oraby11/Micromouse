#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include <string.h>
#include "API.h"
#define SIZE 16
#define INF INT_MAX




typedef struct {
    int x, y;
} Point;

typedef struct {
    Point points[SIZE * SIZE];
    int front, rear;
} Queue;

void enqueue(Queue *q, int x, int y) {
    if (q->rear < SIZE * SIZE) {
        q->points[q->rear].x = x;
        q->points[q->rear].y = y;
        q->rear++;
    }
}

Point dequeue(Queue *q) {
    Point p = { -1, -1 };
    if (q->front < q->rear) {
        p = q->points[q->front];
        q->front++;
    }
    return p;
}

int isEmpty(Queue *q) {
    return q->front == q->rear;
}

typedef struct {
    int horizontalWalls[SIZE][SIZE];
    int verticalWalls[SIZE][SIZE];
    int manhattanDistance[SIZE][SIZE];
} Grid;

void initializeGrid(Grid *grid) {
    for (int i = 0; i < SIZE; i++) {
        for (int j = 0; j < SIZE; j++) {
            grid->manhattanDistance[i][j] = INF;
            grid->horizontalWalls[i][j] = 0;
            grid->verticalWalls[i][j] = 0;
        }
    }

    grid->manhattanDistance[7][7] = 0;
    grid->manhattanDistance[7][8] = 0;
    grid->manhattanDistance[8][7] = 0;
    grid->manhattanDistance[8][8] = 0;
}

void propagateValues(Grid *grid) {
    Queue q = {{0}, 0, 0};
    enqueue(&q, 7, 7);
    enqueue(&q, 7, 8);
    enqueue(&q, 8, 7);
    enqueue(&q, 8, 8);

    int dx[] = {0, 0, -1, 1};
    int dy[] = {-1, 1, 0, 0};

    while (!isEmpty(&q)) {
        Point p = dequeue(&q);
        int currentDist = grid->manhattanDistance[p.x][p.y];

        for (int i = 0; i < 4; i++) {
            int newX = p.x + dx[i];
            int newY = p.y + dy[i];

            if (newX >= 0 && newX < SIZE && newY >= 0 && newY < SIZE) {
                if (grid->manhattanDistance[newX][newY] > currentDist + 1) {
                    grid->manhattanDistance[newX][newY] = currentDist + 1;
                    enqueue(&q, newX, newY);
                }
            }
        }
    }
}

void shortPath(Grid *grid, int startX, int startY) {

    Queue q = {.front = 0, .rear = 0};
    enqueue(&q, startX, startY);

    int dx[] = {0, 0, -1, 1};  // اتجاهات الحركة على المحور X
    int dy[] = {-1, 1, 0, 0};  // اتجاهات الحركة على المحور Y

    while (!isEmpty(&q)) {
        Point p = dequeue(&q);
        int x = p.x;
        int y = p.y;

        int minVal = INT_MAX;
        Point nextMove = {x, y};

        // تحقق من جميع الاتجاهات الممكنة
        for (int i = 0; i < 4; i++) {
            int newX = x + dx[i];
            int newY = y + dy[i];

            // التحقق من أن الموقع الجديد ضمن حدود الشبكة
            if (newX >= 0 && newX < SIZE && newY >= 0 && newY < SIZE) {
                // التحقق من عدم وجود جدار يمنع الحركة
                if (i == 1 && !API_wallFront()) {
                    if (grid->manhattanDistance[newX][newY] < minVal) {
                        minVal=grid->manhattanDistance[newX][newY];
                        nextMove.x = newX;
                        nextMove.y = newY;
						API_moveForward();
                    }
                }
                else if (i == 2 && !API_wallLeft()) {
                    if (grid->manhattanDistance[newX][newY] < minVal) {
                        minVal = grid->manhattanDistance[newX][newY];
                        nextMove.x = newX;
                        nextMove.y = newY;
						API_turnLeft();
                    }
                }
                else if (i == 3 && !API_wallRight()) {
                    if (grid->manhattanDistance[newX][newY] < minVal) {
                        minVal = grid->manhattanDistance[newX][newY];
                        nextMove.x = newX;
                        nextMove.y = newY;
						API_turnRight();
                    }
                }
				else if (API_wallRight() && API_wallFront() && API_wallLeft()) {
					API_turnRight();
					API_turnRight();

				}
            }
        }
          // تحقق مما إذا كانت القيمة الأدنى أكبر من الخلية الحالية
		  //Update 
        if (minVal > grid->manhattanDistance[x][y]) {
           // grid->manhattanDistance[newX][newY] += 1;
			enqueue(&q, newX, newY);// إعادة تقييم الموقع الحالي
            propagateValues(grid);
           


        }
    }

}
/*
void sensorBasedPath(Grid *grid, int startX, int startY, int Reading) {
    int x = startX;
    int y = startY;

    Queue q = {.front = 0, .rear = 0};  // تعريف قائمة انتظار جديدة

    if (Reading == 1) {
        // يجب أن يتحرك في اتجاه معين بناءً على قراءة المستشعر
        if (x + 1 < SIZE && !grid->horizontalWalls[x + 1][y]) {
            // تحرك لليمين
            x += 1;
            API_turnRight();
        } else if (y - 1 >= 0 && !grid->horizontalWalls[x][y - 1]) {
            // تحرك للأعلى
            y -= 1;
            API_moveForward();
        } else if (x - 1 >= 0 && !grid->horizontalWalls[x - 1][y]) {
            // تحرك لليسار
            x -= 1;
            API_turnLeft();
        }
    } else if (Reading > 1) {
        // ابحث عن الأقل
        int min = INF;
        Point p = {x, y};

        if (x + 1 < SIZE && grid->manhattanDistance[x + 1][y] < min && !API_wallRight()) {
            min = grid->manhattanDistance[x + 1][y];
            p.x = x + 1;
        }
        if (y + 1 < SIZE && grid->manhattanDistance[x][y + 1] < min && !API_wallFront()) {
            min = grid->manhattanDistance[x][y + 1];
            p.y = y + 1;
        }
        if (x - 1 >= 0 && grid->manhattanDistance[x - 1][y] < min && !API_wallLeft()) {
            min = grid->manhattanDistance[x - 1][y];
            p.x = x - 1;
        }

        // تحقق مما إذا كانت القيمة الأدنى أكبر من الخلية الحالية
        if (min > grid->manhattanDistance[x][y]) {
            grid->manhattanDistance[x][y] += 1;
            propagateValues(grid);
            enqueue(&q, x, y);
        } else {
            // تحرك للنقطة ذات المسافة الأقل
            x = p.x;
            y = p.y;
        }
    } else {
        // Reading == 0، قم بالدوران أو اتخاذ إجراء آخر
        API_turnRight(); // على سبيل المثال
    }
}*/

int main() {
    Grid grid;
    initializeGrid(&grid);
    propagateValues(&grid);
	while(1){

    // بدء الروبوت من موقع (0, 0)
    shortPath(&grid, 0, 0);
	}
    return 0;
}

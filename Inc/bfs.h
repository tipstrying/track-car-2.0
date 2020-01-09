#ifndef __BFS_H
#define __BFS_H
#include <stdio.h>
#include <fifo.h>
#include "FreeRTOS.h"
#include "queue.h"

#define MAX_ROW 7
#define MAX_COL 2

typedef struct {
    int ID;
    float X;
    float Y;
    /*
    0: 不使用光电定位
    1: 使用下面光电定位
    2: 使用 X- 侧边光电定位
    3: 使用 X+ 侧边光电定位
    */
    int secondLocalizationFlag;
    int bfsFlag;

} MAP;

//FifoClass databuff;
class BfsClass:public FifoClass
{

public:
    typedef struct
    {
        int y, x, predecessor;
    } point;

    typedef struct
    {
        float x, y;
        int cmd;// 0:navigation 1:pushleft 2:popleft 3:pushright 4:popright
    } piontDef;
    struct {
        piontDef pointCache[20];
        int head;
        int tail;
        
    } pointCache;

    MAP position[MAX_ROW][MAX_COL];

    struct {
        int head, tail;
        point list[20];
    } pointList;

    point self, next;

    int getTrails(point start, point end);
    int pointCachePop( piontDef * );
    int pointCachePush( piontDef );
    int pointCacheAvailabe();
    bool init();

//   int mergeLine( std::list < BfsClass::point> trails, std::list <piontDef> *trailsOut );

    BfsClass(/* args */);
    ~BfsClass();

private:
    point queue[50];
    int head, tail;
    void visit(int row, int col);
    void enqueue(point p);
    point dequeue(void);
    int is_empty(void);



    /*
    int maze[MAX_ROW][MAX_COL];
    int map[MAX_ROW][MAX_COL];
    */
};

#endif

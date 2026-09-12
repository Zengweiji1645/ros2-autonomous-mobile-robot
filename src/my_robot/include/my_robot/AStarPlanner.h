#ifndef ASTAR_PLANNER_H
#define ASTAR_PLANNER_H
#include <utility>
#include <vector>

struct AStarNode
{
    int x;
    int y; 
    double g;
    double h;
    double f;
    int parent_x;
    int parent_y;
};

class AStarPlanner
{
public:

    AStarPlanner();
    
    std::vector<std::pair<int, int>> plan(
    int start_x,
    int start_y,
    int goal_x,
    int goal_y
    );
    
    const std::vector<std::vector<int>>& getGrid() const;

private:
    std::vector<std::vector<int>> grid;

    double heuristic(
    int x,
    int y,
    int goal_x,
    int goal_y
);

    std::vector<std::pair<int, int>> directions =
{
    {1, 0},//右
    {-1, 0},//左
    {0, 1},//上
    {0, -1}//下
};

bool isValid(
    int x,
    int y
);
};

#endif
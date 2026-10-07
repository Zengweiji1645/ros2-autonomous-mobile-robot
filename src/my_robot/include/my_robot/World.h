#ifndef WORLD_H
#define WORLD_H

#include <vector>

class World
{
public:
    World();

    bool isObstacle(
        double x,
        double y
    ) const;

private:
    std::vector<std::vector<int>> grid;

    double obstacle_size; // 阈值，表示障碍物的大小
};

#endif
#include "my_robot/World.h"
#include <cmath>

World::World()
{
    obstacle_size = 0.8; // 设置障碍物的大小

    grid =
    {
        {0, 0, 0, 0, 1},
        {0, 0, 1, 0, 0},
        {0, 1, 1, 1, 0},
        {0, 1, 1, 0, 0},
        {1, 0, 0, 0, 0}
    };
}

bool World::isObstacle(double x, double y) const
{
    double half_size = obstacle_size / 2.0;

    int grid_x = static_cast<int>(std::round(x));
    int grid_y = static_cast<int>(std::round(y));

    // 检查是否超出地图数组范围
    if (grid_x < 0 ||
        grid_y < 0 ||
        grid_x >= static_cast<int>(grid.size()) ||
        grid_y >= static_cast<int>(grid[0].size()))
    {
        return false;
    }

    // 对应位置没有障碍物
    if (grid[grid_x][grid_y] == 0)
    {
        return false;
    }

    // 保留原来的0.8m方形障碍物模型
    double obstacle_center_x = static_cast<double>(grid_x);
    double obstacle_center_y = static_cast<double>(grid_y);

    return (
        std::abs(x - obstacle_center_x) <= half_size &&
        std::abs(y - obstacle_center_y) <= half_size
    );
}
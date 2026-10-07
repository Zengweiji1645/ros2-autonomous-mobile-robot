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

bool World::isObstacle(
    double x,
    double y
) const
{
    double half_size = obstacle_size / 2.0;

    for (std::size_t grid_x = 0; grid_x < grid.size(); grid_x++)
    {
        for (std::size_t grid_y = 0; grid_y < grid[grid_x].size(); grid_y++)
        {
            if (grid[grid_x][grid_y] == 0)
            {
                continue;
            }
            
            double obstacle_center_x = static_cast<double>(grid_x);
            double obstacle_center_y = static_cast<double>(grid_y);

        if (
            std::abs(x - obstacle_center_x) <= half_size &&
            std::abs(y - obstacle_center_y) <= half_size
            )
            {
                return true;
            }
        }
    }

    return false;// 如果没有检测到障碍物，返回 false
}
#include "my_robot/AStarPlanner.h"
#include <cmath>
#include <algorithm>

AStarPlanner::AStarPlanner()
{
    grid =
    {
        {0, 0, 0, 0, 1},
        {0, 0, 1, 0, 0},
        {0, 1, 1, 1, 0},
        {0, 1, 1, 0, 0},
        {1, 0, 0, 0, 0}
    };
}

double AStarPlanner::heuristic(
    int x,
    int y,
    int goal_x,
    int goal_y
)
{
    return std::abs(goal_x - x)
         + std::abs(goal_y - y);
}

const std::vector<std::vector<int>> & AStarPlanner::getGrid() const
{
    return grid;
}

bool AStarPlanner::isValid(
    int x,
    int y
)
{
    if(x<0 || x>=static_cast<int>(grid.size()))
    {
        return false;
    }
    if(y<0 || y>=static_cast<int>(grid[0].size()))
    {
        return false;
    }
    if(grid[x][y] == 1)
    {
        return false;
    }
    return true;
}

std::vector<std::pair<int, int>> AStarPlanner::plan(
    int start_x,
    int start_y,
    int goal_x,
    int goal_y
)
{
    std::vector<AStarNode> open_list;
    std::vector<AStarNode> closed_list;

    AStarNode start_node;

    start_node.x = start_x;
    start_node.y = start_y;

    start_node.g = 0.0;

    start_node.h = heuristic(
        start_x,
        start_y,
        goal_x,
        goal_y
    );

    start_node.f =
        start_node.g + start_node.h;

    start_node.parent_x = -1;
    start_node.parent_y = -1;

    open_list.push_back(start_node);

    while (!open_list.empty())
{
    std::size_t best_index = 0;

    for (std::size_t i = 1; i < open_list.size(); i++)
    {
        if (open_list[i].f < open_list[best_index].f)
        {
            best_index = i;
        }
    }

    AStarNode current_node =
        open_list[best_index];

    open_list.erase(
        open_list.begin() + best_index
    );

    if (
    current_node.x == goal_x &&
    current_node.y == goal_y
)
{
    std::vector<std::pair<int, int>> path;// 找到终点
    AStarNode path_node = current_node;
    path.push_back(
    {path_node.x, path_node.y}
);
    while (
    path_node.parent_x != -1 &&
    path_node.parent_y != -1
)
{
    for (const auto & node : closed_list)
    {
        if (
            node.x == path_node.parent_x &&
            node.y == path_node.parent_y
        )
        {
            path_node = node;
            path.push_back(
                {path_node.x, path_node.y}
            );
            break;
        }
    }
}

    std::reverse(path.begin(), path.end());
    return path;
}
    
    closed_list.push_back(current_node);

    for (const auto & direction : directions)
{
    int new_x =
        current_node.x + direction.first;

    int new_y =
        current_node.y + direction.second;

    if (!isValid(new_x, new_y))
    {
        continue;
    }

    bool already_closed = false;

    for (const auto & position : closed_list)    //Check if the neighbor is already in the closed list
    {
        if (
            position.x == new_x &&
            position.y == new_y
        )
        {
            already_closed = true;
            break;
        }
    }

    if (already_closed)
    {
        continue;
    }

    AStarNode neighbor;

    neighbor.x = new_x;
    neighbor.y = new_y;

    neighbor.g = current_node.g + 1.0;

    neighbor.h = heuristic(
        new_x,
        new_y,
        goal_x,
        goal_y
    );

    neighbor.f =
        neighbor.g + neighbor.h;

    neighbor.parent_x = current_node.x;
    neighbor.parent_y = current_node.y;

    bool already_open = false;

    for (auto & open_node : open_list)
    {
        if (
            open_node.x == neighbor.x &&
            open_node.y == neighbor.y
        )
        {
            already_open = true;

            if (neighbor.g < open_node.g)
            {
                open_node = neighbor;
            }

            break;
        }
    }

    if (!already_open)
    {
        open_list.push_back(neighbor);
    }
}

}

return {};

}
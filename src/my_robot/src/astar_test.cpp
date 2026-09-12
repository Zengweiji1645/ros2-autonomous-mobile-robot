#include <iostream>

#include "my_robot/AStarPlanner.h"

int main()
{
    AStarPlanner planner;

    auto path = planner.plan(
        0, 0,   // Start
        4, 4    // Goal
    );

    std::cout << "Path:" << std::endl;

    for (const auto & point : path)
    {
        std::cout
            << "("
            << point.first
            << ", "
            << point.second
            << ")"
            << std::endl;
    }

    return 0;
}
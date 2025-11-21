#include "placer.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>

void Placer::init_place(Design &design)
{
    int current_x = 0;
    int current_y = 0;
    for (auto block : design.logicBlocks)
    {
        block->x = current_x;
        block->y = current_y;
        current_x++;
        if (current_x >= design.C)
        {
            current_x = 0;
            current_y++;
        }
        if (current_y >= design.R)
        {
            std::cerr << "Error: Not enough space on FPGA to place all blocks!" << std::endl;
            break;
        }
    }
    std::cout << "Initial placement completed." << std::endl;
}

double Placer::calc_HPWL(Design &design)
{
    double total_HPWL = 0;
    for (const auto &net : design.nets)
    {
        double min_x = std::numeric_limits<double>::max();
        double max_x = std::numeric_limits<double>::lowest();
        double min_y = std::numeric_limits<double>::max();
        double max_y = std::numeric_limits<double>::lowest();
        bool has_terminals = false;
        for (auto block : net->blocks)
        {
            has_terminals = true;
            double bx = (double)block->x;
            double by = (double)block->y;
            if (bx < min_x)
                min_x = bx;
            if ((bx + 1.0) > max_x)
                max_x = bx + 1.0;
            if (by < min_y)
                min_y = by;
            if ((by + 1.0) > max_y)
                max_y = by + 1.0;
        }
        for (auto pin : net->pins)
        {
            has_terminals = true;
            double px = pin->x;
            double py = pin->y;
            if (px < min_x)
                min_x = px;
            if (px > max_x)
                max_x = px;
            if (py < min_y)
                min_y = py;
            if (py > max_y)
                max_y = py;
        }
        if (!has_terminals)
            continue;
        double hpwl = (max_x - min_x) + (max_y - min_y);
        total_HPWL += hpwl;
    }
    return total_HPWL;
}

double Placer::calc_congestion_coefficient(Design &design)
{
    int R = design.R;
    int C = design.C;
    long long N = (long long)R * C;
    if (N == 0)
        return 0.0;
    std::vector<std::vector<int>> U(C, std::vector<int>(R, 0));
    for (const auto &net : design.nets)
    {
        double min_x = std::numeric_limits<double>::max();
        double max_x = std::numeric_limits<double>::lowest();
        double min_y = std::numeric_limits<double>::max();
        double max_y = std::numeric_limits<double>::lowest();
        bool has_terminals = false;
        for (auto block : net->blocks)
        {
            has_terminals = true;
            double bx = (double)block->x;
            double by = (double)block->y;
            if (bx < min_x)
                min_x = bx;
            if ((bx + 1.0) > max_x)
                max_x = bx + 1.0;
            if (by < min_y)
                min_y = by;
            if ((by + 1.0) > max_y)
                max_y = by + 1.0;
        }
        for (auto pin : net->pins)
        {
            has_terminals = true;
            double px = pin->x;
            double py = pin->y;
            if (px < min_x)
                min_x = px;
            if (px > max_x)
                max_x = px;
            if (py < min_y)
                min_y = py;
            if (py > max_y)
                max_y = py;
        }
        if (!has_terminals)
            continue;
        int start_x = std::max(0, (int)std::ceil(min_x));
        int start_y = std::max(0, (int)std::ceil(min_y));
        for (int x = start_x; x < max_x && x < C; ++x)
        {
            for (int y = start_y; y < max_y && y < R; ++y)
            {
                U[x][y]++;
            }
        }
    }
    double sum_U = 0.0;
    double sum_sq_U = 0.0;
    for (int x = 0; x < C; ++x)
    {
        for (int y = 0; y < R; ++y)
        {
            double val = (double)U[x][y];
            sum_U += val;
            sum_sq_U += (val * val);
        }
    }
    double term1 = sum_sq_U / (double)N;
    double mean_U = sum_U / (double)N;
    double term2 = mean_U * mean_U;
    if (term2 == 0.0)
        return 1.0;
    double CC = term1 / term2;
    return CC;
}

void Placer::run(Design &design)
{
}
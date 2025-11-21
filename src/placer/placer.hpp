#pragma once
#include "../structure/structure.hpp"
#include <vector>

class Placer
{
public:
    void init_place(Design &design);
    double calc_HPWL(Design &design);  
    double calc_congestion_coefficient(Design &design);
    void run(Design &design);
};
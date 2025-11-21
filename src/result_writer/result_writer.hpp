#pragma once
#include "../structure/structure.hpp"
#include <string>
#include <vector>

class ResultWriter
{
public:
    void write_result(const std::string &filename, const Design &design);
};
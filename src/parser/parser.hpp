#pragma once
#include <string>

class Design;

class Parser
{
public:
    static void parse(const std::string &filename, Design &design);
};
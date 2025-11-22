#pragma once
#include <string>

class Design;

class Writer
{
public:
    static void write(const std::string &filename, const Design &design);
};
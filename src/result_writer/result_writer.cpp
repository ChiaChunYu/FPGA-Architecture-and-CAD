#include "result_writer.hpp"
#include <fstream>
#include <iostream>

void ResultWriter::write_result(const std::string &filename, const Design &design)
{
    std::ofstream outfile(filename);
    if (!outfile.is_open())
    {
        std::cerr << "Error: Could not open output file " << filename << std::endl;
        return;
    }
    for (const auto block : design.logicBlocks)
    {
        outfile << block->name << " " << block->x << " " << block->y << "\n";
    }
    outfile.close();
}
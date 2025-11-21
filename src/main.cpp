#include "structure/structure.hpp"
#include "parser/parser.hpp"
#include "placer/placer.hpp"
#include "result_writer/result_writer.hpp"
#include <iostream>
#include <chrono>

int main(int argc, char *argv[])
{
    if (argc != 3)
    {
        std::cerr << "Usage: ./placer <input_file> <output_file>" << std::endl;
        return 1;
    }
    auto startTime = std::chrono::high_resolution_clock::now();
    std::string inputFile = argv[1];
    std::string outputFile = argv[2];
    Design design;
    Parser parser;
    Placer placer;
    ResultWriter result_writer;
    parser.parse(inputFile, design);
    placer.init_place(design);
    double congestion_coefficient = placer.calc_congestion_coefficient(design);
    std::cout << "Congestion Coefficient: " << congestion_coefficient << std::endl;
    double initial_HPWL = placer.calc_HPWL(design);
    std::cout << "Initial HPWL: " << initial_HPWL << std::endl;
    result_writer.write_result(outputFile, design);
    auto endTime = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = endTime - startTime;
    std::cout << "Total runtime: " << elapsed.count() << " seconds." << std::endl;
    return 0;
}
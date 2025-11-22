#include "design/design.hpp"
#include "parser/parser.hpp"
#include "placer/placer.hpp"
#include "writer/writer.hpp"
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
    Parser::parse(inputFile, design);
    Placer::InitPlace(design);
    double congestion_coefficient = Placer::CalculateCongestionCoefficient(design);
    std::cout << "Congestion Coefficient: " << congestion_coefficient << std::endl;
    double initial_HPWL = Placer::CalculateHPWL(design);
    std::cout << "Initial HPWL: " << initial_HPWL << std::endl;
    Writer::write(outputFile, design);
    auto endTime = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = endTime - startTime;
    std::cout << "Total runtime: " << elapsed.count() << " seconds." << std::endl;
    return 0;
}
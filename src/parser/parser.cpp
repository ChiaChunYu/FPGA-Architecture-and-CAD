#include "parser.hpp"
#include <fstream>
#include <sstream>
#include <unordered_map>
#include <iostream>

void Parser::parse(const std::string &filename, Design &design)
{
    std::ifstream infile(filename);
    if (!infile.is_open())
    {
        throw std::runtime_error("Error: Could not open file " + filename);
    }
    std::unordered_map<std::string, LogicBlock *> blockMap;
    std::unordered_map<std::string, IOPin *> pinMap;
    infile >> design.R >> design.C >> design.numLogicBlocks >> design.numIOPins >> design.numNets;
    design.logicBlocks.reserve(design.numLogicBlocks);
    design.ioPins.reserve(design.numIOPins);
    design.nets.reserve(design.numNets);
    for (int i = 0; i < design.numLogicBlocks; ++i)
    {
        std::string name;
        infile >> name;
        LogicBlock *newBlock = new LogicBlock(name);
        design.logicBlocks.push_back(newBlock);
        blockMap[name] = newBlock;
    }
    for (int i = 0; i < design.numIOPins; ++i)
    {
        std::string name;
        int x, y;
        infile >> name >> x >> y;
        IOPin *newPin = new IOPin(name, x, y);
        design.ioPins.push_back(newPin);
        pinMap[name] = newPin;
    }
    for (int i = 0; i < design.numNets; ++i)
    {
        std::string name;
        int degree;
        infile >> name >> degree;
        Net *newNet = new Net(name, degree);
        for (int j = 0; j < degree; ++j)
        {
            std::string termName;
            infile >> termName;
            auto blockIt = blockMap.find(termName);
            if (blockIt != blockMap.end())
            {
                LogicBlock *lb = blockIt->second;
                newNet->blocks.push_back(lb);
                lb->nets.push_back(newNet);
            }
            else
            {
                auto pinIt = pinMap.find(termName);
                IOPin *pin = pinIt->second;
                newNet->pins.push_back(pin);
                pin->nets.push_back(newNet);
            }
        }
        design.nets.push_back(newNet);
    }
    infile.close();
}
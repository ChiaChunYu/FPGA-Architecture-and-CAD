#pragma once
#include <string>
#include <vector>

class LogicBlock;
class IOPin;
class Net;

class LogicBlock
{
public:
    std::string name;
    int x;
    int y;
    std::vector<Net *> nets;
    LogicBlock(std::string n);
};

class IOPin
{
public:
    std::string name;
    int x;
    int y;
    std::vector<Net *> nets;
    IOPin(std::string n, int x, int y);
};

class Net
{
public:
    std::string name;
    int degree;
    std::vector<LogicBlock *> blocks;
    std::vector<IOPin *> pins;
    Net(std::string n, int d);
};

class Design
{
public:
    int R;
    int C;
    int numLogicBlocks;
    int numIOPins;
    int numNets;
    std::vector<LogicBlock *> logicBlocks;
    std::vector<IOPin *> ioPins;
    std::vector<Net *> nets;
};
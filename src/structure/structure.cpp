#include "structure.hpp"

LogicBlock::LogicBlock(std::string n) : name(n), x(0), y(0)
{
}

IOPin::IOPin(std::string n, int x, int y) : name(n), x(x), y(y)
{
}

Net::Net(std::string n, int d) : name(n), degree(d)
{
}

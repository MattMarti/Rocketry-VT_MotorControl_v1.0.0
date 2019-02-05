#include <iostream>
#include <string>
#include <vector>

#include <transmission.h>

int main(int argc, char **argv)
{
    if (argc == 1) return 1;

    uint8_t id = std::stoi(argv[1]);

    if (id == '#')
    {
        std::string message;
        if (argc > 2)
        {
            message = std::string(argv[2]);
        }
        auto packet = transmission::buildPacket(message);
        for (auto e : packet)
            std::cout << e;
    }
    else
    {
        std::vector<uint8_t> data;
        for (int i = 2; i < argc; ++i)
        {
            data.push_back(std::stoi(argv[i]));
        }
        auto packet = transmission::buildPacket(id, data);
        for (auto e : packet)
            std::cout << e;
    }
    return 0;
}

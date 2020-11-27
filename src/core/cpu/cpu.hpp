#pragma once

#include "common/logger.hpp"
#include "common/types.hpp"

namespace CGB {
class Bus;

namespace CPU {

class Interpreter;

class BaseCPU {
public:
    BaseCPU();
    BaseCPU(BaseCPU&&);
    ~BaseCPU();
    BaseCPU& operator=(BaseCPU&&);

    void Install(Bus& bus);
    void Run();

private:
    std::unique_ptr<Interpreter> interpreter;
};

} // namespace CPU
} // namespace CGB
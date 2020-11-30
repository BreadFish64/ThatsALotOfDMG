#pragma once

#include "common/logger.hpp"
#include "common/types.hpp"

namespace CGB::Core {
class Bus;

namespace CPU {

class Interpreter;

class BaseCPU {
public:
    virtual ~BaseCPU(){};

    virtual void Install(Bus& bus) = 0;
    virtual void Run() = 0;
};

class MainCPU : public BaseCPU {
public:
    MainCPU();
    MainCPU(MainCPU&&);
    ~MainCPU();
    MainCPU& operator=(MainCPU&&);

    virtual void Install(Bus& bus) override;
    virtual void Run() override;

    BaseCPU& GetImpl() const { return *impl; }

private:
    std::unique_ptr<BaseCPU> impl;
};

} // namespace CPU
} // namespace CGB
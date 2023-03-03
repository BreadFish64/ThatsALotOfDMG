#pragma once

#include "common/logger.hpp"
#include "common/types.hpp"

namespace CGB::Core {
class Bus;
enum class Event;

namespace CPU {

class Interpreter;

class BaseCPU {
public:
    virtual ~BaseCPU(){};

    virtual void Install(Bus& bus) = 0;
    virtual void Run() = 0;

    virtual void ScheduleEvent(u64 event_timestamp, Event event) = 0;
    virtual void DescheduleEvent(Event event) = 0;

    virtual std::chrono::nanoseconds GetSpeed() = 0;
};

class MainCPU : public BaseCPU {
public:
    MainCPU();
    MainCPU(MainCPU&&);
    ~MainCPU();
    MainCPU& operator=(MainCPU&&);

    virtual void Install(Bus& bus) override;
    virtual void Run() override;

    virtual void ScheduleEvent(u64 timestamp, Event event) override {
        impl->ScheduleEvent(timestamp, event);
    };
    virtual void DescheduleEvent(Event event) override { impl->DescheduleEvent(event); }

    BaseCPU& GetImpl() const { return *impl; }

    virtual std::chrono::nanoseconds GetSpeed() override { return impl->GetSpeed(); };

private:
    std::unique_ptr<BaseCPU> impl;
};

} // namespace CPU
} // namespace CGB::Core
#include "mcu_main/Rt.h"

#ifdef ENABLE_SILSIM_MODE
#include <windows.h>
#include <vector>
#include <ctime>
#include <iostream>
#include "mcu_main/ISS_SILSIM/src/Sensors/Sensor.h"
#include "mcu_main/ISS_SILSIM/src/SILSIM.h"
#include "mcu_main/emulation.h"

void setup();


struct ThreadInfo {
public:
    explicit ThreadInfo(const char* name, void* handle) : _name(name), fiber_handle(handle) { }

    const char* _name;
    void* fiber_handle;
};

struct ThreadManager {
public:
    ThreadManager() : threads(), next_thread(1) { }

    std::vector<ThreadInfo> threads;
    size_t next_thread;

    bool debug = false;

    void yield() {
        if (threads.empty()) {
            return;
        }
        if (next_thread >= threads.size()) {
            next_thread = 0;
        }
        ThreadInfo* switch_to = &threads[next_thread];
        next_thread++;
        if (debug)
            std::cout << "Switching to " << switch_to->_name << std::endl;
        SwitchToFiber(switch_to->fiber_handle);
    }
};

ThreadManager thread_manager;

uint32_t getTime() {
    return clock();
}

void threadYield() {
    thread_manager.yield();
}

void createThread(const char* name, void fn(void*), size_t stack, void* arg) {
    void* handle = CreateFiber(stack, fn, arg);
    thread_manager.threads.emplace_back(name, handle);
}

void threadSleep(int32_t time_ms) {
    clock_t start = clock();
    clock_t wait = time_ms * CLOCKS_PER_SEC / 1000;
    while (true) {
        clock_t curr = clock();
        if (curr >= start + wait) {
            return;
        }

        threadYield();
    }
}

void SerialPatch::println(const char* s) {
    std::cout << s << std::endl;
}

void SerialPatch::begin(int baudrate) {

}

SerialPatch Serial;

void run_sim(void* arg) {
    Simulation* sim = static_cast<Simulation*>(arg);

    for (int i = 0; i < 100000; i++) {
        if (i % 100 == 0)
            std::cout << "Run " << i << std::endl;

        bool res = sim->step();
        threadYield();
    }
    std::cout << "done" << std::endl;
    exit(0);
}


int main() {
    std::cout << "started" << std::endl;
    thread_manager.threads.emplace_back("main", ConvertThreadToFiber(nullptr));

    Rocket rocket = createRocket();

    Accelerometer accel1(rocket, 100);
    accel1.enable_noise_injection();

    GyroscopeSensor gyro1(rocket, 100, 0.001, 0.01);
    gyro1.enable_noise_injection();
    Thermometer thermo1(rocket, 100);
    Barometer baro1(rocket, 100, 0, 150 / 1.645);
    baro1.enable_noise_injection();
    EmulatedGPSSensor gps1(rocket, 10);
    EmulatedMagnetometerSensor mag1(rocket, 100);

    Atmosphere atmo = createAtmosphere();
    ThrustCurveSolidMotor motor = createMotor();
    RungeKutta engine = createPhysics(rocket, motor, atmo);
    Simulation sim = createSimulation(rocket, motor, atmo, &engine);
    createThread("runsim", run_sim, 8000, &sim);

    sim.add_sensor(&gyro1);
    emulatedGyro = &gyro1;
    sim.add_sensor(&accel1);
    emulatedKX = &accel1;
    sim.add_sensor(&thermo1);
    emulatedThermometer = &thermo1;
    sim.add_sensor(&baro1);
    emulatedMS = &baro1;
    sim.add_sensor(&gps1);
    emulatedGPS = &gps1;
    sim.add_sensor(&mag1);
    emulatedMagnetometer = &mag1;

    std::cout << "to setup" << std::endl;

    setup();
}

#endif
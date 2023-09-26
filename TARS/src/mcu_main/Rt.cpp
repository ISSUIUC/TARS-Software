#include "mcu_main/Rt.h"

#ifdef ENABLE_SILSIM_MODE

#include <vector>
#include <iostream>

#include "mcu_main/ISS_SILSIM/src/Sensors/Sensor.h"
#include "mcu_main/ISS_SILSIM/src/SILSIM.h"
#include "mcu_main/emulation.h"
#include "mcu_main/fiber.h"

void setup();

static Simulation* g_sim;
struct ThreadInfo {
   public:
    explicit ThreadInfo(const char* name, FiberHandle handle) : _name(name), fiber_handle(handle) {}

    const char* _name;
    FiberHandle fiber_handle;
};

struct ThreadManager {
   public:
    ThreadManager() : threads(), next_thread(1) {}

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
        if (debug) std::cout << "Switching to " << switch_to->_name << std::endl;
        EmuSwitchToFiber(switch_to->fiber_handle);
    }
};

ThreadManager thread_manager;

uint32_t getTime() { return g_sim->get_time(); }

void threadYield() { thread_manager.yield(); }

void createThread(const char* name, void fn(void*), size_t stack, void* arg) {
    FiberHandle handle = EmuCreateFiber(stack, fn, arg);
    thread_manager.threads.emplace_back(name, handle);
}

void threadSleep(int32_t time_ms) {
    double start = g_sim->get_time();
    double wait = time_ms / 1000.0;
    while (true) {
        double curr = g_sim->get_time();
        if (curr >= start + wait) {
            return;
        }

        threadYield();
    }
}

void tone(uint8_t pin, uint16_t frequency) {}
void tone(uint8_t pin, uint16_t frequency, uint32_t duration) {}
void noTone(uint8_t pin) {}

void SerialPatch::println(const char* s) { std::cout << s << '\n'; }

void SerialPatch::begin(int baudrate) {}

SerialPatch Serial;

void recur() {
    char data[4096]{};
    recur();
}

int catch_over(int v) { std::cout << v << '\n'; }

void run_sim(void* arg) {
    for (int i = 0; i < 100000; i++) {
        if (i % 1000 == 0) std::cout << "Run " << i << std::endl;

        if (i >= 3000) {
            // thread_manager.debug = true;
        }
        bool res = g_sim->step();
        if (!res) break;
        threadYield();
    }
    exit(0);
}

int main() {
    thread_manager.threads.emplace_back("main", EmuConvertThreadToFiber());

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

    g_sim = &sim;

    setup();
}

#endif
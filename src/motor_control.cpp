#include <iostream>
extern "C" {
    #include <pigpio.h>
}
#include <unistd.h>  // for usleep()

// GPIO pins for PUL and DIR (BCM numbering)
const int PUL_PIN = 5;
const int DIR_PIN = 3;

// Function to make one step
void step() {
    gpioWrite(PUL_PIN, 1);  // HIGH
    gpioDelay(500);          // 0.0005 sec (500 microseconds)
    gpioWrite(PUL_PIN, 0);  // LOW
    gpioDelay(500);          // 0.0005 sec (500 microseconds)
}

// Function to make continuous revolutions
void continuous_rotation(int rev_per_minute) {
    double delay = 60.0 / (400.0 * rev_per_minute);  // 400 steps per revolution
    for (int i = 0; i < 400; i++) { // One full revolution
        step();
        usleep(delay * 1e6); // Convert seconds to microseconds
    }
}

int main() {
    if (gpioInitialise() < 0) {
        std::cerr << "Failed to initialize pigpio!" << std::endl;
        return 1;
    }

    // Set up GPIO mode and pins
    gpioSetMode(PUL_PIN, PI_OUTPUT);
    gpioSetMode(DIR_PIN, PI_OUTPUT);

    // Set initial direction (clockwise)
    gpioWrite(DIR_PIN, 0);  // LOW

    try {
        continuous_rotation(100);  // Adjust to desired revolutions per minute
    } catch (...) {
        std::cerr << "An error occurred." << std::endl;
    }

    // Cleanup
    gpioTerminate();
    return 0;
}


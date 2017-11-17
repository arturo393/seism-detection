// Example usage for LSM6 library by pololu.

#include "LSM6.h"

// Initialize objects from the lib
LSM6 lSM6;

void setup() {
    // Call functions on initialized library objects that require hardware
    lSM6.begin();
}

void loop() {
    // Use the library's initialized objects and functions
    lSM6.process();
}

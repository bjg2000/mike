# Trajectory Optimization for ESP32

This project ports a dart trajectory calculation program (`trajopt_pi5`) to the ESP32 microcontroller. The original C code used the NLopt library for nonlinear optimization, which is not suitable for resource-constrained devices like the ESP32.

## Current Implementation (`src/main.cpp`)

The `src/main.cpp` file contains a direct port of the core physics simulation (RK4 method) and a simplified grid search optimizer to replace NLopt.

**Performance Note:**
Running the grid search optimizer directly on the ESP32 is computationally intensive. Even with a reduced search space, a single trajectory calculation can take approximately 25 seconds (as observed in testing). This is too slow for real-time applications.

## Future Improvements: Lookup Table Approach

To achieve real-time performance on the ESP32, a lookup table (LUT) approach is highly recommended. The idea is to pre-compute a wide range of trajectory solutions on a powerful machine (e.g., a PC) and store these solutions in a compact table in the ESP32's flash memory. The ESP32 can then quickly retrieve the optimal parameters by interpolating values from this table.

This project includes the beginnings of a lookup table solution:
- `lookupTableGenerator.py`: A Python script designed to pre-compute trajectory solutions and generate a C++ header file (`lut.h`) containing the lookup table.
- `lookupTableArduino.cpp`: A sample C++ file demonstrating how the generated lookup table can be integrated and used on the ESP32.

The `lookupTableGenerator.py` script needs to be run on a PC to generate the full `lut.h` file, which can then be included in the ESP32 project. This approach will drastically reduce the computation time on the ESP32, making the trajectory calculations near-instantaneous.

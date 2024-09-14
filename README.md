# STM32 FFT and Graph-Based Signal Analysis

## Overview

This project demonstrates an embedded signal analysis pipeline using an STM32 microcontroller. It leverages the FFT (Fast Fourier Transform) algorithm to analyze sampled ADC data, computes magnitudes, and processes the results using a priority queue and a graph structure. The purpose is to simulate signal processing and data relationship analysis in an embedded environment.

### Components
- **FFT Algorithm**: Recursively computes the Fast Fourier Transform of the input signal.
- **Priority Queue**: Stores and sorts magnitude values from the FFT result.
- **Graph**: Stores vertices based on the magnitudes and dynamically adds edges based on the relationships between signal peaks and valleys.

## Features

- **ADC Sampling**: Reads input signals using the STM32 ADC, sampling the real part of the signal.
- **FFT Processing**: Performs FFT on the sampled data to convert it from the time domain to the frequency domain.
- **Magnitude Computation**: Calculates the magnitude of each FFT coefficient and stores it in a priority queue.
- **Graph Creation**: Adds the magnitudes as vertices in the graph and connects them based on peaks and valleys in the signal.

## Code Summary

### Main Flow

1. **ADC Sampling**: Samples data from the ADC, stores it in the real part of the complex array, and passes it for FFT processing.
   ```c
   Poll_ADC_Samples(coefs, SAMPLE_SIZE);

2. FFT: Performs the recursive FFT on the sampled data (real and imaginary parts) to transform the signal to the frequency domain.
FFT(coefs, SAMPLE_SIZE);

3. Magnitude Calculation: Computes the magnitudes of the FFT coefficients.
Compute_Magnitudes(coefs, magbuffer, SAMPLE_SIZE);

4. Priority Queue: The magnitude values are stored in a priority queue, sorted as a max-heap.
priority_queue_from_array(&pq, magbuffer, SAMPLE_SIZE);

5. Graph Construction: Magnitude values are added as vertices in the graph, and edges are added based on the relationships between signal peaks and valleys.
create_graph(&graph, SAMPLE_SIZE);

6. BFS (Breadth-First Search): Runs a BFS on the graph to traverse the connected components based on the magnitude relationships.
bfs(&graph, 3);

Hardware Setup
STM32 Microcontroller: This project uses the STM32F401RE for sampling analog signals, performing FFT, and processing the results.
ADC: Configured for single-channel input to capture analog signal data.
Libraries Used
HAL (Hardware Abstraction Layer): For STM32 peripheral configuration and ADC handling.
Math Functions: Utilizes math.h for operations like cos, sin, and sqrt in the FFT and magnitude computation.

Installation and Setup
Clone the repository:

git clone https://github.com/your-repo/stm32-fft-graph.git
Open the project in your STM32 IDE (STM32CubeIDE, Keil, etc.).

Configure the microcontroller to use the ADC peripheral and enable floating-point operations.
I think it syncs my settings, but make sure SAMPLE_SIZE=32 is in the compiler "define" setting.

Build and flash the program onto your STM32 board.

How It Works
The microcontroller samples ADC values representing an analog signal.
The FFT algorithm processes the sampled signal, converting it to the frequency domain.
The magnitude of each FFT output is calculated and stored in a priority queue.
A graph is constructed from the magnitudes, with edges added based on peak and valley relationships in the signal.
A BFS traversal is performed on the graph to simulate signal analysis.

License
This project is licensed under the terms of the GNU General Public License v2.0. See the LICENSE file for details.



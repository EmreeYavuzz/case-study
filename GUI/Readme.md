Usage
Connect your STM32F429 Discovery board to COM13
Ensure your firmware is running and sending gyroscope data via UART
Run the visualizer:
Click "Connect COM13" to start visualization
Move your board and watch the 3D model rotate in real-time!
Controls
Connect COM13: Connect/disconnect from serial port
Reset Orientation: Reset the 3D view to initial position
Expected Data Format
The program expects gyroscope data in this format:

This matches the output format from your main.c implementation.

Technical Details
Port: COM13
Baud Rate: 115200
Data Bits: 8
Parity: None
Stop Bits: 1
Flow Control: None
3D Model Components
The visualizer shows:

Green PCB base (main board)
Blue LCD screen
Black MCU chip (STM32F429)
Red gyroscope sensor (I3G4250D)
Silver USB connector
Green and Red LEDs
RGB axes (X=Red, Y=Green, Z=Blue)

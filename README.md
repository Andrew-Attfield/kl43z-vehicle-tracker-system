# kl43z-vehicle-tracker-system

A multi-stage embedded systems project developed on the Freescale KL43Z (ARM Cortex-M0+) microcontroller. This system detects vehicle speed using infrared sensors, calculates fines based on dynamic speed limits (via ADC), and logs data to a Linux-based database using a Bash automated pipeline.

🛠 Features

    Real-time Event Handling: Uses external interrupts (GPIO) and IR sensors to detect vehicle entry and exit with microsecond precision.

    Precision Timing: Implements the SysTick Timer to measure elapsed time between sensors, converting pulse-width to vehicle speed (km/h).

    Dynamic Speed Limiting: Uses an Analog-to-Digital Converter (ADC) to map potentiometer voltage to a variable speed limit (20−100 km/h).

    Hardware-Software Integration: Communicates via UART/Serial to a Windows VM using PuTTY for real-time logging.

    Automated Data Pipeline: A Bash script monitors log files every 10 seconds, calculates traffic fines using a mathematical model, and maintains a persistent database.

📐 System Architecture

    Sensors (PTD3/PTA4): Detect objects and trigger Falling-Edge Interrupts.

    Microcontroller (KL43Z): Processes timing, speed calculation, and LED status.

    Communication (UART): Transmits formatted strings (ID:Speed:SpeedLimit) to a host PC.

    Backend (Bash/Linux): Sanitizes data, applies fine logic, and updates Database.txt.

💻 Technical Stack

    Languages: C (Embedded), Bash (Scripting).

    Hardware: FRDM-KL43Z Board, IR Sensors, Potentiometer.

    Protocols: GPIO, Interrupts (NVIC), ADC, UART, SysTick.

    Tools: MCUXpresso IDE, PuTTY, Linux Terminal.

3. Key Mathematical Models

Speed Calculation:
Speed=Elapsed Time10 meters​×3.6

Fine Calculation Logic: Implemented a tiered penalty system:

    Δv<10: 0fine

    10≤Δv<20: $50

    20≤Δv<30: $100

    Δv≥30: $100+10(Δv−30)

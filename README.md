<h1>Two Qubit Simulation via Raspberry Pis</h1>

This repository contains code that represents some quantum gates which are represented through different operations on Raspberry Pis. 
The Raspberry Pis are implemented into pillows to enable physical operations like flipping, dropping, squeezing and touching the pillows, with each corresponding to a quantum gate.

<h2>Contents</h2>

- quantum_2bit_simulation : Code that simulates the implemented qubit operations
- entangleORdie : Small game containing few quantum operations
- pn532 : NFC library

### **Setup Instructions**
1. **Hardware**: 
   - 2 Raspberry Pis
   - Accelerometer (ADXL345)
   - LED strips
   - Physical buttons or switches
   - NFC reader (only needed on one Pi)
   - All Pis need to be connected to the same local network with static IPs (set in the code).

2. **Connections**:
   - Wire up the accelerometer, LEDs, and buttons to the Raspberry Pis based on the GPIO pins defined in the code.
   
3. **Running the Simulation**:
   - After cloning this repo to both Raspberry Pis, do the following:
     1. SSH or VNC into each Pi.
     2. On the Pi with the NFC reader, run:
        ```bash
        sudo python3 qubit_logic.py
        ```
     3. On the other Pi, run:
        ```bash
        sudo python3 qubit_logic_client.py
        ```

### **To-Do**
- [ ] Add guide for the "entangleORdie" game.
- [ ] Tidy up the `pn532` library path.

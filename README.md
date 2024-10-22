<h1>Two Qubit Simulation via Raspberry Pis</h1>

This repository contains code that represents some quantum gates which are represented through different operations on Raspberry Pis. 
The Raspberry Pis are implemented into pillows to enable physical operations like flipping, dropping, squeezing and touching the pillows, with each corresponding to a quantum gate.

<h2>Contents</h2>

- quantum_2bit_simulation : Code that simulates the implemented qubit operations
- entangleORdie : Small game containing few quantum operations
- pn532 : NFC library

<h2>Setup and quick guide</h2>

The Raspbery Pis have to be connected to an accelerometer (adxl345), LED strips, physical buttons/switched and an NFC reader (only 1 needs this), to the corresponding GPIO pins stated in the code.
Additionally, they function via TCP/IP, so they need to be in a local network with static IP addresses also specified in the code.

To run the quantum two qubit simulation (assuming repo has been cloned on the pis):

1. cd into quantum_2bit_simulation
2. SSH or VNC into both raspberries
3. on raspberry with NFC run: sudo python3 qubit_logic.py
4. on the other run: sudo python3 qubit_logic_client.py

TO-DO:
add entangleORdie guide, move pn532 path

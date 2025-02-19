# Digital Delay Filter

## Overview
The **Digital Delay Filter** is a discrete-time filter that introduces a specified delay to an input signal. It is implemented as a [circular buffer](https://en.wikipedia.org/wiki/Circular_buffer), where the output is a delayed version of the input by a predefined amount of time.

## Features
- Implements a **fixed digital delay** in discrete-time systems.
- Uses an **efficient circular buffer** for handling delayed data.
- Supports **customizable sampling time (`Tc`) and delay (`L`)**.
- Provides methods for **initialization (`initialize`), starting conditions (`starting`), and processing (`step`)**.

## Usage
### **Example Usage**
```python
from delay_filter import Delay

# Create a delay filter with 10ms sampling time and 50ms delay
delay = Delay(Tc=0.01, L=0.05)
delay.initialize()

delay.starting(0)  # Initialize buffer with 0

# Apply delay filter to an input sequence
input_signal = [1, 2, 3, 4, 5]
output_signal = [delay.step(x) for x in input_signal]
print(output_signal)  # Expected: [0, 0, 0, 1, 2]
```

## License
This project is licensed under the **GNU General Public License v3 (GPLv3)**. See the [LICENSE](LICENSE) file for details.

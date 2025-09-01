# Motor Trigger Logger – pyX2Cscope GUI

This repository contains a Tkinter-based Python utility that communicates with a motor-control device via [pyX2Cscope](https://pypi.org/project/pyx2cscope/). It performs a triggered capture of motor variables and exports scaled data for analysis.

The logger triggers on the *OmegaElectrical* variable rising past a fixed raw level of 700 and then records a high‑resolution window using a fixed sample-time factor of **f = 50** (2.5 ms per sample, ≈400 Hz). Captured series can be exported to TSV in real units and optionally plotted when `matplotlib` is installed.

## Included file

- `logger.py` – Tkinter application implementing the triggered motor logger with per‑channel scaling and TSV export.

## Requirements

- Python 3.11+
- [pyx2cscope](https://pypi.org/project/pyx2cscope/) for hardware communication (`pip install pyx2cscope`)
- `pyserial` (installed with pyx2cscope)
- Optional: `matplotlib` for plotting captured data

## Running the logger

1. Flash your motor firmware that exposes the necessary variables through X2Cscope and connect the board via USB.
2. Start the GUI:

   ```bash
   python logger.py
   ```

3. Select the compiled ELF file and serial port, then click **Connect**.
4. Enter a speed request and start a capture. The tool issues a one‑shot **RUN**, waits for the trigger, and automatically **STOPs** after 10 seconds or when stopped manually.
5. Use **Export…** to save a TSV with scaled values and **Plot** to visualise currents or speed if `matplotlib` is available.

## License

This code is provided for demonstration purposes without warranty.

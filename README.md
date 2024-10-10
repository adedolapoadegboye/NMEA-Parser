# NMEA Parser and Logger

## Overview

The **NMEA Parser and Logger** is a Python-based tool designed for serial communication with multiple GNSS devices. It reads NMEA sentences from multiple COM ports, parses the sentences, and logs the results into both console output and files (raw NMEA logs and parsed data in Excel format). The tool is multi-threaded, allowing the user to test multiple devices simultaneously.

## Features

- Multi-threaded support for testing multiple devices at once.
- Configurable COM port, baud rate, connection timeout, and test duration per device.
- Raw NMEA data logging.
- Parsing of standard NMEA sentences (e.g., GGA, RMC, GSV) into an Excel file.
- Logging output is available both in the console and in log files.
- Error handling and user-friendly logging with detailed messages.

## Requirements

- Python 3.13 or higher
- Libraries:
  - `serial`
  - `pynmea2`
  - `pandas`
  - `threading`
  - `logging`

Install the dependencies using the provided `requirements.txt` in the root directory.

## Installation

1. Clone the repository or download the project files.
   ```bash
   git clone <repo_url>
   cd nmea-parser
   ```

2. Install dependencies.
   ```bash
   pip install -r requirements.txt
   ```

3. Ensure all COM ports are available, and note the baud rate and timeout settings for the devices being tested.

## Running the Tool

Ensure to make the required "port", "baudrate", and "test duration" changes to the main.py file before running the tool. The tool can be run from the command line as follows:

```bash
python src/main.py
```

### Command Line Options

Currently, there are no command-line arguments required. The tool is configured directly in the source code. It will start reading and logging data from the configured devices.

## Configuration

### Device Configuration

The tool supports multiple devices, and you can configure each device's COM port, baud rate, timeout, and test duration in the `main.py` file. In the `devices` dictionary, update the device information:

```python
devices = {
    "device 1": {"port": "COM9", "baudrate": 115200, "timeout": 1, "duration": 10},
    "device 2": {"port": "COM7", "baudrate": 115200, "timeout": 1, "duration": 15},
    "device 3": {"port": "COM10", "baudrate": 921600, "timeout": 1, "duration": 20}
}
```

- **port**: The COM port to which the device is connected.
- **baudrate**: The baud rate for communication with the device.
- **timeout**: The time (in seconds) to wait for a response from the device before considering it unresponsive.
- **duration**: The length of time (in seconds) to collect data from the device.

### Logging Configuration

The tool uses Python’s `logging` module to log console outputs and error messages. All log files, including the raw NMEA data and parsed Excel data, are stored in the `logs/` directory. The tool creates a timestamped folder for each test run.

## Output

1. **Raw NMEA Log**: Each device will have a raw NMEA log saved as a `.txt` file in the `logs/` directory.
   - Example: `nmea_raw_log_COM9_115200_<timestamp>.txt`

2. **Parsed Data**: The parsed NMEA data for each device will be saved in an Excel file in the `logs/` directory.
   - Example: `nmea_parsed_data_COM9_115200_<timestamp>.xlsx`

3. **Console Log**: A console log file that contains all the outputs printed during the tool execution will also be saved in the `logs/` directory.
   - Example: `console_output_<timestamp>.txt`

## Error Handling

The tool provides comprehensive error handling:
- If a serial port fails to open, an error message is logged, and the tool skips that device.
- Errors during NMEA sentence parsing are caught, and the tool continues to read from the device.
- Logging errors are caught and logged, ensuring that the tool can continue running even if certain logs fail.

## Dependencies

To ensure that the tool can be run smoothly, dependencies are listed in `requirements.txt`:
```
certifi==2024.8.30
charset-normalizer==3.4.0
docker==7.1.0
et-xmlfile==1.1.0
idna==3.10
numpy==2.1.2
openpyxl==3.1.5
pandas==2.2.3
pynmea2==1.19.0
pyserial==3.5
python-dateutil==2.9.0.post0
pytz==2024.2
pywin32==307
requests==2.32.3
six==1.16.0
tzdata==2024.2
urllib3==2.2.3
```

## Future Enhancements

- Implement a GUI using Tkinter for easier interaction.
- Add more detailed test reports for parsed NMEA data.
- Add proprietary sentence parser support - PQTM and PAIR.
Here’s the updated **License** section with the MIT license information:

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.
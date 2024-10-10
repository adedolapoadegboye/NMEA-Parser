import serial
import pynmea2
import pandas as pd
import os
import sys
from datetime import datetime
import threading
from time import time
import logging

class NMEAData:
    def __init__(self, sentence_type, data, parsed_sentences):
        self.baudrate = None
        self.port = None
        self.sentence_type = sentence_type
        self.data = data
        self.parsed_sentences = parsed_sentences  # List to store parsed NMEA data

    def __str__(self):
        # Pretty print the data based on sentence type
        if self.sentence_type == "GGA":
            self.parsed_sentences.append({
                "Type": "GGA",
                "Timestamp": self.data.timestamp.replace(tzinfo=None),
                "Latitude": f"{self.data.latitude} {self.data.lat_dir}",
                "Longitude": f"{self.data.longitude} {self.data.lon_dir}",
                "GPS Quality": self.data.gps_qual,
                "Satellites": self.data.num_sats,
                "Horizontal Dilution (HDOP)": self.data.horizontal_dil,
                "Altitude": f"{self.data.altitude} {self.data.altitude_units}",
                "Geoidal Separation": f"{self.data.geo_sep} {self.data.geo_sep_units}",
                "Age of Differential GPS Data": self.data.age_gps_data,
                "Differential Reference Station ID": self.data.ref_station_id
            })
            return (
                f"GGA - Fix Data:\n"
                f"  Timestamp: {self.data.timestamp}\n"
                f"  Latitude: {self.data.latitude} {self.data.lat_dir}\n"
                f"  Longitude: {self.data.longitude} {self.data.lon_dir}\n"
                f"  GPS Quality Indicator: {self.data.gps_qual}\n"
                f"  Number of Satellites in Use: {self.data.num_sats}\n"
                f"  Horizontal Dilution of Precision (HDOP): {self.data.horizontal_dil}\n"
                f"  Antenna Altitude (Above Mean Sea Level): {self.data.altitude} {self.data.altitude_units}\n"
                f"  Geoidal Separation: {self.data.geo_sep} {self.data.geo_sep_units}\n"
                f"  Age of Differential GPS Data: {self.data.age_gps_data}\n"
                f"  Differential Reference Station ID: {self.data.ref_station_id}\n"
            )

        elif self.sentence_type == "RMC":
            self.parsed_sentences.append({
                "Type": "RMC",
                "Timestamp": self.data.timestamp.replace(tzinfo=None),
                "Status": self.data.status,
                "Latitude": f"{self.data.latitude} {self.data.lat_dir}",
                "Longitude": f"{self.data.longitude} {self.data.lon_dir}",
                "Speed Over Ground": f"{self.data.spd_over_grnd} knots",
                "Course Over Ground": self.data.true_course,
                "Date": self.data.datestamp,
                "Magnetic Variation": f"{self.data.mag_variation} {self.data.mag_var_dir}",
                "Mode Indicator": self.data.mode_indicator,
                "Navigational Status": self.data.nav_status
            })
            return (
                f"RMC - Recommended Minimum:\n"
                f"  Timestamp: {self.data.timestamp}\n"
                f"  Status: {self.data.status}\n"
                f"  Latitude: {self.data.latitude} {self.data.lat_dir}\n"
                f"  Longitude: {self.data.longitude} {self.data.lon_dir}\n"
                f"  Speed over Ground: {self.data.spd_over_grnd} knots\n"
                f"  Course over Ground: {self.data.true_course}\n"
                f"  Date: {self.data.datestamp}\n"
                f"  Magnetic Variation: {self.data.mag_variation} {self.data.mag_var_dir}\n"
                f"  Mode Indicator: {self.data.mode_indicator}\n"
                f"  Navigational Status: {self.data.nav_status}\n"
            )
        elif self.sentence_type == "GSV":
            self.parsed_sentences.append({
                "Type": "GSV",
                "Number of Messages": self.data.num_messages,
                "Message Number": self.data.msg_num,
                "Total Satellites in View": self.data.num_sv_in_view,
                "Satellite 1 PRN": f"{self.data.sv_prn_num_1}",
                "Elevation 1": f"{self.data.elevation_deg_1}°",
                "Azimuth 1": f"{self.data.azimuth_1}°",
                "SNR 1": f"{self.data.snr_1} dB",
                "Satellite 2 PRN": f"{self.data.sv_prn_num_2}",
                "Elevation 2": f"{self.data.elevation_deg_2}°",
                "Azimuth 2": f"{self.data.azimuth_2}°",
                "SNR 2": f"{self.data.snr_2} dB",
                "Satellite 3 PRN": f"{self.data.sv_prn_num_3}",
                "Elevation 3": f"{self.data.elevation_deg_3}°",
                "Azimuth 3": f"{self.data.azimuth_3}°",
                "SNR 3": f"{self.data.snr_3} dB",
                "Satellite 4 PRN": f"{self.data.sv_prn_num_4}",
                "Elevation 4": f"{self.data.elevation_deg_4}°",
                "Azimuth 4": f"{self.data.azimuth_4}°",
                "SNR 4": f"{self.data.snr_4} dB"
            })
            return (
                f"GSV - Satellites in View:\n"
                f"  Number of Messages: {self.data.num_messages}\n"
                f"  Message Number: {self.data.msg_num}\n"
                f"  Total Satellites in View: {self.data.num_sv_in_view}\n"
                f"  Satellite 1 PRN: {self.data.sv_prn_num_1} - Elevation: {self.data.elevation_deg_1}° - Azimuth: {self.data.azimuth_1}° - SNR: {self.data.snr_1} dB\n"
                f"  Satellite 2 PRN: {self.data.sv_prn_num_2} - Elevation: {self.data.elevation_deg_2}° - Azimuth: {self.data.azimuth_2}° - SNR: {self.data.snr_2} dB\n"
                f"  Satellite 3 PRN: {self.data.sv_prn_num_3} - Elevation: {self.data.elevation_deg_3}° - Azimuth: {self.data.azimuth_3}° - SNR: {self.data.snr_3} dB\n"
                f"  Satellite 4 PRN: {self.data.sv_prn_num_4} - Elevation: {self.data.elevation_deg_4}° - Azimuth: {self.data.azimuth_4}° - SNR: {self.data.snr_4} dB\n"
            )
        elif self.sentence_type == "GSA":
            self.parsed_sentences.append({
                "Type": "GSA",
                "Mode": self.data.mode,
                "Mode Fix Type": self.data.mode_fix_type,
                "Satellites Used": f"{', '.join(filter(None, [self.data.sv_id01, self.data.sv_id02, self.data.sv_id03, self.data.sv_id04, self.data.sv_id05, self.data.sv_id06, self.data.sv_id07, self.data.sv_id08, self.data.sv_id09, self.data.sv_id10, self.data.sv_id11, self.data.sv_id12]))}",
                "PDOP": self.data.pdop,
                "HDOP": self.data.hdop,
                "VDOP": self.data.vdop
            })
            return (
                f"GSA - Satellite Info:\n"
                f"  Mode: {self.data.mode}\n"
                f"  Mode Fix Type: {self.data.mode_fix_type}\n"
                f"  Satellites Used: {', '.join(filter(None, [self.data.sv_id01, self.data.sv_id02, self.data.sv_id03, self.data.sv_id04, self.data.sv_id05, self.data.sv_id06, self.data.sv_id07, self.data.sv_id08, self.data.sv_id09, self.data.sv_id10, self.data.sv_id11, self.data.sv_id12]))}\n"
                f"  PDOP: {self.data.pdop}\n"
                f"  HDOP: {self.data.hdop}\n"
                f"  VDOP: {self.data.vdop}\n"
            )
        elif self.sentence_type == "VTG":
            self.parsed_sentences.append({
                "Type": "VTG",
                "True Track": f"{self.data.true_track}°",
                "Magnetic Track": f"{self.data.mag_track}°",
                "Speed over Ground": f"{self.data.spd_over_grnd_kts} knots / {self.data.spd_over_grnd_kmph} km/h",
                "FAA Mode": self.data.faa_mode
            })
            return (
                f"VTG - Course over Ground and Ground Speed:\n"
                f"  True Track: {self.data.true_track}° {self.data.true_track_sym}\n"
                f"  Magnetic Track: {self.data.mag_track}° {self.data.mag_track_sym}\n"
                f"  Speed over Ground: {self.data.spd_over_grnd_kts} knots / {self.data.spd_over_grnd_kmph} km/h\n"
                f"  FAA Mode: {self.data.faa_mode}\n"
            )
        elif self.sentence_type == "GLL":
            self.parsed_sentences.append({
                "Type": "GLL",
                "Latitude": f"{self.data.latitude} {self.data.lat_dir}",
                "Longitude": f"{self.data.longitude} {self.data.lon_dir}",
                "Timestamp": self.data.timestamp.replace(tzinfo=None),
                "Status": self.data.status,
                "FAA Mode": self.data.faa_mode
            })
            return (
                f"GLL - Geographic Position:\n"
                f"  Latitude: {self.data.latitude} {self.data.lat_dir}\n"
                f"  Longitude: {self.data.longitude} {self.data.lon_dir}\n"
                f"  Timestamp: {self.data.timestamp}\n"
                f"  Status: {self.data.status}\n"
                f"  FAA Mode: {self.data.faa_mode}\n"
            )
        elif self.sentence_type == "ZDA":
            self.parsed_sentences.append({
                "Type": "ZDA",
                "UTC Time": self.data.timestamp,
                "Day": self.data.day,
                "Month": self.data.month,
                "Year": self.data.year,
                "Local Zone Hours": self.data.local_zone,
                "Local Zone Minutes": self.data.local_zone_minutes
            })
            return (
                f"ZDA - Time and Date:\n"
                f"  UTC Time: {self.data.timestamp}\n"
                f"  Day: {self.data.day}\n"
                f"  Month: {self.data.month}\n"
                f"  Year: {self.data.year}\n"
                f"  Local Zone Hours: {self.data.local_zone}\n"
                f"  Local Zone Minutes: {self.data.local_zone_minutes}\n"
            )
        elif self.sentence_type == "GNS":
            self.parsed_sentences.append({
                "Type": "GNS",
                "Timestamp": self.data.timestamp.replace(tzinfo=None),
                "Latitude": f"{self.data.latitude} {self.data.lat_dir}",
                "Longitude": f"{self.data.longitude} {self.data.lon_dir}",
                "Mode Indicator": self.data.mode_indicator,
                "Number of Satellites": self.data.num_sats,
                "HDOP": self.data.hdop,
                "Altitude": self.data.altitude,
                "Geoidal Separation": self.data.geo_sep,
                "Age of Differential Data": self.data.age_gps_data,
                "Differential Reference Station ID": self.data.differential
            })
            return (
                f"GNS - GNSS Fix Data:\n"
                f"  Timestamp: {self.data.timestamp}\n"
                f"  Latitude: {self.data.latitude} {self.data.lat_dir}\n"
                f"  Longitude: {self.data.longitude} {self.data.lon_dir}\n"
                f"  Mode Indicator: {self.data.mode_indicator}\n"
                f"  Number of Satellites in Use: {self.data.num_sats}\n"
                f"  HDOP: {self.data.hdop}\n"
                f"  Altitude: {self.data.altitude}\n"
                f"  Geoidal Separation: {self.data.geo_sep}\n"
                f"  Age of Differential Data: {self.data.age_gps_data}\n"
                f"  Differential Reference Station ID: {self.data.differential}\n"
            )
        elif self.sentence_type == "GST":
            self.parsed_sentences.append({
                "Type": "GST",
                "UTC Time": self.data.timestamp,
                "RMS Deviation": self.data.rms,
                "Major Axis Error": self.data.std_dev_major,
                "Minor Axis Error": self.data.std_dev_minor,
                "Orientation of Major Axis": self.data.orientation,
                "Latitude Error": self.data.std_dev_latitude,
                "Longitude Error": self.data.std_dev_longitude,
                "Altitude Error": self.data.std_dev_altitude
            })
            return (
                f"GST - Pseudorange Error Statistics:\n"
                f"  UTC Time: {self.data.timestamp}\n"
                f"  RMS Deviation: {self.data.rms}\n"
                f"  Major Axis Error: {self.data.std_dev_major}\n"
                f"  Minor Axis Error: {self.data.std_dev_minor}\n"
                f"  Orientation of Major Axis: {self.data.orientation}\n"
                f"  Latitude Error (std dev): {self.data.std_dev_latitude}\n"
                f"  Longitude Error (std dev): {self.data.std_dev_longitude}\n"
                f"  Altitude Error (std dev): {self.data.std_dev_altitude}\n"
            )
        elif self.sentence_type == "GRS":
            self.parsed_sentences.append({
                "Type": "GRS",
                "Timestamp": self.data.timestamp.replace(tzinfo=None),
                "Residual Mode": self.data.residuals_mode,
                "Residuals": [self.data.sv_res_01, self.data.sv_res_02, self.data.sv_res_03, self.data.sv_res_04,
                              self.data.sv_res_05, self.data.sv_res_06, self.data.sv_res_07, self.data.sv_res_08,
                              self.data.sv_res_09, self.data.sv_res_10, self.data.sv_res_11, self.data.sv_res_12]
            })
            return (
                f"GRS - GNSS Range Residuals:\n"
                f"  Timestamp: {self.data.timestamp}\n"
                f"  Residual Mode: {self.data.residuals_mode}\n"
                f"  Residuals: {[self.data.sv_res_01, self.data.sv_res_02, self.data.sv_res_03, self.data.sv_res_04, self.data.sv_res_05, self.data.sv_res_06, self.data.sv_res_07, self.data.sv_res_08, self.data.sv_res_09, self.data.sv_res_10, self.data.sv_res_11, self.data.sv_res_12]}\n"
            )
        elif self.sentence_type == "RLM":
            self.parsed_sentences.append({
                "Type": "RLM",
                "Beacon ID": self.data.beacon_id,
                "Message Code": self.data.message_code
            })
            return (
                f"RLM - Return Link Message:\n"
                f"  Beacon ID: {self.data.beacon_id}\n"
                f"  Message Code: {self.data.message_code}\n"
            )
        else:
            return f"Unsupported NMEA sentence type: {self.sentence_type}"

    def write_to_excel(self, port, baudrate, filename="nmea_data"):
        try:
            for entry in self.parsed_sentences:
                if 'Timestamp' in entry and isinstance(entry['Timestamp'], pd.Timestamp):
                    entry['Timestamp'] = entry['Timestamp'].tz_localize(None)

            df = pd.DataFrame(self.parsed_sentences)
            df.to_excel(f"logs/NMEA_{timestamp}/{filename}_{port}_{baudrate}_{timestamp}.xlsx", index=False)
            logging.info(f"Data written to logs/NMEA_{timestamp}/{filename}_{port}_{baudrate}_{timestamp}.xlsx")
        except Exception as e:
            logging.error(f"Error writing to Excel file: {e}")

def read_nmea_data(port, baudrate, timeout, duration, log_folder):
    parsed_sentences = []  # Initialize a list to store NMEA data
    start_time = time()  # Record the start time

    if not os.path.exists("logs"):
        os.makedirs("logs")

    if not os.path.exists(f"logs/NMEA_{timestamp}"):
        os.makedirs(f"logs/NMEA_{timestamp}")

    raw_nmea_log = open(f"logs/NMEA_{timestamp}/nmea_raw_log_{port}_{baudrate}_{timestamp}.txt", "a", encoding="utf-8")
    parsed_nmea_data = NMEAData(None, None, parsed_sentences)  # Placeholder NMEAData object

    try:
        # Configure the serial port
        try:
            ser = serial.Serial(
                port=port,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=timeout
            )
            logging.info(f"Connected to serial port {port} with baudrate {baudrate}.")
        except serial.SerialException as e:
            logging.error(f"Error opening serial port {port}: {e}")
            return  # Exit function if the port cannot be opened

        # Continuously read from the serial port
        while time() - start_time < duration:
            try:
                # Read a line of NMEA data from the serial port
                nmea_sentence = ser.readline().decode('ascii', errors='replace').strip()

                try:
                    raw_nmea_log.write(nmea_sentence + "\n")
                except Exception as e:
                    logging.error(f"Error writing NMEA sentence to log file: {e}")

                # Skip proprietary sentences like $PAIR or $PQTM
                if nmea_sentence.startswith('$P'):
                    logging.info(f"Proprietary sentence ignored: {nmea_sentence}")
                    continue

                if nmea_sentence.startswith('$G'):
                    logging.info(f"Received Standard NMEA Message: {nmea_sentence}")

                    # Parse the NMEA sentence using pynmea2
                    try:
                        msg = pynmea2.parse(nmea_sentence)

                        # Create a NMEAData object and print it nicely
                        nmea_data = NMEAData(msg.sentence_type, msg, parsed_sentences)
                        logging.info(nmea_data)

                    except pynmea2.ParseError as e:
                        logging.info(f"Failed to parse NMEA sentence: {nmea_sentence} - {e}")

            except serial.SerialException as e:
                logging.info(f"Error reading from serial port: {e}")
                break

    except serial.SerialException as e:
        logging.info(f"Error opening serial port: {e}")

    # After reading all raw sentences, write to the log file created
    logging.info(f"Standard and Proprietary logs written to {raw_nmea_log.name} for port {port} with baudrate {baudrate}")
    raw_nmea_log.close()

    # After reading data, write it to an Excel file
    parsed_nmea_data.write_to_excel(port, baudrate)

if __name__ == "__main__":
    try:
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S%f')
        log_folder = f"logs/NMEA_{timestamp}"

        # Create the log folder for each test run
        os.makedirs(log_folder, exist_ok=True)

        # Configure logging to both file and console
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s [%(levelname)s] %(message)s',
            handlers=[
                logging.FileHandler(f"{log_folder}/console_output_{timestamp}.txt", 'a', encoding='utf-8'),
                logging.StreamHandler(sys.stdout)
            ]
        )

        devices = {
            "device 1": {"port": "COM9", "baudrate": 115200, "timeout": 1, "duration": 10},
            "device 2": {"port": "COM7", "baudrate": 115200, "timeout": 1, "duration": 15},
            "device 3": {"port": "COM10", "baudrate": 921600, "timeout": 1, "duration": 20}
        }

        threads = []

        for device, config in devices.items():
            try:
                thread = threading.Thread(target=read_nmea_data, args=(config["port"], config["baudrate"], config["timeout"], config["duration"], log_folder))
                threads.append(thread)
                thread.start()
            except Exception as e:
                logging.error(f"Failed to start thread for {device}: {e}")

        for thread in threads:
            try:
                thread.join()
            except Exception as e:
                logging.error(f"Error while waiting for thread to finish: {e}")

    except Exception as e:
        logging.error(f"An unexpected error occurred: {e}")

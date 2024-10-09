import serial
import pynmea2


class NMEAData:
    def __init__(self, sentence_type, data):
        self.sentence_type = sentence_type
        self.data = data

    def __str__(self):
        # Pretty print the data based on sentence type
        if self.sentence_type == "GGA":
            return (
                f"GGA - Fix Data:\n"
                f"  Timestamp: {self.data.timestamp}\n"
                f"  Latitude: {self.data.latitude} {self.data.lat_dir}\n"
                f"  Longitude: {self.data.longitude} {self.data.lon_dir}\n"
                f"  GPS Quality: {self.data.gps_qual}\n"
                f"  Number of Satellites: {self.data.num_sats}\n"
                f"  Altitude: {self.data.altitude} {self.data.altitude_units}\n"
            )
        elif self.sentence_type == "RMC":
            return (
                f"RMC - Recommended Minimum:\n"
                f"  Timestamp: {self.data.timestamp}\n"
                f"  Status: {self.data.status}\n"
                f"  Latitude: {self.data.latitude} {self.data.lat_dir}\n"
                f"  Longitude: {self.data.longitude} {self.data.lon_dir}\n"
                f"  Speed over Ground: {self.data.spd_over_grnd} knots\n"
                f"  Course over Ground: {self.data.true_course}\n"
                f"  Date: {self.data.datestamp}\n"
            )
        elif self.sentence_type == "GSV":
            return (
                f"GSV - Satellites in View:\n"
                f"  Total Satellites in View: {self.data.num_sv_in_view}\n"
                f"  Satellite PRNs: {self.data.sv_prn_01}, {self.data.sv_prn_02}, {self.data.sv_prn_03}, {self.data.sv_prn_04}\n"
            )
        elif self.sentence_type == "GSA":
            return (
                f"GSA - Satellite Info:\n"
                f"  Mode: {self.data.mode_fix_type}\n"
                f"  Satellites Used: {self.data.sv_id01}, {self.data.sv_id02}, {self.data.sv_id03}\n"
                f"  PDOP: {self.data.pdop}\n"
                f"  HDOP: {self.data.hdop}\n"
                f"  VDOP: {self.data.vdop}\n"
            )
        elif self.sentence_type == "VTG":
            return (
                f"VTG - Course over Ground and Ground Speed:\n"
                f"  True Track: {self.data.true_track}°\n"
                f"  Speed (knots): {self.data.spd_over_grnd_kts}\n"
                f"  Speed (km/h): {self.data.spd_over_grnd_kmph}\n"
            )
        elif self.sentence_type == "GLL":
            return (
                f"GLL - Geographic Position:\n"
                f"  Latitude: {self.data.latitude} {self.data.lat_dir}\n"
                f"  Longitude: {self.data.longitude} {self.data.lon_dir}\n"
                f"  Timestamp: {self.data.timestamp}\n"
                f"  Status: {self.data.status}\n"
            )
        elif self.sentence_type == "ZDA":
            return (
                f"ZDA - Time and Date:\n"
                f"  UTC: {self.data.timestamp}\n"
                f"  Day: {self.data.day}\n"
                f"  Month: {self.data.month}\n"
                f"  Year: {self.data.year}\n"
            )
        elif self.sentence_type == "GNS":
            return (
                f"GNS - GNSS Fix Data:\n"
                f"  Latitude: {self.data.latitude} {self.data.lat_dir}\n"
                f"  Longitude: {self.data.longitude} {self.data.lon_dir}\n"
                f"  Altitude: {self.data.altitude}\n"
                f"  Number of Satellites: {self.data.num_sats}\n"
            )
        elif self.sentence_type == "GST":
            return (
                f"GST - Pseudorange Error Statistics:\n"
                f"  RMS Deviation: {self.data.rms_deviation}\n"
                f"  Major Axis Error: {self.data.major_dev}\n"
                f"  Minor Axis Error: {self.data.minor_dev}\n"
            )
        elif self.sentence_type == "GRS":
            return (
                f"GRS - GNSS Range Residuals:\n"
                f"  Residuals: {self.data.residuals}\n"
            )
        elif self.sentence_type == "RLM":
            return (
                f"RLM - Return Link Message:\n"
                f"  Beacon ID: {self.data.beacon_id}\n"
                f"  Message Code: {self.data.message_code}\n"
            )
        else:
            return f"Unsupported NMEA sentence type: {self.sentence_type}"


def read_nmea_data():
    try:
        # Configure the serial port
        ser = serial.Serial(
            port='COM9',  # Your serial port, e.g., COM9 for Windows
            baudrate=115200,  # Baud rate
            bytesize=serial.EIGHTBITS,  # 8 data bits
            parity=serial.PARITY_NONE,  # No parity
            stopbits=serial.STOPBITS_ONE,  # 1 stop bit
            timeout=1  # Timeout in seconds
        )

        print("Connected to serial port. Reading data...")

        # Continuously read from the serial port
        while True:
            try:
                # Read a line of NMEA data from the serial port
                nmea_sentence = ser.readline().decode('ascii', errors='replace').strip()

                if nmea_sentence.startswith('$'):
                    print(f"Received: {nmea_sentence}")

                    # Parse the NMEA sentence using pynmea2
                    try:
                        msg = pynmea2.parse(nmea_sentence)

                        # Create a NMEAData object and print it nicely
                        nmea_data = NMEAData(msg.sentence_type, msg)
                        print(nmea_data)

                    except pynmea2.ParseError as e:
                        print(f"Failed to parse NMEA sentence: {nmea_sentence} - {e}")
                else:
                    print("No valid NMEA sentence received.")

            except serial.SerialException as e:
                print(f"Error reading from serial port: {e}")
                break

    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")


if __name__ == "__main__":
    read_nmea_data()

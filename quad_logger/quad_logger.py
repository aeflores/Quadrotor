import serial
import argparse
import datetime
from pathlib import Path

def main(port,filename):
    port = serial.Serial(port=port,baudrate=115200,timeout=1)
    with open(filename,mode="w") as logfile:
        print("\t".join(["Time","Power","State","Yaw","Pitch",
                        "Roll","Height","Error_roll","Error_pitch",
                        "Eng1","Eng2","Eng3","Eng4","Delta_T"]),file=logfile)
        while True:
            try:
                line= port.readline().decode("ascii")
            except serial.SerialException as e:
                print("Received exception. Aborting...")
                break
            time = datetime.datetime.now()
            timestr= f"{time.time()}"
            print(f"{timestr} {line}",end="")
            fields = line.split(" ")
            selected_fields = [fields[i] for i in range(1,len(fields),2)]
            # Only record non-error states
            if len(selected_fields)>9:
                print("\t".join([timestr]+selected_fields),file=logfile,end="")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Record Quadcopter serial log')
    parser.add_argument('--port',default="/dev/ttyUSB0",
                        help='The serial port (default /dev/tty/USB0)')
    parser.add_argument('file', type=Path,help="The file where to store the results")
    args = parser.parse_args()
    main(args.port, args.file)
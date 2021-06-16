import serial
import threading
import argparse
import datetime
from pathlib import Path

import numpy as np
from matplotlib.lines import Line2D
import matplotlib.pyplot as plt
import matplotlib.animation as animation

class UpdateInfo:
    def __init__(self, column_names):
        self.changed = True
        self.last_value_map = {key:"0" for key in column_names}

class Scope:
    def __init__(self, ax, columns,ylim):
        colors = ['b','g','r','c','m','y','k']
        self.ax = ax
        self.tdata = [0]
        self.column_data = {}
        self.column_lines = {}
        for i, column in enumerate(columns):
            self.column_data[column] =[0]
            self.column_lines[column] = Line2D(self.tdata, self.column_data[column],color=colors[i])
            self.ax.add_line(self.column_lines[column])
        self.ax.set_ylim(ylim[0], ylim[1])
        self.ax.set_xlim(0, 60)
        self.ax.legend(self.column_lines.values(),self.column_lines.keys())

    def update(self, update_info:UpdateInfo):
        if not update_info.changed:
            return self.column_lines.values()
        update_info.changed = False
        t = float(update_info.last_value_map["Time"].split(":")[-1])
        lastt = self.tdata[-1]
        # new minute, restart plot
        if lastt > t:
            self.tdata = []
            for column in self.column_data:
                self.column_data[column]=[]
            self.ax.figure.canvas.draw()

        self.tdata.append(t)
        for column in self.column_data:
            self.column_data[column].append(float(update_info.last_value_map[column]))
            self.column_lines[column].set_data(self.tdata, self.column_data[column])

        return self.column_lines.values()


def main(port,filename):
    port = serial.Serial(port=port,baudrate=115200,timeout=1)
    fig, ax = plt.subplots(nrows=3,sharex=True)
    yaw_pitch_roll_scope = Scope(ax[0],columns=["Yaw","Pitch","Roll"],ylim=(-180,180))
    error_scope = Scope(ax[1],columns=["Error_roll","Error_pitch"],ylim=(-10,10))
    power_scope = Scope(ax[2],columns=["Power","Eng1","Eng2","Eng3","Eng4"],ylim=(0,1000))


    column_names= ["Time","Power","State","Yaw","Pitch",
                        "Roll","Height","Error_roll","Error_pitch",
                        "Eng1","Eng2","Eng3","Eng4","Delta_T"]

    update_info = UpdateInfo(column_names)
    def emitter():
        while True:
            yield update_info

    yaw_pitch_roll_animation = animation.FuncAnimation(fig, yaw_pitch_roll_scope.update, emitter, interval=40, blit=True)
    error_animation = animation.FuncAnimation(fig, error_scope.update, emitter, interval=40, blit=True)
    power_animation = animation.FuncAnimation(fig, power_scope.update, emitter, interval=40, blit=True)
    window_closed = False
    def reader():
        with open(filename,mode="w") as logfile:
            print("\t".join(column_names),file=logfile)
            while True:
                if window_closed:
                    break
                try:
                    line= port.readline().decode("ascii")
                except serial.SerialException as e:
                    print("Received exception. Aborting...")
                    break
                except UnicodeDecodeError as _:
                    print("Error decoding")
                    continue
                time = datetime.datetime.now()
                timestr= str(f"{time.time()}")
                print(f"{timestr} {line}",end="")
                fields = line.split(" ")
                selected_fields = [timestr]+[fields[i] for i in range(1,len(fields),2)]
                # Only record non-error states
                if len(selected_fields)>9:
                    update_info.changed = True
                    update_info.last_value_map = {key:val for key, val in zip(column_names,selected_fields)}
                    print("\t".join(selected_fields),file=logfile,end="")
    reader_thread = threading.Thread(target=reader)
    reader_thread.start()
    #yaw_pitch_roll_animation.save("video.mp4")
    #error_animation.save("video2.mp4")
    plt.show()
    window_closed = True
    reader_thread.join()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Record Quadcopter serial log')
    parser.add_argument('--port',default="/dev/ttyUSB0",
                        help='The serial port (default /dev/tty/USB0)')
    parser.add_argument('file', type=Path,help="The file where to store the results")
    args = parser.parse_args()
    main(args.port, args.file)
from textwrap import wrap
from pandas.core.frame import DataFrame
from pyglet.window import key
import pyglet
import pandas as pd
import numpy as np
import argparse
from pathlib import Path

vidPath = ""

window= pyglet.window.Window(width=1800,height=1000)
player = pyglet.media.Player()
source = pyglet.media.StreamingSource()
label = pyglet.text.Label("",
                          font_name='Times New Roman',
                          font_size=12,
                          x=window.width//6, y=window.height//2,
                          anchor_x='center', anchor_y='center',multiline=True, width=10)
data = None                        
time_offset = 0.0
step = 1/10

def get_now_row(data: pd.DataFrame,time):
    return data[data["time"]>time].iloc[0]

def estimate_time(data:pd.DataFrame):
    #data["time"] = data["dt"]
    data.insert(0,"time",data["dt"])
    time =0
    for i,elem in enumerate(data["dt"]):
        # radio transmite cada 4 ciclos
        time += data["dt"][i]*4/1000000
        data["time"][i] = time
    return data

def main():
    global data
    global time_offset
    parser=argparse.ArgumentParser("prog")
    parser.add_argument("video",type=Path)
    parser.add_argument("log",type=Path)
    parser.add_argument("--offset",type=float,default=16.47)
    args= parser.parse_args()
    data= pd.read_csv(args.log,sep="\t")
    time_offset = args.offset
    
    MediaLoad = pyglet.media.load(str(args.video))
    player.queue(MediaLoad)
    player.seek_next_frame()

    data.columns=["state","power","dt","yawrate", "pitch", "roll", "height", "Epitch", "Eroll","Eng1","Eng2","Eng3","Eng4","Empty"]
    data=data.drop(columns="Empty")
    data= estimate_time(data)
    #print(data[data["Eng1"]>1000])

@window.event
def on_draw():
    #player.seek_next_frame()
    if player.source and player.source.video_format:
        player.get_texture().blit(0,0)
    real_time= player.time + time_offset
    row = get_now_row(data,real_time)
    label.text=f"time {player.time}\n"+ str(row)
    label.draw()

@window.event
def on_key_press(symbol, modifiers):
    global step
    if symbol == key.RIGHT:
        player.seek(player.time+step)

    if symbol == key.LEFT:
        player.seek(player.time-step)
    if symbol == key.UP:
        step*=2

    if symbol == key.DOWN:
        step*=0.5
 
main()
pyglet.app.run()
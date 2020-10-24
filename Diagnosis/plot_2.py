
import argparse
import matplotlib
import matplotlib.pyplot as plt
import pandas as pd
from pathlib import Path

def load_exps_file(file):
    data = pd.read_csv(file,sep="\t",names=['Power','Mode','Yaw','Pitch','Roll','Error pitch','Error roll','Eng1','Eng2','Eng3','Eng4','Deta T']) 
    data=data.replace('(\w| )*=','',regex=True)
    data= data.drop(['Mode'],axis=1)
    data = data.apply(pd.to_numeric)
    return data
def plot_exps(file):
    data = load_exps_file(file)
    print(data)
    fig= plt.figure()
    gs = matplotlib.gridspec.GridSpec(2, 1, figure=fig)
    ax=fig.add_subplot(gs[0,0])
    data[["Eng1","Eng2","Eng3","Eng4"]].plot(ax=ax)
    ax.grid(b=True)
    ax.set_ylim(1250,1550)
    ax=fig.add_subplot(gs[1,0])
    data[["Error pitch","Error roll"]].plot(ax=ax)
    ax.grid(b=True)
    plt.show()

root = 'C:/Users/Enrique Flores/Documents/GitHub/quadrotor-arduino/logs/Flighttest_2020_10_18_001.txt'
plot_exps(root)
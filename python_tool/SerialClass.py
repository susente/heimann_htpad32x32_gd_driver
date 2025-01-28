'''
@ author: summer
@ tools: pycharm 
@ content: 串口通讯实现类
@ date: 2020.2.12
'''
import serial
import serial.tools.list_ports

import time
from array import array
import numpy as np
import cv2
import matplotlib.pyplot as plt
from matplotlib import animation
#import mayavi.mlab as mlab
#import  moviepy.editor as mpy
from pyheatmap.heatmap import HeatMap
import seaborn as sns
#from pyecharts import HeatMap
import random
import csv
import pandas as pd

class SerialAchieve:
    def __init__(self,band=115200,check="无校验位",data=8,stop=1):
        self.port = None
        # 获取可用串口
        self.port_list = list(serial.tools.list_ports.comports())
        assert (len(self.port_list) != 0),"无可用串口"

        self.bandRate = band
        self.checkbit = check
        self.databit = data
        self.stopbit = stop

        # 读写的数据
        self.read_data = None
        self.write_data = None

        pass
    def show_port(self):
        for i in range(0,len(self.port_list)):
            print(self.port_list[i])

    def show_other(self):
        print("波特率："+self.bandRate)
        print("校验位：" + self.checkbit)
        print("数据位：" + self.databit)
        print("停止位：" + self.stopbit)
    # 返回串口
    def get_port(self):
        return self.port_list
    # 打开串口
    def open_port(self,port):
        self.port = serial.Serial(port, self.bandRate,timeout = None)

    def delete_port(self):
        if self.port != None:
            self.port.close()
            print("关闭串口完成")
        else:
            pass

    def Read_data(self):   # self.port.read(self.port.in_waiting) 表示全部接收串口中的数据
        fig, ax = plt.subplots()

        for t in range(200):
            ax.cla()
            data = [0 for _ in range(32)]
            self.port.write('s'.encode('utf-8'))
            for i in range(0,32,1):
                data[i]=self.port.read(128)
            #print(data)

            heimann = [[0 for _ in range(32)] for _ in range(32)]
        
            for i in range(0,32,1):
                for j in range(0,32,1):
                    heimann[i][j] = data[i][j*4+3]<<24 | data[i][j*4+2]<<16 | data[i][j*4+1]<<8 | data[i][j*4]       
            
            ax.imshow(heimann)
            ax.set_title("frame {}".format(t))
            plt.pause(0.1)

    def Write_data(self,data):
        if self.port.isOpen() == False:
            print("串口打开错误")
        else:
            self.port.write(data.encode("utf-8"))  # 返回的是写入的字节数

if __name__ == '__main__':
    myser = SerialAchieve()
    myser.open_port("COM7")
    myser.delete_port()
    myser.show_port()




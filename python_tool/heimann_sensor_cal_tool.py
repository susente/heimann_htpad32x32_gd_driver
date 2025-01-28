# encoding=utf-8
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

if __name__ == '__main__':

    #com = serial.Serial('COM11', 115200)
    com = serial.Serial('COM11', baudrate=115200,bytesize=8,parity="N", stopbits=1)
    if com.isOpen():
        print("open success")
    else:
        print("open failed")
    com.write('s'.encode('utf-8'))
    print("susente ####1")

    #N = 50
    #M = np.random.random((50, 4,4))

    #Desired labels for heatmap--not sure where to put.
    #labels=['a','b','c','d']
    
    '''a = [[0 for _ in range(32)] for _ in range(32)]
    for i in range(0,32,1):
        for j in range(0,32,1):
            a[i][j] = int(np.random.rand(1)*1024)'''

    fig, ax = plt.subplots()

    for t in range(200):
        ax.cla()
        
        #data = [[0 for _ in range(32)] for _ in range(128)]
        data = [0 for _ in range(32)]
        com.write('s'.encode('utf-8'))
        for i in range(0,32,1):
            data[i]=com.read(128)
        #print(data)
        #print("data:%#x"%(data))
        heimann = [[0 for _ in range(32)] for _ in range(32)]
    
        index = 0;
        for i in range(0,32,1):
            for j in range(0,32,1):
                heimann[i][j] = data[i][j*4+3]<<24 | data[i][j*4+2]<<16 | data[i][j*4+1]<<8 | data[i][j*4]
                #print("index: %#d %#x %#x %#x %#x %#x"%(index,heimann[i][j],data[i][j*4+3]<<24,data[i][j*4+2]<<16,data[i][j*4+1]<<8,data[i][j*4]))
                #index +=1


        '''n=[["路飞","男",100],["索隆","男",99],["娜美","女",90]]
        b=["姓名","性别","分数"]
        with open("data.csv",'w',newline='') as t:#numline是来控制空的行数的
        writer=csv.writer(t)#这一步是创建一个csv的写入器
        writer.writerow(b)#写入标签
        writer.writerows(heimann)#写入样本数据'''
        
        #dataframe = pd.DataFrame(heimann)
        #dataframe.to_csv("test.csv",index=False,sep=',')
        
        
        ax.imshow(heimann)
        ax.set_title("frame {}".format(t))
        plt.pause(0.1)
        
        
        '''sns.set()
        sns.heatmap(heimann, annot=True, fmt="x")
        plt.show() 
        plt.pause(10)'''
    #fig = plt.figure()
    

    #plt.show() 
    
    #sns.heatmap(a, annot=True, fmt="x")


    # 生成数据
    '''a = [[0 for _ in range(32)] for _ in range(32)]
    for i in range(0,32,1):
        for j in range(0,32,1):
            a[i][j] = int(np.random.rand(1)*1024)

    sns.set()

    sns.heatmap(a)
    #fig = plt.figure()
    

    #plt.show() 
    
    #sns.heatmap(a, annot=True, fmt="x")
    plt.show()    '''  


           

    '''
    N = 32
    X = np.random.rand(N) * 255  # [0, 255]
    Y = np.random.rand(N) * 255
    data = []
    for i in range(N):
        for j in range(N):
            tmp = [int(a[i][j]), int(a[i][j]), 1]
            data.append(tmp)
    heat = HeatMap(data)
    heat.clickmap(save_as="1.png") #点击图
    heat.heatmap(save_as="2.png") #热图'''


    


    '''plt.style.use('seaborn-pastel')#   修改图标样式，可以使用print(plt.style.available) 打印样式列表
    fig = plt.figure()  #创建图像fig
    ax = fig.add_subplot(1, 1, 1) #增加1x1子图

    #创建两个列表对应折线图的x、y轴数据
    xs = []
    ys = []
    
    

    #定义动画函数
    def animate(i, xs, ys):         
        # 将x、y轴坐标数据添加到xs、ys列表中
        xs.append(dt.datetime.now().strftime('%H:%M:%S'))#x轴显示时间戳，时分秒
        ys.append(temp_c)#y轴显示温度值

        # 限定xs和ys列表数据范围
        xs = xs[-32:]
        ys = ys[-32:]
        print("11111111111111")
        # 根据xs,ys绘制折线
        ax.clear()
        ax.plot(xs, ys)

        # 图像格式
        #plt.xticks(rotation=45, ha='right')#坐标数值倾斜45°，数据沿x轴正无穷方向偏移
        plt.subplots_adjust(bottom=0.30)#限制图标的区域边界
        plt.title('temperature monitoring')#图标标题
        plt.ylabel('temperature ')#y轴坐标

        #调用animation方法，对象：画布fig,动画函数：animate，函数调用数值：(xs, ys)，数据更新频率interval=1000 ms
    ani = FuncAnimation(fig, animate, fargs=(xs, ys), interval=1000)
    #画布显示
    plt.show()
    
    print("222")

    #print(a)
    #pyplot.imshow(a)'''


    '''# initialize water image
    height = 500
    width = 700
    water_depth = np.zeros((height, width), dtype=float)

    # initialize video writer
    fourcc = cv2.VideoWriter_fourcc('M','J','P','G')
    fps = 30
    video_filename = 'output.avi'
    out = cv2.VideoWriter(video_filename, fourcc, fps, (width, height))

    # new frame after each addition of water
    for i in range(10):
        random_locations = np.random.random_integers(200,450, size=(200, 2))
        for item in random_locations:
            water_depth[item[0], item[1]] += 0.1
            #add this array to the video
            gray = cv2.normalize(water_depth, None, 255, 0, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            gray_3c = cv2.merge([gray, gray, gray])
            out.write(gray_3c)

    # close out the video writer
    out.release()  '''

    
    
  
            
            
            
     
            
            
            
            
            
            
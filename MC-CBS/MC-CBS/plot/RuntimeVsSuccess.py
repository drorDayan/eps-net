import csv
import numpy as np
import matplotlib.mlab as mlab
import matplotlib.pyplot as plt

delta_time=1

filenames = ['D:\\ECBS_different_size\\outputs\\lak503dmap-5agents-size3to5-',
             'D:\\ECBS_different_size\\outputs\\lak503dmap-10agents-size3to5-',
             'D:\\ECBS_different_size\\outputs\\lak503dmap-15agents-size3to5-',
             'D:\\ECBS_different_size\\outputs\\lak503dmap-20agents-size3to5-',
             'D:\\ECBS_different_size\\outputs\\lak503dmap-25agents-size3to5-']

filenames = ['D:\\ECBS_different_size\\outputs\\10obs-20x20map-8agents-size3-',
              'D:\\ECBS_different_size\\outputs\\10obs-20x20map-7agents-size3-',
              'D:\\ECBS_different_size\\outputs\\10obs-20x20map-6agents-size3-',
              'D:\\ECBS_different_size\\outputs\\10obs-20x20map-5agents-size3-',
              'D:\\ECBS_different_size\\outputs\\10obs-20x20map-4agents-size3-',
              'D:\\ECBS_different_size\\outputs\\10obs-20x20map-3agents-size3-',
              'D:\\ECBS_different_size\\outputs\\10obs-20x20map-2agents-size3-']





InsNum = len(filenames)*50
myfont = 20
plt.xscale('log')
plt.xlabel('Runtime Limit (s)',{'size':myfont})
plt.ylabel('Success Rate',{'size':myfont})
plt.tick_params(labelsize=myfont)
plt.ylim(-0.05,1.05)
plt.xticks([10,100,1000,10000,100000], ('0.01','0.1','1', '10', '100'))
plt.xlim(1,330000)
fig = plt.gcf()
fig.set_size_inches(5,5)
fig.subplots_adjust(left=0.18, bottom=0.15, right=0.99, top=0.99, wspace=None, hspace=None)
#EPEA
data=[]
for filename in filenames:
    with open(filename + 'EPEA.csv','r') as csvfile:
        for line in csvfile.readlines():
            array=line.split(",")
            data.append(int(array[0]))

stat=[]
for i in range(0,300000,delta_time):
    count = 0
    for n in data:
        if n <= i:
            count = count + 1
    stat.append(count / InsNum)
plt.plot(range(0,300000,delta_time), stat,label='EPEA*')



#ICBS
data=[]
for filename in filenames:
    with open(filename + 'ICBS.csv','r') as csvfile:
        for line in csvfile.readlines():
            array=line.split(",")
            data.append(int(array[0]))

stat=[]
for i in range(0,300000,delta_time):
    count = 0
    for n in data:
        if n <= i:
            count = count + 1
    stat.append(count / InsNum)
plt.plot(range(0,300000,delta_time), stat,label='CBS')

#SYM
data=[]
for filename in filenames:
    with open(filename + 'SYM.csv','r') as csvfile:
        for line in csvfile.readlines():
            array=line.split(",")
            data.append(int(array[0]))

stat=[]
for i in range(0,300000,delta_time):
    count = 0
    for n in data:
        if n <= i:
            count = count + 1
    stat.append(count / InsNum)
plt.plot(range(0,300000,delta_time), stat,label='SYM')

#MAXSUM-2
data=[]
for filename in filenames:
    with open(filename + 'MAXSUM-2.csv','r') as csvfile:
        for line in csvfile.readlines():
            array=line.split(",")
            data.append(int(array[0]))

stat=[]
for i in range(0,300000,delta_time):
    count = 0
    for n in data:
        if n <= i:
            count = count + 1
    stat.append(count / InsNum)
plt.plot(range(0,300000,delta_time), stat,label='MAX-2')

#SAT
data=[]
for filename in filenames:
    with open(filename + 'SAT.csv','r') as csvfile:
        for line in csvfile.readlines():
            array=line.split(",")
            data.append(int(array[0]))

stat=[]
for i in range(0,300000,delta_time):
    count = 0
    for n in data:
        if n <= i:
            count = count + 1
    stat.append(count / InsNum)
plt.plot(range(0,300000,delta_time), stat,label='MDD-SAT')

#MAXMIN-2
# data=[]
# for filename in filenames:
#     with open(filename + 'MAXSMALL-2.csv','r') as csvfile:
#         for line in csvfile.readlines():
#             array=line.split(",")
#             data.append(int(array[0]))
#
# stat=[]
# for i in range(0,300000,delta_time):
#     count = 0
#     for n in data:
#         if n <= i:
#             count = count + 1
#     stat.append(count)
# plt.plot(range(0,300000 / InsNum,delta_time / InsNum), stat,label='MAXMIN-2')

#ASYM
# data=[]
# for filename in filenames:
#     with open(filename + 'ASYM.csv','r') as csvfile:
#         for line in csvfile.readlines():
#             array=line.split(",")
#             data.append(int(array[0]))
#
# stat=[]
# for i in range(0,300000,delta_time):
#     count = 0
#     for n in data:
#         if n <= i:
#             count = count + 1
#     stat.append(count)
# plt.plot(range(0,300000,delta_time), stat,label='ASYM')





#CBSH-2
# data=[]
# for filename in filenames:
#     with open(filename + 'CBSH-2.csv','r') as csvfile:
#         for line in csvfile.readlines():
#             array=line.split(",")
#             data.append(int(array[0]))
#
# stat=[]
# for i in range(0,300000,delta_time):
#     count = 0
#     for n in data:
#         if n <= i:
#             count = count + 1
#     stat.append(count)
# plt.plot(range(0,300000,delta_time), stat,label='CBSH-2')


legend = plt.legend(loc='best',frameon=False,fontsize='17')
#legend = plt.legend(loc='lower right',frameon=False,fontsize='17')

plt.show()
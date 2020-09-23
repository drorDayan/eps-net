import numpy as np
import matplotlib.mlab as mlab
import matplotlib.pyplot as plt


# small
data = [[1.00,0.98,1.00,1.00,1.00],
        [1.00,0.84,1.00,1.00,1.00],
        [0.86,0.60,0.86,0.92,0.94],
        [0.46,0.46,0.66,0.88,0.94],
        [0.24,0.26,0.50,0.82,0.96],
        [0.14,0.20,0.48,0.66,0.76],
        [0.08,0.08,0.18,0.42,0.74],
]
x = range(2,9)

# large
data = [[0.80,0.96,1.00,1.00,0.94],
        [0.34,0.74,0.94,0.98,0.68],
        [0.16,0.36,0.58,0.84,0.38],
        [0.00,0.10,0.34,0.66,0.24],
        [0.00,0.06,0.16,0.64,0.08]
]
x = range(5,30,5)
x_ticks = range(5,30,5)

# CBS variants
data = [[0.98,1,1,1,1],
        [0.84,1,1,1,1],
        [0.6,0.84,0.86,0.86,0.92],
        [0.46,0.6,0.66,0.68,0.88],
        [0.26,0.48,0.5,0.52,0.82]]
x = range(2,7)
x_ticks = range(2,7)

arr=np.array(data)

data = np.transpose(arr)
myfont = 20
plt.xlabel('Agents',{'size':myfont})
plt.ylabel('Success Rate',{'size':myfont})
plt.tick_params(labelsize=myfont)
plt.ylim(-0.05,1.05)
plt.xticks(x_ticks)
fig = plt.gcf()
fig.set_size_inches(5,5)
fig.subplots_adjust(left=0.18, bottom=0.15, right=0.99, top=0.99, wspace=None, hspace=None)
plt.plot(x, data[0],'>-',markerfacecolor='w',label='CBS')
plt.plot(x, data[1],'o-',markerfacecolor='w',label='ASYM')
plt.plot(x, data[2],'d-',markerfacecolor='w',label='SYM')
plt.plot(x, data[3],'p-',markerfacecolor='w',label='MAX-0')
plt.plot(x, data[4],'s-',markerfacecolor='w',label='MAX-2')
#plt.plot(x, data[0],'o-',markerfacecolor='w',label='EPEA*')
#plt.plot(x, data[1],'>-',markerfacecolor='w',label='CBS')
#plt.plot(x, data[2],'d-',markerfacecolor='w',label='SYM')
#plt.plot(x, data[3],'s-',markerfacecolor='w',label='MAX-2')
#plt.plot(x, data[4],'p-',markerfacecolor='w',label='MDD-SAT')


legend = plt.legend(loc='lower right',frameon=False,fontsize='17')
#legend = plt.legend(loc='upper right',frameon=False,fontsize='17')
# Put a nicer background color on the legend.
legend.get_frame().set_facecolor('w')



plt.show()
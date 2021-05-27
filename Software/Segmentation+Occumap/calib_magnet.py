import io
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D 
import numpy as np
from numpy.linalg import inv
from numpy.linalg import eig
from scipy import linalg
import matplotlib.ticker as mticker
import serial
import io
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D 
import sys

temp = []
temp1 = []
data = []
time = []
dt = 0.000512
currentSample = 0

file_data = open("magnet_calib_car.txt","w")
# mag_data = []
# with serial.Serial('COM8', 115200, timeout=1) as ser:
#     while(len(data)<1000):
#         try:
#             value = ser.readline()
#             print(value)
#             temp = (value.decode().replace('\n','').replace('\x00','')).split(' ')
#             temp1 = []
            
#             for (i,value1) in enumerate(temp):
#                 temp1.append(float(value1))
#             # data = np.array(data)
#             data.append(temp1)
#             print(len(data))
#             string = ""
#             for i in temp1:
#                 string += str(i) + " "
#             file_data.write(string + "\n")
#             #currentSample += dt
#             #time.append(currentSample)
#         except Exception as e:
#             print(e)
# data = np.array(data)
# print(data.shape)
# print(data)

M_x = []
M_y = []
M_z = []

X = []
mag_data = []

file_data = open("data_enc_gps_2.txt","r")
mag_data = []

for mag in file_data.readlines():
    try:
        X = mag.split(' ')
        M_x.append(float(X[0]))
        M_y.append(float(X[1]))
        M_z.append(float(X[2]))
    except Exception as e:
        print(e)

length = len(M_x)
print(len(M_y))
print(len(M_z))
# ax = fig.add_subplot(1,1,1)
# ax.scatter(M_x, M_y, M_z)
file_data.close()

M_x1 = np.sort(M_x, axis=None , kind  = "quicksort") 
M_y1 = np.sort(M_y, axis=None, kind  = "quicksort")  
M_z1 = np.sort(M_z, axis=None ,kind  = "quicksort") 

print("max x: %0.2f min x: %0.2f ",M_x1[length-1],M_x1[100])
# print("max y: %0.2f min y: %0.2f ",M_y1[length-1],M_y1[0])
# print("max z: %0.2f min z: %0.2f ",M_z1[length-1],M_z1[10])

M_x = np.array(M_x)
M_y = np.array(M_y)
M_z = np.array(M_z)

# M_y[np.where(M_y ==  6694.4)] = 145.28
# M_x[np.where(M_x >  654.06)] = 654.06
offset_x = (M_x1[length-1] + M_x1[0]) / 2
offset_y = (M_y1[length-3] + M_y1[17]) / 2
offset_z = (M_z1[length-11] + M_z1[0]) / 2

print("offset x: %0.2f offset y: %0.2f offset z: %0.2f",offset_x,offset_y,offset_z)

fig = plt.figure()
ax1 = fig.add_subplot(1, 3, 1)
ax2 = fig.add_subplot(2, 3, 2)
ax3 = fig.add_subplot(3, 3, 3)

fig, ax = plt.subplots(tight_layout=True)
hist = ax1.hist(M_x)
hist1 = ax2.hist(M_y)
hist2 = ax3.hist(M_z)

# print(max(M_zc))
# print(max(M_yc))
# print(max(M_xc))
# ax.set_xlabel('X Label')
# ax.set_ylabel('Y Label')
# ax.set_zlabel('Z Label')


avg_delta_x = (M_x1[length-150] - M_x1[0]) / 2
avg_delta_y = (M_y1[length-3] - M_y1[17]) / 2
avg_delta_z = (M_z1[length-11] - M_z1[0]) / 2
print("delta x: %0.2f delta y: %0.2f delta z: %0.2f",avg_delta_x,avg_delta_y,avg_delta_z)
avg_delta = (avg_delta_x + avg_delta_y + avg_delta_z) / 3

scale_x = avg_delta / avg_delta_x
scale_y = avg_delta / avg_delta_y
scale_z = avg_delta / avg_delta_z
# M_x = np.subtract(M_x, offset_x)
# M_y = np.subtract(M_y, offset_y)
# M_z = np.subtract(M_z, offset_z)
# M_x = (M_x-offset_x) * scale_x
# M_y = (M_y-offset_y) * scale_y
# M_z = (M_z-offset_z) * scale_z
# print(scale_x)
# print(scale_y)
# print(scale_z)
# scale_x = 0.2

plt.autoscale(enable=True, axis='both', tight=None)
# ax.set_xlim(-300, 300)
# ax.set_ylim(-300,300)

ax.scatter(M_x, M_y, color = "red")
ax.scatter(M_x, M_z, color = "green")
ax.scatter(M_y, M_z, color = "blue")



formatter = mticker.ScalarFormatter()
ax.xaxis.set_major_formatter(formatter)
ax.xaxis.set_major_locator(mticker.FixedLocator(np.arange(M_x.min()+1, M_x.max()+1, 20)))

ax.yaxis.set_major_formatter(formatter)
ax.yaxis.set_major_locator(mticker.FixedLocator(np.arange(M_y.min()+1, M_y.max()+1, 20)))

# fig1 = plt.figure()
# ax_3D = fig1.gca(projection='3d')

# ax_3D.scatter(M_x, M_y, M_z)


# ax.set_aspect("equal")
# ax.set_xlim3d(-200, 200)
# ax.set_ylim3d(-200,s200)

# ax.set_zlim3d(-200,200)

# ax.scatter(M_x, M_y, M_z)

# plt.axis("equal")

def eliptic_fit(d):
    print(d[0].shape)
    D = np.array((d[0]**2,d[1]**2,d[2]**2, 2*d[1]*d[2],2*d[0]*d[2],2*d[0]*d[1],d[0],d[1],d[2],np.ones_like(d[0])))

    C = np.array([[-1,  1,  1,  0,  0,  0],
                    [ 1, -1,  1,  0,  0,  0],
                    [ 1,  1, -1,  0,  0,  0],
                    [ 0,  0,  0, -4,  0,  0],
                    [ 0,  0,  0,  0, -4,  0],
                    [ 0,  0,  0,  0,  0, -4]])
        
    S = np.dot(D,np.transpose(D))

    print(S.shape)
    S11 = S[0:6,0:6]
    S12 = S[0:6,6:10]
    S21 = S[6:10,0:6]
    S22 = S[6:10,6:10]

    E = np.dot(inv(C),S11 - np.dot(S12,np.dot(inv(S22),np.transpose(S12))))
    Ew, Ev = eig(E)
    print(Ew.shape)
    v_1 = Ev[:, np.argmax(Ew)]
    if v_1[0] < 0: v_1 = -v_1
    v_2 = np.dot(np.dot(-np.linalg.inv(S22), S21), v_1)
    #v_2 = np.dot(-np.dot(inv(S22),S21),v_1)
    # print(v_2)
    M = np.array([[v_1[0], v_1[3], v_1[4]],
                    [v_1[3], v_1[1], v_1[5]],
                    [v_1[4], v_1[5], v_1[2]]])
    n = np.array([[v_2[0]],
                [v_2[1]],
                [v_2[2]]])
    d = v_2[3]

    return M, n, d    
def __ellipsoid_fit( s):
        ''' Estimate ellipsoid parameters from a set of points.

            Parameters
            ----------
            s : array_like
              The samples (M,N) where M=3 (x,y,z) and N=number of samples.

            Returns
            -------
            M, n, d : array_like, array_like, float
              The ellipsoid parameters M, n, d.

            References
            ----------
            .. [1] Qingde Li; Griffiths, J.G., "Least squares ellipsoid specific
               fitting," in Geometric Modeling and Processing, 2004.
               Proceedings, vol., no., pp.335-340, 2004
        '''

        # D (samples)
        D = np.array([s[0]**2., s[1]**2., s[2]**2.,
                      2.*s[1]*s[2], 2.*s[0]*s[2], 2.*s[0]*s[1],
                      2.*s[0], 2.*s[1], 2.*s[2], np.ones_like(s[0])])

        # S, S_11, S_12, S_21, S_22 (eq. 11)
        S = np.dot(D, D.T)
        S_11 = S[:6,:6]
        S_12 = S[:6,6:]
        S_21 = S[6:,:6]
        S_22 = S[6:,6:]

        # C (Eq. 8, k=4)
        C = np.array([[-1,  1,  1,  0,  0,  0],
                      [ 1, -1,  1,  0,  0,  0],
                      [ 1,  1, -1,  0,  0,  0],
                      [ 0,  0,  0, -4,  0,  0],
                      [ 0,  0,  0,  0, -4,  0],
                      [ 0,  0,  0,  0,  0, -4]])

        # v_1 (eq. 15, solution)
        E = np.dot(linalg.inv(C),
                   S_11 - np.dot(S_12, np.dot(linalg.inv(S_22), S_21)))

        E_w, E_v = np.linalg.eig(E)

        v_1 = E_v[:, np.argmax(E_w)]
        print("giá trị")
        print(E_w)
        print(v_1.shape)
        print(E_v)
        print(E_w.shape)
        print(E_w)
        print(E_v)
        if v_1[0] < 0: v_1 = -v_1

        # v_2 (eq. 13, solution)
        v_2 = np.dot(np.dot(-np.linalg.inv(S_22), S_21), v_1)

        # quadric-form parameters
        M = np.array([[v_1[0], v_1[3], v_1[4]],
                      [v_1[3], v_1[1], v_1[5]],
                      [v_1[4], v_1[5], v_1[2]]])
        n = np.array([[v_2[0]],
                      [v_2[1]],
                      [v_2[2]]])
        d = v_2[3]

        return M, n, d

M = np.array((M_x[0:500],M_y[0:500],M_z[0:500]))

M_,n_,d_ = eliptic_fit(M)
M_1,n_1,d_1= __ellipsoid_fit(M)
A = (1/np.sqrt(np.dot(n_.T,np.dot(inv(M_),n_))-d_))*linalg.sqrtm(M_)
print(A)

print(M_)
# print(M_1)

M_2 = linalg.inv(M_1)
b = -np.dot(M_2, n_1)
print(b)
A_1 = np.real(1 / np.sqrt(np.dot(n_1.T, np.dot(M_2, n_1)) - d_1) *
                           linalg.sqrtm(M_1))

M_calib = np.dot((A_1 ),(M- b))  

print(np.min(M_calib))

print(A)
print(A_1)
fig2 = plt.figure()
ax_3D_1 = fig2.gca(projection='3d')

ax_3D_1.scatter(M_calib[0,:],M_calib[1,:],M_calib[2,:])
plt.show()
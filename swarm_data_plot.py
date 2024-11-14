import math

from scipy.io import loadmat
import os
import matplotlib.pyplot as plt

file_path = '/home/emmanuel/AFC_Optitrack/robot_swarm/'
files = os.listdir(file_path)
files.sort(key=lambda x: os.path.getmtime(os.path.join(file_path, x)), reverse=True)


last_file = files[0]
mat_data = loadmat(os.path.join(file_path, last_file))


# robot_1
time = mat_data['Data_time']
position1 = mat_data['robot_1']
px1 = [row[0] for row in position1] # column vector
py1 = [row[1] for row in position1]
pz1 = [row[2] for row in position1]
yaw1 = [row[3] for row in position1]

# robot_2
position2 = mat_data['robot_2']
px2 = [row[0] for row in position2] # column vector
py2 = [row[1] for row in position2]
pz2 = [row[2] for row in position2]
yaw2 = [row[3] for row in position2]

# ref_robot_1
ref_position1 = mat_data['ref_pos1']
px1_des = [row[0] for row in ref_position1]
py1_des = [row[1] for row in ref_position1]
pz1_des = [row[2] for row in ref_position1]

# ref_robot_2
ref_position2 = mat_data['ref_pos2']
px2_des = [row[0] for row in ref_position2]
py2_des = [row[1] for row in ref_position2]
pz2_des = [row[2] for row in ref_position2]

# altitude error
altitude_error_squared_1 = [(pz1_des - pz1) ** 2 for pz1_des, pz1 in zip(pz1_des, pz1)]
altitude_error_squared_2 = [(pz2_des - pz2) ** 2 for pz2_des, pz2 in zip(pz2_des, pz2)]

time_flat_list = [item for sublist in time for item in sublist]

time_index_5 = min(range(len(time_flat_list)), key=lambda i: abs(time_flat_list[i] - 5))

trim_error_squared_1 = altitude_error_squared_1[time_index_5:len(altitude_error_squared_1)]
trim_error_squared_2 = altitude_error_squared_2[time_index_5:len(altitude_error_squared_2)]
mean_error_squared_1 = sum(trim_error_squared_1)/len(trim_error_squared_1)
mean_error_squared_2 = sum(trim_error_squared_1)/len(trim_error_squared_1)
rmse_1 = math.sqrt(mean_error_squared_1)
rmse_2 = math.sqrt(mean_error_squared_2)
print("rmse for drone 1 & 2: ", rmse_1, rmse_2)



## plotting out

plt.figure(figsize=(10, 5))

rmse_1 = round(rmse_1, 4)
rmse_2 = round(rmse_2, 4)

text_label = 'RMSE (robot 1 & 2): {} (m)'.format(rmse_1, rmse_2)


plt.plot(time[0], pz1, label='measurements', color='orange')
plt.plot(time[0], pz1_des, label='reference', color='orange', linestyle='dashed')
plt.plot(time[0], pz2, label='measurements', color='grey')
plt.plot(time[0], pz2_des, label='reference', color='grey', linestyle='dashed')
#plt.plot(time[0], altitude_error, label='pz_des', linestyle='dashed')
# plt.title('Altitude Plot')
plt.legend()
plt.xlim(5, 40)
plt.ylim(0, 1.8)
plt.xlabel('time (s)')
plt.ylabel('altitude (m)')

plt.text(15, 1.6, text_label, fontsize=20, ha='left', va='bottom', color='blue')




# Show the figure
plt.show()


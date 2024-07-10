import pandas as pd
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt
from uwb_kalman import *


# Define a low-pass filter function
def low_pass_filter(data, cutoff_freq, fs, order=4):
    nyquist = 0.5 * fs
    normal_cutoff = cutoff_freq / nyquist
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    filtered_data = filtfilt(b, a, data)
    return filtered_data

# Load the uploaded Excel file
file_path = 'NLink_LinkTrack_Node_Frame2_20240710_192226.xlsx'
excel_data = pd.ExcelFile(file_path)

# Display sheet names to understand the structure of the file
sheet_names = excel_data.sheet_names

# Load the data from the first sheet
sheet_data = pd.read_excel(file_path, sheet_name='NLink_LinkTrack_Node_Frame2')

# Display the first few rows of the data to understand its structure
sheet_data.head()

# Extract the relevant columns for visualization
pos_x = sheet_data['pos_x(m)']
pos_y = sheet_data['pos_y(m)']

# # Create a scatter plot to visualize pos_x and pos_y
# plt.figure(figsize=(10, 6))
# # plt.plot(pos_x, pos_y, color='pink', marker='.', linestyle='-', linewidth=1)
# plt.plot(pos_x, pos_y, color='pink', linestyle='-', linewidth=1)
# plt.title('Visualization of pos_x and pos_y')
# plt.xlabel('pos_x (m)')
# plt.ylabel('pos_y (m)')
# plt.xlim(-1,6)
# plt.ylim(-1,5)
# plt.grid(True)
# plt.show()

# # Parameters for the low-pass filter
# cutoff_frequency = 1.0  # Hz
# sampling_frequency = 50  # Assumed sampling frequency in Hz

# # Apply low-pass filter to pos_x and pos_y data
# filtered_pos_x = low_pass_filter(pos_x, cutoff_frequency, sampling_frequency)
# filtered_pos_y = low_pass_filter(pos_y, cutoff_frequency, sampling_frequency)

# # Plot the filtered data
# plt.figure(figsize=(10, 6))
# plt.plot(filtered_pos_x, filtered_pos_y, color='blue', linestyle='-', linewidth=1)
# plt.title('2D Plot of Filtered pos_x and pos_y')
# plt.xlabel('pos_x (m)')
# plt.ylabel('pos_y (m)')
# plt.xlim(-1,6)
# plt.ylim(-1,5)
# plt.grid(True)
# # plt.axis('equal')  # Ensure the aspect ratio is equal to accurately represent distances
# plt.show()
positions=[]
for i in range(0, len(pos_x)):
    positions.append((pos_x[i], pos_y[i]))

pos_kalman=[]
for pos in positions:
    x, P = kalman_filter(x, P, pos)
    print(f"Filtered position: {x[0]:.2f}, {x[2]:.2f}")
    pos_kalman.append((x[0], x[2]))

x_kalman=[]
y_kalman=[]

for pos in pos_kalman:
    x_kalman.append(pos[0])
    y_kalman.append(pos[1])

# Create a scatter plot to visualize pos_x and pos_y
plt.figure(figsize=(10, 6))
# plt.plot(pos_x, pos_y, color='pink', marker='.', linestyle='-', linewidth=1)
plt.plot(pos_x, pos_y, color='pink', linestyle='-', linewidth=1)
plt.plot(x_kalman, y_kalman, color='blue', linestyle='-', linewidth=1)
plt.title('Visualization of kalman filtered pos')
plt.xlabel('pos_x (m)')
plt.ylabel('pos_y (m)')
plt.xlim(-1,6)
plt.ylim(-1,5)
plt.grid(True)
plt.show()

# 设置均值和方差
mean = [0, 0]
variance = 1
std_dev = np.sqrt(variance)  # 标准差是方差的平方根

# 生成正态分布坐标点
num_points = 1000  # 生成的点数
x_coords = np.random.normal(mean[0], std_dev, num_points)
y_coords = np.random.normal(mean[1], std_dev, num_points)

# 合并坐标点
coordinates = np.column_stack((x_coords, y_coords))

# 打印生成的坐标点
print(coordinates)

# 可视化生成的坐标点
plt.scatter(x_coords, y_coords, alpha=0.5)
plt.title('Normally Distributed Points')
plt.xlabel('X')
plt.ylabel('Y')
plt.grid(True)
plt.show()


pos_kalman=[]
for pos in coordinates:
    x, P = kalman_filter(x, P, pos)
    print(f"Filtered position: {x[0]:.2f}, {x[2]:.2f}")
    pos_kalman.append((x[0], x[2]))

x_kalman=[]
y_kalman=[]

for pos in pos_kalman:
    x_kalman.append(pos[0])
    y_kalman.append(pos[1])

# Create a scatter plot to visualize pos_x and pos_y
plt.figure(figsize=(10, 6))
# plt.plot(pos_x, pos_y, color='pink', marker='.', linestyle='-', linewidth=1)
plt.plot(x_coords, y_coords, color='pink', linestyle='-', linewidth=1)
plt.plot(x_kalman, y_kalman, color='blue', linestyle='-', linewidth=1)
plt.title('Visualization of kalman filtered pos')
plt.xlabel('pos_x (m)')
plt.ylabel('pos_y (m)')
plt.axis('equal')
plt.grid(True)
plt.show()
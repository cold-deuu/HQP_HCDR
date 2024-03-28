import matplotlib.pyplot as plt

def read_data(file_path):
    x = []
    y = []
    z = []
    r = []
    p = []
    yaw = []
    q7 =[]

    with open(file_path, 'r') as file:
        for line in file:
            values = line.strip().split('\t')
            x.append(float(values[0]))
            y.append(float(values[1]))
            z.append(float(values[2]))
            r.append(float(values[3]))
            p.append(float(values[4]))
            yaw.append(float(values[5]))
            # q7.append(float(values[6]))

    return x, y, z, r, p, yaw


def read_data_2(file_path):
    x = []

    with open(file_path, 'r') as file:
        for line in file:
            values = line.strip().split('\t')
            x.append(float(values[0]))

    return x




# 파일 경로
se3_pose_file =  '/home/chan/rci_cdpr_ws/src/cdpr_controllers/txt/se3_pose.txt'
se3_desired_pose_file = '/home/chan/rci_cdpr_ws/src/cdpr_controllers/txt/desired_se3_pose.txt'

# se3_pose_txt = '/home/chan/rci_cdpr_ws/src/cdpr_controllers/txt/franka_q_real.txt'
# se3_des_pose_txt = '/home/chan/rci_cdpr_ws/src/cdpr_controllers/txt/franka_q_test.txt'
# input_txt = '/home/chan/rci_cdpr_ws/src/cdpr_controllers/txt/task_3_manip_.txt'
# slack_txt = '/home/chan/rci_cdpr_ws/src/cdpr_controllers/txt/1stSlack.txt'

# 데이터 읽기
# t1_values, t2_values, t3_values, t4_values, t5_values, t6_values, t7_values, t8_values = read_data(tension_txt)
x, y, z, r, p, yaw = read_data(se3_pose_file)
x_desired, y_desired, z_desired, r_desired, p_desired, yaw_desired= read_data(se3_desired_pose_file)
# slack11, slack12, slack13, slack14, slack15, slack16 = read_data(slack_txt) 
# slack21, slack22, slack23, slack24, slack25, slack26 = read_data(slack_txt2) 
# input = read_data_2(input_txt)
# input2 = read_data_2(not_input_txt)


# # X 값 플롯 생성
plt.subplot(6,1,1)
plt.plot(x, label='x_real')
plt.plot(x_desired, label='x_desired')
plt.xlabel('Index')
plt.ylabel('X')
plt.title('X Real vs X Desired Plot')
plt.legend()
plt.grid(True)

# Y 값 플롯 생성
plt.subplot(6,1,2)
plt.plot(y, label='y_real')
plt.plot(y_desired, label='y_desired')
plt.xlabel('Index')
plt.ylabel('Y')
plt.title('Y Real vs Y Desired Plot')
plt.legend()
plt.grid(True)

# Z 값 플롯 생성
plt.subplot(6,1,3)

plt.plot(z, label='z_real')
plt.plot(z_desired, label='z_desired')
plt.xlabel('Index')
plt.ylabel('Z')
plt.title('Z Real vs Z Desired Plot')
plt.legend()
plt.grid(True)

# R 값 플롯 생성
plt.subplot(6,1,4)

plt.plot(r, label='r_real')
plt.plot(r_desired, label='r_desired')
plt.xlabel('Index')
plt.ylabel('R')
plt.title('R Real vs R Desired Plot')
plt.legend()
plt.grid(True)

# P 값 플롯 생성
plt.subplot(6,1,5)
plt.plot(p, label='p_real')
plt.plot(p_desired, label='p_desired')
plt.xlabel('Index')
plt.ylabel('P')
plt.title('P Real vs P Desired Plot')
plt.legend()
plt.grid(True)

# Yaw 값 플롯 생성
plt.subplot(6,1,6)
plt.plot(yaw, label='yaw_real')
plt.plot(yaw_desired, label='yaw_desired')
plt.xlabel('Index')
plt.ylabel('Yaw')
plt.title('Yaw Real vs Yaw Desired Plot')
plt.legend()
plt.grid(True)



# plt.subplot(6,1,1)
# plt.plot(slack11, label='x_real')
# # plt.plot(slack21, label='x_desired')
# plt.xlabel('Index')
# plt.ylabel('X')
# plt.title('X Real vs X Desired Plot')
# plt.legend()
# plt.grid(True)

# # Y 값 플롯 생성
# plt.subplot(6,1,2)
# plt.plot(slack12, label='y_real')
# # plt.plot(slack22, label='y_desired')
# plt.xlabel('Index')
# plt.ylabel('Y')
# plt.title('Y Real vs Y Desired Plot')
# plt.legend()
# plt.grid(True)

# # Z 값 플롯 생성
# plt.subplot(6,1,3)

# plt.plot(slack13, label='z_real')
# # plt.plot(slack23, label='z_desired')
# plt.xlabel('Index')
# plt.ylabel('Z')
# plt.title('Z Real vs Z Desired Plot')
# plt.legend()
# plt.grid(True)

# # R 값 플롯 생성
# plt.subplot(6,1,4)

# plt.plot(slack14, label='r_real')
# # plt.plot(slack24, label='r_desired')
# plt.xlabel('Index')
# plt.ylabel('R')
# plt.title('R Real vs R Desired Plot')
# plt.legend()
# plt.grid(True)

# # P 값 플롯 생성
# plt.subplot(6,1,5)
# plt.plot(slack15, label='p_real')
# # plt.plot(slack25, label='p_desired')
# plt.xlabel('Index')
# plt.ylabel('P')
# plt.title('P Real vs P Desired Plot')
# plt.legend()
# plt.grid(True)

# # Yaw 값 플롯 생성
# plt.subplot(6,1,6)
# plt.plot(slack16, label='yaw_real')
# # plt.plot(slack26, label='yaw_desired')
# plt.xlabel('Index')
# plt.ylabel('Yaw')
# plt.title('Yaw Real vs Yaw Desired Plot')
# plt.legend()
# plt.grid(True)
# plt.show()




# plt.subplot(2,1,1)
# plt.plot(input,label='max manip')
# plt.plot(input2, label = 'min manip') 
# plt.title("platform parallel task")
# plt.legend()

# plt.subplot(2,1,2)
# plt.plot(static_,label='no manipulability')
# plt.plot(static_not, label = 'manipulability')
# plt.title("platform static task")
# plt.legend()
# plt.plot()
plt.show()

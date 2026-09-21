import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# 设置中文字体和负号显示
plt.rcParams['font.sans-serif'] = ['WenQuanYi Micro Hei', 'WenQuanYi Zen Hei', 'Noto Sans CJK SC', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False   # 解决负号显示问题

# 1. 读取数据
df = pd.read_csv('tracking_log.csv')
cmd_ang = df['cmd_angular'].values

# 2. 模拟带噪声的IMU（虚拟里程计）
np.random.seed(42)
noise = np.random.normal(0, 0.1, size=len(cmd_ang))  # 模拟陀螺仪噪声
z_imu = cmd_ang + noise  # 这是“实测”的角速度

# 3. 一维卡尔曼滤波器（核心算法）
def kalman_filter_1d(measurements, Q=0.01, R=0.1):
    x_est = 0.0  # 初始估计值
    P = 1.0      # 初始误差协方差
    result = []
    for z in measurements:
        # 预测（假定状态不变，没有外部输入）
        x_pred = x_est
        P_pred = P + Q
        # 更新（利用测量值z）
        K = P_pred / (P_pred + R)
        x_est = x_pred + K * (z - x_pred)
        P = (1 - K) * P_pred
        result.append(x_est)
    return np.array(result)

filtered_ang = kalman_filter_1d(z_imu, Q=0.01, R=0.1)

# 4. 绘图对比
plt.figure(figsize=(12, 6))
plt.plot(cmd_ang, label='原始控制指令 (无滤波)', alpha=0.7, color='red')
plt.plot(z_imu, label='模拟IMU测量 (含噪声)', alpha=0.5, color='gray')
plt.plot(filtered_ang, label='卡尔曼滤波输出 (平滑指令)', linewidth=2, color='blue')
plt.legend()
plt.title('视觉伺服控制信号平滑效果对比')
plt.grid(True)
plt.savefig('kalman_smooth.png')
plt.show()
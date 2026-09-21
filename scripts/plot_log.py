import pandas as pd
import matplotlib.pyplot as plt

# 设置中文字体和负号显示
plt.rcParams['font.sans-serif'] = ['WenQuanYi Micro Hei', 'WenQuanYi Zen Hei', 'Noto Sans CJK SC', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False   # 解决负号显示问题
# 读取 CSV
df = pd.read_csv('tracking_log.csv')

# 创建双轴图
fig, ax1 = plt.subplots(figsize=(12, 6))

# 左轴：误差（像素）
color = 'tab:blue'
ax1.set_xlabel('帧数 (Frame)')
ax1.set_ylabel('横向误差 (像素)', color=color)
ax1.plot(df['frame'], df['error_x'], color=color, linewidth=1.5, label='误差 e_x')
ax1.tick_params(axis='y', labelcolor=color)
ax1.grid(True, linestyle='--', alpha=0.6)

# 右轴：角速度
ax2 = ax1.twinx()
color = 'tab:red'
ax2.set_ylabel('角速度 (rad/s)', color=color)
ax2.plot(df['frame'], df['cmd_angular'], color=color, linewidth=1.5, linestyle='--', label='角速度')
ax2.tick_params(axis='y', labelcolor=color)

# 标题
plt.title('视觉伺服控制响应曲线 (纯P控制, kp=0.5)')

# 合并图例（手动构造）
lines1, labels1 = ax1.get_legend_handles_labels()
lines2, labels2 = ax2.get_legend_handles_labels()
ax1.legend(lines1 + lines2, labels1 + labels2, loc='upper right')

plt.tight_layout()
plt.savefig('control_response.png', dpi=300)  # 保存高清图，这是你简历要用的！
plt.show()

# 输出统计信息（额外加分项）
print("=== 控制性能统计 ===")
print(f"最大绝对误差: {df['error_x'].abs().max():.2f} 像素")
print(f"平均绝对误差: {df['error_x'].abs().mean():.2f} 像素")
print(f"稳态误差(最后100帧均值): {df['error_x'].tail(100).mean():.2f} 像素")
print(f"最大角速度: {df['cmd_angular'].abs().max():.4f} rad/s")
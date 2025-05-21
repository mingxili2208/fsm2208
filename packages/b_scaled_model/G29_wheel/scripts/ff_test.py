import pygame

# 初始化pygame和joystick模块
pygame.init()
pygame.joystick.init()

# 检查是否有方向盘连接
joystick_count = pygame.joystick.get_count()
if joystick_count == 0:
    print("没有找到方向盘设备")
    exit()

# 初始化G29方向盘
wheel = pygame.joystick.Joystick(0)
wheel.init()
print(f"方向盘名称: {wheel.get_name()}")

# 设置力矩反馈
# 注意：具体的力矩设置方法可能因pygame版本而异
try:
    # 设置恒定力矩
    # 参数范围通常从-1.0到1.0
    force = 0.5  # 50%的力矩
    wheel.set_rumble(0, force, 0)  # 左侧电机
    wheel.set_rumble(1, force, 0)  # 右侧电机
    print(f"已设置力矩为 {force * 100}%")
except:
    print("不支持设置力矩或设置失败")

# 保持程序运行一段时间
import time
time.sleep(5)

# 关闭力矩反馈
try:
    wheel.set_rumble(0, 0, 0)
    wheel.set_rumble(1, 0, 0)
    print("已关闭力矩反馈")
except:
    pass

# 清理资源
pygame.quit()
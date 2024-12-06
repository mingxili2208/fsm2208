import pygame
import sys

# 初始化 Pygame
pygame.init()

# 设置初始窗口大小
width, height = 640, 480
screen = pygame.display.set_mode((width, height), pygame.RESIZABLE)

# 创建一个自定义事件用于延迟重绘
REDRAW_EVENT = pygame.USEREVENT + 1

# 主循环
running = True
resize_timer = None
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.VIDEORESIZE:
            # 当用户调整窗口大小时，设置一个定时器
            width, height = event.w, event.h
            if resize_timer:
                pygame.time.set_timer(REDRAW_EVENT, 0)  # 取消之前的定时器
            pygame.time.set_timer(REDRAW_EVENT, 500)  # 500毫秒后触发重绘事件
        elif event.type == REDRAW_EVENT:
            # 重绘事件触发
            #screen = pygame.display.set_mode((width, height), pygame.RESIZABLE)
            pygame.time.set_timer(REDRAW_EVENT, 0)  # 取消定时器
            print(f"Window resized to: {width}x{height}")
            resize_timer = None

    # 清屏并填充背景色
    screen.fill((0, 120, 255))
    pygame.display.flip()

# 退出 Pygame
pygame.quit()
sys.exit()
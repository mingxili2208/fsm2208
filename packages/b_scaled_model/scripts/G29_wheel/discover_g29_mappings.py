#!/usr/bin/env python3

import evdev
from evdev import ecodes, categorize
import time
import os

# --- 配置 ---
# 尝试自动查找G29，如果找不到，请手动修改此处的名称或在运行时从列表中选择
DEVICE_NAME_HINT = "Logitech G29" 

def find_device_path(device_name_hint):
    """查找包含指定名称提示的输入设备路径"""
    devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
    found_devices = []
    for device in devices:
        if device_name_hint.lower() in device.name.lower():
            found_devices.append(device)
    
    if not found_devices:
        return None
    if len(found_devices) == 1:
        print(f"自动找到设备: {found_devices[0].name} ({found_devices[0].path})")
        return found_devices[0].path
    
    # 如果找到多个匹配项，让用户选择
    print(f"找到多个包含 '{device_name_hint}' 的设备，请选择一个:")
    for i, device in enumerate(found_devices):
        print(f"  {i}: {device.name} ({device.path})")
    
    while True:
        try:
            choice = int(input("输入设备编号: "))
            if 0 <= choice < len(found_devices):
                print(f"已选择: {found_devices[choice].name} ({found_devices[choice].path})")
                return found_devices[choice].path
            else:
                print("无效的选择，请重试。")
        except ValueError:
            print("请输入数字。")

def main():
    if os.geteuid() != 0:
        print("提示: 此脚本可能需要以root权限运行 (sudo python3 discover_g29_mappings.py) 才能访问输入设备。")
        # 即使没有root权限也尝试继续，但可能会失败

    dev_path = find_device_path(DEVICE_NAME_HINT)

    if not dev_path:
        print(f"错误: 未找到包含 '{DEVICE_NAME_HINT}' 的设备。请确保设备已连接。")
        print("可用设备列表:")
        for i, path_in_list in enumerate(evdev.list_devices()):
            try:
                dev_temp = evdev.InputDevice(path_in_list)
                print(f"  设备 {i} -> {path_in_list}: {dev_temp.name}")
            except Exception:
                print(f"  设备 {i} -> {path_in_list}: (无法获取名称或访问权限)")
        
        # 如果自动查找失败，允许用户手动输入设备路径
        manual_path = input("或者，手动输入设备路径 (例如 /dev/input/eventX)，留空则退出: ").strip()
        if manual_path:
            dev_path = manual_path
        else:
            return


    print(f"\n将尝试监听设备: {dev_path}")
    print("请操作您的方向盘、踏板和按钮，脚本将打印出原始事件代码和值。")
    print("按 Ctrl+C 退出。\n")

    try:
        device = evdev.InputDevice(dev_path)
    except PermissionError:
        print(f"错误: 权限不足，无法打开设备 {dev_path}。请尝试使用 'sudo' 运行此脚本。")
        return
    except FileNotFoundError:
        print(f"错误: 设备路径 {dev_path} 未找到。请检查路径是否正确。")
        return
    except Exception as e:
        print(f"打开设备 {dev_path} 时发生未知错误: {e}")
        return

    print(f"成功打开设备: {device.name}")
    print("开始监听事件...\n")
    
    # 存储上一次的轴值，仅在变化时打印（可选，减少输出量）
    last_axis_values = {}

    try:
        for event in device.read_loop():
            if event.type == ecodes.EV_ABS:
                # event.code 是轴的数字代码 (例如 0 for ABS_X, 1 for ABS_Y)
                # ecodes.bytype[event.type][event.code] 可以给出轴的名称 (例如 'ABS_X')
                axis_name = ecodes.bytype[event.type].get(event.code, f"未知轴代码 {event.code}")
                
                # 仅当值变化时打印，以减少输出（对于连续变化的轴）
                if last_axis_values.get(event.code) != event.value:
                    print(f"轴事件 -> 代码: {event.code} (名称: {axis_name}), 当前值: {event.value}")
                    last_axis_values[event.code] = event.value

            elif event.type == ecodes.EV_KEY:
                # event.code 是按钮的数字代码 (例如 288 for BTN_JOYSTICK)
                # ecodes.bytype[event.type][event.code] 可以给出按钮的名称 (例如 'BTN_SOUTH' 或一个列表 ['BTN_JOYSTICK', 'BTN_TRIGGER'])
                button_name_raw = ecodes.bytype[event.type].get(event.code, f"未知按钮代码 {event.code}")
                
                # 如果 button_name_raw 是一个列表 (有些代码有多个别名)，取第一个
                button_name = button_name_raw[0] if isinstance(button_name_raw, list) else button_name_raw

                action = "按下 (Pressed)" if event.value == 1 else \
                         "松开 (Released)" if event.value == 0 else \
                         f"重复/保持 (Repeat/Hold state {event.value})" # event.value == 2 for repeat
                
                print(f"按钮事件 -> 代码: {event.code} (名称: {button_name}), 状态: {action}")

            # 可以取消注释以查看同步事件，但通常对于映射来说不必要
            # elif event.type == ecodes.EV_SYN:
            #     if event.code == ecodes.SYN_REPORT:
            #         print("--- (同步事件 SYN_REPORT) ---")
            #     else:
            #         print(f"同步事件 -> 代码: {event.code}, 值: {event.value}")

    except KeyboardInterrupt:
        print("\n脚本被用户中断。")
    except Exception as e:
        print(f"\n读取事件时发生错误: {e}")
    finally:
        print("关闭设备。")
        if 'device' in locals() and device: # 确保device已定义且打开
            try:
                device.close()
            except Exception as e_close:
                print(f"关闭设备时出错: {e_close}")


if __name__ == "__main__":
    main()
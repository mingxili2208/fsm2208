#!/usr/bin/env python3

import evdev
from evdev import ecodes, ff
import time
import os

# --- 配置 ---
DEVICE_NAME = "Logitech G29 Driving Force Racing Wheel" # G29的设备名，G920可能是 "Logitech G920 Driving Force Racing Wheel"

# 轴的友好名称映射 (根据G29常见情况，您的设备可能略有不同)
# 您可以通过观察输出来调整这些ABS_CODE
AXIS_MAP = {
    ecodes.ABS_X: "方向盘转轴 (Steering Wheel)", # 通常范围 -32768 到 32767
    ecodes.ABS_Z: "油门踏板 (Accelerator)",    # 通常范围 0 (踩到底) 到 255 (松开) 或 -32768 到 32767
    ecodes.ABS_RZ: "刹车踏板 (Brake)",        # 同上
    # ecodes.ABS_Y: "离合器踏板 (Clutch)",      # 同上
    # ecodes.ABS_HAT0X: "方向键 X (D-Pad X)",
    # ecodes.ABS_HAT0Y: "方向键 Y (D-Pad Y)",
    # ecodes.ABS_RX: "红色旋钮? (Red Dial?)", # G29的红色旋钮可能报告为ABS_MISC, ABS_RX, ABS_RY等
    # 添加其他您观察到的ABS代码
}

# 按钮的友好名称映射 (根据G29常见情况)
# 您可以通过观察输出来调整这些BTN_CODE
# 罗技档位器 (Logitech Driving Force Shifter) 通常映射到以下按钮：
# 1档: 288 (BTN_JOYSTICK), 2档: 289 (BTN_THUMB), 3档: 290 (BTN_THUMB2)
# 4档: 291 (BTN_TOP), 5档: 292 (BTN_TOP2), 6档: 293 (BTN_PINKIE)
# R档: 294 (BTN_BASE) (通常是按下档杆再挂入6档位置)
BUTTON_MAP = {
    ecodes.BTN_SOUTH: "X/A 按钮 (Cross/A)",        # PlayStation: Cross, Xbox: A
    ecodes.BTN_EAST: "○/B 按钮 (Circle/B)",       # PlayStation: Circle, Xbox: B
    ecodes.BTN_NORTH: "△/Y 按钮 (Triangle/Y)",     # PlayStation: Triangle, Xbox: Y
    ecodes.BTN_WEST: "□/X 按钮 (Square/X)",       # PlayStation: Square, Xbox: X
    ecodes.BTN_TL: "L2 按钮",                   # L2 Trigger/Button
    ecodes.BTN_TR: "R2 按钮",                   # R2 Trigger/Button
    ecodes.BTN_TL2: "L3 按钮 (拨片左/LSB)",        # L3 Button (often Left Shift Paddle)
    ecodes.BTN_TR2: "R3 按钮 (拨片右/RSB)",        # R3 Button (often Right Shift Paddle)
    ecodes.BTN_SELECT: "Share/View 按钮",
    ecodes.BTN_START: "Options/Menu 按钮",
    ecodes.BTN_MODE: "PS/Xbox 按钮",
    ecodes.BTN_THUMBL: "红色旋钮按下 (Red Dial Press)",

    # 档位 (示例代码，请根据实际evtest输出调整)
    288: "档位 1 (Gear 1)",
    289: "档位 2 (Gear 2)",
    290: "档位 3 (Gear 3)",
    291: "档位 4 (Gear 4)",
    292: "档位 5 (Gear 5)",
    293: "档位 6 (Gear 6)",
    294: "档位 R (Gear Reverse)",
    # 方向盘上的 +/- 按钮, 旋钮选择等也可能是BTN_BASE系列
    ecodes.BTN_BASE: "+ 按钮 (+ Button on wheel)", # 这只是一个例子，具体代码会不同
    ecodes.BTN_BASE2: "- 按钮 (- Button on wheel)", # 这只是一个例子
    # ... 添加更多按钮映射
}

def find_device_path(device_name):
    """查找包含指定名称的输入设备路径"""
    devices = [evdev.InputDevice(path) for path in evdev.list_devices()]
    for device in devices:
        if device_name.lower() in device.name.lower():
            print(f"找到设备: {device.name} ({device.path})")
            return device.path
    return None

def main():
    if os.geteuid() != 0:
        print("警告: 此脚本可能需要以root权限运行 (sudo python3 g29_reader.py) 才能关闭力反馈和访问某些设备。")

    dev_path = find_device_path(DEVICE_NAME)

    if not dev_path:
        print(f"错误: 未找到名为 '{DEVICE_NAME}' 的设备。请检查设备是否连接，或修改脚本中的 DEVICE_NAME。")
        print("可用设备列表:")
        for path in evdev.list_devices():
            try:
                dev = evdev.InputDevice(path)
                print(f"  {path}: {dev.name}")
            except Exception:
                print(f"  {path}: (无法获取名称)")
        return

    try:
        device = evdev.InputDevice(dev_path)
    except PermissionError:
        print(f"错误: 权限不足，无法打开设备 {dev_path}。请尝试使用 'sudo' 运行此脚本。")
        return
    except Exception as e:
        print(f"打开设备时发生错误: {e}")
        return

    print(f"正在监听设备: {device.name}")
    print(f"物理路径: {device.phys}")
    print(f"支持的事件类型: {device.capabilities(verbose=True)}")

    # --- 尝试关闭力反馈 (自动回中) ---
    # 注意: 这取决于驱动程序是否支持以及如何实现。
    # 对于G29/G920，关闭自动回中通常是可行的。
    try:
        # 检查设备是否支持自动回中力反馈
        if ecodes.FF_AUTOCENTER in device.capabilities().get(ecodes.EV_FF, []):
            print("尝试将自动回中力设置为 0...")
            device.set_autocenter(0) # 0 表示关闭自动回中
            print("自动回中力反馈已尝试设置为 0。")
        else:
            print("设备不支持通过 set_autocenter() 直接设置自动回中。")
            # 可以尝试上传一个0强度的恒定力效果，但这更复杂
            # ff_effect = ff.Effect(
            #     ecodes.FF_CONSTANT, -1, 0,
            #     ff.Trigger(0,0),
            #     ff.Replay(1,0), # 播放一次，长度1ms (几乎立即)
            #     ff.EffectType(ff_constant_effect=ff.Constant(level=0, envelope=ff.Envelope(attack_length=0, attack_level=0, fade_length=0, fade_level=0)))
            # )
            # try:
            #     effect_id = device.upload_effect(ff_effect)
            #     device.write(ecodes.EV_FF, effect_id, 1) # 播放效果
            #     print("已尝试上传并播放一个0强度的恒定力效果。")
            #     # device.erase_effect(effect_id) # 可以选择之后移除
            # except Exception as e_ff:
            #     print(f"上传0强度力反馈效果失败: {e_ff}")

    except Exception as e:
        print(f"关闭力反馈时发生错误: {e}")
        print("继续读取输入值...")

    print("\n--- 开始读取输入值 (按 Ctrl+C 退出) ---")
    current_axis_values = {}

    try:
        for event in device.read_loop():
            if event.type == ecodes.EV_ABS:
                axis_name = AXIS_MAP.get(event.code, f"未知轴 (ABS Code {event.code})")
                current_axis_values[axis_name] = event.value
                
                # 清屏并打印所有当前轴值 (可选，为了更整洁的输出)
                # os.system('clear') # Linux/macOS
                # print("--- 当前轴状态 ---")
                # for name, val in sorted(current_axis_values.items()):
                #     print(f"{name}: {val}")
                # print("-" * 20)
                # 如果不清屏，就直接打印变化的轴：
                print(f"轴更新 -> {axis_name}: {event.value}")

            elif event.type == ecodes.EV_KEY:
                button_name = BUTTON_MAP.get(event.code, f"未知按钮 (BTN Code {event.code})")
                action = "按下 (Pressed)" if event.value == 1 else "松开 (Released)" if event.value == 0 else f"重复 (Repeat {event.value})"
                print(f"按钮事件 -> {button_name}: {action}")

            # EV_SYN 事件通常表示一批事件的结束，可以忽略打印
            # elif event.type == ecodes.EV_SYN:
            #     if event.code == ecodes.SYN_REPORT:
            #         print("--- SYN_REPORT (事件同步) ---")


    except KeyboardInterrupt:
        print("\n脚本被用户中断。")
    except Exception as e:
        print(f"\n读取事件时发生错误: {e}")
    finally:
        print("关闭设备。")
        if 'device' in locals(): # 确保device已定义
            device.close()

if __name__ == "__main__":
    main()
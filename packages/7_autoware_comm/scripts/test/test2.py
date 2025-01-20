import carla
import sys
import time

def get_vehicle_blueprint_widths_via_spawn(host='127.0.0.1', port=2000, timeout=10.0, output_file=None):
    """
    连接到 CARLA 服务器，临时生成所有车辆蓝图，获取车宽，并输出结果。

    :param host: CARLA 服务器的主机地址
    :param port: CARLA 服务器的端口
    :param timeout: 与服务器连接的超时时间
    :param output_file: 可选，指定文件路径将结果保存到该文件
    """
    try:
        # 创建客户端并连接到服务器
        client = carla.Client(host, port)
        client.set_timeout(timeout)

        # 获取世界和蓝图库
        world = client.get_world()
        blueprint_library = world.get_blueprint_library()

        # 过滤出所有车辆蓝图
        vehicle_blueprints = blueprint_library.filter('vehicle.*')

        if not vehicle_blueprints:
            print("未找到任何车辆蓝图。")
            return

        print(f"检测到 {len(vehicle_blueprints)} 种车辆蓝图。")
        
        # 准备输出
        output_lines = []
        header = "车辆类型, 车宽 (米)"
        output_lines.append(header)
        print(header)

        # 获取一组安全的生成点
        spawn_points = world.get_map().get_spawn_points()
        if not spawn_points:
            print("未找到任何生成点。")
            return

        spawn_pool = iter(spawn_points)
        idx = 0

        for bp in vehicle_blueprints:
            try:
                # 获取下一个生成点，如果用完则重置
                try:
                    spawn_point = next(spawn_pool)
                except StopIteration:
                    spawn_pool = iter(spawn_points)
                    spawn_point = next(spawn_pool)
                
                # 适当调整生成点的位置以避免重叠
                location = spawn_point.location #+ carla.Location(x=idx * 10, y=0, z=0)  # 每辆车相隔10米
                temp_transform = carla.Transform(location, carla.Rotation(yaw=0))
                
                # 生成临时车辆
                vehicle = world.spawn_actor(bp, temp_transform)
                
                # 确保车辆已经生成
                time.sleep(1)
                
                # 获取车辆的边界盒
                bounding_box = vehicle.bounding_box
                width = bounding_box.extent.y * 2  # extent.y 是半宽度
                
                line = f"{bp.id}, {width:.2f}"
                print(line)
                output_lines.append(line)
                
                # 销毁临时车辆
                vehicle.destroy()
                
                idx += 1
            except Exception as e:
                print(f"{bp.id}, 获取车宽时出错: {e}")
                output_lines.append(f"{bp.id}, 错误")

        # 如果指定了输出文件，保存结果
        if output_file:
            try:
                with open(output_file, 'w', encoding='utf-8') as f:
                    for line in output_lines:
                        f.write(line + '\n')
                print(f"\n结果已保存到 {output_file}")
            except Exception as e:
                print(f"无法保存结果到文件: {e}")

    except Exception as e:
        print(f"发生错误: {e}")

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="输出 CARLA 模型库中所有车辆模型的车宽（通过临时生成车辆）。")
    parser.add_argument('--host', type=str, default='127.0.0.1', help='CARLA 服务器的主机地址 (默认: 127.0.0.1)')
    parser.add_argument('--port', type=int, default=2000, help='CARLA 服务器的端口 (默认: 2000)')
    parser.add_argument('--timeout', type=float, default=10.0, help='连接超时时间 (默认: 10.0 秒)')
    parser.add_argument('--output', type=str, default=None, help='可选，指定文件路径保存结果')

    args = parser.parse_args()

    get_vehicle_blueprint_widths_via_spawn(host=args.host, port=args.port, timeout=args.timeout, output_file=args.output)
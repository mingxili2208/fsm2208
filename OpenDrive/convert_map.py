import os
import sys
import xml.etree.ElementTree as ET

def validate_xodr_file(xodr_file_path):
    """验证OpenDRIVE文件是否有效"""
    try:
        print("正在验证OpenDRIVE文件...")
        
        # 检查文件大小
        file_size = os.path.getsize(xodr_file_path)
        if file_size == 0:
            print("❌ 文件为空")
            return False
        
        print(f"  文件大小: {file_size} 字节")
        
        # 检查XML格式
        try:
            tree = ET.parse(xodr_file_path)
            root = tree.getroot()
            print(f"  XML根元素: {root.tag}")
            
            # 检查是否是OpenDRIVE文件
            if root.tag != 'OpenDRIVE':
                print(f"❌ 不是有效的OpenDRIVE文件，根元素应该是'OpenDRIVE'，当前是'{root.tag}'")
                return False
            
            # 检查版本
            version = root.get('version', 'unknown')
            print(f"  OpenDRIVE版本: {version}")
            
            # 检查是否有道路
            roads = root.findall('.//road')
            print(f"  道路数量: {len(roads)}")
            
            if len(roads) == 0:
                print("⚠️  文件中没有找到道路信息")
                return False
            
            # 检查车道信息
            lanes = root.findall('.//lane')
            print(f"  车道数量: {len(lanes)}")
            
            print("✓ OpenDRIVE文件验证通过")
            return True
            
        except ET.ParseError as e:
            print(f"❌ XML解析错误: {e}")
            return False
            
    except Exception as e:
        print(f"❌ 文件验证失败: {e}")
        return False

def convert_with_opendrive2lanelet(xodr_file_path, output_osm_file_path):
    """使用opendrive2lanelet库进行转换"""
    try:
        print("正在尝试使用 opendrive2lanelet...")
        
        # 尝试不同的导入方式
        try:
            from opendrive2lanelet.opendriveparser.parser import parse_opendrive
            print("✓ 导入 parse_opendrive (路径1)")
        except ImportError:
            try:
                from opendrive2lanelet.opendrive_parser.parser import parse_opendrive
                print("✓ 导入 parse_opendrive (路径2)")
            except ImportError:
                try:
                    from opendrive2lanelet import opendrive_parser
                    parse_opendrive = opendrive_parser.parser.parse_opendrive
                    print("✓ 导入 parse_opendrive (路径3)")
                except ImportError:
                    raise ImportError("无法找到 parse_opendrive 函数")
        
        # 尝试解析，使用不同的参数
        print("正在解析OpenDRIVE文件...")
        
        try:
            # 方法1：直接解析
            lanelet_network = parse_opendrive(xodr_file_path)
        except Exception as e1:
            print(f"直接解析失败: {e1}")
            try:
                # 方法2：读取为字符串后解析
                with open(xodr_file_path, 'r', encoding='utf-8') as f:
                    content = f.read()
                lanelet_network = parse_opendrive(content)
            except Exception as e2:
                print(f"字符串解析失败: {e2}")
                try:
                    # 方法3：使用额外参数
                    import xml.etree.ElementTree as ET
                    tree = ET.parse(xodr_file_path)
                    root = tree.getroot()
                    lanelet_network = parse_opendrive(root)
                except Exception as e3:
                    print(f"XML元素解析失败: {e3}")
                    raise Exception(f"所有解析方法都失败了: {e1}, {e2}, {e3}")
        
        if not lanelet_network:
            raise ValueError("解析得到空的网络")
            
        if not hasattr(lanelet_network, 'lanelets') or not lanelet_network.lanelets:
            raise ValueError("网络中没有车道")
        
        print(f"✓ 成功解析，车道数量: {len(lanelet_network.lanelets)}")
        
        # 尝试写入文件
        return write_with_commonroad(lanelet_network, output_osm_file_path)
        
    except Exception as e:
        print(f"❌ opendrive2lanelet 转换失败: {e}")
        return False

def convert_with_sumo_and_manual(xodr_file_path, output_osm_file_path):
    """使用SUMO转换然后手动转换为Lanelet2格式"""
    try:
        print("正在尝试使用 SUMO + 手动转换...")
        
        import subprocess
        import tempfile
        
        # 检查 netconvert 是否可用
        try:
            result = subprocess.run(['netconvert', '--version'], 
                                  capture_output=True, text=True, timeout=5)
            if result.returncode != 0:
                raise FileNotFoundError("netconvert 执行失败")
        except (FileNotFoundError, subprocess.TimeoutExpired):
            print("❌ SUMO netconvert 不可用")
            return False
        
        print("✓ 找到 SUMO netconvert")
        
        # 创建临时文件
        with tempfile.NamedTemporaryFile(suffix='.net.xml', delete=False) as temp_file:
            temp_net_file = temp_file.name
        
        try:
            # 转换为SUMO网络
            print("正在转换为SUMO网络格式...")
            cmd = [
                'netconvert',
                '--opendrive', xodr_file_path,
                '--output-file', temp_net_file,
                '--geometry.min-radius.fix.railways', 'false',
                '--offset.disable-normalization', 'true',
                '--no-internal-links', 'false',
                '--junctions.join', 'false'
            ]
            
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=60)
            if result.returncode != 0:
                print(f"❌ netconvert 转换失败: {result.stderr}")
                return False
            
            print("✓ SUMO网络转换成功")
            
            # 解析SUMO网络文件
            return convert_sumo_to_lanelet2(temp_net_file, output_osm_file_path)
            
        finally:
            # 清理临时文件
            try:
                if os.path.exists(temp_net_file):
                    os.unlink(temp_net_file)
            except:
                pass
                
    except Exception as e:
        print(f"❌ SUMO转换失败: {e}")
        return False

def convert_sumo_to_lanelet2(sumo_net_file, output_osm_file_path):
    """将SUMO网络文件转换为Lanelet2格式"""
    try:
        print("正在将SUMO网络转换为Lanelet2格式...")
        
        # 解析SUMO网络文件
        tree = ET.parse(sumo_net_file)
        root = tree.getroot()
        
        # 创建标准的OSM文件
        return create_standard_osm_file(root, output_osm_file_path)
        
    except Exception as e:
        print(f"❌ SUMO到Lanelet2转换失败: {e}")
        import traceback
        traceback.print_exc()
        return False

def create_standard_osm_file(sumo_root, output_osm_file_path):
    """创建标准的OSM文件"""
    try:
        print("正在创建标准OSM文件...")
        
        # 手动构建OSM XML内容
        osm_content = []
        osm_content.append('<?xml version="1.0" encoding="UTF-8"?>')
        osm_content.append('<osm version="0.6" generator="opendrive2lanelet2-converter">')
        
        node_id = 1
        way_id = 1
        relation_id = 1
        
        # 存储创建的节点
        created_nodes = {}
        
        # 处理节点 - 从交叉口创建
        print("正在处理交叉口...")
        for junction in sumo_root.findall('junction'):
            junction_id = junction.get('id')
            x = float(junction.get('x', 0))
            y = float(junction.get('y', 0))
            
            # 简单的坐标转换 (这里假设是平面坐标)
            lat = y / 111320.0  # 粗略转换
            lon = x / (111320.0 * 0.7)  # 粗略转换，考虑纬度影响
            
            osm_content.append(f'  <node id="{node_id}" version="1" lat="{lat:.7f}" lon="{lon:.7f}"/>')
            created_nodes[junction_id] = node_id
            node_id += 1
        
        print(f"✓ 创建了 {len(created_nodes)} 个节点")
        
        # 处理边（道路）和车道
        lane_count = 0
        print("正在处理道路和车道...")
        
        for edge in sumo_root.findall('edge'):
            edge_id = edge.get('id')
            from_node = edge.get('from')
            to_node = edge.get('to')
            
            # 跳过内部边
            if edge_id.startswith(':'):
                continue
            
            # 处理车道
            for lane in edge.findall('lane'):
                lane_id = lane.get('id')
                shape = lane.get('shape', '')
                
                if not shape:
                    continue
                
                # 解析形状点
                points = []
                try:
                    for point_str in shape.split():
                        coords = point_str.split(',')
                        if len(coords) >= 2:
                            x, y = float(coords[0]), float(coords[1])
                            # 坐标转换
                            lat = y / 111320.0
                            lon = x / (111320.0 * 0.7)
                            points.append((lat, lon))
                except:
                    continue
                
                if len(points) < 2:
                    continue
                
                # 为形状点创建节点
                lane_nodes = []
                for lat, lon in points:
                    osm_content.append(f'  <node id="{node_id}" version="1" lat="{lat:.7f}" lon="{lon:.7f}"/>')
                    lane_nodes.append(node_id)
                    node_id += 1
                
                # 创建中心线way
                osm_content.append(f'  <way id="{way_id}" version="1">')
                for n in lane_nodes:
                    osm_content.append(f'    <nd ref="{n}"/>')
                osm_content.append('    <tag k="type" v="line_thin"/>')
                osm_content.append('    <tag k="subtype" v="dashed"/>')
                osm_content.append('  </way>')
                
                center_way_id = way_id
                way_id += 1
                
                # 创建简化的lanelet relation
                osm_content.append(f'  <relation id="{relation_id}" version="1">')
                osm_content.append(f'    <member type="way" ref="{center_way_id}" role="centerline"/>')
                osm_content.append('    <tag k="type" v="lanelet"/>')
                osm_content.append('    <tag k="subtype" v="road"/>')
                osm_content.append('    <tag k="location" v="urban"/>')
                osm_content.append('    <tag k="one_way" v="yes"/>')
                osm_content.append(f'    <tag k="lane_id" v="{lane_id}"/>')
                osm_content.append(f'    <tag k="edge_id" v="{edge_id}"/>')
                osm_content.append('  </relation>')
                
                relation_id += 1
                lane_count += 1
                
                if lane_count % 10 == 0:
                    print(f"  已处理 {lane_count} 条车道...")
        
        osm_content.append('</osm>')
        
        print(f"✓ 转换了 {lane_count} 条车道")
        
        # 写入文件
        print("正在写入OSM文件...")
        with open(output_osm_file_path, 'w', encoding='utf-8') as f:
            f.write('\n'.join(osm_content))
        
        print(f"✓ 成功写入 Lanelet2 文件")
        return True
        
    except Exception as e:
        print(f"❌ OSM文件创建失败: {e}")
        import traceback
        traceback.print_exc()
        return False

def create_simple_osm_from_xodr(xodr_file_path, output_osm_file_path):
    """直接从OpenDRIVE创建简单的OSM文件"""
    try:
        print("正在尝试直接从OpenDRIVE创建OSM...")
        
        # 解析OpenDRIVE文件
        tree = ET.parse(xodr_file_path)
        root = tree.getroot()
        
        # 构建OSM内容
        osm_content = []
        osm_content.append('<?xml version="1.0" encoding="UTF-8"?>')
        osm_content.append('<osm version="0.6" generator="xodr2lanelet2-direct-converter">')
        
        node_id = 1
        way_id = 1
        relation_id = 1
        
        print("正在处理OpenDRIVE道路...")
        
        # 处理道路
        roads = root.findall('.//road')
        lane_count = 0
        
        for road in roads:
            road_id = road.get('id', 'unknown')
            print(f"  处理道路: {road_id}")
            
            # 获取道路的参考线
            plan_view = road.find('planView')
            if plan_view is None:
                continue
            
            # 简单处理：只取第一个几何元素
            geometry = plan_view.find('geometry')
            if geometry is None:
                continue
            
            # 获取起始点
            s = float(geometry.get('s', 0))
            x = float(geometry.get('x', 0))
            y = float(geometry.get('y', 0))
            hdg = float(geometry.get('hdg', 0))
            length = float(geometry.get('length', 100))
            
            # 创建简单的直线道路 (这是一个简化版本)
            import math
            
            # 起点
            start_lat = y / 111320.0
            start_lon = x / (111320.0 * 0.7)
            
            # 终点 (沿航向角度)
            end_x = x + length * math.cos(hdg)
            end_y = y + length * math.sin(hdg)
            end_lat = end_y / 111320.0
            end_lon = end_x / (111320.0 * 0.7)
            
            # 创建节点
            start_node_id = node_id
            osm_content.append(f'  <node id="{node_id}" version="1" lat="{start_lat:.7f}" lon="{start_lon:.7f}"/>')
            node_id += 1
            
            end_node_id = node_id
            osm_content.append(f'  <node id="{node_id}" version="1" lat="{end_lat:.7f}" lon="{end_lon:.7f}"/>')
            node_id += 1
            
            # 处理车道段
            lane_sections = road.findall('.//laneSection')
            for lane_section in lane_sections:
                lanes = lane_section.findall('.//lane')
                for lane in lanes:
                    lane_id = lane.get('id', '0')
                    lane_type = lane.get('type', 'driving')
                    
                    # 只处理行车道
                    if lane_type != 'driving' or lane_id == '0':
                        continue
                    
                    # 创建way
                    osm_content.append(f'  <way id="{way_id}" version="1">')
                    osm_content.append(f'    <nd ref="{start_node_id}"/>')
                    osm_content.append(f'    <nd ref="{end_node_id}"/>')
                    osm_content.append('    <tag k="type" v="line_thin"/>')
                    osm_content.append('    <tag k="subtype" v="solid"/>')
                    osm_content.append('  </way>')
                    
                    center_way_id = way_id
                    way_id += 1
                    
                    # 创建lanelet relation
                    osm_content.append(f'  <relation id="{relation_id}" version="1">')
                    osm_content.append(f'    <member type="way" ref="{center_way_id}" role="centerline"/>')
                    osm_content.append('    <tag k="type" v="lanelet"/>')
                    osm_content.append('    <tag k="subtype" v="road"/>')
                    osm_content.append('    <tag k="location" v="urban"/>')
                    osm_content.append('    <tag k="one_way" v="yes"/>')
                    osm_content.append(f'    <tag k="lane_id" v="{lane_id}"/>')
                    osm_content.append(f'    <tag k="road_id" v="{road_id}"/>')
                    osm_content.append('  </relation>')
                    
                    relation_id += 1
                    lane_count += 1
        
        osm_content.append('</osm>')
        
        print(f"✓ 创建了 {lane_count} 条车道")
        
        # 写入文件
        with open(output_osm_file_path, 'w', encoding='utf-8') as f:
            f.write('\n'.join(osm_content))
        
        print(f"✓ 直接转换成功")
        return True
        
    except Exception as e:
        print(f"❌ 直接转换失败: {e}")
        import traceback
        traceback.print_exc()
        return False

def write_with_commonroad(lanelet_network, output_osm_file_path):
    """使用CommonRoad写入器保存"""
    try:
        from commonroad.common.file_writer import CommonRoadFileWriter, OverwriteExistingFile
        from commonroad.scenario.scenario import Scenario
        from commonroad.planning.planning_problem import PlanningProblemSet
        
        # 创建场景ID
        scenario_id = create_scenario_id("customMapConversion", "2024")
        
        # 创建场景
        import inspect
        scenario_params = inspect.signature(Scenario.__init__).parameters
        
        scenario_kwargs = {
            'dt': 0.1,
            'lanelet_network': lanelet_network
        }
        
        if 'benchmark_id' in scenario_params:
            scenario_kwargs['benchmark_id'] = scenario_id
        elif 'scenario_id' in scenario_params:
            scenario_kwargs['scenario_id'] = scenario_id
        
        output_scenario = Scenario(**scenario_kwargs)
        planning_problem_set = PlanningProblemSet()

        # 写入文件
        file_writer = CommonRoadFileWriter(
            scenario=output_scenario,
            planning_problem_set=planning_problem_set,
            author="Map Converter",
            affiliation="CARLA",
            source="CARLA OpenDRIVE to Lanelet2 Conversion Script",
        )

        file_writer.write_to_file(output_osm_file_path, OverwriteExistingFile.ALWAYS)
        print(f"✓ 使用CommonRoad成功保存到 {output_osm_file_path}")
        return True
        
    except Exception as e:
        print(f"CommonRoad写入失败: {e}")
        return False

def create_scenario_id(name, year):
    """创建场景ID"""
    try:
        from commonroad.scenario.scenario import ScenarioID
        return ScenarioID(cooperative=False, country_id="ZAM", map_name=name, map_id=1)
    except ImportError:
        try:
            from commonroad.common.benchmark_id import BenchmarkID
            return BenchmarkID(name, year)
        except ImportError:
            return None

def convert_xodr_to_lanelet2(xodr_file_path, output_osm_file_path):
    """主转换函数"""
    if not os.path.exists(xodr_file_path):
        print(f"❌ 输入文件未找到: {xodr_file_path}")
        return False

    print(f"开始转换: {xodr_file_path} -> {output_osm_file_path}")
    
    # 验证输入文件
    if not validate_xodr_file(xodr_file_path):
        print("❌ 输入文件验证失败")
        return False
    
    print("-" * 30)
    
    # 尝试方法1：opendrive2lanelet
    if convert_with_opendrive2lanelet(xodr_file_path, output_osm_file_path):
        return True
    
    print("-" * 30)
    
    # 尝试方法2：SUMO + 手动转换
    if convert_with_sumo_and_manual(xodr_file_path, output_osm_file_path):
        return True
    
    print("-" * 30)
    
    # 尝试方法3：直接从OpenDRIVE转换
    if create_simple_osm_from_xodr(xodr_file_path, output_osm_file_path):
        return True
    
    print("❌ 所有转换方法都失败了")
    return False

def main():
    """主函数"""
    print("=== 修复版 OpenDRIVE to Lanelet2 转换工具 ===")
    
    # 配置文件路径
    input_xodr_file = "0516_1.xodr"
    output_osm_file = "0516_1.osm"

    print(f"输入文件: {input_xodr_file}")
    print(f"输出文件: {output_osm_file}")
    print("=" * 50)
    
    if not os.path.exists(input_xodr_file):
        print(f"❌ 输入文件不存在: {input_xodr_file}")
        return
    
    # 检查输出目录
    output_dir = os.path.dirname(output_osm_file)
    if output_dir and not os.path.exists(output_dir):
        print(f"创建输出目录: {output_dir}")
        os.makedirs(output_dir, exist_ok=True)
    
    # 执行转换
    success = convert_xodr_to_lanelet2(input_xodr_file, output_osm_file)
    
    # 验证结果
    print("=" * 50)
    if os.path.exists(output_osm_file):
        file_size = os.path.getsize(output_osm_file)
        print(f"✓ 输出文件已创建: {output_osm_file}")
        print(f"  文件大小: {file_size} 字节")
        
        if file_size > 0:
            print("🎉 转换成功！")
            print("\n📋 文件验证:")
            # 验证生成的OSM文件
            try:
                tree = ET.parse(output_osm_file)
                root = tree.getroot()
                nodes = root.findall('node')
                ways = root.findall('way')
                relations = root.findall('relation')
                print(f"  节点数量: {len(nodes)}")
                print(f"  道路数量: {len(ways)}")
                print(f"  车道数量: {len(relations)}")
                print("  ✓ OSM文件格式正确")
            except Exception as e:
                print(f"  ⚠️  OSM文件可能有格式问题: {e}")
        else:
            print("⚠️  输出文件为空")
    else:
        print(f"❌ 输出文件未创建")
    
    if success:
        print("\n📋 使用建议:")
        print("1. 用JOSM编辑器打开 .osm 文件查看")
        print("2. 检查车道连接关系")
        print("3. 根据需要调整车道属性")
    else:
        print("\n💡 故障排除:")
        print("1. 检查OpenDRIVE文件格式")
        print("2. 尝试其他OpenDRIVE文件测试")
        print("3. 检查软件依赖是否完整")
    
    print("-" * 50)

if __name__ == "__main__":
    main()
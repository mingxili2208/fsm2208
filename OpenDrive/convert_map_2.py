import os
import sys
import xml.etree.ElementTree as ET

def check_opendrive2lanelet_structure():
    """检查opendrive2lanelet库的实际结构"""
    try:
        import opendrive2lanelet
        print("正在检查 opendrive2lanelet 库结构...")
        
        # 检查主模块
        print("主模块属性:")
        for attr in dir(opendrive2lanelet):
            if not attr.startswith('_'):
                print(f"  - {attr}")
        
        # 检查子模块
        import pkgutil
        print("\n子模块:")
        for importer, modname, ispkg in pkgutil.iter_modules(opendrive2lanelet.__path__):
            print(f"  - {modname} {'(包)' if ispkg else ''}")
            
        # 尝试不同的导入路径
        possible_imports = [
            'opendrive2lanelet.opendriveparser.parser',
            'opendrive2lanelet.opendrive_parser.parser', 
            'opendrive2lanelet.parser',
            'opendrive2lanelet.conversion',
            'opendrive2lanelet.converter'
        ]
        
        print("\n尝试导入解析器:")
        for import_path in possible_imports:
            try:
                module = __import__(import_path, fromlist=[''])
                print(f"  ✓ {import_path}")
                print(f"    可用函数: {[f for f in dir(module) if not f.startswith('_')]}")
            except ImportError as e:
                print(f"  ❌ {import_path}: {e}")
        
        return True
        
    except ImportError:
        print("❌ opendrive2lanelet 未安装")
        return False

def convert_with_opendrive2lanelet_v2(xodr_file_path, output_osm_file_path):
    """使用opendrive2lanelet库进行转换 - 改进版"""
    try:
        print("正在尝试使用 opendrive2lanelet (改进版)...")
        
        # 首先检查库结构
        check_opendrive2lanelet_structure()
        print("-" * 20)
        
        # 尝试更多可能的导入方式
        parse_function = None
        
        # 方法1: 尝试标准导入
        try:
            from opendrive2lanelet.opendriveparser.parser import parse_opendrive
            parse_function = parse_opendrive
            print("✓ 成功导入 parse_opendrive (标准路径)")
        except ImportError:
            pass
        
        # 方法2: 尝试备用路径
        if parse_function is None:
            try:
                import opendrive2lanelet.opendriveparser as parser_module
                if hasattr(parser_module, 'parse_opendrive'):
                    parse_function = parser_module.parse_opendrive
                    print("✓ 找到 parse_opendrive (备用路径1)")
            except ImportError:
                pass
        
        # 方法3: 尝试主模块
        if parse_function is None:
            try:
                import opendrive2lanelet
                if hasattr(opendrive2lanelet, 'parse_opendrive'):
                    parse_function = opendrive2lanelet.parse_opendrive
                    print("✓ 找到 parse_opendrive (主模块)")
            except ImportError:
                pass
        
        # 方法4: 尝试转换器模块
        if parse_function is None:
            try:
                from opendrive2lanelet import converter
                if hasattr(converter, 'parse_opendrive'):
                    parse_function = converter.parse_opendrive
                    print("✓ 找到 parse_opendrive (转换器模块)")
                elif hasattr(converter, 'convert'):
                    parse_function = converter.convert
                    print("✓ 找到 convert 函数")
            except ImportError:
                pass
        
        # 方法5: 尝试通过网络接口
        if parse_function is None:
            try:
                from opendrive2lanelet.network import Network
                print("✓ 找到 Network 类")
                # 这种情况下需要不同的处理方式
                return convert_using_network_class(xodr_file_path, output_osm_file_path)
            except ImportError:
                pass
        
        if parse_function is None:
            raise ImportError("无法找到任何有效的解析函数")
        
        # 尝试解析
        print("正在解析OpenDRIVE文件...")
        
        try:
            # 直接文件路径
            result = parse_function(xodr_file_path)
        except Exception as e1:
            print(f"文件路径解析失败: {e1}")
            try:
                # 读取文件内容
                with open(xodr_file_path, 'r', encoding='utf-8') as f:
                    content = f.read()
                result = parse_function(content)
            except Exception as e2:
                print(f"文件内容解析失败: {e2}")
                raise Exception(f"所有解析方法都失败了: {e1}, {e2}")
        
        # 检查结果
        if result is None:
            raise ValueError("解析结果为空")
        
        print(f"✓ 解析成功，结果类型: {type(result)}")
        
        # 尝试提取或转换为CommonRoad格式
        lanelet_network = None
        
        # 如果结果已经是CommonRoad网络
        if hasattr(result, 'lanelets'):
            lanelet_network = result
        # 如果结果有网络属性
        elif hasattr(result, 'lanelet_network'):
            lanelet_network = result.lanelet_network
        # 如果结果是元组
        elif isinstance(result, tuple) and len(result) > 0:
            lanelet_network = result[0]
        else:
            # 尝试直接使用result
            lanelet_network = result
        
        if lanelet_network is None:
            raise ValueError("无法从解析结果中提取lanelet网络")
        
        if not hasattr(lanelet_network, 'lanelets') or not lanelet_network.lanelets:
            raise ValueError("lanelet网络为空或无车道")
        
        print(f"✓ 成功提取网络，车道数量: {len(lanelet_network.lanelets)}")
        
        # 尝试写入文件
        return write_with_commonroad(lanelet_network, output_osm_file_path)
        
    except Exception as e:
        print(f"❌ opendrive2lanelet 转换失败: {e}")
        import traceback
        traceback.print_exc()
        return False

def convert_using_network_class(xodr_file_path, output_osm_file_path):
    """使用Network类进行转换"""
    try:
        from opendrive2lanelet.network import Network
        
        print("正在使用Network类转换...")
        
        # 创建Network实例
        network = Network()
        
        # 尝试加载OpenDRIVE文件
        if hasattr(network, 'load_opendrive'):
            network.load_opendrive(xodr_file_path)
        elif hasattr(network, 'from_opendrive'):
            network = Network.from_opendrive(xodr_file_path)
        else:
            raise AttributeError("Network类没有预期的加载方法")
        
        # 尝试导出为CommonRoad格式
        if hasattr(network, 'export_commonroad'):
            lanelet_network = network.export_commonroad()
        elif hasattr(network, 'to_commonroad'):
            lanelet_network = network.to_commonroad()
        else:
            # 直接使用网络对象
            lanelet_network = network
        
        if not hasattr(lanelet_network, 'lanelets') or not lanelet_network.lanelets:
            raise ValueError("网络转换失败或无车道")
        
        print(f"✓ Network类转换成功，车道数量: {len(lanelet_network.lanelets)}")
        
        return write_with_commonroad(lanelet_network, output_osm_file_path)
        
    except Exception as e:
        print(f"❌ Network类转换失败: {e}")
        return False

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
            return create_standard_osm_file_from_sumo(temp_net_file, output_osm_file_path)
            
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

def create_standard_osm_file_from_sumo(sumo_net_file, output_osm_file_path):
    """从SUMO网络创建标准OSM文件"""
    try:
        print("正在将SUMO网络转换为Lanelet2格式...")
        
        # 解析SUMO网络文件
        tree = ET.parse(sumo_net_file)
        root = tree.getroot()
        
        # 构建OSM内容
        osm_content = []
        osm_content.append('<?xml version="1.0" encoding="UTF-8"?>')
        osm_content.append('<osm version="0.6" generator="sumo2lanelet2-converter">')
        
        node_id = 1
        way_id = 1
        relation_id = 1
        
        # 存储创建的节点
        created_nodes = {}
        
        # 处理节点 - 从交叉口创建
        print("正在处理交叉口...")
        for junction in root.findall('junction'):
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
        
        for edge in root.findall('edge'):
            edge_id = edge.get('id')
            
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
    
    # 尝试方法1：opendrive2lanelet (改进版)
    if convert_with_opendrive2lanelet_v2(xodr_file_path, output_osm_file_path):
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
    print("=== 调试版 OpenDRIVE to Lanelet2 转换工具 ===")
    
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
    
    print("-" * 50)

if __name__ == "__main__":
    main()
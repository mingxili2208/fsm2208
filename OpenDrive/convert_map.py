import os
import sys

def convert_xodr_to_lanelet2(xodr_file_path, output_osm_file_path):
    """
    将OpenDRIVE文件 (.xodr) 转换为Lanelet2地图文件 (.osm)。

    :param xodr_file_path: 输入的OpenDRIVE文件路径。
    :param output_osm_file_path: 输出的Lanelet2 OSM文件保存路径。
    """
    if not os.path.exists(xodr_file_path):
        print(f"错误：输入文件未找到于 {xodr_file_path}")
        return

    print(f"开始将 {xodr_file_path} 转换为Lanelet2格式...")

    try:
        # 方法1：尝试使用 opendrive2lanelet 库（推荐）
        print("正在尝试使用 opendrive2lanelet 库...")
        try:
            from opendrive2lanelet.opendriveparser.parser import parse_opendrive
            from opendrive2lanelet.io import write_lanelet2_map
            
            # 解析OpenDRIVE文件
            print("正在解析OpenDRIVE文件...")
            lanelet_network = parse_opendrive(xodr_file_path)
            
            if not lanelet_network or not lanelet_network.lanelets:
                print("警告：解析得到的LaneletNetwork为空或不包含任何lanelet。")
                return
            
            print(f"成功解析OpenDRIVE文件")
            print(f"网络中的车道(Lanelet)数量: {len(lanelet_network.lanelets)}")
            
            # 写入Lanelet2格式
            print("正在写入Lanelet2文件...")
            write_lanelet2_map(lanelet_network, output_osm_file_path)
            
            print(f"成功转换并保存Lanelet2地图到 {output_osm_file_path}")
            return
            
        except ImportError as e:
            print(f"opendrive2lanelet 库未安装: {e}")
            print("正在尝试其他方法...")
        
        # 方法2：尝试使用 commonroad-scenario-designer
        print("正在尝试使用 commonroad-scenario-designer...")
        try:
            from crdesigner.map_conversion.opendrive.opendrive_parser.parser import parse_opendrive
            from commonroad.common.file_writer import CommonRoadFileWriter, OverwriteExistingFile
            from commonroad.scenario.scenario import Scenario
            from commonroad.planning.planning_problem import PlanningProblemSet
            
            # 解析OpenDRIVE文件
            print("正在解析OpenDRIVE文件...")
            lanelet_network = parse_opendrive(xodr_file_path)
            
            if not lanelet_network or not lanelet_network.lanelets:
                print("警告：解析得到的LaneletNetwork为空或不包含任何lanelet。")
                return
            
            print(f"成功解析OpenDRIVE文件")
            print(f"网络中的车道(Lanelet)数量: {len(lanelet_network.lanelets)}")
            
            # 创建场景ID
            scenario_id = create_scenario_id("customMapConversion", "2024")
            
            # 创建场景
            print("正在创建场景...")
            import inspect
            scenario_params = inspect.signature(Scenario.__init__).parameters
            
            if 'benchmark_id' in scenario_params:
                output_scenario = Scenario(
                    dt=0.1, 
                    benchmark_id=scenario_id,
                    lanelet_network=lanelet_network
                )
            elif 'scenario_id' in scenario_params:
                output_scenario = Scenario(
                    dt=0.1, 
                    scenario_id=scenario_id,
                    lanelet_network=lanelet_network
                )
            else:
                output_scenario = Scenario(
                    dt=0.1,
                    lanelet_network=lanelet_network
                )

            # 创建空的规划问题集
            planning_problem_set = PlanningProblemSet()

            # 写入文件
            print("正在写入Lanelet2文件...")
            file_writer = CommonRoadFileWriter(
                scenario=output_scenario,
                planning_problem_set=planning_problem_set,
                author="Map Converter",
                affiliation="CARLA",
                source="CARLA OpenDRIVE to Lanelet2 Conversion Script",
            )

            file_writer.write_to_file(output_osm_file_path, OverwriteExistingFile.ALWAYS)
            print(f"成功转换并保存Lanelet2地图到 {output_osm_file_path}")
            return
            
        except ImportError as e:
            print(f"commonroad-scenario-designer 解析器未找到: {e}")
        except Exception as e:
            print(f"commonroad-scenario-designer 解析失败: {e}")
        
        # 方法3：尝试使用 SUMO netconvert
        print("正在尝试使用 SUMO netconvert...")
        try:
            import subprocess
            import tempfile
            
            # 检查 netconvert 是否可用
            result = subprocess.run(['netconvert', '--version'], 
                                  capture_output=True, text=True)
            if result.returncode != 0:
                raise FileNotFoundError("netconvert not found")
            
            print("找到 SUMO netconvert")
            
            # 创建临时文件
            with tempfile.NamedTemporaryFile(suffix='.net.xml', delete=False) as temp_file:
                temp_net_file = temp_file.name
            
            # 转换为SUMO网络
            print("正在转换为SUMO网络格式...")
            cmd = [
                'netconvert',
                '--opendrive', xodr_file_path,
                '--output-file', temp_net_file,
                '--geometry.min-radius.fix.railways', 'false',
                '--offset.disable-normalization', 'true',
                '--no-internal-links', 'false'
            ]
            
            result = subprocess.run(cmd, capture_output=True, text=True)
            if result.returncode != 0:
                print(f"netconvert 转换失败: {result.stderr}")
                return
            
            print("SUMO网络转换成功，正在转换为Lanelet2...")
            
            # 这里需要额外的步骤将SUMO网络转换为Lanelet2
            # 由于这比较复杂，我们提供指导
            print(f"SUMO网络文件已创建: {temp_net_file}")
            print("请使用专门的工具将SUMO网络转换为Lanelet2格式")
            print("或者安装 opendrive2lanelet 库以获得更好的支持")
            
            # 清理临时文件
            try:
                os.unlink(temp_net_file)
            except:
                pass
                
        except (FileNotFoundError, subprocess.SubprocessError) as e:
            print(f"SUMO netconvert 不可用: {e}")
        
        # 如果所有方法都失败
        print("\n所有转换方法都失败了。请尝试以下解决方案：")
        print("1. 安装 opendrive2lanelet:")
        print("   pip install opendrive2lanelet")
        print("2. 确保 commonroad-scenario-designer 已正确安装:")
        print("   pip install commonroad-scenario-designer")
        print("3. 安装 SUMO 并确保 netconvert 在 PATH 中")
        print("4. 检查 OpenDRIVE 文件是否有效")

    except Exception as e:
        print(f"转换过程中发生错误: {e}")
        import traceback
        traceback.print_exc()

def create_scenario_id(name, year):
    """创建场景ID，兼容不同版本的commonroad-io"""
    try:
        from commonroad.scenario.scenario import ScenarioID
        return ScenarioID(cooperative=False, country_id="ZAM", map_name=name, map_id=1)
    except ImportError:
        try:
            from commonroad.common.benchmark_id import BenchmarkID
            return BenchmarkID(name, year)
        except ImportError:
            try:
                from commonroad.scenario.benchmark_id import BenchmarkID
                return BenchmarkID(name, year)
            except ImportError:
                return None

def check_dependencies():
    """检查依赖库的安装状态"""
    print("=== 依赖检查 ===")
    
    dependencies = [
        ("opendrive2lanelet", "opendrive2lanelet"),
        ("commonroad-io", "commonroad"),
        ("commonroad-scenario-designer", "crdesigner"),
    ]
    
    available = []
    missing = []
    
    for name, module in dependencies:
        try:
            __import__(module)
            available.append(name)
            print(f"✓ {name} - 已安装")
        except ImportError:
            missing.append(name)
            print(f"✗ {name} - 未安装")
    
    print(f"\n可用库: {len(available)}")
    print(f"缺失库: {len(missing)}")
    
    if missing:
        print(f"\n建议安装缺失的库:")
        for lib in missing:
            if lib == "opendrive2lanelet":
                print(f"  pip install {lib}")
            elif lib == "commonroad-scenario-designer":
                print(f"  pip install {lib}")
    
    return len(available) > 0

def main():
    """主函数"""
    print("=== OpenDRIVE to Lanelet2 转换工具 ===")
    
    # 检查依赖
    if not check_dependencies():
        print("\n错误：没有找到任何可用的转换库！")
        print("请至少安装以下库之一：")
        print("  pip install opendrive2lanelet")
        print("  pip install commonroad-scenario-designer")
        return
    
    # 配置文件路径
    input_xodr_file = "0516_1.xodr"
    output_osm_file = "0516_1.osm"

    print(f"\n输入文件: {input_xodr_file}")
    print(f"输出文件: {output_osm_file}")
    print("-" * 50)
    
    if not os.path.exists(input_xodr_file):
        print(f"错误: 输入的OpenDRIVE文件 '{input_xodr_file}' 未找到。")
        return
    
    # 检查输出目录
    output_dir = os.path.dirname(output_osm_file)
    if output_dir and not os.path.exists(output_dir):
        print(f"创建输出目录: {output_dir}")
        os.makedirs(output_dir, exist_ok=True)
    
    # 执行转换
    convert_xodr_to_lanelet2(input_xodr_file, output_osm_file)
    
    # 验证结果
    if os.path.exists(output_osm_file):
        file_size = os.path.getsize(output_osm_file)
        print(f"\n✓ 输出文件已创建: {output_osm_file}")
        print(f"  文件大小: {file_size} 字节")
    else:
        print(f"\n✗ 输出文件未创建")
    
    print("-" * 50)
    print("转换完成!")

if __name__ == "__main__":
    main()
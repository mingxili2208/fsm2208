import lanelet2
from lanelet2.io import Origin, load
from lanelet2.projection import UtmProjector # 或者适合你地图的投影方式
from lanelet2.core import AttributeMap, BasicPoint2d
from lanelet2.routing import RoutingGraph
from lanelet2.traffic_rules import Locations, Participants, TrafficRulesFactory

osm_file_path = "0516_1.osm" # 替换为你的文件路径

# CARLA地图通常是局部坐标系。你需要一个原点进行投影。
# 如果你的OpenDRIVE中有<geoReference>标签，可以尝试从中提取原点。
# 否则，对于CARLA的局部地图，(0,0) 或场景的某个参考点可以作为原点。
# 注意：如果你的 .xodr 没有地理参考，转换器可能已经假设了一个原点或使用了局部平面。
# 你需要确保加载时使用的投影与转换时一致。
# 尝试使用一个简单的原点，如果加载失败或坐标看起来不对，可能需要调整。
try:
    # 对于没有地理参考的局部地图，使用一个简单的原点
    projector = UtmProjector(Origin(0, 0, 0)) 
    # 或者如果你的地图数据本身是WGS84 lat/lon, 但.osm文件通常已投影
    # projector = UtmProjector(lanelet2.io.Origin(lat, lon)) # 如果你知道原始的地理原点

    lanelet_map = load(osm_file_path, projector)
    print(f"成功加载地图: {osm_file_path}")
    print(f"地图中Lanelet数量: {len(lanelet_map.laneletLayer)}")
    print(f"地图中Area数量: {len(lanelet_map.areaLayer)}")
    print(f"地图中RegulatoryElement数量: {len(lanelet_map.regulatoryElementLayer)}")

    # 1. 基本的健全性检查
    if not lanelet_map.laneletLayer:
        print("警告: 地图中没有车道 (Lanelets)!")

    # 2. 检查交通规则的有效性 (示例)
    # 选择交通规则 (例如，德国，车辆)
    traffic_rules = TrafficRulesFactory.create(Locations.Germany, Participants.Vehicle)
    
    # 3. 构建路径规划图
    # 这可以帮助发现连接性问题
    try:
        routing_graph = RoutingGraph(lanelet_map, traffic_rules)
        print("路径规划图构建成功。")
        # 你可以进一步检查图中的连通区域等
        # num_connected_components = routing_graph.numConnectedComponents()
        # print(f"图中连通分量数量: {num_connected_components}")
    except Exception as e:
        print(f"构建路径规划图失败: {e}")


    # 4. 遍历Lanelets并检查特定属性 (示例)
    for lanelet in lanelet_map.laneletLayer:
        if "speed_limit" not in lanelet.attributes:
            # print(f"警告: Lanelet {lanelet.id} 没有speed_limit属性。") # 这可能很常见，因为限速由RegElem定义
            pass
        if lanelet.attributes.get("subtype") == "road" and not traffic_rules.canPass(lanelet):
             print(f"警告: 根据交通规则，车辆不能通过ID为 {lanelet.id} 的 'road' 类型Lanelet。")
        
        # 检查是否有前驱或后继 (对于非终点/起点车道)
        # predecessors = routing_graph.previous(lanelet)
        # successors = routing_graph.following(lanelet)
        # if not predecessors and not successors and len(lanelet_map.laneletLayer)>1:
        #    print(f"警告: Lanelet {lanelet.id} 既没有前驱也没有后继，可能是孤立的。")


except Exception as e:
    print(f"加载或验证地图时发生错误: {e}")
    import traceback
    traceback.print_exc()
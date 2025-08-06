import json
import os

def diagnose_log_files(log_dir='./ros2_topic_logs'):
    """诊断日志文件的格式和内容"""
    print(f"检查日志目录: {log_dir}")
    
    if not os.path.exists(log_dir):
        print(f"错误: 目录不存在 {log_dir}")
        return
    
    files = os.listdir(log_dir)
    print(f"找到文件: {files}")
    
    # 找到最新时间戳
    timestamps = set()
    for file in files:
        if '_' in file and file.endswith('.txt'):
            parts = file.split('_')
            if len(parts) >= 3:
                timestamp = f"{parts[-2]}_{parts[-1].replace('.txt', '')}"
                timestamps.add(timestamp)
    
    if not timestamps:
        print("未找到有效的日志文件")
        return
    
    latest_timestamp = sorted(timestamps)[-1]
    print(f"最新时间戳: {latest_timestamp}")
    
    # 检查每个文件
    file_types = ['behavior_planning_path', 'trajectory', 'start_planner', 'transformed_with_covariance']
    
    for file_type in file_types:
        filepath = f'{log_dir}/{file_type}_{latest_timestamp}.txt'
        print(f"\n=== 检查文件: {filepath} ===")
        
        if not os.path.exists(filepath):
            print(f"文件不存在: {filepath}")
            continue
            
        try:
            with open(filepath, 'r', encoding='utf-8') as f:
                lines = f.readlines()
                print(f"文件行数: {len(lines)}")
                
                if len(lines) == 0:
                    print("文件为空")
                    continue
                
                # 检查前几行的格式
                for i, line in enumerate(lines[:3]):
                    line = line.strip()
                    if line:
                        print(f"第{i+1}行长度: {len(line)}")
                        print(f"第{i+1}行前100字符: {line[:100]}")
                        try:
                            data = json.loads(line)
                            print(f"第{i+1}行JSON解析成功")
                            print(f"顶级键: {list(data.keys())}")
                            
                            # 特别检查transformed_with_covariance的结构
                            if file_type == 'transformed_with_covariance':
                                print("transformed_with_covariance 数据结构:")
                                if 'data' in data:
                                    print(f"  data键存在，包含: {list(data['data'].keys())}")
                                    if 'header' in data['data']:
                                        print(f"  header键存在，包含: {list(data['data']['header'].keys())}")
                                    if 'pose' in data['data']:
                                        print(f"  pose键存在，包含: {list(data['data']['pose'].keys())}")
                                else:
                                    print("  缺少data键")
                                    
                        except json.JSONDecodeError as e:
                            print(f"第{i+1}行JSON解析失败: {e}")
                        except Exception as e:
                            print(f"第{i+1}行处理异常: {e}")
                        
                        if i == 0:  # 只显示第一行的完整内容
                            print(f"第1行完整内容:\n{line}")
                        print("-" * 50)
                        
        except Exception as e:
            print(f"读取文件时出错: {e}")

if __name__ == '__main__':
    diagnose_log_files()
from flask import Flask, render_template, request, jsonify
import os
import threading
import datetime
import time
import json
import logging
import sys
from werkzeug.serving import run_simple

# 导入您的主要程序模块
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
import auto_tracker_data_process as tracker

# 创建Flask应用
app = Flask(__name__)

# 全局变量存储系统状态
system_state = {
    "trackerInitialized": False,
    "serialConnected": False,
    "isRecording": False,
    "recordedDataPoints": 0,
    "eulerPoints": 0,
    "outputDirectory": "",
    "logMessages": []
}

# 全局变量存储系统实例
tracker_system = None
recording_thread = None

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# 自定义日志处理器，将日志同时发送到系统状态
class WebLogHandler(logging.Handler):
    def emit(self, record):
        log_entry = self.format(record)
        log_type = record.levelname.lower()
        system_state["logMessages"].append({"message": log_entry, "type": log_type})

weblog_handler = WebLogHandler()
weblog_handler.setFormatter(logging.Formatter('%(levelname)s - %(message)s'))
logger.addHandler(weblog_handler)

@app.route('/')
def index():
    """渲染主界面"""
    return render_template('index.html')

@app.route('/api/status')
def get_status():
    """返回当前系统状态"""
    return jsonify(system_state)

@app.route('/api/initialize', methods=['POST'])
def initialize_tracker():
    """初始化追踪器"""
    global tracker_system, system_state
    
    data = request.json
    tracker_name = data.get('trackerName', 'tracker_1')
    serial_port = data.get('serialPort', '/dev/ttyUSB0')
    baud_rate = int(data.get('baudRate', 9600))
    
    logger.info(f"Initializing tracker system: {tracker_name} on port {serial_port} at {baud_rate} baud...")
    
    try:
        # 创建时间戳目录
        current_time = datetime.datetime.now()
        timestamp = current_time.strftime("%Y-%B-%d-%a-%H-%M-%S")
        output_dir = f"tracker_data_process_{timestamp}"
        
        # 更新全局状态
        system_state["outputDirectory"] = output_dir
        
        # 创建追踪器系统实例
        tracker_system = tracker.TrackerSystem(
            serial_port=serial_port, 
            baud_rate=baud_rate, 
            tracker_name=tracker_name
        )
        
        # 更新状态
        system_state["trackerInitialized"] = True
        system_state["serialConnected"] = True
        
        logger.info(f"Tracker '{tracker_name}' successfully initialized")
        logger.info(f"Output directory created: {output_dir}")
        
        return jsonify({"success": True, "message": "Tracker initialized successfully"})
    
    except Exception as e:
        logger.error(f"Error initializing tracker: {e}")
        return jsonify({"success": False, "message": f"Error: {str(e)}"})

@app.route('/api/start_recording', methods=['POST'])
def start_recording():
    """开始连续记录数据"""
    global tracker_system, system_state, recording_thread
    
    if not system_state["trackerInitialized"]:
        return jsonify({"success": False, "message": "Please initialize the tracker first"})
    
    try:
        # 设置记录标志
        system_state["isRecording"] = True
        tracker_system.recording = True
        
        logger.info("Starting continuous recording (laser_tracker.csv)...")
        
        # 更新前端状态
        return jsonify({"success": True, "message": "Recording started"})
    
    except Exception as e:
        logger.error(f"Error starting recording: {e}")
        return jsonify({"success": False, "message": f"Error: {str(e)}"})

@app.route('/api/stop_recording', methods=['POST'])
def stop_recording():
    """停止连续记录数据"""
    global tracker_system, system_state
    
    if not system_state["isRecording"]:
        return jsonify({"success": False, "message": "Not currently recording"})
    
    try:
        # 停止记录
        system_state["isRecording"] = False
        if tracker_system:
            tracker_system.recording = False
        
        logger.info("Stopped recording")
        
        # 更新记录点数
        if hasattr(tracker_system, 'recorded_data'):
            system_state["recordedDataPoints"] = len(tracker_system.recorded_data)
        
        return jsonify({"success": True, "message": "Recording stopped"})
    
    except Exception as e:
        logger.error(f"Error stopping recording: {e}")
        return jsonify({"success": False, "message": f"Error: {str(e)}"})

@app.route('/api/record_euler', methods=['POST'])
def record_euler():
    """记录单个欧拉点数据"""
    global tracker_system, system_state
    
    if not system_state["trackerInitialized"]:
        return jsonify({"success": False, "message": "Please initialize the tracker first"})
    
    data = request.json
    angle = float(data.get('angle', 0))
    
    try:
        # 请求当前Vive Tracker姿态
        cam_coord = tracker_system.tracker.get_pose_euler()
        timestamp = time.time()
        
        # 记录欧拉角数据
        tracker_system.euler_data.append((timestamp, cam_coord, angle))
        
        # 添加到euler.csv
        with open(tracker.EULER_CSV, "a") as euler_file:
            euler_file.write(f"{timestamp:.3f},{cam_coord[0]:.4f},{cam_coord[1]:.4f},{cam_coord[2]:.4f},"
                             f"{cam_coord[3]:.4f},{cam_coord[4]:.4f},{cam_coord[5]:.4f},{angle:.2f}\n")
        
        logger.info(f"Euler data recorded: timestamp={timestamp:.3f}, "
                  f"x={cam_coord[0]:.4f}, y={cam_coord[1]:.4f}, z={cam_coord[2]:.4f}, "
                  f"roll={cam_coord[3]:.4f}, yaw={cam_coord[4]:.4f}, pitch={cam_coord[5]:.4f}, "
                  f"angle={angle:.2f}")
        
        # 更新欧拉点计数
        system_state["eulerPoints"] += 1
        
        return jsonify({
            "success": True, 
            "message": "Euler point recorded",
            "data": {
                "timestamp": timestamp,
                "x": cam_coord[0],
                "y": cam_coord[1],
                "z": cam_coord[2],
                "roll": cam_coord[3],
                "yaw": cam_coord[4],
                "pitch": cam_coord[5],
                "angle": angle
            }
        })
    
    except Exception as e:
        logger.error(f"Error recording euler point: {e}")
        return jsonify({"success": False, "message": f"Error: {str(e)}"})

@app.route('/api/save_all_data', methods=['POST'])
def save_all_data():
    """保存所有记录的数据到文件"""
    global tracker_system, system_state
    
    if not system_state["trackerInitialized"]:
        return jsonify({"success": False, "message": "Please initialize the tracker first"})
    
    try:
        logger.info("Saving all data to log files...")
        
        # 保存激光追踪器数据
        if tracker_system.recorded_data:
            with open(tracker.LASER_TRACKER_CSV, "w") as log_file:
                log_file.write("Timestamp,Distance_2,Distance_3,X,Y,Z,Roll,Yaw,Pitch\n")
                for timestamp, dis2, dis3, coord in tracker_system.recorded_data:
                    log_entry = f"{timestamp:.3f},{dis2:.3f},{dis3:.3f},{coord[0]:.4f},{coord[1]:.4f},{coord[2]:.4f},{coord[3]:.4f},{coord[4]:.4f},{coord[5]:.4f}\n"
                    log_file.write(log_entry)
            logger.info(f"Laser tracker data saved to {tracker.LASER_TRACKER_CSV}")
            
            system_state["recordedDataPoints"] = len(tracker_system.recorded_data)
        else:
            logger.warning("No laser tracker data recorded.")
        
        return jsonify({
            "success": True, 
            "message": "All data saved successfully",
            "recordedPoints": system_state["recordedDataPoints"],
            "eulerPoints": system_state["eulerPoints"]
        })
    
    except Exception as e:
        logger.error(f"Error saving data: {e}")
        return jsonify({"success": False, "message": f"Error: {str(e)}"})

@app.route('/api/process_data', methods=['POST'])
def process_data():
    """处理激光追踪器数据"""
    data = request.json
    angle_neg = float(data.get('negativeYawOffset', 122))
    angle_pos = float(data.get('positiveYawOffset', 55))
    
    try:
        if os.path.exists(tracker.LASER_TRACKER_CSV):
            logger.info("Processing laser tracker data...")
            
            # 处理激光追踪器数据
            processed_df = tracker.process_laser_tracker_data(
                tracker.LASER_TRACKER_CSV, 
                tracker.CORRECTED_LASER_TRACKER_CSV, 
                angle_neg, 
                angle_pos
            )
            
            if processed_df is not None:
                rows_count = len(processed_df)
                logger.info(f"Processed {rows_count} data points")
                return jsonify({
                    "success": True, 
                    "message": f"Data processed successfully. {rows_count} data points processed.",
                    "dataPoints": rows_count
                })
            else:
                return jsonify({"success": False, "message": "Error processing data"})
        else:
            return jsonify({"success": False, "message": f"Laser tracker data file {tracker.LASER_TRACKER_CSV} not found"})
    
    except Exception as e:
        logger.error(f"Error processing data: {e}")
        return jsonify({"success": False, "message": f"Error: {str(e)}"})

@app.route('/api/calculate_transformations', methods=['POST'])
def calculate_transformations():
    """计算坐标变换矩阵"""
    try:
        if not os.path.exists(tracker.EULER_CSV) or not os.path.exists(tracker.CORRECTED_LASER_TRACKER_CSV):
            missing_files = []
            if not os.path.exists(tracker.EULER_CSV):
                missing_files.append("Euler data (euler.csv)")
            if not os.path.exists(tracker.CORRECTED_LASER_TRACKER_CSV):
                missing_files.append("Corrected laser tracker data (corrected_laser_tracker.csv)")
                
            return jsonify({
                "success": False, 
                "message": f"Missing required files: {', '.join(missing_files)}"
            })
        
        logger.info("Calculating transformation matrices...")
        
        # 这里调用原始代码中的变换计算函数
        if tracker_system:
            # 使用TrackerSystem类的calculate_transformations方法
            tracker_system.calculate_transformations()
            
            # 从结果目录读取矩阵
            T_pos = None
            R_euler = None
            
            try:
                T_pos = np.loadtxt(os.path.join(tracker.result_dir, "T_pos_matrix.txt"))
                R_euler = np.loadtxt(os.path.join(tracker.result_dir, "R_euler_matrix.txt"))
            except:
                pass
            
            return jsonify({
                "success": True,
                "message": "Transformations calculated successfully",
                "T_pos": T_pos.tolist() if T_pos is not None else None,
                "R_euler": R_euler.tolist() if R_euler is not None else None
            })
        else:
            return jsonify({"success": False, "message": "Tracker system not initialized"})
    
    except Exception as e:
        logger.error(f"Error calculating transformations: {e}")
        return jsonify({"success": False, "message": f"Error: {str(e)}"})

@app.route('/api/visualize', methods=['POST'])
def visualize_data():
    """可视化数据点和变换结果"""
    data = request.json
    error_threshold = float(data.get('errorThreshold', 0.05))
    
    try:
        # 从result_dir获取视觉化图像文件路径
        visualization_files = {
            "registration_3d": os.path.join(tracker.img_dir, "registration_visualization.png"),
            "error_distribution": os.path.join(tracker.img_dir, "error_distribution.png"),
            "registration_2d": os.path.join(tracker.img_dir, "registration_2d_view.png")
        }
        
        # 检查文件是否存在
        available_files = {}
        for key, path in visualization_files.items():
            if os.path.exists(path):
                # 将图像转换为base64或提供相对路径
                relative_path = os.path.relpath(path, os.path.dirname(__file__))
                available_files[key] = relative_path
        
        return jsonify({
            "success": True,
            "message": "Visualization data retrieved",
            "visualizations": available_files
        })
    
    except Exception as e:
        logger.error(f"Error retrieving visualization data: {e}")
        return jsonify({"success": False, "message": f"Error: {str(e)}"})

@app.route('/api/get_errors', methods=['GET'])
def get_error_data():
    """获取错误分析数据"""
    try:
        error_csv_path = os.path.join(tracker.result_dir, "registration_errors.csv")
        summary_path = os.path.join(tracker.result_dir, "registration_summary.txt")
        
        error_data = {}
        
        if os.path.exists(error_csv_path):
            # 读取错误CSV文件
            import pandas as pd
            error_df = pd.read_csv(error_csv_path)
            
            # 提取RMSE和高错误点
            rmse = error_df['error'].mean()
            high_error_points = error_df[error_df['exceeds_threshold'] == 1]
            
            error_data["rmse"] = float(rmse)
            error_data["high_error_points"] = high_error_points.to_dict(orient="records")
        
        if os.path.exists(summary_path):
            # 读取摘要文件
            with open(summary_path, 'r') as f:
                summary_text = f.read()
            error_data["summary"] = summary_text
        
        return jsonify({
            "success": True,
            "message": "Error data retrieved",
            "error_data": error_data
        })
    
    except Exception as e:
        logger.error(f"Error retrieving error data: {e}")
        return jsonify({"success": False, "message": f"Error: {str(e)}"})
@app.route('/api/modify_euler', methods=['POST'])
def modify_euler():
    """修改欧拉点数据的角度"""
    data = request.json
    record_id = data.get('recordId')
    new_angle = float(data.get('newAngle', 0))
    
    try:
        # 打开并读取欧拉CSV文件
        euler_df = pd.read_csv(tracker.EULER_CSV)
        
        # 修改对应记录的角度
        if 0 <= record_id < len(euler_df):
            euler_df.at[record_id, 'DesiredAngle'] = new_angle
            
            # 保存修改后的文件
            euler_df.to_csv(tracker.EULER_CSV, index=False)
            
            logger.info(f"Modified Euler record {record_id} angle to {new_angle}")
            
            return jsonify({
                "success": True, 
                "message": f"Angle for record at index {record_id} modified to {new_angle}."
            })
        else:
            return jsonify({"success": False, "message": f"Invalid record ID: {record_id}"})
            
    except Exception as e:
        logger.error(f"Error modifying Euler record: {e}")
        return jsonify({"success": False, "message": f"Error: {str(e)}"})

@app.route('/api/delete_euler', methods=['POST'])
def delete_euler():
    """删除欧拉点数据"""
    data = request.json
    record_id = data.get('recordId')
    
    try:
        # 打开并读取欧拉CSV文件
        euler_df = pd.read_csv(tracker.EULER_CSV)
        
        # 删除对应记录
        if 0 <= record_id < len(euler_df):
            euler_df = euler_df.drop(record_id).reset_index(drop=True)
            
            # 保存修改后的文件
            euler_df.to_csv(tracker.EULER_CSV, index=False)
            
            logger.info(f"Deleted Euler record {record_id}")
            
            return jsonify({
                "success": True, 
                "message": f"Record at index {record_id} deleted."
            })
        else:
            return jsonify({"success": False, "message": f"Invalid record ID: {record_id}"})
            
    except Exception as e:
        logger.error(f"Error deleting Euler record: {e}")
        return jsonify({"success": False, "message": f"Error: {str(e)}"})

@app.route('/api/get_euler_records', methods=['GET'])
def get_euler_records():
    """获取所有欧拉点数据"""
    try:
        # 检查欧拉CSV文件是否存在
        if not os.path.exists(tracker.EULER_CSV):
            return jsonify({
                "success": False, 
                "message": "No Euler data file found."
            })
        
        # 打开并读取欧拉CSV文件
        euler_df = pd.read_csv(tracker.EULER_CSV)
        
        # 转换为JSON格式
        records = []
        for idx, row in euler_df.iterrows():
            records.append({
                "id": idx,
                "timestamp": row['Timestamp'],
                "angle": row['DesiredAngle'],
                "position": {
                    "x": row['X'],
                    "y": row['Y'],
                    "z": row['Z']
                },
                "orientation": {
                    "roll": row['Roll'],
                    "yaw": row['Yaw'],
                    "pitch": row['Pitch']
                }
            })
        
        return jsonify({
            "success": True,
            "records": records
        })
            
    except Exception as e:
        logger.error(f"Error retrieving Euler records: {e}")
        return jsonify({"success": False, "message": f"Error: {str(e)}"})
@app.route('/api/restart', methods=['POST'])
def restart_application():
    """重启应用程序"""
    try:
        logger.info("Restarting application...")
        
        # 创建一个函数在短暂延迟后重启应用
        def restart():
            time.sleep(1)  # 给客户端一点时间接收响应
            os.execv(sys.executable, [sys.executable] + sys.argv)
        
        # 在新线程中执行重启，以便能够先返回响应
        threading.Thread(target=restart).start()
        
        return jsonify({
            "success": True,
            "message": "Application restarting..."
        })
    
    except Exception as e:
        logger.error(f"Error restarting application: {e}")
        return jsonify({"success": False, "message": f"Error: {str(e)}"})

@app.route('/api/exit', methods=['POST'])
def exit_application():
    """关闭应用程序"""
    try:
        logger.info("Shutting down application...")
        
        # 创建一个函数在短暂延迟后关闭应用
        def shutdown():
            time.sleep(1)  # 给客户端一点时间接收响应
            os._exit(0)  # 强制退出
        
        # 在新线程中执行关闭，以便能够先返回响应
        threading.Thread(target=shutdown).start()
        
        return jsonify({
            "success": True,
            "message": "Application shutting down..."
        })
    
    except Exception as e:
        logger.error(f"Error shutting down application: {e}")
        return jsonify({"success": False, "message": f"Error: {str(e)}"})
if __name__ == '__main__':
    # 运行Flask应用
    port = 5000
    print(f"Starting Auto Tracker Data Process web interface on http://127.0.0.1:{port}/")
    run_simple('127.0.0.1', port, app, use_reloader=True, use_debugger=True)
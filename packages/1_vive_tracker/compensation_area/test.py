import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MplPolygon
from matplotlib.widgets import Button, TextBox, RadioButtons
import yaml
import json
from shapely.geometry import Polygon
import tkinter as tk
from tkinter import filedialog, simpledialog
import os

class CompensationRegionEditor:
    def __init__(self):
        self.fig, self.ax = plt.subplots(figsize=(15, 10))
        self.fig.subplots_adjust(left=0.1, bottom=0.2, right=0.95, top=0.95)
        
        # 存储所有区域信息
        self.regions = {}
        self.current_region = None
        self.current_points = []
        self.current_region_name = ""
        self.editing_mode = "create"  # create, edit, delete
        self.transition_width = 0.15
        
        # 默认输出目录
        self.output_directory = os.getcwd()
        
        # 初始化坐标轴
        self.setup_axes()
        # 设置交互按钮和控件
        self.setup_controls()
        
        # 连接事件
        self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        self.fig.canvas.mpl_connect('key_press_event', self.on_key_press)
        
        plt.show()
    
    def setup_axes(self):
        """设置绘图区域"""
        self.ax.set_xlim(-2, 5)
        self.ax.set_ylim(-5, 1)
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y (inverted for Left-Hand Coordinate System)')
        self.ax.set_title('Compensation Regions Editor')
        self.ax.grid(True)
        
        # 绘制原点和坐标轴
        self.ax.plot(0, 0, 'ko', markersize=8)
        self.ax.text(0.1, 0.1, 'Origin', fontsize=10)
        self.ax.arrow(0, 0, 1, 0, head_width=0.1, head_length=0.1, fc='k', ec='k')
        self.ax.arrow(0, 0, 0, -1, head_width=0.1, head_length=0.1, fc='k', ec='k')
        self.ax.text(1.1, 0, 'X', fontsize=12)
        self.ax.text(0, -1.1, 'Y', fontsize=12)
    
    def setup_controls(self):
        """设置UI控件"""
        # 区域按钮区
        ax_create = plt.axes([0.1, 0.05, 0.1, 0.04])
        self.btn_create = Button(ax_create, 'Create Region')
        self.btn_create.on_clicked(self.start_create_region)
        
        ax_edit = plt.axes([0.21, 0.05, 0.1, 0.04])
        self.btn_edit = Button(ax_edit, 'Edit Region')
        self.btn_edit.on_clicked(self.start_edit_region)
        
        ax_delete = plt.axes([0.32, 0.05, 0.1, 0.04])
        self.btn_delete = Button(ax_delete, 'Delete Region')
        self.btn_delete.on_clicked(self.delete_region)
        
        # 区域名称输入框
        ax_name = plt.axes([0.1, 0.1, 0.15, 0.04])
        self.txt_name = TextBox(ax_name, 'Region Name: ', initial='region0')
        
        # 补偿值输入
        ax_offset_x = plt.axes([0.43, 0.05, 0.1, 0.04])
        self.txt_offset_x = TextBox(ax_offset_x, 'Offset X: ', initial='0.0')
        
        ax_offset_y = plt.axes([0.43, 0.1, 0.1, 0.04])
        self.txt_offset_y = TextBox(ax_offset_y, 'Offset Y: ', initial='0.0')
        
        ax_offset_z = plt.axes([0.43, 0.15, 0.1, 0.04])
        self.txt_offset_z = TextBox(ax_offset_z, 'Offset Z: ', initial='0.0')
        
        # Yaw角范围输入
        ax_yaw_min = plt.axes([0.63, 0.05, 0.1, 0.04])
        self.txt_yaw_min = TextBox(ax_yaw_min, 'Yaw Min (deg): ', initial='-180')
        
        ax_yaw_max = plt.axes([0.63, 0.1, 0.1, 0.04])
        self.txt_yaw_max = TextBox(ax_yaw_max, 'Yaw Max (deg): ', initial='180')
        
        # 方向选择
        ax_direction = plt.axes([0.75, 0.05, 0.1, 0.1])
        self.radio_direction = RadioButtons(
            ax_direction, ('CW', 'CCW', 'Both'), active=2)
        
        # 过渡带宽度
        ax_transition = plt.axes([0.63, 0.15, 0.1, 0.04])
        self.txt_transition = TextBox(
            ax_transition, 'Transition Width: ', initial=str(self.transition_width))
        self.txt_transition.on_submit(self.update_transition_width)
        
        # 保存和加载按钮
        ax_save = plt.axes([0.85, 0.05, 0.1, 0.04])
        self.btn_save = Button(ax_save, 'Save Config')
        self.btn_save.on_clicked(self.save_config)
        
        ax_load = plt.axes([0.85, 0.1, 0.1, 0.04])
        self.btn_load = Button(ax_load, 'Load Config')
        self.btn_load.on_clicked(self.load_config)
        
        # 新增：设置输出目录按钮
        ax_set_dir = plt.axes([0.75, 0.15, 0.1, 0.04])
        self.btn_set_dir = Button(ax_set_dir, 'Set Output Dir')
        self.btn_set_dir.on_clicked(self.set_output_directory)
        
        ax_gen_code = plt.axes([0.85, 0.15, 0.1, 0.04])
        self.btn_gen_code = Button(ax_gen_code, 'Generate Code')
        self.btn_gen_code.on_clicked(self.generate_code)
    
    def set_output_directory(self, event):
        """设置输出目录"""
        root = tk.Tk()
        root.withdraw()
        directory = filedialog.askdirectory(
            initialdir=self.output_directory,
            title="Select Output Directory"
        )
        
        if directory:  # 确保用户没有取消选择
            self.output_directory = directory
            self.ax.set_title(f'Output directory set to: {self.output_directory}')
            self.fig.canvas.draw()
    
    def update_transition_width(self, text):
        """更新过渡带宽度"""
        try:
            self.transition_width = float(text)
            self.redraw_regions()
        except ValueError:
            pass
    
    def on_click(self, event):
        """处理鼠标点击事件"""
        if event.inaxes != self.ax:
            return
        
        if self.editing_mode == "create":
            # 创建模式下，点击添加点
            self.current_points.append((event.xdata, event.ydata))
            # 绘制点
            self.ax.plot(event.xdata, event.ydata, 'ro')
            
            # 如果有多个点，绘制线段
            if len(self.current_points) > 1:
                x = [p[0] for p in self.current_points]
                y = [p[1] for p in self.current_points]
                self.ax.plot(x, y, 'r-')
            
            # 更新显示坐标文本
            self.ax.text(event.xdata, event.ydata, f"({event.xdata:.2f}, {event.ydata:.2f})")
            
            self.fig.canvas.draw()
    
    def on_key_press(self, event):
        """处理键盘事件"""
        if event.key == 'enter' and self.editing_mode == "create" and len(self.current_points) >= 3:
            # 完成创建多边形
            self.finish_region()
        elif event.key == 'escape':
            # 取消当前操作
            self.cancel_current_operation()
    
    def start_create_region(self, event):
        """开始创建新区域"""
        self.editing_mode = "create"
        self.current_region_name = self.txt_name.text
        self.current_points = []
        self.ax.set_title(f'Creating Region: {self.current_region_name} - Click to add points, Enter to finish')
        self.fig.canvas.draw()
    
    def start_edit_region(self, event):
        """开始编辑区域"""
        self.editing_mode = "edit"
        region_name = self.txt_name.text
        if region_name in self.regions:
            self.current_region = region_name
            region_info = self.regions[region_name]
            
            # 显示当前区域信息
            self.txt_offset_x.set_val(str(region_info['offset'][0]))
            self.txt_offset_y.set_val(str(region_info['offset'][1]))
            self.txt_offset_z.set_val(str(region_info['offset'][2]))
            self.txt_yaw_min.set_val(str(region_info['yaw_range'][0]))
            self.txt_yaw_max.set_val(str(region_info['yaw_range'][1]))
            
            # 高亮显示选中的区域
            self.redraw_regions(highlight=region_name)
        else:
            self.ax.set_title(f'Region {region_name} not found!')
            self.fig.canvas.draw()
    
    def delete_region(self, event):
        """删除区域"""
        region_name = self.txt_name.text
        if region_name in self.regions:
            del self.regions[region_name]
            self.redraw_regions()
            self.ax.set_title(f'Region {region_name} deleted')
        else:
            self.ax.set_title(f'Region {region_name} not found!')
        self.fig.canvas.draw()
    
    def finish_region(self):
        """完成区域创建"""
        if len(self.current_points) < 3:
            self.ax.set_title('Need at least 3 points to create a region!')
            self.fig.canvas.draw()
            return
        
        # 获取补偿值
        try:
            offset_x = float(self.txt_offset_x.text)
            offset_y = float(self.txt_offset_y.text)
            offset_z = float(self.txt_offset_z.text)
            yaw_min = float(self.txt_yaw_min.text)
            yaw_max = float(self.txt_yaw_max.text)
        except ValueError:
            self.ax.set_title('Invalid offset or yaw values!')
            self.fig.canvas.draw()
            return
        
        # 获取方向
        direction = self.radio_direction.value_selected
        
        # 创建区域
        self.regions[self.current_region_name] = {
            'vertices': self.current_points,
            'offset': [offset_x, offset_y, offset_z],
            'yaw_range': [yaw_min, yaw_max],
            'direction': direction
        }
        
        # 重绘所有区域
        self.redraw_regions()
        
        # 重置当前操作
        self.current_points = []
        self.editing_mode = None
        self.ax.set_title(f'Region {self.current_region_name} created')
        self.fig.canvas.draw()
    
    def cancel_current_operation(self):
        """取消当前操作"""
        self.current_points = []
        self.editing_mode = None
        self.redraw_regions()
        self.ax.set_title('Operation canceled')
        self.fig.canvas.draw()
    
    def redraw_regions(self, highlight=None):
        """重绘所有区域"""
        self.ax.clear()
        self.setup_axes()
        
        for name, region in self.regions.items():
            vertices = region['vertices']
            poly = MplPolygon(vertices, alpha=0.5, label=name)
            
            # 根据方向选择颜色
            if region['direction'] == 'CW':
                color = 'blue'
            elif region['direction'] == 'CCW':
                color = 'green'
            else:
                color = 'purple'
            
            # 高亮显示选中的区域
            if name == highlight:
                poly.set_alpha(0.7)
                poly.set_linewidth(3)
            
            poly.set_color(color)
            self.ax.add_patch(poly)
            
            # 添加标签
            centroid = np.mean(vertices, axis=0)
            self.ax.text(centroid[0], centroid[1], name, 
                        ha='center', va='center', fontsize=10)
            
            # 绘制顶点和坐标
            xs, ys = zip(*vertices)
            self.ax.plot(xs, ys, 'ko', markersize=5)
            for i, (x, y) in enumerate(vertices):
                self.ax.text(x, y, f"({x:.2f}, {y:.2f})", fontsize=8)
            
            # 绘制过渡区域
            if self.transition_width > 0:
                shapely_poly = Polygon(vertices)
                expanded_poly = shapely_poly.buffer(self.transition_width)
                
                # 绘制扩展的多边形
                x, y = expanded_poly.exterior.xy
                self.ax.plot(x, y, '--', color=color, alpha=0.5)
        
        # 绘制正在创建的点和线
        if self.editing_mode == "create" and len(self.current_points) > 0:
            x_points = [p[0] for p in self.current_points]
            y_points = [p[1] for p in self.current_points]
            self.ax.plot(x_points, y_points, 'ro-')
            
            # 显示坐标文本
            for x, y in self.current_points:
                self.ax.text(x, y, f"({x:.2f}, {y:.2f})", fontsize=8)
            
            # 如果有至少3个点，闭合多边形
            if len(self.current_points) >= 3:
                self.ax.plot([self.current_points[-1][0], self.current_points[0][0]],
                           [self.current_points[-1][1], self.current_points[0][1]], 'r--')
        
        self.ax.legend()
        self.fig.canvas.draw()
    
    def save_config(self, event):
        """保存配置到文件"""
        root = tk.Tk()
        root.withdraw()
        
        # 使用选定的输出目录作为初始目录
        file_path = filedialog.asksaveasfilename(
            initialdir=self.output_directory,
            defaultextension=".yaml",
            filetypes=[("YAML files", "*.yaml"), ("JSON files", "*.json"), ("All files", "*.*")]
        )
        
        if not file_path:
            return
        
        # 更新输出目录为文件所在目录
        self.output_directory = os.path.dirname(file_path)
        
        config = {
            'transition_width': self.transition_width,
            'regions': self.regions
        }
        
        if file_path.endswith('.json'):
            with open(file_path, 'w') as f:
                json.dump(config, f, indent=2)
        else:
            with open(file_path, 'w') as f:
                yaml.dump(config, f, default_flow_style=False)
        
        self.ax.set_title(f'Configuration saved to {file_path}')
        self.fig.canvas.draw()
    
    def load_config(self, event):
        """从文件加载配置"""
        root = tk.Tk()
        root.withdraw()
        file_path = filedialog.askopenfilename(
            initialdir=self.output_directory,
            filetypes=[("YAML files", "*.yaml"), ("JSON files", "*.json"), ("All files", "*.*")]
        )
        
        if not file_path:
            return
        
        # 更新输出目录为文件所在目录
        self.output_directory = os.path.dirname(file_path)
        
        try:
            if file_path.endswith('.json'):
                with open(file_path, 'r') as f:
                    config = json.load(f)
            else:
                with open(file_path, 'r') as f:
                    config = yaml.safe_load(f)
            
            if 'transition_width' in config:
                self.transition_width = config['transition_width']
                self.txt_transition.set_val(str(self.transition_width))
            
            if 'regions' in config:
                self.regions = config['regions']
                self.redraw_regions()
                
            self.ax.set_title(f'Configuration loaded from {file_path}')
        except Exception as e:
            self.ax.set_title(f'Error loading config: {str(e)}')
        
        self.fig.canvas.draw()
    
    def generate_code(self, event):
        """生成Python代码"""
        if not self.regions:
            self.ax.set_title('No regions defined!')
            self.fig.canvas.draw()
            return
        
        code = self.create_python_code()
        
        # 保存代码到文件
        root = tk.Tk()
        root.withdraw()
        file_path = filedialog.asksaveasfilename(
            initialdir=self.output_directory,
            defaultextension=".py",
            filetypes=[("Python files", "*.py"), ("All files", "*.*")]
        )
        
        if file_path:
            # 更新输出目录为文件所在目录
            self.output_directory = os.path.dirname(file_path)
            
            with open(file_path, 'w') as f:
                f.write(code)
            self.ax.set_title(f'Code generated and saved to {file_path}')
            self.fig.canvas.draw()
    
    def create_python_code(self):
        """创建Python代码字符串"""
        code = [
            "# Generated by Compensation Region Editor",
            "from shapely.geometry import Polygon, Point",
            "import numpy as np",
            "",
            "class RegionCompensator:",
            "    def __init__(self):",
            f"        self.transition_width = {self.transition_width}",
            "        self.setup_regions()",
            "        self.direction = None  # Current rotation direction: 'CW' or 'CCW'",
            "",
            "    def setup_regions(self):",
            "        \"\"\"Initialize all region polygons and offsets\"\"\"",
            "        # Region definitions"
        ]
        
        # 添加区域定义
        for name, region in self.regions.items():
            vertices_str = ", ".join([f"({x:.4f}, {y:.4f})" for x, y in region['vertices']])
            code.append(f"        self.polygon_{name} = Polygon([{vertices_str}])")
        
        code.append("")
        code.append("        # Expanded regions for transition zones")
        
        # 添加扩展区域定义
        for name in self.regions.keys():
            code.append(f"        self.expanded_{name} = self.polygon_{name}.buffer(self.transition_width)")
        
        code.append("")
        code.append("        # Region offsets")
        
        # 添加偏移量定义
        code.append("        self.offsets = {")
        for name, region in self.regions.items():
            offset = region['offset']
            direction = region['direction']
            yaw_range = region['yaw_range']
            code.append(f"            '{name}': {{")
            code.append(f"                'offset': np.array([{offset[0]:.6f}, {offset[1]:.6f}, {offset[2]:.6f}]),")
            code.append(f"                'direction': '{direction}',")
            code.append(f"                'yaw_range': [{yaw_range[0]}, {yaw_range[1]}]")
            code.append("            },")
        code.append("        }")
        
        # 添加方法
        code.extend([
            "",
            "    def is_yaw_in_range(self, yaw_deg, region_name):",
            "        \"\"\"Check if yaw angle is within the specified range for a region\"\"\"",
            "        if region_name not in self.offsets:",
            "            return False",
            "        ",
            "        yaw_range = self.offsets[region_name]['yaw_range']",
            "        min_yaw, max_yaw = yaw_range",
            "        ",
            "        # Normalize yaw to [-180, 180]",
            "        while yaw_deg > 180:",
            "            yaw_deg -= 360",
            "        while yaw_deg < -180:",
            "            yaw_deg += 360",
            "        ",
            "        # Handle wrap-around case",
            "        if min_yaw <= max_yaw:",
            "            return min_yaw <= yaw_deg <= max_yaw",
            "        else:  # Wrap around case (e.g., -170 to 170)",
            "            return yaw_deg >= min_yaw or yaw_deg <= max_yaw",
            "",
            "    def calculate_transition_weight(self, point, polygon):",
            "        \"\"\"Calculate weight for transition zone\"\"\"",
            "        distance = polygon.exterior.distance(Point(point))",
            "        return max(0.0, min(1.0, 1 - distance / self.transition_width))",
            "",
            "    def interpolate_offset(self, offset1, offset2, weight):",
            "        \"\"\"Interpolate between two offsets based on weight\"\"\"",
            "        return offset1 * (1 - weight) + offset2 * weight",
            "",
            "    def get_compensation_offset(self, x, y, yaw_rad):",
            "        \"\"\"Get compensation offset for the given position and yaw\"\"\"",
            "        point = Point(x, y)",
            "        yaw_deg = np.degrees(yaw_rad)",
            "        ",
            "        # Default global offsets based on direction",
            "        if self.direction == 'CW':",
            "            default_offset = np.array([-0.0425, 0, 0])  # CW global offset",
            "        elif self.direction == 'CCW':",
            "            default_offset = np.array([0.0225, 0, 0])   # CCW global offset",
            "        else:",
            "            default_offset = np.array([0, 0, 0])        # Neutral offset",
            "",
            "        # Check if point is in any of the specific regions"
        ])
        
        # 添加区域检查逻辑
        for name in self.regions.keys():
            code.extend([
                f"        if self.polygon_{name}.contains(point) and self.is_yaw_in_range(yaw_deg, '{name}'):",
                f"            return self.offsets['{name}']['offset']"
            ])
        
        # 添加过渡区域检查逻辑
        code.append("")
        code.append("        # Check transition zones")
        for name in self.regions.keys():
            code.extend([
                f"        if self.expanded_{name}.contains(point) and self.is_yaw_in_range(yaw_deg, '{name}'):",
                f"            transition_weight = self.calculate_transition_weight((x, y), self.polygon_{name})",
                f"            return self.interpolate_offset(default_offset, self.offsets['{name}']['offset'], transition_weight)"
            ])
        
        # 添加默认返回
        code.extend([
            "",
            "        # Default case",
            "        return default_offset",
            "",
            "    def calculate_B_position(self, A_position, A_yaw):",
            "        \"\"\"Calculate position B from position A and yaw\"\"\"",
            "        x, y, z = A_position",
            "        ",
            "        # Get appropriate offset based on position and yaw",
            "        offset = self.get_compensation_offset(x, y, A_yaw)",
            "        ",
            "        # Rotation matrix (around Z axis)",
            "        rotation_matrix = np.array([",
            "            [np.cos(A_yaw), -np.sin(A_yaw), 0],",
            "            [np.sin(A_yaw),  np.cos(A_yaw), 0],",
            "            [0, 0, 1]",
            "        ])",
            "        ",
            "        # Calculate offset in global coordinates",
            "        offset_in_global = np.dot(rotation_matrix, offset)",
            "        ",
            "        # Calculate B position",
            "        B_position = A_position - offset_in_global",
            "        return B_position"
        ])
        
        return "\n".join(code)

# 运行工具
if __name__ == "__main__":
    editor = CompensationRegionEditor()
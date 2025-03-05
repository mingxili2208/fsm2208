import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MplPolygon
from matplotlib.widgets import Button, TextBox, RadioButtons
import yaml
import json
from shapely.geometry import Polygon, Point as PPoint
import tkinter as tk
from tkinter import filedialog, simpledialog
import os
import re

class CompensationRegionEditor:
    def __init__(self):
        self.fig, self.ax = plt.subplots(figsize=(15, 10))
        self.fig.subplots_adjust(left=0.1, bottom=0.2, right=0.95, top=0.95)
        
        # Store all region information
        self.regions = {}
        self.current_region = None
        self.current_points = []
        self.current_region_name = ""
        self.editing_mode = None  # create, edit, delete, modify_vertices
        self.selected_vertex_index = None
        self.transition_width = 0.15
        
        # Default output directory
        self.output_directory = os.getcwd()
        
        # Initialize axes
        self.setup_axes()
        # Set up interactive controls
        self.setup_controls()
        
        # Connect events
        self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        self.fig.canvas.mpl_connect('key_press_event', self.on_key_press)
        self.fig.canvas.mpl_connect('motion_notify_event', self.on_mouse_move)
        
        # Try to extract existing regions from test9.py
        self.extract_regions_from_code()
        
        plt.show()
    
    def setup_axes(self):
        """Set up plot area"""
        self.ax.set_xlim(-2, 5)
        self.ax.set_ylim(-5, 1)
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y (inverted for Left-Hand Coordinate System)')
        self.ax.set_title('Compensation Regions Editor')
        self.ax.grid(True)
        
        # Draw origin and coordinate axes
        self.ax.plot(0, 0, 'ko', markersize=8)
        self.ax.text(0.1, 0.1, 'Origin', fontsize=10)
        self.ax.arrow(0, 0, 1, 0, head_width=0.1, head_length=0.1, fc='k', ec='k')
        self.ax.arrow(0, 0, 0, -1, head_width=0.1, head_length=0.1, fc='k', ec='k')
        self.ax.text(1.1, 0, 'X', fontsize=12)
        self.ax.text(0, -1.1, 'Y', fontsize=12)
    
    def setup_controls(self):
        """Set up UI controls"""
        # Create a 2-row controls layout
        # First row - Region buttons
        ax_create = plt.axes([0.1, 0.1, 0.1, 0.04])
        self.btn_create = Button(ax_create, 'Create Region')
        self.btn_create.on_clicked(self.start_create_region)
        
        ax_edit = plt.axes([0.21, 0.1, 0.1, 0.04])
        self.btn_edit = Button(ax_edit, 'Edit Region')
        self.btn_edit.on_clicked(self.start_edit_region)
        
        ax_modify = plt.axes([0.32, 0.1, 0.1, 0.04])
        self.btn_modify = Button(ax_modify, 'Modify Vertices')
        self.btn_modify.on_clicked(self.start_modify_vertices)
        
        ax_delete = plt.axes([0.43, 0.1, 0.1, 0.04])
        self.btn_delete = Button(ax_delete, 'Delete Region')
        self.btn_delete.on_clicked(self.delete_region)
        
        # Region name input field
        ax_name = plt.axes([0.54, 0.1, 0.15, 0.04])
        self.txt_name = TextBox(ax_name, 'Region Name: ', initial='region0')
        
        # Save and load buttons
        ax_save = plt.axes([0.75, 0.1, 0.1, 0.04])
        self.btn_save = Button(ax_save, 'Save Config')
        self.btn_save.on_clicked(self.save_config)
        
        ax_load = plt.axes([0.86, 0.1, 0.1, 0.04])
        self.btn_load = Button(ax_load, 'Load Config')
        self.btn_load.on_clicked(self.load_config)
        
        # Second row - Compensation values
        # Offset values input
        ax_offset_x = plt.axes([0.1, 0.05, 0.1, 0.04])
        self.txt_offset_x = TextBox(ax_offset_x, 'Offset X: ', initial='0.0')
        
        ax_offset_y = plt.axes([0.21, 0.05, 0.1, 0.04])
        self.txt_offset_y = TextBox(ax_offset_y, 'Offset Y: ', initial='0.0')
        
        ax_offset_z = plt.axes([0.32, 0.05, 0.1, 0.04])
        self.txt_offset_z = TextBox(ax_offset_z, 'Offset Z: ', initial='0.0')
        
        # Yaw range input
        ax_yaw_min = plt.axes([0.43, 0.05, 0.1, 0.04])
        self.txt_yaw_min = TextBox(ax_yaw_min, 'Yaw Min (deg): ', initial='-180')
        
        ax_yaw_max = plt.axes([0.54, 0.05, 0.1, 0.04])
        self.txt_yaw_max = TextBox(ax_yaw_max, 'Yaw Max (deg): ', initial='180')
        
        # Direction selection
        ax_direction = plt.axes([0.65, 0.05, 0.1, 0.04])
        self.radio_direction = RadioButtons(
            ax_direction, ('CW', 'CCW', 'Both'), active=2)
        
        # Transition width
        ax_transition = plt.axes([0.75, 0.05, 0.1, 0.04])
        self.txt_transition = TextBox(
            ax_transition, 'Transition Width: ', initial=str(self.transition_width))
        self.txt_transition.on_submit(self.update_transition_width)
        
        # Output directory and code generation
        ax_set_dir = plt.axes([0.86, 0.05, 0.1, 0.04])
        self.btn_set_dir = Button(ax_set_dir, 'Set Output Dir')
        self.btn_set_dir.on_clicked(self.set_output_directory)
        
        ax_gen_code = plt.axes([0.75, 0.15, 0.1, 0.04])
        self.btn_gen_code = Button(ax_gen_code, 'Generate Code')
        self.btn_gen_code.on_clicked(self.generate_code)
        
        # Status display area
        ax_status = plt.axes([0.1, 0.01, 0.85, 0.03])
        ax_status.set_axis_off()
        self.status_text = ax_status.text(0, 0.5, "Ready", fontsize=10)
    
    def extract_regions_from_code(self):
        """Extract polygon region definitions from test9.py"""
        try:
            # Polygon region pattern
            polygon_pattern = r'polygon_region(\w+)\s*=\s*Polygon\(\[\s*([\s\S]*?)\]\)'
            
            with open("test9.py", "r") as f:
                code = f.read()
            
            # Extract polygon definitions
            polygon_matches = re.findall(polygon_pattern, code)
            
            for region_name, vertices_str in polygon_matches:
                # Process vertex list
                # Replace excess spaces and newlines
                vertices_str = re.sub(r'\s+', ' ', vertices_str)
                # Extract coordinate pairs
                coord_pairs = re.findall(r'\(\s*([+-]?\d+\.?\d*)\s*,\s*([+-]?\d+\.?\d*)\s*\)', vertices_str)
                
                if coord_pairs:
                    # Convert to float coordinate pairs
                    vertices = [(float(x), float(y)) for x, y in coord_pairs]
                    
                    # Extract region offsets
                    offset_pattern = fr'region{region_name}_offset_A_to_B\s*=\s*np\.array\(\[\s*([+-]?\d+\.?\d*)\s*,\s*([+-]?\d+\.?\d*)\s*,\s*([+-]?\d+\.?\d*)\s*\]\)'
                    offset_match = re.search(offset_pattern, code)
                    
                    if offset_match:
                        offset_x, offset_y, offset_z = map(float, offset_match.groups())
                        offset = [offset_x, offset_y, offset_z]
                    else:
                        # If specific offset not found, try to find global offset
                        if 'CW' in region_name:
                            offset_pattern = r'global_offset_CW_A_to_B\s*=\s*np\.array\(\[\s*([+-]?\d+\.?\d*)\s*,\s*([+-]?\d+\.?\d*)\s*,\s*([+-]?\d+\.?\d*)\s*\]\)'
                            direction = 'CW'
                        else:
                            offset_pattern = r'global_offset_CCW_A_to_B\s*=\s*np\.array\(\[\s*([+-]?\d+\.?\d*)\s*,\s*([+-]?\d+\.?\d*)\s*,\s*([+-]?\d+\.?\d*)\s*\]\)'
                            direction = 'CCW'
                        
                        offset_match = re.search(offset_pattern, code)
                        if offset_match:
                            offset_x, offset_y, offset_z = map(float, offset_match.groups())
                            offset = [offset_x, offset_y, offset_z]
                        else:
                            offset = [0.0, 0.0, 0.0]
                    
                    # Try to extract Yaw angle range
                    yaw_pattern = fr'if\s+(.+?)\s*<=\s*np\.degrees\(yaw\)\s*<=\s*(.+?)\s*:'
                    yaw_matches = re.findall(yaw_pattern, code)
                    
                    # Default to full range
                    yaw_range = [-180, 180]
                    
                    # If found matching yaw ranges, use the first match
                    if yaw_matches:
                        for min_yaw, max_yaw in yaw_matches:
                            try:
                                yaw_range = [float(min_yaw), float(max_yaw)]
                                break
                            except ValueError:
                                continue
                    
                    # Determine region direction
                    if 'CW' in region_name:
                        direction = 'CW'
                    elif 'CCW' in region_name:
                        direction = 'CCW'
                    else:
                        direction = 'Both'
                    
                    # Create region
                    self.regions[f'region{region_name}'] = {
                        'vertices': vertices,
                        'offset': offset,
                        'yaw_range': yaw_range,
                        'direction': direction
                    }
            
            # Update interface
            if self.regions:
                self.redraw_regions()
                self.status_text.set_text(f"Extracted {len(self.regions)} regions from code")
            else:
                self.status_text.set_text("No region definitions found in code")
            
        except Exception as e:
            self.status_text.set_text(f"Error extracting regions: {str(e)}")
            print(f"Error extracting regions: {str(e)}")
    
    def set_output_directory(self, event):
        """Set output directory"""
        root = tk.Tk()
        root.withdraw()
        directory = filedialog.askdirectory(
            initialdir=self.output_directory,
            title="Select Output Directory"
        )
        
        if directory:  # Ensure user didn't cancel
            self.output_directory = directory
            self.status_text.set_text(f'Output directory set to: {self.output_directory}')
            self.fig.canvas.draw()
    
    def update_transition_width(self, text):
        """Update transition width"""
        try:
            self.transition_width = float(text)
            self.redraw_regions()
            self.status_text.set_text(f'Transition width updated to: {self.transition_width}')
        except ValueError:
            self.status_text.set_text(f'Invalid transition width value')
    
    def on_click(self, event):
        """Handle mouse click events"""
        if event.inaxes != self.ax:
            return
        
        if self.editing_mode == "create":
            # In create mode, clicks add points
            self.current_points.append((event.xdata, event.ydata))
            # Redraw current region
            self.redraw_regions()
            self.status_text.set_text(f'Added point ({event.xdata:.2f}, {event.ydata:.2f}) - Press Enter to finish')
            
        elif self.editing_mode == "modify_vertices" and self.current_region:
            # In vertex modification mode
            # Check if a vertex was clicked
            region_vertices = self.regions[self.current_region]['vertices']
            for i, (vx, vy) in enumerate(region_vertices):
                # Calculate distance between click and vertex
                dist = np.sqrt((event.xdata - vx)**2 + (event.ydata - vy)**2)
                if dist < 0.1:  # Tolerance radius
                    # Select vertex
                    self.selected_vertex_index = i
                    self.status_text.set_text(f'Selected vertex {i} ({vx:.2f}, {vy:.2f}) - Drag to modify position')
                    return
            
            # If no vertex was clicked, consider adding new vertex
            # Find closest edge
            min_dist = float('inf')
            insert_index = -1
            
            for i in range(len(region_vertices)):
                p1 = region_vertices[i]
                p2 = region_vertices[(i + 1) % len(region_vertices)]
                
                # Calculate distance from point to line segment
                line_dist = self.point_to_line_dist(event.xdata, event.ydata, p1[0], p1[1], p2[0], p2[1])
                
                if line_dist < min_dist:
                    min_dist = line_dist
                    insert_index = (i + 1) % len(region_vertices)
            
            # If close enough, insert new vertex
            if min_dist < 0.1:
                region_vertices.insert(insert_index, (event.xdata, event.ydata))
                self.redraw_regions(highlight=self.current_region)
                self.status_text.set_text(f'Added new vertex at index {insert_index} ({event.xdata:.2f}, {event.ydata:.2f})')
    
    def on_mouse_move(self, event):
        """Handle mouse movement events"""
        if event.inaxes != self.ax:
            return
        
        if self.editing_mode == "modify_vertices" and self.selected_vertex_index is not None:
            # Modify selected vertex position
            vertices = self.regions[self.current_region]['vertices']
            vertices[self.selected_vertex_index] = (event.xdata, event.ydata)
            self.redraw_regions(highlight=self.current_region)
    
    def on_key_press(self, event):
        """Handle keyboard events"""
        if event.key == 'enter':
            if self.editing_mode == "create" and len(self.current_points) >= 3:
                # Finish creating polygon
                self.finish_region()
            elif self.editing_mode == "modify_vertices":
                # Finish vertex modification
                self.finish_vertex_modification()
                
        elif event.key == 'escape':
            # Cancel current operation
            self.cancel_current_operation()
        elif event.key == 'delete' and self.editing_mode == "modify_vertices" and self.selected_vertex_index is not None:
            # Delete selected vertex
            vertices = self.regions[self.current_region]['vertices']
            if len(vertices) > 3:  # Keep at least 3 vertices
                del vertices[self.selected_vertex_index]
                self.selected_vertex_index = None
                self.redraw_regions(highlight=self.current_region)
                self.status_text.set_text(f'Deleted vertex - {len(vertices)} vertices remaining')
            else:
                self.status_text.set_text('Cannot delete vertex - region requires at least 3 vertices')
    
    def start_create_region(self, event):
        """Start creating new region"""
        self.editing_mode = "create"
        self.current_region_name = self.txt_name.text
        self.current_points = []
        self.selected_vertex_index = None
        self.status_text.set_text(f'Creating region: {self.current_region_name} - Click to add vertices, press Enter to finish')
        self.redraw_regions()
    
    def start_edit_region(self, event):
        """Start editing region properties"""
        self.editing_mode = "edit"
        region_name = self.txt_name.text
        if region_name in self.regions:
            self.current_region = region_name
            region_info = self.regions[region_name]
            
            # Display current region information
            self.txt_offset_x.set_val(str(region_info['offset'][0]))
            self.txt_offset_y.set_val(str(region_info['offset'][1]))
            self.txt_offset_z.set_val(str(region_info['offset'][2]))
            self.txt_yaw_min.set_val(str(region_info['yaw_range'][0]))
            self.txt_yaw_max.set_val(str(region_info['yaw_range'][1]))
            
            # Set direction radio button
            if region_info['direction'] == 'CW':
                self.radio_direction.set_active(0)
            elif region_info['direction'] == 'CCW':
                self.radio_direction.set_active(1)
            else:
                self.radio_direction.set_active(2)
            
            # Highlight selected region
            self.redraw_regions(highlight=region_name)
            self.status_text.set_text(f'Editing region: {region_name} - Modify parameters and select another operation to finish')
        else:
            self.status_text.set_text(f'Region {region_name} not found')
    
    def start_modify_vertices(self, event):
        """Start modifying region vertices"""
        region_name = self.txt_name.text
        if region_name in self.regions:
            self.editing_mode = "modify_vertices"
            self.current_region = region_name
            self.selected_vertex_index = None
            
            # Highlight selected region
            self.redraw_regions(highlight=region_name)
            self.status_text.set_text(f'Modifying vertices of region {region_name} - Drag vertices to move, click edges to add, Delete to remove, Enter to finish')
        else:
            self.status_text.set_text(f'Region {region_name} not found')
    
    def finish_vertex_modification(self):
        """Complete vertex modification"""
        if self.current_region and self.editing_mode == "modify_vertices":
            self.selected_vertex_index = None
            self.editing_mode = None
            self.status_text.set_text(f'Completed vertex modification for region {self.current_region}')
            self.redraw_regions()
    
    def delete_region(self, event):
        """Delete region"""
        region_name = self.txt_name.text
        if region_name in self.regions:
            del self.regions[region_name]
            self.redraw_regions()
            self.status_text.set_text(f'Deleted region {region_name}')
        else:
            self.status_text.set_text(f'Region {region_name} not found')
    
    def finish_region(self):
        """Complete region creation"""
        if len(self.current_points) < 3:
            self.status_text.set_text('Need at least 3 points to create a region!')
            return
        
        # Get compensation values
        try:
            offset_x = float(self.txt_offset_x.text)
            offset_y = float(self.txt_offset_y.text)
            offset_z = float(self.txt_offset_z.text)
            yaw_min = float(self.txt_yaw_min.text)
            yaw_max = float(self.txt_yaw_max.text)
        except ValueError:
            self.status_text.set_text('Invalid offset or yaw values!')
            return
        
        # Get direction
        direction = self.radio_direction.value_selected
        
        # Create region
        self.regions[self.current_region_name] = {
            'vertices': self.current_points,
            'offset': [offset_x, offset_y, offset_z],
            'yaw_range': [yaw_min, yaw_max],
            'direction': direction
        }
        
        # Redraw all regions
        self.redraw_regions()
        
        # Reset current operation
        self.current_points = []
        self.editing_mode = None
        self.status_text.set_text(f'Region {self.current_region_name} created successfully')
    
    def cancel_current_operation(self):
        """Cancel current operation"""
        self.current_points = []
        self.editing_mode = None
        self.selected_vertex_index = None
        self.redraw_regions()
        self.status_text.set_text('Operation cancelled')
    
    def point_to_line_dist(self, px, py, x1, y1, x2, y2):
        """Calculate distance from point to line segment"""
        # Line segment vector
        dx = x2 - x1
        dy = y2 - y1
        
        # If line segment is a point
        if dx == 0 and dy == 0:
            return np.sqrt((px - x1)**2 + (py - y1)**2)
        
        # Calculate projection length ratio
        t = ((px - x1) * dx + (py - y1) * dy) / (dx**2 + dy**2)
        
        # If projection point is outside the line segment
        if t < 0:
            return np.sqrt((px - x1)**2 + (py - y1)**2)
        elif t > 1:
            return np.sqrt((px - x2)**2 + (py - y2)**2)
        
        # Projection point is on line segment
        proj_x = x1 + t * dx
        proj_y = y1 + t * dy
        return np.sqrt((px - proj_x)**2 + (py - proj_y)**2)
    
    def redraw_regions(self, highlight=None):
        """Redraw all regions"""
        self.ax.clear()
        self.setup_axes()
        
        for name, region in self.regions.items():
            vertices = region['vertices']
            poly = MplPolygon(vertices, alpha=0.5, label=name)
            
            # Choose color based on direction
            if region['direction'] == 'CW':
                color = 'blue'
            elif region['direction'] == 'CCW':
                color = 'green'
            else:
                color = 'purple'
            
            # Highlight selected region
            if name == highlight:
                poly.set_alpha(0.7)
                poly.set_linewidth(3)
            
            poly.set_color(color)
            self.ax.add_patch(poly)
            
            # Add label
            centroid = np.mean(vertices, axis=0)
            self.ax.text(centroid[0], centroid[1], name, 
                        ha='center', va='center', fontsize=10)
            
            # Draw vertices and coordinates
            xs, ys = zip(*vertices)
            
            # Highlight vertices of selected region
            if name == highlight and self.editing_mode == "modify_vertices":
                # Mark vertex numbers
                for i, (x, y) in enumerate(vertices):
                    if i == self.selected_vertex_index:
                        self.ax.plot(x, y, 'ro', markersize=10)  # Selected vertex marked with large red circle
                    else:
                        self.ax.plot(x, y, 'ko', markersize=7)
                    self.ax.text(x, y, f"{i}: ({x:.2f}, {y:.2f})", fontsize=8)
            else:
                self.ax.plot(xs, ys, 'ko', markersize=5)
                for i, (x, y) in enumerate(vertices):
                    self.ax.text(x, y, f"({x:.2f}, {y:.2f})", fontsize=8)
            
            # Draw transition zone
            if self.transition_width > 0:
                shapely_poly = Polygon(vertices)
                expanded_poly = shapely_poly.buffer(self.transition_width)
                
                # Draw expanded polygon
                x, y = expanded_poly.exterior.xy
                self.ax.plot(x, y, '--', color=color, alpha=0.5)
        
        # Draw points and lines being created
        if self.editing_mode == "create" and len(self.current_points) > 0:
            x_points = [p[0] for p in self.current_points]
            y_points = [p[1] for p in self.current_points]
            self.ax.plot(x_points, y_points, 'ro-')
            
            # Show coordinate text
            for i, (x, y) in enumerate(self.current_points):
                self.ax.text(x, y, f"{i}: ({x:.2f}, {y:.2f})", fontsize=8)
            
            # If at least 3 points, close the polygon
            if len(self.current_points) >= 3:
                self.ax.plot([self.current_points[-1][0], self.current_points[0][0]],
                           [self.current_points[-1][1], self.current_points[0][1]], 'r--')
        
        self.ax.legend(loc='upper right')
        self.fig.canvas.draw()
    
    def save_config(self, event):
        """Save configuration to file"""
        root = tk.Tk()
        root.withdraw()
        
        # Use selected output directory as initial directory
        file_path = filedialog.asksaveasfilename(
            initialdir=self.output_directory,
            defaultextension=".yaml",
            filetypes=[("YAML files", "*.yaml"), ("JSON files", "*.json"), ("All files", "*.*")]
        )
        
        if not file_path:
            return
        
        # Update output directory to file's directory
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
        
        self.status_text.set_text(f'Configuration saved to {file_path}')
    
    def load_config(self, event):
        """Load configuration from file"""
        root = tk.Tk()
        root.withdraw()
        file_path = filedialog.askopenfilename(
            initialdir=self.output_directory,
            filetypes=[("YAML files", "*.yaml"), ("JSON files", "*.json"), ("All files", "*.*")]
        )
        
        if not file_path:
            return
        
        # Update output directory to file's directory
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
                
            self.status_text.set_text(f'Configuration loaded from {file_path}')
        except Exception as e:
            self.status_text.set_text(f'Error loading configuration: {str(e)}')
    
    def generate_code(self, event):
        """Generate Python code"""
        if not self.regions:
            self.status_text.set_text('No regions defined!')
            return
        
        code = self.create_python_code()
        
        # Save code to file
        root = tk.Tk()
        root.withdraw()
        file_path = filedialog.asksaveasfilename(
            initialdir=self.output_directory,
            defaultextension=".py",
            filetypes=[("Python files", "*.py"), ("All files", "*.*")]
        )
        
        if file_path:
            # Update output directory to file's directory
            self.output_directory = os.path.dirname(file_path)
            
            with open(file_path, 'w') as f:
                f.write(code)
            self.status_text.set_text(f'Code generated and saved to {file_path}')
    
    def create_python_code(self):
        """Create Python code string"""
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
        
        # Add region definitions
        for name, region in self.regions.items():
            vertices_str = ", ".join([f"({x:.4f}, {y:.4f})" for x, y in region['vertices']])
            code.append(f"        self.polygon_{name} = Polygon([{vertices_str}])")
        
        code.append("")
        code.append("        # Expanded regions for transition zones")
        
        # Add expanded region definitions
        for name in self.regions.keys():
            code.append(f"        self.expanded_{name} = self.polygon_{name}.buffer(self.transition_width)")
        
        code.append("")
        code.append("        # Region offsets")
        
        # Add offset definitions
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
        
        # Add methods
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
        
        # Add region check logic
        for name in self.regions.keys():
            code.extend([
                f"        if self.polygon_{name}.contains(point) and self.is_yaw_in_range(yaw_deg, '{name}'):",
                f"            return self.offsets['{name}']['offset']"
            ])
        
        # Add transition zone check logic
        code.append("")
        code.append("        # Check transition zones")
        for name in self.regions.keys():
            code.extend([
                f"        if self.expanded_{name}.contains(point) and self.is_yaw_in_range(yaw_deg, '{name}'):",
                f"            transition_weight = self.calculate_transition_weight((x, y), self.polygon_{name})",
                f"            return self.interpolate_offset(default_offset, self.offsets['{name}']['offset'], transition_weight)"
            ])
        
        # Add default return
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

# Run the tool
if __name__ == "__main__":
    editor = CompensationRegionEditor()
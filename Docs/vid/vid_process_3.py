#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
视频横向拼接工具 - 高清晰度版本
解决编码器要求尺寸为偶数像素的问题，同时保持高清晰度
"""

import subprocess
import os
import sys
import json

def get_video_info(video_path):
    """使用ffprobe获取视频信息：时长、宽度和高度"""
    try:
        # 获取视频流信息
        cmd = [
            'ffprobe', 
            '-v', 'error',
            '-select_streams', 'v:0', 
            '-show_entries', 'stream=width,height,duration,r_frame_rate',
            '-of', 'json',
            video_path
        ]
        
        result = subprocess.run(cmd, capture_output=True, text=True)
        data = json.loads(result.stdout)
        
        # 如果duration不在stream中，尝试从format中获取
        if 'duration' not in data['streams'][0]:
            cmd = [
                'ffprobe', 
                '-v', 'error',
                '-show_entries', 'format=duration',
                '-of', 'json',
                video_path
            ]
            result = subprocess.run(cmd, capture_output=True, text=True)
            format_data = json.loads(result.stdout)
            duration = float(format_data['format']['duration'])
        else:
            duration = float(data['streams'][0]['duration'])
        
        width = int(data['streams'][0]['width'])
        height = int(data['streams'][0]['height'])
        
        # 获取帧率
        if 'r_frame_rate' in data['streams'][0]:
            frame_rate = data['streams'][0]['r_frame_rate']
            # 帧率通常是形如"30000/1001"的格式
            if '/' in frame_rate:
                num, den = map(int, frame_rate.split('/'))
                frame_rate = num / den
            else:
                frame_rate = float(frame_rate)
            print(f"视频帧率: {frame_rate} fps")
        else:
            frame_rate = None
        
        return {"duration": duration, "width": width, "height": height, "frame_rate": frame_rate}
    except Exception as e:
        print(f"读取视频信息失败 {video_path}:")
        print(f"错误: {str(e)}")
        if 'result' in locals():
            print(f"ffprobe输出: {result.stdout}")
            print(f"ffprobe错误: {result.stderr}")
        return None

def hd_concat_videos(left_video_path, right_video_path, output_path):
    """高清版本的视频拼接函数，保证输出视频保持高清晰度"""
    
    # 确认输入文件存在
    if not os.path.exists(left_video_path) or not os.path.exists(right_video_path):
        print("错误: 输入视频文件不存在")
        return False
    
    # 获取视频信息
    left_info = get_video_info(left_video_path)
    right_info = get_video_info(right_video_path)
    
    if left_info is None or right_info is None:
        print("无法获取视频信息")
        return False
    
    print(f"\n===== 视频信息 =====")
    print(f"左侧视频: {left_info['width']}x{left_info['height']}, 时长: {left_info['duration']:.2f}秒")
    print(f"右侧视频: {right_info['width']}x{right_info['height']}, 时长: {right_info['duration']:.2f}秒")
    
    # 优先使用1080p高清分辨率，如果原视频更高则保持原有高度
    target_height = 1080
    if left_info["height"] > target_height and right_info["height"] > target_height:
        # 如果两个视频都超过1080p，就用1080p
        final_height = target_height
    else:
        # 否则使用两者的最大值（但不超过原始视频高度）
        final_height = max(min(left_info["height"], target_height), 
                           min(right_info["height"], target_height))
    
    # 确保高度是偶数
    final_height = final_height if final_height % 2 == 0 else final_height - 1
    
    # 计算宽度以保持宽高比
    left_scale = final_height / left_info["height"]
    right_scale = final_height / right_info["height"]
    
    left_width = int(left_info["width"] * left_scale)
    right_width = int(right_info["width"] * right_scale)
    
    # 确保宽度是偶数
    left_width = left_width if left_width % 2 == 0 else left_width - 1
    right_width = right_width if right_width % 2 == 0 else right_width - 1
    
    # 计算总宽度
    total_width = left_width + right_width
    
    print(f"目标分辨率: 高度={final_height}像素")
    print(f"左侧视频缩放为: {left_width}x{final_height}")
    print(f"右侧视频缩放为: {right_width}x{final_height}")
    print(f"最终输出尺寸: {total_width}x{final_height}")
    
    # 视频总时长取两者的最小值
    min_duration = min(left_info["duration"], right_info["duration"])
    print(f"输出视频时长: {min_duration:.2f}秒")
    print("===================\n")
    
    # 确定帧率 - 使用两个视频中较高的帧率，如果无法确定就用30fps
    if left_info["frame_rate"] and right_info["frame_rate"]:
        output_fps = max(left_info["frame_rate"], right_info["frame_rate"])
        # 将帧率四舍五入到常见值
        if output_fps > 55:
            output_fps = 60
        elif output_fps > 45:
            output_fps = 50
        elif output_fps > 29:
            output_fps = 30
        elif output_fps > 24:
            output_fps = 25
        else:
            output_fps = 24
    else:
        output_fps = 30
    
    print(f"输出视频帧率: {output_fps} fps")
    
    # 设置较高的码率以保证清晰度
    # 计算合适的码率：一个粗略的经验法则是 宽度*高度*帧率*0.07 比特每秒
    bitrate = int(total_width * final_height * output_fps * 0.07 / 1000)
    # 至少要有5Mbps的码率才能保证1080p的清晰度
    bitrate = max(bitrate, 5000)
    
    print(f"设置视频码率: {bitrate}k")
    
    # 执行拼接 - 使用高质量设置
    try:
        cmd = [
            'ffmpeg', '-y',
            '-i', left_video_path,
            '-i', right_video_path,
            '-filter_complex',
            f'[0:v]scale={left_width}:{final_height},setsar=1[l];' +
            f'[1:v]scale={right_width}:{final_height},setsar=1[r];' +
            '[l][r]hstack=inputs=2[v]',
            '-map', '[v]',
            '-c:v', 'libx264',
            '-preset', 'medium',     # 平衡质量和速度
            '-crf', '18',            # 高质量，CRF值越低越好(范围18-28)
            '-b:v', f'{bitrate}k',   # 设置高码率
            '-maxrate', f'{int(bitrate*1.5)}k',
            '-bufsize', f'{bitrate*2}k',
            '-r', str(output_fps),   # 设置输出帧率
            '-t', str(min_duration),
            '-pix_fmt', 'yuv420p',
            output_path
        ]
        
        print("\n执行高清版FFmpeg命令:")
        print(' '.join(cmd))
        
        print("\n开始编码，请耐心等待...")
        print("(高清视频编码需要较长时间，请耐心等待...)")
        # 不捕获输出，让进度显示在终端
        subprocess.run(cmd, check=True)
        
        if os.path.exists(output_path) and os.path.getsize(output_path) > 0:
            print(f"\n拼接成功! 输出文件: {output_path}")
            output_size_mb = os.path.getsize(output_path) / (1024*1024)
            print(f"输出文件大小: {output_size_mb:.2f} MB")
            
            # 检查输出文件大小是否合理
            expected_size_mb = bitrate * min_duration / 8 / 1024
            print(f"预期文件大小约: {expected_size_mb:.2f} MB")
            
            if output_size_mb < expected_size_mb * 0.5:
                print("警告: 输出文件大小明显小于预期，可能编码质量不佳")
            
            return True
        else:
            print(f"输出文件创建失败或大小为0")
            return False
            
    except subprocess.CalledProcessError as e:
        print(f"FFmpeg执行失败: {e}")
        
        # 如果高清版本失败，尝试使用较低的设置
        print("\n高清版本失败，尝试使用较低质量设置...")
        try:
            backup_cmd = [
                'ffmpeg', '-y',
                '-i', left_video_path,
                '-i', right_video_path,
                '-filter_complex',
                '[0:v]scale=-2:720,setsar=1[l];[1:v]scale=-2:720,setsar=1[r];[l][r]hstack=inputs=2[v]',
                '-map', '[v]',
                '-c:v', 'libx264',
                '-preset', 'faster',
                '-crf', '23',
                '-r', '30',
                output_path
            ]
            print("\n执行备用FFmpeg命令:")
            print(' '.join(backup_cmd))
            
            subprocess.run(backup_cmd, check=True)
            
            if os.path.exists(output_path) and os.path.getsize(output_path) > 0:
                print(f"\n使用备用设置拼接成功! 输出文件: {output_path}")
                print(f"输出文件大小: {os.path.getsize(output_path) / (1024*1024):.2f} MB")
                print("注意: 输出质量可能低于预期，因为使用了备用设置")
                return True
            else:
                return False
        except Exception as e2:
            print(f"备用方法也失败了: {str(e2)}")
            return False
    except Exception as e:
        print(f"发生未知错误: {str(e)}")
        return False

def main():
    """主函数，使用固定的输入输出文件"""
    # 固定的文件名
    left_video = "left1.mp4"
    right_video = "right1.mp4"
    output_video = "output_merged3.mp4"
    
    # 显示参数信息
    print("\n===== 视频拼接工具 - 高清晰度版 =====")
    print(f"左侧视频: {left_video}")
    print(f"右侧视频: {right_video}")
    print(f"输出视频: {output_video}")
    print("======================================\n")
    
    # 检查必要的依赖
    try:
        subprocess.run(['ffmpeg', '-version'], stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=True)
        print("已检测到FFmpeg")
    except:
        print("错误: 未安装FFmpeg或无法访问。请安装FFmpeg后再试。")
        return
    
    # 检查输入文件
    print("检查输入文件...")
    for file_path, desc in [(left_video, "左侧视频"), (right_video, "右侧视频")]:
        if os.path.exists(file_path):
            size_mb = os.path.getsize(file_path) / (1024*1024)
            print(f"{desc}文件存在: {file_path} (大小: {size_mb:.2f} MB)")
        else:
            print(f"错误: {desc}文件不存在: {file_path}")
            return
    
    # 使用高清晰度版本的拼接函数
    success = hd_concat_videos(left_video, right_video, output_video)
    
    if success:
        print(f"\n拼接成功完成! 高清视频已保存至: {output_video}")
        
        # 检查输出文件的视频信息
        print("\n验证输出视频信息:")
        output_info = get_video_info(output_video)
        if output_info:
            print(f"输出视频分辨率: {output_info['width']}x{output_info['height']}")
            print(f"输出视频时长: {output_info['duration']:.2f}秒")
            if output_info['frame_rate']:
                print(f"输出视频帧率: {output_info['frame_rate']:.2f} fps")
            
            # 确认输出视频是否为高清
            if output_info['height'] >= 720:
                is_hd = "是" if output_info['height'] >= 1080 else "是(720p)"
                print(f"输出视频是否为高清: {is_hd}")
            else:
                print(f"警告: 输出视频分辨率低于高清标准")
    else:
        print("\n所有拼接方法都失败了。请检查输入视频格式和系统资源。")

if __name__ == "__main__":
    main()
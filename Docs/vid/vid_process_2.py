#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
视频横向拼接工具 - 简化版本
解决编码器要求尺寸为偶数像素的问题
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

def simple_concat_videos(left_video_path, right_video_path, output_path):
    """简化版本的视频拼接函数，使用最基本和最可靠的设置"""
    
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
    
    # 使用固定的720p尺寸进行拼接，这是大多数设备都能支持的分辨率
    target_height = 720
    # 计算宽度以保持宽高比
    left_scale = target_height / left_info["height"]
    right_scale = target_height / right_info["height"]
    
    left_width = int(left_info["width"] * left_scale)
    right_width = int(right_info["width"] * right_scale)
    
    # 确保宽度是偶数
    left_width = left_width if left_width % 2 == 0 else left_width - 1
    right_width = right_width if right_width % 2 == 0 else right_width - 1
    
    print(f"左侧视频缩放为: {left_width}x{target_height}")
    print(f"右侧视频缩放为: {right_width}x{target_height}")
    
    # 视频总时长取两者的最小值
    min_duration = min(left_info["duration"], right_info["duration"])
    print(f"输出视频时长: {min_duration:.2f}秒")
    
    # 执行拼接 - 使用最简单的设置
    try:
        cmd = [
            'ffmpeg', '-y',
            '-i', left_video_path,
            '-i', right_video_path,
            # 明确指定输入帧率以避免自动检测问题
            '-r', '30',
            '-filter_complex',
            f'[0:v]scale={left_width}:{target_height},setsar=1[l];' +
            f'[1:v]scale={right_width}:{target_height},setsar=1[r];' +
            '[l][r]hstack=inputs=2[v]',
            '-map', '[v]',
            '-c:v', 'libx264',
            '-preset', 'veryfast',  # 使用更快的预设
            '-crf', '28',           # 质量略低但速度更快
            '-t', str(min_duration),
            '-pix_fmt', 'yuv420p',
            '-r', '30',             # 明确指定输出帧率
            output_path
        ]
        
        print("\n执行简化版FFmpeg命令:")
        print(' '.join(cmd))
        
        print("\n开始编码，请耐心等待...")
        # 不捕获输出，让进度显示在终端
        subprocess.run(cmd, check=True)
        
        if os.path.exists(output_path) and os.path.getsize(output_path) > 0:
            print(f"\n拼接成功! 输出文件: {output_path}")
            print(f"输出文件大小: {os.path.getsize(output_path) / (1024*1024):.2f} MB")
            return True
        else:
            print(f"输出文件创建失败或大小为0")
            return False
            
    except subprocess.CalledProcessError as e:
        print(f"FFmpeg执行失败: {e}")
        return False
    except Exception as e:
        print(f"发生未知错误: {str(e)}")
        return False

def main():
    """主函数，使用固定的输入输出文件"""
    # 固定的文件名
    left_video = "left1.mp4"
    right_video = "right1.mp4"
    output_video = "output_merged2.mp4"
    
    # 显示参数信息
    print("\n===== 视频拼接工具 - 简化版 =====")
    print(f"左侧视频: {left_video}")
    print(f"右侧视频: {right_video}")
    print(f"输出视频: {output_video}")
    print("==================================\n")
    
    # 检查必要的依赖
    try:
        subprocess.run(['ffmpeg', '-version'], stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=True)
        print("已检测到FFmpeg")
    except:
        print("错误: 未安装FFmpeg或无法访问。请安装FFmpeg后再试。")
        return
    
    # 检查输入文件
    if not os.path.exists(left_video) or not os.path.exists(right_video):
        print("错误: 输入视频文件不存在")
        return
    
    # 使用简化版本的拼接函数
    success = simple_concat_videos(left_video, right_video, output_video)
    
    if success:
        print(f"拼接成功完成! 结果已保存至: {output_video}")
    else:
        print("拼接失败。尝试最后一种方法...")
        
        # 最后的尝试：使用更简单的命令
        try:
            final_cmd = [
                'ffmpeg', '-y',
                '-i', left_video,
                '-i', right_video,
                '-filter_complex',
                '[0:v]scale=640:360[l];[1:v]scale=640:360[r];[l][r]hstack=inputs=2[v]',
                '-map', '[v]',
                '-c:v', 'libx264',
                '-preset', 'ultrafast',
                '-r', '25',
                output_video
            ]
            print("\n尝试最简单的拼接命令:")
            print(' '.join(final_cmd))
            subprocess.run(final_cmd, check=True)
            
            if os.path.exists(output_video) and os.path.getsize(output_video) > 0:
                print(f"最终尝试成功! 输出文件: {output_video}")
            else:
                print("所有尝试都失败了。")
        except Exception as e:
            print(f"最终尝试也失败了: {str(e)}")

if __name__ == "__main__":
    main()
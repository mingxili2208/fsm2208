#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
视频横向拼接工具 - 修复偶数像素版本
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
            '-show_entries', 'stream=width,height,duration',
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
        
        return {"duration": duration, "width": width, "height": height}
    except Exception as e:
        print(f"读取视频信息失败 {video_path}:")
        print(f"错误: {str(e)}")
        if 'result' in locals():
            print(f"ffprobe输出: {result.stdout}")
            print(f"ffprobe错误: {result.stderr}")
        return None

def check_video_codec(video_path):
    """检查视频编解码器信息"""
    try:
        cmd = ['ffprobe', '-v', 'error', '-select_streams', 'v:0', 
               '-show_entries', 'stream=codec_name,width,height', 
               '-of', 'csv=p=0', video_path]
        result = subprocess.run(cmd, capture_output=True, text=True)
        print(f"视频 {video_path} 编解码器信息: {result.stdout.strip()}")
        return True
    except Exception as e:
        print(f"检查编解码器信息失败: {str(e)}")
        return False

def ensure_even(num):
    """确保数字是偶数"""
    return num if num % 2 == 0 else num - 1

def horizontal_concat_videos(left_video_path, right_video_path, output_path, scaling_mode='middle'):
    """横向拼接两个视频，确保所有尺寸都是偶数"""
    
    # 确认输入文件存在
    if not os.path.exists(left_video_path):
        print(f"错误: 左侧视频文件不存在: {left_video_path}")
        return False
    if not os.path.exists(right_video_path):
        print(f"错误: 右侧视频文件不存在: {right_video_path}")
        return False
        
    # 检查文件大小不为0
    if os.path.getsize(left_video_path) == 0:
        print(f"错误: 左侧视频文件大小为0: {left_video_path}")
        return False
    if os.path.getsize(right_video_path) == 0:
        print(f"错误: 右侧视频文件大小为0: {right_video_path}")
        return False
    
    # 检查编解码器
    print("\n检查视频编解码器...")
    check_video_codec(left_video_path)
    check_video_codec(right_video_path)

    # 获取两个视频的信息
    print("\n读取视频信息...")
    left_info = get_video_info(left_video_path)
    right_info = get_video_info(right_video_path)
    
    if left_info is None or right_info is None:
        print("无法获取视频信息，拼接终止")
        return False
    
    left_w, left_h = left_info["width"], left_info["height"]
    right_w, right_h = right_info["width"], right_info["height"]
    
    # 确定最终高度（确保是偶数）
    if scaling_mode == 'smaller':
        final_height = ensure_even(min(left_h, right_h))
    elif scaling_mode == 'larger':
        final_height = ensure_even(max(left_h, right_h))
    else:  # 'middle'
        final_height = ensure_even(int((left_h + right_h) / 2))
    
    # 计算缩放后的宽度，保持原始宽高比并确保是偶数
    left_scale = final_height / left_h
    right_scale = final_height / right_h
    
    scaled_left_w = ensure_even(int(left_w * left_scale))
    scaled_right_w = ensure_even(int(right_w * right_scale))
    
    # 确保总宽度也是偶数
    total_width = scaled_left_w + scaled_right_w
    
    # 确定拼接后的视频时长（取两者最小值）
    min_duration = min(left_info["duration"], right_info["duration"])
    
    print(f"\n===== 视频信息 =====")
    print(f"左侧视频: {left_w}x{left_h}, 时长: {left_info['duration']:.2f}秒")
    print(f"右侧视频: {right_w}x{right_h}, 时长: {right_info['duration']:.2f}秒")
    print(f"缩放模式: {scaling_mode}, 最终高度: {final_height}像素")
    print(f"缩放后尺寸 - 左: {scaled_left_w}x{final_height}, 右: {scaled_right_w}x{final_height}")
    print(f"最终输出尺寸: {total_width}x{final_height}")
    print(f"输出视频时长: {min_duration:.2f}秒")
    print("=====================\n")
    
    # 检查输出目录是否存在，不存在则创建
    output_dir = os.path.dirname(output_path)
    if output_dir and not os.path.exists(output_dir):
        os.makedirs(output_dir)
        print(f"创建输出目录: {output_dir}")
    
    # 直接使用一条命令尝试拼接
    try:
        # 使用单一命令，确保所有尺寸都是偶数
        ffmpeg_cmd = [
            'ffmpeg', '-y',
            '-i', left_video_path,
            '-i', right_video_path,
            '-progress', 'pipe:1',  # 添加进度显示
            '-filter_complex',
            f'[0:v]scale={scaled_left_w}:{final_height},setsar=1[l];' +
            f'[1:v]scale={scaled_right_w}:{final_height},setsar=1[r];' +
            '[l][r]hstack=inputs=2[v]',
            '-map', '[v]',
            '-c:v', 'libx264',
            '-crf', '23',
            '-preset', 'medium',
            '-t', str(min_duration),
            '-pix_fmt', 'yuv420p',
            output_path
        ]
        
        print("\n执行FFmpeg命令:")
        print(' '.join(ffmpeg_cmd))
        
        # 运行命令 - 不捕获输出，让进度直接显示在终端
        print("\n开始编码，请耐心等待(您将看到进度信息)...")
        result = subprocess.run(ffmpeg_cmd, check=True)
        
        # 检查输出文件是否创建且大小不为0
        if os.path.exists(output_path) and os.path.getsize(output_path) > 0:
            print(f"\n拼接成功! 输出文件: {output_path}")
            print(f"输出文件大小: {os.path.getsize(output_path) / (1024*1024):.2f} MB")
            return True
        else:
            print(f"\n错误: 输出文件创建失败或大小为0: {output_path}")
            if os.path.exists(output_path):
                print(f"文件存在但大小为: {os.path.getsize(output_path)} 字节")
            return False
            
    except subprocess.CalledProcessError as e:
        print(f"\nFFmpeg执行失败，返回码: {e.returncode}")
        
        # 尝试使用固定高度而不是缩放比例
        print("\n尝试使用固定高度1080p...")
        try:
            ffmpeg_fixed_cmd = [
                'ffmpeg', '-y',
                '-i', left_video_path,
                '-i', right_video_path,
                '-progress', 'pipe:1',  # 添加进度显示
                '-filter_complex',
                '[0:v]scale=-2:1080,setsar=1[l];' +
                '[1:v]scale=-2:1080,setsar=1[r];' +
                '[l][r]hstack=inputs=2[v]',
                '-map', '[v]',
                '-c:v', 'libx264',
                '-crf', '23',
                '-pix_fmt', 'yuv420p',
                'fixed_height_output.mp4'
            ]
            
            print(' '.join(ffmpeg_fixed_cmd))
            print("\n尝试固定高度方法，请耐心等待...")
            subprocess.run(ffmpeg_fixed_cmd, check=True)
            
            if os.path.exists('fixed_height_output.mp4') and os.path.getsize('fixed_height_output.mp4') > 0:
                print(f"使用固定高度成功! 输出: fixed_height_output.mp4")
                # 复制到最终输出文件
                import shutil
                shutil.copy2('fixed_height_output.mp4', output_path)
                return True
        except Exception as e2:
            print(f"固定高度方法也失败了: {str(e2)}")
        
        return False
        
    except Exception as e:
        print(f"\n执行过程中发生未知错误: {str(e)}")
        return False

def main():
    """主函数，使用固定的输入输出文件"""
    # 固定的文件名
    left_video = "left1.mp4"
    right_video = "right1.mp4"
    output_video = "output_merged.mp4"
    scaling_mode = 'middle'  # 默认使用中间高度
    
    # 显示参数信息
    print("\n===== 视频拼接工具 - 偶数尺寸版 =====")
    print(f"左侧视频: {left_video}")
    print(f"右侧视频: {right_video}")
    print(f"输出视频: {output_video}")
    print(f"缩放模式: {scaling_mode}")
    print("=====================================\n")
    
    # 检查必要的依赖
    try:
        subprocess.run(['ffmpeg', '-version'], stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=True)
        print("已检测到FFmpeg")
        subprocess.run(['ffprobe', '-version'], stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=True)
        print("已检测到FFprobe")
    except (subprocess.SubprocessError, FileNotFoundError):
        print("错误: 未安装FFmpeg/FFprobe或无法访问。请安装FFmpeg后再试。")
        print("安装命令: sudo apt-get install ffmpeg")
        return
    
    # 创建一个简单的视频文件检查
    print(f"检查视频文件...")
    for vid_path, desc in [(left_video, "左侧视频"), (right_video, "右侧视频")]:
        if os.path.exists(vid_path):
            size_mb = os.path.getsize(vid_path) / (1024*1024)
            print(f"{desc}文件存在: {vid_path} (大小: {size_mb:.2f} MB)")
        else:
            print(f"{desc}文件不存在: {vid_path}")
            return
    
    # 执行拼接
    print("\n开始拼接视频过程...")
    success = horizontal_concat_videos(left_video, right_video, output_video, scaling_mode)
    
    if success:
        print(f"\n拼接成功完成! 结果已保存至: {output_video}")
    else:
        print("\n拼接过程失败，尝试最后一种终极方法...")
        
        # 最后尝试最简单的方法，固定为720p分辨率
        final_cmd = [
            'ffmpeg', '-y',
            '-i', left_video, 
            '-i', right_video,
            '-progress', 'pipe:1',  # 添加进度显示
            '-filter_complex',
            '[0:v]scale=-2:720[l];[1:v]scale=-2:720[r];[l][r]hstack=inputs=2[v]',
            '-map', '[v]',
            '-c:v', 'libx264',
            '-crf', '23',
            '-pix_fmt', 'yuv420p',
            'final_720p_output.mp4'
        ]
        print(' '.join(final_cmd))
        print("\n尝试720p方法，请耐心等待...")
        try:
            subprocess.run(final_cmd, check=True)
            if os.path.exists('final_720p_output.mp4') and os.path.getsize('final_720p_output.mp4') > 0:
                print(f"720p方法成功! 输出: final_720p_output.mp4")
                # 复制到最终输出文件
                import shutil
                shutil.copy2('final_720p_output.mp4', output_video)
            else:
                print("所有方法都失败了。")
        except Exception as e:
            print(f"720p方法也失败了: {str(e)}")
    
    # 检查输出文件
    if os.path.exists(output_video):
        size_mb = os.path.getsize(output_video) / (1024*1024)
        print(f"输出文件存在: {output_video} (大小: {size_mb:.2f} MB)")
    else:
        print(f"输出文件不存在: {output_video}")

if __name__ == "__main__":
    main()
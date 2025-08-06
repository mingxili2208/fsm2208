#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
视频分割与重组工具 - 固定输出1920x1024分辨率
1. 读取1.mp4和2.mp4
2. 将1.mp4在width==1921处截断，生成left1和right1
3. 拼接right1.mp4和2.mp4（横向拼接）生成3.mp4
4. 将left1和3.mp4纵向拼接生成最终视频，固定输出1920x1024
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

def split_video_at_width_1921(input_video, left_output, right_output):
    """在width==1921处分割视频"""
    print(f"\n===== 步骤1: 分割视频 {input_video} =====")
    
    # 获取输入视频信息
    video_info = get_video_info(input_video)
    if video_info is None:
        print(f"无法获取视频信息: {input_video}")
        return False
    
    print(f"原视频尺寸: {video_info['width']}x{video_info['height']}")
    print(f"原视频时长: {video_info['duration']:.2f}秒")
    
    if video_info['width'] <= 1921:
        print(f"警告: 视频宽度 {video_info['width']} <= 1921，无法在1921处分割")
        return False
    
    # 计算分割位置
    left_width = 1920  # 调整为1920以便最终输出1920x1024
    right_width = video_info['width'] - 1920
    height = video_info['height']
    
    # 确保尺寸为偶数
    left_width = left_width if left_width % 2 == 0 else left_width - 1
    right_width = right_width if right_width % 2 == 0 else right_width - 1
    height = height if height % 2 == 0 else height - 1
    
    print(f"左侧部分: {left_width}x{height}")
    print(f"右侧部分: {right_width}x{height}")
    
    try:
        # 生成左侧视频 (从x=0开始，宽度为left_width)
        left_cmd = [
            'ffmpeg', '-y',
            '-i', input_video,
            '-filter_complex', f'[0:v]crop={left_width}:{height}:0:0[v]',
            '-map', '[v]',
            '-c:v', 'libx264',
            '-preset', 'medium',
            '-crf', '18',
            '-pix_fmt', 'yuv420p',
            left_output
        ]
        
        print(f"\n生成左侧视频: {left_output}")
        print("执行命令:", ' '.join(left_cmd))
        subprocess.run(left_cmd, check=True)
        
        # 生成右侧视频 (从x=1920开始，宽度为right_width)
        right_cmd = [
            'ffmpeg', '-y',
            '-i', input_video,
            '-filter_complex', f'[0:v]crop={right_width}:{height}:1920:0[v]',
            '-map', '[v]',
            '-c:v', 'libx264',
            '-preset', 'medium',
            '-crf', '18',
            '-pix_fmt', 'yuv420p',
            right_output
        ]
        
        print(f"\n生成右侧视频: {right_output}")
        print("执行命令:", ' '.join(right_cmd))
        subprocess.run(right_cmd, check=True)
        
        # 验证输出文件
        if os.path.exists(left_output) and os.path.exists(right_output):
            left_size = os.path.getsize(left_output) / (1024*1024)
            right_size = os.path.getsize(right_output) / (1024*1024)
            print(f"分割完成!")
            print(f"左侧视频: {left_output} ({left_size:.2f} MB)")
            print(f"右侧视频: {right_output} ({right_size:.2f} MB)")
            return True
        else:
            print("分割失败: 输出文件未生成")
            return False
            
    except subprocess.CalledProcessError as e:
        print(f"视频分割失败: {e}")
        return False

def horizontal_concat_videos(left_video, right_video, output_video):
    """横向拼接两个视频"""
    print(f"\n===== 步骤2: 横向拼接视频 =====")
    print(f"左侧视频: {left_video}")
    print(f"右侧视频: {right_video}")
    print(f"输出视频: {output_video}")
    
    # 获取两个视频的信息
    left_info = get_video_info(left_video)
    right_info = get_video_info(right_video)
    
    if left_info is None or right_info is None:
        print("无法获取视频信息")
        return False
    
    print(f"左侧视频: {left_info['width']}x{left_info['height']}, 时长: {left_info['duration']:.2f}秒")
    print(f"右侧视频: {right_info['width']}x{right_info['height']}, 时长: {right_info['duration']:.2f}秒")
    
    # 以较短视频的时长为准
    min_duration = min(left_info['duration'], right_info['duration'])
    print(f"输出视频时长: {min_duration:.2f}秒 (以较短视频为准)")
    
    # 统一高度 - 使用两者中较小的高度以确保质量
    target_height = min(left_info['height'], right_info['height'])
    target_height = target_height if target_height % 2 == 0 else target_height - 1
    
    # 计算缩放后的宽度
    left_scale = target_height / left_info['height']
    right_scale = target_height / right_info['height']
    
    left_width = int(left_info['width'] * left_scale)
    right_width = int(right_info['width'] * right_scale)
    
    # 确保宽度为偶数
    left_width = left_width if left_width % 2 == 0 else left_width - 1
    right_width = right_width if right_width % 2 == 0 else right_width - 1
    
    total_width = left_width + right_width
    
    print(f"统一高度: {target_height}")
    print(f"左侧缩放为: {left_width}x{target_height}")
    print(f"右侧缩放为: {right_width}x{target_height}")
    print(f"最终尺寸: {total_width}x{target_height}")
    
    try:
        cmd = [
            'ffmpeg', '-y',
            '-i', left_video,
            '-i', right_video,
            '-filter_complex',
            f'[0:v]scale={left_width}:{target_height},setsar=1[l];' +
            f'[1:v]scale={right_width}:{target_height},setsar=1[r];' +
            '[l][r]hstack=inputs=2[v]',
            '-map', '[v]',
            '-c:v', 'libx264',
            '-preset', 'medium',
            '-crf', '18',
            '-t', str(min_duration),
            '-pix_fmt', 'yuv420p',
            output_video
        ]
        
        print("\n执行横向拼接命令:")
        print(' '.join(cmd))
        subprocess.run(cmd, check=True)
        
        if os.path.exists(output_video) and os.path.getsize(output_video) > 0:
            output_size = os.path.getsize(output_video) / (1024*1024)
            print(f"横向拼接完成! 输出: {output_video} ({output_size:.2f} MB)")
            return True
        else:
            print("横向拼接失败: 输出文件未生成")
            return False
            
    except subprocess.CalledProcessError as e:
        print(f"横向拼接失败: {e}")
        return False

def vertical_concat_videos_fixed_size(top_video, bottom_video, output_video):
    """纵向拼接两个视频，固定输出1920x1024"""
    print(f"\n===== 步骤3: 纵向拼接视频 (固定1920x1024) =====")
    print(f"上方视频: {top_video}")
    print(f"下方视频: {bottom_video}")
    print(f"最终输出: {output_video}")
    
    # 获取两个视频的信息
    top_info = get_video_info(top_video)
    bottom_info = get_video_info(bottom_video)
    
    if top_info is None or bottom_info is None:
        print("无法获取视频信息")
        return False
    
    print(f"上方视频: {top_info['width']}x{top_info['height']}, 时长: {top_info['duration']:.2f}秒")
    print(f"下方视频: {bottom_info['width']}x{bottom_info['height']}, 时长: {bottom_info['duration']:.2f}秒")
    
    # 以较短视频的时长为准
    min_duration = min(top_info['duration'], bottom_info['duration'])
    print(f"输出视频时长: {min_duration:.2f}秒 (以较短视频为准)")
    
    # 固定最终输出尺寸
    final_width = 1920
    final_height = 1024
    
    # 每个视频占用的高度都是512像素 (1024/2)
    segment_height = 512
    
    print(f"目标输出尺寸: {final_width}x{final_height}")
    print(f"上方视频调整为: {final_width}x{segment_height}")
    print(f"下方视频调整为: {final_width}x{segment_height}")
    
    try:
        cmd = [
            'ffmpeg', '-y',
            '-i', top_video,
            '-i', bottom_video,
            '-filter_complex',
            f'[0:v]scale={final_width}:{segment_height},setsar=1[t];' +
            f'[1:v]scale={final_width}:{segment_height},setsar=1[b];' +
            '[t][b]vstack=inputs=2[v]',
            '-map', '[v]',
            '-c:v', 'libx264',
            '-preset', 'medium',
            '-crf', '18',
            '-t', str(min_duration),
            '-pix_fmt', 'yuv420p',
            output_video
        ]
        
        print("\n执行纵向拼接命令:")
        print(' '.join(cmd))
        subprocess.run(cmd, check=True)
        
        if os.path.exists(output_video) and os.path.getsize(output_video) > 0:
            output_size = os.path.getsize(output_video) / (1024*1024)
            print(f"纵向拼接完成! 最终输出: {output_video} ({output_size:.2f} MB)")
            
            # 验证输出分辨率
            final_info = get_video_info(output_video)
            if final_info:
                print(f"验证: 最终分辨率为 {final_info['width']}x{final_info['height']}")
                if final_info['width'] == 1920 and final_info['height'] == 1024:
                    print("✅ 输出分辨率正确!")
                else:
                    print("⚠️ 输出分辨率与预期不符")
            
            return True
        else:
            print("纵向拼接失败: 输出文件未生成")
            return False
            
    except subprocess.CalledProcessError as e:
        print(f"纵向拼接失败: {e}")
        return False

def main():
    """主函数"""
    print("\n===== 视频分割与重组工具 - 固定1920x1024输出 =====")
    print("功能:")
    print("1. 将1.mp4在width==1920处分割为left1和right1")
    print("2. 横向拼接right1和2.mp4生成3.mp4")
    print("3. 纵向拼接left1和3.mp4生成1920x1024最终视频")
    print("   (上方left1: 1920x512, 下方3.mp4: 1920x512)")
    print("======================================================\n")
    
    # 定义文件路径
    input_video1 = "1.mp4"
    input_video2 = "2.mp4"
    left1_video = "left1.mp4"
    right1_video = "right1.mp4"
    video3 = "3.mp4"
    final_output = "final_output.mp4"
    
    # 检查FFmpeg
    try:
        subprocess.run(['ffmpeg', '-version'], stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=True)
        print("已检测到FFmpeg")
    except:
        print("错误: 未安装FFmpeg或无法访问。请安装FFmpeg后再试。")
        return
    
    # 检查输入文件
    print("检查输入文件...")
    for file_path in [input_video1, input_video2]:
        if os.path.exists(file_path):
            size_mb = os.path.getsize(file_path) / (1024*1024)
            print(f"文件存在: {file_path} (大小: {size_mb:.2f} MB)")
        else:
            print(f"错误: 文件不存在: {file_path}")
            return
    
    # 步骤1: 分割1.mp4
    if not split_video_at_width_1921(input_video1, left1_video, right1_video):
        print("步骤1失败: 无法分割视频")
        return
    
    # 步骤2: 横向拼接right1.mp4和2.mp4
    if not horizontal_concat_videos(right1_video, input_video2, video3):
        print("步骤2失败: 无法横向拼接视频")
        return
    
    # 步骤3: 纵向拼接left1.mp4和3.mp4，固定输出1920x1024
    if not vertical_concat_videos_fixed_size(left1_video, video3, final_output):
        print("步骤3失败: 无法纵向拼接视频")
        return
    
    print(f"\n🎉 所有步骤完成!")
    print(f"最终输出文件: {final_output}")
    
    # 显示最终视频信息
    final_info = get_video_info(final_output)
    if final_info:
        print(f"\n最终视频信息:")
        print(f"分辨率: {final_info['width']}x{final_info['height']}")
        print(f"时长: {final_info['duration']:.2f}秒")
        if final_info['frame_rate']:
            print(f"帧率: {final_info['frame_rate']:.2f} fps")
        
        final_size = os.path.getsize(final_output) / (1024*1024)
        print(f"文件大小: {final_size:.2f} MB")
        
        # 确认输出规格
        if final_info['width'] == 1920 and final_info['height'] == 1024:
            print("输出规格符合要求: 1920x1024")
        else:
            print(f"!!输出规格不符合要求，实际为: {final_info['width']}x{final_info['height']}")
    
    # 清理临时文件（可选）
    cleanup_choice = input("\n是否删除临时文件 (left1.mp4, right1.mp4, 3.mp4)? (y/N): ")
    if cleanup_choice.lower() == 'y':
        temp_files = [left1_video, right1_video, video3]
        for temp_file in temp_files:
            if os.path.exists(temp_file):
                os.remove(temp_file)
                print(f"已删除: {temp_file}")
        print("临时文件清理完成")

if __name__ == "__main__":
    main()
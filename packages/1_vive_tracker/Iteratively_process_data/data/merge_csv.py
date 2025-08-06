import pandas as pd
import os

def merge_and_sort_csvs(file1_path, file2_path, output_path):
    """
    合并两个结构相同的CSV文件，并根据时间戳排序。

    参数:
    file1_path (str): 第一个输入CSV文件的路径。
    file2_path (str): 第二个输入CSV文件的路径。
    output_path (str): 合并并排序后的输出CSV文件的路径。
    """
    print("开始处理...")
    
    # --- 1. 检查输入文件是否存在 ---
    if not os.path.exists(file1_path):
        print(f"错误: 文件未找到 -> {file1_path}")
        return
    if not os.path.exists(file2_path):
        print(f"错误: 文件未找到 -> {file2_path}")
        return

    try:
        # --- 2. 使用 pandas 读取两个 CSV 文件 ---
        print(f"正在读取第一个文件: {os.path.basename(file1_path)}")
        df1 = pd.read_csv(file1_path)
        print(f"  -> 读取了 {len(df1)} 行数据。")

        print(f"正在读取第二个文件: {os.path.basename(file2_path)}")
        df2 = pd.read_csv(file2_path)
        print(f"  -> 读取了 {len(df2)} 行数据。")

        # --- 3. 合并两个 DataFrame ---
        # ignore_index=True 会重新生成从0开始的连续索引
        print("\n正在合并数据...")
        merged_df = pd.concat([df1, df2], ignore_index=True)
        print(f"  -> 合并后总行数: {len(merged_df)}")

        # --- 4. 检查 'timestamp' 列是否存在 ---
        if 'timestamp' not in merged_df.columns:
            print("错误: 合并后的数据中未找到 'timestamp' 列，无法进行排序。")
            print(f"  -> 可用列: {merged_df.columns.tolist()}")
            return
            
        # --- 5. 根据 'timestamp' 列进行排序 ---
        print("正在按 'timestamp' 排序...")
        # inplace=False (默认) 会返回一个新的排好序的 DataFrame
        sorted_df = merged_df.sort_values(by='timestamp', ascending=True)
        print("  -> 排序完成。")

        # --- 6. 将结果保存到新的 CSV 文件 ---
        # index=False 表示不将 DataFrame 的索引写入到 CSV 文件中
        print(f"\n正在保存结果到: {output_path}")
        sorted_df.to_csv(output_path, index=False)
        
        print("\n处理成功完成！")
        print(f"合并后的文件已保存为: {output_path}")

    except Exception as e:
        print(f"\n处理过程中发生错误: {e}")

# --- 主程序入口 ---
if __name__ == "__main__":
    # --- 请在这里修改您的文件路径 ---
    
    # 输入文件1的路径
    input_file_1 = "/home/cityu-fsm-lab-carla/lmx/packages/1_vive_tracker/Iteratively_process_data/data/final_coordinates_0729_n.csv"
    
    # 输入文件2的路径
    input_file_2 = "/home/cityu-fsm-lab-carla/lmx/packages/1_vive_tracker/Iteratively_process_data/data/final_coordinates_0729_c.csv"
    
    # 输出文件的路径和名称
    output_file = "/home/cityu-fsm-lab-carla/lmx/packages/1_vive_tracker/Iteratively_process_data/data/final_coordinates.csv"

    # 调用主函数
    merge_and_sort_csvs(input_file_1, input_file_2, output_file)
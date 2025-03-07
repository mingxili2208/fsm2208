import pandas as pd

# 定义文件路径
input_file = "tracker_log_0703.txt"  # 输入的 TXT 文件
output_file = "tracker_log_0703.csv"  # 输出的 CSV 文件

# 读取文件并解析数据
with open(input_file, "r", encoding="utf-8") as file:
    # 读取第一行作为列名
    headers = file.readline().strip().split(", ")
    
    # 读取剩余数据
    data = []
    for line in file:
        values = line.strip().split(", ")
        data.append(values)

# 将数据转换为 DataFrame
df = pd.DataFrame(data, columns=headers)

# 将数值列转换为浮点数
df = df.astype(float)

# 保存为 CSV 文件
df.to_csv(output_file, index=False, encoding="utf-8")

print(f"数据已成功保存为 {output_file}")
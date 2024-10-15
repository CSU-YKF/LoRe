import os
import re
import csv

# 定义处理文件的函数
def process_txt_files(directory, output_csv):
    # 正则表达式匹配信息
    pattern = {
        "points": r"Loaded (\d+) data points",
        "load_center": r"Load port center: \((-?[\d.]+), (-?[\d.]+), (-?[\d.]+)\)",
        "load_normal": r"Load port normal: \((-?[\d.]+), (-?[\d.]+), (-?[\d.]+)\)",
        "recycle_center": r"Recycle port center: \((-?[\d.]+), (-?[\d.]+), (-?[\d.]+)\)",
        "recycle_normal": r"Recycle port normal: \((-?[\d.]+), (-?[\d.]+), (-?[\d.]+)\)",
        "runtime": r"Function runtime: ([\d.]+) seconds"
    }

    data_list = []

    # 遍历目录下的所有txt文件
    for filename in os.listdir(directory):
        if filename.endswith('.txt'):
            file_path = os.path.join(directory, filename)
            with open(file_path, 'r', encoding='utf-8') as file:
                content = file.read()

                # 提取信息
                data = {}
                data['filename'] = filename
                data['points'] = int(re.search(pattern["points"], content).group(1)) if re.search(pattern["points"], content) else None
                data['load_center'] = tuple(map(float, re.search(pattern["load_center"], content).groups())) if re.search(pattern["load_center"], content) else None
                data['load_normal'] = tuple(map(float, re.search(pattern["load_normal"], content).groups())) if re.search(pattern["load_normal"], content) else None
                data['recycle_center'] = tuple(map(float, re.search(pattern["recycle_center"], content).groups())) if re.search(pattern["recycle_center"], content) else None
                data['recycle_normal'] = tuple(map(float, re.search(pattern["recycle_normal"], content).groups())) if re.search(pattern["recycle_normal"], content) else None
                data['runtime'] = float(re.search(pattern["runtime"], content).group(1)) if re.search(pattern["runtime"], content) else None

                data_list.append(data)

    # 将数据写入CSV文件
    with open(output_csv, 'w', newline='', encoding='utf-8') as csvfile:
        fieldnames = ['序号', '点数', '位置[误差]（毫米） - 加注口', '位置[误差]（毫米） - 回收口', '角度[误差]（度） - 加注口', '角度[误差]（度） - 回收口', '计算时长']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()

        for index, item in enumerate(data_list, start=1):
            writer.writerow({
                '序号': index,
                '点数': item['points'],
                '位置[误差]（毫米） - 加注口': item['load_center'],
                '位置[误差]（毫米） - 回收口': item['recycle_center'],
                '角度[误差]（度） - 加注口': item['load_normal'],
                '角度[误差]（度） - 回收口': item['recycle_normal'],
                '计算时长': item['runtime']
            })

if __name__ == "__main__":
    input_directory = "./"  # 修改为你的txt文件目录路径
    output_csv = "./output_summary.csv"  # 输出CSV文件的路径
    process_txt_files(input_directory, output_csv)
    print(f"数据已成功汇总至 {output_csv}")
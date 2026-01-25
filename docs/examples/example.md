# 示例 1: 基础使用

本示例演示如何基础使用本项目完成一个简单的任务。

## 概述

本示例将展示：
1. 如何加载和配置项目
2. 如何执行基本操作
3. 如何查看和验证结果

## 前提条件

确保您已经：
- 完成 [环境安装](../getting-started.md)
- 安装所有依赖
- 克隆项目代码

## 示例代码

### 完整示例脚本

创建文件 `run_example1.py`：

```python
#!/usr/bin/env python3
"""
示例 1: 基础使用
"""

import os
import sys
import logging
from pathlib import Path

# 添加项目根目录到路径
project_root = Path(__file__).parent.parent.parent
sys.path.insert(0, str(project_root))

from your_project import Project
from your_project.utils import setup_logging

def setup_environment():
    """设置环境"""
    # 配置日志
    setup_logging(level=logging.INFO)
    
    # 创建必要目录
    directories = ["data/input", "data/output", "logs"]
    for directory in directories:
        Path(directory).mkdir(parents=True, exist_ok=True)
    
    print("环境设置完成")

def create_sample_data():
    """创建示例数据"""
    import json
    import csv
    
    # 创建 JSON 数据
    json_data = {
        "name": "示例项目",
        "version": "1.0.0",
        "parameters": {
            "batch_size": 32,
            "learning_rate": 0.001,
            "epochs": 10
        },
        "features": ["feature1", "feature2", "feature3"]
    }
    
    with open("data/input/sample.json", "w", encoding="utf-8") as f:
        json.dump(json_data, f, indent=2, ensure_ascii=False)
    
    # 创建 CSV 数据
    csv_data = [
        ["id", "name", "value", "category"],
        [1, "item1", 10.5, "A"],
        [2, "item2", 20.3, "B"],
        [3, "item3", 15.7, "A"],
        [4, "item4", 25.1, "C"],
        [5, "item5", 18.9, "B"]
    ]
    
    with open("data/input/sample.csv", "w", encoding="utf-8", newline="") as f:
        writer = csv.writer(f)
        writer.writerows(csv_data)
    
    print("示例数据创建完成")

def run_project_operations():
    """运行项目操作"""
    # 初始化项目
    project = Project(
        config_path="config.yaml",
        log_level="INFO"
    )
    
    # 加载配置
    project.load_config()
    
    # 处理数据
    print("开始处理数据...")
    
    # 示例操作 1: 加载数据
    data = project.load_data("data/input/sample.csv")
    print(f"加载数据: {len(data)} 条记录")
    
    # 示例操作 2: 数据转换
    transformed_data = project.transform_data(data)
    print(f"数据转换完成")
    
    # 示例操作 3: 执行分析
    analysis_result = project.analyze(transformed_data)
    print(f"分析完成: {analysis_result['summary']}")
    
    # 示例操作 4: 保存结果
    project.save_results(
        analysis_result,
        output_path="data/output/results.json"
    )
    
    return analysis_result

def visualize_results(result):
    """可视化结果"""
    try:
        import matplotlib.pyplot as plt
        import pandas as pd
        
        # 创建可视化
        fig, axes = plt.subplots(2, 2, figsize=(12, 10))
        
        # 图表 1: 条形图
        if 'metrics' in result:
            metrics = result['metrics']
            axes[0, 0].bar(range(len(metrics)), list(metrics.values()))
            axes[0, 0].set_title("性能指标")
            axes[0, 0].set_xticks(range(len(metrics)))
            axes[0, 0].set_xticklabels(list(metrics.keys()), rotation=45)
        
        # 图表 2: 饼图
        if 'distribution' in result:
            distribution = result['distribution']
            axes[0, 1].pie(
                list(distribution.values()),
                labels=list(distribution.keys()),
                autopct='%1.1f%%'
            )
            axes[0, 1].set_title("数据分布")
        
        # 图表 3: 折线图
        if 'history' in result:
            history = result['history']
            axes[1, 0].plot(history)
            axes[1, 0].set_title("训练历史")
            axes[1, 0].set_xlabel("迭代")
            axes[1, 0].set_ylabel("损失")
        
        # 图表 4: 散点图
        if 'predictions' in result:
            predictions = result['predictions']
            if len(predictions) > 0 and isinstance(predictions[0], dict):
                x = [p.get('x', i) for i, p in enumerate(predictions)]
                y = [p.get('y', 0) for p in predictions]
                axes[1, 1].scatter(x, y)
                axes[1, 1].set_title("预测结果")
                axes[1, 1].set_xlabel("特征")
                axes[1, 1].set_ylabel("预测值")
        
        plt.tight_layout()
        plt.savefig("data/output/visualization.png", dpi=150)
        plt.close()
        
        print("可视化结果已保存: data/output/visualization.png")
        
    except ImportError:
        print("Matplotlib 未安装，跳过可视化")
        print("安装命令: pip install matplotlib")

def generate_report(result):
    """生成报告"""
    report = f"""
# 示例运行报告

## 执行摘要
- 执行时间: {result.get('timestamp', '未知')}
- 状态: {result.get('status', '未知')}
- 处理记录数: {result.get('processed_count', 0)}

## 性能指标
"""
    
    if 'metrics' in result:
        for key, value in result['metrics'].items():
            report += f"- {key}: {value:.4f}\n"
    
    report += f"""
## 详细结果
{result.get('details', '无')}

## 下一步建议
1. 查看输出文件: data/output/results.json
2. 查看可视化: data/output/visualization.png
3. 尝试修改参数重新运行
"""
    
    with open("data/output/report.md", "w", encoding="utf-8") as f:
        f.write(report)
    
    print("报告已生成: data/output/report.md")

def main():
    """主函数"""
    print("=" * 60)
    print("示例 1: 基础使用")
    print("=" * 60)
    
    try:
        # 步骤 1: 环境设置
        setup_environment()
        
        # 步骤 2: 创建示例数据
        create_sample_data()
        
        # 步骤 3: 运行项目操作
        result = run_project_operations()
        
        # 步骤 4: 可视化结果
        visualize_results(result)
        
        # 步骤 5: 生成报告
        generate_report(result)
        
        print("=" * 60)
        print("示例运行完成!")
        print("输出文件:")
        print("  - data/output/results.json")
        print("  - data/output/visualization.png")
        print("  - data/output/report.md")
        print("=" * 60)
        
    except Exception as e:
        print(f"运行示例时出错: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

if __name__ == "__main__":
    main()
```

## 运行示例

### 方法一: 直接运行

```bash
python run_example1.py
```

### 方法二: 使用提供的脚本

如果项目包含示例脚本：

```bash
python examples/example1.py
```

## 预期输出

运行成功后，您应该看到：

```
============================================================
示例 1: 基础使用
============================================================
环境设置完成
示例数据创建完成
开始处理数据...
加载数据: 5 条记录
数据转换完成
分析完成: 分析成功，包含5个指标
可视化结果已保存: data/output/visualization.png
报告已生成: data/output/report.md
============================================================
示例运行完成!
输出文件:
  - data/output/results.json
  - data/output/visualization.png
  - data/output/report.md
============================================================
```

## 文件结构

运行示例后，目录结构如下：

```
data/
├── input/
│   ├── sample.json
│   └── sample.csv
└── output/
    ├── results.json
    ├── visualization.png
    └── report.md
logs/
└── app.log
```

## 自定义示例

您可以修改以下部分来自定义示例：

1. **修改数据**: 编辑 `data/input/sample.csv` 或 `sample.json`
2. **调整参数**: 修改 `config.yaml` 中的配置
3. **扩展功能**: 在 `run_project_operations()` 函数中添加更多操作

## 故障排除

### 常见问题

1. **导入错误**: 确保项目根目录在 Python 路径中
2. **文件不存在**: 检查 `data/input/` 目录是否存在
3. **权限问题**: 确保对输出目录有写入权限

### 调试模式

要获取更详细的输出，启用调试日志：

```bash
python run_example1.py --log-level DEBUG
```

## 下一步

- 查看 [示例数据目录](../examples/example-data/) 获取更多示例模型和脚本
- 探索 [高级教程](../tutorials/tutorial-advanced.md) 学习更复杂的功能
- 参考 [API 文档](../api/index.md) 了解可用接口

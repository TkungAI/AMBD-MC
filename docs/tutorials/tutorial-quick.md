# 基础教程

本教程将引导您快速了解项目的基本用法和核心功能。

## 概述

本教程适合初学者，将介绍：
- 项目的基本概念
- 核心功能的使用方法
- 常见任务的完成步骤

## 准备工作

在开始之前，请确保您已经：
1. 完成 [环境安装](../getting-started.md)
2. 克隆了项目代码
3. 安装了所有依赖

## 第一步：初始化项目

### 配置文件

创建或修改配置文件 `config.yaml`：

```yaml
# 基本配置
project:
  name: "我的项目"
  version: "1.0.0"

# 数据库配置
database:
  host: "localhost"
  port: 5432
  name: "mydb"
  user: "admin"

# 日志配置
logging:
  level: "INFO"
  file: "logs/app.log"
```

### 初始化脚本

运行初始化脚本：

```bash
python scripts/init.py
```

## 第二步：使用核心功能

### 功能模块 A

```python
from your_project.module_a import FeatureA

# 创建实例
feature = FeatureA(config_path="config.yaml")

# 执行功能
result = feature.process(input_data="example")

# 查看结果
print(result)
```

### 功能模块 B

```python
from your_project.module_b import FeatureB

feature_b = FeatureB()
feature_b.analyze(data_file="data.csv")
```

## 第三步：运行示例

运行提供的示例代码：

```bash
python examples/basic_usage.py
```

## 第四步：验证结果

检查输出文件或日志，确保功能正常运行：

```bash
cat output/result.json
tail -f logs/app.log
```

## 常见任务

### 任务 1：数据处理

```python
from your_project.data_processor import DataProcessor

processor = DataProcessor()
processed_data = processor.clean_and_transform("raw_data.csv")
processor.save("processed_data.csv")
```

### 任务 2：模型训练

```python
from your_project.trainer import ModelTrainer

trainer = ModelTrainer()
model = trainer.train(
    training_data="train.csv",
    validation_data="val.csv",
    epochs=10
)
trainer.save_model("model.pkl")
```

## 下一步

完成本教程后，您可以：
- 探索 [高级教程](./tutorial-advanced.md) 了解更复杂的功能
- 查看 [示例](../examples/example1.md) 学习实际应用
- 阅读 [API 文档](../api/index.md) 了解详细接口

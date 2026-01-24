# 快速开始

本文档将引导您完成环境设置、安装和快速运行项目的三个步骤。

## 步骤 1: 环境准备

### 系统要求

- **操作系统**: Windows 10/11, macOS 10.14+, 或 Linux (Ubuntu 18.04+)
- **Python**: 版本 3.8 或更高
- **内存**: 至少 4GB RAM
- **磁盘空间**: 至少 2GB 可用空间

### 环境检查

运行以下命令检查您的环境：

```bash
python --version
pip --version
```

## 步骤 2: 安装

### 方法一: 使用 pip 安装

```bash
# 克隆项目仓库
git clone https://github.com/your-username/your-project.git
cd your-project

# 安装依赖
pip install -r requirements.txt
```

### 方法二: 使用虚拟环境（推荐）

```bash
# 创建虚拟环境
python -m venv venv

# 激活虚拟环境
# Windows
venv\Scripts\activate
# Linux/macOS
source venv/bin/activate

# 安装依赖
pip install -r requirements.txt
```

## 步骤 3: 快速运行

### 运行示例脚本

```bash
python examples/example1.py
```

### 运行测试

```bash
python -m pytest tests/
```

### 启动开发服务器

```bash
python app.py
```

## 验证安装

运行以下命令验证安装是否成功：

```bash
python -c "import your_project; print('安装成功！')"
```

## 下一步

- 查看 [基础教程](./tutorials/tutorial-quick.md) 学习项目基本用法
- 探索 [高级教程](./tutorials/tutorial-advanced.md) 了解高级功能
- 参考 [API 文档](./api/index.md) 查看完整接口说明

## 遇到问题？

- 查看 [FAQ](./faq.md) 常见问题解答
- 查看 [问题排查指南](./troubleshooting.md)（如果存在）
- 在 GitHub 上 [提交问题](https://github.com/your-username/your-project/issues)

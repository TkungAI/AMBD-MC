# 贡献指南

欢迎您为项目做出贡献！本文档将指导您如何参与项目开发。

## 行为准则

参与本项目，您需要遵守我们的行为准则：
- 尊重所有贡献者，无论其经验水平、性别、性取向、残疾、种族、宗教等
- 使用友好和包容的语言
- 尊重不同的观点和经验
- 建设性地接受批评
- 关注对社区最有利的事情
- 对其他社区成员表现出同理心

## 如何开始贡献

### 第一步：了解项目
1. 阅读项目文档，特别是 [README](../README.md) 和 [架构文档](./architecture.md)
2. 查看现有的 [Issues](https://github.com/your-username/your-project/issues) 和 [Pull Requests](https://github.com/your-username/your-project/pulls)
3. 加入我们的 [讨论区](https://github.com/your-username/your-project/discussions) 了解项目动态

### 第二步：设置开发环境
1. Fork 项目仓库
2. 克隆您的 fork：
   ```bash
   git clone https://github.com/your-username/your-project.git
   cd your-project
   ```
3. 设置上游远程：
   ```bash
   git remote add upstream https://github.com/original-owner/your-project.git
   ```
4. 安装开发依赖：
   ```bash
   pip install -e ".[dev]"
   ```
5. 运行测试确保一切正常：
   ```bash
   pytest tests/
   ```

### 第三步：选择任务
- **初学者友好**: 查看标签为 `good-first-issue` 的 Issues
- **文档改进**: 标签为 `documentation` 的 Issues
- **错误修复**: 标签为 `bug` 的 Issues
- **功能开发**: 标签为 `enhancement` 的 Issues

如果您有新的想法，请先在 Issues 中讨论，获得认可后再开始实现。

## 开发流程

### 1. 创建分支
```bash
# 同步主分支
git fetch upstream
git checkout main
git merge upstream/main

# 创建功能分支
git checkout -b feature/your-feature-name
# 或修复分支
git checkout -b fix/issue-number-description
```

**分支命名规范**:
- `feature/` - 新功能
- `fix/` - 错误修复
- `docs/` - 文档更新
- `test/` - 测试相关
- `refactor/` - 代码重构
- `style/` - 代码格式调整（不影响功能）

### 2. 进行更改
- 遵循项目的代码风格
- 为新增功能编写测试
- 更新相关文档
- 确保所有测试通过

### 3. 提交更改
```bash
# 添加更改的文件
git add .

# 提交更改
git commit -m "类型(范围): 描述性信息

详细说明（可选）

解决: #issue-number"
```

**提交信息规范**:
- **类型**: feat, fix, docs, style, refactor, test, chore
- **范围**: 影响的模块或文件
- **描述**: 简明扼要的说明
- **正文**: 详细说明（可选）
- **页脚**: 关联的 Issue

**示例**:
```
feat(api): 添加用户认证端点

- 添加 POST /auth/login 端点
- 实现 JWT 令牌生成
- 添加相关测试用例

解决: #42
```

### 4. 保持分支更新
```bash
# 获取上游更新
git fetch upstream

# 合并到当前分支
git merge upstream/main
# 或使用变基
git rebase upstream/main
```

### 5. 运行测试
```bash
# 运行所有测试
pytest

# 运行特定测试
pytest tests/test_module.py

# 运行带覆盖率的测试
pytest --cov=your_project tests/

# 代码质量检查
flake8 your_project/
black --check your_project/
mypy your_project/
```

### 6. 推送更改
```bash
git push origin feature/your-feature-name
```

### 7. 创建 Pull Request
1. 访问您的仓库页面
2. 点击 "Compare & pull request"
3. 填写 PR 模板
4. 确保 CI 通过
5. 请求代码审查

## 代码规范

### 代码风格
- 遵循 [PEP 8](https://peps.python.org/pep-0008/) (Python)
- 使用 4 空格缩进
- 行长度限制在 88 个字符（使用 Black 格式化）
- 导入排序：标准库 → 第三方库 → 本地模块

### 文档要求
- 所有公共 API 必须有文档字符串
- 使用 Google 风格文档字符串：
  ```python
  def function_name(param1: str, param2: int) -> bool:
      """函数简要描述。
      
      详细描述（可选）。
      
      Args:
          param1: 参数1的描述
          param2: 参数2的描述
      
      Returns:
          返回值的描述
      
      Raises:
          ValueError: 当参数无效时
      """
  ```

### 测试要求
- 测试覆盖率不低于 80%
- 每个功能必须有对应的测试
- 测试应该独立，不依赖外部状态
- 使用有意义的测试名称

## 测试指南

### 单元测试
```python
import pytest
from your_project.module import your_function

def test_your_function_normal_case():
    """测试正常情况"""
    result = your_function(1, 2)
    assert result == 3

def test_your_function_edge_case():
    """测试边界情况"""
    with pytest.raises(ValueError):
        your_function(-1, 2)

def test_your_function_with_fixture(sample_fixture):
    """使用 fixture 的测试"""
    result = your_function(sample_fixture, 2)
    assert result == expected_value
```

### 集成测试
```python
def test_integration_with_database(db_connection):
    """数据库集成测试"""
    # 设置测试数据
    setup_test_data(db_connection)
    
    # 执行测试
    result = query_database(db_connection)
    
    # 验证结果
    assert result == expected_data
    
    # 清理
    cleanup_test_data(db_connection)
```

### 性能测试
```python
import timeit

def test_performance():
    """性能测试"""
    execution_time = timeit.timeit(
        'your_function(1000)',
        setup='from your_project.module import your_function',
        number=1000
    )
    assert execution_time < 1.0  # 1秒内完成1000次执行
```

## 文档贡献

### 文档结构
```
docs/
├── index.md              # 首页
├── getting-started.md    # 快速开始
├── tutorials/           # 教程
├── api/                # API 文档
├── examples/           # 示例
├── architecture.md     # 架构文档
└── contributing.md     # 本文档
```

### 文档标准
- 使用清晰的 Markdown 语法
- 包含代码示例
- 提供相关链接
- 保持文档更新与代码同步
- 对于复杂概念，提供图表说明

### 添加新文档
1. 在合适的位置创建新文件
2. 遵循现有的文档结构
3. 更新相关索引文件
4. 确保链接正确

## 审查流程

### 作为贡献者
1. 确保 PR 描述清晰
2. 解决所有 CI 检查失败
3. 回应审查意见
4. 保持对话礼貌和专业

### 作为审查者
1. 及时响应 PR
2. 提供建设性反馈
3. 检查代码质量和测试覆盖
4. 确保符合项目标准

### PR 合并条件
- [ ] 所有 CI 检查通过
- [ ] 代码审查通过
- [ ] 测试覆盖率不降低
- [ ] 文档已更新
- [ ] 符合代码规范

## 发布流程

### 版本号
遵循 [语义化版本](https://semver.org/lang/zh-CN/)：
- **主版本号**：不兼容的 API 修改
- **次版本号**：向下兼容的功能性新增
- **修订号**：向下兼容的问题修正

### 发布步骤
1. 更新 `CHANGELOG.md`
2. 更新版本号（在 `pyproject.toml` 或 `setup.py` 中）
3. 创建发布分支
4. 运行完整测试套件
5. 构建发布包
6. 创建 GitHub Release
7. 发布到 PyPI（如果适用）

## 社区角色

### 贡献者级别
- **新手**: 完成第一个 PR
- **活跃贡献者**: 多个有意义的贡献
- **核心贡献者**: 对项目有深入了解，有合并权限
- **维护者**: 负责项目方向和发布

### 如何成为核心贡献者
1. 持续贡献高质量代码
2. 积极参与社区讨论
3. 帮助审查其他人的 PR
4. 在 Issues 中帮助其他用户
5. 维护部分模块或文档

## 获取帮助

### 资源
- 📖 [项目文档](./index.md)
- 💬 [讨论区](https://github.com/your-username/your-project/discussions)
- 🐛 [Issue 跟踪](https://github.com/your-username/your-project/issues)
- 📧 邮件列表: project@example.com

### 沟通渠道
- **一般问题**: GitHub Discussions
- **错误报告**: GitHub Issues
- **安全漏洞**: security@example.com（非公开）
- **实时讨论**: Discord/Slack（如果设置）

## 许可证

贡献者同意其贡献将在项目的许可证下发布。当前项目使用 [MIT 许可证](../LICENSE)。

## 致谢

感谢所有为项目做出贡献的人！您的每一点贡献都让项目变得更好。

---

*本文档受 [开源贡献指南模板](https://github.com/opencollective/opensource-template) 启发*

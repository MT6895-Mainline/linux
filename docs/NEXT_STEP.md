# NEXT STEP - CCCI 核心驱动集成

**最后更新**：2026-09-13 03:38  
**工作目录**：`/home/furruka/文档/项目/Kernel/6.18/`

---

## 当前状态

### ✅ 已完成
- **Tag 读取验证**（§80.12）：27 个 tag 在真实设备上解析成功
- **内存布局提取**：SMEM (0x8e000000, 1.1MB) + CCB (0x89000000, 64MB)
- **探针模块稳定**：加载/卸载路径验证通过，无内存泄漏

### 📍 当前位置
- HEAD: `544c54b39e9b` (CCCI util 提交)
- 设备运行内核: build #492 (`6.18.0-g544c54b39e9b-dirty`)
- 日志已归档: `logs/ccci_probe_success_20260913_033408.log`

---

## Phase 1.1: 提取 tag 解析逻辑（当前任务）

### 目标
将探针中的 tag 解析器提取为共用库，供 CCCI 核心驱动使用。

### 操作清单

#### 1. 创建共用解析器
- [ ] 创建 `drivers/misc/mediatek/ccci_util/ccci_tag_parse.h`
  - 导出 `ccci_tag_hdr`、`ccci_smem_layout` 结构
  - 声明 `parse_tag_chain()` 函数接口
  
- [ ] 创建 `drivers/misc/mediatek/ccci_util/ccci_tag_parse.c`
  - 从 `ccci_util_tag_probe.c` 提取核心解析逻辑
  - 保留边界检查与循环检测
  - 添加 EXPORT_SYMBOL_GPL

#### 2. 重构探针模块
- [ ] 修改 `ccci_util_tag_probe.c`
  - 包含 `ccci_tag_parse.h`
  - 移除重复的结构定义
  - 调用共用解析函数
  - 保持 sysfs 接口不变

#### 3. 更新 Kconfig/Makefile
- [ ] `drivers/misc/mediatek/ccci_util/Kconfig`
  - 添加 `CCCI_TAG_PARSE` 选项
  - 设为 `CCCI_UTIL_TAG_PROBE` 的依赖
  
- [ ] `drivers/misc/mediatek/ccci_util/Makefile`
  - 添加 `ccci_tag_parse.o` 到构建规则

#### 4. 验证
- [ ] 主机测试：`make -C test_ccci_tag clean && make -C test_ccci_tag test`
  - 100 项基础测试通过
  - 5 万次变异测试通过
  
- [ ] 设备测试：
  - 重新编译探针模块
  - 推送至设备并加载
  - 验证 tag 读取结果与 §80.12 一致

---

## Phase 1.2: CCCI 核心框架（下一步）

### 准备工作
- [ ] 克隆官方 5.10 仓库到本地：
  ```bash
  git clone --depth 1 --branch oneplus/mt6895_v_15.0.0_ace_race \
    https://github.com/OnePlusOSS/android_kernel_5.10_oneplus_mt6895.git \
    ~/kernel_5.10_mt6895_ref
  ```

### 核心任务
- [ ] 创建 CCCI 核心目录结构：
  ```
  drivers/misc/mediatek/ccci/
  ├── ccci_core.c       # 主驱动
  ├── ccci_core.h       # 内部头文件
  ├── ccci_platform.c   # 平台相关
  ├── Kconfig
  └── Makefile
  ```

- [ ] 实现设备树绑定：
  - Compatible string: `mediatek,mt6895-ccci`
  - 解析 DT 资源（寄存器、中断、内存区域）
  
- [ ] 实现 probe 函数：
  - 调用 `parse_tag_chain()` 获取布局
  - 初始化 SMEM 映射
  - 初始化 CCB 队列结构
  - 创建字符设备节点

---

## Phase 1.3: 基带启动（后续）

### 参考路径
参考官方 5.10 分支中的以下文件：
- `drivers/misc/mediatek/ccci/ccci_md_all.c` - MD 生命周期
- `drivers/misc/mediatek/ccci/ccci_modem.c` - MD 电源管理
- `drivers/misc/mediatek/ccci/ccci_fsm.c` - 状态机

### 关键步骤
- [ ] 实现 MD 电源控制接口
- [ ] 实现 MD 复位序列
- [ ] 实现启动握手协议
- [ ] 验证基带启动（观察 dmesg 基带日志）

---

## 约束与原则

### 禁止
- ❌ 参考 6.12 内核代码
- ❌ 改变探针已验证的接口
- ❌ 破坏现有 tag 读取路径
- ❌ 引入未在设备上验证的内存访问

### 必须
- ✅ 每次修改后运行主机测试
- ✅ 关键节点在设备上验证
- ✅ 保持最小 diff
- ✅ 每个 Phase 完成后提交

---

## 决策待确认

在开始 Phase 1.1 之前需要用户确认：

1. **Git 管理**：是否先提交当前工作？
   - 当前状态：本地领先 1 个提交 + 3 处 stash
   - 建议：先提交探针验证成功的状态，清理 stash

2. **CCCI 构建方式**：模块 vs builtin？
   - 模块：独立开发，易于调试，可热加载
   - Builtin：启动早期可用，避免 vermagic 问题
   - 建议：初期用模块，稳定后改为 builtin

3. **参考仓库**：是否克隆到本地？
   - 优点：离线访问，快速查找
   - 大小：约 1.2GB
   - 建议：克隆到 `~/kernel_5.10_mt6895_ref/`

---

## 验收标准

### Phase 1.1 完成标志
- ✅ 共用解析器代码独立且可复用
- ✅ 探针模块仍能在设备上正常工作
- ✅ 主机测试全部通过
- ✅ 设备测试结果与 §80.12 完全一致
- ✅ Git 提交信息清晰，可回滚

### Phase 1.2 完成标志
- ✅ CCCI 核心模块能加载
- ✅ 能从 DT 正确解析资源
- ✅ 能成功调用 tag 解析器
- ✅ SMEM 映射成功，地址可验证
- ✅ 字符设备节点创建成功

---

## 风险管理

### 高风险操作
- 修改共享内存映射方式
- 改变 tag 解析逻辑的核心算法
- 引入新的内核 API 调用（需验证 6.18 支持）

### 回退方案
- 探针模块保持独立，不受 CCCI core 影响
- 每个 Phase 打 git tag: `ccci-phase-1.1`, `ccci-phase-1.2` 等
- 关键文件保留 `.orig` 备份

---

**等待用户指令开始 Phase 1.1。**

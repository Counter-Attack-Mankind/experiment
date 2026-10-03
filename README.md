# 批量实验

本仓库包含四种实验方案：

1. `Scheme1_Full_GeoTime_LSENLP`：完整方法，包含几何修复、时间增强以及基于 LSE 平滑的具身足迹 NLP。
2. `Scheme2_NoGeo_Time_LSENLP`：去除几何修复的消融实验。
3. `Scheme3_GeoTime_MaxNLP`：使用 exact-max 替代 LSE 流程的消融实验。
4. `Scheme4_body_only_baseline`：采用匹配 Nfe 的仅车身基线方法。

所有批量实验均从 `public/Environment/real` 中加载任务。实验结果保存在仓库根目录下的 `results.csv` 中。

已有结果默认会被保留，除非有意重新运行相同的 scheme/task，或者使用 `-Clear` 参数。

## 命令

运行四种方案下的全部 real 任务：

```powershell
.\RunAllSchemes.bat
```

仅运行某一种方案：

```powershell
.\RunAllSchemes.bat -Scheme1
.\RunAllSchemes.bat -Scheme2
.\RunAllSchemes.bat -Scheme3
.\RunAllSchemes.bat -Scheme4
```

多个方案参数可以组合使用：

```powershell
.\RunAllSchemes.bat -Scheme1 -Scheme3
```

恢复被中断的批量实验：

```powershell
.\RunAllSchemes.bat -Resume
.\RunAllSchemes.bat -Scheme1 -Scheme3 -Resume
```

`-Resume` 会读取 `results.csv` 中所选 scheme/task 对应的 `success` 单元格。

它会跳过：

- `success=1`
- `success=0`

仅运行 `success=NaN` 的条目。

仅重新运行此前失败的条目：

```powershell
.\RunAllSchemes.bat -RetryFailed
.\RunAllSchemes.bat -Scheme3 -RetryFailed
```

`-RetryFailed` 仅运行 `success=0` 的条目。

它会跳过：

- 已成功的条目；
- 从未运行过的 `NaN` 条目。

`-Resume` 与 `-RetryFailed` 不能同时使用。

仅创建或检查 `results.csv`，而不启动 MATLAB：

```powershell
.\RunAllSchemes.bat -InitializeOnly
```

`-InitializeOnly` 保留为一种轻量级操作，用于：

- 检查 19 列结果表结构是否合法；
- 将新发现的 real 任务添加到结果表中；
- 不修改任何已有实验结果。

## Scheme4 的 matched-Nfe 要求

Scheme4 会从：

```text
Scheme1_Full_GeoTime_LSENLP/Nfe_config.txt
```

中读取当前任务对应的 matched Nfe。

当仅单独选择 Scheme4 运行时，所有待运行任务都必须已经在 Scheme1 中存在有效的 Nfe 记录。

如果缺少对应记录，程序会给出明确错误提示；批处理脚本不会自动隐式运行 Scheme1。

默认的全方案执行顺序为：

```text
Scheme1 -> Scheme2 -> Scheme3 -> Scheme4
```

因此，在正常执行全部方案时，Scheme1 会先生成这些 Nfe 记录，随后 Scheme4 再读取并使用它们。

在每一个 scheme/task 开始运行之前，批处理程序都会重置该方案共享的：

```text
runtime/
```

目录，以及顶层的临时求解器输入文件。

这样可以避免某次未完整结束的实验将上一任务的状态或变量遗留到下一任务中。

以下内容不会被这种逐任务重置操作删除：

- 已归档的 `Results/`；
- `results.csv`；
- 已生成的图片；
- Scheme1 持续累积的 `Nfe_config.txt`。

## 清除生成的实验文件

```powershell
.\RunAllSchemes.bat -Clear
```

`-Clear` 必须单独使用，不能与其他参数组合。

它只会删除以下由实验生成的文件：

- 根目录下的 `results.csv`；
- 每个 scheme 下的 `runtime/` 和 `Results/` 目录；
- Scheme1-3 的临时文件：
  - `Area`
  - `PPP`
  - `PV`
  - `ig.INIVAL`
  - `OBCAData.dat`
  - `written_initial_guess_data.mat`
- Scheme4 的临时文件：
  - `Area_scheme4`
  - `PPP_scheme4`
  - `PV_scheme4`
  - `ig_scheme4.INIVAL`
  - `OBCAData_scheme4.dat`
  - `written_initial_guess_data_scheme4.mat`
- Scheme1 生成的 `Nfe_config.txt`；
- 所有 `EFboxs_photo/` 和 `vehiclesbody_photo/` 目录中直接存放的 `*.png` 文件。

两个图片目录本身会被保留。

其中的非 PNG 文件也不会被删除，例如：

- `README`
- `.gitkeep`

`SaveTaskFigure.m` 会按照固定任务编号生成图片：

```text
task_XX.png
```

因此，当重新运行相同任务时，会直接覆盖对应的同名图片。

`-Clear` 不会删除以下内容：

- 源代码；
- `NLP*.mod`；
- `rr*.run`；
- `RunMe*.m`；
- PowerShell/BAT 文件；
- `public/`；
- 求解器文件；
- `public/Environment/real` 中的任务数据；
- `README.md`；
- `ExperimentDesignReview.md`；
- Git 元数据；
- 任何不在上述明确生成文件白名单中的模型或输入定义文件。

## 结果表

`results.csv` 保持现有的 19 列结构。

每一种 scheme 占用四个数值字段，并通过一个空列与下一种 scheme 分隔：

```text
task_id, success, ipopt_cpu_time, collision_percent
```

运行某个选定 scheme 时，只会更新该 scheme 对应的四列数据。

其他 scheme 的已有实验结果不会被修改。

## 图片输出

`SaveTaskFigure.m` 使用固定的、基于任务编号的 PNG 文件名：

- 具身足迹图：

```text
<scheme>/EFboxs_photo/task_XX.png
```

- 真实车身图：

```text
<scheme>/vehiclesbody_photo/task_XX.png
```
## Scheme 1 — Full Method

**本论文采取的实验方法**

`Original Hybrid A*` + `geometric enhancement` + `temporal enhancement` + `LSE-smoothed buffered-footprint NLP`

---

## Scheme 2 — Ablation Experiment

**验证几何增强**

`Original Hybrid A*` + `temporal enhancement` + `LSE-smoothed buffered-footprint NLP`

---

## Scheme 3 — Ablation Experiment

**验证 LSE 平滑**

`Original Hybrid A*` + `geometric enhancement` + `temporal enhancement` + `exact-max buffered-footprint NLP`

---

## Scheme 4 — Body-only Baseline

**验证缓冲避障模型**

`Original Hybrid A*` + `matched-Nfe initialization` + `body-only NLP`


一键运行schem1-scheme4脚本文件
.\RunAllSchemes.bat

中断后继续运行
.\RunAllSchemes.bat -Resume

只初始化results.csv
.\RunAllSchemes.bat -InitializeOnly
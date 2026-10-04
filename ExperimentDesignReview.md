# Scheme1-Scheme4 experiment design review

This document records design findings only. The model definitions below were not changed to resolve these findings.

## Confirmed intended relationships

- Scheme1 and Scheme2 use the same LSE NLP and the same OBCA formulation. Their intended algorithmic difference is the call to `ShrinkWrittenInitialGuessEF` in Scheme1.
- Scheme1 and Scheme3 use the same dynamics, bounds, EF validity conditions, OBCA constraints, and curvature regularization. The max/LSE choice is propagated consistently through trajectory time distribution, initial-guess generation, and the NLP.
- Scheme4 obtains `Nfe` from Scheme1 and uses physical-body OBCA constraints instead of the embodied-footprint constraints.
- All schemes use the same dense collision evaluator at a nominal 100 Hz.

## Findings requiring author review

### High priority

1. **The objective is not total travel time.** All four models minimize `sum(dt[i]^2)`, while `tf=sum(dt[i])`. Therefore the model encourages uniform small intervals and is not mathematically identical to minimizing `tf`. Calling the result "time optimal" requires justification or a model change.

2. **Scheme4 is not constrained to equal time intervals after optimization.** Its initial guess is resampled with equal `dt`, but `NLP4.mod` keeps every `dt[i]` free between `min_dt` and `max_dt`. Thus it is an equal-time initialization, not a strict equal-time NLP baseline.

3. **Scheme1 versus Scheme3 is not an NLP-only smoothing ablation.** Scheme3 also uses `TimeDistributionMax`, `ConvertPathToTrajMax`, and `WriteEFInitialGuessMax`. Consequently the two schemes can have different initial time grids, initial footprints, warm starts, and potentially different `Nfe`. This is valid as a complete method-level comparison, but it does not isolate only the derivative effect of LSE inside IPOPT.

### Medium priority

4. **Shrinking the EF warm start trades one infeasibility for another.** `ShrinkWrittenInitialGuessEF` scales `up/down/left/right` without changing `s`, `k`, or the defining equations. The repaired initial box can satisfy OBCA, but it no longer satisfies `define_up`, `define_down`, `define_left`, and `define_right` until IPOPT repairs those equalities. Scheme1 therefore produces an obstacle-feasible warm start, not a fully NLP-feasible warm start.

5. **OBCA is imposed only for `i=2..Nfe-1`.** The start state and final state are not checked by the NLP collision constraints. This is acceptable only if endpoint safety is guaranteed separately and documented.

6. **The reported collision percentage uses linear pose interpolation.** `EvaluateOptimizationResult` linearly interpolates `x`, `y`, and `theta` between optimized nodes. It does not integrate the bicycle dynamics or reconstruct the exact continuous motion. The metric is consistent across schemes, but should be described as a dense linear-pose check rather than an exact continuous-time collision certificate.

7. **LSE changes the footprint, not only differentiability.** With `alpha=60`, each two-term tie has an overestimate of `log(2)/60`, approximately `0.01155`. At `s=0`, both smooth positive and negative parts are nonzero. Therefore Scheme1 is systematically more conservative near switching points than Scheme3.

### Operational consistency

8. Manual `RunMe*.m` runs default to `Data_test`; `RunAllSchemes.ps1` explicitly selects `public/Environment/real`. Manual and batch runs therefore use different datasets unless `EXPERIMENT_TASK_SOURCE=real` is set for the manual run.

9. Scheme1 enables Hybrid A* debug plotting, while Scheme2-Scheme4 disable it; Scheme2 also calls `VisualizeHybridAstarPath` directly. This does not affect the recorded IPOPT CPU time, but it makes end-to-end wall-clock comparisons unfair.

10. `alpha=60` is duplicated in initialization and NLP code. If it is changed in only one location, the warm start and optimized model will no longer match.

## Suggested interpretation

- Describe Scheme1 versus Scheme2 as an **OBCA-initialization feasibility repair ablation**, not as a comparison of different feasible sets.
- Describe Scheme1 versus Scheme3 as a **complete LSE-based versus exact-max-based pipeline comparison**. Do not call it an NLP-only substitution unless both schemes are fed the same fixed initial trajectory and `Nfe`.
- Describe Scheme4 as an **equal-time-resampled initialization with body-only node constraints** unless equal `dt` is explicitly enforced in the NLP.

## IPOPT initialization and convergence metrics

- `inf_pr_0` is the `inf_pr` value printed for IPOPT iteration 0: the infinity norm of the unscaled original-constraint violation at IPOPT's initialized point. IPOPT may first push supplied primal values into variable bounds, so this is not necessarily the residual of the literal values written to `ig.INIVAL`, and it is not a Euclidean distance to the feasible set.
- `inf_pr_0` is directly comparable between Scheme1 and Scheme2 because they use the same NLP, dimensions, solver, and options. It should not be used as a simple cross-model ranking for Scheme1 versus Scheme3 or Scheme4 because those models use different constraint functions or constraint sets.
- `ipopt_iterations` is the `Number of Iterations` reported by IPOPT and includes restoration-phase iterations. It complements CPU time but does not represent equal computational work across differently sized NLPs.
- Iteration counts from `Maximum CPU time exceeded` runs are retained as stopped/right-censored observations. They must not be described as iterations required for convergence.
- The primary reporting set is optimization success, IPOPT CPU time, `inf_pr_0`, IPOPT iteration count, dense collision percentage, and safety success. A first-near-feasible iteration metric is diagnostic only because `inf_pr` is nonmonotone and the printed log has limited precision.

/**
 * 性能目标：单次仿真（188 帧 × 遮挡计算 + FPV 预计算）< 30 ms。
 * 该断言在弱机器上可能抖动，因此只做宽松上限保护，同时打印实测值供 README 记录。
 */

import { describe, expect, it } from 'vitest';

import { DEFAULT_PARAMS } from '../src/config';
import { runSimulation } from '../src/core/simulate';

describe('性能', () => {
  it('默认参数单次仿真耗时', () => {
    runSimulation(DEFAULT_PARAMS); // 预热（弧长表缓存）
    const n = 10;
    const t0 = performance.now();
    for (let i = 0; i < n; i += 1) runSimulation(DEFAULT_PARAMS);
    const avg = (performance.now() - t0) / n;
    const single = runSimulation(DEFAULT_PARAMS).elapsedMs;
    // eslint-disable-next-line no-console
    console.log(
      `[perf] runSimulation: avg=${avg.toFixed(2)} ms (n=${n}), last=${single.toFixed(2)} ms, frames=${runSimulation(DEFAULT_PARAMS).n}`,
    );
    expect(avg).toBeLessThan(200);
  });
});

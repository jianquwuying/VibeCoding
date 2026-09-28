import { defineConfig } from 'vitest/config';

/**
 * ffmpeg.wasm 0.12.x 依赖 SharedArrayBuffer，而 SharedArrayBuffer 只在**跨源隔离**
 * 的文档里可用。因此 COOP/COEP 是硬前置条件，不是可选项：
 *   - dev（npm run dev）与 preview（npm run preview）都在这里默认开启；
 *   - 若用其它静态服务器托管 dist/，必须自行补这两个响应头，否则导出 MP4 会失败。
 * 开启 COEP 后，任何跨源资源都必须带 CORP/CORS；本项目全部依赖均随 npm 本地提供。
 */
const crossOriginIsolationHeaders = {
  'Cross-Origin-Embedder-Policy': 'require-corp',
  'Cross-Origin-Opener-Policy': 'same-origin',
};

export default defineConfig({
  base: './',
  // @ffmpeg/ffmpeg 内部用 new Worker(new URL(...)) 加载 worker，被 Vite 预打包会失效。
  optimizeDeps: {
    exclude: ['@ffmpeg/ffmpeg', '@ffmpeg/util'],
  },
  server: {
    headers: crossOriginIsolationHeaders,
  },
  preview: {
    headers: crossOriginIsolationHeaders,
  },
  build: {
    target: 'es2022',
    chunkSizeWarningLimit: 4096,
  },
  test: {
    include: ['tests/**/*.test.ts'],
    testTimeout: 120000,
  },
});

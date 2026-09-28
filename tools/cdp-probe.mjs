/**
 * 用 Chrome DevTools Protocol 驱动无头 Edge：打开页面、等待、执行表达式并打印结果。
 *
 * 用法：
 *   node tools/cdp-probe.mjs <url> <waitMs> "<expression>" [pollMs]
 *
 * 若提供 pollMs，则在等待期间每 pollMs 轮询一次表达式，直到结果不是 "PENDING"。
 */

import { spawn } from 'node:child_process';
import { setTimeout as sleep } from 'node:timers/promises';

const EDGE_CANDIDATES = [
  'C:\\Program Files (x86)\\Microsoft\\Edge\\Application\\msedge.exe',
  'C:\\Program Files\\Microsoft\\Edge\\Application\\msedge.exe',
];

const [, , url, waitMsArg, expression, pollMsArg, shotPath, settleMsArg] = process.argv;
if (!url || !expression) {
  console.error(
    'usage: node tools/cdp-probe.mjs <url> <waitMs> "<expr>" [pollMs] [screenshotPath] [settleMs]',
  );
  process.exit(2);
}
const waitMs = Number(waitMsArg ?? '5000');
const pollMs = pollMsArg ? Number(pollMsArg) : 0;
const port = 9333;

const edge = spawn(
  EDGE_CANDIDATES[0],
  [
    '--headless=new',
    '--disable-gpu-sandbox',
    '--enable-unsafe-swiftshader',
    '--no-first-run',
    '--no-default-browser-check',
    // CDP_PROFILE_DIR 可指定独立 profile：复用同一 profile 时 Chrome 的
    // 磁盘缓存偶发 ERR_CACHE_READ_FAILURE，会让模块加载失败（页面空转）。
    `--user-data-dir=${process.env.TEMP ?? '/tmp'}\\${process.env.CDP_PROFILE_DIR ?? 'edge-cdp-probe'}`,
    '--window-size=1600,900',
    `--remote-debugging-port=${port}`,
    'about:blank',
  ],
  { stdio: 'ignore' },
);

async function targets() {
  const res = await fetch(`http://127.0.0.1:${port}/json/list`);
  return res.json();
}

let wsUrl = null;
for (let i = 0; i < 60; i += 1) {
  try {
    const list = await targets();
    const page = list.find((t) => t.type === 'page');
    if (page?.webSocketDebuggerUrl) {
      wsUrl = page.webSocketDebuggerUrl;
      break;
    }
  } catch {
    /* 端口还没起来 */
  }
  await sleep(250);
}
if (!wsUrl) {
  edge.kill();
  console.error('FAILED: 无法连接 CDP');
  process.exit(1);
}

const ws = new WebSocket(wsUrl);
let nextId = 1;
const pending = new Map();
ws.addEventListener('message', (ev) => {
  const msg = JSON.parse(ev.data);
  if (msg.id && pending.has(msg.id)) {
    pending.get(msg.id)(msg);
    pending.delete(msg.id);
  }
});
await new Promise((resolve, reject) => {
  ws.addEventListener('open', resolve);
  ws.addEventListener('error', reject);
});

function send(method, params = {}) {
  const id = nextId++;
  return new Promise((resolve) => {
    pending.set(id, resolve);
    ws.send(JSON.stringify({ id, method, params }));
  });
}

async function evaluate(expr) {
  const res = await send('Runtime.evaluate', {
    expression: expr,
    returnByValue: true,
    awaitPromise: true,
  });
  if (res.result?.exceptionDetails) return `EXCEPTION: ${JSON.stringify(res.result.exceptionDetails)}`;
  return res.result?.result?.value;
}

await send('Page.enable');
await send('Runtime.enable');
await send('Page.navigate', { url });
await sleep(waitMs);

let value = await evaluate(expression);
if (pollMs > 0) {
  const deadline = Date.now() + 15 * 60 * 1000;
  while (
    (String(value).startsWith('WAIT') || String(value).startsWith('PENDING')) &&
    Date.now() < deadline
  ) {
    console.log(`… ${String(value).slice(0, 160)}`);
    await sleep(pollMs);
    value = await evaluate(expression);
  }
}
console.log(typeof value === 'string' ? value : JSON.stringify(value, null, 2));

if (shotPath) {
  await sleep(Number(settleMsArg ?? '1200'));
  const shot = await send('Page.captureScreenshot', { format: 'png' });
  const base64 = shot.result?.data;
  if (!base64) {
    console.error('FAILED: 截图失败');
  } else {
    const { writeFile } = await import('node:fs/promises');
    await writeFile(shotPath, Buffer.from(base64, 'base64'));
    console.log(`screenshot -> ${shotPath}`);
  }
}

ws.close();
edge.kill();
process.exit(0);

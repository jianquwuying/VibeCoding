/**
 * 临时自检入口（?selftest=ffmpeg）：验证跨源隔离 + ffmpeg.wasm 加载 + libx264 编码。
 * 结果写入 document.title，便于无头浏览器读取。
 */

export async function runFfmpegSelftest(): Promise<void> {
  const log: string[] = [];
  try {
    log.push(`coi=${(self as unknown as { crossOriginIsolated?: boolean }).crossOriginIsolated}`);
    const [{ FFmpeg }, coreModule, wasmModule, workerModule] = await Promise.all([
      import('@ffmpeg/ffmpeg'),
      import('@ffmpeg/core?url'),
      import('@ffmpeg/core/wasm?url'),
      import('@ffmpeg/ffmpeg/worker?url'),
    ]);
    log.push(`coreURL=${coreModule.default.split('/').pop()}`);
    const ffmpeg = new FFmpeg();
    ffmpeg.on('log', ({ message }) => {
      if (log.length < 40) log.push(`L:${message.slice(0, 90)}`);
    });
    const t0 = performance.now();
    await ffmpeg.load({
      coreURL: coreModule.default,
      wasmURL: wasmModule.default,
      workerURL: workerModule.default,
    });
    log.push(`load=${(performance.now() - t0).toFixed(0)}ms`);

    const canvas = document.createElement('canvas');
    canvas.width = 320;
    canvas.height = 180;
    const ctx = canvas.getContext('2d')!;
    const frames = 8;
    for (let i = 0; i < frames; i += 1) {
      ctx.fillStyle = `hsl(${i * 40}, 70%, 45%)`;
      ctx.fillRect(0, 0, 320, 180);
      ctx.fillStyle = '#fff';
      ctx.fillText(`frame ${i}`, 12, 24);
      const blob = await new Promise<Blob | null>((res) => canvas.toBlob(res, 'image/png'));
      const bytes = new Uint8Array(await blob!.arrayBuffer());
      await ffmpeg.writeFile(`f_${String(i).padStart(4, '0')}.png`, bytes);
    }
    const t1 = performance.now();
    const rc = await ffmpeg.exec([
      '-framerate',
      '60',
      '-i',
      'f_%04d.png',
      '-c:v',
      'libx264',
      '-pix_fmt',
      'yuv420p',
      '-b:v',
      '4000k',
      '-y',
      'o.mp4',
    ]);
    const out = await ffmpeg.readFile('o.mp4');
    const size = typeof out === 'string' ? out.length : out.byteLength;
    log.push(`exec_rc=${rc} encode=${(performance.now() - t1).toFixed(0)}ms out=${size}B`);
    const head =
      typeof out === 'string'
        ? ''
        : Array.from(out.slice(4, 12))
            .map((b) => b.toString(16).padStart(2, '0'))
            .join('');
    log.push(`ftyp=${head}`);
  } catch (error) {
    log.push(`ERR:${String(error).slice(0, 220)}`);
  }
  document.title = `SELFTEST ${log.join(' | ')}`;
}

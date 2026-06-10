// 地图缓存重建 Web Worker
// 在后台线程处理像素数据，避免阻塞主线程

// 最大缓存边长，超过此值会预先缩放
const MAX_CACHE_SIZE = 720;

self.onmessage = function(e) {
  const { mapData, width, height, colors, resolution } = e.data;
  
  const startTime = performance.now();
  
  // 计算是否需要预缩放
  const maxDimension = Math.max(width, height);
  let targetWidth = width;
  let targetHeight = height;
  let scaleFactor = 1.0;
  
  if (maxDimension > MAX_CACHE_SIZE) {
    scaleFactor = MAX_CACHE_SIZE / maxDimension;
    targetWidth = Math.round(width * scaleFactor);
    targetHeight = Math.round(height * scaleFactor);
    console.log(`[Worker] 地图预缩放: ${width}x${height} -> ${targetWidth}x${targetHeight} (scale: ${scaleFactor.toFixed(3)})`);
  }
  
  // 解析颜色
  const obstacleColor = hexToRgb(colors.obstacle);
  const possibleObstacleColor = hexToRgb(colors.possibleObstacle);
  const unknownColor = hexToRgb(colors.unknown);
  const freeColor = hexToRgb(colors.free);
  
  // 创建像素数据数组
  const totalPixels = targetWidth * targetHeight;
  const pixels = new Uint8ClampedArray(totalPixels * 4);
  
  if (scaleFactor < 1.0) {
    // 需要预缩放 - 使用双线性插值
    for (let targetRow = 0; targetRow < targetHeight; targetRow++) {
      for (let targetCol = 0; targetCol < targetWidth; targetCol++) {
        // 计算源坐标（浮点数）
        const srcX = targetCol / scaleFactor;
        const srcY = (height - 1) - (targetRow / scaleFactor);
        
        // 四个邻近像素坐标（含边界保护）
        const x0_f = Math.floor(srcX);
        const y0_f = Math.floor(srcY);
        const fracX = Math.max(0, Math.min(1, srcX - x0_f));
        const fracY = Math.max(0, Math.min(1, srcY - y0_f));
        const x0 = Math.max(0, Math.min(x0_f, width - 1));
        const x1 = Math.max(0, Math.min(x0_f + 1, width - 1));
        const y0 = Math.max(0, Math.min(y0_f, height - 1));
        const y1 = Math.max(0, Math.min(y0_f + 1, height - 1));
        
        // 获取四个邻近像素的值
        const v00 = mapData[y0 * width + x0];
        const v10 = mapData[y0 * width + x1];
        const v01 = mapData[y1 * width + x0];
        const v11 = mapData[y1 * width + x1];
        
        // 计算未知权重（值为-1表示未知区域）
        const u00 = v00 === -1 ? 1 : 0;
        const u10 = v10 === -1 ? 1 : 0;
        const u01 = v01 === -1 ? 1 : 0;
        const u11 = v11 === -1 ? 1 : 0;
        
        // 双线性插值 - 未知权重
        const unkTop = u00 * (1 - fracX) + u10 * fracX;
        const unkBottom = u01 * (1 - fracX) + u11 * fracX;
        const unknownWeight = unkTop * (1 - fracY) + unkBottom * fracY;
        
        // 将-1（未知）转换为0用于占用值插值
        const n00 = v00 === -1 ? 0 : v00;
        const n10 = v10 === -1 ? 0 : v10;
        const n01 = v01 === -1 ? 0 : v01;
        const n11 = v11 === -1 ? 0 : v11;
        
        // 双线性插值 - 占用值
        const top = n00 * (1 - fracX) + n10 * fracX;
        const bottom = n01 * (1 - fracX) + n11 * fracX;
        const interpolatedValue = top * (1 - fracY) + bottom * fracY;
        
        // 像素索引
        const pixelIndex = (targetRow * targetWidth + targetCol) * 4;
        
        let color;
        // 如果超过半数为未知区域，则标记为未知
        if (unknownWeight > 0.5) {
          color = unknownColor;
        } else {
          const value = Math.round(interpolatedValue);
          if (value > 50) {
            color = obstacleColor;
          } else if (value > 0) {
            color = possibleObstacleColor;
          } else {
            color = freeColor;
          }
        }
        
        pixels[pixelIndex] = color.r;
        pixels[pixelIndex + 1] = color.g;
        pixels[pixelIndex + 2] = color.b;
        pixels[pixelIndex + 3] = 255;
      }
    }
  } else {
    // 不需要预缩放 - 原始处理
    for (let row = 0; row < height; row++) {
      // 翻转行号，实现地图上下翻转
      const flippedRow = height - 1 - row;
      
      for (let col = 0; col < width; col++) {
        // 计算原始数据索引（使用翻转后的行号）
        const dataIndex = flippedRow * width + col;
        const value = mapData[dataIndex];
        
        // 计算像素索引（正常顺序）
        const pixelIndex = (row * width + col) * 4;
        
        let color;
        if (value === -1) {
          color = unknownColor;
        } else if (value > 50) {
          color = obstacleColor;
        } else if (value > 0) {
          color = possibleObstacleColor;
        } else {
          color = freeColor;
        }
        
        pixels[pixelIndex] = color.r;
        pixels[pixelIndex + 1] = color.g;
        pixels[pixelIndex + 2] = color.b;
        pixels[pixelIndex + 3] = 255;
      }
    }
  }
  
  const endTime = performance.now();
  
  // 发送结果回主线程
  self.postMessage({
    pixels: pixels,
    width: targetWidth,
    height: targetHeight,
    originalWidth: width,
    originalHeight: height,
    scaleFactor: scaleFactor,
    processTime: endTime - startTime
  }, [pixels.buffer]); // 使用 Transferable Objects 避免复制
};

// 十六进制颜色转RGB
function hexToRgb(hex) {
  const result = /^#?([a-f\d]{2})([a-f\d]{2})([a-f\d]{2})$/i.exec(hex);
  return result ? {
    r: parseInt(result[1], 16),
    g: parseInt(result[2], 16),
    b: parseInt(result[3], 16)
  } : { r: 0, g: 0, b: 0 };
}

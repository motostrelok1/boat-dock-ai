/**
 * Отрисовка симуляции на Canvas 2D
 */
export class Renderer {
  constructor(canvas, width = 1800, height = 1000) {
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d');
    this.width = width;
    this.height = height;
    this.canvas.width = width;
    this.canvas.height = height;
    
    this.colors = {
      water: '#001a2e',
      waterLight: '#003049',
      island: '#2d5016',
      islandLight: '#3d6b1f',
      islandBorder: '#1a3009',
      gate: '#00e5ff',
      boat: '#e0f7fa',
      boatHull: '#b2ebf2',
      trail: 'rgba(0, 229, 255, 0.3)',
      prediction: '#ff5252',
      text: '#80deea'
    };
  }
  
  clear() {
    const grad = this.ctx.createLinearGradient(0, 0, 0, this.height);
    grad.addColorStop(0, this.colors.water);
    grad.addColorStop(1, this.colors.waterLight);
    this.ctx.fillStyle = grad;
    this.ctx.fillRect(0, 0, this.width, this.height);
  }
  
  drawPlannedRoute(path) {
    if (!path || path.length < 2) return;
    const ctx = this.ctx;
    ctx.save();
    ctx.beginPath();
    ctx.moveTo(path[0].x, path[0].y);
    for (const point of path.slice(1)) ctx.lineTo(point.x, point.y);
    ctx.strokeStyle = '#ffdc39';
    ctx.lineWidth = 3;
    ctx.setLineDash([12, 8]);
    ctx.lineJoin = 'round';
    ctx.stroke();
    ctx.restore();
  }

  drawGrid() {
    this.ctx.strokeStyle = 'rgba(0, 229, 255, 0.05)';
    this.ctx.lineWidth = 1;
    const gridSize = 50;
    
    for (let x = 0; x <= this.width; x += gridSize) {
      this.ctx.beginPath();
      this.ctx.moveTo(x, 0);
      this.ctx.lineTo(x, this.height);
      this.ctx.stroke();
    }
    
    for (let y = 0; y <= this.height; y += gridSize) {
      this.ctx.beginPath();
      this.ctx.moveTo(0, y);
      this.ctx.lineTo(this.width, y);
      this.ctx.stroke();
    }
  }
  
  drawIsland(islandX, islandY, islandW, islandH, gateY, gateWidth, entryX) {
    const ctx = this.ctx;
    
    // Створ дока
    ctx.strokeStyle = 'rgba(0, 229, 255, 0.15)';
    ctx.lineWidth = 1;
    ctx.setLineDash([8, 8]);
    ctx.beginPath();
    ctx.moveTo(entryX, gateY - gateWidth / 2);
    ctx.lineTo(islandX + islandW, gateY - gateWidth / 2);
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(entryX, gateY + gateWidth / 2);
    ctx.lineTo(islandX + islandW, gateY + gateWidth / 2);
    ctx.stroke();
    ctx.setLineDash([]);
    
    // Центральная линия
    ctx.strokeStyle = 'rgba(0, 229, 255, 0.2)';
    ctx.setLineDash([6, 6]);
    ctx.beginPath();
    ctx.moveTo(entryX, gateY);
    ctx.lineTo(islandX + islandW, gateY);
    ctx.stroke();
    ctx.setLineDash([]);
    
    // Точка входа
    ctx.fillStyle = '#00e676';
    ctx.beginPath();
    ctx.arc(entryX, gateY, 4, 0, Math.PI * 2);
    ctx.fill();
    ctx.strokeStyle = '#00e676';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(entryX, gateY - 8);
    ctx.lineTo(entryX, gateY + 8);
    ctx.moveTo(entryX - 8, gateY);
    ctx.lineTo(entryX + 8, gateY);
    ctx.stroke();
    
    // Остров
    const topH = gateY - gateWidth / 2 - islandY;
    const bottomY = gateY + gateWidth / 2;
    const bottomH = islandY + islandH - bottomY;
    
    ctx.fillStyle = this.colors.island;
    ctx.fillRect(islandX, islandY, islandW, topH);
    ctx.fillRect(islandX, bottomY, islandW, bottomH);
    
    ctx.strokeStyle = this.colors.islandBorder;
    ctx.lineWidth = 2;
    if (topH > 0) ctx.strokeRect(islandX, islandY, islandW, topH);
    if (bottomH > 0) ctx.strokeRect(islandX, bottomY, islandW, bottomH);
    
    // Левый край ворот
    ctx.shadowColor = this.colors.gate;
    ctx.shadowBlur = 15;
    ctx.strokeStyle = this.colors.gate;
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(islandX, gateY - gateWidth / 2);
    ctx.lineTo(islandX, gateY + gateWidth / 2);
    ctx.stroke();
    ctx.shadowBlur = 0;
    
    ctx.fillStyle = this.colors.gate;
    ctx.font = 'bold 11px sans-serif';
    ctx.textAlign = 'center';
    ctx.fillText('ВХОД', islandX + islandW / 2, islandY + 15);
  }
  
  drawWaypoints(waypoints, currentIndex) {
    const ctx = this.ctx;
    for (let i = 0; i < waypoints.length; i++) {
      const wp = waypoints[i];
      const isPassed = wp.passed;
      const isCurrent = i === currentIndex && !isPassed;
      
      ctx.beginPath();
      ctx.arc(wp.x, wp.y, 3, 0, Math.PI * 2);
      ctx.fillStyle = isPassed ? '#00e676' : (isCurrent ? '#00e676' : 'rgba(0, 230, 118, 0.5)');
      ctx.fill();
      
      if (isCurrent) {
        ctx.strokeStyle = '#00e676';
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        ctx.arc(wp.x, wp.y, 18, 0, Math.PI * 2);
        ctx.stroke();
      }
      
      ctx.font = 'bold 12px sans-serif';
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillStyle = '#ffffff';
      ctx.fillText(String(i + 1), wp.x, wp.y);
      
      ctx.font = '10px sans-serif';
      ctx.fillStyle = isPassed ? '#00e676' : '#80deea';
      ctx.fillText(isPassed ? '✓' : `${i + 1}`, wp.x, wp.y - 12);
    }
  }
  
  drawAvoidZones(avoidZones) {
    const ctx = this.ctx;
    for (const az of avoidZones) {
      ctx.beginPath();
      ctx.arc(az.x, az.y, az.radius, 0, Math.PI * 2);
      ctx.fillStyle = 'rgba(255, 82, 82, 0.15)';
      ctx.fill();
      ctx.strokeStyle = 'rgba(255, 82, 82, 0.6)';
      ctx.lineWidth = 1.5;
      ctx.stroke();
      
      ctx.beginPath();
      ctx.arc(az.x, az.y, 3, 0, Math.PI * 2);
      ctx.fillStyle = '#ff5252';
      ctx.fill();
      
      ctx.strokeStyle = '#ff5252';
      ctx.lineWidth = 1.5;
      const cs = 6;
      ctx.beginPath();
      ctx.moveTo(az.x - cs, az.y - cs);
      ctx.lineTo(az.x + cs, az.y + cs);
      ctx.moveTo(az.x + cs, az.y - cs);
      ctx.lineTo(az.x - cs, az.y + cs);
      ctx.stroke();
    }
  }
  
  drawBoat(boat) {
    const ctx = this.ctx;
    ctx.save();
    ctx.translate(boat.x, boat.y);
    ctx.rotate(boat.heading);
    
    ctx.fillStyle = this.colors.boatHull;
    ctx.beginPath();
    ctx.ellipse(0, 0, boat.length / 2, boat.width / 2, 0, 0, Math.PI * 2);
    ctx.fill();
    
    ctx.fillStyle = this.colors.boat;
    ctx.beginPath();
    ctx.moveTo(boat.length / 2, 0);
    ctx.lineTo(boat.length / 4, -boat.width / 3);
    ctx.lineTo(boat.length / 4, boat.width / 3);
    ctx.closePath();
    ctx.fill();
    
    ctx.fillStyle = '#4dd0e1';
    ctx.fillRect(-5, -4, 10, 8);
    
    ctx.strokeStyle = '#ff5252';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(-boat.length / 3, 0);
    ctx.lineTo(-boat.length / 2, boat.rudderAngle * 15);
    ctx.stroke();
    
    ctx.strokeStyle = this.colors.gate;
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(boat.length / 2 + 5, 0);
    ctx.lineTo(boat.length / 2 + 15, 0);
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(boat.length / 2 + 12, -3);
    ctx.lineTo(boat.length / 2 + 15, 0);
    ctx.lineTo(boat.length / 2 + 12, 3);
    ctx.stroke();
    
    ctx.restore();
  }
  
  drawTrail(trail) {
    if (trail.length < 2) return;
    const ctx = this.ctx;
    ctx.strokeStyle = this.colors.trail;
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(trail[0].x, trail[0].y);
    for (let i = 1; i < trail.length; i++) {
      ctx.lineTo(trail[i].x, trail[i].y);
    }
    ctx.stroke();
  }
  
  drawPrediction(boat, prediction) {
    const ctx = this.ctx;
    ctx.strokeStyle = this.colors.prediction;
    ctx.lineWidth = 2;
    ctx.setLineDash([8, 4]);
    ctx.beginPath();
    ctx.moveTo(boat.x, boat.y);
    ctx.lineTo(prediction.x, prediction.y);
    ctx.stroke();
    ctx.setLineDash([]);
    
    ctx.fillStyle = this.colors.prediction;
    ctx.beginPath();
    ctx.arc(prediction.x, prediction.y, 5, 0, Math.PI * 2);
    ctx.fill();
    
    ctx.font = '11px sans-serif';
    ctx.textAlign = 'left';
    ctx.fillText('+3с', prediction.x + 8, prediction.y + 3);
  }
  
  drawNetworkDebug(inputs, output) {
    const ctx = this.ctx;
    const x = this.width - 190;
    const y = this.height - 155;
    const w = 180;
    const h = 155;
    
    ctx.fillStyle = 'rgba(0, 10, 20, 0.85)';
    ctx.fillRect(x, y, w, h);
    ctx.strokeStyle = 'rgba(0, 229, 255, 0.3)';
    ctx.lineWidth = 1;
    ctx.strokeRect(x, y, w, h);
    
    ctx.font = 'bold 10px sans-serif';
    ctx.textAlign = 'left';
    ctx.fillStyle = '#00e5ff';
    ctx.fillText('🧠 ВХОДЫ СЕТИ', x + 8, y + 14);
    
    const labels = ['dist', 'angle', 'sin(h)', 'cos(h)', 'avoid_d', 'avoid_a'];
    const barW = 80;
    const barH = 10;
    const startX = x + 8;
    const startY = y + 26;
    
    for (let i = 0; i < 6; i++) {
      const by = startY + i * 18;
      ctx.fillStyle = '#80deea';
      ctx.font = '9px sans-serif';
      ctx.fillText(labels[i], startX, by + 8);
      
      ctx.fillStyle = 'rgba(255,255,255,0.08)';
      ctx.fillRect(startX + 40, by, barW, barH);
      
      let val = inputs[i];
      if (i === 0) val = Math.min(val, 1);
      else val = Math.max(-1, Math.min(1, val));
      
      const fillW = Math.abs(val) * barW;
      ctx.fillStyle = val >= 0 ? '#00e676' : '#ff5252';
      if (i === 0) {
        ctx.fillRect(startX + 40, by, fillW, barH);
      } else {
        const center = startX + 40 + barW / 2;
        ctx.fillRect(val >= 0 ? center : center - fillW, by, fillW, barH);
      }
      
      ctx.fillStyle = '#e0f7fa';
      ctx.font = '9px sans-serif';
      ctx.textAlign = 'right';
      ctx.fillText(val.toFixed(2), startX + 40 + barW + 28, by + 8);
      ctx.textAlign = 'left';
    }
    
    const outY = startY + 6 * 18 + 4;
    ctx.fillStyle = '#ff5252';
    ctx.font = 'bold 10px sans-serif';
    ctx.fillText('→ РУЛЬ', startX, outY + 8);
    
    ctx.fillStyle = 'rgba(255,255,255,0.08)';
    ctx.fillRect(startX + 40, outY, barW, barH);
    
    let rudder = Math.max(-1, Math.min(1, output[0]));
    const fillW = Math.abs(rudder) * barW;
    const center = startX + 40 + barW / 2;
    ctx.fillStyle = rudder >= 0 ? '#00e676' : '#ff5252';
    ctx.fillRect(rudder >= 0 ? center : center - fillW, outY, fillW, barH);
    
    ctx.fillStyle = '#e0f7fa';
    ctx.font = '9px sans-serif';
    ctx.textAlign = 'right';
    ctx.fillText(rudder.toFixed(2), startX + 40 + barW + 28, outY + 8);
    ctx.textAlign = 'left';
  }
  
  drawStartLine() {
    const ctx = this.ctx;
    ctx.strokeStyle = 'rgba(0, 229, 255, 0.3)';
    ctx.lineWidth = 1;
    ctx.setLineDash([5, 5]);
    ctx.beginPath();
    ctx.moveTo(50, 20);
    ctx.lineTo(50, this.height - 20);
    ctx.stroke();
    ctx.setLineDash([]);
    
    ctx.fillStyle = 'rgba(0, 229, 255, 0.5)';
    ctx.font = '11px sans-serif';
    ctx.textAlign = 'center';
    ctx.fillText('СТАРТ', 50, 15);
  }
}

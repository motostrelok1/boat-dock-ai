/**
 * Класс лодки — физика и состояние
 * Нейросеть управляет только рулём
 */
export class Boat {
  constructor(x, y) {
    this.x = x;
    this.y = y;
    this.heading = 0;
    this.speed = 30;
    this.maxTurnRate = 1.2;
    this.rudderAngle = 0;
    this.length = 30;
    this.width = 12;
    this.trail = [];
    this.maxTrailLength = 200;
  }
  
  update(dt, rudderCmd) {
    this.rudderAngle = Math.max(-1, Math.min(1, rudderCmd));
    
    const turnRate = this.rudderAngle * this.maxTurnRate;
    this.heading += turnRate * dt;
    this.heading = this.heading % (Math.PI * 2);
    
    const vx = Math.cos(this.heading) * this.speed;
    const vy = Math.sin(this.heading) * this.speed;
    this.x += vx * dt;
    this.y += vy * dt;
    
    this.trail.push({ x: this.x, y: this.y });
    if (this.trail.length > this.maxTrailLength) {
      this.trail.shift();
    }
  }
  
  getPrediction(seconds) {
    const px = this.x + Math.cos(this.heading) * this.speed * seconds;
    const py = this.y + Math.sin(this.heading) * this.speed * seconds;
    return { x: px, y: py };
  }
  
  getState() {
    return {
      x: this.x,
      y: this.y,
      heading: this.heading,
      speed: this.speed,
      rudderAngle: this.rudderAngle
    };
  }
  
  reset(x, y) {
    this.x = x;
    this.y = y;
    this.heading = 0;
    this.rudderAngle = 0;
    this.trail = [];
  }
}

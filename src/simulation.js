import { Boat } from './boat.js';
import { NeuralNetwork } from './neuralNetwork.js';

// Детерминированный PRNG — для фиксированных карт оценки особей
function mulberry32(a) {
  return function () {
    a |= 0; a = (a + 0x6D2B79F5) | 0;
    let t = Math.imul(a ^ (a >>> 15), 1 | a);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

export class Simulation {
  constructor(width, height) {
    this.width = width;
    this.height = height;
    
    this.boat = new Boat(0, 0);
    
    // 1 выход — только руль. Входы: dist, angle, sin(h), cos(h), avoid_dist, avoid_angle
    this.nn = new NeuralNetwork(6, 8, 1);
    this.aiEnabled = true;
    
    this.generation = 1;
    this.bestScore = -Infinity;
    this.bestNetwork = null;
    this.population = [];
    this.populationSize = 50;
    this.evalMaps = 5; // особь оценивается на N фиксированных картах (средний score)
    this.lastGenerationStats = null;
    
    this.running = true;
    this.finished = false;
    this.score = 0;
    this.time = 0;
    this.maxTime = 200;
    this.trainingActive = false;
    
    this.entryChecked = false;
    this.entrySuccess = false;
    this.prevDist = null;
    this.prevEntryDist = null;
    
    // Параметры острова
    this.islandX = 0;
    this.islandY = 0;
    this.islandW = 0;
    this.islandH = 0;
    this.gateY = 0;
    this.gateWidth = 24;
    this.entryX = 0;
    
    // === Stage 2.1: Waypoints (обязательные точки маршрута) ===
    this.waypoints = []; // [{x, y, passed}]
    this.currentWaypointIndex = 0;
    this.waypointRadius = 18;
    this.waypointBonus = 1000;
    
    // === Stage 2.1: Avoid zones (точки обхода) ===
    this.avoidZones = []; // [{x, y, radius}]
    this.avoidPenalty = 1000;
    this.randomAvoidEnabled = false;
    this.randomAvoidCount = 2;
    this.randomAvoidRadius = 20;
    this.randomWaypointEnabled = false;
    this.randomWaypointCount = 3;
    this.safetyMargin = 12;
    this.holding = null;
    this.notice = '';
    
    this.randomizeLayout();
    this.loadBestBrain();
  }
  
  randomizeLayout(rng = Math.random) {
    this.holding = null;
    this.boat.speed = 30;
    this.notice = '';
    if (this.randomAvoidEnabled) this.avoidZones = [];
    this.islandW = 120 + rng() * 80;
    this.islandH = 180 + rng() * 120;
    this.islandX = this.width - this.islandW + 20;
    this.islandY = 40 + rng() * (this.height - this.islandH - 80);
    
    this.gateWidth = this.boat.width * 3.5;
    this.gateY = this.islandY + 30 + rng() * (this.islandH - 60);
    
    this.entryX = this.islandX - 60;
    
    let startX, startY;
    let attempts = 0;
    do {
      startX = 40 + rng() * Math.max(1, this.entryX - 80);
      startY = 40 + rng() * (this.height - 80);
      attempts++;
    } while (this.isOverIsland(startX, startY) && attempts < 100);
    
    const startHeading = rng() * Math.PI * 2;
    
    this.boat.reset(startX, startY);
    this.boat.heading = startHeading;
    this.boat.trail = [];
    
    this.currentWaypointIndex = 0;
    this.waypoints.forEach(wp => wp.passed = false);
    if (this.randomWaypointEnabled) this.generateRandomWaypoints(rng);
    if (this.randomAvoidEnabled) this.generateRandomAvoidZones(rng);
    this.checkLayoutSafety();
  }
  
  // Check the boundary of every danger zone, including hull clearance.
  zoneClearance(zone, x, y, radius = 0) {
    return Math.hypot(x - zone.x, y - zone.y) - zone.radius - radius - this.safetyMargin;
  }

  isValidWaypoint(x, y) {
    const hull = this.boat.length / 2;
    return x >= hull + this.safetyMargin && y >= hull + this.safetyMargin &&
      x <= this.entryX - hull - this.safetyMargin &&
      y <= this.height - hull - this.safetyMargin &&
      !this.isOverIsland(x,y) &&
      this.avoidZones.every(z => this.zoneClearance(z,x,y,hull) > 0) &&
      Math.hypot(x-this.boat.x,y-this.boat.y) > hull + 10 &&
      this.waypoints.every(wp => Math.hypot(x-wp.x,y-wp.y)>hull*2+this.safetyMargin);
  }

  generateRandomWaypoints(rng = Math.random) {
    this.clearWaypoints();
    const count = this.randomWaypointCount;
    const lo = this.boat.length/2 + this.safetyMargin + 5;
    const hi = this.entryX - lo;
    for(let i=0;i<count;i++){
      for(let attempt=0;attempt<300;attempt++){
        const x=lo+rng()*Math.max(1,hi-lo);
        const y=lo+rng()*Math.max(1,this.height-2*lo);
        if(this.isValidWaypoint(x,y)) { this.addWaypoint(x,y);break; }
      }
    }
    if(this.waypoints.length<count) this.notice = 'Не все случайные точки помещаются безопасно';
    return this.waypoints.length;
  }

  getLayoutConflict() {
    for (const wp of this.waypoints) {
      for (const z of this.avoidZones) {
        if (this.zoneClearance(z,wp.x,wp.y,this.boat.length/2) <= 0) return 'Точка маршрута пересекает запретную зону';
      }
    }
    for(let i=0;i<this.avoidZones.length;i++){
      const a=this.avoidZones[i];
      for(let j=i+1;j<this.avoidZones.length;j++){
        const b=this.avoidZones[j];
        if(Math.hypot(a.x-b.x,a.y-b.y)<=a.radius+b.radius+this.safetyMargin) return 'Запретные зоны пересекаются';
      }
    }
    return null;
  }

  // Conservative 2-D grid flood-fill; verifies geometric connectivity, not dynamic steering feasibility.
  getRouteReachability(step = 12) {
    const hull = this.boat.length / 2 + this.safetyMargin;
    const cols = Math.floor(this.width / step) + 1;
    const rows = Math.floor(this.height / step) + 1;
    const free = (x,y) => {
      if (x < hull || y < hull || x > this.width-hull || y > this.height-hull) return false;
      if(this.avoidZones.some(z => this.zoneClearance(z,x,y,this.boat.length/2) <= 0)) return false;
      // The dock is behind the island. This check is only for the approach to its entry line.
      const blockedIsland = x >= this.islandX-hull && x <= this.islandX+this.islandW+hull &&
        y >= this.islandY-hull && y <= this.islandY+this.islandH+hull &&
        Math.abs(y-this.gateY) >= this.gateWidth/2-hull;
      return !blockedIsland;
    };
    const index=(x,y)=>Math.max(0,Math.min(cols-1,Math.round(x/step)))+
      cols*Math.max(0,Math.min(rows-1,Math.round(y/step)));
    const centers=i=>({x:(i%cols)*step,y:Math.floor(i/cols)*step});
    const targets=[...this.waypoints,{x:this.entryX,y:this.gateY}];
    let start={x:this.boat.x,y:this.boat.y};
    for(let targetIndex=0;targetIndex<targets.length;targetIndex++){
      const target=targets[targetIndex];
      if(!free(start.x,start.y)||!free(target.x,target.y))
        return {reachable:false,reason:`Точка ${targetIndex+1} находится в запретной области`};
      const source=index(start.x,start.y), goal=index(target.x,target.y);
      const seen=new Uint8Array(cols*rows), queue=new Int32Array(cols*rows);
      let head=0,tail=0;queue[tail++]=source;seen[source]=1;
      const directions=[[1,0],[-1,0],[0,1],[0,-1],[1,1],[-1,-1],[1,-1],[-1,1]];
      const validSegment=(a,b)=>{
        for(let j=1;j<=4;j++){
          const t=j/4;
          if(!free(a.x+(b.x-a.x)*t,a.y+(b.y-a.y)*t))return false;
        }
        return true;
      };
      while(head<tail&&!seen[goal]){
        const current=queue[head++],p=centers(current);
        for(const [dx,dy] of directions){
          const cx=current%cols+dx,cy=Math.floor(current/cols)+dy;
          if(cx<0||cy<0||cx>=cols||cy>=rows)continue;
          const next=cy*cols+cx;
          if(seen[next])continue;
          const point=centers(next);
          if(!free(point.x,point.y)||!validSegment(p,point))continue;
          seen[next]=1;queue[tail++]=next;
        }
      }
      if(!seen[goal]) return {reachable:false,reason:`Нет прохода к ${targetIndex<targets.length-1?'маршрутной точке '+(targetIndex+1):'входу в док'}`};
      start=target;
    }
    return {reachable:true,reason:null};
  }

  // A holding circle is only allowed when its entire swept path stays safe.
  findSafeHoldingTurn() {
    const speed=10, dt=0.05, steps=200;
    for(const rudder of [-1,1]){
      let {x,y,heading}=this.boat;
      let safe=true;
      for(let i=0;i<steps;i++){
        heading+=rudder*this.boat.maxTurnRate*dt;
        x+=Math.cos(heading)*speed*dt;
        y+=Math.sin(heading)*speed*dt;
        const hull=this.boat.length/2+this.safetyMargin;
        if(x<hull||x>this.width-hull||y<hull||y>this.height-hull||
          this.isOverIsland(x,y)||
          this.avoidZones.some(z=>this.zoneClearance(z,x,y,this.boat.length/2)<=0)){
          safe=false;break;
        }
      }
      if(safe) return rudder;
    }
    return null;
  }

  checkLayoutSafety() {
    if(this.holding) return;
    const conflict=this.getLayoutConflict() || (this.avoidZones.length ? this.getRouteReachability().reason : null);
    if(!conflict) return;
    // During batch training, report invalid layouts without running wall-clock waits.
    if(this.trainingActive) { this.notice=conflict;return; }
    const rudder=this.findSafeHoldingTurn();
    if(rudder===null) {
      // Never move toward an obstacle when no physically safe waiting loop exists.
      this.clearAvoidZones();
      this.notice='Небезопасная карта: нет места для ожидания, запреты сняты немедленно';
      return;
    }
    this.holding={remaining:10,rudder,previousSpeed:this.boat.speed};
    this.boat.speed=10;
    this.notice=`⚠️ ${conflict}. Ожидание 10 с на скорости 10 px/с`;
  }

  // === Waypoints ===
  addWaypoint(x, y) {
    this.waypoints.push({ x, y, passed: false });
    if (!this.randomWaypointEnabled) this.checkLayoutSafety();
  }
  
  removeWaypointAt(x, y) {
    const idx = this.findWaypointAt(x, y);
    if (idx >= 0) {
      this.waypoints.splice(idx, 1);
      this.currentWaypointIndex = 0;
      this.waypoints.forEach(wp => wp.passed = false);
      return true;
    }
    return false;
  }
  
  findWaypointAt(x, y) {
    const clickRadius = 12;
    for (let i = 0; i < this.waypoints.length; i++) {
      const wp = this.waypoints[i];
      if (Math.hypot(wp.x - x, wp.y - y) <= clickRadius) return i;
    }
    return -1;
  }
  
  clearWaypoints() {
    this.waypoints = [];
    this.currentWaypointIndex = 0;
  }
  
  // === Avoid zones ===
  addAvoidZone(x, y, radius) {
    this.avoidZones.push({ x, y, radius });
    this.checkLayoutSafety();
  }
  
  removeAvoidZoneAt(x, y) {
    const idx = this.findAvoidZoneAt(x, y);
    if (idx >= 0) {
      this.avoidZones.splice(idx, 1);
      return true;
    }
    return false;
  }
  
  findAvoidZoneAt(x, y) {
    const clickRadius = 15;
    for (let i = 0; i < this.avoidZones.length; i++) {
      const az = this.avoidZones[i];
      if (Math.hypot(az.x - x, az.y - y) <= clickRadius) return i;
    }
    return -1;
  }
  
  clearAvoidZones() {
    this.avoidZones = [];
  }
  
  // Generate new obstacles from the layout RNG, including seeded training layouts.
  generateRandomAvoidZones(rng = Math.random) {
    this.avoidZones = [];
    const radius = this.randomAvoidRadius;
    const count = this.randomAvoidCount;
    for (let i = 0; i < count; i++) {
      let placed = false;
      for (let attempt = 0; attempt < 200 && !placed; attempt++) {
        const x = radius + 20 + rng() * (this.entryX - radius - 60);
        const y = radius + 20 + rng() * (this.height - 2 * (radius + 20));
        if (Math.hypot(x - this.boat.x, y - this.boat.y) < radius + 60) continue;
        if (Math.hypot(x - this.entryX, y - this.gateY) < radius + 75) continue;
        if (this.waypoints.some(wp => this.zoneClearance({x,y,radius},wp.x,wp.y,this.boat.length/2) <= 0)) continue;
        if (this.avoidZones.some(zone => Math.hypot(x-zone.x,y-zone.y) <= zone.radius+radius+this.safetyMargin)) continue;
        if (this.isOverIsland(x,y)) continue;
        this.avoidZones.push({x,y,radius});
        placed = true;
      }
    }
    if(this.avoidZones.length<count) this.notice='Не все запретные зоны помещаются безопасно';
    return this.avoidZones.length;
  }

  // === Навигация ===
  getNavTarget() {
    if (this.currentWaypointIndex < this.waypoints.length) {
      const wp = this.waypoints[this.currentWaypointIndex];
      return { x: wp.x, y: wp.y };
    }
    
    if (!this.entryChecked || !this.entrySuccess) {
      return { x: this.entryX, y: this.gateY };
    } else {
      return { x: this.islandX + this.islandW + 60, y: this.gateY };
    }
  }
  
  getInputs() {
    const target = this.getNavTarget();
    const dx = target.x - this.boat.x;
    const dy = target.y - this.boat.y;
    const dist = Math.sqrt(dx * dx + dy * dy);
    const angleToTarget = Math.atan2(dy, dx);
    const angleDiff = this.normalizeAngle(angleToTarget - this.boat.heading);
    const diag = Math.sqrt(this.width ** 2 + this.height ** 2);
    
    let avoidDist = Infinity;
    let avoidAngle = 0;
    
    if (this.avoidZones.length > 0) {
      let nearestAz = null;
      for (const az of this.avoidZones) {
        const d = Math.hypot(az.x - this.boat.x, az.y - this.boat.y);
        if (d < avoidDist) {
          avoidDist = d;
          nearestAz = az;
        }
      }
      if (nearestAz) {
        const azDx = nearestAz.x - this.boat.x;
        const azDy = nearestAz.y - this.boat.y;
        avoidAngle = this.normalizeAngle(Math.atan2(azDy, azDx) - this.boat.heading);
      }
    }
    
    return [
      dist / diag,
      angleDiff / Math.PI,
      Math.sin(this.boat.heading),
      Math.cos(this.boat.heading),
      avoidDist === Infinity ? 0 : avoidDist / diag,
      avoidAngle / Math.PI
    ];
  }
  
  normalizeAngle(angle) {
    while (angle > Math.PI) angle -= Math.PI * 2;
    while (angle < -Math.PI) angle += Math.PI * 2;
    return angle;
  }
  
  update(dt) {
    if (!this.running || this.finished) return;
    if(this.holding){
      const slice=Math.min(dt,this.holding.remaining);
      this.boat.speed=10;
      this.boat.update(slice,this.holding.rudder);
      this.time+=slice;
      this.holding.remaining-=slice;
      if(this.holding.remaining<=0){
        this.boat.speed=this.holding.previousSpeed;
        this.holding=null;
        this.clearAvoidZones();
        this.notice='✅ 10 секунд прошло. Запретные зоны удалены, продолжаем маршрут';
      }
      return;
    }
    
    this.time += dt;
    
    if (this.time >= this.maxTime) {
      this.finished = true;
      this.score = this.calculateScore() - 500;
      return;
    }
    
    if (this.boat.x < -50 || this.boat.x > this.width + 50 ||
        this.boat.y < -50 || this.boat.y > this.height + 50) {
      this.finished = true;
      this.score = this.calculateScore() - 2000;
      return;
    }
    
    this.checkWaypoints();
    this.checkEntryPoint();
    if (this.finished) return;
    
    if (this.checkIslandCollision()) {
      this.finished = true;
      this.score = this.calculateScore() - 5000;
      return;
    }
    
    if (this.checkAvoidZones()) {
      this.finished = true;
      this.score = this.calculateScore() - this.avoidPenalty;
      return;
    }
    
    if (this.checkDockEntry()) {
      this.finished = true;
      this.score = this.calculateScore() + 10000 + (this.maxTime - this.time) * 100;
      return;
    }
    
    let rudder = 0;
    
    if (this.aiEnabled) {
      const inputs = this.getInputs();
      const output = this.nn.forward(inputs);
      rudder = output[0];
    } else {
      // Ручной режим: сохраняем руль, заданный со стрелок (boat.rudderAngle)
      rudder = this.boat.rudderAngle;
    }
    
    // === DENSE REWARD ===
    const entryDist = Math.hypot(this.entryX - this.boat.x, this.gateY - this.boat.y);
    if (this.prevEntryDist == null) this.prevEntryDist = entryDist;
    this.score += (this.prevEntryDist - entryDist) * 1.0;
    this.prevEntryDist = entryDist;
    
    const target = this.getNavTarget();
    const dx = target.x - this.boat.x;
    const dy = target.y - this.boat.y;
    const currDist = Math.sqrt(dx * dx + dy * dy);
    const angleToTarget = Math.atan2(dy, dx);
    const angleDiff = this.normalizeAngle(angleToTarget - this.boat.heading);
    
    if (this.prevDist === null) this.prevDist = currDist;
    
    this.score += this.boat.speed * Math.cos(angleDiff) * dt * 20;
    this.score += (this.prevDist - currDist) * 1;
    
    const deepBonus = this.entrySuccess ? 8 : 2;
    this.score += this.boat.speed * Math.cos(this.boat.heading) * dt * deepBonus;
    this.score -= Math.abs(rudder) * dt * 2;
    
    this.prevDist = currDist;
    
    const previousPosition = {x:this.boat.x, y:this.boat.y};
    this.boat.update(dt, rudder);
    this.checkWaypoints(previousPosition);
  }
  
  checkWaypoints(previousPosition = null) {
    if (this.currentWaypointIndex >= this.waypoints.length) return;
    
    const wp = this.waypoints[this.currentWaypointIndex];
    const ax=previousPosition?.x ?? this.boat.x, ay=previousPosition?.y ?? this.boat.y;
    const dx=this.boat.x-ax, dy=this.boat.y-ay, d2=dx*dx+dy*dy;
    const t=d2 ? Math.max(0,Math.min(1,((wp.x-ax)*dx+(wp.y-ay)*dy)/d2)) : 0;
    const dist=Math.hypot(ax+t*dx-wp.x, ay+t*dy-wp.y);
    
    if (dist <= this.waypointRadius) {
      wp.passed = true;
      this.score += this.waypointBonus;
      this.currentWaypointIndex++;
      this.prevDist = null;
    }
  }
  
  checkIslandCollision() {
    const boat = this.boat;
    const ix = this.islandX;
    const iy = this.islandY;
    const iw = this.islandW;
    const ih = this.islandH;
    const gy = this.gateY;
    const gw = this.gateWidth;
    
    if (boat.x > ix && boat.x < ix + iw && boat.y > iy && boat.y < iy + ih) {
      if (boat.y < gy - gw / 2 || boat.y > gy + gw / 2) {
        return true;
      }
    }
    return false;
  }
  
  checkAvoidZones() {
    for (const az of this.avoidZones) {
      const dist = Math.hypot(this.boat.x - az.x, this.boat.y - az.y);
      if (dist <= az.radius) {
        return true;
      }
    }
    return false;
  }
  
  checkEntryPoint() {
    // Пока не пройдены все точки маршрута, линия входа не проверяется
    if (this.entryChecked || this.currentWaypointIndex < this.waypoints.length) return;
    if (this.boat.x <= this.entryX) return;
    
    this.entryChecked = true;
    
    const inGateY = Math.abs(this.boat.y - this.gateY) < this.gateWidth / 2;
    const alignedHeading = Math.abs(this.normalizeAngle(this.boat.heading)) < 0.3;
    
    if (inGateY && alignedHeading) {
      this.entrySuccess = true;
      this.prevDist = null;
    } else {
      this.entrySuccess = false;
      this.finished = true;
      this.score = this.calculateScore() - 3000;
    }
  }
  
  checkDockEntry() {
    const passedDock = this.boat.x > this.islandX + this.islandW;
    const alignedY = Math.abs(this.boat.y - this.gateY) < this.gateWidth / 2;
    return passedDock && alignedY && this.entrySuccess;
  }
  
  isOverIsland(x, y) {
    if (x >= this.islandX && x <= this.islandX + this.islandW) {
      const topH = this.gateY - this.gateWidth / 2 - this.islandY;
      const bottomY = this.gateY + this.gateWidth / 2;
      const bottomH = this.islandY + this.islandH - bottomY;
      
      if (y >= this.islandY && y <= this.islandY + topH) return true;
      if (y >= bottomY && y <= bottomY + bottomH) return true;
    }
    return false;
  }
  
  calculateScore() {
    const target = this.getNavTarget();
    const dx = target.x - this.boat.x;
    const dy = target.y - this.boat.y;
    const dist = Math.sqrt(dx * dx + dy * dy);
    const angleToTarget = Math.atan2(dy, dx);
    const angleDiff = Math.abs(this.normalizeAngle(angleToTarget - this.boat.heading));
    
    const yOffset = Math.abs(this.boat.y - this.gateY);
    
    let score = this.score;
    
    score -= dist * 0.3;
    score -= angleDiff * 10;
    score -= yOffset * 0.2;
    
    return score;
  }
  
  reset(network = null, randomize = true) {
    if (randomize) {
      this.randomizeLayout();
    }
    this.time = 0;
    this.holding = null;
    this.notice = '';
    this.boat.speed = 30;
    this.finished = false;
    this.running = true;
    this.score = 0;
    this.entryChecked = false;
    this.entrySuccess = false;
    this.prevDist = null;
    this.prevEntryDist = null;
    
    this.currentWaypointIndex = 0;
    this.waypoints.forEach(wp => wp.passed = false);
    
    if (network) {
      this.nn = network;
    }
  }
  
  saveBestBrain() {
    if (!this.bestNetwork) return;
    const data = {
      network: this.bestNetwork.toJSON(),
      generation: this.generation,
      bestScore: this.bestScore,
      savedAt: new Date().toISOString()
    };
    localStorage.setItem('boat-dock-ai-best', JSON.stringify(data));
  }
  
  loadBestBrain() {
    try {
      const raw = localStorage.getItem('boat-dock-ai-best');
      if (!raw) return;
      const data = JSON.parse(raw);
      if (data.network.outputSize !== 1) {
        localStorage.removeItem('boat-dock-ai-best');
        return;
      }
      let loaded = NeuralNetwork.fromJSON(data.network);
      if (loaded.inputSize < 6) {
        loaded = loaded.migrateInputSize(6);
      }
      this.nn = loaded;
      this.bestNetwork = loaded;
      this.generation = data.generation || 1;
      this.bestScore = data.bestScore || -Infinity;
    } catch (e) {
      console.warn('Не удалось загрузить сохранённую сеть:', e);
    }
  }
  
  exportBrain() {
    const network = this.bestNetwork || this.nn;
    const data = {
      network: network.toJSON(),
      generation: this.generation,
      bestScore: this.bestScore,
      exportedAt: new Date().toISOString()
    };
    const blob = new Blob([JSON.stringify(data, null, 2)], { type: 'application/json' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = `boat-dock-brain-gen${this.generation}.json`;
    a.click();
    URL.revokeObjectURL(url);
  }
  
  importBrain(jsonString) {
    const data = JSON.parse(jsonString);
    if (data.network.outputSize !== 1) throw new Error('Несовместимый мозг');
    let loaded = NeuralNetwork.fromJSON(data.network);
    if (loaded.inputSize < 6) {
      loaded = loaded.migrateInputSize(6);
    }
    this.nn = loaded;
    this.bestNetwork = loaded;
    this.generation = data.generation || 1;
    this.bestScore = data.bestScore || -Infinity;
    this.population = []; // загруженный мозг должен стать основой новой популяции
    this.saveBestBrain();
    this.reset(this.nn, true);
  }
  
  initPopulation() {
    this.population = [];
    const base = this.bestNetwork || this.nn;
    this.population.push({ network: base, score: 0 });
    
    for (let i = 0; i < 15; i++) {
      this.population.push({
        network: base.mutate(0.1 + Math.random() * 0.2, 0.2 + Math.random() * 0.3),
        score: 0
      });
    }
    
    while (this.population.length < this.populationSize) {
      this.population.push({
        network: new NeuralNetwork(6, 8, 1),
        score: 0
      });
    }
  }
  
  // Оценка на фиксированных картах: сид = f(поколение, номер карты),
  // поэтому все особи одного поколения играют на одинаковых картах,
  // а фитнес перестаёт быть лотереей.
  evaluateIndividual(index) {
    const individual = this.population[index];
    const dt = 0.05;
    let total = 0, successes = 0;
    
    for (let k = 0; k < this.evalMaps; k++) {
      const rng = mulberry32(this.generation * 10007 + k * 131 + 17);
      this.reset(individual.network, false);
      this.randomizeLayout(rng);
      
      while (!this.finished && this.time < this.maxTime) {
        this.update(dt);
      }
      
      total += this.score;
      if (this.checkDockEntry()) successes++;
    }
    
    individual.score = total / this.evalMaps;
    individual.successRate = successes / this.evalMaps;
    
    if (individual.score > this.bestScore) {
      this.bestScore = individual.score;
      this.bestNetwork = individual.network;
    }
  }
  
  // Repeatable diagnostics on the same set of scenarios (independent of training state).
  benchmark(network = this.bestNetwork || this.nn, seeds=Array.from({length:20},(_,i)=>1001+i)) {
    const sim = new Simulation(this.width,this.height);
    sim.aiEnabled=true;
    sim.clearWaypoints(); sim.clearAvoidZones();
    const runs=[];
    for (const seed of seeds) {
      sim.reset(network,false);
      sim.randomizeLayout(mulberry32(seed));
      const layout={islandX:sim.islandX,islandY:sim.islandY,islandW:sim.islandW,
        islandH:sim.islandH,gateY:sim.gateY,gateWidth:sim.gateWidth,entryX:sim.entryX};
      const trail=[{x:sim.boat.x,y:sim.boat.y}];
      let steps=0;
      while(!sim.finished && sim.time<sim.maxTime) {
        sim.update(0.05);
        if(++steps%10===0)trail.push({x:sim.boat.x,y:sim.boat.y});
      }
      const success=sim.finished && sim.checkDockEntry();
      runs.push({seed,success,score:sim.score,time:sim.time,layout,trail});
    }
    const successes=runs.filter(r=>r.success).length;
    return {runs,summary:{total:runs.length,successes,failures:runs.length-successes,
      successRate:runs.length?successes/runs.length:0,
      averageScore:runs.length?runs.reduce((sum,r)=>sum+r.score,0)/runs.length:0}};
  }

  computeGenerationStats() {
    const n = this.population.length || 1;
    const meanScore = this.population.reduce((a, p) => a + p.score, 0) / n;
    const meanSuccess = this.population.reduce((a, p) => a + (p.successRate || 0), 0) / n;
    const best = this.population[0];
    this.lastGenerationStats = {
      generation: this.generation,
      meanScore,
      meanSuccess,
      bestScore: best ? best.score : 0,
      bestSuccess: best ? best.successRate || 0 : 0
    };
    console.log(
      `[gen ${this.generation}] best: ${this.lastGenerationStats.bestScore.toFixed(0)} ` +
      `(успех ${(this.lastGenerationStats.bestSuccess * 100).toFixed(0)}%), ` +
      `среднее: ${meanScore.toFixed(0)} (успех ${(meanSuccess * 100).toFixed(0)}%)`
    );
  }
  
  evaluateGeneration() {
    for (let i = 0; i < this.population.length; i++) {
      this.evaluateIndividual(i);
    }
    this.population.sort((a, b) => b.score - a.score);
    this.computeGenerationStats();
  }
  
  async evaluateGenerationAsync(onIndividualDone = null) {
    for (let i = 0; i < this.population.length; i++) {
      if (!this.trainingActive) return;
      this.evaluateIndividual(i);
      if (onIndividualDone) {
        onIndividualDone(i + 1, this.population.length, this.bestScore);
      }
      if (i % 5 === 4) {
        await new Promise(r => requestAnimationFrame(r));
      }
    }
    this.population.sort((a, b) => b.score - a.score);
    this.computeGenerationStats();
  }
  
  nextGeneration() {
    const eliteKeep = Math.min(3, this.population.length);
    const eliteMutate = Math.floor(this.populationSize * 0.2);
    const newPopulation = [];
    
    // Элиты сохраняются без мутации — лучшие решения не разрушаются
    for (let i = 0; i < eliteKeep; i++) {
      newPopulation.push({
        network: this.population[i].network,
        score: 0
      });
    }
    
    for (let i = 0; i < eliteMutate; i++) {
      newPopulation.push({
        network: this.population[i].network.mutate(0.05, 0.1),
        score: 0
      });
    }
    
    while (newPopulation.length < this.populationSize) {
      const parent1 = this.tournamentSelect();
      const parent2 = this.tournamentSelect();
      let child;
      if (Math.random() < 0.7) {
        child = NeuralNetwork.crossover(parent1.network, parent2.network);
      } else {
        child = parent1.network.mutate(0.1, 0.2);
      }
      child = child.mutate(0.2, 0.3);
      newPopulation.push({ network: child, score: 0 });
    }
    
    this.population = newPopulation;
    this.generation++;
  }
  
  tournamentSelect(tournamentSize = 5) {
    let best = this.population[Math.floor(Math.random() * this.population.length)];
    for (let i = 1; i < tournamentSize; i++) {
      const candidate = this.population[Math.floor(Math.random() * this.population.length)];
      if (candidate.score > best.score) best = candidate;
    }
    return best;
  }
  
  async trainContinuous(onProgress = null, onGenerationEnd = null) {
    this.trainingActive = true;
    if (this.population.length === 0) this.initPopulation();
    
    while (this.trainingActive) {
      await this.evaluateGenerationAsync((current, total, bestScore) => {
        if (onProgress) onProgress(this.generation, current, total, bestScore);
      });
      if (!this.trainingActive) break;
      this.nextGeneration();
      if (onGenerationEnd) onGenerationEnd(this.generation, this.bestScore, this.bestNetwork, this.lastGenerationStats);
    }
    
    if (this.bestNetwork) this.nn = this.bestNetwork;
    this.saveBestBrain();
    this.reset(this.nn, true);
  }
  
  stopTraining() {
    this.trainingActive = false;
  }
  
  mutateCurrent() {
    this.nn = this.nn.mutate(0.3, 0.5);
    this.reset(this.nn, true);
  }
  
  resetBrain() {
    localStorage.removeItem('boat-dock-ai-best');
    this.nn = new NeuralNetwork(6, 8, 1);
    this.bestNetwork = null;
    this.bestScore = -Infinity;
    this.generation = 1;
    this.population = [];
    this.reset(null, true);
  }
}

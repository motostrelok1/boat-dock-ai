import { minimumTurnRadius, validateTurnRadius } from './turnRadius.js';

// Circular tangent fillets for a collision-free polygonal path.
// Returns provisional geometry if any corner cannot be replaced safely.
export function buildTurnArcRoute(sim, path, speed = 30) {
  if (!path || path.length < 2) return {valid:false,path:[],reason:'Маршрут отсутствует'};
  const radius = minimumTurnRadius(sim.boat,speed) * 1.15;
  const margin = sim.boat.length/2 + sim.safetyMargin;
  const safe = p => {
    if(p.x<margin||p.y<margin||p.x>sim.width-margin||p.y>sim.height-margin)return false;
    if(sim.avoidZones.some(z=>sim.zoneClearance(z,p.x,p.y,sim.boat.length/2)<=0))return false;
    const x=sim.islandX,y=sim.islandY,w=sim.islandW,h=sim.islandH;
    const onIsland=p.x>=x-margin&&p.x<=x+w+margin&&p.y>=y-margin&&p.y<=y+h+margin;
    return !onIsland||Math.abs(p.y-sim.gateY)<sim.gateWidth/2-margin;
  };
  const result=[path[0]];
  const required=sim.waypoints.slice(sim.currentWaypointIndex);
  for(let i=1;i<path.length-1;i++) {
    const a=path[i-1],b=path[i],c=path[i+1];
    const lenA=Math.hypot(b.x-a.x,b.y-a.y),lenB=Math.hypot(c.x-b.x,c.y-b.y);
    if(lenA<1e-6||lenB<1e-6)continue;
    const ux=(b.x-a.x)/lenA,uy=(b.y-a.y)/lenA;
    const vx=(c.x-b.x)/lenB,vy=(c.y-b.y)/lenB;
    const theta=Math.acos(Math.max(-1,Math.min(1,ux*vx+uy*vy)));
    if(theta<0.025){result.push(b);continue;}
    const cross=ux*vy-uy*vx;
    if(Math.abs(cross)<1e-5)return {valid:false,path,reason:'Разворот на месте недоступен'};
    const tangent=radius*Math.tan(theta/2);
    // No overlapping arcs; avoid cutting required targets outside arrival radius.
    const limit=Math.min(lenA,lenB)*0.46;
    const isWaypoint=required.some(p=>Math.hypot(p.x-b.x,p.y-b.y)<1);
    if(tangent>limit || (isWaypoint && tangent>sim.waypointRadius))
      return {valid:false,path,reason:'Не хватает места для дуги допустимого радиуса'};
    const enter={x:b.x-ux*tangent,y:b.y-uy*tangent};
    const exit={x:b.x+vx*tangent,y:b.y+vy*tangent};
    const direction=Math.sign(cross);
    const center={x:enter.x-direction*uy*radius,y:enter.y+direction*ux*radius};
    const startAngle=Math.atan2(enter.y-center.y,enter.x-center.x);
    const arc=[];
    const segments=Math.max(8,Math.ceil(radius*theta/3));
    for(let j=0;j<=segments;j++){
      const angle=startAngle+direction*theta*j/segments;
      arc.push({x:center.x+radius*Math.cos(angle),y:center.y+radius*Math.sin(angle)});
    }
    // Check every small interval of the new arc and the tangent approach.
    const samples=(p,q)=> {
      const steps=Math.max(1,Math.ceil(Math.hypot(p.x-q.x,p.y-q.y)/3));
      for(let k=0;k<=steps;k++) {
        const t=k/steps;
        if(!safe({x:p.x+(q.x-p.x)*t,y:p.y+(q.y-p.y)*t}))return false;
      }
      return true;
    };
    if(!samples(result.at(-1),enter)||!arc.every(safe)||!samples(arc.at(-1),exit))
      return {valid:false,path,reason:'Дуга пересекает препятствие или границу'};
    result.push(...arc);
  }
  result.push(path.at(-1));
  const validation=validateTurnRadius(result,sim.boat,speed);
  if(!validation.valid) return {valid:false,path,reason:validation.reason};
  return {valid:true,path:result,reason:null};
}

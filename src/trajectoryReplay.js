// Independent virtual playback of a planned path. This checks whether a simple
// bounded steering controller can follow it from the current pose at fixed speed.
// Failure means "not verified by this follower", not mathematical impossibility.
export function replayTrajectory(sim, path, options = {}) {
  const speed=options.speed ?? 30, dt=0.05;
  if(!Array.isArray(path)||path.length<2) return {valid:false,reason:'Маршрут отсутствует'};
  const hull=sim.boat.length/2+sim.safetyMargin;
  const safe=(x,y)=>{
    if(x<hull||x>sim.width-hull||y<hull||y>sim.height-hull)return false;
    if(sim.avoidZones.some(z=>sim.zoneClearance(z,x,y,sim.boat.length/2)<=0))return false;
    const ix=sim.islandX,iy=sim.islandY,iw=sim.islandW,ih=sim.islandH;
    const island=x>=ix-hull&&x<=ix+iw+hull&&y>=iy-hull&&y<=iy+ih+hull;
    return !island||Math.abs(y-sim.gateY)<sim.gateWidth/2-hull;
  };
  let x=sim.boat.x,y=sim.boat.y,h=sim.boat.heading,wp=sim.currentWaypointIndex;
  let routeIndex=0,stagnation=0,prevProgress=0;
  const track=[{x,y}];
  const distanceToSegment=(p,a,b)=>{
    const dx=b.x-a.x,dy=b.y-a.y,len2=dx*dx+dy*dy;
    const t=len2?Math.max(0,Math.min(1,((p.x-a.x)*dx+(p.y-a.y)*dy)/len2)):0;
    return Math.hypot(p.x-a.x-t*dx,p.y-a.y-t*dy);
  };
  if(!safe(x,y))return {valid:false,reason:'Старт внутри препятствия',track};
  const deadline=Math.min(options.maxSeconds??sim.maxTime,sim.maxTime-sim.time);
  for(let t=0;t<deadline;t+=dt){
    // Progress monotonically through route; never seek a point behind the boat.
    while(routeIndex<path.length-2 && Math.hypot(path[routeIndex+1].x-x,path[routeIndex+1].y-y)<15)routeIndex++;
    const follow=Math.min(path.length-1,routeIndex+1);
    const goal=path[follow];
    const target=Math.atan2(goal.y-y,goal.x-x);
    const error=Math.atan2(Math.sin(target-h),Math.cos(target-h));
    const rudder=Math.max(-1,Math.min(1,2.2*error/sim.boat.maxTurnRate));
    const old={x,y};
    h=Math.atan2(Math.sin(h+rudder*sim.boat.maxTurnRate*dt),Math.cos(h+rudder*sim.boat.maxTurnRate*dt));
    x+=Math.cos(h)*speed*dt;y+=Math.sin(h)*speed*dt;
    if(!safe(x,y))return {valid:false,reason:'Прогноз пересекает препятствие или границу',track};
    if(wp<sim.waypoints.length && distanceToSegment(sim.waypoints[wp],old,{x,y})<=sim.waypointRadius)wp++;
    // Advance past sampled points already crossed along the movement direction.
    while(routeIndex<path.length-2) {
      const a=path[routeIndex],b=path[routeIndex+1];
      const dx=b.x-a.x,dy=b.y-a.y;
      if((x-b.x)*dx+(y-b.y)*dy>=0)routeIndex++;
      else break;
    }
    if(Math.floor(t/dt)%5===0)track.push({x,y});
    const progress=routeIndex;
    stagnation=progress===prevProgress?stagnation+dt:0;prevProgress=progress;
    if(stagnation>40)return {valid:false,reason:'Лодка не продвигается по маршруту',track};
    if(wp===sim.waypoints.length &&
       Math.hypot(x-sim.entryX,y-sim.gateY)<=12 &&
       Math.abs(Math.atan2(Math.sin(h),Math.cos(h)))<0.3)
      return {valid:true,reason:null,track,time:t+dt};
  }
  return {valid:false,reason:wp<sim.waypoints.length?'Не пройдены все обязательные точки':'Не удалось войти в док с нужным курсом',track};
}

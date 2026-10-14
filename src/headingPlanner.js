// Heading-aware motion-primitive A*. Returns only physically simulated candidate paths.
// It is intentionally separate from geometric reachability (which remains the safety precheck).
export function planHeadingRoute(sim, options = {}) {
  const cell = options.cell ?? 24, bins = 16, speed = 20;
  const stepTime = 0.6, substeps = 6, turnRate = sim.boat.maxTurnRate;
  const maxNodes = options.maxNodes ?? 12000;
  const margin = sim.boat.length / 2 + sim.safetyMargin;
  const targets = [...sim.waypoints.slice(sim.currentWaypointIndex).map(p=>({x:p.x,y:p.y,tolerance:sim.waypointRadius})),
    {x:sim.entryX,y:sim.gateY,tolerance:12}];
  const free = (x,y) => {
    if(x<margin || y<margin || x>sim.width-margin || y>sim.height-margin)return false;
    if(sim.avoidZones.some(z=>sim.zoneClearance(z,x,y,sim.boat.length/2)<=0))return false;
    const ix=sim.islandX, iy=sim.islandY, iw=sim.islandW, ih=sim.islandH;
    const withinIsland=x>=ix-margin&&x<=ix+iw+margin&&y>=iy-margin&&y<=iy+ih+margin;
    const throughGate=Math.abs(y-sim.gateY)<sim.gateWidth/2-margin;
    return !withinIsland || throughGate;
  };
  let start={x:sim.boat.x,y:sim.boat.y,heading:sim.boat.heading};
  const full=[{...start}];
  let explored=0;
  const normalize=a=>Math.atan2(Math.sin(a),Math.cos(a));
  const quantize=n=>Math.round(n/cell);
  const key=n=>quantize(n.x)+','+quantize(n.y)+','+
    ((Math.round((normalize(n.heading)+Math.PI)/(2*Math.PI)*bins))%bins);
  const heuristic=(n,t)=>Math.hypot(n.x-t.x,n.y-t.y);
  for (let segment=0;segment<targets.length;segment++){
    const target=targets[segment];
    if(!free(start.x,start.y)||!free(target.x,target.y))
      return {reachable:false,path:[],reason:'Маршрут перекрыт препятствием',explored};
    const open=[{...start,g:0,f:heuristic(start,target),parent:null,trail:[]}];
    const best=new Map();
    let goal=null, iterations=0;
    while(open.length && iterations++<maxNodes){
      let bestIndex=0;
      for(let i=1;i<open.length;i++)if(open[i].f<open[bestIndex].f)bestIndex=i;
      const node=open.splice(bestIndex,1)[0];
      const k=key(node);
      if(best.has(k)&&best.get(k)<node.g-0.001)continue;
      explored++;
      if(heuristic(node,target)<=Math.max(target.tolerance,cell*0.55)){
        goal=node;break;
      }
      for(const rudder of [-1,0,1]){
        let x=node.x,y=node.y,h=node.heading;
        const trail=[];let valid=true;
        for(let i=0;i<substeps;i++){
          const dt=stepTime/substeps;
          h=normalize(h+rudder*turnRate*dt);
          x+=Math.cos(h)*speed*dt;y+=Math.sin(h)*speed*dt;
          if(!free(x,y)){valid=false;break;}
          trail.push({x,y});
        }
        if(!valid)continue;
        const next={x,y,heading:h,g:node.g+speed*stepTime+Math.abs(rudder)*0.15,
          parent:node,trail};
        const nk=key(next);
        if(best.has(nk)&&best.get(nk)<=next.g)continue;
        best.set(nk,next.g);
        next.f=next.g+heuristic(next,target);
        open.push(next);
      }
    }
    if(!goal) return {reachable:false,path:[],reason:'Не найден путь с учётом ограничений поворота',explored};
    const segments=[];
    let n=goal;
    while(n.parent){segments.push(n.trail);n=n.parent;}
    segments.reverse();
    for(const piece of segments)full.push(...piece);
    start={x:goal.x,y:goal.y,heading:goal.heading};
  }
  return {reachable:true,path:full,reason:null,explored};
}

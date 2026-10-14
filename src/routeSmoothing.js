// Corner rounding for the already collision-checked, waypoint-ordered grid route.
// This is a display/reference path, not a promise that the current neural controller follows it.
export function smoothSafeRoute(sim, path) {
  if (!path || path.length < 2) return [];
  const margin = sim.boat.length / 2 + sim.safetyMargin;
  const safe = p => {
    if (p.x < margin || p.y < margin || p.x > sim.width-margin || p.y > sim.height-margin) return false;
    if (sim.avoidZones.some(z=>sim.zoneClearance(z,p.x,p.y,sim.boat.length/2)<=0)) return false;
    const x=sim.islandX,y=sim.islandY,w=sim.islandW,h=sim.islandH;
    const island=p.x>=x-margin && p.x<=x+w+margin && p.y>=y-margin && p.y<=y+h+margin;
    return !island || Math.abs(p.y-sim.gateY)<sim.gateWidth/2-margin;
  };
  const route=[path[0]];
  const remaining=sim.waypoints.slice(sim.currentWaypointIndex);
  for(let i=1;i<path.length-1;i++){
    const before=path[i-1],corner=path[i],after=path[i+1];
    const incoming=Math.hypot(corner.x-before.x,corner.y-before.y);
    const outgoing=Math.hypot(after.x-corner.x,after.y-corner.y);
    if(incoming<0.1 || outgoing<0.1){route.push(corner);continue;}
    const ax=(corner.x-before.x)/incoming,ay=(corner.y-before.y)/incoming;
    const bx=(after.x-corner.x)/outgoing,by=(after.y-corner.y)/outgoing;
    const dot=ax*bx+ay*by;
    if(dot>0.995 || dot< -0.95){route.push(corner);continue;}
    // Leave mandatory targets inside their accepted 18px capture radius.
    const required=remaining.some(w=>Math.hypot(w.x-corner.x,w.y-corner.y)<1);
    const d=Math.min(22,incoming*0.30,outgoing*0.30,required?sim.waypointRadius:22);
    if(d<3){route.push(corner);continue;}
    const p0={x:corner.x-ax*d,y:corner.y-ay*d};
    const p2={x:corner.x+bx*d,y:corner.y+by*d};
    const curve=[];
    const segments=Math.max(8,Math.ceil(d/2));
    for(let j=0;j<=segments;j++){
      const t=j/segments,q=1-t;
      curve.push({x:q*q*p0.x+2*q*t*corner.x+t*t*p2.x,
        y:q*q*p0.y+2*q*t*corner.y+t*t*p2.y});
    }
    // Reject unsafe shortcut; do not cut through a red zone or the island.
    if(curve.every(safe)) route.push(...curve);
    else route.push(corner);
  }
  route.push(path.at(-1));
  return route;
}

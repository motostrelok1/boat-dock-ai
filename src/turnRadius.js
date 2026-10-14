// Validate turn curvature for a sampled path against boat steering physics.
// A pass checks local turn radius only: it does NOT prove a feasible approach heading,
// actuator dynamics, waypoint capture or global route reachability.
export function minimumTurnRadius(boat, speed = boat.speed) {
  if (!(speed > 0) || !(boat.maxTurnRate > 0)) return Infinity;
  return speed / boat.maxTurnRate;
}

export function validateTurnRadius(path, boat, speed = boat.speed) {
  const minimumRadius = minimumTurnRadius(boat, speed);
  if (!Array.isArray(path) || path.length < 2)
    return {valid:false, minimumRadius, reason:'Маршрут отсутствует'};
  if (!Number.isFinite(minimumRadius))
    return {valid:false, minimumRadius, reason:'Недопустимая скорость или скорость поворота'};
  // Evaluate the curvature as the circumradius of each consecutive triangle.
  // Ignore almost-collinear samples; handle hard reversals as zero-radius turns.
  for (let i=1;i<path.length-1;i++) {
    const a=path[i-1], b=path[i], c=path[i+1];
    const ab=Math.hypot(b.x-a.x,b.y-a.y);
    const bc=Math.hypot(c.x-b.x,c.y-b.y);
    const ac=Math.hypot(c.x-a.x,c.y-a.y);
    if(ab<1e-6 || bc<1e-6)continue;
    const cross=(b.x-a.x)*(c.y-b.y)-(b.y-a.y)*(c.x-b.x);
    const dot=(b.x-a.x)*(c.x-b.x)+(b.y-a.y)*(c.y-b.y);
    if (Math.abs(cross)<1e-7 && dot>=0)continue;
    const twiceArea=Math.abs(cross);
    const radius=twiceArea<1e-7?0:ab*bc*ac/(2*twiceArea);
    if (radius+1e-6 < minimumRadius)
      return {valid:false,minimumRadius,reason:'Слишком резкий поворот',at:i,radius};
  }
  return {valid:true,minimumRadius,reason:null};
}

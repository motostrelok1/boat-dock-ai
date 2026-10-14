import test from 'node:test';
import assert from 'node:assert/strict';
import { Simulation } from '../src/simulation.js';
import { replayTrajectory } from '../src/trajectoryReplay.js';
import { buildTurnArcRoute } from '../src/arcPlanner.js';
import { minimumTurnRadius, validateTurnRadius } from '../src/turnRadius.js';
import { smoothSafeRoute } from '../src/routeSmoothing.js';
import { planHeadingRoute } from '../src/headingPlanner.js';
import { NeuralNetwork } from '../src/neuralNetwork.js';

globalThis.localStorage = {
  data: new Map(),
  getItem(k) { return this.data.get(k) ?? null; },
  setItem(k,v) { this.data.set(k,v); },
  removeItem(k) { this.data.delete(k); }
};
function fresh() { localStorage.data.clear(); return new Simulation(1800,1000); }

test('manual steering changes heading',()=>{
  const s=fresh();s.reset(null,false);s.aiEnabled=false;
  s.boat.x=100;s.boat.y=100;s.boat.heading=0;s.boat.rudderAngle=1;
  s.update(0.1);assert.ok(s.boat.heading>0);
});
test('saved brain survives initialization',()=>{
  const s=fresh();s.bestNetwork=s.nn;s.bestScore=123;s.saveBestBrain();
  const reload=new Simulation(1800,1000);
  assert.equal(reload.bestScore,123);
});
test('spawn is left of entry',()=>{
  const s=fresh();
  for(let i=0;i<100;i++){s.randomizeLayout();assert.ok(s.boat.x<s.entryX);}
});
test('island bounds and channel',()=>{
  const s=fresh();s.islandX=1500;s.islandY=300;s.islandW=180;s.islandH=200;
  s.gateY=400;s.gateWidth=42;s.boat.x=1550;
  s.boat.y=200;assert.equal(s.checkIslandCollision(),false);
  s.boat.y=310;assert.equal(s.checkIslandCollision(),true);
  s.boat.y=400;assert.equal(s.checkIslandCollision(),false);
});
test('segment crossing passes waypoint',()=>{
  const s=fresh();s.addWaypoint(101,100);s.boat.x=105;s.boat.y=100;
  s.checkWaypoints({x:95,y:100});assert.equal(s.currentWaypointIndex,1);
});
test('three elite networks survive unchanged',()=>{
  const s=fresh();s.populationSize=10;
  s.population=Array.from({length:10},(_,i)=>({network:new NeuralNetwork(6,8,1),score:100-i}));
  const elite=s.population.slice(0,3).map(p=>p.network);
  s.nextGeneration();assert.deepEqual(s.population.slice(0,3).map(p=>p.network),elite);
});
test('benchmark uses reproducible scenarios',()=>{
  const s=fresh(),a=s.benchmark(s.nn,[1001,1002]),b=s.benchmark(s.nn,[1001,1002]);
  assert.deepEqual(a.summary,b.summary);
  assert.deepEqual(a.runs.map(r=>r.score),b.runs.map(r=>r.score));
});


test('random avoid zones generate requested count and stay clear of spawn',()=>{
  const s=fresh();
  s.randomAvoidEnabled=true;
  s.randomAvoidCount=5;
  s.randomAvoidRadius=20;
  s.reset(null,true);
  assert.equal(s.avoidZones.length,5);
  for (const z of s.avoidZones) {
    assert.equal(z.radius,20);
    assert.ok(z.x < s.entryX);
    assert.ok(Math.hypot(z.x-s.boat.x,z.y-s.boat.y)>=z.radius+60);
  }
});
test('random avoid zones are reproducible for deterministic layout',()=>{
  const s=fresh();
  s.randomAvoidEnabled=true;
  s.randomAvoidCount=3;
  s.randomAvoidRadius=30;
  const rng=()=>0.5;
  s.randomizeLayout(rng);
  const first=JSON.stringify(s.avoidZones);
  s.randomizeLayout(rng);
  assert.equal(JSON.stringify(s.avoidZones),first);
});


test('random waypoints and random avoid zones never touch',()=>{
  const s=fresh();
  s.randomWaypointEnabled=true;s.randomWaypointCount=5;
  s.randomAvoidEnabled=true;s.randomAvoidCount=10;s.randomAvoidRadius=30;
  for(let i=0;i<8;i++){
    s.reset(null,true);
    assert.equal(s.getLayoutConflict(),null);
    for(const wp of s.waypoints) for(const z of s.avoidZones)
      assert.ok(s.zoneClearance(z,wp.x,wp.y,s.boat.length/2)>0);
  }
});
test('overlapping manual zone triggers safe 10-second movement and removal',()=>{
  const s=fresh();
  s.clearWaypoints();s.clearAvoidZones();
  s.boat.x=100;s.boat.y=100;s.boat.heading=0;
  s.addWaypoint(800,450);
  s.addAvoidZone(800,450,20);
  assert.ok(s.holding);
  assert.equal(s.boat.speed,10);
  for(let i=0;i<200;i++)s.update(0.05);
  assert.equal(s.holding,null);
  assert.equal(s.avoidZones.length,0);
  assert.equal(s.boat.speed,30);
  assert.equal(s.finished,false);
});
test('no safe holding loop removes zones without sending boat through danger',()=>{
  const s=fresh();
  s.clearWaypoints();s.clearAvoidZones();
  s.boat.x=25;s.boat.y=25;
  s.addWaypoint(400,400);
  s.addAvoidZone(400,400,20);
  assert.equal(s.holding,null);
  assert.equal(s.avoidZones.length,0);
  assert.match(s.notice,/нет места для ожидания/);
});

test('waypoint is accepted within wider 18 px arrival radius',()=>{
  const s=fresh();
  assert.equal(s.waypointRadius,18);
  s.addWaypoint(200,200);
  s.boat.x=217;s.boat.y=200;
  s.checkWaypoints();
  assert.equal(s.currentWaypointIndex,1);
});
test('crossing edge of the enlarged waypoint circle counts',()=>{
  const s=fresh();s.addWaypoint(200,200);
  s.boat.x=230;s.boat.y=217;
  s.checkWaypoints({x:170,y:217});
  assert.equal(s.currentWaypointIndex,1);
});

test('route connectivity works for a clear approach',()=>{
 const s=fresh();s.clearWaypoints();s.clearAvoidZones();
 s.boat.x=100;s.boat.y=500;
 s.addWaypoint(500,500);
 assert.equal(s.getRouteReachability().reachable,true);
});
test('route connectivity detects a fully blocked barrier',()=>{
 const s=fresh();s.clearWaypoints();s.clearAvoidZones();
 s.boat.x=100;s.boat.y=500;
 s.addWaypoint(900,500);
 s.avoidZones=[];
 for(let y=0;y<=1000;y+=65) s.avoidZones.push({x:600,y,radius:55});
 const result=s.getRouteReachability();
 assert.equal(result.reachable,false);
 assert.match(result.reason,/Нет прохода/);
});
test('route precheck notices a waypoint inside a hazard',()=>{
 const s=fresh();s.clearWaypoints();s.clearAvoidZones();
 s.boat.x=100;s.boat.y=500;
 s.waypoints=[{x:500,y:500,passed:false}];
 s.avoidZones=[{x:500,y:500,radius:30}];
 assert.equal(s.getRouteReachability().reachable,false);
});

test('reachable route exposes ordered positions from boat to waypoints and dock',()=>{
 const s=fresh();s.clearWaypoints();s.clearAvoidZones();s.boat.x=100;s.boat.y=500;
 s.addWaypoint(500,500);const r=s.getRouteReachability();
 assert.equal(r.reachable,true);assert.ok(r.path.length>2);
 assert.deepEqual(r.path[0],{x:100,y:500});
 assert.deepEqual(r.path.at(-1),{x:s.entryX,y:s.gateY});
 assert.ok(r.path.some(p=>p.x===500&&p.y===500));
});
test('blocked route returns empty visualization path',()=>{
 const s=fresh();s.clearWaypoints();s.clearAvoidZones();s.boat.x=100;s.boat.y=500;
 s.avoidZones=[{x:100,y:500,radius:30}];
 const r=s.getRouteReachability();assert.equal(r.reachable,false);assert.deepEqual(r.path,[]);
});

test('heading planner respects turn rate and avoids hazards',()=>{
 const s=fresh();s.clearWaypoints();s.clearAvoidZones();s.boat.x=150;s.boat.y=500;s.boat.heading=0;
 s.entryX=390;s.gateY=500;s.islandX=450;s.islandY=50;s.islandH=300;s.islandW=150;
 const result=planHeadingRoute(s,{maxNodes:4000});assert.equal(result.reachable,true);
 assert.ok(result.path.length>=2);
 assert.deepEqual(result.path[0],{x:150,y:500,heading:0});
 assert.ok(Math.hypot(result.path.at(-1).x-390,result.path.at(-1).y-500)<15);
 for(const p of result.path)assert.ok(Math.hypot(p.x-150,p.y-500)>=0);
});
test('heading planner rejects obstructed starting point',()=>{
 const s=fresh();s.clearWaypoints();s.clearAvoidZones();s.boat.x=100;s.boat.y=500;
 s.avoidZones=[{x:100,y:500,radius:35}];
 assert.equal(planHeadingRoute(s).reachable,false);
});

test('geometric planner omits already passed waypoints',()=>{
 const s=fresh();s.clearWaypoints();s.clearAvoidZones();
 s.boat.x=250;s.boat.y=500;
 s.waypoints=[{x:100,y:500,passed:true},{x:400,y:500,passed:false}];
 s.currentWaypointIndex=1;
 const r=s.getRouteReachability();
 assert.equal(r.reachable,true);
 assert.ok(r.path.some(p=>p.x===400&&p.y===500));
 assert.ok(!r.path.some(p=>p.x===100&&p.y===500));
});
test('heading planner ignores already passed waypoint',()=>{
 const s=fresh();s.clearWaypoints();s.clearAvoidZones();
 s.boat.x=150;s.boat.y=500;s.boat.heading=0;
 s.entryX=390;s.gateY=500;s.islandX=450;s.islandY=50;s.islandH=300;s.islandW=150;
 s.waypoints=[{x:100,y:500,passed:true}];s.currentWaypointIndex=1;
 const r=planHeadingRoute(s,{maxNodes:4000});
 assert.equal(r.reachable,true);
 assert.ok(r.path.every(p=>p.x>100));
});

test('heading planner produces a route without excessive zigzag in open water',()=>{
 const s=fresh();s.clearWaypoints();s.clearAvoidZones();
 s.boat.x=150;s.boat.y=500;s.boat.heading=0;
 s.entryX=420;s.gateY=500;s.islandX=480;s.islandY=100;s.islandH=200;s.islandW=120;
 const r=planHeadingRoute(s,{maxNodes:4000});
 assert.equal(r.reachable,true);
 let deviation=0;
 for(const p of r.path) deviation=Math.max(deviation,Math.abs(p.y-500));
 assert.ok(deviation<35,`unnecessary steering: ${deviation}`);
});

test('geometric route takes a direct shortcut in clear water',()=>{
 const s=fresh();s.clearWaypoints();s.clearAvoidZones();s.boat.x=150;s.boat.y=500;
 s.entryX=420;s.gateY=500;s.islandX=480;s.islandY=100;s.islandH=200;s.islandW=120;
 const r=s.getRouteReachability();assert.equal(r.reachable,true);
 assert.ok(r.path.length<=3);
});
test('geometric shortcut does not cross an avoid zone',()=>{
 const s=fresh();s.clearWaypoints();s.clearAvoidZones();s.boat.x=150;s.boat.y=500;
 s.entryX=900;s.gateY=500;s.islandX=1100;s.islandY=100;s.islandH=200;s.islandW=120;
 s.avoidZones=[{x:500,y:500,radius:70}];
 const r=s.getRouteReachability();assert.equal(r.reachable,true);
 for(let i=1;i<r.path.length;i++){
  const a=r.path[i-1],b=r.path[i],steps=Math.ceil(Math.hypot(b.x-a.x,b.y-a.y)/3);
  for(let j=0;j<=steps;j++){
   const x=a.x+(b.x-a.x)*j/steps,y=a.y+(b.y-a.y)*j/steps;
   assert.ok(s.zoneClearance(s.avoidZones[0],x,y,s.boat.length/2)>0);
  }
 }
});

test('smoothing keeps mandatory route endpoints and rounds a clear bend',()=>{
 const s=fresh();s.clearWaypoints();s.clearAvoidZones();
 s.islandX=1600;s.islandY=100;s.islandW=100;s.islandH=200;
 const original=[{x:100,y:200},{x:250,y:200},{x:250,y:350}];
 const result=smoothSafeRoute(s,original);
 assert.deepEqual(result[0],original[0]);
 assert.deepEqual(result.at(-1),original.at(-1));
 assert.ok(result.length>3);
});
test('smoothing preserves a corner if rounding would cross a hazard',()=>{
 const s=fresh();s.clearWaypoints();s.clearAvoidZones();
 s.avoidZones=[{x:244,y:206,radius:4}];
 const original=[{x:100,y:200},{x:250,y:200},{x:250,y:350}];
 const result=smoothSafeRoute(s,original);
 assert.ok(result.some(p=>p.x===250&&p.y===200));
});

test('minimum turning radius obeys speed and max turn rate',()=>{
 const s=fresh();assert.ok(Math.abs(minimumTurnRadius(s.boat,30)-25)<0.001);
 assert.ok(Math.abs(minimumTurnRadius(s.boat,10)-10/1.2)<0.001);
});
test('turn validation accepts straight path and rejects sharp corner',()=>{
 const boat=fresh().boat;
 assert.equal(validateTurnRadius([{x:0,y:0},{x:100,y:0},{x:200,y:0}],boat,30).valid,true);
 const tight=[{x:0,y:0},{x:4,y:0},{x:4,y:4}];
 assert.equal(validateTurnRadius(tight,boat,30).valid,false);
});
test('turn validation accepts sampled gentle circular arc',()=>{
 const boat=fresh().boat;
 const arc=Array.from({length:15},(_,i)=>({x:80*Math.cos(i*0.08),y:80*Math.sin(i*0.08)}));
 assert.equal(validateTurnRadius(arc,boat,30).valid,true);
});

test('30px/s right-angle bend produces permissible circular arc',()=>{
 const s=fresh();s.clearWaypoints();s.clearAvoidZones();s.islandX=1600;s.islandY=100;s.islandW=100;s.islandH=200;
 const r=buildTurnArcRoute(s,[{x:100,y:200},{x:250,y:200},{x:250,y:350}],30);
 assert.equal(r.valid,true,r.reason);
 assert.ok(r.path.length>8);
});
test('not enough tangent length is reported rather than faking a valid turn',()=>{
 const s=fresh();s.clearWaypoints();s.clearAvoidZones();
 const r=buildTurnArcRoute(s,[{x:100,y:200},{x:104,y:200},{x:104,y:240}],30);
 assert.equal(r.valid,false);
 assert.match(r.reason,/Не хватает места/);
});

test('virtual route replay reaches a straight unobstructed dock approach',()=>{
 const s=fresh();s.clearWaypoints();s.clearAvoidZones();s.boat.x=150;s.boat.y=500;s.boat.heading=0;
 s.entryX=390;s.gateY=500;s.islandX=450;s.islandY=100;s.islandH=200;s.islandW=120;
 const r=replayTrajectory(s,[{x:150,y:500},{x:390,y:500}],{maxSeconds:20});
 assert.equal(r.valid,true,r.reason);
});
test('virtual route replay rejects a boat starting in a hazard',()=>{
 const s=fresh();s.boat.x=150;s.boat.y=500;s.avoidZones=[{x:150,y:500,radius:40}];
 const r=replayTrajectory(s,[{x:150,y:500},{x:390,y:500}]);
 assert.equal(r.valid,false);
 assert.match(r.reason,/Старт/);
});

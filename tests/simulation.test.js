import test from 'node:test';
import assert from 'node:assert/strict';
import { Simulation } from '../src/simulation.js';
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

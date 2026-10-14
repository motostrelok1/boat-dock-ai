import { buildTurnArcRoute } from './arcPlanner.js';
import { validateTurnRadius, minimumTurnRadius } from './turnRadius.js';
import { smoothSafeRoute } from './routeSmoothing.js';
import { planHeadingRoute } from './headingPlanner.js';
import { Renderer } from './renderer.js';
import { Simulation } from './simulation.js';

const canvas = document.getElementById('simCanvas');
const renderer = new Renderer(canvas, 1800, 1000);
const simulation = new Simulation(1800, 1000);

const hudSpeed = document.getElementById('hud-speed');
const hudHeading = document.getElementById('hud-heading');
const hudRudder = document.getElementById('hud-rudder');
const hudScore = document.getElementById('hud-score');
const hudInputDist = document.getElementById('hud-input-dist');
const hudInputAngle = document.getElementById('hud-input-angle');
const hudInputSin = document.getElementById('hud-input-sin');
const hudInputCos = document.getElementById('hud-input-cos');
const hudInputAvoidDist = document.getElementById('hud-input-avoid-dist');
const hudInputAvoidAngle = document.getElementById('hud-input-avoid-angle');
const hudOutputRudder = document.getElementById('hud-output-rudder');
const hudLifetime = document.getElementById('hud-lifetime');
const hudGeneration = document.getElementById('hud-generation');
const hudBestScore = document.getElementById('hud-best-score');
const hudSaveStatus = document.getElementById('hud-save-status');

const progressContainer = document.getElementById('progress-container');
const progressFill = document.getElementById('progress-fill');
const progressText = document.getElementById('progress-text');

const btnReset = document.getElementById('btn-reset');
const btnToggleAI = document.getElementById('btn-toggle-ai');
const btnTrain = document.getElementById('btn-train');
const btnMutate = document.getElementById('btn-mutate');
const btnSave = document.getElementById('btn-save');
const btnLoad = document.getElementById('btn-load');
const btnExport = document.getElementById('btn-export');
const btnImport = document.getElementById('btn-import');
const simSpeedSlider = document.getElementById('sim-speed');
const speedVal = document.getElementById('speed-val');

const btnWaypointMode = document.getElementById('btn-waypoint-mode');
const waypointCount = document.getElementById('waypoint-count');
const waypointIndicator = document.getElementById('waypoint-indicator');
const btnClearWaypoints = document.getElementById('btn-clear-waypoints');

const btnAvoidMode = document.getElementById('btn-avoid-mode');
const avoidRadius = document.getElementById('avoid-radius');
const avoidCount = document.getElementById('avoid-count');
const avoidIndicator = document.getElementById('avoid-indicator');
const btnClearAvoid = document.getElementById('btn-clear-avoid');
const chkRandomWaypoints = document.getElementById('chk-random-waypoints');
const chkRandomAvoid = document.getElementById('chk-random-avoid');

let simSpeed = 1;
let lastTime = 0;
let isTraining = false;
let autoRestartTimer = null;
let generationCounter = 0;

let clickMode = 'none';

function setClickMode(mode) {
  clickMode = mode;
  btnWaypointMode.classList.toggle('active', mode === 'waypoint');
  btnAvoidMode.classList.toggle('active', mode === 'avoid');
  waypointIndicator.textContent = mode === 'waypoint' ? 'Режим: установка точек' : 'Режим: выкл';
  waypointIndicator.style.color = mode === 'waypoint' ? '#00e676' : '#80deea';
  avoidIndicator.textContent = mode === 'avoid' ? 'Режим: установка запретов' : 'Режим: выкл';
  avoidIndicator.style.color = mode === 'avoid' ? '#ff5252' : '#80deea';
  canvas.style.cursor = mode === 'none' ? 'crosshair' : 'pointer';
}

btnWaypointMode.addEventListener('click', () => {
  setClickMode(clickMode === 'waypoint' ? 'none' : 'waypoint');
});

btnAvoidMode.addEventListener('click', () => {
  setClickMode(clickMode === 'avoid' ? 'none' : 'avoid');
});

btnClearWaypoints.addEventListener('click', () => {
  chkRandomWaypoints.checked=false;
  simulation.randomWaypointEnabled=false;
  simulation.clearWaypoints();
});
btnClearAvoid.addEventListener('click', () => {
  chkRandomAvoid.checked = false;
  simulation.randomAvoidEnabled = false;
  simulation.clearAvoidZones();
});
function syncRandomAvoid() {
  simulation.randomAvoidCount = Number(avoidCount.value);
  simulation.randomAvoidRadius = Number(avoidRadius.value);
  if (simulation.randomAvoidEnabled) {
    simulation.generateRandomAvoidZones();
    simulation.checkLayoutSafety();
  }
}
chkRandomAvoid.addEventListener('change', () => {
  simulation.randomAvoidEnabled = chkRandomAvoid.checked;
  if (simulation.randomAvoidEnabled) syncRandomAvoid();
  else simulation.clearAvoidZones();
});
avoidCount.addEventListener('change', syncRandomAvoid);
avoidRadius.addEventListener('change', syncRandomAvoid);

function generateRandomWaypoints() {
  simulation.randomWaypointEnabled = chkRandomWaypoints.checked;
  simulation.randomWaypointCount = Number(waypointCount.value);
  if (chkRandomWaypoints.checked) simulation.generateRandomWaypoints();
}
waypointCount.addEventListener('change', () => {
  if(chkRandomWaypoints.checked) generateRandomWaypoints();
});

chkRandomWaypoints.addEventListener('change', (e) => {
  simulation.randomWaypointEnabled = e.target.checked;
  if (e.target.checked) generateRandomWaypoints();
  else simulation.clearWaypoints();
});

canvas.addEventListener('click', (e) => {
  if (clickMode === 'none') return;
  const rect = canvas.getBoundingClientRect();
  const scaleX = canvas.width / rect.width;
  const scaleY = canvas.height / rect.height;
  const x = (e.clientX - rect.left) * scaleX;
  const y = (e.clientY - rect.top) * scaleY;

  if (clickMode === 'waypoint') {
    if (simulation.removeWaypointAt(x, y)) return;
    const maxCount = parseInt(waypointCount.value, 10);
    if (simulation.waypoints.length < maxCount) simulation.addWaypoint(x, y);
  } else if (clickMode === 'avoid') {
    if (chkRandomAvoid.checked) {
      chkRandomAvoid.checked = false;
      simulation.randomAvoidEnabled = false;
    }
    if (simulation.removeAvoidZoneAt(x, y)) return;
    const maxCount = parseInt(avoidCount.value, 10);
    if (simulation.avoidZones.length < maxCount) {
      simulation.addAvoidZone(x, y, parseInt(avoidRadius.value, 10));
    }
  }
});

function updateHUD() {
  const state = simulation.boat.getState();
  hudSpeed.textContent = state.speed.toFixed(1);
  hudHeading.textContent = (state.heading * 180 / Math.PI).toFixed(1);
  hudRudder.textContent = (state.rudderAngle * 45).toFixed(1);
  hudScore.textContent = simulation.score.toFixed(1);

  const inputs = simulation.getInputs();
  const output = simulation.aiEnabled ? simulation.nn.forward(inputs) : [simulation.boat.rudderAngle];

  hudInputDist.textContent = inputs[0].toFixed(2);
  hudInputAngle.textContent = inputs[1].toFixed(2);
  hudInputSin.textContent = inputs[2].toFixed(2);
  hudInputCos.textContent = inputs[3].toFixed(2);
  hudInputAvoidDist.textContent = inputs[4].toFixed(2);
  hudInputAvoidAngle.textContent = inputs[5].toFixed(2);
  hudOutputRudder.textContent = output[0].toFixed(2);

  hudGeneration.textContent = simulation.generation;
  hudBestScore.textContent = Number.isFinite(simulation.bestScore) ? simulation.bestScore.toFixed(0) : 'Нет данных';

  const saved = localStorage.getItem('boat-dock-ai-best');
  hudSaveStatus.textContent = saved ? '✓ Есть' : '✗ Нет';
  hudSaveStatus.style.color = saved ? '#00e676' : '#ff5252';

  hudLifetime.textContent = Math.max(0, simulation.maxTime - simulation.time).toFixed(1);
}

let lastRouteKey = '';
let plannedRoute = [];
let routeStatus = '';
let lastPlanTime = -Infinity;
function routeDistance(path, position) {
  if (!path.length) return Infinity;
  let min = Infinity;
  for (let i=1;i<path.length;i++) {
    const a=path[i-1],b=path[i],dx=b.x-a.x,dy=b.y-a.y;
    const t=Math.max(0,Math.min(1,((position.x-a.x)*dx+(position.y-a.y)*dy)/(dx*dx+dy*dy||1)));
    min=Math.min(min,Math.hypot(position.x-a.x-t*dx,position.y-a.y-t*dy));
  }
  return min;
}
function getPlannedRoute() {
  // Replan on map/target changes, not on every animation frame.
  const key = JSON.stringify({
    waypointIndex: simulation.currentWaypointIndex,
    targets: simulation.waypoints.slice(simulation.currentWaypointIndex).map(p=>[p.x,p.y]),
    zones: simulation.avoidZones.map(z=>[z.x,z.y,z.radius]),
    dock: [simulation.entryX,simulation.gateY,simulation.islandX,simulation.islandY,simulation.islandW,simulation.islandH],
    reset: simulation.boat.trail.length === 0 ? [simulation.boat.x,simulation.boat.y] : null
  });
  const changed = key !== lastRouteKey;
  const offRoute = plannedRoute.length > 1 && routeDistance(plannedRoute,simulation.boat) > 65;
  if (changed || (offRoute && simulation.time - lastPlanTime >= 5)) {
    lastRouteKey=key;
    lastPlanTime=simulation.time;
    const connectivity=simulation.getRouteReachability();
    if (!connectivity.reachable) {
      plannedRoute = [];
      routeStatus = 'Маршрут заблокирован';
    } else {
      const result = planHeadingRoute(simulation, {maxNodes: 6000});
      // The heading-aware search is bounded and can fail on multi-point routes.
      // Fall back only to the verified collision-free geometric route, never to a straight chord.
      if (result.reachable) {
        plannedRoute = result.path;
        const validation = validateTurnRadius(result.path, simulation.boat, 20);
        routeStatus = validation.valid ? 'Маршрут проверен на радиус поворота при 20 px/с' : 'Маршрут: требуется проверка поворотов';
      } else {
        const arc = buildTurnArcRoute(simulation, connectivity.path, 30);
        plannedRoute = arc.valid ? arc.path : smoothSafeRoute(simulation, connectivity.path);
        routeStatus = arc.valid ? 'Дуги проверены при 30 px/с; начальный курс ещё не проверен' : `Предварительный маршрут — ${arc.reason}`;
      }
    }
  }
  // Hide only the already travelled prefix; keep the planned geometry stable.
  if(plannedRoute.length<2)return [];
  let nearest=0, distance=Infinity;
  for(let i=0;i<plannedRoute.length;i++) {
    const d=Math.hypot(plannedRoute[i].x-simulation.boat.x,plannedRoute[i].y-simulation.boat.y);
    if(d<distance){distance=d;nearest=i;}
  }
  return plannedRoute.slice(Math.max(0,nearest-1));
}

function render() {
  renderer.clear();
  renderer.drawGrid();
  renderer.drawStartLine();
  renderer.drawIsland(
    simulation.islandX, simulation.islandY, simulation.islandW, simulation.islandH,
    simulation.gateY, simulation.gateWidth, simulation.entryX
  );
  renderer.drawWaypoints(simulation.waypoints, simulation.currentWaypointIndex);
  renderer.drawAvoidZones(simulation.avoidZones);
  renderer.drawPlannedRoute(getPlannedRoute());
  if (routeStatus) {
    renderer.ctx.save();
    renderer.ctx.font = 'bold 13px sans-serif';
    renderer.ctx.fillStyle = routeStatus.startsWith('Предварительный') ? '#ffdc39' : '#80deea';
    renderer.ctx.fillText(routeStatus, 20, 190);
    renderer.ctx.restore();
  }
  renderer.drawTrail(simulation.boat.trail);
  renderer.drawBoat(simulation.boat);
  renderer.drawPrediction(simulation.boat, simulation.boat.getPrediction(3));

  const ctx = renderer.ctx;
  ctx.font = 'bold 14px sans-serif';
  ctx.textAlign = 'left';

  if (simulation.finished) {
    if (simulation.checkDockEntry()) {
      ctx.fillStyle = '#00e676';
      ctx.fillText('✓ ФАРВАТЕР ПРОЙДЕН!', 20, 30);
    } else if (simulation.entryChecked && !simulation.entrySuccess) {
      ctx.fillStyle = '#ff5252';
      ctx.fillText('✗ НЕЗАЧЁТ — НЕВЕРНЫЙ ВХОД', 20, 30);
    } else {
      ctx.fillStyle = '#ff5252';
      ctx.fillText('✗ ПОПЫТКА ЗАВЕРШЕНА', 20, 30);
    }
  } else if (simulation.entrySuccess) {
    ctx.fillStyle = '#00e676';
    ctx.fillText('✓ ТОЧКА ВХОДА — ИДЁМ ПО ФАРВАТЕРУ', 20, 30);
  } else {
    ctx.fillStyle = '#00e5ff';
    ctx.fillText(isTraining ? '🧠 ОБУЧЕНИЕ...' : '▶ СИМУЛЯЦИЯ', 20, 30);
  }

  ctx.fillStyle = '#80deea';
  ctx.fillText(`Время: ${simulation.time.toFixed(1)} / ${simulation.maxTime}с`, 20, 50);
  ctx.fillStyle = simulation.aiEnabled ? '#00e676' : '#ff5252';
  ctx.fillText(simulation.aiEnabled ? '🤖 ИИ АКТИВЕН' : '👤 РУЧНОЙ', 20, 70);

  if (simulation.waypoints.length > 0) {
    const passed = simulation.waypoints.filter(wp => wp.passed).length;
    ctx.fillStyle = '#00e676';
    ctx.fillText(`📍 Точки: ${passed}/${simulation.waypoints.length}`, 20, 90);
  }
  if (simulation.avoidZones.length > 0) {
    ctx.fillStyle = '#ff5252';
    ctx.fillText(`🚫 Запретов: ${simulation.avoidZones.length}`, 20, 110);
  }

  if (simulation.holding || simulation.notice) {
    ctx.fillStyle = simulation.holding ? '#ffbf47' : '#e0f7fa';
    ctx.font = 'bold 16px sans-serif';
    const message = simulation.holding
      ? `⚠️ Безопасное ожидание: ${Math.ceil(simulation.holding.remaining)} с · 10 px/с`
      : simulation.notice;
    ctx.fillText(message, 20, 160);
  }

  const timeRatio = simulation.time / simulation.maxTime;
  ctx.fillStyle = 'rgba(255,255,255,0.1)';
  ctx.fillRect(20, 125, 200, 6);
  ctx.fillStyle = timeRatio > 0.8 ? '#ff5252' : (timeRatio > 0.5 ? '#ffab40' : '#00e676');
  ctx.fillRect(20, 125, 200 * (1 - timeRatio), 6);
}

function gameLoop(timestamp) {
  if (!lastTime) lastTime = timestamp;
  const rawDt = (timestamp - lastTime) / 1000;
  lastTime = timestamp;
  const dt = Math.min(rawDt, 0.05) * simSpeed;

  if (!isTraining) {
    simulation.update(dt);
    if (simulation.finished && simulation.aiEnabled && !autoRestartTimer) {
      autoRestartTimer = setTimeout(() => {
        simulation.reset(simulation.nn, true);
        if (chkRandomWaypoints.checked) generateRandomWaypoints();
        autoRestartTimer = null;
      }, 800);
    }
    updateHUD();
    render();
  }
  requestAnimationFrame(gameLoop);
}

btnReset.addEventListener('click', () => {
  if (autoRestartTimer) { clearTimeout(autoRestartTimer); autoRestartTimer = null; }
  simulation.reset(null, true);
  if (chkRandomWaypoints.checked) generateRandomWaypoints();
  btnToggleAI.classList.remove('ai-off');
});

btnToggleAI.addEventListener('click', () => {
  simulation.aiEnabled = !simulation.aiEnabled;
  btnToggleAI.classList.toggle('ai-off', !simulation.aiEnabled);
});

btnTrain.addEventListener('click', async () => {
  if (isTraining) {
    simulation.stopTraining();
    isTraining = false;
    btnTrain.textContent = '🏋️ Обучать непрерывно';
    btnTrain.style.background = '#1565c0';
    progressContainer.style.display = 'none';
    simulation.reset(simulation.bestNetwork || simulation.nn, true);
    if (chkRandomWaypoints.checked) generateRandomWaypoints();
    updateHUD();
    return;
  }
  isTraining = true;
  generationCounter = 0;
  btnTrain.textContent = '⏹ Стоп обучение';
  btnTrain.style.background = '#c62828';
  progressContainer.style.display = 'block';
  progressFill.style.width = '0%';

  await simulation.trainContinuous(
    (generation, current, total, bestScore) => {
      const pct = (current / total) * 100;
      progressFill.style.width = pct + '%';
      progressText.textContent = `Поколение ${generation} — особь ${current}/${total} — скор: ${bestScore.toFixed(0)}`;
      hudGeneration.textContent = generation;
      hudBestScore.textContent = bestScore.toFixed(0);
    },
    (generation, bestScore, bestNetwork, stats) => {
      generationCounter++;
      if (stats) {
        progressText.textContent = `Поколение ${generation} — успех лучшего: ${(stats.bestSuccess * 100).toFixed(0)}%, средний: ${(stats.meanSuccess * 100).toFixed(0)}%`;
      }
      if (generationCounter % 10 === 0 && bestNetwork) {
        simulation.nn = bestNetwork;
        simulation.reset(bestNetwork, true);
        if (chkRandomWaypoints.checked) generateRandomWaypoints();
      }
    }
  );

  isTraining = false;
  btnTrain.textContent = '🏋️ Обучать непрерывно';
  btnTrain.style.background = '#1565c0';
  progressContainer.style.display = 'none';
  simulation.reset(simulation.bestNetwork || simulation.nn, true);
  if (chkRandomWaypoints.checked) generateRandomWaypoints();
  updateHUD();
});

btnMutate.addEventListener('click', () => simulation.mutateCurrent());

btnSave.addEventListener('click', () => {
  simulation.saveBestBrain();
  updateHUD();
  const orig = btnSave.textContent;
  btnSave.textContent = '✓ Сохранено!';
  setTimeout(() => btnSave.textContent = orig, 1500);
});

btnLoad.addEventListener('click', () => {
  simulation.loadBestBrain();
  simulation.reset(simulation.nn, true);
  if (chkRandomWaypoints.checked) generateRandomWaypoints();
  updateHUD();
  const orig = btnLoad.textContent;
  btnLoad.textContent = '✓ Загружено!';
  setTimeout(() => btnLoad.textContent = orig, 1500);
});

btnExport.addEventListener('click', () => simulation.exportBrain());

btnImport.addEventListener('change', (e) => {
  const file = e.target.files[0];
  if (!file) return;
  const reader = new FileReader();
  reader.onload = (event) => {
    try { simulation.importBrain(event.target.result); updateHUD(); alert('✓ Импорт'); }
    catch (err) { alert('Ошибка: ' + err.message); }
  };
  reader.readAsText(file);
  e.target.value = '';
});

simSpeedSlider.addEventListener('input', (e) => {
  simSpeed = parseFloat(e.target.value);
  speedVal.textContent = simSpeed.toFixed(1);
});

document.addEventListener('keydown', (e) => {
  if (simulation.aiEnabled) return;
  if (e.key === 'ArrowLeft') simulation.boat.rudderAngle = -1;
  else if (e.key === 'ArrowRight') simulation.boat.rudderAngle = 1;
});

document.addEventListener('keyup', (e) => {
  if (e.key === 'ArrowLeft' || e.key === 'ArrowRight') simulation.boat.rudderAngle = 0;
});

requestAnimationFrame(gameLoop);

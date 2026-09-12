/* ===========================================================================
   Страница «Траектория»: трёхмерная картина полёта и профиль высоты.

   Что здесь рисуется. Только то, что записала модель: точки траектории —
   это положения тел на тактах интегрирования (см. simulator/trajectory.go),
   события — такты, на которых модель действительно сменила фазу. Ничего не
   интерполируется «для красоты» и не достраивается: между узлами трассы
   проводится прямая, и это единственное допущение на всей странице.

   Координаты приходят в связанной с Землёй системе (ECEF). Это важно:
   в инерциальных осях за десять минут полёта площадка уезжает на полторы
   сотни километров, и возврат бустера «к площадке» выглядел бы промахом.

   Отрисовщик свой, без библиотек: панель зашита в бинарник и обязана
   работать без сети (тот же довод, что и у scene3d.js).

   Заслонение считается точно, а не сортировкой: для каждой точки решается,
   пересекает ли отрезок «глаз — точка» шар Земли. Приближённая сортировка
   по глубине на трассе, которая уходит за горизонт, ошибается там, где это
   заметнее всего.
   =========================================================================== */

(function () {
'use strict';

const EARTH_KM = 6371;          // подставляется из ответа сервера
const COLORS = {
  ship:      '#4a9edd',
  booster:   '#d9a441',
  boostback: '#d9534f',
  pad:       '#4ea87a',
  landing:   '#4ea87a',
  grid:      '#243044',
  globe:     '#151d29',
  globeLit:  '#22405c',
};

// Событие → цвет и короткая подпись. Порядок задаёт и порядок в легенде.
const EVENT_STYLE = {
  'liftoff':         { c: '#4ea87a', t: 'Старт' },
  'separation':      { c: '#e0e6ef', t: 'Разделение' },
  'second-stage':    { c: '#4a9edd', t: 'Вторая ступень' },
  'boostback-start': { c: '#d9534f', t: 'Разворотный импульс' },
  'flip':            { c: '#f07a5a', t: 'Разворот завершён' },
  'boostback-end':   { c: '#d97ad9', t: 'Конец разворотного импульса' },
  'coast':           { c: '#7f8da0', t: 'Баллистика' },
  'circularization': { c: '#4a9edd', t: 'Довыведение' },
  'orbit':           { c: '#4a9edd', t: 'Орбита' },
  'deorbit':         { c: '#a970d8', t: 'Сход с орбиты' },
  'entry-burn':      { c: '#a970d8', t: 'Вход в атмосферу' },
  'landing-flip':    { c: '#f0a05a', t: 'Посадочный разворот' },
  'landing-burn':    { c: '#d9a441', t: 'Посадочный импульс' },
  'touchdown':       { c: '#4ea87a', t: 'Касание' },
  'destroyed':       { c: '#d9534f', t: 'Разрушение' },
  'prelaunch':       { c: '#5d6b7d', t: 'На столе' },
  'phase':           { c: '#8b98a9', t: 'Смена фазы' },
};

const S = {
  run: null,
  earth: EARTH_KM,
  cam: { yaw: -0.6, pitch: 0.42, dist: 1.0 },   // dist — доля от «подогнанной»
  center: { x: 0, y: 0, z: 0 },                 // точка, вокруг которой вращаемся
  fitDist: 1000,
  drag: null,
  hover: null,      // {body, index}
  hover2: null,
  proj3: [],        // экранные координаты точек: {body,i,x,y,vis}
  proj2: [],

  // view2 — показанный участок графика «высота по дальности» в единицах
  // данных {x0,x1 — метры дальности, y0,y1 — метры высоты}. null означает
  // «весь полёт»: границы тогда считаются по самим данным.
  view2: null,
  sel2: null,       // идущее выделение: {x0,y0,x1,y1} в пикселях
  map2: null,       // прямое и обратное отображение экрана и данных
};

/* --------------------------------------------------------------------------
   Небольшая векторная арифметика. Всё в километрах: в метрах координаты
   Земли — восьмизначные числа, и на них теряется точность там, где она
   как раз нужна — у самой поверхности.
   -------------------------------------------------------------------------- */
const V = {
  sub: (a, b) => ({ x: a.x - b.x, y: a.y - b.y, z: a.z - b.z }),
  add: (a, b) => ({ x: a.x + b.x, y: a.y + b.y, z: a.z + b.z }),
  mul: (a, k) => ({ x: a.x * k, y: a.y * k, z: a.z * k }),
  dot: (a, b) => a.x * b.x + a.y * b.y + a.z * b.z,
  cross: (a, b) => ({
    x: a.y * b.z - a.z * b.y,
    y: a.z * b.x - a.x * b.z,
    z: a.x * b.y - a.y * b.x,
  }),
  len: a => Math.sqrt(V.dot(a, a)),
  unit: a => { const n = V.len(a) || 1; return V.mul(a, 1 / n); },
};

/* --------------------------------------------------------------------------
   Загрузка
   -------------------------------------------------------------------------- */

async function loadOptions() {
  const r = await fetch('/api/trajectory/runs');
  if (!r.ok) return;
  const d = await r.json();
  fillSelect('tj-profile', d.profiles, d.current && d.current.profile);
  fillSelect('tj-mission', d.missions, d.current && d.current.mission);
}

function fillSelect(id, items, current) {
  const el = document.getElementById(id);
  if (!el || !items) return;
  el.innerHTML = '';
  for (const it of items) {
    const o = document.createElement('option');
    o.value = it.id;
    o.textContent = it.title;
    if (it.id === current) o.selected = true;
    el.appendChild(o);
  }
}

function sourceMode() {
  const el = document.getElementById('tj-source');
  return el ? el.value : 'live';
}

// loadRun тянет траекторию из выбранного источника.
//
// keepView=true оставляет камеру на месте: при слежении за идущим прогоном
// трасса дорастает каждые пару секунд, и сбрасывать вид на каждое обновление
// значило бы не дать её рассмотреть.
async function loadRun(keepView) {
  const live = sourceMode() === 'live';

  setState(live ? 'беру текущий прогон…' : 'считаю прогон…');
  document.getElementById('tj-load').disabled = true;
  try {
    let url;
    if (live) {
      url = '/api/trajectory/live';
    } else {
      const q = new URLSearchParams({
        seed: document.getElementById('tj-seed').value || '1',
        profile: document.getElementById('tj-profile').value,
        mission: document.getElementById('tj-mission').value,
        duration: document.getElementById('tj-duration').value,
      });
      url = '/api/trajectory?' + q.toString();
    }

    const r = await fetch(url);
    if (!r.ok) {
      const text = (await r.text()).trim();
      setState(r.status === 503
        ? 'прогон не запущен — нажмите Start на пульте'
        : 'ошибка: ' + text);
      return;
    }
    S.run = await r.json();
    S.earth = (S.run.earthRadius || 6371000) / 1000;
    onRunLoaded(keepView, live);
  } catch (e) {
    setState('ошибка: ' + e.message);
  } finally {
    document.getElementById('tj-load').disabled = false;
  }
}

// follow периодически перечитывает живую трассу, пока включено слежение.
let followTimer = null;
function updateFollow() {
  if (followTimer) { clearInterval(followTimer); followTimer = null; }
  const on = document.getElementById('tj-follow').checked && sourceMode() === 'live';
  if (on) followTimer = setInterval(() => loadRun(true), 2000);
}

function setState(text) {
  const el = document.getElementById('tj-state');
  if (el) el.textContent = text;
}

function onRunLoaded(keepView, live) {
  const run = S.run;
  document.getElementById('tj-empty').classList.add('hidden');

  setState(live
    ? (run.truncated ? 'идёт прогон, запись упёрлась в предел' : 'идёт прогон')
    : (run.truncated ? 'запись оборвана по пределу времени' : 'полёт завершён'));
  document.getElementById('tj-seedval').textContent = run.seed;
  document.getElementById('tj-dur').textContent = fmtTime(run.duration);
  document.getElementById('tj-points').textContent =
    `${run.ship.length} + ${run.booster.length}`;

  // Промах бустера — расстояние последней записанной точки до цели.
  // Берётся из самих данных, а не считается заново на странице.
  const b = run.booster;
  document.getElementById('tj-miss').textContent =
    b.length ? fmtDist(b[b.length - 1].dr) : '—';

  // Камера сбрасывается только при смене прогона. Точки при слежении
  // дописываются в конец, и вид обязан остаться там, куда его поставили.
  if (!keepView || !S.fitDist) resetCamera();
  buildEventTable();
  draw();
}

/* --------------------------------------------------------------------------
   Камера
   -------------------------------------------------------------------------- */

function allPoints() {
  if (!S.run) return [];
  return S.run.ship.concat(S.run.booster);
}

// resetCamera ставит вид: камера смотрит на СТАРТОВУЮ ПЛОЩАДКУ, дальность —
// такая, чтобы в кадр попал весь полёт.
//
// Площадка как точка взгляда выбрана не для красоты. Очевидная
// альтернатива — середина облака точек — не работает: координаты связаны с
// Землёй, и у дальнего полёта середина отрезка между стартом и точкой в
// тысяче километров лежит ВНУТРИ планеты. Камера тогда вращается вокруг
// точки под поверхностью, местная вертикаль в ней ни на что не похожа, а
// площадка уходит за горизонт (проверено: луч от глаза до площадки
// опускался до 6190 км при радиусе Земли 6371). Площадка же — настоящая
// точка на поверхности, вокруг которой и построен весь возврат: местная
// вертикаль в ней даёт ровный горизонт, а сама она всегда в кадре.
function resetCamera() {
  const pts = allPoints();
  if (!pts.length) return;

  const pad = S.run.pad;
  S.center = { x: pad.x / 1000, y: pad.y / 1000, z: pad.z / 1000 };

  let far = 0;
  for (const p of pts) {
    const dx = p.x / 1000 - S.center.x;
    const dy = p.y / 1000 - S.center.y;
    const dz = p.z / 1000 - S.center.z;
    far = Math.max(far, Math.sqrt(dx * dx + dy * dy + dz * dz));
  }
  S.fitDist = Math.max(far * 2.2 + 200, 800);
  S.cam = { yaw: -0.6, pitch: 0.42, dist: 1.0 };
}

// camera строит положение глаза и экранный базис.
//
// Камера смотрит на S.center, а «верхом» кадра выбрана местная вертикаль в
// этой точке — не мировая ось Z. Иначе на широте площадки горизонт въезжал
// бы в кадр под углом, и картинка читалась бы хуже без всякой причины.
function camera() {
  const up = V.unit(S.center.x || S.center.y || S.center.z
    ? S.center : { x: 0, y: 0, z: 1 });

  // Опорная пара осей местного горизонта в точке взгляда.
  const north0 = { x: 0, y: 0, z: 1 };
  let east = V.cross(north0, up);
  if (V.len(east) < 1e-6) east = { x: 1, y: 0, z: 0 };
  east = V.unit(east);
  const north = V.unit(V.cross(up, east));

  const d = S.fitDist * S.cam.dist;
  const cp = Math.cos(S.cam.pitch), sp = Math.sin(S.cam.pitch);
  const cy = Math.cos(S.cam.yaw), sy = Math.sin(S.cam.yaw);

  // Направление «от центра к глазу» в осях местного горизонта.
  const dir = V.add(V.add(V.mul(east, cp * cy), V.mul(north, cp * sy)), V.mul(up, sp));
  const eye = V.add(S.center, V.mul(dir, d));

  const forward = V.unit(V.sub(S.center, eye));
  let right = V.cross(forward, up);
  if (V.len(right) < 1e-6) right = east;
  right = V.unit(right);
  const camUp = V.unit(V.cross(right, forward));

  return { eye, forward, right, up: camUp };
}

/* --------------------------------------------------------------------------
   Проекция и заслонение
   -------------------------------------------------------------------------- */

function project(cam, p, w, h) {
  const v = V.sub(p, cam.eye);
  const z = V.dot(v, cam.forward);
  if (z <= 1) return null;                       // за камерой или вплотную
  const f = 0.9 * Math.min(w, h);
  return {
    x: w / 2 + f * V.dot(v, cam.right) / z,
    y: h / 2 - f * V.dot(v, cam.up) / z,
    z,
  };
}

// occluded сообщает, закрывает ли шар Земли точку от камеры.
//
// Решается точно: отрезок «глаз — точка» подставляется в уравнение сферы.
// Радиус берётся чуть меньше настоящего — иначе точки, лежащие ровно на
// поверхности (стартовая площадка, момент касания), считались бы закрытыми
// сами собой из-за ошибок округления и мигали бы.
function occluded(cam, p, R) {
  const d = V.sub(p, cam.eye);
  const a = V.dot(d, d);
  if (a < 1e-9) return false;
  const b = 2 * V.dot(cam.eye, d);
  const c = V.dot(cam.eye, cam.eye) - R * R;
  const disc = b * b - 4 * a * c;
  if (disc <= 0) return false;
  const s = Math.sqrt(disc);
  const t1 = (-b - s) / (2 * a);
  const t2 = (-b + s) / (2 * a);
  return (t1 > 1e-4 && t1 < 1) || (t2 > 1e-4 && t2 < 1);
}

/* --------------------------------------------------------------------------
   Отрисовка трёхмерной сцены
   -------------------------------------------------------------------------- */

// draw перерисовывает обе картины сразу: трёхмерная и профиль высоты
// читаются вместе, и расходиться им нельзя.
function draw() {
  draw3d();
  draw2d();
}

function draw3d() {
  const canvas = document.getElementById('tj-3d');
  if (!canvas || !S.run) return;
  const ctx = fitCanvas(canvas);
  const w = canvas.clientWidth, h = canvas.clientHeight;

  ctx.clearRect(0, 0, w, h);
  ctx.fillStyle = '#0b0f16';
  ctx.fillRect(0, 0, w, h);

  const cam = camera();
  const R = S.earth;
  const Rocc = R * 0.9995;

  drawGlobe(ctx, cam, w, h, R);

  S.proj3 = [];
  drawTrack(ctx, cam, w, h, Rocc, S.run.ship, 'ship', COLORS.ship, null);
  drawTrack(ctx, cam, w, h, Rocc, S.run.booster, 'booster', COLORS.booster, boostbackRange());

  // Цель посадки бустера в этой модели — стартовый комплекс (см.
  // Booster.launchTarget), поэтому обычно это одна и та же точка. Рисовать
  // на ней два кружка и подписывать их порознь значило бы намекать на две
  // разные площадки там, где она одна.
  if (siteDistance(S.run.pad, S.run.landing) > 1) {
    drawSite(ctx, cam, w, h, Rocc, S.run.pad, 'Старт');
    drawSite(ctx, cam, w, h, Rocc, S.run.landing, 'Цель посадки');
  } else {
    drawSite(ctx, cam, w, h, Rocc, S.run.pad, 'Старт / цель посадки');
  }

  drawEvents(ctx, cam, w, h, Rocc);
  drawHoverMarker(ctx);
}

// drawGlobe рисует Землю сеткой параллелей и меридианов с отсечением
// обратной стороны. Заливка идёт четырёхугольниками, отсортированными по
// глубине, — того же рода алгоритм художника, что и в сцене корабля.
function drawGlobe(ctx, cam, w, h, R) {
  const LAT = 24, LON = 48;
  const quads = [];
  const light = V.unit({ x: 0.4, y: 0.5, z: 0.75 });

  for (let i = 0; i < LAT; i++) {
    const th0 = Math.PI * i / LAT, th1 = Math.PI * (i + 1) / LAT;
    for (let j = 0; j < LON; j++) {
      const ph0 = 2 * Math.PI * j / LON, ph1 = 2 * Math.PI * (j + 1) / LON;
      const corners = [
        sphere(R, th0, ph0), sphere(R, th0, ph1),
        sphere(R, th1, ph1), sphere(R, th1, ph0),
      ];
      const mid = V.mul(corners.reduce(V.add, { x: 0, y: 0, z: 0 }), 0.25);
      const n = V.unit(mid);
      // Обратная сторона шара не рисуется вовсе.
      if (V.dot(n, V.unit(V.sub(cam.eye, mid))) <= 0) continue;

      const scr = corners.map(c => project(cam, c, w, h));
      if (scr.some(s => !s)) continue;

      const lit = Math.max(0, V.dot(n, light));
      quads.push({ scr, depth: V.len(V.sub(mid, cam.eye)), lit });
    }
  }
  quads.sort((a, b) => b.depth - a.depth);

  for (const q of quads) {
    const k = 0.20 + 0.80 * q.lit;
    ctx.fillStyle = mix(COLORS.globe, COLORS.globeLit, k);
    ctx.beginPath();
    ctx.moveTo(q.scr[0].x, q.scr[0].y);
    for (let i = 1; i < 4; i++) ctx.lineTo(q.scr[i].x, q.scr[i].y);
    ctx.closePath();
    ctx.fill();
    ctx.strokeStyle = COLORS.grid;
    ctx.lineWidth = 0.5;
    ctx.stroke();
  }
}

function sphere(R, theta, phi) {
  const st = Math.sin(theta);
  return { x: R * st * Math.cos(phi), y: R * st * Math.sin(phi), z: R * Math.cos(theta) };
}

// drawTrack рисует трассу тела. highlight — диапазон индексов, который надо
// выделить отдельным цветом (участок разворотного импульса у бустера).
function drawTrack(ctx, cam, w, h, R, pts, body, color, highlight) {
  if (!pts || !pts.length) return;

  const scr = new Array(pts.length);
  for (let i = 0; i < pts.length; i++) {
    const p = { x: pts[i].x / 1000, y: pts[i].y / 1000, z: pts[i].z / 1000 };
    const s = project(cam, p, w, h);
    if (!s) { scr[i] = null; continue; }
    const vis = !occluded(cam, p, R);
    scr[i] = { x: s.x, y: s.y, vis };
    S.proj3.push({ body, i, x: s.x, y: s.y, vis });
  }

  ctx.lineJoin = 'round';
  ctx.lineCap = 'round';

  for (let i = 1; i < pts.length; i++) {
    const a = scr[i - 1], b = scr[i];
    if (!a || !b) continue;

    const inHi = highlight && i >= highlight.from && i <= highlight.to;
    const hidden = !a.vis || !b.vis;

    ctx.strokeStyle = inHi ? COLORS.boostback : color;
    ctx.lineWidth = inHi ? 3.4 : 2;
    // Закрытая Землёй часть трассы не выбрасывается, а гасится: иначе
    // трасса просто обрывалась бы, и было бы непонятно, кончился полёт
    // или тело ушло за горизонт.
    ctx.globalAlpha = hidden ? 0.16 : 1;
    ctx.beginPath();
    ctx.moveTo(a.x, a.y);
    ctx.lineTo(b.x, b.y);
    ctx.stroke();
  }
  ctx.globalAlpha = 1;
}

// boostbackRange возвращает диапазон точек бустера, попадающих на
// разворотный импульс, — по событиям, а не по догадке о времени.
function boostbackRange() {
  if (!S.run) return null;
  let from = null, to = null;
  for (const e of S.run.events) {
    if (e.body !== 'booster') continue;
    if (e.kind === 'boostback-start' && from === null) from = e.index;
    if (e.kind === 'boostback-end' && to === null) to = e.index;
  }
  if (from === null) return null;
  if (to === null) to = S.run.booster.length - 1;
  return { from, to };
}

function drawSite(ctx, cam, w, h, R, site, label) {
  const p = { x: site.x / 1000, y: site.y / 1000, z: site.z / 1000 };
  const s = project(cam, p, w, h);
  if (!s || occluded(cam, p, R)) return;

  ctx.fillStyle = COLORS.pad;
  ctx.beginPath();
  ctx.arc(s.x, s.y, 5, 0, 2 * Math.PI);
  ctx.fill();
  ctx.strokeStyle = '#0b0f16';
  ctx.lineWidth = 1.5;
  ctx.stroke();

  ctx.fillStyle = '#c8d4e2';
  ctx.font = '11px Inter, system-ui, sans-serif';
  ctx.fillText(label, s.x + 9, s.y + 3);
}

function drawEvents(ctx, cam, w, h, R) {
  if (!S.run) return;
  for (const e of S.run.events) {
    const pts = e.body === 'ship' ? S.run.ship : S.run.booster;
    const pt = pts[e.index];
    if (!pt) continue;
    const p = { x: pt.x / 1000, y: pt.y / 1000, z: pt.z / 1000 };
    const s = project(cam, p, w, h);
    if (!s) continue;
    const hidden = occluded(cam, p, R);

    const st = EVENT_STYLE[e.kind] || EVENT_STYLE.phase;
    ctx.globalAlpha = hidden ? 0.2 : 1;
    ctx.fillStyle = st.c;
    ctx.beginPath();
    ctx.arc(s.x, s.y, 4, 0, 2 * Math.PI);
    ctx.fill();
    ctx.strokeStyle = '#0b0f16';
    ctx.lineWidth = 1.2;
    ctx.stroke();
    ctx.globalAlpha = 1;
  }
}

function drawHoverMarker(ctx) {
  if (!S.hover) return;
  const hit = S.proj3.find(q => q.body === S.hover.body && q.i === S.hover.index);
  if (!hit) return;
  ctx.strokeStyle = '#ffffff';
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  ctx.arc(hit.x, hit.y, 7, 0, 2 * Math.PI);
  ctx.stroke();
}

/* --------------------------------------------------------------------------
   Двумерный профиль: высота по дальности
   -------------------------------------------------------------------------- */

// dataBounds2 — границы по всем данным: «весь полёт» на графике.
function dataBounds2() {
  let maxDr = 0, maxAlt = 0;
  for (const p of allPoints()) {
    maxDr = Math.max(maxDr, p.dr);
    maxAlt = Math.max(maxAlt, p.alt);
  }
  return { x0: 0, x1: Math.max(maxDr, 1), y0: 0, y1: Math.max(maxAlt, 1) };
}

function view2() { return S.view2 || dataBounds2(); }

function draw2d() {
  const canvas = document.getElementById('tj-2d');
  if (!canvas || !S.run) return;
  const ctx = fitCanvas(canvas);
  const w = canvas.clientWidth, h = canvas.clientHeight;

  ctx.clearRect(0, 0, w, h);
  ctx.fillStyle = '#0b0f16';
  ctx.fillRect(0, 0, w, h);

  const pts = allPoints();
  if (!pts.length) return;

  const pad = { l: 62, r: 16, t: 14, b: 30 };
  const plotW = w - pad.l - pad.r, plotH = h - pad.t - pad.b;
  const v = view2();
  const spanX = Math.max(v.x1 - v.x0, 1e-6);
  const spanY = Math.max(v.y1 - v.y0, 1e-6);

  const X = dr => pad.l + plotW * (dr - v.x0) / spanX;
  const Y = alt => (h - pad.b) - plotH * (alt - v.y0) / spanY;
  // Обратное отображение нужно выделению: рамку оператор тянет в пикселях,
  // а запоминать участок надо в метрах — иначе при следующей отрисовке
  // (другой размер окна, другой масштаб) он показал бы не то место.
  S.map2 = {
    pad, w, h,
    invX: px => v.x0 + (px - pad.l) / plotW * spanX,
    invY: py => v.y0 + ((h - pad.b) - py) / plotH * spanY,
  };

  // Сетка и подписи — по показанному участку, а не по всем данным.
  ctx.strokeStyle = '#1c2635';
  ctx.fillStyle = '#5d6b7d';
  ctx.font = '10px JetBrains Mono, monospace';
  ctx.lineWidth = 1;
  for (let i = 0; i <= 5; i++) {
    const y = pad.t + plotH * i / 5;
    ctx.beginPath(); ctx.moveTo(pad.l, y); ctx.lineTo(w - pad.r, y); ctx.stroke();
    ctx.fillText(fmtDist(v.y1 - spanY * i / 5), 6, y + 3);
  }
  for (let i = 0; i <= 6; i++) {
    const x = pad.l + plotW * i / 6;
    ctx.beginPath(); ctx.moveTo(x, pad.t); ctx.lineTo(x, h - pad.b); ctx.stroke();
    ctx.fillText(fmtDist(v.x0 + spanX * i / 6), x - 14, h - pad.b + 14);
  }
  ctx.fillStyle = '#8b98a9';
  ctx.fillText('высота', 6, pad.t - 3);
  ctx.fillText('дальность до точки посадки', w - pad.r - 160, h - 6);

  // Всё, что рисуется по данным, обрезается областью графика: при
  // приближении трасса уходит далеко за её пределы и без обрезки залезла бы
  // на подписи осей.
  ctx.save();
  ctx.beginPath();
  ctx.rect(pad.l, pad.t, plotW, plotH);
  ctx.clip();

  S.proj2 = [];
  plot2d(ctx, S.run.ship, 'ship', COLORS.ship, null, X, Y);
  plot2d(ctx, S.run.booster, 'booster', COLORS.booster, boostbackRange(), X, Y);

  ctx.fillStyle = COLORS.pad;
  ctx.beginPath(); ctx.arc(X(0), Y(0), 4, 0, 2 * Math.PI); ctx.fill();

  for (const e of S.run.events) {
    const pts2 = e.body === 'ship' ? S.run.ship : S.run.booster;
    const p = pts2[e.index];
    if (!p) continue;
    const st = EVENT_STYLE[e.kind] || EVENT_STYLE.phase;
    ctx.fillStyle = st.c;
    ctx.beginPath(); ctx.arc(X(p.dr), Y(p.alt), 3.5, 0, 2 * Math.PI); ctx.fill();
    ctx.strokeStyle = '#0b0f16'; ctx.lineWidth = 1; ctx.stroke();
  }

  if (S.hover2) {
    const arr = S.hover2.body === 'ship' ? S.run.ship : S.run.booster;
    const p = arr[S.hover2.index];
    if (p) {
      ctx.strokeStyle = '#ffffff'; ctx.lineWidth = 1.5;
      ctx.beginPath(); ctx.arc(X(p.dr), Y(p.alt), 6, 0, 2 * Math.PI); ctx.stroke();
    }
  }
  ctx.restore();

  // Рамка выделения поверх всего: она про экран, а не про данные.
  if (S.sel2) {
    const x = Math.min(S.sel2.x0, S.sel2.x1), y = Math.min(S.sel2.y0, S.sel2.y1);
    const rw = Math.abs(S.sel2.x1 - S.sel2.x0), rh = Math.abs(S.sel2.y1 - S.sel2.y0);
    ctx.fillStyle = 'rgba(74,158,221,0.16)';
    ctx.fillRect(x, y, rw, rh);
    ctx.strokeStyle = '#4a9edd';
    ctx.lineWidth = 1;
    ctx.strokeRect(x + 0.5, y + 0.5, rw, rh);
  }

  if (S.view2) {
    ctx.fillStyle = '#8b98a9';
    ctx.font = '10px JetBrains Mono, monospace';
    const label = 'участок · двойной щелчок — весь полёт';
    ctx.fillText(label, w - pad.r - ctx.measureText(label).width, pad.t - 3);
  }
}

function plot2d(ctx, pts, body, color, highlight, X, Y) {
  if (!pts || !pts.length) return;
  const m = S.map2;
  ctx.lineJoin = 'round';
  for (let i = 1; i < pts.length; i++) {
    const inHi = highlight && i >= highlight.from && i <= highlight.to;
    ctx.strokeStyle = inHi ? COLORS.boostback : color;
    ctx.lineWidth = inHi ? 3 : 1.8;
    ctx.beginPath();
    ctx.moveTo(X(pts[i - 1].dr), Y(pts[i - 1].alt));
    ctx.lineTo(X(pts[i].dr), Y(pts[i].alt));
    ctx.stroke();
  }
  // В список для наведения попадают только точки, реально видимые в
  // области графика: иначе при приближении курсор у края цеплял бы точку,
  // которой на экране нет.
  for (let i = 0; i < pts.length; i++) {
    const x = X(pts[i].dr), y = Y(pts[i].alt);
    if (x < m.pad.l || x > m.w - m.pad.r || y < m.pad.t || y > m.h - m.pad.b) continue;
    S.proj2.push({ body, i, x, y });
  }
}

/* --------------------------------------------------------------------------
   Наведение мыши
   -------------------------------------------------------------------------- */

function nearest(list, mx, my, limit) {
  let best = null, bestD = limit * limit;
  for (const q of list) {
    const dx = q.x - mx, dy = q.y - my;
    const d = dx * dx + dy * dy;
    if (d < bestD) { bestD = d; best = q; }
  }
  return best;
}

function tooltipText(body, p) {
  const name = body === 'ship' ? 'Корабль' : 'Бустер';
  return `${name}\n`
    + `время      ${fmtTime(p.t)}\n`
    + `высота     ${fmtDist(p.alt)}\n`
    + `скорость   ${p.v.toFixed(1)} м/с\n`
    + `вертик.    ${p.vz >= 0 ? '+' : ''}${p.vz.toFixed(1)} м/с\n`
    + `до посадки ${fmtDist(p.dr)}\n`
    + `фаза       ${p.phase}`;
}

function showTooltip(el, wrap, mx, my, text) {
  el.textContent = text;
  el.classList.remove('hidden');
  const r = wrap.getBoundingClientRect();
  const bw = el.offsetWidth, bh = el.offsetHeight;
  let x = mx + 14, y = my + 14;
  if (x + bw > r.width) x = mx - bw - 14;
  if (y + bh > r.height) y = my - bh - 14;
  el.style.left = Math.max(0, x) + 'px';
  el.style.top = Math.max(0, y) + 'px';
}

/* --------------------------------------------------------------------------
   Таблица событий
   -------------------------------------------------------------------------- */

function buildEventTable() {
  const host = document.getElementById('tj-events');
  if (!host || !S.run) return;
  const rows = S.run.events.map(e => {
    const st = EVENT_STYLE[e.kind] || EVENT_STYLE.phase;
    const body = e.body === 'ship' ? 'Корабль' : 'Бустер';
    return `<tr>
      <td><span class="ev-dot" style="background:${st.c}"></span>${st.t}</td>
      <td>${body}</td>
      <td class="num">${fmtTime(e.t)}</td>
      <td class="num">${fmtDist(e.alt)}</td>
      <td class="num">${e.v.toFixed(0)} м/с</td>
      <td class="num">${fmtDist(e.dr)}</td>
      <td>${escapeHtml(e.label)}</td>
    </tr>`;
  }).join('');

  host.innerHTML = `<table>
    <thead><tr>
      <th>Событие</th><th>Тело</th><th class="num">T+</th>
      <th class="num">Высота</th><th class="num">Скорость</th>
      <th class="num">До посадки</th><th>Фаза модели</th>
    </tr></thead>
    <tbody>${rows}</tbody></table>`;
}

/* --------------------------------------------------------------------------
   Мелочи
   -------------------------------------------------------------------------- */

function fitCanvas(canvas) {
  const dpr = window.devicePixelRatio || 1;
  const w = canvas.clientWidth, h = canvas.clientHeight;
  if (canvas.width !== Math.round(w * dpr) || canvas.height !== Math.round(h * dpr)) {
    canvas.width = Math.round(w * dpr);
    canvas.height = Math.round(h * dpr);
  }
  const ctx = canvas.getContext('2d');
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  return ctx;
}

function fmtTime(s) {
  if (!isFinite(s)) return '—';
  const m = Math.floor(s / 60), r = s - m * 60;
  return m > 0 ? `${m}:${r.toFixed(1).padStart(4, '0')}` : `${r.toFixed(1)} с`;
}

function fmtDist(m) {
  if (!isFinite(m)) return '—';
  const a = Math.abs(m);
  if (a >= 10000) return (m / 1000).toFixed(0) + ' км';
  if (a >= 1000) return (m / 1000).toFixed(1) + ' км';
  return m.toFixed(0) + ' м';
}

function siteDistance(a, b) {
  const dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
  return Math.sqrt(dx * dx + dy * dy + dz * dz);
}

function mix(c1, c2, k) {
  const p = h => [parseInt(h.slice(1, 3), 16), parseInt(h.slice(3, 5), 16), parseInt(h.slice(5, 7), 16)];
  const a = p(c1), b = p(c2);
  const v = a.map((x, i) => Math.round(x + (b[i] - x) * k));
  return `rgb(${v[0]},${v[1]},${v[2]})`;
}

function escapeHtml(s) {
  return String(s).replace(/[&<>"]/g, c =>
    ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
}

/* --------------------------------------------------------------------------
   Ввод
   -------------------------------------------------------------------------- */

function bind() {
  const src = document.getElementById('tj-source');
  const seedBox = document.getElementById('tj-seed-box');
  const syncSource = () => {
    seedBox.style.display = sourceMode() === 'seed' ? '' : 'none';
    updateFollow();
  };
  src.addEventListener('change', syncSource);
  document.getElementById('tj-follow').addEventListener('change', updateFollow);
  syncSource();

  document.getElementById('tj-load').addEventListener('click', () => loadRun(false));
  document.getElementById('tj-seed').addEventListener('keydown', e => {
    if (e.key === 'Enter') loadRun(false);
  });

  const canvas = document.getElementById('tj-3d');
  const wrap = canvas.parentElement;
  const tip = document.getElementById('tj-tooltip');

  canvas.addEventListener('mousedown', e => {
    S.drag = { x: e.clientX, y: e.clientY, yaw: S.cam.yaw, pitch: S.cam.pitch };
  });
  window.addEventListener('mouseup', () => { S.drag = null; });
  window.addEventListener('mousemove', e => {
    if (!S.drag) return;
    S.cam.yaw = S.drag.yaw + (e.clientX - S.drag.x) * 0.006;
    S.cam.pitch = S.drag.pitch + (e.clientY - S.drag.y) * 0.006;
    // Снизу — местный горизонт площадки, сверху — зенит без самого полюса
    // (там местная вертикаль совпадает с осью взгляда, базис вырождается и
    // картинка прыгает). Уходить под горизонт незачем: оттуда площадку
    // закрывает сама Земля, и смотреть не на что.
    S.cam.pitch = Math.max(0.05, Math.min(1.45, S.cam.pitch));
    draw();
  });

  canvas.addEventListener('wheel', e => {
    e.preventDefault();
    S.cam.dist *= Math.exp(e.deltaY * 0.0012);
    S.cam.dist = Math.max(0.03, Math.min(6, S.cam.dist));
    draw();
  }, { passive: false });

  canvas.addEventListener('dblclick', () => { resetCamera(); draw(); });

  canvas.addEventListener('mousemove', e => {
    if (S.drag || !S.run) return;
    const r = canvas.getBoundingClientRect();
    const mx = e.clientX - r.left, my = e.clientY - r.top;
    const hit = nearest(S.proj3.filter(q => q.vis), mx, my, 14);
    if (!hit) {
      tip.classList.add('hidden');
      if (S.hover) { S.hover = null; draw(); }
      return;
    }
    const arr = hit.body === 'ship' ? S.run.ship : S.run.booster;
    S.hover = { body: hit.body, index: hit.i };
    showTooltip(tip, wrap, mx, my, tooltipText(hit.body, arr[hit.i]));
    draw();
  });
  canvas.addEventListener('mouseleave', () => {
    tip.classList.add('hidden');
    S.hover = null; draw();
  });

  // --- график «высота по дальности»: выделение зоны, как в Grafana.
  const c2 = document.getElementById('tj-2d');
  const wrap2 = c2.parentElement;
  const tip2 = document.getElementById('tj-tooltip2');

  const inPlot = (x, y) => {
    const m = S.map2;
    if (!m) return false;
    return x >= m.pad.l && x <= m.w - m.pad.r && y >= m.pad.t && y <= m.h - m.pad.b;
  };
  const clampToPlot = (x, y) => {
    const m = S.map2;
    return {
      x: Math.max(m.pad.l, Math.min(m.w - m.pad.r, x)),
      y: Math.max(m.pad.t, Math.min(m.h - m.pad.b, y)),
    };
  };
  const localPos = e => {
    const r = c2.getBoundingClientRect();
    return { x: e.clientX - r.left, y: e.clientY - r.top };
  };

  c2.addEventListener('mousedown', e => {
    if (!S.run || !S.map2) return;
    const p = localPos(e);
    if (!inPlot(p.x, p.y)) return;
    e.preventDefault();
    S.sel2 = { x0: p.x, y0: p.y, x1: p.x, y1: p.y };
    tip2.classList.add('hidden');
    S.hover2 = null;
  });

  window.addEventListener('mousemove', e => {
    if (!S.sel2) return;
    const p = clampToPlot(localPos(e).x, localPos(e).y);
    S.sel2.x1 = p.x; S.sel2.y1 = p.y;
    draw2d();
  });

  window.addEventListener('mouseup', () => {
    if (!S.sel2) return;
    const sel = S.sel2;
    S.sel2 = null;

    // Слишком мелкая рамка — это промах или обычный щелчок, а не выделение.
    // Приблизить по ней значило бы прыгнуть в случайную точку и потерять
    // картину; такое выделение отбрасывается.
    if (Math.abs(sel.x1 - sel.x0) < 6 || Math.abs(sel.y1 - sel.y0) < 6) {
      draw2d();
      return;
    }
    const m = S.map2;
    const x0 = m.invX(Math.min(sel.x0, sel.x1)), x1 = m.invX(Math.max(sel.x0, sel.x1));
    const y0 = m.invY(Math.max(sel.y0, sel.y1)), y1 = m.invY(Math.min(sel.y0, sel.y1));
    S.view2 = { x0, x1, y0, y1 };
    draw2d();
  });

  c2.addEventListener('dblclick', () => {
    S.view2 = null;
    draw2d();
  });

  // Колесо приближает и отдаляет вокруг курсора: то место, на которое
  // смотрит оператор, остаётся под ним, а не уезжает к середине.
  c2.addEventListener('wheel', e => {
    if (!S.run || !S.map2) return;
    const p = localPos(e);
    if (!inPlot(p.x, p.y)) return;
    e.preventDefault();

    const v = view2();
    const k = Math.exp(e.deltaY * 0.0015);
    const ax = S.map2.invX(p.x), ay = S.map2.invY(p.y);
    let nx0 = ax + (v.x0 - ax) * k, nx1 = ax + (v.x1 - ax) * k;
    let ny0 = ay + (v.y0 - ay) * k, ny1 = ay + (v.y1 - ay) * k;

    const full = dataBounds2();
    // Дальше «всего полёта» отдалять некуда: за границами данных ничего
    // нет, и пустое поле только сбивало бы масштаб.
    if (nx1 - nx0 >= full.x1 - full.x0 && ny1 - ny0 >= full.y1 - full.y0) {
      S.view2 = null;
    } else {
      S.view2 = { x0: nx0, x1: nx1, y0: ny0, y1: ny1 };
    }
    draw2d();
  }, { passive: false });

  c2.addEventListener('mousemove', e => {
    if (!S.run || S.sel2) return;
    const p = localPos(e);
    const hit = nearest(S.proj2, p.x, p.y, 12);
    if (!hit) {
      tip2.classList.add('hidden');
      if (S.hover2) { S.hover2 = null; draw2d(); }
      return;
    }
    const arr = hit.body === 'ship' ? S.run.ship : S.run.booster;
    S.hover2 = { body: hit.body, index: hit.i };
    showTooltip(tip2, wrap2, p.x, p.y, tooltipText(hit.body, arr[hit.i]));
    draw2d();
  });
  c2.addEventListener('mouseleave', () => {
    tip2.classList.add('hidden');
    S.hover2 = null; draw2d();
  });

  window.addEventListener('resize', () => { draw(); draw2d(); });
}

// Запуск — только в браузере. В Node страницы нет, и файл в этом случае
// просто отдаёт свою геометрию наружу: проверять проекцию и заслонение
// удобнее без браузера, а дублировать эти формулы в отдельном скрипте
// значило бы проверять копию вместо того, что работает на странице.
if (typeof document !== 'undefined') {
  loadOptions().then(() => {
  bind();
  // Страница открывается на том, что идёт прямо сейчас: чаще всего за
  // траекторией приходят именно к идущему прогону.
  loadRun(false);
});
} else if (typeof module !== 'undefined' && module.exports) {
  module.exports = { S, V, camera, project, occluded, draw3d, draw2d, onRunLoaded };
}

})();

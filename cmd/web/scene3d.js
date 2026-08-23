/* ===========================================================================
   Трёхмерная сцена: корабль крупным планом и мини-глобус.

   Зачем вообще картинка. Числа в телеметрии не дают представления о том, чем
   корабль сейчас повёрнут к потоку, раскрыты ли плавники и куда он летит.
   Сцена берёт те же величины, по которым модель считает силы, и показывает их
   как есть: оси корпуса приходят из телеметрии готовой тройкой, направление
   потока — оттуда же. Здесь ничего не досчитывается и не подгоняется, иначе
   картинка начала бы врать независимо от модели.

   Отрисовщик свой, без внешних библиотек. Панель зашита в бинарник и обязана
   работать в Docker без сети, а корпус ракеты — это несколько десятков
   четырёхугольников: тянуть ради них мегабайт стороннего кода незачем.

   Устройство простое:
     — мир — местный горизонт: X на восток, Y на север, Z вверх;
     — корпус строится в связанных осях (X к носу, Y вправо, Z к брюху)
       и переносится в мир той самой тройкой осей из телеметрии;
     — грани сортируются по глубине и заливаются — алгоритм художника.
       Для выпуклого тела вращения этого достаточно.

   Весь файл завёрнут в замыкание, и это не украшение. Оба файла панели —
   обычные скрипты с общей глобальной областью, поэтому одноимённые функции
   молча затирают друг друга. Так и вышло: здешняя project() — проекция точки
   мира на экран — перекрыла project() из app.js, которая рисует указатель
   ориентации. Обработчик снимка начал падать на первом же кадре, и вместе
   с ним переставали обновляться все вкладки: значения застывали до тех пор,
   пока переключение вкладки не вызывало отрисовку напрямую.
   =========================================================================== */

(function () {

const S3 = {
  cam: { yaw: 2.2, pitch: 0.22, dist: 320 },
  zoom: 1,          // во сколько раз оператор приблизил камеру
  fitStage: null,   // ступень, под которую подобрана дальность
  drag: null,
  track: [],        // след трассы: широта и долгота
  lastTime: -1,
  ready: false,
  showGrid: true,
  showFlow: true,
  followFlow: false,
  bodyCam: true,    // камера привязана к корпусу, а не к горизонту

  // Буфер последних состояний и номер хода отрисовки.
  buf: [],
  loop: 0,

  // Часы воспроизведения: положение в модельном времени, скорость их хода
  // в модельных секундах на реальную и метка последнего продвижения.
  clock: 0,
  rate: 0,
  clockAt: 0,
};

/* Глубина буфера кадров.

   Четырёх хватает: часы держатся на полтора кадра позади, и остаётся ещё
   столько же запаса на опоздание. Держать больше — значит копить состояния,
   до которых картинка доберётся с задержкой в четверть секунды. */
const SCENE_BUFFER = 4;

/* --------------------------------------------------------------------------
   Векторная мелочь
   -------------------------------------------------------------------------- */

const v3 = (x = 0, y = 0, z = 0) => ({ x, y, z });
const vAdd = (a, b) => v3(a.x + b.x, a.y + b.y, a.z + b.z);
const vSub = (a, b) => v3(a.x - b.x, a.y - b.y, a.z - b.z);
const vMul = (a, k) => v3(a.x * k, a.y * k, a.z * k);
const vDot = (a, b) => a.x * b.x + a.y * b.y + a.z * b.z;
const vCross = (a, b) => v3(
  a.y * b.z - a.z * b.y,
  a.z * b.x - a.x * b.z,
  a.x * b.y - a.y * b.x);
const vLen = a => Math.sqrt(vDot(a, a));
const vUnit = a => { const n = vLen(a); return n > 1e-9 ? vMul(a, 1 / n) : v3(); };

/* Вектор из телеметрии (восток, север, верх) в мировые оси сцены. */
const fromScene = s => v3(s?.e || 0, s?.n || 0, s?.u || 0);

/* --------------------------------------------------------------------------
   Запуск вкладки
   -------------------------------------------------------------------------- */

function initScene3D() {
  if (S3.ready) return;

  const canvas = document.getElementById('scene-canvas');
  if (!canvas) return;

  S3.canvas = canvas;
  S3.ctx = canvas.getContext('2d');
  S3.globe = document.getElementById('scene-globe');
  S3.globeCtx = S3.globe ? S3.globe.getContext('2d') : null;

  // Мышь: тяга вращает камеру, колесо приближает.
  canvas.addEventListener('pointerdown', e => {
    S3.drag = { x: e.clientX, y: e.clientY };
    canvas.setPointerCapture?.(e.pointerId);
  });
  canvas.addEventListener('pointermove', e => {
    if (!S3.drag) return;
    S3.cam.yaw -= (e.clientX - S3.drag.x) * 0.008;
    S3.cam.pitch += (e.clientY - S3.drag.y) * 0.006;
    S3.cam.pitch = Math.max(-1.45, Math.min(1.45, S3.cam.pitch));
    S3.drag = { x: e.clientX, y: e.clientY };
    drawScene3D();
  });
  const stop = () => { S3.drag = null; };
  canvas.addEventListener('pointerup', stop);
  canvas.addEventListener('pointercancel', stop);

  canvas.addEventListener('wheel', e => {
    e.preventDefault();
    S3.zoom *= e.deltaY > 0 ? 1.12 : 0.89;
    S3.zoom = Math.max(0.25, Math.min(6, S3.zoom));
    drawScene3D();
  }, { passive: false });

  const bind = (id, key) => {
    const box = document.getElementById(id);
    if (box) box.onchange = () => { S3[key] = box.checked; drawScene3D(); };
  };
  bind('scene-grid', 'showGrid');
  bind('scene-flow', 'showFlow');
  bind('scene-follow', 'followFlow');
  bind('scene-body', 'bodyCam');

  const reset = document.getElementById('scene-reset');
  if (reset) reset.onclick = () => {
    S3.cam.yaw = 2.2;
    S3.cam.pitch = 0.22;
    S3.zoom = 1;
    S3.fitStage = null;
    drawScene3D();
  };

  S3.ready = true;
}

/* --------------------------------------------------------------------------
   Кадр
   -------------------------------------------------------------------------- */

/* Приход телеметрии.

   Снимки приходят несколько раз в секунду, а кадров глазу нужно шестьдесят.
   Поэтому обработчик снимка сам ничего не рисует: он кладёт снимок в буфер
   из двух последних, а рисует отдельный ход по requestAnimationFrame,
   подставляя промежуточное состояние между ними. Иначе корабль на экране
   дёргается — ровно один кадр на снимок. */
function drawScene3D() {
  initScene3D();
  if (!S3.ctx) return;

  pushFrame(S.snapshot?.telemetry);
  renderScene();
  ensureLoop();
}

/* Приход лёгкого кадра сцены — того, что идёт между полными снимками.
   Для буфера кадров он равноправен снимку: поля, которые сцена читает,
   в нём те же и называются так же. */
function onSceneFrame(frame) {
  initScene3D();
  if (!S3.ctx) return;

  pushFrame(frame);
  ensureLoop();
}

function pushFrame(tel) {
  if (!tel) return;

  const last = S3.buf[S3.buf.length - 1];

  // Тот же снимок второй раз не считаем.
  //
  // Сравнивать приходится и по ссылке, и по модельному времени. По ссылке —
  // потому что drawScene3D вызывается не только по приходу данных, но и на
  // каждое движение мыши. По времени — потому что на паузе состояния приходят
  // разными объектами, а модель стоит: приняв их за движение, интерполяция
  // растянула бы нулевой промежуток и картинка поехала бы сама собой.
  if (last && (last.tel === tel || last.mt === tel.time)) return;

  // Ход назад означает перезапуск или перемотку: старые кадры к новому отсчёту
  // отношения не имеют, и вести по ним часы нельзя.
  if (last && (tel.time || 0) < last.mt) resetClock();

  S3.buf.push({ at: performance.now(), mt: tel.time || 0, tel });
  if (S3.buf.length > SCENE_BUFFER) S3.buf.shift();
}

function resetClock() {
  S3.buf = [];
  S3.clock = 0;
  S3.rate = 0;
  S3.clockAt = 0;
}

/* Ход отрисовки живёт, пока вкладка на экране. */
function ensureLoop() {
  if (S3.loop) return;

  const step = () => {
    if (!sceneVisible()) { S3.loop = 0; return; }
    S3.loop = window.requestAnimationFrame(step);
    renderScene();
  };
  S3.loop = window.requestAnimationFrame(step);
}

/* Крутить ход отрисовки, когда вкладка не на экране, незачем: это чистая
   трата кадров. Спрашиваем не про размещение узла, а про состояние панели —
   так же, как это делает обработчик снимка. */
function sceneVisible() {
  if (!S3.canvas) return false;
  if (document.visibilityState === 'hidden') return false;
  return S.dataView === 'scene';
}

/* Часы воспроизведения.

   Промежуточное состояние надо чем-то отмерять, и напрасно это делалось по
   времени прихода сообщений. Сеть и таймеры дают разброс: пакет приходит то
   через сорок миллисекунд, то через семьдесят, изредка два подряд. Доля
   промежутка, посчитанная по такому разбросу, повторяет его целиком, и корабль
   на экране то замирает, то дёргается вперёд — при том, что модель шла ровно.

   Поэтому отсчёт ведётся по модельному времени. Часы идут сами, со скоростью,
   измеренной по буферу кадров, и держатся на полтора кадра позади последнего
   известного состояния. Этот запас и есть плавность: пока опоздание кадра
   меньше запаса, на картинке оно никак не сказывается. Платой служит та же
   задержка — семьдесят пять миллисекунд, которых на глаз не видно. */
function sceneRate(buf) {
  const first = buf[0], last = buf[buf.length - 1];
  const real = (last.at - first.at) / 1000;
  const model = last.mt - first.mt;

  // Скорость меряется по всему буферу разом, а не усреднением отношений
  // по соседним парам. Отношение — величина несимметричная: опоздание вдвое
  // даёт половину скорости, а приход вдвое раньше — двойную, и среднее таких
  // отношений всегда завышено. Стенд ловил на этом ход часов 1.15 вместо 1.00.
  if (real < 0.02 || model <= 0) return S3.rate || 0;

  const rate = model / real;
  return S3.rate > 0 ? S3.rate * 0.85 + rate * 0.15 : rate;
}

/* Промежуток по модельному времени между соседними кадрами.

   Берётся наименьший в буфере, а не средний: пропуск кадра сливает два
   промежутка в один, и среднее от такого пропуска уезжает, а наименьшее —
   нет. */
function sceneInterval(buf) {
  let best = Infinity;
  for (let i = 1; i < buf.length; i++) {
    const d = buf[i].mt - buf[i - 1].mt;
    if (d > 0 && d < best) best = d;
  }
  return best;
}

function blendFrames() {
  const buf = S3.buf;
  if (!buf.length) return null;

  const last = buf[buf.length - 1];
  if (buf.length < 2) return last.tel;

  const interval = sceneInterval(buf);
  if (!isFinite(interval)) return last.tel;

  // Промежуточные состояния имеют смысл только между близкими снимками.
  //
  // На ускорении в пятьдесят раз соседние снимки разделены секундами
  // модельного времени: за это время корабль проходит сотни метров и успевает
  // развернуться. Плавно перетекать между такими состояниями — значит рисовать
  // движение, которого не было. Тогда честнее показывать последнее известное
  // состояние, пусть и рывками. То же при смене фазы: между «посадочным
  // импульсом» и «сел» промежутка не существует.
  const prev = buf[buf.length - 2];
  if (interval > 1.05 || prev.tel.phase !== last.tel.phase) {
    S3.clock = 0;
    return last.tel;
  }

  const now = performance.now();
  const dt = S3.clockAt ? Math.min((now - S3.clockAt) / 1000, 0.25) : 0;
  S3.clockAt = now;
  S3.rate = sceneRate(buf);

  // Где часам полагается быть сейчас: полтора кадра позади последнего
  // состояния, плюс то, что натикало с момента его прихода.
  const target = last.mt - interval * 1.5 + (now - last.at) / 1000 * S3.rate;

  // Уход больше буфера означает, что поток прерывался: пауза, перезапуск,
  // возврат на вкладку. Догонять такой разрыв плавно незачем — это будет
  // длинная поездка через состояния, которых уже нет.
  if (!(S3.clock > 0) || Math.abs(S3.clock - target) > interval * SCENE_BUFFER) {
    S3.clock = target;
  } else {
    // Ход собственный, а приходящие кадры лишь мягко его подправляют: так
    // разброс времени прихода в картинку не попадает.
    S3.clock += dt * S3.rate + (target - S3.clock) * 0.06;
  }
  S3.clock = Math.max(buf[0].mt, Math.min(last.mt, S3.clock));

  // Пара кадров, между которыми оказались часы.
  let i = buf.length - 2;
  while (i > 0 && buf[i].mt > S3.clock) i--;

  const from = buf[i], to = buf[i + 1];
  const span = to.mt - from.mt;
  if (span <= 0) return to.tel;

  const k = Math.max(0, Math.min(1, (S3.clock - from.mt) / span));

  // Углы и оси перекладываются линейно и нормируются: за двадцатую долю
  // секунды корпус поворачивается на доли градуса, и разница между честной
  // сферической интерполяцией и линейной там меньше толщины линии.
  const a = from.tel, b = to.tel;
  const mix = (x, y) => x + (y - x) * k;
  const vec = (x, y) => vUnit(v3(
    mix(x?.e || 0, y?.e || 0), mix(x?.n || 0, y?.n || 0), mix(x?.u || 0, y?.u || 0)));

  const out = Object.create(b);
  out.altitude = mix(a.altitude, b.altitude);
  out.totalVelocity = mix(a.totalVelocity, b.totalVelocity);
  out.verticalVelocity = mix(a.verticalVelocity, b.verticalVelocity);
  out.totalAoA = mix(a.totalAoA, b.totalAoA);
  out.dynamicPressure = mix(a.dynamicPressure, b.dynamicPressure);
  out.heatFlux = mix(a.heatFlux, b.heatFlux);
  out.throttle = mix(a.throttle, b.throttle);
  out.mach = mix(a.mach, b.mach);

  const sa = a.scene, sb = b.scene;
  if (sa && sb) {
    const toScene = u => ({ e: u.x, n: u.y, u: u.z });
    out.scene = {
      forward: toScene(vec(sa.forward, sb.forward)),
      right: toScene(vec(sa.right, sb.right)),
      down: toScene(vec(sa.down, sb.down)),
      airflow: toScene(vec(sa.airflow, sb.airflow)),
      velocity: toScene(vec(sa.velocity, sb.velocity)),
      downrange: mix(sa.downrange, sb.downrange),
    };
  }

  if (a.flaps && b.flaps && a.flaps.length === b.flaps.length) {
    out.flaps = b.flaps.map((f, i) => ({
      ...f, deflection: mix(a.flaps[i].deflection, f.deflection),
    }));
  }
  return out;
}

function renderScene() {
  const ctx = S3.ctx;
  if (!ctx) return;

  S3.framesDrawn = (S3.framesDrawn || 0) + 1;

  const t = blendFrames();
  const canvas = S3.canvas;

  // Размер холста в устройстве и в разметке разный: без этого на экране
  // с удвоенной плотностью картинка мылится.
  const dpr = window.devicePixelRatio || 1;
  const w = canvas.clientWidth || 640;
  const h = canvas.clientHeight || 420;
  if (canvas.width !== Math.round(w * dpr) || canvas.height !== Math.round(h * dpr)) {
    canvas.width = Math.round(w * dpr);
    canvas.height = Math.round(h * dpr);
  }
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);

  drawSky(ctx, w, h, t?.altitude || 0);

  if (!t) {
    hudText(ctx, w, h, ['Нет телеметрии']);
    return;
  }

  collectTrack(t);
  fitCamera(t);

  const cam = buildCamera(t, w, h);
  if (window.__scene) window.__scene.cam = cam; // для стенда

  const faces = [];
  if (S3.showGrid) planetFaces(faces, t);
  vehicleFaces(faces, t);
  drawFaces(ctx, faces, cam);

  if (S3.showGrid) drawGroundGrid(ctx, cam, t);
  if (S3.showFlow) drawArrows(ctx, cam, t);

  drawGlobe(t);
  hudText(ctx, w, h, hudLines(t));
}

/* Дальность камеры под то, что сейчас летит.

   До разделения в кадре сто двадцать метров сборки, после — полсотни метров
   корабля. Одна и та же дальность годится только для чего-то одного: ставим
   её по длине, а приближение оператора храним отдельным множителем, чтобы
   разделение не сбрасывало выбранный им масштаб. */
function fitCamera(t) {
  const layout = S.layout;
  if (!layout?.stages?.length) return;
  if (S3.fitStage === t.stage) return;

  S3.base = vehicleLength(t) * 2.4;
  S3.fitStage = t.stage;
}

/* Длина того, что сейчас летит: до разделения вся сборка, после — корабль. */
function vehicleLength(t) {
  const layout = S.layout;
  if (!layout?.stages?.length) return 120;

  return t.stage >= 2
    ? (layout.stages.find(s => s.index === 2)?.length || layout.totalLength)
    : layout.totalLength;
}

function vehicleHalf(t) { return vehicleLength(t) / 2; }

/* Небо: у земли синее, выше — чёрное. Граница проходит там же, где
   кончается заметная атмосфера. */
function drawSky(ctx, w, h, altitude) {
  const air = Math.max(0, 1 - altitude / 90000);
  const g = ctx.createLinearGradient(0, 0, 0, h);
  g.addColorStop(0, `rgb(${8 + 20 * air}, ${10 + 34 * air}, ${16 + 70 * air})`);
  g.addColorStop(1, `rgb(${10 + 60 * air}, ${13 + 80 * air}, ${20 + 120 * air})`);
  ctx.fillStyle = g;
  ctx.fillRect(0, 0, w, h);
}

/* --------------------------------------------------------------------------
   Камера
   -------------------------------------------------------------------------- */

function buildCamera(t, w, h) {
  let yaw = S3.cam.yaw;

  // «По потоку»: камера встаёт так, чтобы поток шёл слева направо,
  // и угол атаки читался с картинки напрямую.
  if (S3.followFlow) {
    const flow = fromScene(t.scene?.airflow);
    if (vLen(flow) > 0.5) yaw = Math.atan2(flow.x, flow.y) + Math.PI / 2;
  }

  const dist = (S3.base || S3.cam.dist) * S3.zoom;
  S3.cam.dist = dist;

  const cp = Math.cos(S3.cam.pitch), sp = Math.sin(S3.cam.pitch);
  let pos = v3(
    dist * cp * Math.sin(yaw),
    dist * cp * Math.cos(yaw),
    dist * sp);

  // Камера по корпусу. Корабль на возвращении непрерывно вращается: держать
  // камеру привязанной к горизонту — значит смотреть на кувыркающуюся машину
  // и не разглядеть ни положения плавников, ни того, какой стороной она идёт.
  // Привязанная к корпусу камера оставляет корабль на месте, а поворачивается
  // вместе с ним мир — стрелка потока и сетка земли показывают, как именно.
  if (S3.bodyCam && !S3.followFlow) {
    const toWorld = bodyToWorld(t);
    // Точка обзора задаётся в связанных осях: смотрим сбоку и чуть со спины,
    // чтобы плавники были видны.
    pos = toWorld(v3(
      dist * sp,
      dist * cp * Math.sin(yaw),
      -dist * cp * Math.cos(yaw)));
  }

  // Смотрим на середину корпуса, а не на начало координат.
  //
  // Начало координат — срез сопел: так высота из телеметрии совпадает
  // с уровнем, которым изделие стоит на площадке. Но держать в центре кадра
  // хвост незачем, поэтому точка прицеливания поднимается на полкорпуса.
  const target = vAdd(v3(0, 0, 0), vMul(bodyToWorld(t)(v3(1, 0, 0)), vehicleHalf(t)));

  pos = vAdd(pos, target);

  // Камера не уходит под грунт.
  //
  // Привязанная к корпусу точка обзора вращается вместе с кораблём, и у земли
  // она запросто оказывается ниже поверхности: тогда планета закрывает кадр
  // снизу, корабль висит в пустоте, и кажется, что земли нет вовсе. Поднимаем
  // камеру до нескольких метров над грунтом, направление взгляда не меняется —
  // она всё так же смотрит на корабль.
  const groundZ = -(t.altitude || 0) + 8;
  if (pos.z < groundZ) pos = v3(pos.x, pos.y, groundZ);

  const fwd = vUnit(vSub(target, pos));
  let right = vCross(fwd, v3(0, 0, 1));
  if (vLen(right) < 1e-6) right = v3(1, 0, 0);
  right = vUnit(right);
  const up = vCross(right, fwd);

  return { pos, target, fwd, right, up, cx: w / 2, cy: h / 2, f: h * 1.15 };
}

/* Точка мира на экран. z — глубина по оси взгляда. */
function project(p, cam) {
  const d = vSub(p, cam.pos);
  const z = vDot(d, cam.fwd);
  if (z < 0.4) return null;
  return {
    x: cam.cx + cam.f * vDot(d, cam.right) / z,
    y: cam.cy - cam.f * vDot(d, cam.up) / z,
    z,
  };
}

/* --------------------------------------------------------------------------
   Корпус

   Строится в связанных осях: X к носу, Y вправо, Z к брюху. Обводы берутся
   из тех же данных, что и развёртка, — из /api/vehicle/layout.
   -------------------------------------------------------------------------- */

function bodyToWorld(t) {
  const f = vUnit(fromScene(t.scene?.forward) || v3(0, 0, 1));
  const r = vUnit(fromScene(t.scene?.right));
  const d = vUnit(fromScene(t.scene?.down));

  // Оси приходят готовой тройкой, и собирать матрицу из углов не нужно:
  // любое расхождение в порядке поворотов дало бы корабль, летящий боком.
  return p => v3(
    f.x * p.x + r.x * p.y + d.x * p.z,
    f.y * p.x + r.y * p.y + d.y * p.z,
    f.z * p.x + r.z * p.y + d.z * p.z);
}

/* Кольцо радиуса r на продольной координате x. */
function ring(x, r, segments) {
  const out = [];
  for (let i = 0; i < segments; i++) {
    const a = (i / segments) * Math.PI * 2;
    // Отсчёт от брюха: угол ноль смотрит в +Z, чтобы плитки ложились
    // на ту же сторону, которую модель считает наветренной.
    out.push({ x, y: r * Math.sin(a), z: r * Math.cos(a), a });
  }
  return out;
}

function vehicleFaces(faces, t) {
  const layout = S.layout;
  if (!layout || !layout.stages?.length) return;

  const toWorld = bodyToWorld(t);
  const seg = 20;

  // После разделения рисуется только корабль.
  const stages = t.stage >= 2
    ? layout.stages.filter(s => s.index === 2)
    : layout.stages;

  const total = stages.reduce((s, x) => s + x.length, 0);

  // Начало отсчёта — срез сопел, а не середина корпуса.
  //
  // Высота в телеметрии — это высота точки, которой изделие стоит на площадке:
  // на старте она равна нулю, и при касании тоже. Если строить корпус вокруг
  // середины, то при нулевой высоте середина оказывается на уровне грунта,
  // и нижняя половина корабля уходит под землю — ровно это и было видно
  // после посадки.
  let x = total;

  const shield = t.heatShield;
  const tileColour = shieldColour(shield?.tiles, [118, 100, 88]);
  const steelColour = shieldColour(shield?.steel, [186, 194, 202]);

  for (const stage of stages) {
    for (const section of stage.sections) {
      const r = section.diameter / 2;
      const x0 = x, x1 = x - section.length;

      // Верх верхней ступени — носовая часть. В обводах она называется
      // грузовым отсеком: у корабля это одно и то же место, там же и купол.
      //
      // Плиточная теплозащита — не у всякой второй ступени: у Falcon 9 она
      // одноразовая и не входит в атмосферу управляемо, плиток там нет.
      // layout.flaps — тот же признак «корабль класса Starship», что и у
      // плавников (сейчас это одно и то же семейство носителей), поэтому
      // им же гасится и рисовка плиток на чужой ракете.
      const tiled = stage.index === 2 && !!layout.flaps;
      const top = stage === stages[0] && section === stage.sections[0];

      if (section.kind === 'nose' || top) {
        const coneLen = Math.min(section.length, r * 2.6);
        // Тот же признак семейства, что и у плиток: тупой купол — примета
        // корабля Starship, у обтекателя/носа прочих носителей (Falcon 9
        // и т.п.) профиль заметно острее — заострённый оживал, а не купол.
        // Без этого различия силуэт читался как Starship даже без плавников
        // и плиток: форма носа — самая узнаваемая часть контура.
        const pointed = !layout.flaps;
        noseFaces(faces, toWorld, x0, x0 - coneLen, r, seg, steelColour, tileColour, tiled, pointed);
        if (section.length > coneLen) {
          hullFaces(faces, toWorld, x0 - coneLen, x1, r, seg, steelColour, tileColour, tiled);
        }
      } else {
        hullFaces(faces, toWorld, x0, x1, r, seg, steelColour, tileColour, tiled);
      }
      x = x1;
    }

    // Двигатели на срезе юбки.
    engineFaces(faces, toWorld, x, stage, t);
  }

  // Плавники корабля.
  //
  // До разделения телеметрия про них молчит: ими никто не управляет, они
  // прижаты к борту. Но стоят они на корабле с самого старта, и не рисовать
  // их на выведении — значит показывать не ту машину.
  const flaps = t.flaps?.length ? t.flaps : stowedFlaps(layout);
  if (flaps.length) flapFaces(faces, toWorld, { ...t, flaps }, layout, total);

  // Свечение при входе.
  plasmaFaces(faces, toWorld, t, layout.diameter / 2, total);
}

/* Цвет стороны по её температуре: холодная сталь серая, раскалённая
   светится. Числа берутся из телеметрии теплозащиты. */
function shieldColour(side, base) {
  if (!side) return base;
  const hot = Math.max(0, Math.min(1, (side.temperature - 700) / 1100));
  return [
    base[0] + (255 - base[0]) * hot,
    base[1] * (1 - hot * 0.45) + 90 * hot,
    base[2] * (1 - hot * 0.75) + 40 * hot,
  ];
}

/* Обечайка между двумя кольцами. Наветренная половина корабля закрыта
   плитками, подветренная — голая сталь. */
function hullFaces(faces, toWorld, x0, x1, r, seg, steel, tiles, tiled) {
  const a = ring(x0, r, seg), b = ring(x1, r, seg);
  for (let i = 0; i < seg; i++) {
    const j = (i + 1) % seg;
    // Плитки лежат на стороне брюха: там, где составляющая по +Z
    // положительна.
    const belly = Math.cos(a[i].a) > 0.05;
    faces.push({
      p: [b[i], b[j], a[j], a[i]].map(toWorld),
      c: tiled && belly ? tiles : steel,
    });
  }
}

/* Носовая часть: конус со скруглением. */
function noseFaces(faces, toWorld, x0, x1, r, seg, steel, tiles, tiled, pointed) {
  const steps = 5;
  let prev = ring(x1, r, seg);
  for (let s = 1; s <= steps; s++) {
    const k = s / steps;
    const x = x1 + (x0 - x1) * k;
    // Купол Starship: радиус падает как корень, тупой профиль. У носителей
    // с заострённым обтекателем (pointed) — оживал ближе к прямому конусу,
    // с лёгкой выпуклостью, а не полукруглый купол.
    const rr = pointed
      ? r * Math.pow(Math.max(0, 1 - k), 0.85)
      : r * Math.sqrt(Math.max(0, 1 - k * k));
    const cur = ring(x, Math.max(rr, 0.05), seg);
    for (let i = 0; i < seg; i++) {
      const j = (i + 1) % seg;
      const belly = Math.cos(prev[i].a) > 0.05;
      faces.push({
        p: [cur[i], cur[j], prev[j], prev[i]].map(toWorld),
        c: tiled && belly ? tiles : steel,
      });
    }
    prev = cur;
  }
}

/* Сопла и факелы. */
function engineFaces(faces, toWorld, xBase, stage, t) {
  const total = stage.engines?.length || 0;
  if (!total) return;

  // Телеметрия называет состояние и тягу каждого двигателя по имени —
  // раньше горели первые N по списку просто потому, что их число совпадало
  // со счётчиком enginesRunning; теперь берётся, что реально горит
  // и с какой тягой, а не порядковый номер в развёртке.
  const byID = new Map();
  if (t.stage === stage.index) {
    for (const e of t.engines || []) byID.set(e.id, e);
  }

  stage.engines.forEach(e => {
    const rr = e.exitRadius;
    const cx = e.x, cy = e.y;
    const seg = 8;

    const mouth = [], throat = [];
    for (let i = 0; i < seg; i++) {
      const a = (i / seg) * Math.PI * 2;
      mouth.push({ x: xBase, y: cx + rr * Math.sin(a), z: cy + rr * Math.cos(a) });
      throat.push({ x: xBase + rr * 1.6, y: cx + rr * 0.35 * Math.sin(a), z: cy + rr * 0.35 * Math.cos(a) });
    }
    for (let i = 0; i < seg; i++) {
      const j = (i + 1) % seg;
      faces.push({
        p: [throat[i], throat[j], mouth[j], mouth[i]].map(toWorld),
        c: [70, 74, 80],
      });
    }

    const data = byID.get(e.id);
    if (!data || !data.running || !(data.thrust > 0)) return;

    // Длина и яркость факела — от доли паспортной тяги, которую реально
    // выдаёт именно этот двигатель, а не от общей уставки на ступень:
    // форсированный на компенсации отказа собрат честно горит длиннее
    // соседей, а не одинаково со всеми остальными.
    const ratio = data.maxThrust > 0 ? data.thrust / data.maxThrust : 0;
    const len = rr * (6 + 26 * ratio);
    const tip = { x: xBase - len, y: cx, z: cy };
    for (let i = 0; i < seg; i++) {
      const j = (i + 1) % seg;
      faces.push({
        p: [mouth[i], mouth[j], tip].map(toWorld),
        c: [255, 190, 120], glow: 0.5 + 0.4 * Math.min(ratio, 1),
      });
    }
  });
}

/* Плавники.

   Панель жёсткая. Это главное, что нужно про неё знать, и ровно это картинка
   раньше врала: прижатый плавник рисовался согнутым по борту, звеньями вокруг
   корпуса. Настоящий плавник — полый стальной клин на трёх неподвижных петлях,
   он не гнётся ни на градус. Прижатым он лежит вровень с обшивкой, а щель
   между ним и корпусом закрыта отдельным неподвижным обтекателем — на корабле
   эта деталь называется static aero и видна всегда, даже когда плавник убран.

   Из-за той же выдумки врал и предел хода. Раскрытое положение задавалось не
   углом, а точкой «поперёк борта», и путь до неё выходил куда длиннее хода
   привода: на предельном отклонении панель уезжала за перпендикуляр и
   заваливалась обратно на корпус. Вылет от оси при этом сначала рос, а после
   пятидесяти градусов начинал падать — то есть «раскрытый» плавник выглядел
   сложенным. Теперь угол и есть угол: панель поворачивается вокруг оси навески
   ровно на то, что показывает телеметрия.

   Размах и хорда по-прежнему выводятся из площади, по которой модель считает
   силу. А место на обводе больше не выдумывается: оно приходит развёрткой,
   из той же функции, по которой собраны управляющие поверхности.
   -------------------------------------------------------------------------- */

/* Прижатые к борту плавники корабля — те, что стоят на нём до разделения.
   Собираются по привязкам развёртки, поэтому площади и ход привода тут
   не свои, а модельные. */
function stowedFlaps(layout) {
  if (!layout?.flaps) return [];

  return (layout.flapMounts || []).map(m => ({
    name: m.name, deflection: 0, command: 0, limit: m.limit, area: m.area,
  }));
}

function flapFaces(faces, toWorld, t, layout, total) {
  const r = layout.diameter / 2;
  const mounts = layout.flapMounts || [];
  if (!mounts.length) return;

  const ship = t.stage >= 2
    ? total
    : (layout.stages.find(s => s.index === 2)?.length || total * 0.4);

  // Станции привязок отсчитаны от среза сопел корабля. Пока ступеней две,
  // корабль сидит наверху носителя, и весь набор надо поднять на его низ.
  const base = total - ship;
  const rad = Math.PI / 180;

  for (const f of t.flaps) {
    const mount = mounts.find(m => m.name === f.name);
    if (!mount) continue;

    const hand = mount.azimuth < 0 ? -1 : 1;
    const az = mount.azimuth * rad;
    const half = mount.stowHalf * rad;       // половина закрытой дуги
    const span = mount.span;

    // Хорда меняется по размаху: у корня шире, к концу уже. Средняя хорда
    // приходит из привязки, по ней же модель считает площадь.
    const rootChord = mount.chord * 1.35;
    const tipChord = mount.chord * 0.65;
    const sweep = rootChord * 0.35;          // наклон передней кромки

    // Толщина: у корня панель заметно толще, к концу сходит почти на нет.
    // Настоящий плавник — полый стальной клин, и без толщины он читается
    // как лист бумаги.
    const rootThick = Math.max(0.35, mount.chord * 0.16);
    const tipThick = rootThick * 0.3;

    const lead = base + mount.station + rootChord * 0.5;

    // Прижатая панель касается обшивки серединой. Значит, ось навески стоит
    // не на обводе, а выше — ровно настолько, чтобы её конец пришёлся туда же.
    // Разница и есть высота прилива под корнем: у настоящего корабля прижатый
    // плавник тоже не заподлицо, а горбом на борту.
    const rh = r / Math.cos(half);
    const hy = rh * Math.sin(az);
    const hz = -rh * Math.cos(az);

    // Азимут середины прижатой панели — точки касания.
    const mid = az - hand * half;

    // Прижатое положение — касательная в точке касания, уводящая к спине.
    // Раскрытие поворачивает панель вокруг оси навески наружу ровно на угол
    // отклонения: тот же поворот, каким модель считает нормаль панели.
    const stowed = mid + (hand > 0 ? Math.PI : 0);
    const dir = stowed + hand * (f.deflection || 0) * rad;

    const along = { y: Math.cos(dir), z: Math.sin(dir) };
    // Нормаль панели — поперёк размаха в той же плоскости шпангоута.
    const norm = { y: -hand * along.z, z: hand * along.y };

    const colour = f.jammed ? [150, 60, 55] : f.manual ? [90, 120, 170] : [150, 158, 168];

    // Узел панели: точка на расстоянии s от оси навески, со сдвигом
    // по толщине и по хорде.
    const node = (s, thick, back) => ({
      x: lead - sweep * (s / span) - back,
      y: hy + along.y * s + norm.y * thick,
      z: hz + along.z * s + norm.z * thick,
    });

    const c0 = rootChord, c1 = tipChord;
    const h0 = rootThick / 2, h1 = tipThick / 2;

    // Четыре грани клина: две плоскости и две кромки. Торцы не рисуются —
    // корневой прикрыт обтекателем, концевой на глаз не читается.
    const quad = (a, b, c, d, shade) => faces.push({
      p: [a, b, c, d].map(toWorld),
      c: colour.map(v => Math.round(v * shade)), twoSided: true,
    });

    quad(node(0, h0, 0), node(0, h0, c0), node(span, h1, c1), node(span, h1, 0), 1.0);
    quad(node(0, -h0, 0), node(0, -h0, c0), node(span, -h1, c1), node(span, -h1, 0), 0.72);
    quad(node(0, h0, 0), node(span, h1, 0), node(span, -h1, 0), node(0, -h0, 0), 0.88);
    quad(node(0, h0, c0), node(span, h1, c1), node(span, -h1, c1), node(0, -h0, c0), 0.62);

    // Прилив под корнем — на корабле его называют static aero.
    //
    // Это не украшение: он закрывает щель между бортом и приподнятой осью
    // навески и прикрывает саму навеску от плазмы. Стоит он неподвижно,
    // поэтому рисуется одинаково при любом отклонении и остаётся на месте,
    // когда панель убрана.
    const steps = 4;
    for (let i = 0; i < steps; i++) {
      const a0 = az + hand * half * (i / steps);
      const a1 = az + hand * half * ((i + 1) / steps);

      // Наружная кромка идёт от оси навески вниз к обшивке.
      const lift0 = 1 - i / steps, lift1 = 1 - (i + 1) / steps;
      const edge = (a, lift) => ({
        y: (r + (rh - r) * lift) * Math.sin(a),
        z: -(r + (rh - r) * lift) * Math.cos(a),
      });

      const e0 = edge(a0, lift0), e1 = edge(a1, lift1);
      const s0 = { y: r * Math.sin(a0), z: -r * Math.cos(a0) };
      const s1 = { y: r * Math.sin(a1), z: -r * Math.cos(a1) };

      faces.push({
        p: [
          { x: lead, y: e0.y, z: e0.z },
          { x: lead - rootChord, y: e0.y, z: e0.z },
          { x: lead - rootChord, y: e1.y, z: e1.z },
          { x: lead, y: e1.y, z: e1.z },
        ].map(toWorld),
        c: [120, 128, 138], twoSided: true,
      });
      faces.push({
        p: [
          { x: lead, y: e0.y, z: e0.z },
          { x: lead, y: e1.y, z: e1.z },
          { x: lead, y: s1.y, z: s1.z },
          { x: lead, y: s0.y, z: s0.z },
        ].map(toWorld),
        c: [104, 112, 122], twoSided: true,
      });
    }
  }
}

/* Свечение при входе.

   Это не «эффект», а показание: светится ударный слой перед наветренной
   стороной, и ровно там, где модель считает нагрев. Порог поставлен по делу —
   заметное свечение начинается, когда поток идёт сотнями киловатт с квадрата,
   а не с первых ватт на высоте ста километров. Прозрачность растёт круче
   линейной: глазу разница между 0,2 и 1,2 МВт/м² должна быть очевидна. */
function plasmaFaces(faces, toWorld, t, r, total) {
  const flux = t.heatFlux || 0;
  if (flux < 1.5e5) return;

  const heat = Math.min(1, flux / 1.2e6);
  const alpha = 0.06 + 0.34 * heat * heat;

  const seg = 12;
  const off = r * (0.25 + 0.45 * heat);
  const arc = Math.PI * 0.62; // дуга вокруг брюха, без спины

  const a = [], b = [];
  for (let i = 0; i <= seg; i++) {
    const ang = -arc / 2 + (i / seg) * arc;
    const rr = r + off;
    // Оболочка тянется вдоль борта: от середины вверх и вниз, считая
    // от среза сопел.
    a.push({ x: total * 0.80, y: rr * Math.sin(ang), z: rr * Math.cos(ang) });
    b.push({ x: total * 0.08, y: rr * Math.sin(ang), z: rr * Math.cos(ang) });
  }
  for (let i = 0; i < seg; i++) {
    faces.push({
      p: [a[i], a[i + 1], b[i + 1], b[i]].map(toWorld),
      c: [255, 170 - 70 * heat, 90],
      glow: alpha,
      twoSided: true,
    });
  }
}

/* --------------------------------------------------------------------------
   Планета

   Земля строится как то, чем она и является: видимая шапка сферы. Центр
   планеты лежит под кораблём на расстоянии радиуса плюс высота, и от него
   набираются точки поверхности вплоть до горизонта — угла, на котором луч
   зрения касается шара.

   Одна и та же формула даёт и площадку под опорами, и изогнутый край планеты
   с орбиты: на километре горизонт уходит на сотню километров и поверхность
   выглядит плоской, с двухсот километров в кадр попадает дуга. Прежде здесь
   была проволочная сетка, из-за которой корабль висел в пустоте и казался
   прозрачным — сквозь него было видно линии.
   -------------------------------------------------------------------------- */

const EARTH_RADIUS = 6371000;

function planetFaces(faces, t) {
  const alt = Math.max(t.altitude || 0, 0.5);
  const R = EARTH_RADIUS;

  // Центр планеты в местном горизонте: точно под кораблём.
  const centre = v3(0, 0, -(R + alt));

  // Угол до горизонта. Дальше поверхность скрыта самим шаром.
  const horizon = Math.acos(Math.min(1, R / (R + alt)));

  const rings = 9, sectors = 28;
  const point = (theta, phi) => {
    const st = Math.sin(theta), ct = Math.cos(theta);
    return vAdd(centre, v3(R * st * Math.cos(phi), R * st * Math.sin(phi), R * ct));
  };

  // Кольца идут в геометрической прогрессии: у ног клетка в десятки метров,
  // к горизонту — в десятки километров.
  //
  // Равномерное деление здесь не годится: первое кольцо занимало бы километры
  // и вырождалось в звезду из треугольников, сходящихся в точке под кораблём.
  // Ровно так и выглядела земля — веером.
  const inner = Math.max(40 / R, horizon * 1e-3);
  const ringAngle = i => (i === 0 ? 0
    : inner * Math.pow(horizon / inner, (i - 1) / (rings - 1)));

  for (let i = 0; i < rings; i++) {
    const t0 = ringAngle(i), t1 = ringAngle(i + 1);

    for (let j = 0; j < sectors; j++) {
      const p0 = (j / sectors) * Math.PI * 2;
      const p1 = ((j + 1) / sectors) * Math.PI * 2;

      // Клетка: чередование даёт масштаб, не превращая землю в сетку.
      const even = (i + j) % 2 === 0;
      const shade = even ? 1 : 0.88;

      // К горизонту поверхность уходит в дымку.
      const haze = Math.pow(i / rings, 2);
      const base = [
        (46 * shade) * (1 - haze) + 96 * haze,
        (62 * shade) * (1 - haze) + 118 * haze,
        (52 * shade) * (1 - haze) + 150 * haze,
      ];

      faces.push({
        p: [point(t0, p0), point(t0, p1), point(t1, p1), point(t1, p0)],
        c: base, flat: true, ground: true,
      });
    }
  }

  // Свечение атмосферы над краем планеты.
  //
  // Видно его только оттуда, откуда виден сам край: с земли атмосфера — это
  // небо над головой, а не полоска у горизонта, и рисовать её там значит
  // класть синий прямоугольник поперёк кадра.
  if (alt < 20000) return;

  const glowHeight = 60000 * Math.min(1, 0.2 + alt / 200000);
  for (let j = 0; j < sectors; j++) {
    const p0 = (j / sectors) * Math.PI * 2;
    const p1 = ((j + 1) / sectors) * Math.PI * 2;

    const lift = u => {
      const d = vSub(u, centre);
      return vAdd(centre, vMul(vUnit(d), R + glowHeight));
    };

    const a0 = point(horizon, p0), a1 = point(horizon, p1);
    faces.push({
      p: [a0, a1, lift(a1), lift(a0)],
      c: [120, 170, 255], glow: 0.16, twoSided: true,
    });
  }
}

/* Разметка под кораблём: по ней читается высота и снос.

   Рисуется линиями поверх поверхности — заливкой такую мелкую клетку
   не покажешь, а без неё на однотонной земле не видно ни снижения,
   ни сноса. */
function drawGroundGrid(ctx, cam, t) {
  const alt = t.altitude || 0;
  if (alt > 20000) return;

  const z = -Math.max(alt, 0.5);
  const step = alt < 400 ? 20 : alt < 4000 ? 200 : 2000;
  const half = 10;
  const fade = 1 - Math.min(1, alt / 20000);

  ctx.strokeStyle = `rgba(190,220,240,${0.08 + 0.22 * fade})`;
  ctx.lineWidth = 1;

  const line = (a, b) => {
    const p = project(a, cam), q = project(b, cam);
    if (!p || !q) return;
    ctx.beginPath();
    ctx.moveTo(p.x, p.y);
    ctx.lineTo(q.x, q.y);
    ctx.stroke();
  };

  for (let i = -half; i <= half; i++) {
    line(v3(i * step, -half * step, z), v3(i * step, half * step, z));
    line(v3(-half * step, i * step, z), v3(half * step, i * step, z));
  }

  // Точка под кораблём: видно, куда он идёт.
  const p = project(v3(0, 0, z), cam);
  if (p) {
    ctx.strokeStyle = `rgba(255,190,120,${0.3 + 0.5 * fade})`;
    ctx.beginPath();
    ctx.arc(p.x, p.y, 4, 0, Math.PI * 2);
    ctx.stroke();
  }
}

/* --------------------------------------------------------------------------
   Отрисовка граней
   -------------------------------------------------------------------------- */

function drawFaces(ctx, faces, cam) {
  const light = vUnit(v3(0.35, 0.55, 0.75));
  const ready = [];

  for (const face of faces) {
    const pts = face.p.map(p => project(p, cam));
    if (pts.some(p => !p)) continue;

    const n = vUnit(vCross(vSub(face.p[1], face.p[0]), vSub(face.p[2], face.p[0])));
    let lam = vDot(n, light);
    if (face.twoSided || face.glow) lam = Math.abs(lam);

    // Задняя сторона корпуса не рисуется: тело выпуклое, и без этого
    // дальняя половина обечайки лезла бы поверх ближней.
    if (!face.twoSided && !face.glow && !face.flat) {
      if (vDot(n, vSub(cam.pos, face.p[0])) <= 0) continue;
    }

    // За кадром рисовать нечего: грани у самой камеры проецируются
    // на десятки тысяч пикселей и только тратят время.
    const minX = Math.min(...pts.map(p => p.x)), maxX = Math.max(...pts.map(p => p.x));
    const minY = Math.min(...pts.map(p => p.y)), maxY = Math.max(...pts.map(p => p.y));
    if (maxX < 0 || minX > cam.cx * 2 || maxY < 0 || minY > cam.cy * 2) continue;

    // Освещение: солнце плюс слабая подсветка со стороны камеры. Без второй
    // корпус на теневой стороне превращается в чёрный силуэт, и вся картинка
    // теряет смысл ровно там, где она нужнее всего — на ночной стороне витка.
    const toCam = vUnit(vSub(cam.pos, face.p[0]));
    const head = Math.abs(vDot(n, toCam));
    const shade = face.flat ? 1 : 0.42 + 0.48 * Math.max(0, lam) + 0.18 * head;
    ready.push({
      pts,
      z: pts.reduce((s, p) => s + p.z, 0) / pts.length,
      c: face.c, shade, glow: face.glow, ground: face.ground,
    });
  }

  // Планета рисуется первой при любой глубине.
  //
  // Иначе клетка поверхности, оказавшаяся ближе к камере, чем середина
  // корпуса, ложится поверх корабля: сортировка по средней глубине для
  // объектов такой разной величины не работает. Земля — фон, и место у неё
  // всегда сзади.
  ready.sort((a, b) => {
    if (!!a.ground !== !!b.ground) return a.ground ? -1 : 1;
    return b.z - a.z;
  });

  for (const f of ready) {
    ctx.beginPath();
    ctx.moveTo(f.pts[0].x, f.pts[0].y);
    for (let i = 1; i < f.pts.length; i++) ctx.lineTo(f.pts[i].x, f.pts[i].y);
    ctx.closePath();

    const c = f.c.map(v => Math.round(Math.max(0, Math.min(255, v * f.shade))));
    if (f.glow) {
      ctx.globalCompositeOperation = 'lighter';
      ctx.globalAlpha = f.glow;
      ctx.fillStyle = `rgb(${c[0]},${c[1]},${c[2]})`;
      ctx.fill();
      ctx.globalAlpha = 1;
      ctx.globalCompositeOperation = 'source-over';
    } else {
      ctx.fillStyle = `rgb(${c[0]},${c[1]},${c[2]})`;
      ctx.fill();
      ctx.strokeStyle = 'rgba(0,0,0,0.35)';
      ctx.lineWidth = 0.6;
      ctx.stroke();
    }
  }
}

/* Стрелки: набегающий поток и вектор скорости. */
function drawArrows(ctx, cam, t) {
  // Стрелки выходят из середины корпуса — из точки, на которую смотрит камера.
  const origin = cam.target || v3(0, 0, 0);

  const arrow = (dir, len, colour, label) => {
    if (vLen(dir) < 0.5) return;
    const from = vAdd(origin, vMul(dir, -len));
    const a = project(from, cam), b = project(origin, cam);
    if (!a || !b) return;

    ctx.strokeStyle = colour;
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(a.x, a.y);
    ctx.lineTo(b.x, b.y);
    ctx.stroke();

    ctx.fillStyle = colour;
    ctx.font = '11px ui-monospace, monospace';
    ctx.fillText(label, a.x + 6, a.y - 4);
  };

  arrow(fromScene(t.scene?.airflow), S3.cam.dist * 0.55, 'rgba(120,190,255,0.85)', 'поток');
  arrow(vMul(fromScene(t.scene?.velocity), -1), S3.cam.dist * 0.40, 'rgba(150,255,170,0.7)', 'скорость');
}

/* --------------------------------------------------------------------------
   Мини-глобус

   Ортографическая проекция с точки, над которой корабль сейчас находится:
   видна половина планеты, на ней — пройденная трасса и текущее место.
   -------------------------------------------------------------------------- */

function collectTrack(t) {
  // Новый прогон — новый след.
  if (t.time < S3.lastTime) S3.track = [];
  S3.lastTime = t.time;

  const last = S3.track[S3.track.length - 1];
  if (!last || Math.abs(last.lat - t.lat) > 0.02 || Math.abs(last.lon - t.lon) > 0.02) {
    S3.track.push({ lat: t.lat, lon: t.lon });
    if (S3.track.length > 4000) S3.track.shift();
  }
}

function drawGlobe(t) {
  const ctx = S3.globeCtx;
  if (!ctx) return;

  const canvas = S3.globe;
  const dpr = window.devicePixelRatio || 1;
  const w = canvas.clientWidth || 180, h = canvas.clientHeight || 180;
  if (canvas.width !== Math.round(w * dpr)) {
    canvas.width = Math.round(w * dpr);
    canvas.height = Math.round(h * dpr);
  }
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, w, h);

  const cx = w / 2, cy = h / 2, R = Math.min(w, h) / 2 - 14;
  const lat0 = t.lat * Math.PI / 180, lon0 = t.lon * Math.PI / 180;

  // Планета.
  ctx.beginPath();
  ctx.arc(cx, cy, R, 0, Math.PI * 2);
  ctx.fillStyle = '#16283c';
  ctx.fill();
  ctx.strokeStyle = '#2f4a68';
  ctx.stroke();

  const to = (lat, lon) => {
    const p = lat * Math.PI / 180, l = lon * Math.PI / 180;
    const cosc = Math.sin(lat0) * Math.sin(p) +
      Math.cos(lat0) * Math.cos(p) * Math.cos(l - lon0);
    if (cosc < 0) return null; // обратная сторона планеты
    return {
      x: cx + R * Math.cos(p) * Math.sin(l - lon0),
      y: cy - R * (Math.cos(lat0) * Math.sin(p) -
        Math.sin(lat0) * Math.cos(p) * Math.cos(l - lon0)),
    };
  };

  // Сетка через тридцать градусов.
  ctx.strokeStyle = 'rgba(120,160,200,0.18)';
  ctx.lineWidth = 1;
  for (let lat = -60; lat <= 60; lat += 30) {
    ctx.beginPath();
    let started = false;
    for (let lon = -180; lon <= 180; lon += 5) {
      const p = to(lat, lon);
      if (!p) { started = false; continue; }
      if (!started) { ctx.moveTo(p.x, p.y); started = true; } else ctx.lineTo(p.x, p.y);
    }
    ctx.stroke();
  }
  for (let lon = -180; lon < 180; lon += 30) {
    ctx.beginPath();
    let started = false;
    for (let lat = -90; lat <= 90; lat += 5) {
      const p = to(lat, lon);
      if (!p) { started = false; continue; }
      if (!started) { ctx.moveTo(p.x, p.y); started = true; } else ctx.lineTo(p.x, p.y);
    }
    ctx.stroke();
  }

  // Трасса.
  ctx.strokeStyle = 'rgba(255,180,90,0.9)';
  ctx.lineWidth = 1.6;
  ctx.beginPath();
  let started = false;
  for (const p of S3.track) {
    const q = to(p.lat, p.lon);
    if (!q) { started = false; continue; }
    if (!started) { ctx.moveTo(q.x, q.y); started = true; } else ctx.lineTo(q.x, q.y);
  }
  ctx.stroke();

  // Корабль: высота показана вынесенной точкой, иначе на глобусе
  // двести километров — это меньше пикселя.
  const lift = R * (1 + Math.min(0.35, (t.altitude || 0) / 400000 * 0.3));
  ctx.beginPath();
  ctx.arc(cx, cy - lift + R, 3.5, 0, Math.PI * 2);
  ctx.fillStyle = '#ffd479';
  ctx.fill();

  ctx.beginPath();
  ctx.moveTo(cx, cy);
  ctx.lineTo(cx, cy - lift + R);
  ctx.strokeStyle = 'rgba(255,212,121,0.5)';
  ctx.stroke();

  ctx.fillStyle = 'rgba(200,215,230,0.75)';
  ctx.font = '10px ui-monospace, monospace';
  ctx.fillText(`${num(t.lat, 2)}° ${num(t.lon, 2)}°`, 6, h - 6);
}

/* --------------------------------------------------------------------------
   Подписи
   -------------------------------------------------------------------------- */

function hudLines(t) {
  const flap = (t.flaps || []).map(f => `${num(f.deflection, 0)}°`).join(' ');
  const shield = t.heatShield;

  const lines = [
    `${t.phase} · T+${num(t.time, 0)} с · ступень ${t.stage}`,
    `высота ${num((t.altitude || 0) / 1000, 1)} км · скорость ${num(t.totalVelocity, 0)} м/с · ` +
    `M ${num(t.mach, 1)}`,
    `угол атаки ${num(t.totalAoA, 1)}° · напор ${num((t.dynamicPressure || 0) / 1000, 2)} кПа · ` +
    `поток ${num((t.heatFlux || 0) / 1000, 0)} кВт/м²`,
  ];

  const limit = t.flaps?.[0]?.limit;
  if (flap) {
    lines.push(`плавники ${flap} (0° — прижат` +
      (limit ? `, ${num(limit, 0)}° — раскрыт)` : ')'));
  }
  if (shield) {
    lines.push(`плитки ${num(shield.tiles.temperature, 0)} К · ` +
      `сталь ${num(shield.steel.temperature, 0)} К · ` +
      `брюхо к потоку ${num(shield.exposure, 2)}`);
  }
  if ((t.altitude || 0) > 100000 && (t.dynamicPressure || 0) < 1) {
    lines.push('вне атмосферы: тормозить и греться нечем');
  }

  // Итог посадки: то, ради чего весь спуск и затевался.
  const l = t.landing;
  if (l) {
    lines.push(l.intact
      ? `СЕЛ: касание ${num(l.speed, 1)} м/с при пределе ${num(l.limit, 0)}, ` +
        `отклонение от вертикали ${num(l.tilt, 1)}°, остаток ${num(l.fuelLeft / 1000, 1)} т`
      : l.toppled
        ? `ЗАВАЛИЛСЯ: касание ${num(l.speed, 1)} м/с было мягким, но корпус ` +
          `отклонён на ${num(l.tilt, 0)}°`
        : `РАЗБИЛСЯ: касание ${num(l.speed, 1)} м/с при пределе ${num(l.limit, 0)} м/с`);
  }
  return lines;
}

function hudText(ctx, w, h, lines) {
  ctx.font = '12px ui-monospace, monospace';
  ctx.textBaseline = 'top';

  const pad = 8;
  const width = Math.max(...lines.map(l => ctx.measureText(l).width)) + pad * 2;
  ctx.fillStyle = 'rgba(8,12,18,0.55)';
  ctx.fillRect(8, 8, width, lines.length * 16 + pad * 2 - 4);

  ctx.fillStyle = '#c8d6e4';
  lines.forEach((l, i) => ctx.fillText(l, 8 + pad, 8 + pad + i * 16));
}

/* Наружу выходит только то, чем пользуются другие: отрисовка кадра и её
   запуск. Состояние вкладки и построение плавников открыты для стенда —
   на них есть проверки, и заводить ради этого отдельный способ достучаться
   до внутренностей было бы хуже. */
window.drawScene3D = drawScene3D;
window.onSceneFrame = onSceneFrame;
window.initScene3D = initScene3D;
window.S3 = S3;
window.flapFaces = flapFaces;

/* Внутренности для стенда — под своим именем.
   Складывать их прямо в глобальную область нельзя: project() здесь и project()
   в app.js — разные функции, и одна затрёт другую. На этом уже спотыкались. */
window.__scene = { planetFaces, project, vehicleFaces, blendFrames };

})();

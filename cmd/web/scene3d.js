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
  // fit — дальность камеры отдельно для корабля и для бустера (режимы
  // «Корабль»/«Бустер», см. renderBody): у одной и той же дальности
  // подгонка под пятидесятиметровый корабль тут же сбивается подгонкой
  // под семидесятиметровый бустер и наоборот. Режим «Оба» (renderBoth)
  // этими слотами не пользуется — там дальность считается заново каждый
  // раз под оба тела сразу, см. buildComboCamera.
  fit: { ship: { key: null, base: null }, booster: { key: null, base: null } },
  drag: null,
  track: [],        // след трассы корабля на мини-глобусе, широта и долгота
  lastTime: -1,
  ready: false,
  showGrid: true,
  showFlow: true,
  followFlow: false,
  bodyCam: true,    // камера привязана к корпусу, а не к горизонту

  // cameraTarget — что показывать: 'ship' | 'booster' | 'both'. Переключается
  // вкладками #scene-target (см. initScene3D/renderSceneTargetTabs), не
  // связано с #vehicle-tabs в app.js (тот выбирает адресата команд,
  // а не то, что рисует камера, — вещи разные: можно управлять кораблём,
  // разглядывая при этом бустер).
  cameraTarget: 'ship',

  // Башня-ловушка: геометрия с /api/catch/tower и признак того, что камера
  // уже развёрнута вдоль рук (см. buildCatchCamera).
  tower: null,
  catchCam: false,

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
  loadTower();
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
    S3.fit.ship.key = null;
    S3.fit.booster.key = null;
    drawScene3D();
  };

  // Делегированный обработчик: сам #scene-target не пересоздаётся, кнопки
  // внутри — каждый кадр (см. renderSceneTargetTabs), и обычный onclick
  // на кнопке норовил потеряться между нажатием и отпусканием (та же
  // гонка, что уже лечили у #vehicle-tabs и #engine-strip в app.js).
  const targetNav = document.getElementById('scene-target');
  if (targetNav) targetNav.addEventListener('pointerdown', e => {
    const btn = e.target.closest('[data-target]');
    if (btn && !btn.disabled) S3.cameraTarget = btn.dataset.target;
  });

  S3.ready = true;
}

const SCENE_TARGETS = [
  { id: 'ship', title: 'Корабль' },
  { id: 'booster', title: 'Бустер' },
  { id: 'both', title: 'Оба' },
];

/* Перерисовывает вкладки выбора тела — каждый кадр, а не один раз при
   инициализации: доступность «Бустер»/«Оба» зависит от того, есть ли он
   сейчас (после отделения), а это меняется по ходу полёта. */
function renderSceneTargetTabs(haveBooster) {
  const nav = document.getElementById('scene-target');
  if (!nav) return;
  for (const target of SCENE_TARGETS) {
    let btn = nav.querySelector(`[data-target="${target.id}"]`);
    if (!btn) {
      btn = document.createElement('button');
      btn.className = 'tab';
      btn.dataset.target = target.id;
      btn.textContent = target.title;
      nav.appendChild(btn);
    }
    const disabled = target.id !== 'ship' && !haveBooster;
    btn.disabled = disabled;
    btn.title = disabled
      ? 'Бустер сейчас не летит: носитель без активного возврата либо ступени ещё не разделились'
      : '';
    btn.classList.toggle('active', S3.cameraTarget === target.id);
  }
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

  const toScene = u => ({ e: u.x, n: u.y, u: u.z });

  // Вектор на башню — смещение в метрах, а не направление: складывается
  // покомпонентно и НЕ нормируется, иначе башня уехала бы на единичное
  // расстояние от корпуса.
  const offset = (x, y) => x && y
    ? { e: mix(x.e, y.e), n: mix(x.n, y.n), u: mix(x.u, y.u) }
    : (y || x);

  const sa = a.scene, sb = b.scene;
  if (sa && sb) {
    out.scene = {
      forward: toScene(vec(sa.forward, sb.forward)),
      right: toScene(vec(sa.right, sb.right)),
      down: toScene(vec(sa.down, sb.down)),
      airflow: toScene(vec(sa.airflow, sb.airflow)),
      velocity: toScene(vec(sa.velocity, sb.velocity)),
      downrange: mix(sa.downrange, sb.downrange),
      tower: offset(sa.tower, sb.tower),
    };
  }

  if (a.flaps && b.flaps && a.flaps.length === b.flaps.length) {
    out.flaps = b.flaps.map((f, i) => ({
      ...f, deflection: mix(a.flaps[i].deflection, f.deflection),
    }));
  }

  // Бустер теперь рисуется каждый кадр наравне с кораблём (см. renderBody
  // в renderScene), а не только пока за ним следит камера, — без своей
  // интерполяции его отрисовка дёргалась бы между снимками там, где
  // корабль уже плавно скользит.
  const ba = a.booster, bb = b.booster;
  if (ba && bb) {
    out.booster = { ...bb };
    out.booster.altitude = mix(ba.altitude, bb.altitude);
    out.booster.totalVelocity = mix(ba.totalVelocity, bb.totalVelocity);
    out.booster.verticalVelocity = mix(ba.verticalVelocity, bb.verticalVelocity);

    const bsa = ba.scene, bsb = bb.scene;
    if (bsa && bsb) {
      out.booster.scene = {
        forward: toScene(vec(bsa.forward, bsb.forward)),
        right: toScene(vec(bsa.right, bsb.right)),
        down: toScene(vec(bsa.down, bsb.down)),
        airflow: toScene(vec(bsa.airflow, bsb.airflow)),
        velocity: toScene(vec(bsa.velocity, bsb.velocity)),
        downrange: mix(bsa.downrange, bsb.downrange),
        tower: offset(bsa.tower, bsb.tower),
      };
    }

    if (ba.gridFins && bb.gridFins && ba.gridFins.length === bb.gridFins.length) {
      out.booster.gridFins = bb.gridFins.map((f, i) => ({
        ...f, deflection: mix(ba.gridFins[i].deflection, f.deflection),
      }));
    }
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

  // Что показывать, решает оператор через #scene-target (см. initScene3D)
  // — «Корабль» / «Бустер» / «Оба», хранится в S3.cameraTarget. «Бустер»
  // и «Оба» сами откатываются на корабль, пока бустера нет: смотреть там
  // не на что, а не показывать вовсе — хуже, чем молча остаться на корабле.
  const haveBooster = !!t.booster;
  const mode = haveBooster ? S3.cameraTarget : 'ship';
  renderSceneTargetTabs(haveBooster);

  let cam;
  if (mode === 'both') {
    cam = renderBoth(ctx, 0, 0, w, h, t);
  } else if (mode === 'booster') {
    cam = renderBody(ctx, 0, 0, w, h, boosterFrame(t), 1, boosterHudLines(t.booster, t));
  } else {
    cam = renderBody(ctx, 0, 0, w, h, t, undefined, hudLines(t));
  }
  if (window.__scene) window.__scene.cam = cam;

  // Мини-глобус всегда следит за кораблём — маршрут возврата бустера
  // за десятки-сотни километров от него всё равно не поместился бы
  // на той же проекции без отдельной перерисовки под второй трек,
  // а корабль здесь главное тело.
  collectTrack(t);
  drawGlobe(t);
}

/* Один аппарат в своей прямоугольной области холста: та же камера/грани/
   HUD, что были в renderScene() до разделения на два тела, только со
   сдвигом и обрезкой под свой прямоугольник (x, y, w, h), а не на весь
   холст. ctx.translate после clip — все внутренние функции отрисовки
   по-прежнему считают, что им принадлежит область от (0,0) до (w,h). */
function renderBody(ctx, x, y, w, h, active, onlyStage, lines) {
  ctx.save();
  ctx.beginPath();
  ctx.rect(x, y, w, h);
  ctx.clip();
  ctx.translate(x, y);

  // Небо перерисовывается на каждое тело своей высотой: корабль на орбите
  // и бустер над самой водой в один и тот же момент — обычное дело после
  // отделения, и общее на двоих небо красило бы половину бустера чернотой
  // космоса, до которого ему как до площадки.
  drawSky(ctx, w, h, active.altitude || 0);

  fitCamera(active);
  // На подходе к площадке камера сама встаёт так, чтобы в кадре были и
  // башня с обеими руками, и подходящий бустер (см. buildCatchCamera).
  const cam = buildCatchCamera(active, w, h) || buildCamera(active, w, h);

  const faces = [];
  if (S3.showGrid) planetFaces(faces, active);
  towerFaces(faces, active);
  vehicleFaces(faces, active, onlyStage);
  drawFaces(ctx, faces, cam);

  if (S3.showGrid) drawGroundGrid(ctx, cam, active);
  drawCatch(ctx, cam, active);
  if (S3.showFlow) drawArrows(ctx, cam, active);

  hudText(ctx, w, h, lines);
  ctx.restore();
  return cam;
}

/* Камера режима «Оба»: не следит за одним корпусом, а держит в кадре сразу
   оба — цель между их условными центрами, дальность подобрана под больший
   из двух корпусов и разнос между ними. Азимут, тангаж и приближение —
   те же ручки оператора (S3.cam, S3.zoom), что и у обычной камеры;
   привязка к грунту и «по потоку» здесь ни при чём — тела стоят не на
   своей истинной высоте, а в общей точке сравнения, грунту в этом кадре
   искать нечего. */
function buildComboCamera(shipT, boosterT, sep, w, h) {
  const yaw = S3.cam.yaw, pitch = S3.cam.pitch;
  const shipLen = vehicleLength(shipT);
  const boosterLen = boosterT ? vehicleLength(boosterT) : shipLen;
  const dist = (sep + Math.max(shipLen, boosterLen) * 0.6) * 1.5 * S3.zoom;
  S3.cam.dist = dist;

  const cp = Math.cos(pitch), sp = Math.sin(pitch);
  const target = v3(0, sep / 2, 0);
  const pos = vAdd(target, v3(
    dist * cp * Math.sin(yaw),
    dist * cp * Math.cos(yaw),
    dist * sp));

  const fwd = vUnit(vSub(target, pos));
  let right = vCross(fwd, v3(0, 0, 1));
  if (vLen(right) < 1e-6) right = v3(1, 0, 0);
  right = vUnit(right);
  const up = vCross(right, fwd);
  return { pos, target, fwd, right, up, cx: w / 2, cy: h / 2, f: h * 1.15 };
}

/* Оба тела в одном кадре — не в истинном взаимном положении (после
   отделения счёт идёт на километры, а то и сотни километров, в кадре
   с различимыми деталями это всё равно не поместилось бы), а нарочно
   разнесены на фиксированное расстояние вдоль мировой оси Y, каждое вокруг
   своего начала координат со своей ориентацией — см. offset у bodyToWorld.
   Ради этого режим и заведён: увидеть, куда развёрнут бустер относительно
   корабля в тот же момент, а не гоняться взглядом за одним из них по
   очереди (что и так умеют режимы «Корабль»/«Бустер»). Сетка земли и
   стрелки потока здесь не рисуются: оба тела стоят не на своей истинной
   высоте, и то, и другое было бы просто неверным. */
function renderBoth(ctx, x, y, w, h, t) {
  ctx.save();
  ctx.beginPath();
  ctx.rect(x, y, w, h);
  ctx.clip();
  ctx.translate(x, y);

  drawSky(ctx, w, h, t.altitude || 0);

  const bt = boosterFrame(t);
  const sep = Math.max(vehicleLength(t), bt ? vehicleLength(bt) : 0) * 1.8;
  const cam = buildComboCamera(t, bt, sep, w, h);

  const faces = [];
  vehicleFaces(faces, t, undefined);
  if (bt) vehicleFaces(faces, bt, 1, v3(0, sep, 0));
  drawFaces(ctx, faces, cam);

  hudText(ctx, w, h, hudLines(t), 'left', w / 2);
  if (bt) hudText(ctx, w, h, boosterHudLines(t.booster, t), 'right', w / 2);

  ctx.restore();
  return cam;
}

/* Кадр бустера в форме, которую понимают функции отрисовки тела: та же
   форма, что и у ship-телеметрии сцены (Scene, altitude, lat/lon), но
   геометрия и оснастка — только первой ступени. */
function boosterFrame(t) {
  const b = t.booster;
  if (!b) return null;
  return {
    time: t.time,
    phase: b.phase,
    stage: 1,
    isBooster: true,
    altitude: b.altitude,
    lat: b.lat,
    lon: b.lon,
    scene: b.scene,
    gridFins: b.gridFins,
    catch: b.catch,
    engines: b.engines || [],
    flaps: [],
    heatShield: null,
  };
}

/* Подписи для бустера — свой набор строк вместо ship-специфичных (мах,
   скоростной напор, тепловой поток на входе, теплозащита у бустера
   не считаются). t — полный кадр сцены, для общего модельного времени. */
function boosterHudLines(b, t) {
  if (!b) return ['бустер сейчас не летит'];
  const fin = (b.gridFins || []).map(f => `${num(f.deflection, 0)}°`).join(' ');
  const lines = [
    `Бустер: ${b.phase} · T+${num(t.time, 0)} с`,
    `высота ${num((b.altitude || 0) / 1000, 1)} км`,
  ];
  if (fin) lines.push(`рули ${fin}`);
  if (b.phase === 'Caught') lines.push('ПОЙМАН БАШНЕЙ');
  lines.push(...catchLines(b.catch));
  if (b.destroyed) lines.push('РАЗРУШЕН: приводнился и завалился набок');
  return lines;
}

/* Дальность камеры под то, что сейчас летит.

   До разделения в кадре сто двадцать метров сборки, после — полсотни метров
   корабля. Одна и та же дальность годится только для чего-то одного: ставим
   её по длине, а приближение оператора храним отдельным множителем, чтобы
   разделение не сбрасывало выбранный им масштаб. */
function fitCamera(t) {
  const layout = S.layout;
  if (!layout?.stages?.length) return;

  const slot = t.isBooster ? S3.fit.booster : S3.fit.ship;
  // Внутри одного тела ключ подгонки — это то, что меняет его длину:
  // у корабля смена ступени (сброс второй, сама она короче связки),
  // у бустера — ничего, ступень всегда первая.
  const key = t.isBooster ? 'booster' : t.stage;
  if (slot.key === key) return;

  slot.base = vehicleLength(t) * 2.4;
  slot.key = key;
}

/* Длина того, что сейчас летит: до разделения вся сборка, после — корабль
   (или, при слежении за бустером, только первая ступень). */
function vehicleLength(t) {
  const layout = S.layout;
  if (!layout?.stages?.length) return 120;

  if (t.isBooster) {
    return layout.stages.find(s => s.index === 1)?.length || layout.totalLength;
  }
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

  const base = (t.isBooster ? S3.fit.booster.base : S3.fit.ship.base) || S3.cam.dist;
  const dist = base * S3.zoom;
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

// offset сдвигает построенное тело в мировых осях — по умолчанию нулевой
// (тело строится в начале координат, как и раньше). Нужен только режиму
// «Оба» (renderBoth): корабль и бустер там показаны не в истинном взаимном
// положении (после отделения это километры, в кадре с деталями всё равно
// не поместились бы), а нарочно разнесены на фиксированное расстояние,
// каждый вокруг своего начала координат, — иначе, оба в буквальном центре,
// их корпуса просто взаимно проступали бы друг сквозь друга на экране.
function bodyToWorld(t, offset) {
  const f = vUnit(fromScene(t.scene?.forward) || v3(0, 0, 1));
  const r = vUnit(fromScene(t.scene?.right));
  const d = vUnit(fromScene(t.scene?.down));
  const off = offset || v3();

  // Оси приходят готовой тройкой, и собирать матрицу из углов не нужно:
  // любое расхождение в порядке поворотов дало бы корабль, летящий боком.
  return p => v3(
    off.x + f.x * p.x + r.x * p.y + d.x * p.z,
    off.y + f.y * p.x + r.y * p.y + d.y * p.z,
    off.z + f.z * p.x + r.z * p.y + d.z * p.z);
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

function vehicleFaces(faces, t, onlyStage, offset) {
  const layout = S.layout;
  if (!layout || !layout.stages?.length) return;

  const toWorld = bodyToWorld(t, offset);
  const seg = 20;

  // onlyStage — явный запрос нарисовать одну конкретную ступень (сейчас
  // им пользуется только бустер: у него всегда первая, независимо от того,
  // какая ступень активна у корабля в этом же кадре). Без явного запроса —
  // прежнее правило: после разделения рисуется только корабль.
  const stages = onlyStage != null
    ? layout.stages.filter(s => s.index === onlyStage)
    : t.stage >= 2
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
      // Купол/оживал — только у корабля. У бустера нос — плоская крышка
      // бака (перед решёткой межступенного стыка), не купол: без этой
      // оговорки booster-рендер (onlyStage === 1) наследовал носовую секцию
      // корабля просто потому, что она первая по порядку в его собственном
      // списке ступеней.
      const top = onlyStage == null && stage === stages[0] && section === stage.sections[0];

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

  if (onlyStage === 1) {
    // Решётчатые рули бустера — свои крепления, не по развёртке ship-flaps
    // (для них нет привязок в /api/vehicle/layout, только у ship-flapMounts).
    if (t.gridFins?.length) gridFinFaces(faces, toWorld, t.gridFins, total, layout.diameter / 2);
  } else {
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
}

/* Решётчатые рули бустера Super Heavy V3: три плоских щитка (не четыре),
   практически у самого верха ступени, перед кольцом горячего разделения.

   Раскладка — НЕ равномерное кольцо через сто двадцать градусов (так было
   нарисовано раньше, и ровно то же по ошибке когда-то стояло и в physics
   — simulator/vehicle/surfaces.go). По доступным фотографиям показа рулей
   V3 (август 2025) раскладка T-образная: два рули друг напротив друга
   ("порт"/"старборд", 180° друг от друга), третий — перпендикулярно им,
   на стороне без башни. Азимуты (0°, 90°, 180°) захардкожены здесь ЖЁСТКО
   СИНХРОННО с physics-стороной (vehicle.gridFinAzimuths) — единственный
   источник правды по факту физики, а не по рисовке, но развёртки-привязки
   как таковой между ними нет: азимуты и станция продублированы руками,
   и при следующей правке geometry на физической стороне их придётся
   поправить и здесь тоже.

   Станция — 0.97 длины от среза сопел, то есть 0.03 длины от носа: та же
   точка, что и в physics (GridFins→PositionFromNose = length·0.03,
   simulator/vehicle/surfaces.go). Размах 7.5 м, ширина 3.75 м — те же
   числа, что и там же (gridFinSpan/gridFinChord).
   Форма, число щитков и раскладка по азимуту с бэкенда не читаются —
   в отличие от ship-flapMounts, эта геометрия здесь не с телеметрии,
   а зафиксирована руками: угол раскрытия и температура берутся из
   телеметрии (то же FlapTelemetry, что и у плавников корабля), геометрия —
   нет.

   Бустер спускается двигателями вниз, носом вперёд по потоку: набегающий
   воздух идёт вдоль продольной оси корпуса. Чтобы решётка вообще работала
   (гасила и создавала момент обтеканием), её плоскость должна стоять
   поперёк этого потока — размах радиально от борта наружу и поперёк
   по окружности, — а не вдоль потока. Первая версия строила щиток
   в плоскости (ось корпуса × радиус) — это разворачивало решётку на
   девяносто градусов относительно набегающего потока, ребром по потоку
   вместо лицом к нему. Здесь та же решётка стоит в плоскости
   (радиус × поперёк потока), а рыскание (deflection) поворачивает эту
   плоскость вокруг радиальной оси щитка — так руление видно как
   разворот решётки, а не как её изгиб. */
function gridFinFaces(faces, toWorld, fins, total, r) {
  const station = total * 0.97;
  const rad = Math.PI / 180;
  const span = 7.5;   // вылет от борта наружу (по радиусу), м
  const width = 3.75; // ширина решётки поперёк потока (по окружности), м

  // Те же азимуты, что и vehicle.gridFinAzimuths (simulator/vehicle/surfaces.go):
  // fin_1/fin_3 друг напротив друга (0°/180°), fin_2 перпендикулярно (90°).
  const gridFinAzimuths = [0, Math.PI / 2, Math.PI];

  fins.forEach((f, i) => {
    const az = gridFinAzimuths[i] ?? (2 * Math.PI / fins.length) * i;
    const s = Math.sin(az), c = Math.cos(az);
    // Орт «поперёк потока» в плоскости (y, z) на этом азимуте — касательная
    // к окружности корпуса, перпендикулярная радиальному орту (s, c).
    const tc = c, ts = -s;

    // Рыскание при рулении: поворот плоскости щитка вокруг его радиальной
    // оси — часть поперечного размаха уходит в осевое смещение, и решётка
    // видна развёрнутой, а не изогнутой.
    const openA = (f.deflection || 0) * rad;
    const cs = Math.cos(openA), sn = Math.sin(openA);
    const halfW = width / 2;

    // rr — радиус от оси корпуса, t — координата вдоль ширины щитка
    // в его собственной (повёрнутой рысканьем) плоскости.
    const p = (rr, t) => ({
      x: station + t * sn,
      y: rr * s + t * cs * tc,
      z: rr * c + t * cs * ts,
    });

    const a0 = p(r, -halfW), a1 = p(r, halfW);
    const b0 = p(r + span, -halfW), b1 = p(r + span, halfW);

    const colour = f.manual ? [90, 120, 170] : [96, 100, 108];
    faces.push({ p: [a0, a1, b1, b0].map(toWorld), c: colour });
    faces.push({ p: [b0, b1, a1, a0].map(toWorld), c: colour }); // видна с обеих сторон
  });
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

  // Грунт у площадки лежит НИЖЕ нулевой отметки телеметрии на высоту
  // стартового стола: ноль — это верх стола, на нём изделие и стоит
  // (см. towerFaces). Поправка вводится только там, где башня в кадре, —
  // вдали от площадки разница в два десятка метров невидима, а сетка
  // должна сходиться с горизонтом, а не с фермой.
  const drop = towerBase(t) ? (S3.tower.tableHeight || 0) : 0;
  const z = -Math.max(alt, 0.5) - drop;
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
  const p = project(v3(0, 0, -Math.max(alt, 0.5)), cam);
  if (p) {
    ctx.strokeStyle = `rgba(255,190,120,${0.3 + 0.5 * fade})`;
    ctx.beginPath();
    ctx.arc(p.x, p.y, 4, 0, Math.PI * 2);
    ctx.stroke();
  }
}

/* --------------------------------------------------------------------------
   Башня-ловушка

   Цель возврата бустера — не точка на земле, а ПРОСВЕТ МЕЖДУ РУКАМИ башни на
   высоте нескольких десятков метров над стартовым столом. Поэтому здесь
   рисуется не условная отметка площадки, а сама конструкция: ферма, стол, обе
   руки и коридор захвата между ними. По такой картинке видно то, чего не видно
   ни по одному числу, — проходит ли корпус между руками или бьётся о них.

   Геометрия приходит один раз с /api/catch/tower, положение — вектором от
   корпуса к основанию фермы в кадре сцены (booster.tower): считать его из
   широт и долгот на стороне картинки значило бы повторять преобразования
   координат и расходиться с моделью на десятки метров.
   -------------------------------------------------------------------------- */

/* Дальность, с которой башня начинает рисоваться, м. Дальше она занимает
   меньше пикселя, а граней стоит полторы сотни. */
const TOWER_DRAW_RANGE = 6000;

function loadTower() {
  fetch('/api/catch/tower')
    .then(r => (r.ok ? r.json() : null))
    .then(g => { if (g && g.towerHeight) S3.tower = g; })
    .catch(() => {});
}

/* Вектор от корпуса к основанию фермы в осях сцены. null — башни в кадре нет:
   либо геометрия ещё не загружена, либо это не бустер, либо до площадки
   слишком далеко. */
function towerBase(t) {
  if (!S3.tower || !t || !t.scene || !t.scene.tower) return null;
  const base = fromScene(t.scene.tower);
  return vLen(base) > TOWER_DRAW_RANGE ? null : base;
}

/* Орты системы башни в осях сцены: вдоль рук (от фермы к просвету), поперёк
   рук и вверх. Сходимость меридианов на километре площадки — доли угловой
   секунды, поэтому азимут берётся как есть. */
function towerAxes(g) {
  const a = (g.armAzimuth || 0) * Math.PI / 180;
  const along = v3(Math.sin(a), Math.cos(a), 0);
  const up = v3(0, 0, 1);
  return { along, across: vCross(up, along), up };
}

/* Построитель точки в осях башни: вдоль, поперёк, вверх — в метрах. */
function towerMapper(g, base) {
  const { along, across, up } = towerAxes(g);
  return (x, y, z) => vAdd(base,
    vAdd(vAdd(vMul(along, x), vMul(across, y)), vMul(up, z)));
}

/* Параллелепипед по центру и трём полуосям. twoSided — чтобы не зависеть от
   обхода вершин: тела выпуклые, сортировка по глубине рисует дальние грани
   раньше ближних сама. */
function boxFaces(faces, P, c, half, colour, extra) {
  const [cx, cy, cz] = c;
  const [hx, hy, hz] = half;
  const p = (sx, sy, sz) => P(cx + hx * sx, cy + hy * sy, cz + hz * sz);
  const quads = [
    [[1, -1, -1], [1, 1, -1], [1, 1, 1], [1, -1, 1]],
    [[-1, -1, -1], [-1, -1, 1], [-1, 1, 1], [-1, 1, -1]],
    [[-1, 1, -1], [-1, 1, 1], [1, 1, 1], [1, 1, -1]],
    [[-1, -1, -1], [1, -1, -1], [1, -1, 1], [-1, -1, 1]],
    [[-1, -1, 1], [1, -1, 1], [1, 1, 1], [-1, 1, 1]],
    [[-1, -1, -1], [-1, 1, -1], [1, 1, -1], [1, -1, -1]],
  ];
  for (const q of quads) {
    faces.push(Object.assign({
      p: q.map(s => p(s[0], s[1], s[2])), c: colour, twoSided: true,
    }, extra || {}));
  }
}

/* Плоский стержень фермы: отрезок от a до b, поднятый на ширину w. Один
   четырёхугольник на связь вместо шести — решётка из сотни коробок стоила бы
   дороже самого корпуса ракеты. */
function strutFace(faces, P, a, b, w, colour) {
  const p0 = P(a[0], a[1], a[2]);
  const p1 = P(b[0], b[1], b[2]);
  const p2 = P(b[0], b[1], b[2] + w);
  const p3 = P(a[0], a[1], a[2] + w);
  faces.push({ p: [p0, p1, p2, p3], c: colour, twoSided: true, flat: true });
}

const TOWER_STEEL = [118, 126, 138];
const TOWER_DARK = [86, 92, 102];
const TABLE_STEEL = [104, 100, 94];
const ARM_STEEL = [150, 156, 166];

function towerFaces(faces, t) {
  const g = S3.tower;
  const base = towerBase(t);
  if (!base) return;

  const P = towerMapper(g, base);
  const w = g.towerWidth / 2;

  // Нулевая отметка сцены — УРОВЕНЬ СТАРТА, то есть верх стартового стола.
  //
  // Высота в телеметрии отсчитывается от точки, в которой изделие стоит на
  // старте, и в трёхмерной сцене корпус строится от той же нулевой отметки.
  // Значит, стол обязан быть ПОД ней, а не над: пока тумба рисовалась вверх
  // от нуля, ракета на старте оказывалась внутри неё по самые решётчатые
  // рули. Грунт вокруг лежит на высоту стола ниже — там же стоит и ферма.
  const ground = -g.tableHeight;

  // Стартовый стол — восьмигранная тумба под центром зоны захвата.
  const tableSeg = 8, tr = g.tableRadius;
  const ringPt = i => {
    const a = (i / tableSeg) * Math.PI * 2;
    return [g.armReach + tr * Math.cos(a), tr * Math.sin(a)];
  };
  for (let i = 0; i < tableSeg; i++) {
    const a = ringPt(i), b = ringPt((i + 1) % tableSeg);
    faces.push({
      p: [P(a[0], a[1], ground), P(b[0], b[1], ground),
        P(b[0], b[1], 0), P(a[0], a[1], 0)],
      c: TABLE_STEEL, twoSided: true,
    });
  }
  const top = [];
  for (let i = 0; i < tableSeg; i++) {
    const a = ringPt(i);
    top.push(P(a[0], a[1], 0));
  }
  faces.push({ p: top, c: [126, 122, 116], twoSided: true });

  // Ферма: четыре пояса и решётка по секциям.
  const legs = [[w, w], [w, -w], [-w, -w], [-w, w]];
  for (const [x, y] of legs) {
    boxFaces(faces, P, [x, y, ground + g.towerHeight / 2],
      [0.7, 0.7, g.towerHeight / 2], TOWER_STEEL);
  }
  const bays = Math.max(1, g.towerBays | 0);
  const bay = g.towerHeight / bays;
  for (let i = 0; i < bays; i++) {
    const z0 = ground + i * bay, z1 = ground + (i + 1) * bay;
    for (let k = 0; k < 4; k++) {
      const a = legs[k], b = legs[(k + 1) % 4];
      strutFace(faces, P, [a[0], a[1], z1], [b[0], b[1], z1], 0.6, TOWER_DARK);
      strutFace(faces, P, [a[0], a[1], z0], [b[0], b[1], z1], 0.5, TOWER_DARK);
      strutFace(faces, P, [b[0], b[1], z0], [a[0], a[1], z1], 0.5, TOWER_DARK);
    }
  }

  // Руки: две балки на высоте захвата, просвет между ними — то самое, во что
  // обязан войти корпус.
  const armMid = w + g.armLength / 2;
  const side = g.armGap / 2 + g.armWidth / 2;
  for (const s of [1, -1]) {
    boxFaces(faces, P, [armMid, s * side, g.armHeight],
      [g.armLength / 2, g.armWidth / 2, g.armThickness / 2], ARM_STEEL);
    // Обойма на конце руки — по ней видно, где руки смыкаются.
    boxFaces(faces, P, [g.armReach, s * (g.armGap / 2 + 0.6), g.armHeight],
      [3.0, 0.6, g.armThickness / 2 + 0.4], [190, 160, 90]);
  }

  // Коридор захвата: прозрачный объём от высоты опорной точки до рук —
  // канал, по которому корпус обязан пройти.
  const chTop = g.armHeight + 6;
  boxFaces(faces, P, [g.armReach, 0, (g.catchHeight + chTop) / 2],
    [g.corridorAlong, g.corridorAcross, (chTop - g.catchHeight) / 2],
    [70, 190, 255], { glow: 0.10 });

  // Само окно захвата — допуск на опорную точку корпуса.
  boxFaces(faces, P, [g.armReach, 0, g.catchHeight],
    [g.corridorAlong, g.corridorAcross, g.catchWindow],
    [90, 255, 170], { glow: 0.22 });
}

/* Разметка промаха поверх сцены: центр зоны, фактическая точка прохода
   плоскости рук, отрезок между ними и подпись.

   Точка прохода — именно та, что модель записала при пересечении высоты
   захвата (booster.catch), а не текущее положение корпуса: после того как
   ступень прошла зону насквозь и упала, её координаты описывают уже другое
   событие. */
function drawCatch(ctx, cam, t) {
  const g = S3.tower;
  const base = t && t.isBooster ? towerBase(t) : null;
  if (!base) return;

  const P = towerMapper(g, base);
  const c = t.catch || {};
  const centre = P(g.armReach, 0, g.catchHeight);

  const mark = (p, colour, r) => {
    const s = project(p, cam);
    if (!s) return null;
    ctx.strokeStyle = colour;
    ctx.lineWidth = 1.6;
    ctx.beginPath();
    ctx.moveTo(s.x - r, s.y); ctx.lineTo(s.x + r, s.y);
    ctx.moveTo(s.x, s.y - r); ctx.lineTo(s.x, s.y + r);
    ctx.stroke();
    ctx.beginPath();
    ctx.arc(s.x, s.y, r * 0.62, 0, Math.PI * 2);
    ctx.stroke();
    return s;
  };

  const sc = mark(centre, 'rgba(110,255,180,0.95)', 9);
  if (sc) {
    ctx.fillStyle = 'rgba(110,255,180,0.95)';
    ctx.font = '11px ui-monospace, monospace';
    ctx.fillText('центр захвата', sc.x + 12, sc.y - 6);
  }

  if (!Number.isFinite(c.catchActualX)) return;

  const actual = P(c.catchActualX, c.catchActualY, c.catchActualZ);
  const hit = c.catchSuccess;
  const colour = hit ? 'rgba(120,255,140,0.95)'
    : c.catchCrossed ? 'rgba(255,140,120,0.95)' : 'rgba(255,205,110,0.95)';

  const sa = mark(actual, colour, 8);
  if (sa && sc) {
    ctx.strokeStyle = colour;
    ctx.setLineDash([5, 4]);
    ctx.lineWidth = 1.4;
    ctx.beginPath();
    ctx.moveTo(sc.x, sc.y);
    ctx.lineTo(sa.x, sa.y);
    ctx.stroke();
    ctx.setLineDash([]);

    const miss = c.catchMissHorizontal;
    if (Number.isFinite(miss)) {
      ctx.fillStyle = colour;
      ctx.font = 'bold 12px ui-monospace, monospace';
      ctx.fillText(`${hit ? 'ЗАХВАТ' : 'ПРОМАХ'}: ${num(miss, 0)} м`,
        (sc.x + sa.x) / 2 + 8, (sc.y + sa.y) / 2 - 6);
    }
  }
}

/* Камера финального участка: башня, обе руки, стол и подходящий бустер
   одновременно в кадре.

   Взгляд ставится ВДОЛЬ рук — только с этого направления видно, проходит ли
   корпус между ними: вдоль просвета руки расходятся влево и вправо, и зазор
   читается напрямую. Дальше оператор волен вертеть камеру как обычно —
   направление задаётся один раз, при входе в зону. */
function buildCatchCamera(t, w, h) {
  const g = S3.tower;
  const base = t && t.isBooster ? towerBase(t) : null;
  if (!base) { leaveCatchCamera(); return null; }

  const P = towerMapper(g, base);
  const centre = P(g.armReach, 0, g.catchHeight);
  if (vLen(centre) > 1500) { leaveCatchCamera(); return null; }

  if (!S3.catchCam) {
    S3.catchCam = true;
    S3.bodyCamSaved = S3.bodyCam;
    S3.cam.yaw = (g.armAzimuth || 0) * Math.PI / 180;
    S3.cam.pitch = 0.10;
    S3.bodyCam = false;
  }

  const target = vMul(centre, 0.5);
  const span = vLen(centre) + vehicleLength(t) * 0.7 + g.armLength;
  const dist = Math.max(span * 0.9, 120) * S3.zoom;
  S3.cam.dist = dist;

  const cp = Math.cos(S3.cam.pitch), sp = Math.sin(S3.cam.pitch);
  let pos = vAdd(target, v3(
    dist * cp * Math.sin(S3.cam.yaw),
    dist * cp * Math.cos(S3.cam.yaw),
    dist * sp));

  const groundZ = -(t.altitude || 0) + 12;
  if (pos.z < groundZ) pos = v3(pos.x, pos.y, groundZ);

  const fwd = vUnit(vSub(target, pos));
  let right = vCross(fwd, v3(0, 0, 1));
  if (vLen(right) < 1e-6) right = v3(1, 0, 0);
  right = vUnit(right);
  return { pos, target, fwd, right, up: vCross(right, fwd), cx: w / 2, cy: h / 2, f: h * 1.15 };
}

/* Выход из режима захвата возвращает оператору его же настройку привязки
   камеры к корпусу: режим её временно снимает, чтобы башня не вращалась
   вместе с ракетой, но забирать чужую галочку насовсем он не вправе. */
function leaveCatchCamera() {
  if (!S3.catchCam) return;
  S3.catchCam = false;
  if (S3.bodyCamSaved !== undefined) S3.bodyCam = S3.bodyCamSaved;
}

/* Строки о захвате для подписи под бустером. */
function catchLines(c) {
  if (!c || !Number.isFinite(c.catchMissHorizontal)) return [];
  const out = [];
  if (c.catchSuccess) {
    out.push(`ЗАХВАТ: промах ${num(c.catchMissHorizontal, 1)} м`);
  } else if (c.catchCrossed) {
    out.push(`ПРОМАХ ЗАХВАТА: ${num(c.catchMissHorizontal, 0)} м`);
  } else {
    out.push(`подход к зоне: ${num(c.catchMissHorizontal, 0)} м`);
  }
  out.push(`вдоль ${num(c.catchMissX, 0)} · поперёк ${num(c.catchMissY, 0)} · верт ${num(c.catchMissZ, 0)} м`);
  if (Number.isFinite(c.catchMiss3D)) out.push(`полный промах ${num(c.catchMiss3D, 0)} м`);
  if (Number.isFinite(c.catchMissDownrange)) {
    out.push(`продольный ${num(c.catchMissDownrange, 0)} · боковой ${num(c.catchMissCrossrange, 0)} м`);
  }
  if (Number.isFinite(c.catchVerticalVelocity)) {
    out.push(`в зоне: Vверт ${num(c.catchVerticalVelocity, 1)} · Vгор ${num(c.catchHorizontalVelocity, 1)} м/с`);
    out.push(`наклон ${num(c.catchTilt, 1)}° · вращение ${num(c.catchAngularRate, 1)}°/с`);
  }
  return out;
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
  // Новый прогон — новый след, камера снова не развёрнута вдоль рук, и
  // геометрия башни перечитывается: между прогонами оператор мог сменить
  // носитель, а высота захвата считается от длины ступени.
  if (t.time < S3.lastTime) { S3.track = []; leaveCatchCamera(); loadTower(); }
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

// align: 'left' (по умолчанию) — коробка у левого края, 'right' — у
// правого. maxWidth — сколько места ей вообще отведено (по умолчанию весь
// холст); в режиме «Оба» (renderBoth) там же два тела и два независимых
// HUD, левый и правый, и без явного предела в половину холста они наползли
// бы друг на друга посередине.
function hudText(ctx, w, h, lines, align = 'left', maxWidth) {
  const pad = 8;
  let size = 12;
  ctx.font = `${size}px ui-monospace, monospace`;
  ctx.textBaseline = 'top';

  // Строки HUD подобраны под ширину всего холста, а не под половину —
  // самые длинные, например у корабля с плавниками и теплозащитой,
  // в отведённый предел не помещаются. Раз строка не влезает — шрифт
  // мельче, а не текст короче: цифры важнее миллиметровой точности букв.
  const avail = Math.max(80, (maxWidth || w) - pad * 2 - 16);
  let width = Math.max(...lines.map(l => ctx.measureText(l).width));
  if (width > avail) {
    size = Math.max(7, Math.floor(size * avail / width));
    ctx.font = `${size}px ui-monospace, monospace`;
    width = Math.max(...lines.map(l => ctx.measureText(l).width));
  }

  // Даже семи пикселей может не хватить — дальше мельчить нечитаемо. Тут
  // уже не шрифт, а сами строки: обрезаем по символам с многоточием, лишь
  // бы не резало посреди слова случайным клипом canvas.
  if (width > avail) {
    lines = lines.map(l => {
      if (ctx.measureText(l).width <= avail) return l;
      let cut = l;
      while (cut.length > 1 && ctx.measureText(cut + '…').width > avail) cut = cut.slice(0, -1);
      return cut + '…';
    });
    width = avail;
  }

  const lineHeight = size + 4;
  const boxW = width + pad * 2;
  const x0 = align === 'right' ? w - boxW - 8 : 8;
  ctx.fillStyle = 'rgba(8,12,18,0.55)';
  ctx.fillRect(x0, 8, boxW, lines.length * lineHeight + pad * 2 - 4);

  ctx.fillStyle = '#c8d6e4';
  lines.forEach((l, i) => ctx.fillText(l, x0 + pad, 8 + pad + i * lineHeight));
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

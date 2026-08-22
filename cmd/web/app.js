/* ---------------------------------------------------------------------------
   Пульт управления симулятором.

   Здесь нет ни одной физической формулы. Интерфейс умеет ровно три вещи:
   отправить команду, показать то, что вернул симулятор, и нарисовать историю
   присланных значений. Все связи между параметрами вычисляет модель на сервере,
   поэтому «увеличить обороты» здесь — это отправка команды, а не изменение
   числа на экране.
   --------------------------------------------------------------------------- */

'use strict';

/* ===========================================================================
   Состояние
   =========================================================================== */

const S = {
  registry: null,        // реестр параметров с сервера
  params: new Map(),     // id → описание параметра
  snapshot: null,        // последний снимок состояния
  log: [],               // журнал вмешательств
  commands: new Map(),   // id команды → её статус
  series: new Map(),     // ключ величины → массив точек {t, v}
  markers: [],           // отметки вмешательств на графиках
  subsystem: 'turbopump',
  dataView: 'charts',
  latency: null,
  connected: false,
  seq: 0,
  chartKeys: [],
  profiles: [],
  missions: [],
  layout: null,          // обводы носителя для развёртки
  fuelLoad: null,        // заправка на следующий пуск
  cutZoom: 1,            // увеличение чертежа
  endStage: 0,           // ступень на виде с торца, 0 — активная
  bellsDrawn: {},        // сколько сопел попало на разрез, по ступеням
  lastRunState: null,
  scenarios: [],
  scenarioIndex: 0,
  scenarioRun: null,
  pendingConfirm: null,

  // Двигатель, которому адресуются команды. Пусто — вся ступень.
  target: '',
};

const SUBSYSTEMS = [
  { id: 'turbopump', title: 'Турбонасос' },
  { id: 'valves',    title: 'Клапаны' },
  { id: 'tanks',     title: 'Баки' },
  { id: 'chamber',   title: 'Камера' },
  { id: 'nozzle',    title: 'Сопло' },
  { id: 'engine',    title: 'Двигатель' },
  { id: 'control',   title: 'Рулевой тракт' },
];

/* Величины, которые интерфейс рисует после вмешательства в турбонасос.
   Список задан постановкой задачи и не зависит от того, что менял оператор. */
const PUMP_CHART_KEYS = [
  'turbopump.rpm',
  'turbopump.fuelPumpOutlet',
  'turbopump.oxPumpOutlet',
  'engine.fuelFlow',
  'engine.oxFlow',
  'chamber.mixtureRatio',
  'chamber.pressure',
  'chamber.temperature',
  'engine.thrust',
  'engine.specificImpulse',
  'turbopump.vibration',
  'turbopump.fuelCavitationMargin',
  'turbopump.bearingTemperature',
  'vehicle.acceleration',
];

const KEY_VALUES = [
  { key: 'engine.thrust',          title: 'Тяга',            unit: 'кН',  big: true,  digits: 0 },
  { key: 'chamber.pressure',       title: 'Давление камеры', unit: 'МПа', big: true,  digits: 2 },
  { key: 'turbopump.rpm',          title: 'Обороты вала',    unit: 'об/мин', big: true, digits: 0 },
  { key: 'chamber.mixtureRatio',   title: 'O/F',             unit: '',    digits: 3 },
  { key: 'engine.specificImpulse', title: 'Удельный импульс', unit: 'с',  digits: 1 },
  { key: 'chamber.temperature',    title: 'Температура камеры', unit: 'К', digits: 0 },
  { key: 'engine.fuelFlow',        title: 'Расход горючего', unit: 'кг/с', digits: 1 },
  { key: 'engine.oxFlow',          title: 'Расход окислителя', unit: 'кг/с', digits: 1 },
  { key: 'chamber.cstarEfficiency', title: 'Полнота сгорания', unit: '',  digits: 3 },
  { key: 'chamber.stabilityMargin', title: 'Запас устойчивости', unit: '', digits: 3 },
  { key: 'nozzle.wallMargin',      title: 'Запас стенки',    unit: 'К',   digits: 0 },
  { key: 'turbopump.vibration',    title: 'Вибрация',        unit: 'g',   digits: 2 },
  { key: 'fuelTank.mass',          title: 'Остаток горючего', unit: 'т',  digits: 1 },
  { key: 'oxTank.mass',            title: 'Остаток окислителя', unit: 'т', digits: 1 },
  { key: 'vehicle.acceleration',   title: 'Перегрузка',      unit: 'g',   digits: 2 },
  { key: 'vehicle.altitude',       title: 'Высота',          unit: 'км',  digits: 1 },
];

/* Показатели компенсации отказа. Приходят готовыми из снимка: интерфейс
   ничего не пересчитывает. */
const COMPENSATION = [
  { get: s => s.enginesOut,          title: 'Двигателей выбыло', unit: '',  digits: 0,
    bad: v => v > 0 },
  { get: s => s.engineThrottle * 100, title: 'Газ на двигатель', unit: '%', digits: 1,
    bad: v => v > 100.5 },
  { get: s => s.thrustDeficit * 100,  title: 'Недобор тяги',     unit: '%', digits: 1,
    bad: v => v > 0.1 },
];

/* ===========================================================================
   Мелкие помощники
   =========================================================================== */

const $  = (sel, root = document) => root.querySelector(sel);
const $$ = (sel, root = document) => [...root.querySelectorAll(sel)];

function el(tag, cls, text) {
  const n = document.createElement(tag);
  if (cls) n.className = cls;
  if (text !== undefined) n.textContent = text;
  return n;
}

function num(v, digits = 2) {
  if (v === null || v === undefined || Number.isNaN(v)) return '—';
  const a = Math.abs(v);
  if (a !== 0 && (a >= 1e6 || a < 1e-4)) return v.toExponential(1);
  return v.toLocaleString('ru-RU', {
    minimumFractionDigits: digits, maximumFractionDigits: digits,
  });
}

function clock(seconds) {
  if (seconds === null || seconds === undefined || Number.isNaN(seconds)) return '—';
  const sign = seconds < 0 ? '−' : '+';
  const s = Math.abs(seconds);
  const h = Math.floor(s / 3600);
  const m = Math.floor((s % 3600) / 60);
  const sec = s % 60;
  const mm = String(m).padStart(2, '0');
  const ss = sec.toFixed(1).padStart(4, '0');
  return h > 0 ? `T${sign}${h}:${mm}:${ss}` : `T${sign}${mm}:${ss}`;
}

function uuid() {
  S.seq += 1;
  return `c${Date.now().toString(36)}-${S.seq}`;
}

function toast(kind, title, text) {
  const t = el('div', `toast ${kind}`);
  t.append(el('b', null, title));
  if (text) t.append(el('small', null, text));
  $('#toasts').append(t);
  setTimeout(() => t.remove(), kind === 'err' ? 9000 : 4500);
}

/* Выбор носителя. Профиль меняет всё: массы, число двигателей, топливо,
   геометрию и точку старта, — поэтому применяется только при новом запуске. */
function selectProfile(id) {
  fetch('/api/profiles/select', {
    method: 'POST', headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ id }),
  })
    .then(r => r.json())
    .then(r => {
      describeProfile(id);
      toast('ok', `Носитель: ${r.title}`, r.note);
      // Обводы меняются вместе с носителем: развёртка перестраивается.
      loadLayout();
    })
    .catch(() => toast('err', 'Не удалось выбрать носитель'));
}

/* Выбор задания. Задание не трогает железо: оно говорит наведению, где
   выключать двигатели. Отсюда и разница в возвращении — с замкнутой орбиты
   корабль сам не сойдёт, а с трансатмосферной траектории только и делает,
   что возвращается. */
function selectMission(id) {
  fetch('/api/missions/select', {
    method: 'POST', headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ id }),
  })
    .then(r => r.ok ? r.json() : Promise.reject(r.statusText))
    .then(r => {
      describeMission(id);
      toast('ok', `Задание: ${r.title}`, r.note);
    })
    .catch(() => toast('err', 'Не удалось выбрать задание'));
}

function describeMission(id) {
  const m = (S.missions || []).find(x => x.id === id);
  if (!m) return;

  const shape = m.periapsisAltitude < 0
    ? `верхняя точка ${num(m.apoapsisAltitude / 1000, 0)} км, ` +
      `нижняя ${num(m.periapsisAltitude / 1000, 0)} км — внутри атмосферы`
    : `${num(m.apoapsisAltitude / 1000, 0)} × ${num(m.periapsisAltitude / 1000, 0)} км`;

  toast('ok', m.title, `${shape}. ${m.summary}`);
  for (const note of m.notes || []) toast('warn', 'Что это значит', note);
}

function describeProfile(id) {
  const p = (S.profiles || []).find(x => x.id === id);
  if (!p) return;

  const lines = [
    `${p.firstStageEngines} + ${p.secondStageEngines} двигателей, ` +
    `${p.propellants}, схема ${p.cycle}`,
    `стартовая масса ${num(p.liftoffMass, 0)} т, тяга ${num(p.liftoffThrust, 1)} МН, ` +
    `нагрузка ${num(p.payload, 0)} т`,
  ];
  toast('ok', p.title, lines.join('; '));
  for (const note of p.notes || []) toast('warn', 'Упрощение модели', note);
}

/* ===========================================================================
   Соединение
   =========================================================================== */

let ws = null;
let reconnectDelay = 500;

function connect() {
  const proto = location.protocol === 'https:' ? 'wss' : 'ws';
  ws = new WebSocket(`${proto}://${location.host}/api/ws`);

  ws.onopen = () => {
    S.connected = true;
    reconnectDelay = 500;
    setLink('есть');
    // Очередь команд, не получивших подтверждения, отправляется заново.
    // Идентификатор у каждой свой, поэтому повторно она не применится.
    for (const [id, c] of S.commands) {
      if (c.status === 'queued' || c.status === 'sent') sendCommand(c.command, id);
    }
  };

  ws.onclose = () => {
    S.connected = false;
    setLink('нет');
    // Переподключение с нарастающей паузой: сервер мог перезапускаться.
    setTimeout(connect, reconnectDelay);
    reconnectDelay = Math.min(reconnectDelay * 2, 8000);
  };

  ws.onerror = () => ws.close();
  ws.onmessage = (ev) => handleMessage(JSON.parse(ev.data));
}

function setLink(text) {
  const n = $('#stat-link');
  n.textContent = text;
  n.style.color = text === 'есть' ? 'var(--normal)' : 'var(--critical)';
}

function handleMessage(msg) {
  switch (msg.type) {
    case 'telemetry': onSnapshot(msg.snapshot); break;

    // Кадр сцены между полными снимками: положение, оси корпуса и плавники
    // без подробностей двигательной установки. Приборные панели его не видят —
    // им незачем, а сцена получает вдвое больше опорных состояний.
    case 'scene': if (S.dataView === 'scene') guarded('кадр сцены', () => onSceneFrame(msg.scene)); break;

    case 'log':       S.log = msg.log; renderLog(); renderCausal(); break;
    case 'result':    onResult(msg.result); break;
    case 'pong': {
      S.latency = Date.now() - msg.sent;
      $('#stat-latency').textContent = `${S.latency} мс`;
      break;
    }
    case 'idle':  setState('не запущена'); break;
    case 'error': toast('err', 'Ошибка сервера', msg.error); break;
  }
}

function send(obj) {
  if (ws && ws.readyState === WebSocket.OPEN) { ws.send(JSON.stringify(obj)); return true; }
  return false;
}

setInterval(() => send({ type: 'ping', sent: Date.now() }), 2000);

/* ===========================================================================
   Отправка команд
   =========================================================================== */

/* Жизненный цикл команды: Queued → Sent → Accepted/Applied либо Rejected.
   Статус хранится у клиента и показывается оператору. */
function sendCommand(command, existingId) {
  const id = existingId || command.id || uuid();
  command.id = id;

  const rec = S.commands.get(id) || { command, status: 'queued', at: Date.now() };
  rec.command = command;
  S.commands.set(id, rec);

  if (send({ type: 'command', command })) {
    rec.status = 'sent';
  } else {
    // Соединения нет: команда останется в очереди до переподключения.
    rec.status = 'queued';
    toast('warn', 'Нет связи', 'Команда поставлена в очередь');
  }

  // Срок годности: команда, не подтверждённая за десять секунд,
  // считается потерянной и больше не повторяется.
  setTimeout(() => {
    const r = S.commands.get(id);
    if (r && (r.status === 'sent' || r.status === 'queued')) {
      r.status = 'expired';
      toast('err', 'Команда не подтверждена', describe(command));
    }
  }, 10000);

  return id;
}

function onResult(res) {
  const rec = S.commands.get(res.id);
  if (rec) rec.status = res.status;

  if (res.status === 'applied') {
    S.markers.push({ t: res.modelTime, label: labelFor(res.parameter) });
    if (S.markers.length > 60) S.markers.shift();

    const from = Number.isFinite(res.previous) ? num(res.previous, 2) : '—';
    const to   = Number.isFinite(res.target)   ? num(res.target, 2)   : '—';
    toast('ok', `${labelFor(res.parameter)}: применено`,
      `${from} → ${to} ${res.unit || ''}`);

    (res.warnings || []).forEach(w => toast('warn', 'Предупреждение', w));
  } else if (res.status === 'rejected' || res.status === 'failed') {
    toast('err', 'Команда отклонена', res.reason || '');
  }
  renderEffects();
}

function labelFor(id) {
  const p = S.params.get(id);
  return p ? p.title : (id || 'команда');
}

function describe(cmd) {
  return `${labelFor(cmd.parameter)} · ${cmd.mode} ${num(cmd.value, 2)}`;
}

function action(name, value) {
  if (!send({ type: 'action', action: name, value: value || 0 })) {
    fetch(`/api/sim/${name}` + (value ? `?value=${value}&steps=${value}` : ''),
      { method: 'POST' }).catch(() => toast('err', 'Нет связи с сервером'));
  }
}

// igniteAll — зажигание в отличие от пауз и переключений скорости может
// быть отклонено (носитель разбит, корабль на площадке), и тогда молчаливое
// «отправили и забыли», как у action(), покажет успех там, где ничего
// не произошло. Поэтому идём напрямую по REST и показываем то, что реально
// ответил сервер, а не оптимистичное подтверждение по факту клика.
function igniteAll() {
  fetch('/api/sim/ignite', { method: 'POST' })
    .then(r => r.ok
      ? toast('ok', 'Зажигание', 'Камеры ступени запущены на холостом режиме.')
      : r.text().then(t => toast('err', 'Не удалось зажечь', t)))
    .catch(() => toast('err', 'Нет связи с сервером'));
}

/* ===========================================================================
   Приём снимка
   =========================================================================== */

function onSnapshot(s) {
  S.snapshot = s;

  setState(s.runState);
  $('#stat-model').textContent = clock(s.modelTime);
  $('#stat-real').textContent  = clock(s.realTime);
  $('#stat-phase').textContent = s.phase || '—';
  $('#stat-run').textContent   = `№${s.runNumber}`;
  $('#stat-scenario').textContent = s.scenario || '—';
  $('#speed').value = String(s.scale);

  $('#mode-unsafe').checked = s.unsafeMode;
  $('#mode-direct').checked = s.directMode;
  updateBanner(s);
  updateFlightBanner(s);

  // Развёртка привязана к носителю: пока ракета летит — к летящему,
  // после прогона — к выбранному. Значит, при смене состояния прогона
  // обводы надо перечитать.
  if (S.lastRunState !== s.runState) {
    S.lastRunState = s.runState;
    loadLayout();
  }

  // Каждая панель рисуется отдельно и под защитой.
  //
  // Прежде это был один список вызовов подряд, и первое же исключение
  // обрывало его целиком: телеметрия продолжала приходить, а панель застывала
  // до тех пор, пока переключение вкладки не вызывало отрисовку напрямую.
  // Оператор при этом видел живые цифры вверху и мёртвые везде остальные.
  // Ошибка в одном узле не должна останавливать пульт.
  collectSeries(s);
  guarded('блок двигателей', renderEngineStrip);
  guarded('турбонасосы', renderPumps);
  guarded('указатель ориентации', drawAttitude);
  guarded('тяга', updateThrottlePanel);
  guarded('ключевые показатели', renderKeyValues);
  guarded('предупреждения', renderAlarms);
  guarded('воздействия', renderEffects);
  guarded('параметры', renderParamValues);
  guarded('графики', drawCharts);

  if (S.dataView === 'cutaway') guarded('развёртка', drawCutaway);
  if (S.dataView === 'entry') guarded('вход в атмосферу', drawEntry);
  if (S.dataView === 'scene') guarded('трёхмерная сцена', drawScene3D);

  guarded('сценарий', () => runScenarioTick(s.modelTime));
}

/* guarded выполняет отрисовку одной панели, не давая её ошибке остановить
   остальные. О каждой поломке сообщается один раз: сыпать одинаковыми
   сообщениями по десять раз в секунду бессмысленно. */
const brokenPanels = new Set();

function guarded(name, fn) {
  try {
    fn();
  } catch (e) {
    console.error(`Ошибка отрисовки: ${name}`, e);
    if (!brokenPanels.has(name)) {
      brokenPanels.add(name);
      toast('err', `Панель «${name}» не отрисована`,
        `${e.message}. Остальные панели продолжают работать.`);
    }
  }
}

function setState(state) {
  const map = {
    idle: 'не запущена', running: 'работает', paused: 'пауза',
    stopped: 'остановлена', ended: 'завершена',
  };
  const n = $('#stat-state');
  n.textContent = map[state] || state;
  n.style.color = state === 'running' ? 'var(--normal)'
                : state === 'paused'  ? 'var(--warning)'
                : 'var(--text-dim)';

  $('#btn-pause').disabled  = state !== 'running';
  $('#btn-resume').disabled = state !== 'paused';
  $('#btn-step').disabled   = state !== 'paused';

  // Кнопка старта доступна всегда: во время прогона она означает «начать
  // заново». Раньше она выглядела рабочей, но команда уходила в очередь
  // и терялась по завершении прогона.
  $('#btn-start').textContent = (state === 'running' || state === 'paused')
    ? 'Restart' : 'Start';
}

function updateBanner(s) {
  const b = $('#mode-banner');
  if (s.directMode) {
    b.className = 'direct';
    b.textContent = 'DIRECT STATE OVERRIDE — значения подменяются напрямую, ' +
      'физическая причинность на этих величинах нарушена';
  } else if (s.unsafeMode) {
    b.className = '';
    b.textContent = 'UNSAFE OVERRIDES — разрешён выход за допустимые пределы параметров';
  } else {
    b.className = 'hidden';
    b.textContent = '';
  }
  // Высоту раскладки пересчитывает updateFlightBanner: строк может быть две.
}

/* Состояние полёта отдельной строкой.

   Остановленный турбонасос гасит двигатель, и ракета минуту падает с нулевой
   тягой. Числа при этом меняются мало, и без явного сообщения происходящее
   неотличимо от зависшего интерфейса. */
function updateFlightBanner(s) {
  const b = $('#flight-banner');
  const t = s.telemetry || {};
  const thrust = (t.propulsion && t.propulsion.totalThrust) || 0;
  const burning = ['First Stage Burn', 'Second Stage Burn', 'Circularization']
    .includes(s.phase);
  const falling = t.verticalVelocity < -5 && t.altitude > 0;

  let cls = '', text = '';

  if (t.crashed || s.runState === 'ended') {
    cls = 'ended';
    text = `Прогон завершён на T+${num(s.modelTime, 1)} с: носитель столкнулся ` +
      `с поверхностью. Нажмите Start, чтобы начать новый прогон.`;
  } else if (s.runState === 'stopped') {
    cls = 'ended';
    text = 'Прогон остановлен оператором. Start начинает новый прогон.';
  } else if (s.runState === 'paused') {
    cls = 'warn';
    text = `Пауза на T+${num(s.modelTime, 1)} с. Модельное время не идёт; ` +
      `Step продвигает модель по шагам, Resume возобновляет ход.`;
  } else if (burning && s.enginesOut > 0 && thrust > 1000) {
    // Отказ части двигателей: бортовой контур форсирует оставшиеся,
    // но потерю тяги он компенсирует лишь частично.
    //
    // "Ступень будет работать дольше" верно только когда тяги всё ещё
    // хватает на подъём (TWR ≥ 1). Если носитель ещё на столе и форсирования
    // не хватает даже до веса, честнее сказать, что взлёта не будет вовсе —
    // иначе зажигание одного двигателя из тридцати трёх выглядит как
    // "ничего не произошло", хотя на деле он честно работает на пределе.
    const grounded = t.altitude <= 0;
    const twrShort = typeof t.twr === 'number' && t.twr < 1;
    cls = grounded && twrShort ? 'crit' : 'warn';
    text = `Не создают тяги двигателей: ${s.enginesOut}. Оставшиеся форсированы ` +
      `до ${num(s.engineThrottle * 100, 1)} % при требуемых ` +
      `${num(s.throttleDemand * 100, 1)} %` +
      (grounded && twrShort
        ? `; тяговооружённость ${num(t.twr, 2)} — тяги не хватает для взлёта, ` +
          `носитель останется на столе.`
        : s.thrustDeficit > 0.001
        ? `; недобор тяги ${num(s.thrustDeficit * 100, 1)} % — запас форсирования ` +
          `исчерпан, ступень будет работать дольше.`
        : `; потеря скомпенсирована полностью.`);
  } else if (burning && thrust < 1000) {
    cls = 'crit';
    text = `Двигатели не создают тяги на T+${num(s.modelTime, 1)} с` +
      (falling ? `, носитель снижается со скоростью ${num(-t.verticalVelocity, 0)} м/с. `
               : '. ') +
      `«Штатный режим» снимает воздействия и запускает двигатели заново.`;
  } else if (falling && s.phase !== 'Orbital') {
    cls = 'warn';
    text = `Носитель снижается: вертикальная скорость ${num(t.verticalVelocity, 0)} м/с.`;
  }

  b.textContent = text;
  b.className = text ? cls : 'hidden';

  const layout = $('#layout');
  const banners = (text ? 1 : 0) + ($('#mode-banner').classList.contains('hidden') ? 0 : 1);
  layout.classList.toggle('with-banner', banners === 1);
  layout.classList.toggle('with-banner2', banners === 2);
}

/* Накопление рядов для графиков. Интерфейс хранит только то, что прислал
   сервер: ни одно значение здесь не вычисляется. */
function collectSeries(s) {
  const t = s.modelTime;
  const sample = sampleFromSnapshot(s);
  for (const [k, v] of Object.entries(sample)) {
    if (!Number.isFinite(v)) continue;
    let arr = S.series.get(k);
    if (!arr) { arr = []; S.series.set(k, arr); }
    if (arr.length && arr[arr.length - 1].t === t) continue;
    if (arr.length && arr[arr.length - 1].t > t) arr.length = 0; // прогон сброшен
    arr.push({ t, v });
    if (arr.length > 4000) arr.shift();
  }
}

/* Снимок величин собирается из тех же полей, что показывает сервер. */
function sampleFromSnapshot(s) {
  const out = {};
  for (const v of s.values || []) {
    if (v.actual !== null && v.actual !== undefined) out[readsOf(v.id)] = v.actual;
  }
  const p = s.telemetry && s.telemetry.propulsion;
  if (!p) return out;
  const e = p.engine || {};
  Object.assign(out, {
    'turbopump.rpm': e.shaftRpm,
    'turbopump.fuelPumpOutlet': e.fuelPumpOutlet / 1e6,
    'turbopump.oxPumpOutlet': e.oxPumpOutlet / 1e6,
    'turbopump.fuelPumpInlet': e.fuelPumpInlet / 1e3,
    'turbopump.oxPumpInlet': e.oxPumpInlet / 1e3,
    'turbopump.turbinePower': e.turbinePower / 1e6,
    'turbopump.vibration': e.vibration,
    'turbopump.bearingTemperature': e.bearingTemperature,
    'turbopump.fuelCavitationMargin': e.fuelCavitationMargin,
    'turbopump.oxCavitationMargin': e.oxCavitationMargin,
    'turbopump.powerMargin': e.powerMargin / 1e6,
    'engine.fuelFlow': p.totalFuelFlow,
    'engine.oxFlow': p.totalOxFlow,
    'engine.thrust': p.totalThrust / 1e3,
    'engine.specificImpulse': p.specificImpulse,
    'chamber.mixtureRatio': e.mixtureRatio,
    'chamber.pressure': e.chamberPressureMean / 1e6,
    'chamber.temperature': e.chamberTemperature,
    'chamber.cstarEfficiency': e.cstarEfficiency,
    'chamber.stabilityMargin': e.stabilityMargin,
    'chamber.pressureRMS': e.pressureRms / 1e3,
    'nozzle.wallMargin': e.wallMargin,
    'nozzle.coolingFlow': e.coolingFlow,
    'nozzle.throatErosion': e.throatErosion,
    'fuelTank.mass': p.fuelTank ? p.fuelTank.mass / 1000 : undefined,
    'oxTank.mass': p.oxTank ? p.oxTank.mass / 1000 : undefined,
    'fuelTank.pressure': p.fuelTank ? p.fuelTank.pressure / 1e3 : undefined,
    'oxTank.pressure': p.oxTank ? p.oxTank.pressure / 1e3 : undefined,
    'vehicle.altitude': s.telemetry.altitude / 1000,
    'vehicle.velocity': s.telemetry.totalVelocity,
    'vehicle.mass': s.telemetry.totalMass / 1000,
    'vehicle.acceleration': s.telemetry.totalMass
      ? (s.telemetry.totalThrust - s.telemetry.drag) / (s.telemetry.totalMass * 9.80665)
      : undefined,
  });
  return out;
}

function readsOf(id) {
  const p = S.params.get(id);
  return p ? p.reads : id;
}

/* ===========================================================================
   Панель насосов
   =========================================================================== */

/* ===========================================================================
   Управление тягой

   Уровень тяги — единственная величина, которой оператор пользуется постоянно,
   и до сих пор её приходилось задавать через общий список параметров. Здесь
   она вынесена отдельно: ползунок на всю ступень, несколько уставок и «погасить
   всё».

   Ползунок не перестраивается, пока за него держатся: телеметрия приходит
   несколько раз в секунду, а полная перерисовка отрывала бы ручку из-под
   курсора. Ровно на этом уже спотыкались заправка и плавники.
   =========================================================================== */

const THR = {
  slider: null, value: null, facts: null,
  active: false,    // за ползунок держится указатель
  manual: false,    // уставка задана оператором
  floor: 0.4,       // нижний предел дросселирования, доля
};

const sendThrottle = throttled(fraction => {
  submit({ parameter: 'engine.throttle', mode: 'set', basis: 'absolute',
           value: fraction });
}, 120);

function buildThrottlePanel() {
  const box = $('#throttle-panel');
  if (!box) return;

  const row = el('div', 'throttle-row');

  const slider = document.createElement('input');
  slider.type = 'range';
  // Ползунок ходит от нуля: аттестованный нижний предел (THR.floor,
  // приходит с телеметрией) — это где кончается штатный режим, а не где
  // кончается физическая возможность. Ниже него горение обычно срывается,
  // и слайдер туда пускает нарочно — оператору иногда и нужно проверить,
  // удержится ли камера на экстремально низком газе, а не просто узнать
  // об этом из документации.
  slider.min = 0; slider.max = 100; slider.step = 0.1; slider.value = 100;
  row.append(slider);

  // Поле рядом со слайдером — тот же приём, что и в общей панели параметров:
  // ползунком удобно грубо прикинуть, а точное значение вроде 98.2 %
  // проще набрать с клавиатуры, чем поймать мышью.
  const field = document.createElement('input');
  field.type = 'number';
  field.className = 'throttle-field';
  field.min = 0; field.max = 100; field.step = 0.1; field.value = 100;
  row.append(field);
  row.append(el('span', 'throttle-unit', '%'));
  box.append(row);

  const facts = el('div', 'throttle-facts', '');
  box.append(facts);

  const setBoth = v => {
    const s = (Math.round(v * 10) / 10).toFixed(1);
    slider.value = s;
    field.value = s;
  };

  const buttons = el('div', 'throttle-buttons');
  // Уставка null означает «на нижний предел»: он свой у каждой ступени
  // и приходит с телеметрией.
  const preset = (title, percent, hint) => {
    const b = el('button', 'btn btn-sm', title);
    b.title = hint;
    b.onclick = () => {
      const v = percent === null ? THR.floor * 100 : percent;
      setBoth(v);
      THR.manual = true;
      sendThrottle(v / 100);
    };
    buttons.append(b);
  };

  preset('Полная', 100, 'Номинальный режим камер');
  preset('70 %', 70, 'Обычное дросселирование на участке максимального напора');
  preset('Малый газ', null,
    'Нижний предел камеры: у Raptor аттестованный диапазон — от сорока ' +
    'процентов до номинала. Ниже него горение срывается, и это не режим, ' +
    'а погасание');

  const ignite = el('button', 'btn btn-sm btn-go', 'Зажечь двигатели');
  ignite.title = 'Запускает все камеры ступени в обход наведения — например, ' +
    'на орбите, где сама автоматика газ не даёт. Тягу после зажигания задайте ' +
    'ползунком или пресетом';
  ignite.onclick = igniteAll;
  buttons.append(ignite);

  const off = el('button', 'btn btn-sm btn-crit', 'Погасить все двигатели');
  off.title = 'Снимает команду со всех камер ступени и обнуляет уставку тяги';
  off.onclick = () => {
    action('emergency-shutdown');
    setBoth(0);
    THR.manual = true;
    toast('warn', 'Двигатели погашены',
      'Все камеры ступени выключены. Вернуть их в работу — «Штатный режим».');
  };
  buttons.append(off);

  const auto = el('button', 'btn btn-sm btn-go', 'Вернуть автомату');
  auto.title = 'Снимает ручную уставку: тягой снова распоряжается наведение';
  auto.onclick = () => {
    THR.manual = false;
    submit({ parameter: 'engine.throttle', mode: 'release' });
  };
  buttons.append(auto);

  box.append(buttons);

  slider.oninput = () => {
    field.value = slider.value;
    THR.manual = true;
    sendThrottle(Number(slider.value) / 100);
  };
  const grab = () => { THR.active = true; };
  const drop = () => { THR.active = false; };
  slider.addEventListener('pointerdown', grab);
  slider.addEventListener('keydown', grab);
  slider.addEventListener('pointerup', drop);
  slider.addEventListener('pointercancel', drop);
  slider.addEventListener('blur', drop);
  slider.addEventListener('change', drop);

  // Поле принимает точное значение по Enter/потере фокуса, а не на каждое
  // нажатие клавиши — иначе половина введённого числа улетала бы отдельной
  // командой, пока оператор ещё набирает следующую цифру.
  field.addEventListener('change', () => {
    let v = Number(field.value);
    if (!Number.isFinite(v)) { field.value = slider.value; return; }
    v = Math.max(0, Math.min(100, v));
    setBoth(v);
    THR.manual = true;
    sendThrottle(v / 100);
  });

  THR.slider = slider;
  THR.field = field;
  THR.facts = facts;
}

function updateThrottlePanel() {
  if (!THR.slider) return;

  const t = S.snapshot?.telemetry;
  if (!t) return;

  // Пока за ползунок или поле держатся, телеметрия их не двигает.
  //
  // Значения input'ов — не текст: toFixed даёт точку разделителем, как того
  // требует HTML, а не запятую из num(), которую браузер молча отбросит.
  if (!THR.active && !THR.manual) {
    const percent = ((t.throttle ?? 0) * 100).toFixed(1);
    THR.slider.value = percent;
    THR.field.value = percent;
  }
  THR.field.classList.toggle('manual', THR.manual);

  // Нижний предел камеры — свойство ступени, и после разделения он другой.
  // Ползунок ниже него не ограничивается: это аттестованный предел штатной
  // работы, а не физический потолок ввода, — оператор волен попробовать
  // удержать камеру ниже него и увидеть, выдержит она или погаснет.
  if (t.throttleFloor > 0 && t.throttleFloor !== THR.floor) {
    THR.floor = t.throttleFloor;
  }

  const running = t.enginesRunning ?? 0;
  const total = (t.engines || []).length;

  THR.facts.textContent =
    `фактически ${num((t.throttle ?? 0) * 100, 0)} % · ` +
    `тяга ${num((t.totalThrust || 0) / 1e6, 2)} МН · ` +
    `работают ${running} из ${total} · ` +
    `перегрузка ${num(t.totalMass ? (t.totalThrust - (t.drag || 0)) /
      (t.totalMass * 9.80665) : 0, 2)} g · ` +
    `предел камеры ${num(THR.floor * 100, 0)} %` +
    (t.throttleLimited ? ' (уставка удержана на пределе)' : '') +
    (THR.manual ? ' · уставка ручная' : ' · уставку держит наведение');
}

/* ===========================================================================
   Турбонасосный агрегат: быстрые воздействия

   Прежний набор был свалкой из пятнадцати кнопок с английскими подписями,
   половина которых делала одно и то же с разным шагом, а разницу между
   «Stop pump» и «Stuck shaft» нельзя было понять, не читая код. Теперь
   воздействия разложены по смыслу и подписаны тем, что они делают с железом.
   =========================================================================== */

const PUMP_GROUPS = [
  {
    name: 'Режим',
    actions: [
      { label: 'Обороты −5 %', hint: 'Плавно снизить уставку оборотов вала на пять процентов',
        build: () => relative('tp.shaft.speed', -5) },
      { label: 'Обороты +5 %', hint: 'Плавно поднять уставку оборотов вала на пять процентов',
        build: () => relative('tp.shaft.speed', 5) },
      { label: 'Задать обороты…', hint: 'Ввести уставку в оборотах в минуту',
        build: () => askRPM() },
      { label: 'Плавный переход…',
        hint: 'Перевести вал к новым оборотам за заданное время, а не скачком',
        build: () => askRamp() },
      { label: 'Половина оборотов',
        hint: 'Скачком до половины номинала: видно, как падает давление в камере',
        build: () => ({ parameter: 'tp.shaft.speed', mode: 'step',
                        basis: 'percent_nominal', value: 50 }) },
      { label: 'Импульс +15 % на 5 с',
        hint: 'Кратковременный заброс оборотов с возвратом к номиналу',
        build: () => ({ parameter: 'tp.shaft.speed', mode: 'timed', curve: 'pulse',
                        basis: 'percent_current', value: 15, duration: 5,
                        after: 'revert_nominal' }) },
    ],
  },
  {
    name: 'Отказы',
    actions: [
      { label: 'Обесточить вал', crit: true,
        hint: 'Газ на турбину не подаётся. Вал не встаёт мгновенно: его тормозит ' +
              'нагрузка насосов, обороты падают за секунды, давление в камере ' +
              'уходит следом',
        build: () => fail('tp.shaft.speed', 'stopped') },
      { label: 'Заклинить вал', crit: true,
        hint: 'Вал застопорен: обороты фиксируются на месте, момент турбины ' +
              'уходит в трение. В отличие от обесточивания подача прекращается ' +
              'сразу, а подшипники греются',
        build: () => fail('tp.shaft.speed', 'stuck') },
      { label: 'Кавитация горючего', warn: true,
        hint: 'Газ на входе насоса горючего: подача срывается, растёт вибрация',
        build: () => ({ parameter: 'tp.fuel_pump.gas', mode: 'step', value: 0.35 }) },
      { label: 'Кавитация окислителя', warn: true,
        hint: 'То же со стороны окислителя: смесь уходит в сторону горючего',
        build: () => ({ parameter: 'tp.ox_pump.gas', mode: 'step', value: 0.35 }) },
    ],
  },
  {
    name: 'Возврат',
    actions: [
      { label: 'Штатный режим', restore: true, go: true,
        hint: 'Снять все воздействия и вернуть установку под управление модели' },
    ],
  },
];

function relative(parameter, percent, mode = 'relative') {
  return { parameter, mode, basis: 'percent_current', value: percent };
}
function fail(parameter, failure) {
  return { parameter, mode: 'failure', failure };
}

function askRPM() {
  const now = S.snapshot?.pumps?.[0]?.actualRpm ?? 30000;
  const v = prompt('Задать обороты вала, об/мин:', Math.round(now));
  if (v === null) return null;
  const value = parseFloat(v.replace(',', '.'));
  if (!Number.isFinite(value)) return null;
  return { parameter: 'tp.shaft.speed', mode: 'set', value };
}

function askRamp() {
  const now = S.snapshot?.pumps?.[0]?.actualRpm ?? 30000;
  const v = prompt('Конечные обороты, об/мин:', Math.round(now * 1.1));
  if (v === null) return null;
  const d = prompt('За сколько секунд модельного времени?', '6');
  if (d === null) return null;
  const value = parseFloat(v.replace(',', '.'));
  const duration = parseFloat(d.replace(',', '.'));
  if (!Number.isFinite(value) || !Number.isFinite(duration) || duration <= 0) return null;
  return { parameter: 'tp.shaft.speed', mode: 'ramp', curve: 'smooth', value, duration };
}

function renderPumpButtons() {
  const box = $('#pump-buttons');
  box.replaceChildren();

  for (const group of PUMP_GROUPS) {
    const row = el('div', 'pump-group');
    row.append(el('span', 'pump-group-name', group.name));

    for (const a of group.actions) {
      const b = el('button',
        'btn btn-sm' + (a.crit ? ' btn-crit' : a.warn ? ' btn-warn' : a.go ? ' btn-go' : ''),
        a.label);
      b.title = a.hint || '';
      b.onclick = () => {
        if (a.restore) {
          action('restore-nominal');
          THR.manual = false;
          toast('ok', 'Штатный режим', 'Все воздействия сняты');
          S.markers.push({ t: S.snapshot?.modelTime ?? 0, label: 'Возврат в штатный режим' });
          showPumpCharts();
          return;
        }
        const cmd = a.build();
        if (cmd) submit(cmd);
        showPumpCharts();
      };
      row.append(b);
    }
    box.append(row);
  }

  // Разницу между двумя отказами вала стоит держать на виду: по названиям
  // она не читается, а ведут они себя по-разному.
  box.append(el('div', 'hint',
    'Обесточить вал — снять газ с турбины: вал тормозится нагрузкой насосов ' +
    'и останавливается за секунды, подача падает постепенно. Заклинить вал — ' +
    'застопорить его на месте: подача прекращается сразу, а момент турбины ' +
    'уходит в трение и греет подшипники.'));
}

/* Сводка по всем двигателям блока.

   Каждый двигатель — отдельное изделие со своим турбонасосом, поэтому
   отказ одного агрегата обязан быть виден на фоне остальных восьми. */
function renderEngineStrip() {
  const box = $('#engine-strip');
  const engines = S.snapshot?.engines || [];
  if (!engines.length) {
    box.replaceChildren(el('div', 'muted', 'Нет данных'));
    return;
  }

  box.replaceChildren();
  for (const e of engines) {
    const state = (e.state || '').split(' ')[0];
    const n = el('div',
      `eng s-${state}${e.manual ? ' manual' : ''}` +
      `${S.target === e.id ? ' selected' : ''}${e.running ? '' : ' off'}`);

    const head = el('div', 'eng-head');
    head.append(el('span', 'eng-id', e.id));
    if (e.manual) head.append(el('span', 'eng-flag', 'ручное'));
    else if (e.primary) head.append(el('span', 'eng-flag', 'спектр'));
    n.append(head);

    n.append(el('div', 'eng-rpm', num(e.shaftRpm, 0)));
    n.append(el('div', 'eng-sub',
      `${num(e.thrust, 0)} кН · ${num(e.chamberPressure, 1)} МПа`));
    n.append(el('div', 'eng-state', stateName(e.state)));

    // Погашенный двигатель можно зажечь точечно, не трогая остальные.
    // Сама тяга при этом не появляется: клапан выходит на холостой режим,
    // а уровень газа задаётся отдельно, как и при общем зажигании ступени.
    //
    // Обработчик клика — не здесь. Телеметрия приходит каждые 100 мс,
    // и весь блок перестраивается заново на каждый снимок: между нажатием
    // и отпусканием кнопки узел, на который повесили onclick, успевал
    // замениться на свежий с тем же именем, и клик терялся — снаружи это
    // выглядело как «кнопка иногда не работает». Ниже, в bindControls(),
    // висит один делегированный слушатель на #engine-strip — сам контейнер
    // не пересоздаётся никогда, и клик долетает независимо от того,
    // сколько раз плитки внутри перерисовались между нажатием и отпусканием.
    if (!e.running) {
      const ignite = el('button', 'eng-ignite', 'Зажечь');
      ignite.dataset.igniteEngine = e.id;
      ignite.title = `Запустить только ${e.id}, минуя наведение`;
      n.append(ignite);
    }

    // Щелчок по остальной части плитки выбирает адресата команд —
    // тоже через делегированный слушатель, той же причины ради.
    n.dataset.selectEngine = e.id;
    box.append(n);
  }

  // Список адресатов.
  const sel = $('#engine-target');
  const want = ['', ...engines.map(e => e.id)];
  const have = [...sel.options].map(o => o.value);
  if (want.join() !== have.join()) {
    sel.replaceChildren();
    sel.append(new Option('Все двигатели ступени', ''));
    for (const e of engines) sel.append(new Option(e.id, e.id));
  }
  sel.value = S.target;
}

function setTarget(id) {
  S.target = S.target === id ? '' : id;
  $('#engine-target').value = S.target;
  renderEngineStrip();
  renderPumps();
  drawAttitude();
  $('#target-hint').textContent = S.target
    ? `Команды адресуются двигателю ${S.target}. Остальные продолжают работать ` +
      `штатно. Внутри агрегата вал общий: насосы горючего и окислителя сидят ` +
      `на одном валу, и раскрутить их порознь физически нельзя.`
    : 'Команды применяются ко всем двигателям ступени. Внутри агрегата вал ' +
      'общий: насосы горючего и окислителя сидят на одном валу, и раскрутить ' +
      'их порознь физически нельзя.';
}

function renderPumps() {
  const grid = $('#pump-grid');
  const all = S.snapshot?.pumps || [];
  const shown = S.target || S.snapshot?.selected || '';
  const pumps = all.filter(p => p.engine === shown);

  if (!pumps.length) { grid.replaceChildren(el('div', 'muted', 'Нет данных')); return; }

  grid.replaceChildren();
  for (const p of pumps) {
    const card = el('div', 'pump');

    const head = el('div', 'pump-title');
    head.append(el('b', null, `${p.name} · ${p.engine}`));
    head.append(el('span', `pump-state st-${(p.state || '').split(' ')[0]}`,
      stateName(p.state)));
    card.append(head);

    const rpm = el('div', 'rpm-block');
    rpm.append(cell('Target', p.targetRpm, 0, 'target'));
    rpm.append(cell('Commanded', p.commandedRpm, 0));
    rpm.append(cell('Actual', p.actualRpm, 0, 'actual'));
    rpm.append(cell('Measured', p.measuredRpm, 0));
    card.append(rpm);

    const rows = el('div', 'pump-rows');
    row(rows, 'Вход',        num(p.inletPressure, 0), 'кПа');
    row(rows, 'Выход',       num(p.outletPressure, 2), 'МПа');
    row(rows, 'Напор',       num(p.head, 0), 'м');
    row(rows, 'Расход',      num(p.massFlow, 1), 'кг/с');
    row(rows, 'Мощность',    num(p.power, 2), 'МВт');
    row(rows, 'КПД',         num(p.efficiency, 3), '');
    row(rows, 'Вибрация',    num(p.vibration, 2), 'g',
        p.vibration > 15 ? 'crit' : p.vibration > 6 ? 'warn' : '');
    row(rows, 'Подшипники',  num(p.bearingTemperature, 0), 'К',
        p.bearingTemperature > 500 ? 'crit' : p.bearingTemperature > 420 ? 'warn' : '');
    row(rows, 'Запас NPSH',  num(p.cavitationMargin, 1), 'м',
        p.cavitationMargin <= 0 ? 'crit' : p.cavitationMargin < 3 ? 'warn' : '');
    row(rows, 'Потребный',   num(p.npshRequired, 1), 'м');
    card.append(rows);

    grid.append(card);
  }

  function cell(label, value, digits, cls) {
    const c = el('div', `rpm-cell ${cls || ''}`);
    c.append(el('span', null, label));
    c.append(el('b', null, num(value, digits)));
    return c;
  }
  function row(parent, label, value, unit, cls) {
    const r = el('div', `prow ${cls || ''}`);
    r.append(el('span', null, label));
    r.append(el('b', null, `${value}${unit ? ' ' + unit : ''}`));
    parent.append(r);
  }
}

/* После команды турбонасосу графики переключаются на связанные величины. */
function showPumpCharts() {
  S.chartKeys = PUMP_CHART_KEYS.slice();
  switchDataView('charts');
  buildCharts();
}

/* ===========================================================================
   Ориентация корпуса
   ===========================================================================

   Трёхмерное представление положения носителя. Углы приходят готовыми из
   модели: тангаж — возвышение продольной оси над местным горизонтом,
   рыскание — азимут от севера, крен — поворот вокруг продольной оси.
   Здесь они только рисуются, никакой физики не пересчитывается. */

const RAD = Math.PI / 180;

/* Поворот вектора на углы Эйлера в том же порядке, в каком их задаёт модель:
   сперва азимут вокруг местной вертикали, затем тангаж, затем крен вокруг
   получившейся продольной оси. */
function bodyAxes(pitch, yaw, roll) {
  const cp = Math.cos(pitch * RAD), sp = Math.sin(pitch * RAD);
  const cy = Math.cos(yaw * RAD),   sy = Math.sin(yaw * RAD);
  const cr = Math.cos(roll * RAD),  sr = Math.sin(roll * RAD);

  // Локальный базис: x — восток, y — север, z — вверх.
  const fwd = [cp * sy, cp * cy, sp];

  // Правая нормаль до крена: горизонтальна, перпендикулярна курсу.
  const right0 = [cy, -sy, 0];
  const up0 = cross(fwd, right0);

  const right = add(scale(right0, cr), scale(up0, -sr));
  const up = cross(fwd, right);
  return { fwd, right, up };
}

const cross = (a, b) => [
  a[1] * b[2] - a[2] * b[1],
  a[2] * b[0] - a[0] * b[2],
  a[0] * b[1] - a[1] * b[0],
];
const add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
const scale = (a, k) => [a[0] * k, a[1] * k, a[2] * k];

/* Изометрическая проекция: восток вправо-вниз, север вправо-вверх,
   вертикаль вверх. Наглядно и не требует библиотек. */
function project(v, cx, cy, k) {
  const [e, n, u] = v;
  return [
    cx + (e * 0.866 + n * 0.866) * k,
    cy - (u - e * 0.5 + n * 0.5) * k,
  ];
}

function drawAttitude() {
  const canvas = $('#attitude-canvas');
  const t = S.snapshot?.telemetry;
  if (!canvas || !t) return;

  const dpr = window.devicePixelRatio || 1;
  const w = canvas.clientWidth || 240, h = 190;
  if (canvas.width !== w * dpr || canvas.height !== h * dpr) {
    canvas.width = w * dpr; canvas.height = h * dpr;
  }
  const ctx = canvas.getContext('2d');
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, w, h);

  const cx = w / 2, cy = h / 2 + 10, k = Math.min(w, h) * 0.30;
  const { fwd, right, up } = bodyAxes(t.pitch || 0, t.yaw || 0, t.roll || 0);

  const line = (a, b, color, width = 1) => {
    const p = project(a, cx, cy, k), q = project(b, cx, cy, k);
    ctx.strokeStyle = color; ctx.lineWidth = width;
    ctx.beginPath(); ctx.moveTo(p[0], p[1]); ctx.lineTo(q[0], q[1]); ctx.stroke();
  };

  // Местный горизонт: сетка в плоскости восток–север.
  ctx.strokeStyle = '#202936'; ctx.lineWidth = 1;
  for (let i = -2; i <= 2; i++) {
    const f = i / 2;
    line([f, -1, 0], [f, 1, 0], '#1c2530');
    line([-1, f, 0], [1, f, 0], '#1c2530');
  }
  line([0, 0, 0], [1.15, 0, 0], '#33414f');  // восток
  line([0, 0, 0], [0, 1.15, 0], '#33414f');  // север
  line([0, 0, 0], [0, 0, 1.15], '#3a4a5c');  // вертикаль

  ctx.fillStyle = '#5d6b7d';
  ctx.font = '9px monospace';
  const lbl = (v, text) => {
    const p = project(v, cx, cy, k);
    ctx.fillText(text, p[0] + 2, p[1] - 2);
  };
  lbl([1.15, 0, 0], 'В'); lbl([0, 1.15, 0], 'С'); lbl([0, 0, 1.2], '↑');

  // Корпус: продольная ось с носом и хвостовым оперением.
  const nose = scale(fwd, 1.05), tail = scale(fwd, -0.55);
  line(tail, nose, '#8fc6ec', 3);

  // Поперечные оси в хвосте показывают крен.
  const finA = add(tail, scale(right, 0.28));
  const finB = add(tail, scale(right, -0.28));
  const finC = add(tail, scale(up, 0.28));
  const finD = add(tail, scale(up, -0.28));
  line(finA, finB, '#4a9edd', 2);
  line(finC, finD, '#2f6b8f', 2);

  // Нос.
  const p = project(nose, cx, cy, k);
  ctx.fillStyle = '#cfe6fa';
  ctx.beginPath(); ctx.arc(p[0], p[1], 3.5, 0, Math.PI * 2); ctx.fill();

  // Направление набегающего потока: именно расхождение с продольной осью
  // и есть угол атаки.
  if ($('#att-flow').checked) {
    const vh = t.groundSpeed || 0;
    const vv = t.verticalVelocity || 0;
    const speed = Math.hypot(vh, vv);
    if (speed > 1) {
      const gamma = Math.atan2(vv, vh);
      const az = (t.azimuth || 0) * RAD;
      const dir = [
        Math.cos(gamma) * Math.sin(az),
        Math.cos(gamma) * Math.cos(az),
        Math.sin(gamma),
      ];
      line([0, 0, 0], scale(dir, 1.0), '#d9a441', 2);
      const q = project(scale(dir, 1.0), cx, cy, k);
      ctx.fillStyle = '#d9a441';
      ctx.fillText('V', q[0] + 3, q[1]);
    }
  }

  // Значения углов.
  const box = $('#attitude-values');
  box.replaceChildren();
  const items = [
    ['Тангаж', t.pitch, '°'], ['Рыскание', t.yaw, '°'], ['Крен', t.roll, '°'],
    ['ωy', t.pitchRate, '°/с'], ['ωz', t.yawRate, '°/с'], ['ωx', t.rollRate, '°/с'],
    ['Угол атаки', t.angleOfAttack, '°'],
    ['Скольжение', t.sideslipAngle, '°'],
    ['Полный угол', t.totalAoA, '°'],
  ];
  for (const [title, v, unit] of items) {
    const d = el('div');
    d.append(el('span', null, title));
    d.append(el('b', null, `${num(v, 1)}${unit}`));
    box.append(d);
  }
}

/* ===========================================================================
   Ключевые показатели и предупреждения
   =========================================================================== */

function renderKeyValues() {
  const box = $('#key-values');
  const sample = S.snapshot ? sampleFromSnapshot(S.snapshot) : {};
  const manual = new Set((S.snapshot?.effects || []).map(e => readsOf(e.parameter)));
  const alarmed = new Map((S.snapshot?.alarms || []).map(a => [a.key, a.severity]));

  box.replaceChildren();

  for (const c of COMPENSATION) {
    const v = c.get(S.snapshot || {});
    if (!Number.isFinite(v)) continue;
    const n = el('div', `kv s-${c.bad(v) ? 'warning' : 'normal'}`);
    n.append(el('span', null, c.title));
    const b = el('b', null, num(v, c.digits));
    if (c.unit) b.append(el('i', null, c.unit));
    n.append(b);
    box.append(n);
  }

  for (const k of KEY_VALUES) {
    const v = sample[k.key];
    let status = 'normal';
    if (!Number.isFinite(v)) status = 'no_data';
    else if (alarmed.has(k.key)) status = alarmed.get(k.key);
    else if (manual.has(k.key)) status = 'manual';

    const c = el('div', `kv s-${status}${k.big ? ' big' : ''}`);
    c.append(el('span', null, k.title));
    const b = el('b', null, num(v, k.digits));
    if (k.unit) b.append(el('i', null, k.unit));
    c.append(b);
    box.append(c);
  }
}

function renderAlarms() {
  const box = $('#alarm-list');
  const alarms = S.snapshot?.alarms || [];
  if (!alarms.length) {
    box.replaceChildren(el('div', 'muted', 'Нет отклонений'));
    return;
  }
  box.replaceChildren();
  for (const a of alarms) {
    const n = el('div', `alarm ${a.severity}`);
    n.append(el('b', null, `${a.title}: ${num(a.value, 2)} ${a.unit}`));
    n.append(el('div', 'alarm-msg', a.message));
    box.append(n);
  }
}

/* ===========================================================================
   Причинно-следственная картина
   =========================================================================== */

function renderCausal() {
  const box = $('#causal');
  const applied = S.log.filter(e => e.status === 'applied' || e.status === 'released');
  const last = applied[applied.length - 1];

  if (!last) {
    box.replaceChildren(el('div', 'muted', 'Воздействий не было'));
    return;
  }

  box.replaceChildren();

  const head = el('div', 'causal-head');
  head.append(el('b', null, last.title || last.parameter));
  head.append(el('span', 'causal-when',
    `${clock(last.modelTime)} · ${num(last.previous, 2)} → ${num(last.target, 2)} ${last.unit || ''}`));
  if (last.unsafe) head.append(el('span', 'flag unsafe', 'unsafe'));
  if (last.direct) head.append(el('span', 'flag direct', 'direct'));
  box.append(head);

  const r = last.reaction;
  if (!r) {
    box.append(el('div', 'muted',
      'Отклик ещё регистрируется: модели нужно несколько секунд, ' +
      'чтобы изменения проявились.'));
    return;
  }

  // Цепочка строится по времени появления отклика в данных модели.
  const chain = el('div', 'chain');
  chain.append(el('div', 'chain-node first', last.title || last.parameter));
  for (const step of r.chain) {
    chain.append(el('span', 'chain-arrow', '→'));
    chain.append(el('div', 'chain-node', step));
  }
  box.append(chain);

  const table = el('table', 'resp-table');
  const thead = el('thead');
  const hr = el('tr');
  ['Величина', 'Было', 'Стало', 'Изменение', 'Отклик через', 'Связь'].forEach(h =>
    hr.append(el('th', null, h)));
  thead.append(hr);
  table.append(thead);

  const tbody = el('tbody');
  for (const resp of r.responses) {
    const tr = el('tr');
    tr.append(el('td', 'name', `${resp.title}, ${resp.unit || '—'}`));
    tr.append(el('td', null, num(resp.before, 3)));
    tr.append(el('td', null, num(resp.after, 3)));
    const pct = el('td', resp.change >= 0 ? 'up' : 'down',
      `${resp.change >= 0 ? '+' : ''}${num(resp.change * 100, 1)} %`);
    tr.append(pct);
    tr.append(el('td', null, `${num(resp.onset, 1)} с`));
    tr.append(el('td', 'name', resp.expected ? 'непосредственная' : 'косвенная'));
    tbody.append(tr);
  }
  table.append(tbody);
  box.append(table);

  if (r.silent && r.silent.length) {
    box.append(el('div', 'silent',
      'Ожидались, но не изменились: ' + r.silent.join(', ') +
      '. Модель этой связи здесь не подтвердила.'));
  }
}

/* ===========================================================================
   Активные воздействия
   =========================================================================== */

function renderEffects() {
  const box = $('#effects');
  const list = S.snapshot?.effects || [];
  if (!list.length) {
    box.replaceChildren(el('div', 'muted', 'Все параметры под управлением модели'));
    return;
  }

  box.replaceChildren();
  for (const e of list) {
    const n = el('div', `effect${e.unsafe ? ' unsafe' : ''}${e.direct ? ' direct' : ''}`);

    const left = el('div');
    left.append(el('div', 'effect-title',
      (e.title || e.parameter) + (e.engine ? ` · ${e.engine}` : ' · вся ступень')));
    const mode = e.mode === 'failure'
      ? `отказ: ${e.failure}`
      : `${e.mode}${e.curve && e.mode !== 'step' ? ' · ' + e.curve : ''}` +
        (e.remaining ? ` · осталось ${num(e.remaining, 1)} с` : '');
    left.append(el('div', 'effect-mode', mode));
    n.append(left);

    n.append(el('div', 'effect-num',
      e.mode === 'failure' ? '—' : `${num(e.current, 2)} / ${num(e.target, 2)} ${e.unit || ''}`));

    const b = el('button', 'btn btn-sm', 'Снять');
    b.onclick = () => submit({ parameter: e.parameter, mode: 'release' });
    n.append(b);

    box.append(n);
  }
}

/* ===========================================================================
   Левая панель: параметры
   =========================================================================== */

function renderSubsystemTabs() {
  const nav = $('#subsystem-tabs');
  nav.replaceChildren();
  for (const s of SUBSYSTEMS) {
    const b = el('button', `tab${S.subsystem === s.id ? ' active' : ''}`, s.title);
    b.onclick = () => { S.subsystem = s.id; renderSubsystemTabs(); renderParams(); };
    nav.append(b);
  }
}

const QUICK = {
  turbopump: [
    { l: 'Номинальные обороты', c: { parameter: 'tp.shaft.speed', mode: 'set', basis: 'percent_nominal', value: 100 } },
    { l: 'Обесточить агрегат', c: fail('tp.shaft.speed', 'unpowered'), crit: 1 },
    { l: 'Перегрев подшипников', c: { parameter: 'tp.bearing.cooling', mode: 'step', value: 0.15 }, warn: 1 },
    { l: 'Износ крыльчатки', c: { parameter: 'tp.fuel_pump.head', mode: 'ramp', value: 0.7, duration: 8 }, warn: 1 },
  ],
  valves: [
    { l: 'Прикрыть горючее до 70 %', c: { parameter: 'valve.fuel.command', mode: 'step', value: 0.7 } },
    { l: 'Заклинить клапан горючего', c: fail('valve.fuel.command', 'stuck'), crit: 1 },
    { l: 'Медленный привод', c: { parameter: 'valve.fuel.rate', mode: 'step', value: 0.15 }, warn: 1 },
    { l: 'Задержка реакции 0.5 с', c: { parameter: 'valve.fuel.delay', mode: 'step', value: 0.5 }, warn: 1 },
    { l: 'Ложное показание датчика', c: fail('valve.ox.command', 'bad_sensor'), warn: 1 },
  ],
  tanks: [
    { l: 'Потеря наддува горючего', c: { parameter: 'tank.fuel.pressurant_valve', mode: 'step', value: 0 }, crit: 1 },
    { l: 'Медленное падение давления', c: { parameter: 'tank.ox.pressurant_flow', mode: 'ramp', value: 0.2, duration: 20 }, warn: 1 },
    { l: 'Резкое падение давления', c: { parameter: 'tank.ox.pressurant_valve', mode: 'step', value: 0 }, crit: 1 },
    { l: 'Утечка горючего 20 кг/с', c: { parameter: 'tank.fuel.leak', mode: 'step', value: 20 }, warn: 1 },
    { l: 'Утечка окислителя 40 кг/с', c: { parameter: 'tank.ox.leak', mode: 'step', value: 40 }, warn: 1 },
    { l: 'Засорение магистрали', c: { parameter: 'tank.fuel.blockage', mode: 'ramp', value: 0.5, duration: 6 }, warn: 1 },
    { l: 'Разрыв магистрали', c: fail('tank.fuel.blockage', 'ruptured'), crit: 1 },
    { l: 'Раскачать жидкость', c: { parameter: 'tank.ox.slosh_excitation', mode: 'step', value: 8 }, warn: 1 },
    { l: 'Газ на входе насоса', c: { parameter: 'tp.ox_pump.gas', mode: 'step', value: 0.4 }, crit: 1 },
  ],
  chamber: [
    { l: 'Нестабильное горение', c: { parameter: 'chamber.stability_bias', mode: 'ramp', value: -0.6, duration: 5 }, crit: 1 },
    { l: 'Отклонение O/F вверх', c: { parameter: 'engine.mixture_trim', mode: 'step', value: 0.85 }, warn: 1 },
    { l: 'Недостаток горючего', c: { parameter: 'chamber.injector.fuel_area', mode: 'step', value: 0.6 }, warn: 1 },
    { l: 'Недостаток окислителя', c: { parameter: 'chamber.injector.ox_area', mode: 'step', value: 0.6 }, warn: 1 },
    { l: 'Засорение форсунок', c: { parameter: 'chamber.injector.fuel_area', mode: 'ramp', value: 0.7, duration: 10 }, warn: 1 },
    { l: 'Рост пульсаций', c: { parameter: 'chamber.oscillation', mode: 'ramp', value: 6, duration: 6 }, warn: 1 },
    { l: 'Падение давления камеры', c: { parameter: 'chamber.cstar_efficiency', mode: 'step', value: 0.7 }, crit: 1 },
    { l: 'Погасание двигателя', c: fail('engine.throttle', 'stopped'), crit: 1 },
    { l: 'Повторное воспламенение', c: { parameter: 'engine.throttle', mode: 'release' } },
  ],
  nozzle: [
    { l: 'Снижение расхода охлаждения', c: { parameter: 'nozzle.cooling_flow', mode: 'step', value: 0.5 }, warn: 1 },
    { l: 'Полная потеря охлаждения', c: fail('nozzle.cooling_flow', 'stopped'), crit: 1 },
    { l: 'Засорение каналов', c: { parameter: 'nozzle.cooling_conductance', mode: 'ramp', value: 0.4, duration: 8 }, warn: 1 },
    { l: 'Утечка охладителя', c: { parameter: 'nozzle.cooling_drop', mode: 'step', value: 3 }, warn: 1 },
    { l: 'Локальный перегрев', c: { parameter: 'nozzle.hot_spot', mode: 'step', value: 0.25 }, crit: 1 },
    { l: 'Ускоренная эрозия', c: { parameter: 'nozzle.erosion_rate', mode: 'step', value: 60 }, crit: 1 },
  ],
  engine: [
    { l: 'Дроссель 70 %', c: { parameter: 'engine.throttle', mode: 'step', value: 0.7 } },
    { l: 'Дроссель 40 %', c: { parameter: 'engine.throttle', mode: 'step', value: 0.4 }, warn: 1 },
    { l: 'Полная тяга', c: { parameter: 'engine.throttle', mode: 'step', value: 1 } },
    { l: 'Шум датчиков ×10', c: { parameter: 'engine.sensor_noise', mode: 'step', value: 10 }, warn: 1 },
    { l: 'Замер заморожен', c: fail('engine.sensor_noise', 'frozen'), warn: 1 },
    { l: 'Датчики не отвечают', c: fail('engine.sensor_noise', 'bad_sensor'), crit: 1 },
  ],
  control: [
    { l: 'Ход привода 40 %', c: { parameter: 'control.gimbal_limit', mode: 'step', value: 0.4 }, warn: 1 },
    { l: 'Заклинить привод', c: fail('control.gimbal_limit', 'stuck'), crit: 1 },
    { l: 'Обесточить привод', c: fail('control.gimbal_limit', 'unpowered'), crit: 1 },
    { l: 'Медленная перекладка', c: { parameter: 'control.gimbal_rate', mode: 'step', value: 0.25 }, warn: 1 },
    { l: 'Смещение привода 1°', c: { parameter: 'control.gimbal_bias', mode: 'step', value: 1 }, warn: 1 },
    { l: 'Раскачать автопилот', c: { parameter: 'control.bandwidth', mode: 'ramp', value: 2.5, duration: 8 }, crit: 1 },
    { l: 'Убрать демпфирование', c: { parameter: 'control.damping', mode: 'step', value: 0.2 }, crit: 1 },
    { l: 'Отключить ориентацию', c: { parameter: 'control.rcs', mode: 'step', value: 0 }, warn: 1 },
  ],
};

function renderQuickActions() {
  const box = $('#quick-actions');
  box.replaceChildren();
  for (const q of (QUICK[S.subsystem] || [])) {
    const b = el('button', 'btn btn-sm' + (q.crit ? ' btn-crit' : q.warn ? ' btn-warn' : ''), q.l);
    b.onclick = () => submit({ ...q.c });
    box.append(b);
  }
}

function renderParams() {
  renderQuickActions();

  const list = $('#param-list');
  const filter = $('#param-filter').value.trim().toLowerCase();
  list.replaceChildren();

  const params = [...S.params.values()].filter(p =>
    p.subsystem === S.subsystem &&
    (!filter || p.title.toLowerCase().includes(filter) || p.id.includes(filter)));

  let group = null;
  for (const p of params) {
    if (p.group !== group) {
      group = p.group;
      list.append(el('div', 'group-title', group));
    }
    list.append(paramCard(p));
  }
  if (!params.length) list.append(el('div', 'muted', 'Ничего не найдено'));
  renderParamValues();
}

function paramCard(p) {
  const card = el('div', `param${p.direct ? ' direct' : ''}`);
  card.dataset.id = p.id;

  /* --- заголовок --- */
  const head = el('div', 'param-head');
  head.append(el('div', 'dot s-normal'));
  head.append(el('div', 'param-name', p.title));
  head.append(el('div', 'param-now', '—'));
  head.append(el('div', 'param-unit', p.unit || ''));
  head.onclick = () => card.classList.toggle('open');
  card.append(head);

  /* --- тело --- */
  const body = el('div', 'param-body');

  body.append(el('div', 'reads-label muted', ''));

  const vals = el('div', 'values-line');
  for (const [label, cls] of [['Target', 'target'], ['Commanded', 'cmd'],
                              ['Actual', 'actual'], ['Measured', 'measured']]) {
    const d = el('div');
    d.append(el('span', null, label));
    d.append(el('b', `v-${cls}`, '—'));
    vals.append(d);
  }
  body.append(vals);

  const range = el('div', 'range-line');
  range.append(el('span', null, `допустимо ${num(p.min, 2)}…${num(p.max, 2)}`));
  range.append(el('span', null, `номинал ${num(p.nominal, 2)}`));
  body.append(range);

  /* Ввод: ползунок и поле рядом — точное значение всегда можно набрать
     с клавиатуры, ползунок лишь для грубой настройки. */
  const line = el('div', 'input-line');
  const slider = el('input');
  slider.type = 'range';
  slider.min = p.min; slider.max = p.max; slider.step = p.step;
  slider.value = p.nominal;

  const field = el('input');
  field.type = 'number';
  field.step = p.step;
  field.value = p.nominal;

  slider.oninput = () => { field.value = slider.value; };
  field.oninput = () => { slider.value = field.value; };

  const stepper = el('div', 'stepper');
  const minus = el('button', 'btn btn-sm', '−');
  const plus  = el('button', 'btn btn-sm', '+');
  minus.onclick = () => { field.value = (+field.value - p.step).toFixed(4); slider.value = field.value; };
  plus.onclick  = () => { field.value = (+field.value + p.step).toFixed(4); slider.value = field.value; };
  stepper.append(minus, plus);

  line.append(slider, field, stepper);
  body.append(line);

  /* --- режим воздействия --- */
  const modeLine = el('div', 'mode-line');

  const mode = el('select');
  [['set', 'Установить'], ['step', 'Скачком'], ['ramp', 'Плавно'],
   ['relative', 'Относительно'], ['timed', 'Временно'], ['failure', 'Отказ']]
    .forEach(([v, t]) => mode.append(new Option(t, v)));

  const basis = el('select');
  [['absolute', 'в единицах'], ['percent_current', '% от текущего'],
   ['percent_nominal', '% от номинала'], ['delta', 'приращение']]
    .forEach(([v, t]) => basis.append(new Option(t, v)));

  const curve = el('select');
  [['linear', 'линейно'], ['smooth', 'плавно'], ['staircase', 'ступенями'],
   ['pulse', 'импульс'], ['periodic', 'периодически']]
    .forEach(([v, t]) => curve.append(new Option(t, v)));

  const failure = el('select');
  [['stopped', 'остановлен'], ['stuck', 'заклинил'], ['unpowered', 'обесточен'],
   ['unresponsive', 'не отвечает'], ['frozen', 'фиксированное значение'],
   ['bad_sensor', 'ложная телеметрия'], ['ruptured', 'разрушен']]
    .forEach(([v, t]) => failure.append(new Option(t, v)));

  const duration = el('input'); duration.type = 'number'; duration.value = 5; duration.min = 0;
  duration.title = 'Длительность, с модельного времени';
  const period = el('input'); period.type = 'number'; period.value = 1; period.min = 0;
  period.title = 'Период, с';

  const after = el('select');
  [['revert_nominal', 'вернуть номинал'], ['revert_previous', 'вернуть прежнее'],
   ['hold', 'оставить']].forEach(([v, t]) => after.append(new Option(t, v)));

  modeLine.append(mode, basis, curve, duration, period, after, failure);
  body.append(modeLine);

  function syncMode() {
    const m = mode.value;
    basis.style.display    = (m === 'relative' || m === 'timed' || m === 'set' || m === 'step') ? '' : 'none';
    curve.style.display    = (m === 'ramp' || m === 'timed') ? '' : 'none';
    duration.style.display = (m === 'ramp' || m === 'timed') ? '' : 'none';
    period.style.display   = (m === 'ramp' || m === 'timed') ? '' : 'none';
    after.style.display    = (m === 'timed') ? '' : 'none';
    failure.style.display  = (m === 'failure') ? '' : 'none';
    line.style.display     = (m === 'failure') ? 'none' : '';
  }
  mode.onchange = syncMode;
  syncMode();

  /* --- действия --- */
  const actions = el('div', 'param-actions');

  const apply = el('button', 'btn btn-sm btn-go', 'Применить');
  apply.onclick = () => submit({
    parameter: p.id,
    mode: mode.value,
    basis: basis.value,
    curve: curve.value,
    value: parseFloat(field.value),
    duration: parseFloat(duration.value) || 0,
    period: parseFloat(period.value) || 0,
    after: after.value,
    failure: failure.value,
    direct: !!p.direct,
  });

  const auto = el('button', 'btn btn-sm', 'В автоматический режим');
  auto.onclick = () => submit({ parameter: p.id, mode: 'release' });

  const plot = el('button', 'btn btn-sm', 'На график');
  plot.onclick = () => {
    if (p.reads && !S.chartKeys.includes(p.reads)) {
      S.chartKeys.unshift(p.reads);
      S.chartKeys = S.chartKeys.slice(0, 16);
      buildCharts();
      switchDataView('charts');
    }
  };

  actions.append(apply, auto, plot);
  body.append(actions);

  if (p.consequence) {
    body.append(el('div', 'muted', p.consequence));
  }

  card.append(body);
  return card;
}

/* Обновление значений в карточках параметров без перерисовки разметки. */
function renderParamValues() {
  const values = new Map((S.snapshot?.values || []).map(v => [v.id, v]));
  for (const card of $$('#param-list .param')) {
    const v = values.get(card.dataset.id);
    if (!v) continue;

    card.classList.toggle('manual', v.manual);
    $('.dot', card).className = `dot s-${v.status}`;

    // В строке параметра показывается его собственное значение — то, чем
    // управляет оператор. Наблюдаемая величина уходит в тело карточки
    // и подписывается своим именем и своей размерностью: у множителя напора
    // это давление за насосом в мегапаскалях, и путать их нельзя.
    $('.param-now', card).textContent = num(v.setting, 2);

    $('.v-target', card).textContent   = num(v.target, 2);
    $('.v-cmd', card).textContent      = num(v.commanded, 2);
    $('.v-actual', card).textContent   = num(v.actual, 2);
    $('.v-measured', card).textContent = num(v.measured, 2);

    const label = $('.reads-label', card);
    if (label) {
      label.textContent = v.readsTitle
        ? `читает: ${v.readsTitle}${v.readsUnit ? ', ' + v.readsUnit : ''}`
        : '';
    }
  }
}

/* ===========================================================================
   Подтверждение и отправка
   =========================================================================== */

/* submit проверяет пределы на стороне интерфейса только ради предупреждения.
   Окончательное решение принимает сервер: он же и отклонит команду. */
function submit(cmd) {
  cmd.id = uuid();

  // Адресат подставляется только параметрам двигателя: бак на ступени один,
  // и адресовать его конкретному агрегату бессмысленно.
  const p0 = S.params.get(cmd.parameter);
  if (S.target && p0 && p0.subsystem !== 'tanks') cmd.engine = S.target;
  cmd.operator = 'operator';
  cmd.source = 'ui';
  cmd.issuedAtModelTime = S.snapshot?.modelTime ?? 0;
  cmd.ttl = 30;

  const p = S.params.get(cmd.parameter);
  const unsafe = p && cmd.mode !== 'release' && cmd.mode !== 'failure' &&
    cmd.basis === 'absolute' && (cmd.value < p.min || cmd.value > p.max);

  const needsConfirm = unsafe || cmd.mode === 'failure' || (p && p.direct);

  if (!needsConfirm) { sendCommand(cmd); return; }

  const where = cmd.engine ? `двигатель ${cmd.engine}` : 'все двигатели ступени';
  const title = unsafe ? 'Значение вне допустимых пределов'
              : cmd.mode === 'failure' ? `Имитация отказа: ${where}`
              : 'Прямая подмена состояния модели';

  let text;
  if (unsafe) {
    text = `${p.title}: ${num(cmd.value, 3)} ${p.unit} при допустимых ` +
      `${num(p.min, 3)}…${num(p.max, 3)} ${p.unit}. ` +
      `Требуется режим Unsafe overrides.`;
    cmd.unsafe = true;
  } else if (cmd.mode === 'failure') {
    text = `${p ? p.title : cmd.parameter}: перевод в состояние «${cmd.failure}».`;
    cmd.unsafe = true;
  } else {
    text = `${p.title}: значение будет записано в модель напрямую. ` +
      `Величина перестанет следовать из физических связей до снятия воздействия.`;
    cmd.direct = true;
  }

  askConfirm(title, text, p ? p.consequence : '', () => {
    if (unsafe && !S.snapshot?.unsafeMode) {
      fetch('/api/control/modes', {
        method: 'POST', headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ unsafe: true }),
      }).then(() => sendCommand(cmd));
      return;
    }
    if (cmd.direct && !S.snapshot?.directMode) {
      fetch('/api/control/modes', {
        method: 'POST', headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ direct: true }),
      }).then(() => sendCommand(cmd));
      return;
    }
    sendCommand(cmd);
  });
}

function askConfirm(title, text, consequence, onOk) {
  $('#confirm-title').textContent = title;
  $('#confirm-text').textContent = text;
  $('#confirm-consequence').textContent = consequence || '';
  $('#confirm-consequence').style.display = consequence ? '' : 'none';
  $('#confirm').classList.remove('hidden');
  S.pendingConfirm = onOk;
}

/* ===========================================================================
   Графики
   =========================================================================== */

function buildCharts() {
  const box = $('#charts');
  box.replaceChildren();
  const watches = new Map((S.registry?.watches || []).map(w => [w.key, w]));

  for (const key of S.chartKeys) {
    const w = watches.get(key) || { title: key, unit: '' };
    const c = el('div', 'chart');
    c.dataset.key = key;

    const head = el('div', 'chart-head');
    head.append(el('b', null, w.title));
    head.append(el('span', 'chart-val', '—'));
    c.append(head);

    const canvas = el('canvas');
    c.append(canvas);
    box.append(c);
  }
  drawCharts();
}

function drawCharts() {
  const windowSec = parseFloat($('#chart-window').value);
  const now = S.snapshot?.modelTime ?? 0;
  const watches = new Map((S.registry?.watches || []).map(w => [w.key, w]));

  for (const node of $$('#charts .chart')) {
    const key = node.dataset.key;
    const all = S.series.get(key) || [];
    const from = windowSec > 0 ? now - windowSec : -Infinity;
    const pts = all.filter(p => p.t >= from);

    const w = watches.get(key) || { unit: '' };
    const last = pts.length ? pts[pts.length - 1].v : NaN;
    $('.chart-val', node).textContent = `${num(last, 3)} ${w.unit || ''}`;

    drawSeries($('canvas', node), pts, from, now);
  }
}

function drawSeries(canvas, pts, from, to) {
  const dpr = window.devicePixelRatio || 1;
  const cssW = canvas.clientWidth || 300;
  const cssH = 74;
  if (canvas.width !== cssW * dpr || canvas.height !== cssH * dpr) {
    canvas.width = cssW * dpr;
    canvas.height = cssH * dpr;
  }
  const ctx = canvas.getContext('2d');
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, cssW, cssH);

  if (pts.length < 2) return;

  const t0 = Number.isFinite(from) ? from : pts[0].t;
  const t1 = Math.max(to, pts[pts.length - 1].t);
  const span = Math.max(t1 - t0, 1e-6);

  let lo = Infinity, hi = -Infinity;
  for (const p of pts) { if (p.v < lo) lo = p.v; if (p.v > hi) hi = p.v; }
  if (hi - lo < 1e-9) { hi += 1; lo -= 1; }
  const pad = (hi - lo) * 0.12;
  lo -= pad; hi += pad;

  const x = t => 4 + (t - t0) / span * (cssW - 8);
  const y = v => cssH - 6 - (v - lo) / (hi - lo) * (cssH - 12);

  // Сетка.
  ctx.strokeStyle = '#202936';
  ctx.lineWidth = 1;
  for (let i = 0; i <= 2; i++) {
    const yy = 6 + i * (cssH - 12) / 2;
    ctx.beginPath(); ctx.moveTo(0, yy); ctx.lineTo(cssW, yy); ctx.stroke();
  }

  // Отметки ручных вмешательств: вертикальная линия в момент команды.
  ctx.strokeStyle = 'rgba(74,158,221,.55)';
  ctx.setLineDash([3, 3]);
  for (const m of S.markers) {
    if (m.t < t0 || m.t > t1) continue;
    const xx = x(m.t);
    ctx.beginPath(); ctx.moveTo(xx, 2); ctx.lineTo(xx, cssH - 2); ctx.stroke();
  }
  ctx.setLineDash([]);

  // Сам ряд.
  ctx.strokeStyle = '#6fb8e8';
  ctx.lineWidth = 1.4;
  ctx.beginPath();
  pts.forEach((p, i) => (i ? ctx.lineTo(x(p.t), y(p.v)) : ctx.moveTo(x(p.t), y(p.v))));
  ctx.stroke();

  // Заливка под кривой.
  ctx.lineTo(x(pts[pts.length - 1].t), cssH);
  ctx.lineTo(x(pts[0].t), cssH);
  ctx.closePath();
  ctx.fillStyle = 'rgba(111,184,232,.08)';
  ctx.fill();

  // Подписи границ.
  ctx.fillStyle = '#5d6b7d';
  ctx.font = '9px monospace';
  ctx.fillText(num(hi, 2), 4, 10);
  ctx.fillText(num(lo, 2), 4, cssH - 2);
}

/* ===========================================================================
   Журнал
   =========================================================================== */

function renderLog() {
  const box = $('#log-table');
  const filter = $('#log-filter').value.trim().toLowerCase();
  const status = $('#log-status').value;

  const rows = S.log.filter(e =>
    (!status || e.status === status) &&
    (!filter ||
      (e.title || '').toLowerCase().includes(filter) ||
      (e.parameter || '').toLowerCase().includes(filter) ||
      (e.operator || '').toLowerCase().includes(filter)));

  const table = el('table', 'data');
  const thead = el('thead');
  const hr = el('tr');
  ['Время', 'Параметр', 'Режим', 'Было', 'Стало', 'Длит.', 'Оператор', 'Статус']
    .forEach(h => hr.append(el('th', null, h)));
  thead.append(hr);
  table.append(thead);

  const tbody = el('tbody');
  for (const e of [...rows].reverse()) {
    const tr = el('tr', 'clickable');
    tr.title = [e.reason, ...(e.warnings || [])].filter(Boolean).join('\n');

    tr.append(el('td', null, clock(e.modelTime)));

    const name = el('td', 'txt',
      (e.title || e.parameter) + (e.engine ? ` · ${e.engine}` : ''));
    if (e.unsafe) name.append(el('span', 'flag unsafe', 'unsafe'));
    if (e.direct) name.append(el('span', 'flag direct', 'direct'));
    tr.append(name);

    tr.append(el('td', 'txt', e.mode === 'failure' ? `отказ: ${e.failure}` : e.mode));
    tr.append(el('td', null, num(e.previous, 2)));
    tr.append(el('td', null, num(e.target, 2)));
    tr.append(el('td', null, e.duration ? `${num(e.duration, 1)} с` : '—'));
    tr.append(el('td', 'txt', e.operator || '—'));
    tr.append(el('td', `txt st-${e.status}`, e.status));

    // Переход от записи журнала к соответствующему моменту на графике.
    tr.onclick = () => {
      S.markers.push({ t: e.modelTime, label: e.title });
      $('#chart-follow').checked = false;
      switchDataView('charts');
      drawCharts();
    };

    tbody.append(tr);
  }
  table.append(tbody);
  box.replaceChildren(table);
}

function exportLog() {
  const blob = new Blob([JSON.stringify(S.log, null, 2)], { type: 'application/json' });
  download(blob, `intervention-log-run${S.snapshot?.runNumber ?? 0}.json`);
}

function download(blob, name) {
  const a = el('a');
  a.href = URL.createObjectURL(blob);
  a.download = name;
  a.click();
  setTimeout(() => URL.revokeObjectURL(a.href), 1000);
}

/* ===========================================================================
   Сравнение с эталоном
   =========================================================================== */

async function refreshCompare() {
  const box = $('#compare-table');
  try {
    const cmp = await (await fetch('/api/control/compare')).json();
    if (!cmp.available) {
      box.replaceChildren(el('div', 'muted',
        'Эталон не задан. Нажмите «Set baseline» на вкладке графиков, ' +
        'чтобы сохранить текущий прогон как базовый.'));
      $('#baseline-label').textContent = 'Эталон не задан';
      return;
    }
    $('#baseline-label').textContent =
      `Эталон: ${cmp.label} · seed ${cmp.seed} · отрезок ${num(cmp.from, 0)}…${num(cmp.to, 0)} с`;

    const table = el('table', 'data');
    const thead = el('thead');
    const hr = el('tr');
    ['Величина', 'Эталон', 'Текущий', 'Разница', 'Макс. отклонение', 'Начало', 'Предел']
      .forEach(h => hr.append(el('th', null, h)));
    thead.append(hr);
    table.append(thead);

    const tbody = el('tbody');
    for (const r of cmp.rows) {
      const tr = el('tr');
      tr.append(el('td', 'txt', `${r.title}, ${r.unit || '—'}`));
      tr.append(el('td', null, num(r.baseline, 3)));
      tr.append(el('td', null, num(r.current, 3)));
      const d = el('td', r.delta >= 0 ? 'up' : 'down',
        `${r.delta >= 0 ? '+' : ''}${num(r.delta, 3)}`);
      tr.append(d);
      tr.append(el('td', null, `${num(r.maxDeviation, 3)} @ ${num(r.maxDeviationAt, 0)} с`));
      tr.append(el('td', null, r.divergedAt >= 0 ? `${num(r.divergedAt, 1)} с` : '—'));
      tr.append(el('td', 'txt', r.outOfRange ? 'вышла за пределы' : '—'));
      tbody.append(tr);
    }
    table.append(tbody);
    box.replaceChildren(table);
  } catch (e) {
    box.replaceChildren(el('div', 'muted', 'Не удалось получить сравнение'));
  }
}

/* ===========================================================================
   Сценарии
   =========================================================================== */

const SCENARIO_STORE = 'rocket-sim-scenarios';

function loadScenarios() {
  try {
    S.scenarios = JSON.parse(localStorage.getItem(SCENARIO_STORE) || '[]');
  } catch { S.scenarios = []; }

  if (!S.scenarios.length) S.scenarios = [defaultScenario()];
  renderScenarioSelect();
  renderScenarioSteps();
}

function saveScenarios() {
  try { localStorage.setItem(SCENARIO_STORE, JSON.stringify(S.scenarios)); } catch {}
}

function defaultScenario() {
  return {
    name: 'Разгон насоса и прикрытие клапана',
    steps: [
      { at: 20, parameter: 'tp.shaft.speed', mode: 'relative', basis: 'percent_current', value: 15, duration: 0 },
      { at: 22, parameter: 'valve.fuel.command', mode: 'step', basis: 'absolute', value: 0.7, duration: 0 },
      { at: 27, parameter: 'tank.fuel.pressurant_valve', mode: 'ramp', basis: 'absolute', value: 0.3, duration: 5 },
      { at: 40, parameter: 'valve.fuel.command', mode: 'release', basis: 'absolute', value: 0, duration: 0 },
    ],
  };
}

function currentScenario() { return S.scenarios[S.scenarioIndex] || null; }

function renderScenarioSelect() {
  const sel = $('#scenario-select');
  sel.replaceChildren();
  S.scenarios.forEach((s, i) => sel.append(new Option(s.name, String(i))));
  sel.value = String(S.scenarioIndex);
  $('#scenario-name').value = currentScenario()?.name || '';
}

function renderScenarioSteps() {
  const box = $('#scenario-steps');
  box.replaceChildren();
  const sc = currentScenario();
  if (!sc) return;

  const head = el('div', 'sc-head');
  ['Время, с', 'Параметр', 'Режим', 'Отсчёт', 'Значение', 'Длит.', ''].forEach(h =>
    head.append(el('div', null, h)));
  box.append(head);

  sc.steps.forEach((st, i) => {
    const row = el('div', 'sc-step');
    row.dataset.index = String(i);

    row.append(input('number', st.at, v => { st.at = +v; saveScenarios(); }));

    const param = el('select');
    for (const p of S.params.values()) param.append(new Option(p.title, p.id));
    param.value = st.parameter;
    param.onchange = () => { st.parameter = param.value; saveScenarios(); };
    row.append(param);

    const mode = el('select');
    [['set', 'Установить'], ['step', 'Скачком'], ['ramp', 'Плавно'],
     ['relative', 'Относительно'], ['timed', 'Временно'], ['failure', 'Отказ'],
     ['release', 'Снять']].forEach(([v, t]) => mode.append(new Option(t, v)));
    mode.value = st.mode;
    mode.onchange = () => { st.mode = mode.value; saveScenarios(); };
    row.append(mode);

    const basis = el('select');
    [['absolute', 'единицы'], ['percent_current', '% тек.'],
     ['percent_nominal', '% ном.'], ['delta', 'приращ.']]
      .forEach(([v, t]) => basis.append(new Option(t, v)));
    basis.value = st.basis || 'absolute';
    basis.onchange = () => { st.basis = basis.value; saveScenarios(); };
    row.append(basis);

    row.append(input('number', st.value, v => { st.value = +v; saveScenarios(); }));
    row.append(input('number', st.duration || 0, v => { st.duration = +v; saveScenarios(); }));

    const del = el('button', 'btn btn-sm', '×');
    del.onclick = () => { sc.steps.splice(i, 1); saveScenarios(); renderScenarioSteps(); };
    row.append(del);

    box.append(row);
  });

  function input(type, value, onChange) {
    const n = el('input');
    n.type = type; n.value = value;
    n.onchange = () => onChange(n.value);
    return n;
  }
}

function startScenario() {
  const sc = currentScenario();
  if (!sc) return;
  S.scenarioRun = { name: sc.name, fired: new Set(), startedAt: S.snapshot?.modelTime ?? 0 };
  action('scenario');
  fetch(`/api/sim/scenario?name=${encodeURIComponent(sc.name)}`, { method: 'POST' })
    .catch(() => {});
  toast('ok', 'Сценарий запущен', sc.name);
  renderScenarioSteps();
}

function stopScenario() {
  S.scenarioRun = null;
  $('#scenario-progress').textContent = 'остановлен';
  renderScenarioSteps();
}

/* Шаги сценария отсчитываются от модельного времени его запуска: при смене
   скорости симуляции расписание не должно сдвигаться. */
function runScenarioTick(modelTime) {
  const run = S.scenarioRun;
  if (!run) return;
  const sc = currentScenario();
  if (!sc) return;

  const elapsed = modelTime - run.startedAt;
  let fired = 0;

  sc.steps.forEach((st, i) => {
    if (run.fired.has(i)) { fired += 1; return; }
    if (elapsed < st.at) return;
    run.fired.add(i);
    fired += 1;
    submit({
      parameter: st.parameter,
      mode: st.mode,
      basis: st.basis || 'absolute',
      curve: st.curve || 'linear',
      value: st.value,
      duration: st.duration || 0,
      source: 'scenario',
    });
  });

  $('#scenario-progress').textContent =
    `выполнено ${fired} из ${sc.steps.length}, T+${num(elapsed, 1)} с`;

  if (fired === sc.steps.length) S.scenarioRun = null;
}

/* ===========================================================================
   Развёртка носителя

   Обводы приходят с сервера: длины отсеков, диаметр, расстановка двигателей
   по кольцам и радиусы сопел выводятся из той же конфигурации, по которой
   собирается физическая модель. Здесь не вычисляется ни одна физическая
   величина — только перевод метров в пиксели.

   Уровни в баках берутся из телеметрии работающей ступени. У ступени, которая
   ещё не включалась, компоненты не расходовались, поэтому её баки полны — это
   не догадка, а следствие того, что модель их не трогает. У отделившейся
   ступени уровней нет вовсе: она больше не считается как часть носителя,
   и рисовать ей какое-либо заполнение значило бы выдумывать.
   =========================================================================== */

const CUT_H = 1180;   // высота поля чертежа, px
const CUT_W = 560;
const CUT_CX = 150;   // ось ракеты

async function loadLayout() {
  try {
    S.layout = await (await fetch('/api/vehicle/layout')).json();
  } catch {
    S.layout = null;
  }
  await loadFuelLoad();
  if (S.dataView === 'cutaway') drawCutaway();
}

/* Заправка на следующий пуск. Считает её сервер: интерфейс только показывает
   доли и тонны, которые вернула модель. */
async function loadFuelLoad(post) {
  try {
    const opts = post
      ? { method: 'POST', headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify(post) }
      : undefined;
    S.fuelLoad = await (await fetch('/api/vehicle/load', opts)).json();
  } catch {
    S.fuelLoad = null;
  }
  return S.fuelLoad;
}

/* Заданная заправка ступени, доля от штатной. */
function plannedLoad(stageIndex) {
  const st = (S.fuelLoad?.stages || []).find(s => s.stage === stageIndex);
  return st ? st.fraction : 1;
}

/* Состояние ступени относительно текущего момента полёта. */
function stagePhase(stageIndex) {
  const active = S.snapshot?.stage;
  if (!active) return 'preflight';
  if (stageIndex === active) return 'active';
  return stageIndex > active ? 'waiting' : 'detached';
}

/* Заполнение бака, 0…1, либо null, если величина не определена. */
function tankFill(stageIndex, side) {
  const phase = stagePhase(stageIndex);
  if (phase === 'detached') return null;
  if (phase === 'active') {
    const t = S.snapshot?.telemetry?.propulsion?.[side === 'ox' ? 'oxTank' : 'fuelTank'];
    return t ? t.fillFraction : null;
  }
  // Ступень ещё не включалась: в баках ровно столько, сколько залито.
  // До старта это заданная оператором заправка, после старта — полный бак
  // верхней ступени, которую пока не трогали.
  return phase === 'preflight' ? plannedLoad(stageIndex) : 1;
}

function tankInfo(stageIndex, side) {
  if (stagePhase(stageIndex) !== 'active') return null;
  return S.snapshot?.telemetry?.propulsion?.[side === 'ox' ? 'oxTank' : 'fuelTank'] || null;
}

function svg(tag, attrs, text) {
  const n = document.createElementNS('http://www.w3.org/2000/svg', tag);
  for (const [k, v] of Object.entries(attrs || {})) n.setAttribute(k, v);
  if (text !== undefined) n.textContent = text;
  return n;
}

/* Состояния агрегата приходят из модели короткими метками. Панель русская,
   и держать в ней английские слова незачем — тем более что «Throttled»
   и «Degraded» путаются друг с другом именно в переводе: первое означает
   штатное дросселирование, второе — потерю КПД. */
const STATE_NAMES = {
  'Normal': 'номинал',
  'Throttled': 'дросселирован',
  'Degraded': 'падение КПД',
  'Cavitation': 'кавитация',
  'Failed': 'отказ',
  'Overspeed': 'превышение оборотов',
  'Stuck': 'заклинил',
  'Stopped': 'остановлен',
  'Bearing overheating': 'перегрев подшипников',
  'Shaft displacement': 'смещение ротора',
};

function stateName(state) {
  return STATE_NAMES[state] || state || '—';
}

function engineClass(e) {
  if (!e) return 'nodata';
  const state = (e.state || '').split(' ')[0];
  if (!e.running) return 'off';
  if (['Stopped', 'Stuck', 'Failed', 'Overspeed', 'Bearing'].includes(state)) return 'crit';
  if (['Degraded', 'Cavitation', 'Shaft'].includes(state)) return 'warn';
  // Дросселирование — режим, а не отклонение: агрегат исправен.
  return 'ok';
}

function cutawayDetail(text) {
  $('#cutaway-detail').textContent = text;
}

/* Высота чертежа в пикселях: ракета целиком должна помещаться в панель.

   На тринадцатидюймовом экране правая панель — это примерно шестьсот точек
   по высоте, и жёстко заданный чертёж в тысячу с лишним точек в неё не влезал:
   разрез приходилось прокручивать, а целиком он не был виден никогда.
   Поэтому высота считается по фактическому размеру панели, а увеличение
   оператор задаёт сам. */
function cutawayHeight() {
  const box = $('#cutaway');
  const avail = Math.max(320, (box?.clientHeight || 640) - 16);
  return avail * (S.cutZoom || 1);
}

function drawCutaway() {
  const box = $('#cutaway');
  if (!box) return;

  const L = S.layout;
  if (!L || !L.stages?.length) {
    box.replaceChildren(el('div', 'muted', 'Обводы носителя не получены'));
    return;
  }

  $('#cutaway-title').textContent =
    `${L.title} · ${num(L.totalLength, 1)} м · диаметр ${num(L.diameter, 2)} м`;

  const showLabels  = $('#cutaway-labels').checked;
  const showEngines = $('#cutaway-engines').checked;

  const scale = (CUT_H - 70) / L.totalLength;
  const root = svg('svg', {
    viewBox: `0 0 ${CUT_W} ${CUT_H}`,
    class: 'cutaway-svg',
    preserveAspectRatio: 'xMidYMid meet',
  });

  // Размер задаётся явно, чтобы перевод координат мыши в координаты чертежа
  // был однозначным: тянуть ползунок заправки иначе нельзя.
  const px = cutawayHeight();
  root.style.height = `${px}px`;
  root.style.width = `${(CUT_W / CUT_H) * px}px`;

  let y = 30;
  for (const stage of L.stages) {
    const phase = stagePhase(stage.index);
    if (phase === 'detached') y += 22; // отделившаяся ступень отходит вниз

    const g = svg('g', { class: `cut-stage phase-${phase}` });
    drawStage(g, stage, y, scale, showLabels);
    root.append(g);

    y += stage.length * scale;
  }

  const wrap = el('div', 'cutaway-wrap');
  const left = el('div', 'cutaway-plate');
  left.append(root);
  wrap.append(left);

  if (showEngines) wrap.append(engineEndView());

  box.replaceChildren(wrap);
}

/* Один отсек за другим сверху вниз. */
function drawStage(g, stage, top, scale, showLabels) {
  const phase = stagePhase(stage.index);
  let y = top;

  // Границы группы баков нужны ползунку заправки: он тянется вдоль них.
  let tankTop = null, tankBottom = null;

  for (const s of stage.sections) {
    const h = s.length * scale;
    const w = s.diameter * scale;
    const x = CUT_CX - w / 2;

    if (s.kind === 'nose' || s.kind === 'payload') {
      // Носовая часть рисуется обводом, а не прямоугольником.
      g.append(svg('path', {
        class: 'cut-nose',
        d: `M ${x} ${y + h} L ${x} ${y + h * 0.45} Q ${CUT_CX} ${y - h * 0.1} ` +
           `${x + w} ${y + h * 0.45} L ${x + w} ${y + h} Z`,
      }));
    } else {
      g.append(svg('rect', {
        class: `cut-sec kind-${s.kind}`,
        x, y, width: w, height: h, rx: 2,
      }));
    }

    if (s.tank) {
      if (tankTop === null) tankTop = y;
      tankBottom = y + h;

      const f = tankFill(stage.index, s.tank);
      if (f !== null && f !== undefined) {
        const lh = Math.max(0, Math.min(1, f)) * (h - 4);
        g.append(svg('rect', {
          class: `cut-liquid ${s.tank}`,
          x: x + 2, y: y + h - 2 - lh, width: w - 4, height: lh,
          'data-stage': stage.index, 'data-y': y, 'data-h': h,
        }));
      }
    }

    if (s.kind === 'engines') drawEngineBell(g, stage, x, y, w, h);

    // Наведение показывает состояние отсека в подробностях.
    const hit = svg('rect', {
      class: 'cut-hit', x, y, width: w, height: h,
      'data-section': `${stage.index}:${s.title}`,
    });
    hit.addEventListener('mouseenter', () => cutawayDetail(sectionDetail(stage, s)));
    g.append(hit);

    if (showLabels) drawLabel(g, stage, s, y, h, x + w);

    y += h;
  }

  // Подпись ступени слева.
  g.append(svg('text', {
    class: 'cut-stage-name', x: 8, y: top + 14,
  }, `${stage.name}${phase === 'detached' ? ' · отделилась' : ''}`));

  if (tankTop !== null) drawFuelSlider(g, stage, tankTop, tankBottom, scale);
}

/* Ползунок заправки вдоль баков ступени.

   Заправка — свойство изделия на старте, а не состояние полёта: залить
   компоненты в летящую ракету нельзя. Поэтому ползунок правит паспорт
   следующего пуска, и модель получает ракету уже с этой массой — со всеми
   последствиями для тяговооружённости, центровки и дальности. */
function drawFuelSlider(g, stage, top, bottom, scale) {
  const info = (S.fuelLoad?.stages || []).find(s => s.stage === stage.index);
  if (!info) return;

  const w = S.layout.diameter * scale;
  const trackX = CUT_CX - w / 2 - 18;
  const yFor = f => bottom - (Math.max(0, Math.min(1.2, f)) / 1.2) * (bottom - top);

  // Во время прогона ползунок правит уже следующий пуск, и подпись обязана
  // об этом говорить: иначе оператор ждал бы, что уровень в баке поедет.
  const flying = stagePhase(stage.index) !== 'preflight';

  const box = svg('g', { class: 'cut-slider' });
  box.append(svg('line', {
    class: 'cut-track', x1: trackX, y1: top, x2: trackX, y2: bottom,
  }));
  // Отметка штатной заправки.
  box.append(svg('line', {
    class: 'cut-track-nominal',
    x1: trackX - 5, y1: yFor(1), x2: trackX + 5, y2: yFor(1),
  }));

  const fill = svg('line', { class: 'cut-track-fill', x1: trackX, y1: bottom, x2: trackX });
  box.append(fill);

  const handle = svg('path', { class: 'cut-handle-shape' });
  box.append(handle);

  const value = svg('text', { class: 'cut-slider-value', x: trackX - 14, 'text-anchor': 'end' });
  const sub   = svg('text', { class: 'cut-slider-sub',   x: trackX - 14, 'text-anchor': 'end' });
  box.append(value, sub);

  // Тянуть можно и за ручку, и за саму дорожку.
  const grab = svg('rect', {
    class: 'cut-slider-hit',
    x: trackX - 14, y: top - 8, width: 28, height: bottom - top + 16,
  });
  box.append(grab);

  // Перерисовывается только то, что двигается. Полная перестройка чертежа
  // на каждое движение мыши и стоила дорого, и ломала перетаскивание:
  // элемент, за который тянут, выбрасывался из документа, а у выброшенного
  // элемента прямоугольник нулевой — доля обращалась в ноль, и ползунок
  // падал на дно шкалы.
  const apply = f => {
    const hy = yFor(f);
    fill.setAttribute('y2', hy);
    handle.setAttribute('d',
      `M ${trackX - 9} ${hy - 6} L ${trackX + 9} ${hy} L ${trackX - 9} ${hy + 6} Z`);
    value.setAttribute('y', hy - 6);
    value.textContent = `${num(f * (info.nominal || 0) / 1000, 0)} т`;
    sub.setAttribute('y', hy + 8);
    sub.textContent = flying
      ? `${num(f * 100, 0)} % на следующий пуск`
      : `${num(f * 100, 0)} % заправки`;

    // До пуска уровень в баках идёт за ползунком: это и есть заправка.
    // В полёте уровень принадлежит телеметрии, и трогать его нельзя.
    if (flying) return;
    for (const n of $$(`#cutaway .cut-liquid[data-stage="${stage.index}"]`)) {
      const y0 = Number(n.dataset.y), h = Number(n.dataset.h);
      const lh = Math.max(0, Math.min(1, f)) * (h - 4);
      n.setAttribute('height', lh);
      n.setAttribute('y', y0 + h - 2 - lh);
    }
  };

  apply(info.fraction);

  // Прямоугольник берётся у живого чертежа, а не у элемента из замыкания:
  // после перерисовки тот может оказаться уже вне документа.
  const fractionAt = clientY => {
    const live = $('#cutaway .cutaway-svg');
    const r = live ? live.getBoundingClientRect() : null;
    if (!r || !r.height) return info.fraction;

    const vy = (clientY - r.top) / r.height * CUT_H;
    const f = (bottom - vy) / (bottom - top) * 1.2;
    return Math.max(0, Math.min(1.2, Math.round(f * 100) / 100));
  };

  const drag = ev => {
    info.fraction = fractionAt(ev.clientY);
    info.mass = info.fraction * (info.nominal || 0);
    apply(info.fraction);
  };

  const onUp = ev => {
    window.removeEventListener('mousemove', drag);
    window.removeEventListener('mouseup', onUp);
    drag(ev);
    commitFuelLoad(stage.index, info.fraction);
  };

  grab.addEventListener('mousedown', ev => {
    ev.preventDefault();
    drag(ev);
    window.addEventListener('mousemove', drag);
    window.addEventListener('mouseup', onUp);
  });

  grab.addEventListener('mouseenter', () => cutawayDetail(
    `${stage.name}: заправка ${num(info.mass / 1000, 1)} т из штатных ` +
    `${num((info.nominal || 0) / 1000, 1)} т (${num(info.fraction * 100, 0)} %). ` +
    `Применится при следующем запуске: залить компоненты в летящую ракету нельзя.`));

  g.append(box);
}

/* Отправка заправки на сервер. Пересчитывает её модель, а не интерфейс. */
function commitFuelLoad(stage, fraction) {
  loadFuelLoad({ stage, fraction }).then(r => {
    drawCutaway();
    if (!r) return toast('err', 'Заправка не принята');
    toast('ok', `Заправка ступени ${stage}: ${num(fraction * 100, 0)} %`,
      `Стартовая масса ${num(r.liftoffMass / 1000, 0)} т, ` +
      `тяговооружённость ${num(r.twr, 2)}. ${r.note}`);
  });
}

/* Сопла в разрезе.

   На виде сбоку двигатели закрывают друг друга: видны те, что ближе к краю.
   Поэтому сопла проецируются на плоскость разреза и рисуются от края к оси,
   пропуская те, что попали бы на уже занятое место, — иначе тридцать три
   сопла Super Heavy наползали друг на друга сплошным пятном. */
function drawEngineBell(g, stage, x, y, w, h) {
  const engines = stage.engines || [];
  if (!engines.length) return;

  const half = S.layout.diameter / 2;
  const bottom = y + h;
  const scale = w / (half * 2);

  // Сопла отбираются от края к оси: наружные видны целиком, внутренние
  // выглядывают между ними. Небольшое перекрытие оставлено намеренно — так
  // читается глубина, а без него от тридцати трёх двигателей на разрезе
  // оставалось бы пять.
  const placed = [];
  const byEdge = [...engines].sort((a, b) => Math.abs(b.x) - Math.abs(a.x));

  for (const e of byEdge) {
    const rx = Math.max(2, e.exitRadius * scale);
    const cx = CUT_CX + e.x * scale;
    if (placed.some(p => Math.abs(p.cx - cx) < (p.rx + rx) * 0.7)) continue;
    placed.push({ cx, rx, kind: e.kind, ring: e.ring });
  }

  // Внутренние кольца рисуются первыми и чуть короче: они дальше от зрителя.
  const outer = Math.max(...placed.map(p => p.ring));
  placed.sort((a, b) => a.ring - b.ring);

  S.bellsDrawn = S.bellsDrawn || {};
  S.bellsDrawn[stage.index] = placed.length;

  for (const p of placed) {
    const depth = p.ring === outer ? 1 : 0.76;
    const bell = Math.min(h * 0.82, p.rx * 2.6) * depth;
    g.append(svg('path', {
      class: `cut-bell ${p.kind}${p.ring === outer ? '' : ' inner'}`,
      d: `M ${p.cx - p.rx * 0.34} ${bottom - bell} L ${p.cx - p.rx} ${bottom} ` +
         `L ${p.cx + p.rx} ${bottom} L ${p.cx + p.rx * 0.34} ${bottom - bell} Z`,
    }));
  }
}

function drawLabel(g, stage, s, y, h, right) {
  const my = y + h / 2;
  const lx = right + 18;
  g.append(svg('line', { class: 'cut-lead', x1: right, y1: my, x2: lx, y2: my }));
  g.append(svg('line', { class: 'cut-lead', x1: lx, y1: my, x2: lx + 14, y2: my }));

  g.append(svg('text', { class: 'cut-label', x: lx + 20, y: my - 2 }, s.title));
  g.append(svg('text', { class: 'cut-sub', x: lx + 20, y: my + 13 },
    labelValue(stage, s)));
}

function labelValue(stage, s) {
  const parts = [`${num(s.length, 1)} м`];

  if (s.tank) {
    const f = tankFill(stage.index, s.tank);
    const t = tankInfo(stage.index, s.tank);
    if (f === null || f === undefined) parts.push('нет данных');
    else parts.push(`${num(f * 100, 0)} %`);
    if (t) {
      parts.push(`${num(t.mass / 1000, 1)} т`);
      parts.push(`${num(t.pressure / 1e5, 2)} бар`);
    }
  } else if (s.kind === 'engines') {
    const n = (stage.engines || []).length;
    parts.push(`${n} дв.`);

    // На виде сбоку часть сопел закрыта соседними, и число нарисованных
    // меньше числа двигателей. Без этой подписи разрез выглядит ошибкой.
    const drawn = S.bellsDrawn?.[stage.index];
    if (drawn && drawn < n) parts.push(`на разрезе ${drawn}`);

    if (stagePhase(stage.index) === 'active') {
      const running = (S.snapshot?.engines || []).filter(e => e.running).length;
      parts.push(`работают ${running}`);
    }
  }
  return parts.join(' · ');
}

function sectionDetail(stage, s) {
  const head = `${stage.name} → ${s.title}: длина ${num(s.length, 1)} м, ` +
               `диаметр ${num(s.diameter, 2)} м`;

  if (s.kind === 'engines') {
    const n = (stage.engines || []).length;
    const drawn = S.bellsDrawn?.[stage.index] || n;
    const vac = (stage.engines || []).filter(e => e.kind === 'vacuum').length;
    const kinds = vac
      ? `${n - vac} с атмосферным соплом и ${vac} с вакуумным`
      : `все с атмосферным соплом`;
    return `${head}. Двигателей ${n}: ${kinds}. На разрезе видно ${drawn} — ` +
           `остальные закрыты этими же соплами, ракета не прозрачная. ` +
           `Все ${n} показаны на виде с торца.`;
  }

  if (!s.tank) return head;

  const phase = stagePhase(stage.index);
  if (phase === 'detached') {
    return `${head}. Ступень отделилась — модель её баки больше не считает.`;
  }
  if (phase === 'waiting') {
    return `${head}. Компонент (${s.propellant || '—'}) не расходовался: ` +
           `ступень ещё не включалась.`;
  }
  if (phase === 'preflight') {
    const info = (S.fuelLoad?.stages || []).find(x => x.stage === stage.index);
    return `${head}. Заправка перед пуском: ` +
           `${info ? num(info.mass / 1000, 1) : '—'} т на ступень.`;
  }

  const t = tankInfo(stage.index, s.tank);
  if (!t) return head;

  const drain = t.drainRate > 0 ? `, расход ${num(t.drainRate, 1)} кг/с` : '';
  return `${head}. ${s.propellant || t.name}: ${num(t.mass / 1000, 1)} т ` +
         `(${num(t.fillFraction * 100, 0)} %), уровень ${num(t.level, 2)} м, ` +
         `давление ${num(t.pressure / 1e5, 2)} бар, наддув ` +
         `${num(t.pressurantMass, 0)} кг${drain}`;
}

/* Вид с торца: расстановка двигателей и отклонение камер.

   Показывается любая ступень, а не только работающая: шесть двигателей
   корабля надо видеть и до разделения, когда считается ещё первая ступень. */
function engineEndView() {
  const stages = S.layout.stages || [];
  const active = stages.find(s => stagePhase(s.index) === 'active');
  const chosen = stages.find(s => s.index === S.endStage)
              || active || stages[stages.length - 1];

  const wrap = el('div', 'cut-end');

  const head = el('div', 'cut-end-head');
  head.append(el('span', '', 'Вид с торца'));
  for (const st of stages) {
    const b = el('button',
      'btn btn-sm' + (st.index === chosen.index ? ' btn-go' : ''),
      `${st.index} ступень`);
    b.onclick = () => { S.endStage = st.index; drawCutaway(); };
    head.append(b);
  }
  wrap.append(head);

  const engines = chosen.engines || [];
  if (!engines.length) {
    wrap.append(el('div', 'muted', 'Нет двигателей'));
    return wrap;
  }

  wrap.append(el('div', 'cut-end-sub', `${chosen.name}: ${engines.length} дв.`));

  const R = S.layout.diameter / 2;
  const size = 260;
  const k = (size / 2 - 6) / R;
  const c = size / 2;

  const root = svg('svg', { viewBox: `0 0 ${size} ${size}`, class: 'cut-end-svg' });
  root.append(svg('circle', { class: 'cut-skirt', cx: c, cy: c, r: R * k }));

  const live = stagePhase(chosen.index) === 'active';
  const byID = new Map((S.snapshot?.engines || []).map(e => [e.id, e]));

  for (const e of engines) {
    const data = byID.get(e.id);
    const cls = live ? engineClass(data) : 'nodata';
    const cx = c + e.x * k;
    const cy = c - e.y * k;
    const r = Math.max(3, e.exitRadius * k);

    if (e.gimbal) {
      root.append(svg('circle', { class: 'cut-eng-gimbal', cx, cy, r: r + 3 }));
    }

    const node = svg('circle', {
      class: `cut-eng ${cls}${data?.manual ? ' manual' : ''}` +
             `${S.target === e.id ? ' selected' : ''}`,
      cx, cy, r, 'data-id': e.id,
    });
    node.addEventListener('mouseenter', () => cutawayDetail(engineDetail(e, data)));
    node.addEventListener('click', () => { setTarget(e.id); drawCutaway(); });
    root.append(node);
  }

  // Отклонение камер: направление и величина берутся из телеметрии.
  const t = live ? S.snapshot?.telemetry : null;
  if (t && (t.gimbalPitch || t.gimbalYaw)) {
    const lim = Math.max(1, t.gimbalLimit || 5);
    const len = size / 2 - 10;
    root.append(svg('line', {
      class: 'cut-gimbal',
      x1: c, y1: c,
      x2: c + (t.gimbalYaw / lim) * len,
      y2: c + (t.gimbalPitch / lim) * len,
    }));
  }

  wrap.append(root);

  if (t) {
    wrap.append(el('div', 'cut-end-sub',
      `Качание: тангаж ${num(t.gimbalPitch, 2)}° · рыскание ${num(t.gimbalYaw, 2)}°` +
      ` при пределе ${num(t.gimbalLimit, 1)}°`));
    wrap.append(el('div', 'cut-end-sub',
      `Потребный момент к располагаемому: ${num(t.controlAuthority, 2)}` +
      `${t.controlSaturated ? ' — упор в предел' : ''}`));
  } else if (!live) {
    wrap.append(el('div', 'cut-end-sub',
      'Ступень не работает: показана только компоновка.'));
  }

  wrap.append(el('div', 'cut-legend',
    'Кольцо вокруг сопла — двигатель в подвесе. Линия из центра — фактическое ' +
    'отклонение камер. Щелчок по соплу делает двигатель адресатом команд.'));
  return wrap;
}

function engineDetail(e, live) {
  const base = `${e.id}: ${e.kind === 'vacuum' ? 'вакуумное сопло' : 'атмосферное сопло'}, ` +
               `срез ${num(e.exitRadius * 2, 2)} м, ` +
               `${e.gimbal ? 'в подвесе' : 'закреплён жёстко'}`;
  if (!live) return `${base}. Данных по двигателю нет: ступень не работает.`;

  return `${base}. ${stateName(live.state)}: тяга ${num(live.thrust, 0)} кН, ` +
         `давление в камере ${num(live.chamberPressure, 1)} МПа, ` +
         `обороты ${num(live.shaftRpm, 0)}, ` +
         `запас стенки ${num(live.wallMargin, 0)} К`;
}

/* ===========================================================================
   Вход в атмосферу

   Корабль возвращается брюхом вперёд, под углом атаки около семидесяти
   градусов, и держат его четыре плавника. Здесь они и рисуются: вид сбоку
   с фактическими углами отклонения, а рядом — ползунок на каждый привод.

   Взятый в руки плавник автопилоту больше не подчиняется. Момент от него
   считает модель: сила по ньютоновскому обтеканию, плечо от центра масс.
   Интерфейс только показывает положение и отправляет команду.
   =========================================================================== */

function entryCommand(body) {
  return fetch('/api/entry/flap', {
    method: 'POST', headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(body),
  }).then(r => r.ok ? r.json() : Promise.reject(r.statusText));
}

function attitudeCommand(body) {
  return fetch('/api/entry/attitude', {
    method: 'POST', headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(body),
  }).then(r => r.ok ? r.json() : Promise.reject(r.statusText));
}

/* Состояние вкладки.

   Разметка строится один раз на состав поверхностей, а телеметрия потом
   только меняет числа в готовых узлах. Полная перерисовка на каждом кадре
   отрывала ползунок из-под курсора: элемент, за который держится указатель,
   исчезал вместе со всей разметкой, и перетаскивание превращалось в серию
   отдельных щелчков. Ровно на это и жаловались. */
const EN = {
  key: null, plate: null, rows: new Map(),
  att: null, shield: null,
  active: null, // ползунок, который сейчас держит указатель
};

/* Отправка не чаще, чем раз в восемьдесят миллисекунд: за ползунком
   тянется поток событий, а команда идёт на сервер и меняет модель. */
function throttled(fn, ms) {
  let last = 0, timer = null, pending = null;
  return (...args) => {
    pending = args;
    const now = Date.now();
    if (now - last >= ms) {
      last = now; fn(...pending); pending = null;
    } else if (!timer) {
      timer = setTimeout(() => {
        timer = null; last = Date.now();
        if (pending) { fn(...pending); pending = null; }
      }, ms - (now - last));
    }
  };
}

const sendFlap = throttled((name, angle) => {
  entryCommand({ name, angle })
    .catch(e => toast('err', 'Плавник не принял команду', String(e)));
}, 80);

const sendAttitude = throttled((pitch, yaw, roll) => {
  attitudeCommand({ pitch, yaw, roll })
    .catch(e => toast('err', 'Ориентация не принята', String(e)));
}, 80);

/* Ползунок помечается захваченным, пока за него держится указатель:
   обновление телеметрии не должно перебивать то, что делает рука. */
function grabbable(slider) {
  const grab = () => { EN.active = slider; };
  const drop = () => { if (EN.active === slider) EN.active = null; };
  slider.addEventListener('pointerdown', grab);
  slider.addEventListener('keydown', grab);
  slider.addEventListener('pointerup', drop);
  slider.addEventListener('pointercancel', drop);
  slider.addEventListener('blur', drop);
  slider.addEventListener('change', drop);
  return slider;
}

function rangeInput(min, max, step, value) {
  const s = document.createElement('input');
  s.type = 'range';
  s.min = min; s.max = max; s.step = step; s.value = value;
  return grabbable(s);
}

function drawEntry() {
  const box = $('#entry-panel');
  if (!box) return;

  const t = S.snapshot?.telemetry;
  const flaps = t?.flaps || [];

  $('#entry-state').textContent = entryStateText(t, flaps);

  const key = flaps.map(f => f.name).join(',');
  if (key !== EN.key) { buildEntryPanel(box, flaps); EN.key = key; }
  if (!flaps.length) return;

  EN.plate.replaceChildren(entryDrawing(t, flaps));
  updateShield(t);
  updateAttitude(t);
  updateFlapRows(flaps);
}

function entryStateText(t, flaps) {
  if (!t) return 'Нет данных';
  if (!flaps.length) return 'Плавников нет';

  const manual = flaps.filter(f => f.manual).length;

  // Кто именно держит корпус. Пока напора нет, это двигатели ориентации;
  // ниже корпус берут на себя плавники, и двигатели замолкают.
  const holder = t.usingRcs ? 'двигатели ориентации' : 'плавники';

  return `${t.flapsDeployed ? 'Плавники выпущены' : 'Плавники прижаты'} · ` +
    `угол атаки ${num(t.totalAoA, 1)}° · напор ${num(t.dynamicPressure / 1000, 1)} кПа · ` +
    `поток ${num(t.heatFlux / 1000, 0)} кВт/м² · ` +
    `власть ${num(t.flapAuthority / 1e6, 2)} МН·м · держит: ${holder}` +
    (manual ? ` · вручную ${manual} из ${flaps.length}` : '');
}

/* Разметка строится один раз на состав поверхностей. */
function buildEntryPanel(box, flaps) {
  EN.rows.clear();
  EN.plate = null; EN.att = null; EN.shield = null; EN.active = null;

  if (!flaps.length) {
    box.replaceChildren(el('div', 'muted',
      'У этого носителя нет аэродинамических поверхностей, либо корабль ещё ' +
      'не отделился. Плавники появляются вместе со второй ступенью.'));
    return;
  }

  const wrap = el('div', 'entry-wrap');

  EN.plate = el('div', 'entry-plate');
  wrap.append(EN.plate);

  const controls = el('div', 'entry-controls');
  controls.append(buildShieldBlock());
  controls.append(buildAttitudeBlock());
  for (const f of flaps) controls.append(buildFlapRow(f));

  controls.append(el('div', 'entry-note',
    'Ноль — плавник прижат к борту и почти ничего не даёт, упор — раскрыт ' +
    'поперёк потока и даёт наибольшую силу. Чтобы поднять нос, передние ' +
    'раскрывают, а задние прижимают. Момент считает модель — по площади, ' +
    'плечу и скоростному напору, — поэтому в разрежённых слоях перекладка ' +
    'плавников почти ничего не даёт: на ста километрах напора всего ' +
    'семнадцать паскалей.'));

  wrap.append(controls);
  box.replaceChildren(wrap);
}

/* ---------------------------------------------------------------------------
   Теплозащита

   У корабля две стороны: брюхо в керамических плитках и голая сталь спины.
   Держит вход только первая, поэтому здесь показывается не одна температура,
   а обе — и подставленность брюха потоку, из которой они и получаются.
   --------------------------------------------------------------------------- */
function buildShieldBlock() {
  const box = el('div', 'entry-shield');
  box.append(el('div', 'entry-block-head', 'Теплозащита'));

  const mk = title => {
    const row = el('div', 'shield-row');
    row.append(el('span', 'shield-name', title));
    const bar = el('div', 'shield-bar');
    const fill = el('i', 'shield-fill');
    bar.append(fill);
    row.append(bar);
    const val = el('span', 'shield-val mono', '—');
    row.append(val);
    box.append(row);
    return { row, fill, val };
  };

  EN.shield = {
    box,
    tiles: mk('Плитки'),
    steel: mk('Сталь'),
    note: el('div', 'entry-note', ''),
  };
  box.append(EN.shield.note);
  return box;
}

function updateShield(t) {
  const s = EN.shield;
  if (!s) return;

  const h = t?.heatShield;
  s.box.style.display = h ? '' : 'none';
  if (!h) return;

  const side = (ui, data) => {
    const share = Math.max(0, Math.min(1, data.temperature / data.limit));
    ui.fill.style.width = (share * 100).toFixed(0) + '%';
    ui.row.classList.toggle('hot', data.margin < 150);
    ui.row.classList.toggle('over', data.margin < 0 || data.damage > 0);
    ui.val.textContent = `${num(data.temperature, 0)} К (запас ${num(data.margin, 0)})` +
      (data.damage > 0 ? ` · прогар ${num(data.damage * 100, 0)}%` : '');
  };

  side(s.tiles, h.tiles);
  side(s.steel, h.steel);

  const e = h.exposure;
  const facing = e > 0.7 ? 'плитками в поток'
    : e < -0.7 ? 'спиной в поток — сталь голая'
    : 'боком к потоку';
  s.note.textContent = `Подставленность брюха ${num(e, 2)}: ${facing}. ` +
    (h.burnedThrough ? 'Корпус прогорел.' :
      'Поток делится между сторонами по этому числу; затенённой достаётся ' +
      'восемь процентов — донные вихри.');
}

/* ---------------------------------------------------------------------------
   Ручная ориентация

   Оператор назначает углы, а не моменты: автопилот остаётся в работе, просто
   цель ему задаёт человек. Корпус придёт к ней ровно настолько быстро,
   насколько хватит плавников и двигателей ориентации.
   --------------------------------------------------------------------------- */
function buildAttitudeBlock() {
  const box = el('div', 'entry-attitude');
  box.append(el('div', 'entry-block-head', 'Ориентация корпуса'));

  const mode = el('div', 'muted', 'под автоматом');
  box.append(mode);

  const axes = {};
  const mk = (key, title, min, max) => {
    const row = el('div', 'entry-row-head');
    row.append(el('b', '', title));
    const val = el('span', 'muted mono', '—');
    row.append(val);
    box.append(row);

    const slider = rangeInput(min, max, 1, 0);
    slider.oninput = () => { axes[key].val.textContent = slider.value + '°'; push(); };
    box.append(slider);
    axes[key] = { slider, val };
  };

  const push = () => {
    EN.att.manual = true;
    sendAttitude(Number(axes.pitch.slider.value),
      Number(axes.yaw.slider.value), Number(axes.roll.slider.value));
  };

  mk('pitch', 'Тангаж', -90, 90);
  mk('yaw', 'Курс', 0, 360);
  mk('roll', 'Крен', -180, 180);

  // Тяга здесь же: разворот без сброса тяги уводит корабль с траектории,
  // а гнать вторую ступень на полном газу посреди входа незачем.
  const trow = el('div', 'entry-row-head');
  trow.append(el('b', '', 'Тяга'));
  const tval = el('span', 'muted mono', '100 %');
  trow.append(tval);
  box.append(trow);

  const throttle = rangeInput(0, 100, 5, 100);
  throttle.oninput = () => { tval.textContent = throttle.value + ' %'; };
  // Команда уходит по отпусканию: каждая попадает в журнал, и сыпать их
  // на каждое движение ручки — значит завалить журнал мусором.
  throttle.onchange = () => sendCommand({
    parameter: 'engine.throttle', mode: 'step', value: Number(throttle.value) / 100,
  });
  box.append(throttle);

  const foot = el('div', 'entry-row-foot');

  const belly = el('button', 'btn btn-sm', 'Плитками в поток');
  belly.onclick = () => { axes.roll.slider.value = 0; push(); };
  foot.append(belly);

  const back = el('button', 'btn btn-sm', 'Спиной в поток');
  back.onclick = () => { axes.roll.slider.value = 180; push(); };
  foot.append(back);

  const auto = el('button', 'btn btn-sm', 'Вернуть автомату');
  auto.onclick = () => {
    EN.att.manual = false;
    attitudeCommand({ release: true })
      .then(() => toast('ok', 'Ориентация под автоматом'))
      .catch(e => toast('err', 'Не отдалось автомату', String(e)));
  };
  foot.append(auto);
  box.append(foot);

  box.append(el('div', 'entry-note',
    'Снизьте тягу и разверните корпус: крен ноль — брюхо с плитками против ' +
    'потока, сто восемьдесят — на поток выходит голая сталь. Что из этого ' +
    'выйдет, считает модель нагрева, а не картинка.'));

  EN.att = { box, axes, mode, throttle, manual: false };
  return box;
}

function updateAttitude(t) {
  const a = EN.att;
  if (!a || !t) return;

  const manual = !!t.heatShield?.manualAttitude;
  a.manual = manual;
  a.mode.textContent = manual
    ? 'вручную — автомат ориентацией не управляет'
    : 'под автоматом (сдвиньте ползунок, чтобы взять управление)';
  a.box.classList.toggle('manual', manual);

  // Пока управление у автомата, ползунки показывают фактическую ориентацию:
  // взявшись за них, оператор продолжает с того положения, в котором корабль
  // сейчас, а не с нуля.
  const set = (key, value) => {
    const ui = a.axes[key];
    if (EN.active === ui.slider) return;
    if (!manual) ui.slider.value = Math.round(value);
    ui.val.textContent = num(manual ? Number(ui.slider.value) : value, 0) + '°' +
      (manual ? ` (сейчас ${num(value, 0)}°)` : '');
  };

  const th = a.throttle;
  if (EN.active !== th && Number.isFinite(t.throttle)) {
    th.value = Math.round(t.throttle * 100);
    th.previousElementSibling.querySelector('.mono').textContent = th.value + ' %';
  }

  set('pitch', t.pitch ?? 0);
  set('yaw', t.yaw ?? 0);
  set('roll', t.roll ?? 0);
}

/* ---------------------------------------------------------------------------
   Плавники
   --------------------------------------------------------------------------- */
function buildFlapRow(f) {
  const row = el('div', 'entry-row');

  const head = el('div', 'entry-row-head');
  head.append(el('b', '', flapTitle(f.name)));
  const read = el('span', 'muted mono', '');
  head.append(read);
  row.append(head);

  const slider = rangeInput(0, f.limit, 1, f.command);
  slider.oninput = () => {
    read.textContent = `${num(Number(slider.value), 0)}° из ${num(f.limit, 0)}°`;
    sendFlap(f.name, Number(slider.value));
  };
  row.append(slider);

  const foot = el('div', 'entry-row-foot');
  const mode = el('span', 'muted', 'автомат');
  foot.append(mode);

  const back = el('button', 'btn btn-sm', 'Вернуть автомату');
  back.onclick = () => entryCommand({ name: f.name, release: true })
    .catch(e => toast('err', 'Плавник не принял команду', String(e)));
  foot.append(back);
  row.append(foot);

  EN.rows.set(f.name, { row, read, slider, mode, back });
  return row;
}

function updateFlapRows(flaps) {
  for (const f of flaps) {
    const ui = EN.rows.get(f.name);
    if (!ui) continue;

    ui.row.classList.toggle('manual', f.manual);
    ui.row.classList.toggle('jammed', f.jammed);
    ui.mode.textContent = f.jammed ? 'заклинило' : f.manual ? 'ручное' : 'автомат';
    ui.back.style.display = f.manual ? '' : 'none';
    ui.read.textContent = `${num(f.deflection, 1)}° из ${num(f.limit, 0)}° · ` +
      `${num(f.area, 0)} м²`;

    // Значение ползунка перебивается телеметрией только тогда, когда за него
    // никто не держится.
    if (EN.active !== ui.slider) ui.slider.value = f.command;
  }
}

/* ---------------------------------------------------------------------------
   Вид сбоку

   Поток на чертеже всегда идёт снизу вверх — корабль падает. Корпус повёрнут
   к нему на фактический угол атаки: при семидесяти градусах он лежит почти
   поперёк потока, подставив брюхо, и это не украшение, а то же число, по
   которому модель считает сопротивление и нагрев.

   Плитки — на брюхе, плавники — на спине: у корабля они растут с подветренной
   стороны, а наветренная сплошь закрыта теплозащитой. Прежний чертёж вешал
   плавники по обе стороны корпуса, будто это крылья самолёта, — на настоящем
   корабле их там нет.
   --------------------------------------------------------------------------- */
function entryDrawing(t, flaps) {
  const W = 460, H = 320;
  const root = svg('svg', { viewBox: `0 0 ${W} ${H}`, class: 'entry-svg' });

  const cx = W / 2 - 30, cy = H / 2 + 6;
  const bodyLen = 230, bodyR = 26;
  const aoa = t?.totalAoA ?? 0;

  // Поток снизу вверх.
  for (let i = -2; i <= 2; i++) {
    const x = cx + i * 76;
    root.append(svg('line', { class: 'entry-flow', x1: x, y1: H - 6, x2: x, y2: H - 44 }));
    root.append(svg('path', {
      class: 'entry-flow',
      d: `M ${x - 4} ${H - 38} L ${x} ${H - 46} L ${x + 4} ${H - 38}`,
      fill: 'none',
    }));
  }

  // Нос отклонён от вектора скорости на угол атаки, брюхо смотрит в поток.
  const g = svg('g', { transform: `translate(${cx} ${cy}) rotate(${180 - aoa})` });

  const nose = -bodyLen / 2, tail = bodyLen / 2;

  g.append(svg('path', {
    class: 'entry-body',
    d: `M ${-bodyR} ${tail} L ${-bodyR} ${nose + 34} ` +
       `Q 0 ${nose - 22} ${bodyR} ${nose + 34} L ${bodyR} ${tail} Z`,
  }));

  const shield = t?.heatShield;

  // Наветренная сторона: плитки. Толщина полосы — тепловой поток, цвет —
  // близость к пределу материала.
  const heat = Math.min(1, (t?.heatFlux || 0) / 1.2e6);
  g.append(svg('path', {
    class: 'entry-tiles' + shieldClass(shield?.tiles),
    d: `M ${bodyR} ${tail} L ${bodyR} ${nose + 30}`,
    'stroke-width': 5 + 8 * heat,
  }));

  // Подветренная: голая сталь.
  g.append(svg('path', {
    class: 'entry-steel' + shieldClass(shield?.steel),
    d: `M ${-bodyR} ${tail} L ${-bodyR} ${nose + 34}`,
  }));

  // Плавники. Оба ряда — на подветренной стороне: передние у носа, задние
  // у юбки. Дальняя пара рисуется бледнее и короче: это вид сбоку, и левый
  // с правым стоят друг за другом.
  for (const f of flaps) {
    const fwd = f.name.startsWith('fwd');
    const far = f.name.endsWith('left');

    const y = fwd ? nose + 52 : tail - 34;
    const len = (fwd ? 34 : 48) * (far ? 0.78 : 1);
    const hinge = far ? -bodyR * 0.5 : -bodyR;

    // Ноль — плавник прижат к борту, упор — раскрыт поперёк потока.
    const open = Math.min(1, f.deflection / Math.max(f.limit, 1));
    const angle = 20 + 70 * open;

    const fg = svg('g', {
      class: `entry-flap${far ? ' far' : ''}` +
        `${f.manual ? ' manual' : ''}${f.jammed ? ' jammed' : ''}`,
      transform: `translate(${hinge} ${y}) rotate(${90 + angle})`,
    });
    fg.append(svg('rect', { x: 0, y: -5, width: len, height: 10, rx: 2 }));
    g.append(fg);

    if (far) continue;

    // Подпись ставится у корня плавника и разворачивается обратно, чтобы
    // остаться горизонтальной на экране.
    const lx = hinge - 10, ly = y + (fwd ? -10 : 16);
    g.append(svg('text', {
      class: 'entry-flap-label', x: lx, y: ly, 'text-anchor': 'end',
      transform: `rotate(${aoa - 180} ${lx} ${ly})`,
    }, `${num(f.deflection, 0)}°`));
  }

  root.append(g);
  root.append(entryEndView(W - 66, 74, shield));

  root.append(svg('text', { class: 'entry-caption', x: 8, y: 16 },
    `Угол атаки ${num(aoa, 1)}° · высота ${num((t?.altitude || 0) / 1000, 1)} км · ` +
    `скорость ${num(t?.totalVelocity, 0)} м/с`));

  const plate = el('div', 'entry-plate-inner');
  plate.append(root);
  return plate;
}

function shieldClass(side) {
  if (!side) return '';
  if (side.damage > 0 || side.margin < 0) return ' over';
  if (side.margin < 150) return ' hot';
  return '';
}

/* Взгляд с хвоста: какая сторона сейчас подставлена потоку.
   Дуга плиток повёрнута ровно на тот угол, который модель держит
   в подставленности брюха. */
function entryEndView(cx, cy, shield) {
  const g = svg('g', { class: 'entry-end' });
  const r = 26;

  g.append(svg('circle', {
    class: 'entry-end-body' + shieldClass(shield?.steel), cx, cy, r,
  }));

  const exposure = Math.max(-1, Math.min(1, shield?.exposure ?? 1));
  const centre = Math.acos(exposure) * 180 / Math.PI;

  // Ноль отсчёта — низ круга, куда приходит поток.
  const pt = deg => {
    const a = deg * Math.PI / 180;
    return [cx + r * Math.sin(a), cy + r * Math.cos(a)];
  };

  const pts = [];
  for (let i = 0; i <= 24; i++) pts.push(pt(centre - 70 + (140 * i) / 24));

  g.append(svg('path', {
    class: 'entry-end-tiles' + shieldClass(shield?.tiles),
    d: 'M ' + pts.map(p => `${p[0].toFixed(1)} ${p[1].toFixed(1)}`).join(' L '),
    fill: 'none',
  }));

  g.append(svg('line', {
    class: 'entry-flow', x1: cx, y1: cy + r + 22, x2: cx, y2: cy + r + 6,
  }));
  g.append(svg('path', {
    class: 'entry-flow', fill: 'none',
    d: `M ${cx - 4} ${cy + r + 12} L ${cx} ${cy + r + 4} L ${cx + 4} ${cy + r + 12}`,
  }));

  g.append(svg('text', {
    class: 'entry-flap-label', x: cx, y: cy - r - 8, 'text-anchor': 'middle',
  }, shield ? `брюхо ${num(exposure, 2)}` : 'вид с хвоста'));

  return g;
}

function flapTitle(name) {
  return {
    fwd_left: 'Передний левый',
    fwd_right: 'Передний правый',
    aft_left: 'Задний левый',
    aft_right: 'Задний правый',
  }[name] || name;
}

/* ===========================================================================
   Переключение вкладок правой панели
   =========================================================================== */

function switchDataView(view) {
  S.dataView = view;
  $$('#data-tabs .tab').forEach(t => t.classList.toggle('active', t.dataset.view === view));
  $$('#panel-data .view').forEach(v => v.classList.toggle('active', v.id === `view-${view}`));
  if (view === 'compare') refreshCompare();
  if (view === 'charts') drawCharts();
  if (view === 'cutaway') drawCutaway();
  if (view === 'entry') drawEntry();
  if (view === 'scene') drawScene3D();
}

/* ===========================================================================
   Запуск
   =========================================================================== */

async function boot() {
  // Реестр параметров приходит с сервера: интерфейс не знает заранее ни одного
  // параметра, ни его пределов, ни последствий выхода за них.
  // Каталог носителей. Профиль применяется при следующем запуске: массы,
  // геометрия и состав двигательной установки задаются на старте.
  const cat = await (await fetch('/api/profiles')).json();
  S.profiles = cat.profiles || [];
  const profileSel = $('#profile');
  for (const p of S.profiles) profileSel.append(new Option(p.title, p.id));
  profileSel.value = cat.selected;
  profileSel.onchange = () => selectProfile(profileSel.value);
  describeProfile(cat.selected);

  // Обводы носителя для развёртки: длины отсеков, расстановка двигателей,
  // радиусы сопел. Интерфейс их не выдумывает и не пересчитывает.
  await loadLayout();

  const missions = await (await fetch('/api/missions')).json();
  S.missions = missions.missions || [];
  const missionSel = $('#mission');
  if (missionSel) {
    for (const m of S.missions) missionSel.append(new Option(m.title, m.id));
    missionSel.value = missions.selected;
    missionSel.onchange = () => selectMission(missionSel.value);
  }

  S.registry = await (await fetch('/api/control/registry')).json();
  for (const p of S.registry.parameters) S.params.set(p.id, p);

  const speed = $('#speed');
  for (const v of S.registry.speeds) speed.append(new Option(`${v}×`, String(v)));
  speed.value = '1';
  speed.onchange = () => action('speed', parseFloat(speed.value));

  S.chartKeys = PUMP_CHART_KEYS.slice();

  renderSubsystemTabs();
  renderParams();
  renderPumpButtons();
  buildThrottlePanel();
  setTarget('');
  buildCharts();
  loadScenarios();
  bindControls();
  connect();
}

function bindControls() {
  // Делегированный обработчик нажатия по блоку двигателей.
  //
  // Сам #engine-strip не пересоздаётся никогда — только его содержимое,
  // каждые 100 мс, на каждый снимок телеметрии. Слушатель на click лечил
  // только потерю обработчика при подмене узла, но не саму гонку: click
  // у мыши рождается лишь тогда, когда mousedown и mouseup пришлись на один
  // и тот же, ещё существующий узел. Если перерисовка успевала подменить
  // нажатую кнопку между mousedown и mouseup, click не рождался вовсе —
  // ни на кнопке, ни на контейнере, — и делегирование тут бессильно.
  // Снаружи это выглядело как «клик иногда не срабатывает с первого раза».
  // pointerdown избавлен от этой гонки: он летит в момент нажатия, когда
  // кнопка ещё точно на месте, и не зависит от того, что случится с DOM
  // до отпускания.
  $('#engine-strip').addEventListener('pointerdown', ev => {
    const ignite = ev.target.closest('[data-ignite-engine]');
    if (ignite) {
      const id = ignite.dataset.igniteEngine;
      fetch(`/api/engines/${encodeURIComponent(id)}/ignite`, { method: 'POST' })
        .then(r => r.ok
          ? toast('ok', 'Зажигание', `${id} запущен на холостом режиме`)
          : r.text().then(t => toast('err', 'Не удалось зажечь', t)))
        .catch(() => toast('err', 'Нет связи с сервером'));
      return;
    }
    const tile = ev.target.closest('[data-select-engine]');
    if (tile) setTarget(tile.dataset.selectEngine);
  });

  $('#btn-start').onclick = () => {
    const running = S.snapshot &&
      (S.snapshot.runState === 'running' || S.snapshot.runState === 'paused');

    const go = () => {
      // История прогона относится к прогону, а не к сеансу: при новом
      // запуске графики и отметки начинаются с чистого листа.
      S.series.clear();
      S.markers = [];
      S.log = [];
      renderLog();
      renderCausal();
      // Ручная уставка газа — тоже история прогона, а не настройка сеанса.
      // Не снятая с прошлого запуска (например, после «Погасить все
      // двигатели»), она блокировала бы синхронизацию с телеметрией нового
      // прогона: панель показывала бы газ прошлого запуска — вплоть до 0 %,
      // даже когда новый носитель уже вовсю летит на полной тяге.
      THR.manual = false;
      THR.active = false;
      fetch('/api/sim/start', { method: 'POST' })
        .then(r => r.ok ? toast('ok', 'Новый прогон запускается')
                        : toast('err', 'Не удалось запустить'))
        .catch(() => toast('err', 'Нет связи с сервером'));
    };

    if (!running) { go(); return; }
    askConfirm('Начать новый прогон',
      'Текущий прогон будет остановлен, и симуляция начнётся заново.',
      'Журнал вмешательств и накопленные графики текущего прогона будут очищены.',
      go);
  };
  $('#btn-pause').onclick  = () => action('pause');
  $('#btn-resume').onclick = () => action('resume');
  $('#btn-stop').onclick   = () => action('stop');
  $('#btn-step').onclick   = () => action('step', 10);
  $('#btn-reset').onclick  = () => {
    askConfirm('Сброс симуляции',
      'Модель вернётся в исходное состояние, все воздействия будут сняты, ' +
      'режимы Unsafe overrides и Direct state override выключены.',
      'Накопленная история текущего прогона будет очищена.',
      () => {
        action('reset'); S.series.clear(); S.markers = []; S.log = []; renderLog();
        THR.manual = false; THR.active = false;
      });
  };

  $('#btn-ignite').onclick = igniteAll;
  $('#btn-emergency').onclick = () => {
    askConfirm('Аварийное выключение двигателей',
      'Все двигатели будут выключены немедленно, в обход приоритетов воздействий.',
      'Носитель перейдёт в баллистический полёт.',
      () => action('emergency-shutdown'));
  };
  $('#btn-nominal').onclick = () => {
    action('restore-nominal');
    toast('ok', 'Штатный режим', 'Все воздействия сняты');
  };
  $('#btn-release-all').onclick = () =>
    submit({ parameter: '*', mode: 'release' });

  $('#mode-unsafe').onchange = e => setMode({ unsafe: e.target.checked });
  $('#mode-direct').onchange = e => setMode({ direct: e.target.checked });

  $('#engine-target').onchange = e => {
    S.target = e.target.value;
    setTarget(S.target === '' ? '' : S.target);
    if (S.target === '') { renderEngineStrip(); renderPumps(); }
  };

  $('#param-filter').oninput = renderParams;
  $('#log-filter').oninput = renderLog;
  $('#log-status').onchange = renderLog;
  $('#btn-export-log').onclick = exportLog;

  $('#chart-window').onchange = drawCharts;
  $('#btn-baseline').onclick = () => {
    fetch('/api/control/baseline', {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ label: `Прогон №${S.snapshot?.runNumber ?? 0}` }),
    }).then(() => toast('ok', 'Эталон сохранён',
      'Текущий прогон принят за базовый для сравнения'));
  };
  $('#btn-refresh-compare').onclick = refreshCompare;

  $('#btn-entry').onclick = () => {
    fetch('/api/entry/start', { method: 'POST' })
      .then(r => r.ok ? r.json() : Promise.reject(r.statusText))
      .then(r => toast('ok', 'Сход с орбиты', r.note))
      .catch(e => toast('err', 'Сход с орбиты не принят', String(e)));
  };
  $('#btn-land').title = 'Немедленно передать управление посадочной программе, ' +
    'из любой точки полёта корабля — не только с орбиты. Двигатели выключатся, ' +
    'корабль развернётся брюхом к потоку и пойдёт по стандартному профилю ' +
    'возвращения. Доступно только кораблю (второй ступени) после разделения.';
  $('#btn-land').onclick = () => {
    fetch('/api/entry/land', { method: 'POST' })
      .then(r => r.ok ? r.json() : r.text().then(t => Promise.reject(t)))
      .then(r => toast('ok', 'Посадка начата', r.note))
      .catch(e => toast('err', 'Посадка не начата', String(e)));
  };
  $('#btn-flaps-deploy').onclick = () => {
    const deployed = S.snapshot?.telemetry?.flapsDeployed;
    entryCommand({ deploy: !deployed })
      .then(() => { drawEntry(); toast('ok', deployed ? 'Плавники прижаты' : 'Плавники выпущены'); })
      .catch(e => toast('err', 'Команда не принята', String(e)));
  };
  $('#btn-flaps-auto').onclick = () => {
    const flaps = S.snapshot?.telemetry?.flaps || [];
    Promise.all(flaps.map(f => entryCommand({ name: f.name, release: true })))
      .then(() => { drawEntry(); toast('ok', 'Плавники под автоматом'); });
  };

  $('#cutaway-labels').onchange = drawCutaway;
  $('#cutaway-engines').onchange = drawCutaway;
  $('#cutaway-zoom').oninput = ev => {
    S.cutZoom = Number(ev.target.value);
    $('#cutaway-zoom-value').textContent = `${Math.round(S.cutZoom * 100)} %`;
    drawCutaway();
  };

  // Панель меняет размер вместе с окном, и чертёж обязан за ней следовать:
  // на маленьком экране иначе не видно ничего.
  window.addEventListener('resize', () => {
    if (S.dataView === 'cutaway') drawCutaway();
  });

  $$('#data-tabs .tab').forEach(t => t.onclick = () => switchDataView(t.dataset.view));

  $('#confirm-cancel').onclick = () => {
    $('#confirm').classList.add('hidden');
    S.pendingConfirm = null;
  };
  $('#confirm-ok').onclick = () => {
    $('#confirm').classList.add('hidden');
    const fn = S.pendingConfirm;
    S.pendingConfirm = null;
    if (fn) fn();
  };

  // Сценарии.
  $('#scenario-select').onchange = e => {
    S.scenarioIndex = +e.target.value;
    renderScenarioSelect();
    renderScenarioSteps();
  };
  $('#scenario-name').onchange = e => {
    const sc = currentScenario();
    if (sc) { sc.name = e.target.value; saveScenarios(); renderScenarioSelect(); }
  };
  $('#btn-step-add').onclick = () => {
    const sc = currentScenario();
    if (!sc) return;
    sc.steps.push({ at: 0, parameter: 'tp.shaft.speed', mode: 'step',
                    basis: 'percent_current', value: 0, duration: 0 });
    saveScenarios();
    renderScenarioSteps();
  };
  $('#btn-scenario-run').onclick  = startScenario;
  $('#btn-scenario-stop').onclick = stopScenario;
  $('#btn-scenario-new').onclick  = () => {
    S.scenarios.push({ name: 'Новый сценарий', steps: [] });
    S.scenarioIndex = S.scenarios.length - 1;
    saveScenarios(); renderScenarioSelect(); renderScenarioSteps();
  };
  $('#btn-scenario-copy').onclick = () => {
    const sc = currentScenario();
    if (!sc) return;
    S.scenarios.push({ name: sc.name + ' (копия)', steps: structuredClone(sc.steps) });
    S.scenarioIndex = S.scenarios.length - 1;
    saveScenarios(); renderScenarioSelect(); renderScenarioSteps();
  };
  $('#btn-scenario-del').onclick = () => {
    if (S.scenarios.length <= 1) return;
    S.scenarios.splice(S.scenarioIndex, 1);
    S.scenarioIndex = 0;
    saveScenarios(); renderScenarioSelect(); renderScenarioSteps();
  };
  $('#btn-scenario-exp').onclick = () => {
    const sc = currentScenario();
    if (!sc) return;
    download(new Blob([JSON.stringify(sc, null, 2)], { type: 'application/json' }),
      `${sc.name}.json`);
  };
  $('#scenario-import').onchange = async e => {
    const file = e.target.files[0];
    if (!file) return;
    try {
      const sc = JSON.parse(await file.text());
      if (!sc.name || !Array.isArray(sc.steps)) throw new Error('нет полей name и steps');
      S.scenarios.push(sc);
      S.scenarioIndex = S.scenarios.length - 1;
      saveScenarios(); renderScenarioSelect(); renderScenarioSteps();
      toast('ok', 'Сценарий импортирован', sc.name);
    } catch (err) {
      toast('err', 'Не удалось прочитать файл', String(err.message || err));
    }
    e.target.value = '';
  };

  // Grafana.
  const stored = localStorage.getItem('grafana-url') || 'http://localhost:3000';
  $('#grafana-url').value = stored;
  $('#btn-grafana-open').onclick = () => {
    const url = $('#grafana-url').value.trim();
    localStorage.setItem('grafana-url', url);
    $('#grafana-frame').src =
      `${url}/d/rocket-propulsion/rocket-propulsion?kiosk&refresh=5s&theme=dark`;
    window.open(`${url}/d/rocket-telemetry/rocket-telemetry`, '_blank', 'noopener');
  };

  // Клавиши: управление ходом симуляции без мыши.
  document.addEventListener('keydown', e => {
    if (e.target.matches('input, select, textarea')) return;
    switch (e.key) {
      case ' ': e.preventDefault();
        (S.snapshot?.runState === 'running') ? action('pause') : action('resume');
        break;
      case '.': action('step', 10); break;
      case 'Escape': $('#confirm-cancel').click(); break;
    }
  });

  $('#att-flow').onchange = drawAttitude;
  window.addEventListener('resize', () => { drawCharts(); drawAttitude(); });
}

boot();

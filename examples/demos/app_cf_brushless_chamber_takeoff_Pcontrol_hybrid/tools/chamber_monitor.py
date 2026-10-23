#!/usr/bin/env python3
"""
chamber_monitor.py -- live view of the `chamber` log group (35 variables) of
app_cf_brushless_chamber_takeoff_Pcontrol_hybrid, received over Crazyradio,
showing what the on-board controller is doing.

  Top   : current state (state machine) and flags
  Left  : horizontal velocity setpoint (vxCmd / vyCmd, body frame, top view)
          walls with a successful plane fit (planeOk) are drawn in red
  Right : rotation command from yawRate (direction and magnitude)
          walls with a successful plane fit are drawn in red (rotated by wallYaw)

Usage:
  python3 chamber_monitor.py                         # radio://0/80/2M/E7E7E7E7E7
  python3 chamber_monitor.py --uri radio://0/90/2M/E7E7E7E7E7
  CFLIB_URI=radio://0/90/2M/E7E7E7E7E7 python3 chamber_monitor.py
  python3 chamber_monitor.py --period 50             # log period [ms] (default 100)
  python3 chamber_monitor.py --minimal               # subscribe only the one block needed for display
  python3 chamber_monitor.py --sim                   # simulated data, no drone needed
  python3 chamber_monitor.py --scale 2               # UI size (default 3 = 3x text and layout)

Requires: cflib (not needed with --sim), tkinter (standard library)

A log packet carries at most 26 bytes, so the 35 variables (104 bytes) are
split over 5 LogConfigs. Blocks arrive independently, so values from
different blocks are not guaranteed to share a timestamp (fine for display).
"""
import argparse
import math
import os
import random
import threading
import time
import tkinter as tk

DEFAULT_URI = 'radio://0/80/2M/E7E7E7E7E7'

# ========================= ログ / パラメータ定義 =========================
# 1 ブロック <= 26 バイト (uint8/int8 = 1, float = 4)
LOG_BLOCKS = [
    # 表示に必須のものはすべてここ (24 バイト)
    ('ctrl', ['state', 'planeOk', 'holdX', 'holdY', 'srcX', 'srcY',
              'yawAlignOk', 'armed', 'isTumbled', 'usdOn', 'flowOk',
              'handNear', 'vxCmd', 'vyCmd', 'yawRate']),
    ('yaw', ['wallYaw', 'yawCmd', 'yawErr', 'phaseElapsedS', 'yawDeg', 'estZ']),
    ('dist', ['dFront', 'dBack', 'dLeft', 'dRight', 'estX', 'estY']),
    ('plane', ['pFront', 'pBack', 'pLeft', 'pRight']),
    ('zone', ['zFront', 'zBack', 'zLeft', 'zRight']),
]

# スケーリングや「制御が有効か」の判定に使うファームウェア側パラメータ
PARAMS = {
    'centerMaxV': 0.15,   # m/s, 速度指令のクランプ
    'yawRateMax': 9.0,    # deg/s, yawRate のクランプ
    'yawDbDeg': 5.0,      # deg, yaw 不感帯
    'holdBandM': 0.05,    # m, センタリング不感帯 (前後/左右差)
    'tkoffPctl': 1,       # TAKEOFF 中もセンタリングするか
    'landPCtl': 1,        # LAND 中もセンタリングするか
    'hovWallSrc': 1,      # HOVER の距離ソース 0=zone 1=hybrid 2=plane
    'yawAlign': 1,        # yaw 合わせ有効
}

STATES = ['IDLE', 'WAIT_FOR_DECK', 'PREARM', 'ARMING',
          'ARM_DWELL', 'TAKEOFF', 'HOVER', 'LAND']
STATE_COLORS = ['#607d8b', '#8d6e63', '#ab47bc', '#ef6c00',
                '#f9a825', '#1e88e5', '#43a047', '#00897b']
ST_TAKEOFF, ST_HOVER, ST_LAND = 5, 6, 7

# planeOk のビット: bit0=前 bit1=後 bit2=左 bit3=右
WALLS = [  # (key, 表示名, センサー, planeOk ビット)
    ('Front', 'Front', 's1', 1),
    ('Back', 'Back', 's7', 2),
    ('Left', 'Left', 's5', 4),
    ('Right', 'Right', 's6', 8),
]

# ========================= 色 / フォント =========================
BG = '#1e1f24'
PANEL_BG = '#26282e'
FG = '#e8e8e8'
DIM = '#8a8d96'
WALL_FIT = '#e53935'      # 平面フィット成功
WALL_NOFIT = '#555a63'    # 失敗
VEL_COLOR = '#29b6f6'
ROT_COLOR = '#ffa726'
HOLD_COLOR = '#fdd835'
WARN = '#ff5252'
FONT = 'DejaVu Sans'
MONO = 'DejaVu Sans Mono'

# 文字とレイアウト (余白・線幅など) の倍率。main() で --scale から設定する
SCALE = 3.0


def font(size, bold=False, family=FONT):
    size = max(round(size * SCALE), 1)
    return (family, size, 'bold') if bold else (family, size)


def px(n):
    """1 倍のときのピクセル値を SCALE 倍する。"""
    return n * SCALE


def dsh(*pattern):
    """破線パターンも SCALE 倍する (Tk は 1..255 の整数のみ受け付ける)。"""
    return tuple(min(max(round(d * SCALE), 1), 255) for d in pattern)


# ========================= 受信データの保管 =========================
class Store:
    """cflib のコールバックスレッドと GUI スレッドの間で値を受け渡す。"""

    def __init__(self):
        self.lock = threading.Lock()
        self.values = {v: 0 for _, vs in LOG_BLOCKS for v in vs}
        for w in ('d', 'p', 'z'):
            for key, *_ in WALLS:
                self.values[w + key] = -1.0
        self.params = dict(PARAMS)
        self.last_rx = None
        self.rx_count = 0
        self.status = 'Not connected'
        self.errors = []

    def update(self, data):
        with self.lock:
            self.values.update(data)
            self.last_rx = time.monotonic()
            self.rx_count += 1

    def set_param(self, name, value):
        with self.lock:
            self.params[name] = value

    def set_status(self, status):
        with self.lock:
            self.status = status

    def add_error(self, msg):
        with self.lock:
            self.errors.append(msg)
            self.errors = self.errors[-3:]

    def snapshot(self):
        with self.lock:
            return (dict(self.values), dict(self.params), self.last_rx,
                    self.rx_count, self.status, list(self.errors))


# ========================= Crazyflie 接続 =========================
class CrazyflieLink:
    def __init__(self, uri, period_ms, minimal, store):
        import cflib.crtp
        from cflib.crazyflie import Crazyflie

        self.uri = uri
        self.period_ms = period_ms
        self.blocks = LOG_BLOCKS[:1] if minimal else LOG_BLOCKS
        self.store = store
        self.logconfs = []

        cflib.crtp.init_drivers()
        cache = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'cache')
        self.cf = Crazyflie(rw_cache=cache)
        self.cf.connected.add_callback(self._connected)
        self.cf.disconnected.add_callback(self._disconnected)
        self.cf.connection_failed.add_callback(self._failed)
        self.cf.connection_lost.add_callback(self._lost)

    def start(self):
        self.store.set_status(f'Connecting... {self.uri}')
        self.cf.open_link(self.uri)

    def close(self):
        for lc in self.logconfs:
            try:
                lc.stop()
            except Exception:
                pass
        self.cf.close_link()

    def _connected(self, uri):
        from cflib.crazyflie.log import LogConfig

        self.store.set_status(f'Connected {uri}')
        for name, variables in self.blocks:
            lc = LogConfig(name=f'chamber_{name}', period_in_ms=self.period_ms)
            for v in variables:
                lc.add_variable(f'chamber.{v}')
            try:
                self.cf.log.add_config(lc)
            except KeyError as e:
                # TOC に chamber.* が無い = このデモのファームウェアではない
                self.store.add_error(f'Log variable not in TOC: {e}')
                continue
            except AttributeError as e:
                self.store.add_error(f'Invalid LogConfig ({name}): {e}')
                continue
            lc.data_received_cb.add_callback(self._on_data)
            lc.error_cb.add_callback(self._on_log_error)
            lc.start()
            self.logconfs.append(lc)

        for p in PARAMS:
            full = f'chamber.{p}'
            try:
                self.cf.param.add_update_callback(group='chamber', name=p,
                                                  cb=self._on_param)
                self.cf.param.request_param_update(full)
            except Exception as e:
                self.store.add_error(f'Failed to read param {full}: {e}')

    def _on_data(self, timestamp, data, logconf):
        self.store.update({k.split('.', 1)[1]: v for k, v in data.items()})

    def _on_log_error(self, logconf, msg):
        self.store.add_error(f'{logconf.name}: {msg}')

    def _on_param(self, name, value):
        try:
            self.store.set_param(name.split('.', 1)[1], float(value))
        except ValueError:
            pass

    def _disconnected(self, uri):
        self.store.set_status(f'Disconnected {uri}')

    def _failed(self, uri, msg):
        self.store.set_status(f'Connection failed {uri}: {msg}')

    def _lost(self, uri, msg):
        self.store.set_status(f'Connection lost {uri}: {msg}')


# ========================= 疑似データ (--sim) =========================
class SimLink:
    """ファームウェアの状態遷移と制御則を真似た疑似データを流す。"""

    SCHEDULE = [(0, 2.0), (1, 1.0), (2, 1.0), (3, 0.5),
                (4, 3.0), (5, 2.0), (6, 10.0), (7, 1.5)]

    def __init__(self, store):
        self.store = store
        self.stop = threading.Event()

    def start(self):
        self.store.set_status('SIM (simulated data)')
        threading.Thread(target=self._run, daemon=True).start()

    def close(self):
        self.stop.set()

    def _run(self):
        p = PARAMS
        yaw_cmd = 0.0
        plane_ok = 0
        next_plane_change = 0.0
        while not self.stop.is_set():
            for state, dur in self.SCHEDULE:
                t0 = time.monotonic()
                while not self.stop.is_set():
                    t = time.monotonic() - t0
                    if t >= dur:
                        break
                    v = {'state': state, 'phaseElapsedS': t,
                         'armed': int(state >= 4), 'flowOk': int(state >= 1),
                         'usdOn': int(state >= ST_TAKEOFF), 'isTumbled': 0,
                         'handNear': int(state == 0 and t > 1.2)}
                    if state >= ST_TAKEOFF:
                        now = time.monotonic()
                        if now >= next_plane_change:
                            plane_ok = (random.choice([15, 15, 15, 13, 11, 7, 14])
                                        if state == ST_HOVER
                                        else random.choice([0, 0, 1, 4, 8]))
                            next_plane_change = now + random.uniform(0.4, 1.2)
                        offx = 0.08 * math.sin(0.5 * now)
                        offy = 0.06 * math.cos(0.7 * now)
                        z = {'Front': 0.25 - offx, 'Back': 0.25 + offx,
                             'Left': 0.25 - offy, 'Right': 0.25 + offy}
                        pl = {k: z[k] + 0.03 for k in z}
                        for key, _, _, bit in WALLS:
                            v['z' + key] = z[key]
                            v['p' + key] = pl[key] if plane_ok & bit else -1.0
                        use_px = state == ST_HOVER and (plane_ok & 3) == 3
                        use_py = state == ST_HOVER and (plane_ok & 12) == 12
                        src = {'Front': pl if use_px else z, 'Back': pl if use_px else z,
                               'Left': pl if use_py else z, 'Right': pl if use_py else z}
                        for key, *_ in WALLS:
                            v['d' + key] = src[key][key]
                        v['srcX'] = int(use_px)
                        v['srcY'] = int(use_py)
                        dx = v['dFront'] - v['dBack']
                        dy = v['dLeft'] - v['dRight']
                        v['holdX'] = int(abs(dx) <= p['holdBandM'])
                        v['holdY'] = int(abs(dy) <= p['holdBandM'])
                        lim = p['centerMaxV']
                        v['vxCmd'] = 0.0 if v['holdX'] else max(-lim, min(lim, 0.3 * dx))
                        v['vyCmd'] = 0.0 if v['holdY'] else max(-lim, min(lim, 0.3 * dy))
                        v['planeOk'] = plane_ok
                        v['estX'] = -offx * 0.5
                        v['estY'] = -offy * 0.5
                        v['estZ'] = (min(t / dur, 1) * 0.25 if state == ST_TAKEOFF
                                     else 0.25 if state == ST_HOVER
                                     else max(0.25 * (1 - t / dur), 0))
                    else:
                        v.update(vxCmd=0.0, vyCmd=0.0, holdX=0, holdY=0)
                    if state == ST_HOVER:
                        wall_yaw = 18.0 * math.cos(0.45 * t)
                        err = -wall_yaw
                        rate = 0.0
                        if abs(wall_yaw) >= p['yawDbDeg']:
                            rate = max(-p['yawRateMax'], min(p['yawRateMax'], 0.2 * err))
                            yaw_cmd += rate * 0.05
                        v.update(wallYaw=wall_yaw, yawErr=err, yawRate=rate,
                                 yawCmd=yaw_cmd, yawAlignOk=1, yawDeg=yaw_cmd)
                    elif state == 0:
                        yaw_cmd = 0.0
                    self.store.update(v)
                    time.sleep(0.05)


# ========================= 描画ヘルパ =========================
def arrow_polyline(cx, cy, r, start_deg, extent_deg, steps=40):
    """数学座標の角度 (反時計回り正, 0=右) で円弧上の点列を返す。"""
    pts = []
    for i in range(steps + 1):
        a = math.radians(start_deg + extent_deg * i / steps)
        pts += [cx + r * math.cos(a), cy - r * math.sin(a)]
    return pts


def rotate(x, y, deg):
    a = math.radians(deg)
    return (x * math.cos(a) - y * math.sin(a), x * math.sin(a) + y * math.cos(a))


def draw_drone(c, cx, cy, size, color=FG):
    """上から見た機体。前 (機首) が画面上向き。"""
    arm = size
    for ang in (45, 135, 225, 315):
        x, y = rotate(arm, 0, ang)
        c.create_line(cx, cy, cx + x, cy - y, fill=color, width=px(3))
        c.create_oval(cx + x - size * 0.35, cy - y - size * 0.35,
                      cx + x + size * 0.35, cy - y + size * 0.35,
                      outline=color, width=px(2))
    c.create_polygon(cx, cy - size * 0.9, cx - size * 0.3, cy - size * 0.3,
                     cx + size * 0.3, cy - size * 0.3, fill=color, outline='')


def draw_notice(c, x, y, text):
    """背景付きの注意書き (下の図形と重なっても読めるように)。"""
    t = c.create_text(x, y, fill=HOLD_COLOR, font=font(12, True), text=text,
                      justify='center')
    x1, y1, x2, y2 = c.bbox(t)
    r = c.create_rectangle(x1 - px(10), y1 - px(6), x2 + px(10), y2 + px(6),
                           fill='#2f2a18', outline=HOLD_COLOR, width=px(1))
    c.tag_lower(r, t)


def fmt_dist(v):
    return f'{v:.3f}' if v is not None and v >= 0 else '  —  '


# ========================= GUI =========================
class MonitorApp:
    def __init__(self, root, store, link):
        self.root = root
        self.store = store
        self.link = link
        self.rate_hist = []

        root.title('Chamber Monitor')
        root.configure(bg=BG)
        root.geometry(f'{round(px(1280))}x{round(px(800))}')
        root.minsize(800, 500)
        if SCALE > 1.5:
            # 拡大表示では画面に収まらないことが多いので最大化して開く
            try:
                root.attributes('-zoomed', True)
            except tk.TclError:
                pass

        self.header = tk.Canvas(root, height=px(150), bg=BG, highlightthickness=0)
        self.header.pack(fill='x', padx=px(12), pady=(px(10), px(4)))

        body = tk.Frame(root, bg=BG)
        body.pack(fill='both', expand=True, padx=px(12))
        self.left = tk.Canvas(body, bg=PANEL_BG, highlightthickness=0)
        self.right = tk.Canvas(body, bg=PANEL_BG, highlightthickness=0)
        self.left.pack(side='left', fill='both', expand=True, padx=(0, px(6)), pady=px(4))
        self.right.pack(side='left', fill='both', expand=True, padx=(px(6), 0), pady=px(4))

        self.statusbar = tk.Label(root, anchor='w', justify='left', bg=BG, fg=DIM,
                                  font=font(10))
        self.statusbar.pack(fill='x', padx=px(12), pady=(px(2), px(8)))

        root.protocol('WM_DELETE_WINDOW', self.on_close)
        self.tick()

    def on_close(self):
        try:
            self.link.close()
        finally:
            self.root.destroy()

    # ---------- 周期更新 ----------
    def tick(self):
        v, p, last_rx, rx_count, status, errors = self.store.snapshot()
        now = time.monotonic()
        age = None if last_rx is None else now - last_rx
        stale = age is None or age > 1.0

        self.rate_hist.append((now, rx_count))
        self.rate_hist = [(t, n) for t, n in self.rate_hist if now - t <= 2.0]
        rate = 0.0
        if len(self.rate_hist) >= 2:
            (t0, n0), (t1, n1) = self.rate_hist[0], self.rate_hist[-1]
            rate = (n1 - n0) / (t1 - t0) if t1 > t0 else 0.0

        self.draw_header(v, stale)
        self.draw_left(v, p, stale)
        self.draw_right(v, p, stale)

        age_txt = '—' if age is None else f'{age:.2f} s ago'
        # 拡大すると 1 行に収まらないので 2 行に分ける
        txt = (f'{status}   |   RX {rate:5.1f} pkt/s   |   last RX {age_txt}\n'
               f'centerMaxV={p["centerMaxV"]:.2f} m/s  yawRateMax={p["yawRateMax"]:.1f} deg/s  '
               f'yawDb={p["yawDbDeg"]:.1f}°  holdBand={p["holdBandM"]:.3f} m  '
               f'hovWallSrc={int(p["hovWallSrc"])}')
        if errors:
            txt += '\n' + ' / '.join(errors)
        self.statusbar.configure(text=txt, fg=WARN if (errors or stale) else DIM,
                                 wraplength=max(self.root.winfo_width() - px(24), 200))
        self.root.after(50, self.tick)

    # ---------- 上部: state ----------
    def draw_header(self, v, stale):
        c = self.header
        c.delete('all')
        w = c.winfo_width()
        st = int(v['state'])
        name = STATES[st] if 0 <= st < len(STATES) else f'UNKNOWN({st})'
        color = STATE_COLORS[st] if 0 <= st < len(STATES) else WARN

        c.create_rectangle(0, 0, px(360), px(92), fill=color, outline='')
        c.create_text(px(16), px(18), text='STATE', anchor='w', fill='#ffffff', font=font(11))
        c.create_text(px(16), px(56), text=name, anchor='w', fill='#ffffff', font=font(30, True))
        c.create_text(px(348), px(18), text=f'{st}', anchor='e', fill='#ffffff', font=font(11))

        # 状態遷移の帯
        x0, chip_w = px(380), max((w - px(380)) / len(STATES), px(60))
        for i, s in enumerate(STATES):
            x = x0 + i * chip_w
            active = i == st
            c.create_rectangle(x + px(2), px(8), x + chip_w - px(2), px(48),
                               fill=STATE_COLORS[i] if active else '#33363d', outline='')
            fsize = 10 if chip_w >= len(s) * px(9) else 8
            c.create_text(x + chip_w / 2, px(28), text=s,
                          fill='#ffffff' if active else DIM,
                          font=font(fsize, active))
            if i < len(STATES) - 1:
                c.create_text(x + chip_w, px(28), text='›', fill=DIM, font=font(12))

        c.create_text(x0, px(70), anchor='w', fill=FG, font=font(12),
                      text=f'Elapsed {v["phaseElapsedS"]:5.2f} s     '
                           f'est x={v["estX"]:+.3f}  y={v["estY"]:+.3f}  z={v["estZ"]:.3f} m     '
                           f'yaw(est)={v["yawDeg"]:+.1f}°')

        flags = [('ARMED', v['armed'], '#43a047'), ('FLOW', v['flowOk'], '#43a047'),
                 ('uSD LOG', v['usdOn'], '#1e88e5'), ('HAND', v['handNear'], '#f9a825'),
                 ('TUMBLED', v['isTumbled'], WARN)]
        x = 0
        for label, on, col in flags:
            c.create_rectangle(x, px(106), x + px(110), px(140),
                               fill=col if on else '#33363d', outline='')
            c.create_text(x + px(55), px(123), text=label, fill='#ffffff' if on else DIM,
                          font=font(11, bool(on)))
            x += px(118)
        if stale:
            c.create_text(w - px(4), px(123), anchor='e', text='NO DATA', fill=WARN,
                          font=font(18, True))

    # ---------- 左: 水平速度 setpoint ----------
    def centering_active(self, st, p):
        return (st == ST_HOVER or (st == ST_TAKEOFF and p['tkoffPctl'])
                or (st == ST_LAND and p['landPCtl']))

    def draw_left(self, v, p, stale):
        c = self.left
        c.delete('all')
        w, h = c.winfo_width(), c.winfo_height()
        st = int(v['state'])
        active = self.centering_active(st, p) and not stale

        c.create_text(px(16), px(18), anchor='w', fill=FG, font=font(14, True),
                      text='Horizontal velocity setpoint (body frame, top view)')
        c.create_text(px(16), px(42), anchor='w', fill=DIM, font=font(10),
                      text='Arrow = (vxCmd, vyCmd)   Red wall = plane fit OK')

        table_h = px(150)
        side = max(min(w - px(140), h - px(80) - table_h - px(60)), px(120))
        cx, cy = w / 2, px(70) + px(30) + side / 2
        half = side / 2
        plane_ok = int(v['planeOk'])

        # 壁 (前=上, 後=下, 左=左, 右=右)
        edges = {'Front': (cx - half, cy - half, cx + half, cy - half),
                 'Back': (cx - half, cy + half, cx + half, cy + half),
                 'Left': (cx - half, cy - half, cx - half, cy + half),
                 'Right': (cx + half, cy - half, cx + half, cy + half)}
        src_name = {1: 'plane', 0: 'zone', -1: 'none'}
        for key, label_name, sensor, bit in WALLS:
            fit = bool(plane_ok & bit)
            c.create_line(*edges[key], fill=WALL_FIT if fit else WALL_NOFIT,
                          width=px(10), capstyle='round')
            src = int(v['srcX'] if key in ('Front', 'Back') else v['srcY'])
            label = (f'{label_name} {sensor}  d={fmt_dist(v["d" + key])} m '
                     f'[{src_name.get(src, "?")}]')
            col = WALL_FIT if fit else DIM
            if key == 'Front':
                c.create_text(cx, cy - half - px(18), text=label, fill=col, font=font(11))
            elif key == 'Back':
                c.create_text(cx, cy + half + px(18), text=label, fill=col, font=font(11))
            elif key == 'Left':
                c.create_text(cx - half - px(20), cy, text=label, fill=col, font=font(11),
                              angle=90)
            else:
                c.create_text(cx + half + px(20), cy, text=label, fill=col, font=font(11),
                              angle=-90)

        # 最大速度の目安円と軸
        max_v = p['centerMaxV'] if p['centerMaxV'] > 0 else 0.15
        r_max = half * 0.82
        c.create_oval(cx - r_max, cy - r_max, cx + r_max, cy + r_max,
                      outline='#3c4049', dash=dsh(4, 4), width=px(1))
        c.create_text(cx + r_max * 0.72, cy - r_max * 0.72, anchor='se', fill='#5c616b',
                      font=font(9), text=f'{max_v:.2f} m/s')
        c.create_line(cx, cy - r_max, cx, cy + r_max, fill='#3c4049', width=px(1))
        c.create_line(cx - r_max, cy, cx + r_max, cy, fill='#3c4049', width=px(1))

        # hold 中の軸を強調
        if active and v['holdX']:
            c.create_line(cx, cy - r_max, cx, cy + r_max, fill=HOLD_COLOR, width=px(3),
                          dash=dsh(6, 4))
        if active and v['holdY']:
            c.create_line(cx - r_max, cy, cx + r_max, cy, fill=HOLD_COLOR, width=px(3),
                          dash=dsh(6, 4))

        draw_drone(c, cx, cy, max(side * 0.04, px(10)), '#6b6f78' if active else '#4a4e56')

        vx, vy = float(v['vxCmd']), float(v['vyCmd'])
        scale = r_max / max_v
        speed = math.hypot(vx, vy)
        # 小さい指令でも向きが読めるよう、矢印は最低 30 px 描く (大きさは数値で読む)
        min_len = px(30)
        if 0 < speed * scale < min_len:
            scale = min_len / speed
        ex, ey = cx - vy * scale, cy - vx * scale   # +x=前=上, +y=左=左
        if speed > 1e-4:
            c.create_line(cx, cy, ex, cy, fill=VEL_COLOR, width=px(2), dash=dsh(3, 3))
            c.create_line(cx, cy, cx, ey, fill=VEL_COLOR, width=px(2), dash=dsh(3, 3))
            c.create_line(cx, cy, ex, ey, fill=VEL_COLOR if active else DIM,
                          width=px(7), arrow='last', arrowshape=(px(18), px(22), px(9)))

        # 数値と向き
        if speed > 1e-4:
            ang = math.degrees(math.atan2(vy, vx))   # 0=前, +=左
            if abs(ang) < 1:
                dir_txt = 'forward'
            elif abs(abs(ang) - 180) < 1:
                dir_txt = 'backward'
            else:
                dir_txt = f'{"left" if ang > 0 else "right"} {abs(ang):.0f}° from forward'
        else:
            dir_txt = 'stopped'
        y0 = cy + half + px(46)
        c.create_text(cx, y0, fill=VEL_COLOR if active else DIM, font=font(14, True),
                      text=f'|v| = {speed:.3f} m/s   Direction: {dir_txt}')
        hold_x = '  [X HOLD]' if v['holdX'] else ''
        hold_y = '  [Y HOLD]' if v['holdY'] else ''
        c.create_text(cx, y0 + px(24), fill=FG, font=font(11),
                      text=f'vxCmd = {vx:+.3f} m/s{hold_x}      vyCmd = {vy:+.3f} m/s{hold_y}')

        # 距離の比較表 (d = 制御に使用, p = 平面, z = ゾーン平均)
        ty = y0 + px(50)
        cols = [cx - px(190), cx - px(90), cx + px(10), cx + px(110)]
        for i, head in enumerate(['Wall', 'd used', 'p plane', 'z zone']):
            c.create_text(cols[i], ty, anchor='w', fill=DIM, font=font(10), text=head)
        for row, (key, label_name, sensor, bit) in enumerate(WALLS):
            yy = ty + px(18) + row * px(18)
            col = WALL_FIT if plane_ok & bit else FG
            c.create_text(cols[0], yy, anchor='w', fill=col, font=font(10),
                          text=f'{label_name} ({sensor})')
            for i, pre in enumerate(['d', 'p', 'z']):
                c.create_text(cols[i + 1], yy, anchor='w', fill=FG, font=font(10, family=MONO),
                              text=fmt_dist(v[pre + key]))

        if not active:
            reason = ('NO DATA' if stale else
                      f'Centering inactive (state={STATES[st] if 0 <= st < 8 else st})')
            draw_notice(c, cx, cy + half * 0.55, reason + '\nShowing last received values')

    # ---------- 右: yaw 回転指令 ----------
    def draw_right(self, v, p, stale):
        c = self.right
        c.delete('all')
        w, h = c.winfo_width(), c.winfo_height()
        st = int(v['state'])
        active = st == ST_HOVER and not stale
        align_ok = bool(v['yawAlignOk'])

        c.create_text(px(16), px(18), anchor='w', fill=FG, font=font(14, True),
                      text='Yaw rotation command (yawRate)')
        c.create_text(px(16), px(42), anchor='w', fill=DIM, font=font(10),
                      text='Walls rotated by wallYaw   Red wall = plane fit OK   CCW = +')

        side = max(min(w - px(140), h - px(80) - px(210)), px(120))
        cx, cy = w / 2, px(70) + px(30) + side / 2
        half = side / 2
        plane_ok = int(v['planeOk'])

        # 壁を -wallYaw 回転: wallYaw>0 は機体が壁に対して反時計回りにずれている
        # (yawAlignSign=-1 で正しく収束する、というファーム側の設定に基づく)
        wall_yaw = float(v['wallYaw']) if align_ok else 0.0
        phi = -wall_yaw
        segs = {'Front': ((-half, half), (half, half)),
                'Back': ((-half, -half), (half, -half)),
                'Left': ((-half, -half), (-half, half)),
                'Right': ((half, -half), (half, half))}
        for key, label_name, sensor, bit in WALLS:
            fit = bool(plane_ok & bit)
            (x1, y1), (x2, y2) = segs[key]
            x1, y1 = rotate(x1, y1, phi)
            x2, y2 = rotate(x2, y2, phi)
            c.create_line(cx + x1, cy - y1, cx + x2, cy - y2,
                          fill=WALL_FIT if fit else WALL_NOFIT, width=px(10), capstyle='round',
                          dash=() if align_ok else dsh(8, 6))
            mx, my = rotate(*[(a + b) / 2 for a, b in zip(*segs[key])], phi)
            nx, ny = (mx / half * px(14), my / half * px(14))
            # 文字が壁に重ならないよう、壁の外側にアンカーを置く
            anchor = {'Front': 's', 'Back': 'n', 'Left': 'e', 'Right': 'w'}[key]
            c.create_text(cx + mx + nx, cy - my - ny, text=f'{label_name} {sensor}',
                          fill=WALL_FIT if fit else DIM, font=font(11), anchor=anchor)

        # 機首方向と、壁に正対したときの向き
        r_ring = half * 0.55
        c.create_line(cx, cy, cx, cy - r_ring * 1.25, fill=FG, width=px(2))
        if align_ok:
            sx, sy = rotate(0, r_ring * 1.25, phi)
            c.create_line(cx, cy, cx + sx, cy - sy, fill=WALL_FIT, width=px(2), dash=dsh(5, 4))
            db = p['yawDbDeg']
            c.create_arc(cx - r_ring * 1.25, cy - r_ring * 1.25, cx + r_ring * 1.25, cy + r_ring * 1.25,
                         start=90 + phi - db, extent=2 * db, style='pieslice',
                         outline='', fill='#3a3020')
        draw_drone(c, cx, cy, max(side * 0.06, px(14)), FG if active else DIM)

        # 回転矢印: 0 の向き (機首=上=90°) から yawRate に比例した円弧
        rate = float(v['yawRate'])
        rate_max = p['yawRateMax'] if p['yawRateMax'] > 0 else 9.0
        extent = max(-1.0, min(1.0, rate / rate_max)) * 160.0
        c.create_oval(cx - r_ring, cy - r_ring, cx + r_ring, cy + r_ring,
                      outline='#3c4049', dash=dsh(4, 4), width=px(1))
        if abs(extent) > 0.5:
            pts = arrow_polyline(cx, cy, r_ring, 90, extent)
            c.create_line(*pts, fill=ROT_COLOR if active else DIM, width=px(9),
                          arrow='last', arrowshape=(px(20), px(24), px(11)), smooth=True)

        if abs(rate) < 1e-3:
            if active and align_ok:
                dir_txt = f'No rotation (|wallYaw| < {p["yawDbDeg"]:.0f}° deadband)'
            elif active:
                dir_txt = 'No rotation (wall angle invalid → heading held)'
            else:
                dir_txt = 'No rotation'
        else:
            dir_txt = 'Rotate left (CCW)' if rate > 0 else 'Rotate right (CW)'

        y0 = cy + half + px(46)
        c.create_text(cx, y0, fill=ROT_COLOR if active else DIM, font=font(14, True),
                      text=f'yawRate = {rate:+.2f} deg/s   {dir_txt}')

        # yawRate ゲージ (-max .. +max)
        gw = min(w - px(80), px(520))
        gx0, gy = cx - gw / 2, y0 + px(30)
        c.create_rectangle(gx0, gy, gx0 + gw, gy + px(14), fill='#33363d', outline='')
        frac = max(-1.0, min(1.0, rate / rate_max))
        c.create_rectangle(cx, gy, cx - frac * gw / 2, gy + px(14),
                           fill=ROT_COLOR if active else DIM, outline='')
        c.create_line(cx, gy - px(4), cx, gy + px(18), fill=FG, width=px(1))
        c.create_text(gx0, gy + px(26), anchor='w', fill=DIM, font=font(9),
                      text=f'+{rate_max:.0f} left (CCW)')
        c.create_text(gx0 + gw, gy + px(26), anchor='e', fill=DIM, font=font(9),
                      text=f'right (CW) -{rate_max:.0f}')

        c.create_text(cx, gy + px(56), fill=FG, font=font(11),
                      text=f'wallYaw = {v["wallYaw"]:+.1f}°    yawErr = {v["yawErr"]:+.1f}°    '
                           f'yawCmd = {v["yawCmd"]:+.1f}°    yawAlignOk = {int(v["yawAlignOk"])}')

        if not active:
            reason = ('NO DATA' if stale else
                      'Yaw alignment runs only in HOVER (other states hold holdYawDeg)')
            draw_notice(c, cx, cy + half * 0.6, reason + '\nShowing last received values')


def main():
    global SCALE
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--uri', default=os.environ.get('CFLIB_URI', DEFAULT_URI))
    ap.add_argument('--period', type=int, default=100, help='log period [ms] (default 100)')
    ap.add_argument('--minimal', action='store_true',
                    help='subscribe only the ctrl block needed for display (minimal radio load)')
    ap.add_argument('--sim', action='store_true', help='show simulated data, no drone needed')
    ap.add_argument('--scale', type=float, default=SCALE,
                    help=f'size of text and layout (default {SCALE:g} = {SCALE:g}x)')
    args = ap.parse_args()
    SCALE = args.scale

    store = Store()
    link = SimLink(store) if args.sim else CrazyflieLink(args.uri, args.period,
                                                        args.minimal, store)
    root = tk.Tk()
    MonitorApp(root, store, link)
    link.start()
    root.mainloop()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
chamber_monitor.py -- for app_cf_brushless_chamber_without_Pcontrol.
Receives the `chamber` log group (35 variables) over Crazyradio in real time
and shows what the on-board controller is doing.

This build is the control-experiment side (no wall centering, no yaw
alignment), so the panels mean something different from the hybrid monitor:

  Top   : current state and flags
  Left  : actual horizontal setpoint = absolute hold of the takeoff x/y.
          solid arrow  = takeoff point - estimated position
                         (direction the position controller pulls, body frame)
          dashed arrow = vxCmd/vyCmd (what P control would command; counterfactual, not applied)
          walls with a successful plane fit are drawn in red
  Right : actual yaw setpoint = fixed holdYawDeg (yawRate is always 0).
          solid arc    = holdYawDeg - estimated yaw (direction attitude control turns)
          dashed arc   = rate the hybrid yaw alignment would command (reference, not applied)
          walls rotated by wallYaw; walls with a successful plane fit are drawn in red

Usage:
  python3 chamber_monitor.py                         # radio://0/80/2M/E7E7E7E7E7
  python3 chamber_monitor.py --uri radio://0/90/2M/E7E7E7E7E7
  CFLIB_URI=radio://0/90/2M/E7E7E7E7E7 python3 chamber_monitor.py
  python3 chamber_monitor.py --period 50             # log period [ms] (default 100)
  python3 chamber_monitor.py --minimal               # subscribe only the 2 blocks needed for display
  python3 chamber_monitor.py --sim                   # simulated data, no drone needed
  python3 chamber_monitor.py --scale 2               # UI size (default 3 = 3x text and layout)

Requires: cflib (not needed with --sim), tkinter (standard library)

A log packet carries at most 26 bytes, so the 35 variables (104 bytes) are
split over 5 LogConfigs. The takeoff point (takeoffX/Y) is not logged, so the
estX/estY at the moment state changes ARM_DWELL -> TAKEOFF is recorded here
(the firmware uses the estimate at that same moment as the takeoff point).
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
    # 表示に必須 (24 バイト): 離陸点からのずれに estX/estY が要る
    ('ctrl', ['state', 'planeOk', 'yawAlignOk', 'armed', 'isTumbled', 'usdOn',
              'flowOk', 'handNear', 'vxCmd', 'vyCmd', 'estX', 'estY']),
    # 表示に必須 (26 バイト): 機体座標への回転と壁角度
    ('yaw', ['wallYaw', 'yawCmd', 'yawDeg', 'phaseElapsedS', 'estZ', 'yawRate',
             'holdX', 'holdY']),
    ('dist', ['dFront', 'dBack', 'dLeft', 'dRight', 'yawErr', 'srcX', 'srcY']),
    ('plane', ['pFront', 'pBack', 'pLeft', 'pRight']),
    ('zone', ['zFront', 'zBack', 'zLeft', 'zRight']),
]
MINIMAL_BLOCKS = 2

# このビルドに存在するパラメータ
PARAMS = {
    'centerMaxV': 0.15,   # m/s, 反事実の速度指令のクランプ
    'kCenter': 0.3,       # 反事実の P ゲイン
    'holdYawDeg': 0.0,    # deg, 全フェーズで保持する yaw
}

# 参考表示用: hybrid 版の yaw 合わせの既定値 (このビルドには存在しない)
REF_YAW_KP = 0.2          # 1/s
REF_YAW_SIGN = -1.0
REF_YAW_RATE_MAX = 9.0    # deg/s
REF_YAW_DB = 5.0          # deg

POS_ERR_FULL = 0.10       # m, 位置誤差矢印がリング半径になる大きさ
YAW_ERR_FULL = 15.0       # deg, 向き誤差円弧が最大になる大きさ

STATES = ['IDLE', 'WAIT_FOR_DECK', 'PREARM', 'ARMING',
          'ARM_DWELL', 'TAKEOFF', 'HOVER', 'LAND']
STATE_COLORS = ['#607d8b', '#8d6e63', '#ab47bc', '#ef6c00',
                '#f9a825', '#1e88e5', '#43a047', '#00897b']
ST_IDLE, ST_ARM_DWELL, ST_TAKEOFF, ST_HOVER, ST_LAND = 0, 4, 5, 6, 7
FLYING = (ST_TAKEOFF, ST_HOVER, ST_LAND)

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
POS_COLOR = '#66bb6a'     # 実際の位置保持の補正方向
VEL_COLOR = '#29b6f6'     # 反事実の速度指令
ROT_COLOR = '#ffa726'     # 実際の向き保持の補正方向
REF_COLOR = '#ce93d8'     # 参考: yaw 合わせの回転
NOTE_COLOR = '#fdd835'
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


def wrap180(deg):
    return (deg + 180.0) % 360.0 - 180.0


# ========================= 受信データの保管 =========================
class Store:
    """cflib のコールバックスレッドと GUI スレッドの間で値を受け渡す。"""

    def __init__(self):
        self.lock = threading.Lock()
        self.values = {v: 0 for _, vs in LOG_BLOCKS for v in vs}
        for w in ('d', 'p', 'z'):
            for key, *_ in WALLS:
                self.values[w + key] = -1.0
        self.values['srcX'] = self.values['srcY'] = -2
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
        self.blocks = LOG_BLOCKS[:MINIMAL_BLOCKS] if minimal else LOG_BLOCKS
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
    """ファームウェアの状態遷移と記録内容を真似た疑似データを流す。
    位置は離陸点からゆっくりドリフトし、壁距離はそのドリフトに追従する。"""

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
        plane_ok = 0
        next_plane_change = 0.0
        while not self.stop.is_set():
            ex, ey, yaw = 0.0, 0.0, 0.0
            for state, dur in self.SCHEDULE:
                t0 = time.monotonic()
                while not self.stop.is_set():
                    t = time.monotonic() - t0
                    if t >= dur:
                        break
                    now = time.monotonic()
                    v = {'state': state, 'phaseElapsedS': t,
                         'armed': int(state >= 4), 'flowOk': int(state >= 1),
                         'usdOn': int(state in FLYING), 'isTumbled': 0,
                         'handNear': int(state == 0 and t > 1.2),
                         'yawCmd': p['holdYawDeg'], 'yawRate': 0.0, 'yawErr': 0.0,
                         'holdX': 0, 'holdY': 0, 'srcX': -2, 'srcY': -2}
                    if state in FLYING:
                        # 推定位置は離陸点 (0,0) の周りでゆらぐ。真の位置はさらにドリフト
                        ex = 0.03 * math.sin(0.6 * now)
                        ey = 0.025 * math.cos(0.45 * now)
                        yaw = 3.0 * math.sin(0.35 * now)
                        drift_x = 0.06 * math.sin(0.15 * now)
                        drift_y = 0.05 * math.cos(0.2 * now)
                        v.update(estX=ex, estY=ey, yawDeg=yaw,
                                 estZ=(min(t / dur, 1) * 0.25 if state == ST_TAKEOFF
                                       else 0.25 if state == ST_HOVER
                                       else max(0.25 * (1 - t / dur), 0)))
                    if state == ST_HOVER:
                        if now >= next_plane_change:
                            plane_ok = random.choice([15, 15, 15, 13, 11, 7, 14])
                            next_plane_change = now + random.uniform(0.4, 1.2)
                        z = {'Front': 0.22 - drift_x, 'Back': 0.22 + drift_x,
                             'Left': 0.22 - drift_y, 'Right': 0.22 + drift_y}
                        for key, _, _, bit in WALLS:
                            v['d' + key] = z[key]
                            v['z' + key] = z[key]
                            v['p' + key] = z[key] + 0.03 if plane_ok & bit else -1.0
                        lim = p['centerMaxV']
                        v['vxCmd'] = max(-lim, min(lim, p['kCenter'] * (v['dFront'] - v['dBack'])))
                        v['vyCmd'] = max(-lim, min(lim, p['kCenter'] * (v['dLeft'] - v['dRight'])))
                        v['planeOk'] = plane_ok
                        v['wallYaw'] = 12.0 * math.sin(0.3 * now)
                        v['yawAlignOk'] = 1
                    elif state == ST_IDLE:
                        v.update(vxCmd=0.0, vyCmd=0.0)
                    self.store.update(v)
                    time.sleep(0.05)


# ========================= 描画ヘルパ =========================
def arc_points(cx, cy, r, start_deg, extent_deg, steps=40):
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
    for ang in (45, 135, 225, 315):
        x, y = rotate(size, 0, ang)
        c.create_line(cx, cy, cx + x, cy - y, fill=color, width=px(3))
        c.create_oval(cx + x - size * 0.35, cy - y - size * 0.35,
                      cx + x + size * 0.35, cy - y + size * 0.35,
                      outline=color, width=px(2))
    c.create_polygon(cx, cy - size * 0.9, cx - size * 0.3, cy - size * 0.3,
                     cx + size * 0.3, cy - size * 0.3, fill=color, outline='')


def draw_notice(c, x, y, text):
    """背景付きの注意書き (下の図形と重なっても読めるように)。"""
    t = c.create_text(x, y, fill=NOTE_COLOR, font=font(12, True), text=text,
                      justify='center')
    x1, y1, x2, y2 = c.bbox(t)
    r = c.create_rectangle(x1 - px(10), y1 - px(6), x2 + px(10), y2 + px(6),
                           fill='#2f2a18', outline=NOTE_COLOR, width=px(1))
    c.tag_lower(r, t)


def draw_body_arrow(c, cx, cy, fwd, left, scale, color, width, dash=()):
    """機体座標 (前=+, 左=+) のベクトルを矢印で描く。最低 30 px (1 倍時)。
    width と dash は 1 倍のときの値で渡す。"""
    mag = math.hypot(fwd, left)
    if mag < 1e-6:
        return
    if mag * scale < px(30):
        scale = px(30) / mag
    shape = (18, 22, 9) if width > 4 else (12, 15, 6)
    c.create_line(cx, cy, cx - left * scale, cy - fwd * scale, fill=color,
                  width=px(width), dash=dsh(*dash) if dash else (), arrow='last',
                  arrowshape=tuple(px(a) for a in shape))


def direction_text(fwd, left):
    if math.hypot(fwd, left) < 1e-6:
        return '—'
    ang = math.degrees(math.atan2(left, fwd))   # 0=前, +=左
    if abs(ang) < 1:
        return 'forward'
    if abs(abs(ang) - 180) < 1:
        return 'backward'
    return f'{"left" if ang > 0 else "right"} {abs(ang):.0f}° from fwd'


def fmt_dist(v):
    return f'{v:.3f}' if v is not None and v >= 0 else '  —  '


# ========================= GUI =========================
class MonitorApp:
    def __init__(self, root, store, link):
        self.root = root
        self.store = store
        self.link = link
        self.rate_hist = []
        self.prev_state = None
        self.takeoff_xy = None   # ARM_DWELL -> TAKEOFF の瞬間の推定位置

        root.title('Chamber Monitor (without P control)')
        root.configure(bg=BG)
        root.geometry(f'{round(px(1280))}x{round(px(820))}')
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

    def track_takeoff(self, v):
        st = int(v['state'])
        if st == ST_TAKEOFF and self.prev_state == ST_ARM_DWELL:
            self.takeoff_xy = (float(v['estX']), float(v['estY']))
        elif st not in FLYING:
            self.takeoff_xy = None
        self.prev_state = st

    # ---------- 周期更新 ----------
    def tick(self):
        v, p, last_rx, rx_count, status, errors = self.store.snapshot()
        now = time.monotonic()
        age = None if last_rx is None else now - last_rx
        stale = age is None or age > 1.0
        if last_rx is not None:
            self.track_takeoff(v)

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
               f'kCenter={p["kCenter"]:.2f}  centerMaxV={p["centerMaxV"]:.2f} m/s  '
               f'holdYawDeg={p["holdYawDeg"]:+.1f}°')
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
        c.create_text(px(16), px(52), text=name, anchor='w', fill='#ffffff', font=font(28, True))
        c.create_text(px(348), px(18), text=f'{st}', anchor='e', fill='#ffffff', font=font(11))
        c.create_text(px(348), px(82), text='without P control', anchor='e', fill='#ffffff',
                      font=font(9))

        x0, chip_w = px(380), max((w - px(380)) / len(STATES), px(60))
        for i, s in enumerate(STATES):
            x = x0 + i * chip_w
            active = i == st
            c.create_rectangle(x + px(2), px(8), x + chip_w - px(2), px(48),
                               fill=STATE_COLORS[i] if active else '#33363d', outline='')
            fsize = 10 if chip_w >= len(s) * px(9) else 8
            c.create_text(x + chip_w / 2, px(28), text=s,
                          fill='#ffffff' if active else DIM, font=font(fsize, active))
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

    # ---------- 左: 水平 setpoint (位置保持) ----------
    def draw_left(self, v, p, stale):
        c = self.left
        c.delete('all')
        w, h = c.winfo_width(), c.winfo_height()
        st = int(v['state'])
        flying = st in FLYING and not stale
        hover = st == ST_HOVER and not stale

        c.create_text(px(16), px(18), anchor='w', fill=FG, font=font(14, True),
                      text='Horizontal setpoint: hold takeoff x/y (top view)')
        c.create_text(px(16), px(42), anchor='w', fill=POS_COLOR, font=font(10),
                      text='━ solid = takeoff point − estimated position (position control pulls)')
        c.create_text(px(16), px(60), anchor='w', fill=VEL_COLOR, font=font(10),
                      text='┅ dashed = vxCmd/vyCmd (what P control would command, not applied)')
        c.create_text(px(16), px(78), anchor='w', fill=DIM, font=font(10),
                      text=f'Dotted circle = {POS_ERR_FULL:.2f} m / {p["centerMaxV"]:.2f} m/s')

        table_h = px(150)
        side = max(min(w - px(140), h - px(116) - table_h - px(80)), px(120))
        cx, cy = w / 2, px(106) + px(30) + side / 2
        half = side / 2
        plane_ok = int(v['planeOk'])

        edges = {'Front': (cx - half, cy - half, cx + half, cy - half),
                 'Back': (cx - half, cy + half, cx + half, cy + half),
                 'Left': (cx - half, cy - half, cx - half, cy + half),
                 'Right': (cx + half, cy - half, cx + half, cy + half)}
        for key, label_name, sensor, bit in WALLS:
            fit = bool(plane_ok & bit)
            c.create_line(*edges[key], fill=WALL_FIT if fit else WALL_NOFIT,
                          width=px(10), capstyle='round')
            label = f'{label_name} {sensor}  d={fmt_dist(v["d" + key])} m'
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

        r_max = half * 0.82
        c.create_oval(cx - r_max, cy - r_max, cx + r_max, cy + r_max,
                      outline='#3c4049', dash=dsh(4, 4), width=px(1))
        c.create_line(cx, cy - r_max, cx, cy + r_max, fill='#3c4049', width=px(1))
        c.create_line(cx - r_max, cy, cx + r_max, cy, fill='#3c4049', width=px(1))
        draw_drone(c, cx, cy, max(side * 0.04, px(10)), '#6b6f78' if flying else '#4a4e56')

        # 実際: 離陸点 - 推定位置 (ワールド) を推定 yaw で機体座標へ
        err_txt = 'Takeoff point: not recorded (TAKEOFF start not seen)'
        err_f = err_l = 0.0
        if self.takeoff_xy is not None:
            dx = self.takeoff_xy[0] - float(v['estX'])
            dy = self.takeoff_xy[1] - float(v['estY'])
            yaw = math.radians(float(v['yawDeg']))
            err_f = math.cos(yaw) * dx + math.sin(yaw) * dy
            err_l = -math.sin(yaw) * dx + math.cos(yaw) * dy
            err_txt = (f'Pos. error {math.hypot(err_f, err_l) * 100:.1f} cm  '
                       f'{direction_text(err_f, err_l)}  '
                       f'(fwd {err_f * 100:+.1f} / left {err_l * 100:+.1f} cm)')
            if flying:
                draw_body_arrow(c, cx, cy, err_f, err_l, r_max / POS_ERR_FULL,
                                POS_COLOR, 7)

        # 反事実: P 制御なら出していた速度 (HOVER 中のみ更新)
        vx, vy = float(v['vxCmd']), float(v['vyCmd'])
        max_v = p['centerMaxV'] if p['centerMaxV'] > 0 else 0.15
        if hover:
            draw_body_arrow(c, cx, cy, vx, vy, r_max / max_v, VEL_COLOR, 4, dash=(8, 5))

        y0 = cy + half + px(46)
        c.create_text(cx, y0, fill=POS_COLOR if flying else DIM, font=font(13, True),
                      text=err_txt)
        c.create_text(cx, y0 + px(24), fill=VEL_COLOR if hover else DIM, font=font(11),
                      text=f'(not applied) vxCmd={vx:+.3f} vyCmd={vy:+.3f} m/s  '
                           f'|v|={math.hypot(vx, vy):.3f}  {direction_text(vx, vy)}')

        ty = y0 + px(50)
        cols = [cx - px(190), cx - px(90), cx + px(10), cx + px(110)]
        for i, head in enumerate(['Wall', 'd (P input)', 'p plane', 'z zone']):
            c.create_text(cols[i], ty, anchor='w', fill=DIM, font=font(10), text=head)
        for row, (key, label_name, sensor, bit) in enumerate(WALLS):
            yy = ty + px(18) + row * px(18)
            col = WALL_FIT if plane_ok & bit else FG
            c.create_text(cols[0], yy, anchor='w', fill=col, font=font(10),
                          text=f'{label_name} ({sensor})')
            for i, pre in enumerate(['d', 'p', 'z']):
                c.create_text(cols[i + 1], yy, anchor='w', fill=FG, font=font(10, family=MONO),
                              text=fmt_dist(v[pre + key]))

        if stale:
            draw_notice(c, cx, cy + half * 0.55, 'NO DATA\nShowing last received values')
        elif not flying:
            draw_notice(c, cx, cy + half * 0.55,
                        f'Not flying (state={STATES[st] if 0 <= st < 8 else st})\n'
                        'No horizontal setpoint')
        elif not hover:
            draw_notice(c, cx, cy + half * 0.55,
                        'Wall distances, vxCmd, planeOk update only in HOVER\n'
                        '(values are from the last HOVER)')

    # ---------- 右: yaw (向き保持) ----------
    def draw_right(self, v, p, stale):
        c = self.right
        c.delete('all')
        w, h = c.winfo_width(), c.winfo_height()
        st = int(v['state'])
        flying = st in FLYING and not stale
        hover = st == ST_HOVER and not stale
        align_ok = bool(v['yawAlignOk'])

        c.create_text(px(16), px(18), anchor='w', fill=FG, font=font(14, True),
                      text='Yaw setpoint: fixed holdYawDeg (yawRate = 0)')
        c.create_text(px(16), px(42), anchor='w', fill=ROT_COLOR, font=font(10),
                      text='━ solid = holdYawDeg − estimated yaw (attitude control turns)')
        c.create_text(px(16), px(60), anchor='w', fill=REF_COLOR, font=font(10),
                      text='┅ dashed = yaw alignment rate (reference, not applied)')

        side = max(min(w - px(140), h - px(98) - px(230)), px(120))
        cx, cy = w / 2, px(88) + px(30) + side / 2
        half = side / 2
        plane_ok = int(v['planeOk'])

        # 壁を -wallYaw 回転 (hybrid 版と同じ向きの約束)
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

        r_ring = half * 0.55
        c.create_line(cx, cy, cx, cy - r_ring * 1.25, fill=FG, width=px(2))
        draw_drone(c, cx, cy, max(side * 0.06, px(14)), FG if flying else DIM)
        c.create_oval(cx - r_ring, cy - r_ring, cx + r_ring, cy + r_ring,
                      outline='#3c4049', dash=dsh(4, 4), width=px(1))

        # 実際: holdYawDeg への向き誤差 (正 = 反時計回りに戻そうとする)
        yaw_err = wrap180(p['holdYawDeg'] - float(v['yawDeg']))
        if flying:
            ext = max(-1.0, min(1.0, yaw_err / YAW_ERR_FULL)) * 160.0
            if abs(ext) > 0.5:
                c.create_line(*arc_points(cx, cy, r_ring, 90, ext), fill=ROT_COLOR,
                              width=px(9), arrow='last', arrowshape=(px(20), px(24), px(11)),
                              smooth=True)

        # 参考: hybrid 版の yaw 合わせが出していたはずの回転速度
        ref_rate = 0.0
        if align_ok and abs(wall_yaw) >= REF_YAW_DB:
            ref_rate = max(-REF_YAW_RATE_MAX,
                           min(REF_YAW_RATE_MAX, REF_YAW_KP * REF_YAW_SIGN * wall_yaw))
        if hover and abs(ref_rate) > 1e-3:
            ext = ref_rate / REF_YAW_RATE_MAX * 160.0
            c.create_line(*arc_points(cx, cy, r_ring * 1.28, 90, ext), fill=REF_COLOR,
                          width=px(4), dash=dsh(8, 5), arrow='last',
                          arrowshape=(px(12), px(15), px(6)), smooth=True)

        if not flying:
            dir_txt = '—'
        elif abs(yaw_err) < 0.05:
            dir_txt = 'no error'
        else:
            dir_txt = 'turning back CCW (left)' if yaw_err > 0 else 'turning back CW (right)'
        y0 = cy + half + px(46)
        err_val = f'{yaw_err:+.1f}°' if flying else '—'
        c.create_text(cx, y0, fill=ROT_COLOR if flying else DIM, font=font(13, True),
                      text=f'Heading error = {err_val}   {dir_txt}')

        # 向き誤差ゲージ (-full .. +full)
        gw = min(w - px(80), px(520))
        gx0, gy = cx - gw / 2, y0 + px(28)
        c.create_rectangle(gx0, gy, gx0 + gw, gy + px(14), fill='#33363d', outline='')
        frac = max(-1.0, min(1.0, yaw_err / YAW_ERR_FULL)) if flying else 0.0
        c.create_rectangle(cx, gy, cx - frac * gw / 2, gy + px(14),
                           fill=ROT_COLOR if flying else DIM, outline='')
        c.create_line(cx, gy - px(4), cx, gy + px(18), fill=FG, width=px(1))
        c.create_text(gx0, gy + px(26), anchor='w', fill=DIM, font=font(9),
                      text=f'+{YAW_ERR_FULL:.0f}° CCW (left)')
        c.create_text(gx0 + gw, gy + px(26), anchor='e', fill=DIM, font=font(9),
                      text=f'CW (right) -{YAW_ERR_FULL:.0f}°')

        ref_txt = (f'would give {ref_rate:+.1f} deg/s' if align_ok else '— wall angle invalid')
        c.create_text(cx, gy + px(54), fill=FG, font=font(11),
                      text=f'wallYaw = {v["wallYaw"]:+.1f}°   yawAlignOk = {int(v["yawAlignOk"])}   '
                           f'yawCmd = {v["yawCmd"]:+.1f}°   yawRate = {v["yawRate"]:+.1f}')
        c.create_text(cx, gy + px(76), fill=REF_COLOR if hover else DIM, font=font(10),
                      text=f'Reference: hybrid yaw alignment (Kp={REF_YAW_KP}, '
                           f'deadband {REF_YAW_DB:.0f}°) {ref_txt}')

        if stale:
            draw_notice(c, cx, cy + half * 0.6, 'NO DATA\nShowing last received values')
        elif not flying:
            draw_notice(c, cx, cy + half * 0.6,
                        f'Not flying (state={STATES[st] if 0 <= st < 8 else st})\n'
                        'No yaw setpoint')
        elif not hover:
            draw_notice(c, cx, cy + half * 0.6,
                        'wallYaw, planeOk update only in HOVER\n(values are from the last HOVER)')


def main():
    global SCALE
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--uri', default=os.environ.get('CFLIB_URI', DEFAULT_URI))
    ap.add_argument('--period', type=int, default=100, help='log period [ms] (default 100)')
    ap.add_argument('--minimal', action='store_true',
                    help='subscribe only the ctrl/yaw blocks needed for display (less radio load)')
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

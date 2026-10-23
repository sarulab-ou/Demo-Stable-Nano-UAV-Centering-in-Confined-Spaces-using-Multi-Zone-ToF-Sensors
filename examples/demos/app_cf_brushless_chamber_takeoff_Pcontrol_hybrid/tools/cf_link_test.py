#!/usr/bin/env python3
"""
cf_link_test.py -- Crazyflie との無線接続を段階的に試し、どこで・いつ落ちるかを切り分ける。

「接続できない」の原因候補を、下の層から順に確かめる:

  scan    : Crazyradio から機体が見えるか (nRF51 が生きているか)
  probe   : cflib の通信層だけで STM32 に問い合わせ続け、
            - STM32 が応答するか (platform protocol version を 0.5 s ごとに要求)
            - 何秒後に応答しなくなるか (= MCU が止まった/リセットした時刻)
            - コンソール出力 (watchdog / assert の起動時メッセージを含む)
            を記録する。TOC は取得しないので、TOC 取得そのものの負荷は掛からない。
  connect : cflib の通常接続 (log/param TOC の取得) に何秒掛かるか、どこで止まるか
  log     : 指定ログ変数を購読し、受信が途切れないか

使い方 (チャンネル/アドレスは --uri か CFLIB_URI で指定):
  python cf_link_test.py                       # scan → probe(20 s) → connect → log(20 s)
  python cf_link_test.py probe --watch 60      # 60 s 間 STM32 を監視
  python cf_link_test.py connect --cache       # TOC キャッシュを使って接続
  python cf_link_test.py log --vars chamber.state chamber.planeOk --seconds 30

起動時のコンソール (watchdog / assert の報告) を確実に拾うには:
  1. 機体の電源を切った状態で `python cf_link_test.py probe --watch 60` を起動
  2. 「機体を待っています」と出たら電源を入れる
  起動後に見えるメッセージの例:
    "The system resumed after watchdog timeout [WARNING]"  -> 前回は watchdog リセット
    "The system resumed after a failed assert [WARNING]"
    "Assert failed at <file>:<line>"                        -> 前回は ASSERT で停止

出力は画面と同時に cf_link_test_YYYYmmdd_HHMMSS.txt に保存する。
"""
import argparse
import datetime
import os
import sys
import threading
import time

DEFAULT_URI = 'radio://0/80/2M/E7E7E7E7E7'

PORT_CONSOLE = 0x0
PORT_PARAM = 0x2
PORT_LOG = 0x5
PORT_PLATFORM = 0xD
PORT_LINK = 0xF

KEYWORDS = ('watchdog', 'assert', 'fail', 'warning', 'error', 'hardfault', 'stack')


# ========================= 出力 =========================
class Out:
    def __init__(self, path):
        self.t0 = time.monotonic()
        self.f = open(path, 'w', encoding='utf-8')
        self.lock = threading.Lock()
        self.path = path

    def __call__(self, msg, mark=''):
        line = f'[{time.monotonic() - self.t0:7.2f}s] {mark}{msg}'
        with self.lock:
            print(line, flush=True)
            self.f.write(line + '\n')
            self.f.flush()

    def section(self, title):
        self(f'===== {title} =====')

    def close(self):
        self.f.close()


class ConsoleBuffer:
    """コンソールは 1 パケット最大 30 文字で分割されて届くので、行単位にまとめる。"""

    def __init__(self, out):
        self.out = out
        self.buf = ''
        self.lines = []

    def feed(self, text):
        self.buf += text
        while '\n' in self.buf:
            line, self.buf = self.buf.split('\n', 1)
            self.lines.append(line)
            hit = any(k in line.lower() for k in KEYWORDS)
            self.out(f'CONSOLE: {line}', mark='!! ' if hit else '')

    def flush(self):
        if self.buf:
            self.feed('\n')


# ========================= 1. scan =========================
def step_scan(out, uri):
    import cflib.crtp
    out.section('1. scan: Crazyradio から機体が見えるか')
    addr = int(uri.rsplit('/', 1)[1], 16) if uri.count('/') >= 5 else None
    found = cflib.crtp.scan_interfaces(addr) if addr is not None else cflib.crtp.scan_interfaces()
    out(f'見つかったインターフェース: {found}')
    base = uri.rsplit('/', 1)[0] if addr is not None else uri
    if any(f[0].startswith(base) for f in found):
        out(f'OK: {base} で機体が応答 (nRF51 は生きている)')
        return True
    out(f'NG: {base} で機体が見つからない。電源・チャンネル・アドレスを確認', mark='!! ')
    return False


def nrf_visible(uri):
    """スキャンで機体 (nRF51) が見えるか。STM32 の状態には依らない。"""
    import cflib.crtp
    if uri.count('/') < 5:
        return bool(cflib.crtp.scan_interfaces())
    base, addr = uri.rsplit('/', 1)
    return any(f[0].startswith(base) for f in cflib.crtp.scan_interfaces(int(addr, 16)))


# ========================= 2. probe =========================
def step_probe(out, uri, watch_s, wait_s):
    """cflib の通信層 (link driver) だけで STM32 に問い合わせ続ける。"""
    import cflib.crtp
    from cflib.crtp.crtpstack import CRTPPacket

    out.section(f'2. probe: STM32 の応答を {watch_s:.0f} s 監視 (TOC は取得しない)')
    console = ConsoleBuffer(out)
    state = {'link_err': None, 'quality': None}

    def on_err(msg):
        state['link_err'] = msg

    def on_stats(stats):
        state['quality'] = stats

    def open_link():
        state['link_err'] = None
        return cflib.crtp.get_link_driver(uri, on_stats, on_err)

    # 機体 (nRF51) がスキャンで見えるまで待つ (電源投入前に起動してもよい)
    out(f'機体を待っています... (最大 {wait_s:.0f} s, この間に電源を入れてもよい)')
    t_wait = time.monotonic()
    while not nrf_visible(uri):
        if time.monotonic() - t_wait >= wait_s:
            out('NG: 機体がスキャンで見えない。電源が入っているか・バッテリー残量・'
                'チャンネル/アドレスを確認', mark='!! ')
            return False
        time.sleep(0.3)
    out(f'機体を検出 ({time.monotonic() - t_wait:.1f} s)。リンクを開きます')
    link = open_link()

    t_start = time.monotonic()
    last_req = 0.0
    last_reply = None
    first_reply = None
    n_req = n_rep = 0
    stopped_reported = False
    link_errors = 0
    last_status = t_start

    while time.monotonic() - t_start < watch_s:
        now = time.monotonic()

        if state['link_err'] is not None:
            link_errors += 1
            try:
                link.close()
            except Exception:
                pass
            # nRF51 ごと消えたのか (電源断/ブラウンアウト/nRF リセット)、
            # 無線は生きていて一時的に ack が抜けただけなのかを区別する
            if nrf_visible(uri):
                out(f'リンクエラー: {state["link_err"]} -> 機体はスキャンで見える '
                    '(無線は生きている。開き直す)', mark='!! ')
            else:
                out(f'リンクエラー: {state["link_err"]} -> 機体がスキャンで見えない = '
                    'nRF51 も応答していない (電源断・ブラウンアウト・バッテリー切れ・'
                    '機体全体のリセット)。STM32 の watchdog だけなら nRF51 は残るので別の原因',
                    mark='!! ')
                t_gone = time.monotonic()
                while not nrf_visible(uri) and time.monotonic() - t_start < watch_s:
                    time.sleep(0.3)
                if not nrf_visible(uri):
                    break
                out(f'機体が再び見えた ({time.monotonic() - t_gone:.1f} s 後)。'
                    '再起動した可能性 -> 直後のコンソールを確認', mark='!! ')
            link = open_link()
            continue

        if now - last_req >= 0.5:
            pk = CRTPPacket()
            pk.set_header(PORT_PLATFORM, 1)
            pk.data = bytes([0])              # protocol version 要求
            link.send_packet(pk)
            n_req += 1
            last_req = now

        pk = link.receive_packet(0.05)
        if pk is not None:
            if pk.port == PORT_CONSOLE:
                console.feed(bytes(pk.data).decode('ascii', errors='replace'))
            elif pk.port == PORT_PLATFORM and pk.channel == 1 and len(pk.data) >= 2 \
                    and pk.data[0] == 0:
                n_rep += 1
                if first_reply is None:
                    first_reply = now
                    out(f'OK: STM32 応答あり (protocol version {pk.data[1]})')
                if stopped_reported:
                    out(f'STM32 の応答が再開 (停止 {now - last_reply:.1f} s 後)。'
                        'リセットして再起動した可能性 -> 直後のコンソールを確認', mark='!! ')
                    stopped_reported = False
                last_reply = now

        if last_reply is not None and not stopped_reported and now - last_reply > 2.0:
            out(f'STM32 が応答しなくなった (最後の応答から {now - last_reply:.1f} s, '
                f'監視開始から {last_reply - t_start:.1f} s 時点で停止)', mark='!! ')
            stopped_reported = True

        if now - last_status >= 5.0:
            q = state['quality']
            out(f'状態: 要求 {n_req} / 応答 {n_rep}   リンク統計 {q}')
            last_status = now

    console.flush()
    link.close()
    out(f'probe 結果: 要求 {n_req} / 応答 {n_rep}, リンクエラー {link_errors} 回')
    if first_reply is None:
        out('NG: STM32 が一度も応答しなかった (起動途中で止まっている/クラッシュしている可能性)',
            mark='!! ')
        return False
    if stopped_reported:
        return False
    out('OK: 監視期間中 STM32 は応答し続けた')
    return True


# ========================= 3. connect =========================
def step_connect(out, uri, use_cache, timeout_s, keep=False):
    """cflib の通常接続。log/param TOC の取得時間を計る。"""
    from cflib.crazyflie import Crazyflie

    out.section(f'3. connect: 通常接続 (TOC 取得, キャッシュ {"あり" if use_cache else "なし"})')
    cache = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'cache') if use_cache else None
    cf = Crazyflie(rw_cache=cache)
    console = ConsoleBuffer(out)
    ev_conn = threading.Event()
    ev_full = threading.Event()
    ev_fail = threading.Event()

    cf.console.receivedChar.add_callback(console.feed)
    cf.connection_requested.add_callback(lambda u: out('接続要求'))
    cf.link_established.add_callback(lambda u: out('無線リンク確立'))
    cf.connected.add_callback(lambda u: (out('log/param TOC 取得完了 (connected)'), ev_conn.set()))
    cf.fully_connected.add_callback(lambda u: (out('param 値の取得完了 (fully connected)'),
                                                ev_full.set()))
    cf.connection_failed.add_callback(lambda u, m: (out(f'接続失敗: {m}', mark='!! '),
                                                    ev_fail.set()))
    cf.connection_lost.add_callback(lambda u, m: (out(f'接続断: {m}', mark='!! '), ev_fail.set()))

    t0 = time.monotonic()
    cf.open_link(uri)
    while time.monotonic() - t0 < timeout_s and not ev_full.is_set() and not ev_fail.is_set():
        time.sleep(0.1)
    console.flush()

    if not ev_conn.is_set():
        out(f'NG: {time.monotonic() - t0:.1f} s 以内に TOC 取得が終わらなかった', mark='!! ')
        cf.close_link()
        return None if keep else False

    groups = cf.log.toc.toc
    n_log = sum(len(v) for v in groups.values())
    n_par = sum(len(v) for v in cf.param.toc.toc.values())
    out(f'log TOC: {len(groups)} グループ / {n_log} 変数,  param TOC: {n_par} 変数')
    if 'chamber' in groups:
        variant = 'hybrid' if 'hovWallSrc' in cf.param.toc.toc.get('chamber', {}) else 'without_Pcontrol'
        out(f'chamber ロググループあり ({len(groups["chamber"])} 変数) -> {variant} のファームウェア')
    else:
        out('chamber ロググループが無い -> デモのファームウェアではない', mark='!! ')
    if ev_full.is_set():
        try:
            out(f'system.selftestPassed = {cf.param.get_value("system.selftestPassed")}')
        except Exception as e:
            out(f'system.selftestPassed 取得失敗: {e}')

    if keep:
        return cf
    cf.close_link()
    return True


# ========================= 4. log =========================
def step_log(out, uri, use_cache, variables, period_ms, seconds, timeout_s):
    from cflib.crazyflie.log import LogConfig

    cf = step_connect(out, uri, use_cache, timeout_s, keep=True)
    if not cf:
        return False
    out.section(f'4. log: {variables} を {period_ms} ms 周期で {seconds:.0f} s 購読')
    lc = LogConfig(name='linktest', period_in_ms=period_ms)
    for v in variables:
        lc.add_variable(v)
    count = [0]
    last = [None]

    def on_data(ts, data, conf):
        count[0] += 1
        last[0] = data

    lc.data_received_cb.add_callback(on_data)
    lc.error_cb.add_callback(lambda conf, msg: out(f'ログエラー: {msg}', mark='!! '))
    try:
        cf.log.add_config(lc)
    except (KeyError, AttributeError) as e:
        out(f'NG: LogConfig を作れない: {e}', mark='!! ')
        cf.close_link()
        return False
    lc.start()

    t0 = time.monotonic()
    prev = 0
    gap = 0
    expected = 1000.0 / period_ms
    while time.monotonic() - t0 < seconds and cf.is_connected():
        time.sleep(1.0)
        n = count[0] - prev
        prev = count[0]
        gap = gap + 1 if n == 0 else 0
        out(f'受信 {n:3d} pkt/s (期待 {expected:.0f})  最新 {last[0]}',
            mark='!! ' if n == 0 else '')
        if gap >= 3:
            out('NG: 3 s 以上ログが届かない', mark='!! ')
            break
    ok = cf.is_connected() and gap < 3
    try:
        lc.stop()
    except Exception:
        pass
    cf.close_link()
    out('OK: ログ受信は途切れなかった' if ok else 'NG: ログ受信が途切れた', mark='' if ok else '!! ')
    return ok


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('step', nargs='?', default='all',
                    choices=['all', 'scan', 'probe', 'connect', 'log'])
    ap.add_argument('--uri', default=os.environ.get('CFLIB_URI', DEFAULT_URI))
    ap.add_argument('--watch', type=float, default=20.0, help='probe の監視時間 [s]')
    ap.add_argument('--wait', type=float, default=30.0, help='probe で機体を待つ時間 [s]')
    ap.add_argument('--cache', action='store_true', help='TOC キャッシュを使う')
    ap.add_argument('--timeout', type=float, default=40.0, help='connect のタイムアウト [s]')
    ap.add_argument('--vars', nargs='+', default=['chamber.state'], help='log で購読する変数')
    ap.add_argument('--period', type=int, default=100, help='log の周期 [ms]')
    ap.add_argument('--seconds', type=float, default=20.0, help='log の購読時間 [s]')
    args = ap.parse_args()

    import cflib.crtp
    cflib.crtp.init_drivers()

    stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    out = Out(os.path.join(os.getcwd(), f'cf_link_test_{stamp}.txt'))
    out(f'URI = {args.uri}')

    results = []
    try:
        if args.step in ('all', 'scan'):
            results.append(('scan', step_scan(out, args.uri)))
        if args.step in ('all', 'probe') and all(r for _, r in results):
            results.append(('probe', step_probe(out, args.uri, args.watch, args.wait)))
        if args.step == 'connect' or (args.step == 'all' and all(r for _, r in results)):
            results.append(('connect', step_connect(out, args.uri, args.cache, args.timeout)))
        if args.step == 'log' or (args.step == 'all' and all(r for _, r in results)):
            results.append(('log', step_log(out, args.uri, args.cache, args.vars,
                                            args.period, args.seconds, args.timeout)))
    except KeyboardInterrupt:
        out('中断')
    finally:
        out.section('まとめ')
        for name, ok in results:
            out(f'{name:8s}: {"OK" if ok else "NG"}')
        out(f'ログファイル: {out.path}')
        out.close()
    sys.exit(0 if results and all(r for _, r in results) else 1)


if __name__ == '__main__':
    main()

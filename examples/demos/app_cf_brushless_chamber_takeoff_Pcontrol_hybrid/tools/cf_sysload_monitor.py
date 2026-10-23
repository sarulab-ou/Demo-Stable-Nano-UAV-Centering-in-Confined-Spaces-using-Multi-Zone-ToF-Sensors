#!/usr/bin/env python3
"""
cf_sysload_monitor.py -- ファームウェアの sysload ロググループを受信し、CPU 使用率を表示・記録する。

ファームウェア側 (src/modules/src/sysload.c) は 500 ms ごとに次を計算して公開する:
  cpu        : CPU 使用率 = 1000 - idle  [permille]
  idleGap    : 直近 500 ms で idle タスクが走らなかった最長時間 [ms]
  idleGapPk  : 起動からの idleGap の最大値 [ms]
               watchdog は idle フックでしか更新されないので、これが数百 ms に届くと MCU がリセットする
  rangingUs  : 直近の Gget_Ranging() (全 vl53l8cx の SPI 読み出し) の所要時間 [us]
  l<Task>    : タスクごとの CPU 使用率 [permille]
               Tof = vl53l8cx_get_tof_task, Deck = vl53l8cx (オドメトリ/壁フィット), Other = その他の合計
  s<Task>    : タスクのスタック残量 (ピーク時) [word = 4 byte],  stkMin = 全タスク中の最小値

使い方:
  python cf_sysload_monitor.py                  # 60 s 表示、CSV 保存
  python cf_sysload_monitor.py --seconds 0      # Ctrl-C まで
  python cf_sysload_monitor.py --dump           # 開始時に全タスクの負荷/スタックをコンソールへ出させる
                                                # (stkMin がどのタスクかはこの表で分かる)

途中で接続が切れた場合は、切れる直前の値をまとめて表示する (MCU が落ちる直前の負荷が分かる)。
"""
import argparse
import csv
import datetime
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from cf_link_test import DEFAULT_URI, Out, step_connect  # noqa: E402

# 1 ブロック 26 byte 以内
BLOCK_A = [('cpu', 'uint16_t'), ('idleGap', 'uint16_t'), ('idleGapPk', 'uint16_t'),
           ('rangingUs', 'uint32_t'), ('nTasks', 'uint8_t'), ('stkMin', 'uint16_t'),
           ('lTof', 'uint16_t'), ('lDeck', 'uint16_t'), ('lSyslink', 'uint16_t'),
           ('lCrtpRx', 'uint16_t'), ('lCrtpTx', 'uint16_t'), ('sTmr', 'uint16_t')]
BLOCK_B = [('lStab', 'uint16_t'), ('lKalman', 'uint16_t'), ('lSensors', 'uint16_t'),
           ('lApp', 'uint16_t'), ('lLog', 'uint16_t'), ('lTmr', 'uint16_t'), ('lFlow', 'uint16_t'),
           ('lOther', 'uint16_t'), ('sTof', 'uint16_t'), ('sDeck', 'uint16_t'),
           ('sSyslink', 'uint16_t'), ('sCrtpRx', 'uint16_t'), ('sApp', 'uint16_t')]
ALL_VARS = [n for n, _ in BLOCK_A + BLOCK_B]
LOAD_VARS = ['lTof', 'lDeck', 'lSyslink', 'lCrtpRx', 'lCrtpTx', 'lStab', 'lKalman',
             'lSensors', 'lApp', 'lLog', 'lTmr', 'lFlow', 'lOther']
STACK_VARS = ['sTof', 'sDeck', 'sSyslink', 'sCrtpRx', 'sApp', 'sTmr', 'stkMin']

IDLE_GAP_WARN_MS = 150   # watchdog (約 350 ms) に対して余裕が無くなってきた目安
STACK_WARN_WORDS = 50


def pct(v):
    return f'{v / 10:5.1f}'


def format_row(d):
    loads = ' '.join(f'{k[1:]}={d[k] / 10:.1f}' for k in LOAD_VARS if d.get(k))
    return (f'CPU {pct(d["cpu"])}%  idleGap {d["idleGap"]:4d} ms (peak {d["idleGapPk"]:4d})  '
            f'ranging {d["rangingUs"] / 1000:6.1f} ms  | {loads}')


def format_stacks(d):
    return 'stack free [words]: ' + '  '.join(f'{k}={d[k]}' for k in STACK_VARS)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--uri', default=os.environ.get('CFLIB_URI', DEFAULT_URI))
    ap.add_argument('--seconds', type=float, default=60.0, help='監視時間 [s] (0 = Ctrl-C まで)')
    ap.add_argument('--period', type=int, default=500, help='ログ周期 [ms] (FW の更新は 500 ms)')
    ap.add_argument('--cache', action='store_true', help='TOC キャッシュを使う')
    ap.add_argument('--timeout', type=float, default=40.0, help='接続タイムアウト [s]')
    ap.add_argument('--dump', action='store_true', help='system.taskDump=1 で全タスクをコンソール出力')
    args = ap.parse_args()

    import cflib.crtp
    from cflib.crazyflie.log import LogConfig
    cflib.crtp.init_drivers()

    stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    out = Out(os.path.join(os.getcwd(), f'cf_sysload_{stamp}.txt'))
    csv_path = os.path.join(os.getcwd(), f'cf_sysload_{stamp}.csv')
    out(f'URI = {args.uri}')

    cf = step_connect(out, args.uri, args.cache, args.timeout, keep=True)
    if not cf:
        out.close()
        sys.exit(1)
    if 'sysload' not in cf.log.toc.toc:
        out('NG: sysload ロググループが無い -> CPU 負荷計測入りのファームウェアを書き込んでください',
            mark='!! ')
        cf.close_link()
        out.close()
        sys.exit(1)

    latest = {}
    rows = []
    lock = threading.Lock()
    ev_lost = threading.Event()
    cf.connection_lost.add_callback(lambda u, m: ev_lost.set())

    csv_f = open(csv_path, 'w', newline='')
    writer = csv.writer(csv_f)
    writer.writerow(['t_s'] + ALL_VARS)
    t0 = time.monotonic()

    def on_a(ts, data, conf):
        with lock:
            latest.update({k.split('.')[1]: v for k, v in data.items()})
            if len(latest) == len(ALL_VARS):
                row = dict(latest)
                rows.append(row)
                writer.writerow([f'{time.monotonic() - t0:.2f}'] + [row[k] for k in ALL_VARS])
                csv_f.flush()
                out(format_row(row), mark='!! ' if row['idleGap'] >= IDLE_GAP_WARN_MS else '')

    def on_b(ts, data, conf):
        with lock:
            latest.update({k.split('.')[1]: v for k, v in data.items()})

    configs = []
    for name, block, cb in (('sysloadA', BLOCK_A, on_a), ('sysloadB', BLOCK_B, on_b)):
        lc = LogConfig(name=name, period_in_ms=args.period)
        for var, typ in block:
            lc.add_variable(f'sysload.{var}', typ)
        lc.data_received_cb.add_callback(cb)
        lc.error_cb.add_callback(lambda conf, msg: out(f'ログエラー {conf.name}: {msg}', mark='!! '))
        cf.log.add_config(lc)
        configs.append(lc)
    # B を先に始めて、A 受信時にはスタック値が揃っているようにする
    for lc in reversed(configs):
        lc.start()

    out.section(f'sysload 監視 ({args.period} ms 周期, Ctrl-C で終了)')
    if args.dump:
        cf.param.set_value('system.taskDump', 1)
        out('system.taskDump=1 を送信 (全タスクの表がコンソールに出ます)')

    last_stack_print = 0.0
    try:
        while (args.seconds <= 0 or time.monotonic() - t0 < args.seconds) and not ev_lost.is_set():
            time.sleep(0.2)
            if time.monotonic() - last_stack_print >= 5.0 and rows:
                last_stack_print = time.monotonic()
                with lock:
                    out(format_stacks(rows[-1]),
                        mark='!! ' if rows[-1]['stkMin'] < STACK_WARN_WORDS else '')
    except KeyboardInterrupt:
        out('中断')

    out.section('まとめ')
    with lock:
        if ev_lost.is_set():
            out(f'接続が切れた ({time.monotonic() - t0:.1f} s)。切れる直前の値:', mark='!! ')
            for r in rows[-4:]:
                out('  ' + format_row(r))
            if rows:
                out('  ' + format_stacks(rows[-1]))
        if rows:
            n = len(rows)
            out(f'サンプル数 {n}')
            out(f'CPU 使用率   平均 {sum(r["cpu"] for r in rows) / n / 10:.1f}%  '
                f'最大 {max(r["cpu"] for r in rows) / 10:.1f}%')
            out(f'idle 停止    最大 {max(r["idleGap"] for r in rows)} ms  '
                f'(起動から最大 {rows[-1]["idleGapPk"]} ms)')
            out(f'Gget_Ranging 平均 {sum(r["rangingUs"] for r in rows) / n / 1000:.1f} ms  '
                f'最大 {max(r["rangingUs"] for r in rows) / 1000:.1f} ms')
            avg = {k: sum(r[k] for r in rows) / n / 10 for k in LOAD_VARS}
            out('タスク別平均 [%]: ' + '  '.join(
                f'{k[1:]}={v:.1f}' for k, v in sorted(avg.items(), key=lambda kv: -kv[1])))
            out('スタック残量の最小 [words]: ' + '  '.join(
                f'{k}={min(r[k] for r in rows)}' for k in STACK_VARS))
        else:
            out('sysload を 1 回も受信できなかった', mark='!! ')
    out(f'CSV: {csv_path}')
    out(f'ログファイル: {out.path}')

    if not ev_lost.is_set():
        for lc in configs:
            try:
                lc.stop()
            except Exception:
                pass
    cf.close_link()
    csv_f.close()
    out.close()


if __name__ == '__main__':
    main()

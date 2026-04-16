"""
test_dummy_server.py — Level 1 & 2 test server for CSIRS_128T128R_v4_0_SOCKET.m

Level 1: validates socket handshake + TTI_START/LAYER_ACT protocol
Level 2: inspects observation values (saved to obs_dump/)

Usage:
    python test_dummy_server.py [--port 6666] [--dump-ttis 5] [--random-action]

Protocol (nrDRLScheduler TrainingMode):
    MATLAB → Python : TTI_START  {type, tti, n_layers, n_rbg,
                                   buf, avg_tp, ue_rank, wb_cqi,
                                   curr_mcs, sub_cqi, max_cross_corr,
                                   ue_i1, eligible_ues, occupied_rbgs,
                                   max_users_tti, csi_on, ...}
    Python → MATLAB : LAYER_ACT  {type, actions: [[n_layers x n_rbg]]}
    MATLAB → Python : TTI_DONE   {type, metrics}
    MATLAB → Python : STOP
"""

import socket
import json
import os
import sys
import random
import argparse
from datetime import datetime

# ── CLI args ──────────────────────────────────────────────────────────────────
parser = argparse.ArgumentParser()
parser.add_argument('--port',         type=int,  default=6666,  help='TCP port (default 6666)')
parser.add_argument('--dump-ttis',    type=int,  default=5,     help='Save first N TTI_START payloads to obs_dump/ (default 5)')
parser.add_argument('--random-action',action='store_true',      help='Send random valid actions instead of all-NOOP')
args = parser.parse_args()

DUMP_DIR  = os.path.join(os.path.dirname(__file__), 'obs_dump')
NOOP_RNTI = 9999    # any value >= MaxUEs → treated as NOOP by MATLAB

os.makedirs(DUMP_DIR, exist_ok=True)

# ── Helpers ───────────────────────────────────────────────────────────────────

def recv_lines(conn):
    """Generator: yields one complete JSON line at a time from TCP stream."""
    buf = b''
    while True:
        chunk = conn.recv(131072)
        if not chunk:
            return
        buf += chunk
        while b'\n' in buf:
            line, buf = buf.split(b'\n', 1)
            line = line.strip()
            if line:
                yield line.decode('utf-8', errors='replace')


def make_noop_action(n_layers, n_rbg):
    return [[NOOP_RNTI] * n_rbg for _ in range(n_layers)]


def make_random_action(n_layers, n_rbg, eligible_ues):
    """Random valid action: each cell = 0-based RNTI from eligible list or NOOP."""
    choices = list(eligible_ues) + [NOOP_RNTI]
    return [[random.choice(choices) for _ in range(n_rbg)]
            for _ in range(n_layers)]


def summarize_obs(msg):
    """Print a compact per-TTI observation summary to terminal."""
    tti       = msg.get('tti', '?')
    csi_on    = msg.get('csi_on', False)
    elig      = msg.get('eligible_ues', [])
    n_rbg     = msg.get('n_rbg', '?')
    n_layers  = msg.get('n_layers', '?')
    occ       = msg.get('occupied_rbgs', [])
    buf       = msg.get('buf', [])
    avg_tp    = msg.get('avg_tp', [])
    wb_cqi    = msg.get('wb_cqi', [])
    ue_rank   = msg.get('ue_rank', [])
    curr_mcs  = msg.get('curr_mcs', [])
    ue_i1     = msg.get('ue_i1', [])
    sub_cqi   = msg.get('sub_cqi', [])       # [MaxUEs][numRBGs]
    cross_corr = msg.get('max_cross_corr', [])  # [MaxUEs][MaxUEs][numRBGs]

    print(f"\n{'─'*72}")
    print(f"  TTI {tti:>4d} | csi_on={csi_on} | eligible={elig} | "
          f"n_layers={n_layers} | n_rbg={n_rbg}")
    print(f"  occupied_rbgs : {occ}")

    # Only print per-UE details for eligible UEs
    if elig:
        print(f"  {'UE':>4}  {'buf':>8}  {'avg_tp':>8}  {'wb_cqi':>7}  "
              f"{'rank':>5}  {'mcs':>5}  {'i1':>18}  sub_cqi(first 3 RBGs)")
        print(f"  {'─'*4}  {'─'*8}  {'─'*8}  {'─'*7}  {'─'*5}  {'─'*5}  "
              f"{'─'*18}  {'─'*24}")
        for rnti0 in elig:
            idx = rnti0   # 0-based RNTI = index into arrays
            b    = buf[idx]     if idx < len(buf)      else '?'
            tp   = avg_tp[idx]  if idx < len(avg_tp)   else '?'
            cqi  = wb_cqi[idx]  if idx < len(wb_cqi)   else '?'
            rk   = ue_rank[idx] if idx < len(ue_rank)  else '?'
            mcs  = curr_mcs[idx]if idx < len(curr_mcs) else '?'
            i1   = ue_i1[idx]   if idx < len(ue_i1)    else '?'
            sb   = sub_cqi[idx] if idx < len(sub_cqi)  else []
            sb3  = sb[:3] if isinstance(sb, list) else '?'

            b_str  = f"{b:.0f}" if isinstance(b, (int,float)) else str(b)
            tp_str = f"{tp:.3f}"if isinstance(tp,(int,float)) else str(tp)
            print(f"  {rnti0+1:>4}  {b_str:>8}  {tp_str:>8}  {cqi:>7}  "
                  f"{rk:>5}  {mcs:>5}  {str(i1):>18}  {sb3}")

    # Cross-corr: print max off-diagonal value as a sanity check
    if cross_corr and isinstance(cross_corr, list):
        try:
            max_corr = 0.0
            for i, row in enumerate(cross_corr):
                for j, val_per_rbg in enumerate(row):
                    if i == j:
                        continue
                    if isinstance(val_per_rbg, list):
                        max_corr = max(max_corr, max(val_per_rbg))
                    elif isinstance(val_per_rbg, (int, float)):
                        max_corr = max(max_corr, val_per_rbg)
            print(f"  max_cross_corr (off-diagonal) : {max_corr:.4f}")
        except Exception:
            pass

    # Subband CQI variation check: is sub_cqi flat (wideband) or varying?
    if sub_cqi and elig:
        idx0 = elig[0]
        sb0 = sub_cqi[idx0] if idx0 < len(sub_cqi) else []
        if isinstance(sb0, list) and len(sb0) > 1:
            flat = all(v == sb0[0] for v in sb0)
            print(f"  sub_cqi UE{elig[0]+1}: {sb0}  "
                  f"→ {'FLAT (wideband mode?)' if flat else 'VARYING ✓ (subband OK)'}")


def check_obs_correctness(msg, tti_count):
    """Print warnings if any observation looks suspicious."""
    warnings = []
    elig   = msg.get('eligible_ues', [])
    buf    = msg.get('buf', [])
    avg_tp = msg.get('avg_tp', [])
    sub_cqi = msg.get('sub_cqi', [])
    ue_i1   = msg.get('ue_i1', [])
    csi_on  = msg.get('csi_on', False)

    # buf: at least some UEs should have non-zero buffer (FTP3 traffic)
    if elig and all(buf[e] == 0 for e in elig if e < len(buf)):
        warnings.append('buf: ALL eligible UEs have zero buffer — FTP3 traffic not generating?')

    # avg_tp: after a few TTIs, should start updating (EMA)
    if tti_count > 10 and elig:
        if all(avg_tp[e] == 0.0 for e in elig if e < len(avg_tp)):
            warnings.append('avg_tp: still all 0 after 10+ TTIs — ActualTputEMA not updating?')

    # sub_cqi: after CSI arrives, should be non-zero for eligible UEs
    if csi_on and elig:
        idx0 = elig[0]
        if idx0 < len(sub_cqi):
            sb = sub_cqi[idx0]
            if isinstance(sb, list) and all(v == 0 for v in sb):
                warnings.append(f'sub_cqi UE{idx0+1}: all zeros even though csi_on=True')

    # ue_i1: eligible UEs should have i1 != [-1,-1,-1] after CSI
    if csi_on and elig:
        for e in elig[:3]:
            if e < len(ue_i1):
                i1 = ue_i1[e]
                if isinstance(i1, list) and all(v == -1 for v in i1):
                    warnings.append(f'ue_i1 UE{e+1}: still [-1,-1,-1] after csi_on=True')

    if warnings:
        print(f"  ⚠  WARNINGS:")
        for w in warnings:
            print(f"     • {w}")
    else:
        print(f"  ✓  No obvious observation issues detected")


# ── Main server loop ──────────────────────────────────────────────────────────

def run_server():
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(('127.0.0.1', args.port))
    server.listen(1)

    print(f"[test_dummy_server] Listening on 127.0.0.1:{args.port}")
    print(f"  Mode   : {'random action' if args.random_action else 'all-NOOP'}")
    print(f"  Dump   : first {args.dump_ttis} TTI_START → {DUMP_DIR}/")
    print(f"  Start  : {datetime.now().strftime('%H:%M:%S')}")
    print(f"  Waiting for MATLAB to connect...")

    conn, addr = server.accept()
    print(f"\n[test_dummy_server] MATLAB connected from {addr}\n")

    tti_count   = 0
    done_count  = 0
    start_time  = datetime.now()

    try:
        for line in recv_lines(conn):
            try:
                msg = json.loads(line)
            except json.JSONDecodeError as e:
                print(f"  [WARN] JSON parse error: {e} — line[:120]: {line[:120]}")
                continue

            t = msg.get('type', '')

            # ── TTI_START ─────────────────────────────────────────────────────
            if t == 'TTI_START':
                tti      = msg.get('tti', tti_count)
                n_layers = msg.get('n_layers', 1)
                n_rbg    = msg.get('n_rbg', 1)
                elig     = msg.get('eligible_ues', [])

                summarize_obs(msg)
                check_obs_correctness(msg, tti_count)

                # Dump first N TTIs to file for Level 2 inspection
                if tti_count < args.dump_ttis:
                    dump_path = os.path.join(DUMP_DIR, f'obs_tti_{tti:05d}.json')
                    with open(dump_path, 'w') as f:
                        json.dump(msg, f, indent=2)
                    print(f"  → Saved {dump_path}")

                # Build action
                if args.random_action and elig:
                    actions = make_random_action(n_layers, n_rbg, elig)
                else:
                    actions = make_noop_action(n_layers, n_rbg)

                resp = json.dumps({'type': 'LAYER_ACT', 'actions': actions})
                conn.sendall((resp + '\n').encode())
                tti_count += 1

            # ── TTI_DONE ──────────────────────────────────────────────────────
            elif t == 'TTI_DONE':
                metrics = msg.get('metrics', {})
                tput    = metrics.get('total_cell_tput', '?')
                jain    = metrics.get('jain_throughput', '?')
                nsr     = metrics.get('no_schedule_rate', '?')
                if isinstance(tput, (int, float)):
                    print(f"  TTI_DONE: cell_tput={tput:.3f} Mbps | "
                          f"jain={jain:.3f} | noop_rate={nsr:.2f}")
                done_count += 1

            # ── STOP ──────────────────────────────────────────────────────────
            elif t == 'STOP':
                elapsed = (datetime.now() - start_time).total_seconds()
                print(f"\n{'═'*72}")
                print(f"  STOP received — simulation complete")
                print(f"  TTI_START processed : {tti_count}")
                print(f"  TTI_DONE  received  : {done_count}")
                print(f"  Elapsed             : {elapsed:.1f} s")
                print(f"  Obs dumps saved to  : {DUMP_DIR}/")
                print(f"{'═'*72}\n")
                break

            else:
                print(f"  [WARN] Unknown message type: {t}")

    except KeyboardInterrupt:
        print("\n[test_dummy_server] Interrupted by user")
    except Exception as e:
        print(f"[test_dummy_server] ERROR: {e}")
        import traceback; traceback.print_exc()
    finally:
        conn.close()
        server.close()
        print("[test_dummy_server] Server closed.")


if __name__ == '__main__':
    run_server()

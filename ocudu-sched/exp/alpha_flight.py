#!/usr/bin/env python3
"""
alpha_flight.py -- alpha(t) dinamico no gNB OCUDU a partir do corredor B do RC14 (cenario Interlagos).

Celula sem radio (test mode, 8 UEs emulados, K=1 grant UL por slot):
  - corredor A (solo vazio): RNTIs 0x44..0x47, alpha fixo = ALPHA_FLOOR;
  - corredor B: RNTIs 0x48..0x4b voam (9750,-350) -> (14000,-700) em fila, separados por --headway metros,
    sobre a multidao do evento (24 hotspots, amp 0.75, sigma 180 m; mesma geometria do e3_driver.py do OAI).

A cada segundo:  alpha_u(t) = ALPHA_FLOOR + ALPHA_GAIN * rho_norm(x_u(t))  -> arquivo lido pela politica risk_sched.
Antes de decolar e depois de pousar o drone fica em ALPHA_FLOOR.

O mesmo processo sobe o gNB, carimba as metricas por RNTI do stdout (1 relatorio/s) e grava:
  logs/flight/<tag>/driver.csv   t,rnti,dist,rho_norm,alpha
  logs/flight/<tag>/metrics.csv  t,rnti,ul_brate_bps,ul_grants

Uso: exp/alpha_flight.py --tag risk --policy risk [--speed 45] [--headway 700]
"""
import argparse
import os
import subprocess
import threading
import time

import numpy as np

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RNTI_A = ["44", "45", "46", "47"]
RNTI_B = ["48", "49", "4a", "4b"]
ALPHA_FLOOR, ALPHA_GAIN = 1.0, 7.0

START = np.array([9750.0, -350.0])
EVT = np.array([14000.0, -700.0])
SIGMA, AMP = 180.0, 0.75
L = float(np.linalg.norm(EVT - START))
U = (EVT - START) / L
PERP = np.array([-U[1], U[0]]) * 150.0
HOT = np.array([c + k * PERP for x in np.linspace(0.0, 2843.0, 8) for c in [EVT - x * U] for k in (0, 1, -1)])


def rho(dist):
    pos = START + dist * U
    return AMP * np.sum(np.exp(-np.sum((HOT - pos) ** 2, axis=1) / (2 * SIGMA**2)))


PEAK = max(rho(x) for x in np.linspace(0, L, 2000))


def alphas_at(t, speed, headway):
    """Returns [(rnti, dist, rho_norm, alpha)] for every UE at flight time t."""
    rows = [(r, 0.0, 0.0, ALPHA_FLOOR) for r in RNTI_A]
    for j, r in enumerate(RNTI_B):
        s = speed * t - j * headway
        if 0.0 <= s < L:
            rn = rho(s) / PEAK
            rows.append((r, s, rn, ALPHA_FLOOR + ALPHA_GAIN * rn))
        else:
            rows.append((r, min(max(s, 0.0), L), 0.0, ALPHA_FLOOR))
    return rows


def write_alpha(path, rows):
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        f.writelines(f"{r} {a:.4f}\n" for r, _, _, a in rows)
    os.replace(tmp, path)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tag", required=True)
    ap.add_argument("--policy", choices=["risk", "pf"], default="risk")
    ap.add_argument("--speed", type=float, default=45.0, help="m/s (15 = tempo real do RC14)")
    ap.add_argument("--headway", type=float, default=700.0, help="separacao entre drones do corredor B, m")
    ap.add_argument("--tail", type=float, default=8.0, help="segundos apos o ultimo pouso")
    args = ap.parse_args()

    out = os.path.join(HERE, "logs", "flight", args.tag)
    os.makedirs(out, exist_ok=True)
    alpha_host = os.path.join(out, "alpha.txt")
    alpha_cont = f"/work/logs/flight/{args.tag}/alpha.txt"
    flight = (L + (len(RNTI_B) - 1) * args.headway) / args.speed
    dur = flight + args.tail
    write_alpha(alpha_host, alphas_at(-1.0, args.speed, args.headway))

    cfgs = ["-c", "/work/configs/smoke_testmode.yml", "-c", "/work/configs/k1_per_slot.yml"]
    if args.policy == "risk":
        with open(os.path.join(out, "policy.yml"), "w") as f:
            f.write("cell_cfg:\n  scheduler:\n    policy:\n      risk_sched:\n        ues_per_slot: 1\n"
                    f"        success_prob: 0.9\n        alpha_file: {alpha_cont}\n        alpha_reload_period_ms: 500\n")
        cfgs += ["-c", f"/work/logs/flight/{args.tag}/policy.yml"]
    cmd = ["bash", os.path.join(HERE, "dev.sh"), "timeout", "-s", "INT", str(int(dur + 15)),
           "/build/apps/gnb/gnb", *cfgs, "log", "--filename", f"/work/logs/flight/{args.tag}/gnb.log"]
    env = dict(os.environ, DEV_TTY="0", DEV_NAME=f"ocudu-flight-{args.tag}")
    print(f"[flight] rota {L:.0f} m, {len(RNTI_B)} drones a {args.speed} m/s, headway {args.headway} m, "
          f"voo {flight:.0f} s, politica {args.policy}")
    proc = subprocess.Popen(cmd, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)

    t0 = [None]
    stop = threading.Event()

    def driver():
        with open(os.path.join(out, "driver.csv"), "w") as f:
            f.write("t,rnti,dist,rho_norm,alpha\n")
            while not stop.is_set():
                t = time.time() - t0[0]
                rows = alphas_at(t, args.speed, args.headway)
                write_alpha(alpha_host, rows)
                f.writelines(f"{t:.2f},{r},{d:.1f},{rn:.4f},{a:.4f}\n" for r, d, rn, a in rows)
                f.flush()
                stop.wait(1.0)

    mult = {"k": 1e3, "M": 1e6, "G": 1e9}
    th = None
    with open(os.path.join(out, "metrics.csv"), "w") as m, open(os.path.join(out, "stdout.log"), "w") as raw:
        m.write("t,rnti,ul_brate_bps,ul_grants\n")
        for line in proc.stdout:
            raw.write(line)
            parts = line.split("|")
            head = parts[0].split()
            if len(parts) != 3 or len(head) != 2 or head[0] != "1":
                continue
            if t0[0] is None:
                # The flight clock starts at the first metrics report, when the cell is up.
                t0[0] = time.time()
                th = threading.Thread(target=driver, daemon=True)
                th.start()
            ul = parts[2].split()
            b = ul[4]
            brate = float(b[:-1]) * mult[b[-1]] if b[-1] in mult else float(b)
            t = time.time() - t0[0]
            m.write(f"{t:.2f},{head[1]},{brate:.0f},{ul[5]}\n")
            m.flush()
            if t > dur:
                break
    stop.set()
    subprocess.run(["docker", "kill", "-s", "INT", env["DEV_NAME"]], capture_output=True)
    proc.wait(timeout=30)
    print(f"[flight] concluido -> {out}")


if __name__ == "__main__":
    main()

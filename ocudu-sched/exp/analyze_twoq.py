#!/usr/bin/env python3
"""
analyze_twoq.py -- AoI por fonte a partir do log de entregas do modo de teste (fonte C2 + video GOP).

Entrada: CSV do gNB (t_us,rnti,source,event,seq,bytes,gen_us); source = c2 | vI | vP; event = rx | drop.
Para cada UE e fonte (C2, video):
  n_rx, n_drop        atualizacoes entregues e substituidas antes de sair
  delay_ms            media de (entrega - geracao)
  aoi_ms              AoI medio no tempo (dente de serra: idade = t - geracao da ultima entregue)
  peak_p99_ms         p99 do pico de idade imediatamente antes de cada entrega
Uso: exp/analyze_twoq.py logs/twoq/<tag>.csv [--skip 5]   (descarta os primeiros --skip segundos)
"""
import argparse
import csv
from collections import defaultdict

import numpy as np


def aoi_stats(rx):
    """rx: lista (t_entrega_us, t_geracao_us) em ordem de entrega."""
    if len(rx) < 2:
        return float("nan"), float("nan")
    d = np.array([r[0] for r in rx], float)
    g = np.array([r[1] for r in rx], float)
    area = ((d[1:] - g[:-1]) ** 2 - (d[:-1] - g[:-1]) ** 2) / 2.0
    peaks = d[1:] - g[:-1]
    return area.sum() / (d[-1] - d[0]) / 1e3, np.percentile(peaks, 99) / 1e3


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--skip", type=float, default=5.0)
    a = ap.parse_args()

    rx = defaultdict(list)
    drop = defaultdict(int)
    t_min = None
    with open(a.csv) as f:
        for row in csv.DictReader(f):
            t = int(row["t_us"])
            t_min = t if t_min is None else t_min
            if t - t_min < a.skip * 1e6:
                continue
            key = (row["rnti"], "c2" if row["source"] == "c2" else "video")
            if row["event"] == "rx":
                rx[key].append((t, int(row["gen_us"])))
            else:
                drop[key] += 1

    print(f"{'rnti':>5} {'src':>6} {'n_rx':>6} {'n_drop':>6} {'delay_ms':>9} {'aoi_ms':>8} {'peak_p99_ms':>12}")
    for key in sorted(set(rx) | set(drop)):
        r = rx.get(key, [])
        delay = np.mean([t - g for t, g in r]) / 1e3 if r else float("nan")
        aoi, p99 = aoi_stats(r)
        print(f"{key[0]:>5} {key[1]:>6} {len(r):6d} {drop.get(key, 0):6d} {delay:9.1f} {aoi:8.1f} {p99:12.1f}")


if __name__ == "__main__":
    main()

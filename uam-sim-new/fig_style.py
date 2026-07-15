"""
fig_style.py -- shared IEEE single-column style for the INFOCOM figures.
Import this from every figure script (impossibility, CCDF, lower-bound) so the
policy->(color, linestyle, marker) mapping and typography stay identical.

Grayscale-safe: each policy is distinguished by linestyle AND marker, not color
alone (RR=light gray/dashed/triangle, PF=mid gray/dash-dot/square,
Risk-Aware=black/solid/circle).
"""
import matplotlib as mpl

# canonical policy id -> display label
POLICIES = ["round-robin-sw", "pf-classic-sw", "mr-mw-ch-sw"]
# RC8 extension: canonical 3 + risk-weighted PF (opt-in via POLICIES_RC8)
POLICIES_RC8 = POLICIES + ["max-risk-pf-sw"]
LABEL = {
    "round-robin-sw": "RR",
    "pf-classic-sw":  "PF",
    "mr-mw-ch-sw":    "Risk-Aware",
    "max-risk-pf-sw": "Risk-PF",
    "mw-sw-otimo-ch": "MW-opt (sw)",
    "mw-otimo-ch":    "MW-opt",
    "mw-sw-v8":       "MW-V8",
    "mw-sw-v9":       "MW-V9",
    "mw-v9-nsw":      "MW-V9-nsw",
}
# policy -> (color, linestyle, marker)   -- grayscale-safe
STYLE = {
    "round-robin-sw": ("#B0B0B0", (0, (5, 2)),      "^"),   # light gray, dashed,   triangle
    "pf-classic-sw":  ("#6E6E6E", (0, (4, 1, 1, 1)), "s"),  # mid gray,   dash-dot, square
    "mr-mw-ch-sw":    ("#000000", "-",              "o"),   # black,      solid,    circle
    "max-risk-pf-sw": ("#3C3C3C", (0, (1, 1)),      "D"),   # dark gray,  dotted,   diamond
    "mw-sw-otimo-ch": ("#555555", (0, (6, 2, 1, 2)), "v"),  # dark gray,  dash-dot-long, tri-down
    "mw-otimo-ch":    ("#8A8A8A", (0, (2, 1)),      "P"),   # mid gray,   short-dash, plus
    "mw-sw-v8":       ("#4F4F4F", (0, (5, 1)),      "X"),   # dark gray,  dense-dash, X
    "mw-sw-v9":       ("#2B2B2B", (0, (3, 1, 1, 1)), "^"),  # near-black, dash-dot-dot, tri-up
    "mw-v9-nsw":      ("#7A7A7A", (0, (2, 1, 1, 1)), "d"),  # gray, dot-dash, thin-diamond
}

# IEEE single-column typography: serif/Times, every text element >= 7 pt.
RC = {
    "font.family": "serif",
    "font.serif": ["Times New Roman", "Nimbus Roman", "Times", "DejaVu Serif"],
    "mathtext.fontset": "stix",
    "font.size": 8,
    "axes.titlesize": 8,
    "axes.labelsize": 8,
    "legend.fontsize": 7,
    "xtick.labelsize": 7,
    "ytick.labelsize": 7,
    "axes.linewidth": 0.6,
    "lines.linewidth": 1.2,
    "lines.markersize": 4,
    "legend.frameon": False,
    "pdf.fonttype": 42,   # embed TrueType (editable, IEEE-friendly)
    "ps.fonttype": 42,
}

def apply():
    mpl.rcParams.update(RC)

def line_kwargs(policy):
    c, ls, mk = STYLE[policy]
    return dict(color=c, linestyle=ls, marker=mk, markerfacecolor=c,
                markeredgecolor=c, label=LABEL[policy])

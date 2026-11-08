"""Die Lernkurve zeichnen - macht sichtbar, was das Training unsichtbar tut.
START:  python scripts/34_plot_curve.py
"""
import pathlib
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

runs = {
    "Lauf 2 (korrigiert)": pathlib.Path("runs/stand/progress.csv"),
}
alt = pathlib.Path("runs/stand/lauf1_progress.csv")
if alt.exists():
    runs["Lauf 1 (zu viel Exploration)"] = alt

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 4.5))
colors = {"Lauf 2 (korrigiert)": "#1a7f37", "Lauf 1 (zu viel Exploration)": "#b0b0b0"}

for label, path in runs.items():
    if not path.exists():
        continue
    d = np.loadtxt(path, delimiter=",", skiprows=1)
    if d.ndim == 1:
        d = d[None, :]
    steps, reward, ep_len = d[:, 0], d[:, 1], d[:, 2]
    c = colors.get(label, "#888")
    ax1.plot(steps, reward, color=c, lw=1.8, label=label)
    ax2.plot(steps, 100 * ep_len / 400, color=c, lw=1.8, label=label)

# Referenzlinie: 'nichts tun' schafft im Schnitt 169 von 400 Schritten
ax2.axhline(100 * 169 / 400, color="#d1242f", ls="--", lw=1.5,
            label="'nichts tun' (169/400)")
ax2.axhline(100, color="#0969da", ls=":", lw=1.5, label="Ziel: nie umfallen")

ax1.set_xlabel("Trainingsschritte"); ax1.set_ylabel("Belohnung pro Episode")
ax1.set_title("Belohnung - geht sie hoch, lernt er"); ax1.grid(alpha=.3); ax1.legend(fontsize=8)
ax2.set_xlabel("Trainingsschritte"); ax2.set_ylabel("% der Episode durchgehalten")
ax2.set_title("Wie lange bleibt er stehen?"); ax2.grid(alpha=.3); ax2.legend(fontsize=8)
ax2.set_ylim(0, 105)
plt.tight_layout()
out = pathlib.Path("docs/lernkurve.png")
plt.savefig(out, dpi=130)
print("gespeichert:", out)

d = np.loadtxt(list(runs.values())[0], delimiter=",", skiprows=1)
print(f"aktuell: {int(d[-1,0]):,} Schritte | Belohnung {d[-1,1]:.0f} "
      f"| Episode {d[-1,2]:.0f}/400 | seit {d[-1,3]/60:.1f} min")

"""Generate the Bench2Drive v0.0.4 update banner (README figure).

Design: title + "strictly uniform scenario distribution" comparison
(v0.0.4 uniform donut vs Base uneven donut) + three update features +
acknowledgement footer. All colors come from the validated dataviz palette.
"""
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

# ---- palette (dataviz reference, all validated) ----
SURFACE  = "#fcfcfb"   # chart surface
INK      = "#0b0b0b"   # primary ink
SEC      = "#52514e"   # secondary ink
MUTED    = "#898781"   # muted ink
BLUE     = "#2a78d6"   # categorical slot 1
BLUE_SEQ = ["#cde2fb", "#9ec5f4", "#6da7ec", "#3987e5",
            "#2a78d6", "#1c5cab", "#184f95", "#0d366b"]  # blue sequential ramp
FEAT = ["#2a78d6", "#eb6834", "#1baf7a"]   # slots 1-3 for the three features

W, H = 16, 9
fig = plt.figure(figsize=(W, H), dpi=200)
fig.patch.set_facecolor(SURFACE)

# ---------------------------------------------------------------- title
fig.text(0.60, 0.885, "Bench2Drive", fontsize=46, fontweight="bold",
         color=INK, ha="left", va="center")
fig.text(1.02, 0.885, "v0.0.4", fontsize=46, fontweight="bold",
         color=BLUE, ha="left", va="center")

fig.text(0.60, 0.815, "Strictly uniform scenario distribution — the motivation behind this update",
         fontsize=17, color=SEC, ha="left", va="center")

# ---------------------------------------------------------------- donut 1: v0.0.4 uniform
ax1 = fig.add_axes([0.07, 0.17, 0.28, 0.48])
n = 44
ax1.pie(np.ones(n), colors=[BLUE] * n, startangle=90, counterclock=False,
        wedgeprops=dict(width=0.46, edgecolor=SURFACE, linewidth=1.4))
ax1.text(0, 0, "44", ha="center", va="center", color=INK,
         fontsize=44, fontweight="bold")
ax1.text(0, -0.22, "scenarios", ha="center", va="center", color=SEC, fontsize=15)
ax1.set_facecolor(SURFACE)
ax1.axis("equal")

# badge above donut 1
fig.text(0.21, 0.70, "NEW · v0.0.4", ha="center", va="center", color=BLUE,
         fontsize=15, fontweight="bold",
         bbox=dict(boxstyle="round,pad=0.35", facecolor="#eaf1fb", edgecolor="none"))
fig.text(0.21, 0.615, "44 scenarios × 25 routes", ha="center", va="center",
         color=INK, fontsize=16, fontweight="bold")
fig.text(0.21, 0.585, "= 1,100 routes, strictly uniform", ha="center", va="center",
         color=SEC, fontsize=13)

# ---------------------------------------------------------------- donut 2: Base uneven
rng = np.random.RandomState(7)
ax2 = fig.add_axes([0.40, 0.17, 0.28, 0.48])
m = 12
# deliberately uneven slice sizes (small number of slices, wildly different)
raw = rng.dirichlet(np.ones(m)) * (1.0 + rng.rand(m) * 3.0)
sizes = np.sort(raw)[::-1]
ax2.pie(sizes, colors=[BLUE] * m, startangle=90, counterclock=False,
        wedgeprops=dict(width=0.46, edgecolor=SURFACE, linewidth=1.4))
ax2.text(0, 0, "Base", ha="center", va="center", color=INK,
         fontsize=30, fontweight="bold")
ax2.text(0, -0.22, "previous", ha="center", va="center", color=SEC, fontsize=15)
ax2.set_facecolor(SURFACE)
ax2.axis("equal")

fig.text(0.54, 0.70, "OLD · Base", ha="center", va="center", color=MUTED,
         fontsize=15, fontweight="bold",
         bbox=dict(boxstyle="round,pad=0.35", facecolor="#f2f1ec", edgecolor="none"))
fig.text(0.54, 0.615, "scenario distribution uneven", ha="center", va="center",
         color=INK, fontsize=16, fontweight="bold")
fig.text(0.54, 0.585, "some scenarios over-represented", ha="center", va="center",
         color=SEC, fontsize=13)

# arrow from OLD to NEW
ax2.annotate("", xy=(0.335, 0.42), xytext=(0.675, 0.42),
             arrowprops=dict(arrowstyle="-|>", color=BLUE, lw=3))

# ---------------------------------------------------------------- features (right)
FX = 0.70
fig.text(FX, 0.70, "What's new in 0.0.4", fontsize=20, fontweight="bold",
         color=INK, ha="left", va="center")

features = [
    ("3D Occupancy",        "New modality added to the dataset"),
    ("Separate Val Set",    "220 routes (44 scenarios × 5), independent of training"),
    ("Bug Fixes",           "Radar pose, skeleton collection, lane points, Zoo fixes"),
]
y0 = 0.58
for i, (title, desc) in enumerate(features):
    y = y0 - i * 0.115
    fig.patches.append(plt.Rectangle((FX + 0.002, y - 0.018), 0.012, 0.036,
                                     facecolor=FEAT[i], edgecolor="none"))
    fig.text(FX + 0.02, y, title, fontsize=17, fontweight="bold",
             color=INK, ha="left", va="center")
    fig.text(FX + 0.02, y - 0.043, desc, fontsize=12.5, color=SEC,
             ha="left", va="center")

# ---------------------------------------------------------------- footer
fig.text(0.5, 0.045, "Thanks to all open-source contributors for their feedback and help improving Bench2Drive.",
         ha="center", va="center", color=MUTED, fontsize=12.5)

fig.savefig("/tmp/v004_banner.png", facecolor=SURFACE, bbox_inches=None)
print("saved /tmp/v004_banner.png")

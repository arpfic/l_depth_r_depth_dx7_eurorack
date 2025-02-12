import re
import matplotlib.pyplot as plt

# This pattern captures:
# 1) cv_raw   2) cv_volts  3) vol
# 4) left_vol 5) left_ratio
# 6) right_vol 7) right_ratio
# 8) hz
pattern = re.compile(
    r"CV=([\d\.]+)/([\d\.]+)V\s+"   # e.g. CV=624.1/0.03V
    r"vol=(\d+)\s+"                 # e.g. vol=1373
    r"C=(\d+)/([\d\.]+)\s+"         # e.g. C=1373/0.02
    r"L=(\d+)/([\d\.]+)\s+"         # e.g. L=1373/0.02
    r"R=(\d+)/([\d\.]+)\s+"         # e.g. R=0/0.00
    r"Hz=(\d+)"                     # e.g. Hz=10201
)

file_path = "data_test.log"  # put your actual file here

cv_vals = []       # x-axis: CV raw
left_vols = []   # y1: left ratio
right_vols = []  # y2: right ratio
center_vols = [] # y3: center ratio

with open(file_path, "r", encoding="utf-8") as f:
    for line in f:
        match = pattern.search(line)
        if match:
            # extract strings
            cv_raw_str       = match.group(1)  # e.g. "624.1"
            cv_volt_str      = match.group(2)  # e.g. "0.03"
            vol_str          = match.group(3)  # "1373"
            center_vol_str   = match.group(4)  # "1373"
            center_ratio_str = match.group(5)  # "0.02"
            left_vol_str     = match.group(6)  # "1373"
            left_ratio_str   = match.group(7)  # "0.02"
            right_vol_str    = match.group(8)  # "0"
            right_ratio_str  = match.group(9)  # "0.00"
            hz_str           = match.group(10)  # "10201"

            # convert to numeric
            cv_raw         = float(cv_raw_str)
            # cv_volt      = float(cv_volt_str)  # if you also want it
            # volume       = int(vol_str)
            center_vol     = int(center_vol_str)
            #center_ratio  = float(center_ratio_str)
            left_vol       = int(left_vol_str)
            #left_ratio    = float(left_ratio_str)
            right_vol      = int(right_vol_str)
            #right_ratio   = float(right_ratio_str)
            # hz           = int(hz_str)

            cv_vals.append(cv_raw)
            center_vols.append(center_vol)
            left_vols.append(left_vol)
            right_vols.append(right_vol)

# OPTIONAL: sort data by ascending CV so the plot is "CV in ascending order"
data = list(zip(cv_vals, left_vols, right_vols, center_vols))
data.sort(key=lambda row: row[0])  # sort by CV
cv_vals      = [row[0] for row in data]
left_vols    = [row[1] for row in data]
right_vols   = [row[2] for row in data]
center_vols  = [row[3] for row in data]

# Plot
plt.figure(figsize=(8, 5))
plt.plot(cv_vals, left_vols,  label="Left ratio",  marker='o')
plt.plot(cv_vals, right_vols, label="Right ratio", marker='o')
plt.plot(cv_vals, center_vols, label="Center ratio", marker='o')
plt.xlabel("CV (raw)")
plt.ylabel("Ratio")
plt.title("Left/Right/Center Ratios vs. CV")
plt.grid(True)
plt.legend()
plt.show()


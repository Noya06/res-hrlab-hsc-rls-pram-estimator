from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# ---------------------------------------------------------
# 1. データ読み込み
# ---------------------------------------------------------
workspace_dir = Path(__file__).resolve().parents[3]
filename = workspace_dir / 'data' / 'ref_hum_onlyleft.csv'
df = pd.read_csv(filename, comment='#')

# 時間刻み dt の計算
dt = np.diff(df['time']).mean()

# ---------------------------------------------------------
# 2. 運動学に基づく機体位置 (x, y) および 正解Path の算出
# ---------------------------------------------------------
x = np.zeros(len(df))
y = np.zeros(len(df))

for i in range(1, len(df)):
    v_xi = df.loc[i, 'v_xi']
    v_r  = df.loc[i, 'v_r']
    e3   = df.loc[i, 'e_3']
    
    # 速度成分の分解
    dx = (v_xi * np.cos(e3) - v_r * np.sin(e3)) * dt
    dy = (v_xi * np.sin(e3) + v_r * np.cos(e3)) * dt
    
    x[i] = x[i-1] + dx
    y[i] = y[i-1] + dy

# ---------------------------------------------------------
# SplinePathGenerator.cs と同じ計算式による正解 Path の算出
# ---------------------------------------------------------
# Unityの指定パラメータ設定
start_z = -1.0
main_amplitude = 1.0
main_wavelength = 20.0
sub_amplitude = 1.0
sub_wavelength = 10.0

# 進行方向 x に対応する dz を計算
dz = x - start_z

k1 = 2.0 * np.pi / main_wavelength
k2 = 2.0 * np.pi / sub_wavelength

wave1 = main_amplitude * np.sin(dz * k1)
wave2 = sub_amplitude * np.sin(dz * k2)

# 正解Pathのy座標
y_path = wave1 + wave2

# ---------------------------------------------------------
# 3. グラフ描画の設定
# ---------------------------------------------------------
plt.rcParams.update({
    'font.size': 14,
    'axes.titlesize': 12,  # タイトルを小さめ
    'axes.labelsize': 15,
    'xtick.labelsize': 13,
    'ytick.labelsize': 13,
    'legend.fontsize': 13
})

fig, axs = plt.subplots(2, 2, figsize=(15, 12))

# --- 左上: x-y 座標 (機体の動き vs 正解Path) ---
axs[0, 0].plot(x, y, label='Vehicle Trajectory', color='blue', linewidth=2)
axs[0, 0].plot(x, y_path, label='Reference Path', color='red', linestyle='--', linewidth=2)
axs[0, 0].set_title('Vehicle Trajectory vs Reference Path')
axs[0, 0].set_xlabel('x [m]')
axs[0, 0].set_ylabel('y [m]')
axs[0, 0].grid(True, linestyle=':', alpha=0.7)
axs[0, 0].legend()
axs[0, 0].axis('equal')  # アスペクト比を1:1に保持

# --- 右上: 横偏差 e_2 - time ---
axs[0, 1].plot(df['time'], df['e_2'], color='darkorange', linewidth=2)
axs[0, 1].set_title('Lateral Error $e_2$ over Time')
axs[0, 1].set_xlabel('Time [s]')
axs[0, 1].set_ylabel('$e_2$ [m]')
axs[0, 1].grid(True, linestyle=':', alpha=0.7)

# --- 左下: 回転誤差 e_3 - time ---
axs[1, 0].plot(df['time'], df['e_3'], color='green', linewidth=2)
axs[1, 0].set_title('Orientation Error $e_3$ over Time')
axs[1, 0].set_xlabel('Time [s]')
axs[1, 0].set_ylabel('$e_3$ [rad]')
axs[1, 0].grid(True, linestyle=':', alpha=0.7)

# --- 右下: phi_hum_y - time ---
axs[1, 1].plot(df['time'], df['phi_hum_y'], color='purple', linewidth=2)
axs[1, 1].set_title('Human Input $\phi_{hum\_y}$ over Time')
axs[1, 1].set_xlabel('Time [s]')
axs[1, 1].set_ylabel('$\phi_{hum\_y}$')
axs[1, 1].grid(True, linestyle=':', alpha=0.7)

# 全体レイアウト調整後に上下の間隔(hspace)を広げる
plt.tight_layout()
fig.subplots_adjust(hspace=0.4)

# ---------------------------------------------------------
# 4. ソースファイル名に応じた画像保存名を作成
# ---------------------------------------------------------
output_image_path = filename.parent / f"result_plots_{filename.stem}.png"

# 画像ファイルとして保存
plt.savefig(output_image_path, dpi=300, bbox_inches='tight')
print(f"グラフ画像を保存しました: {output_image_path}")

# 画面表示
plt.show()
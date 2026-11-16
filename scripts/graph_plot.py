from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# ---------------------------------------------------------
# 1. データ読み込み
# ---------------------------------------------------------
workspace_dir = Path(__file__).resolve().parents[3]
# プロット対象のCSV指定
filename = workspace_dir / 'data' / 'result_20261116_193712_steps10000.csv'
df = pd.read_csv(filename, comment='#')

# 時間刻み dt の計算
dt = np.diff(df['time']).mean()

# ---------------------------------------------------------
# 2. 運動学に基づく機体位置 (x, y) および 正解Path の算出
# ---------------------------------------------------------
# Unityの指定パラメータ設定 (SplinePathGenerator.cs と同じ計算式)
start_z = -1.0
main_amplitude = 1.0
main_wavelength = 20.0
sub_amplitude = 1.0
sub_wavelength = 10.0

k1 = 2.0 * np.pi / main_wavelength
k2 = 2.0 * np.pi / sub_wavelength

def calc_path_y_and_angle(x_val):
    dz = x_val - start_z
    y_p = main_amplitude * np.sin(dz * k1) + sub_amplitude * np.sin(dz * k2)
    # 経路の勾配 dy/dx から接線角 theta_path を算出
    dy_dx = (main_amplitude * k1 * np.cos(dz * k1)
             + sub_amplitude * k2 * np.cos(dz * k2))
    theta_p = np.arctan(dy_dx)
    return y_p, theta_p

x = np.zeros(len(df))
y = np.zeros(len(df))

# 初期位置の補正（CSVに e_2 がある場合は初期横偏差を反映）
y_init, _ = calc_path_y_and_angle(x[0])
y[0] = y_init + (df.loc[0, 'e_2'] if 'e_2' in df.columns else 0.0)

for i in range(1, len(df)):
    v_xi = df.loc[i, 'v_xi']
    e3   = df.loc[i, 'e_3']
    
    # 現在位置 x[i-1] における経路接線角を取得し、グローバル方位角 psi を算出
    # ※ e_3 の符号定義が逆の場合は (theta_path - e3) に変更してください
    # ※ e_3 自体がグローバル方位角の場合は psi = e3 としてください
    _, theta_path = calc_path_y_and_angle(x[i-1])
    psi = theta_path + e3
    
    # 前進速度 v_xi のみで速度成分を分解（v_r の混入による45度傾き・√2倍化を解消）
    dx = v_xi * np.cos(psi) * dt
    dy = v_xi * np.sin(psi) * dt
    
    x[i] = x[i-1] + dx
    y[i] = y[i-1] + dy

# 正解Pathのy座標（全区間）
y_path, _ = calc_path_y_and_angle(x)

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

fig, axs = plt.subplots(4, 2, figsize=(15, 20))

# --- 1行目左: x-y 座標 (機体の動き vs 正解Path) ---
axs[0, 0].plot(x, y, label='Vehicle Trajectory', color='blue', linewidth=2)
axs[0, 0].plot(x, y_path, label='Reference Path', color='red', linestyle='--', linewidth=2)
axs[0, 0].set_title('Vehicle Trajectory vs Reference Path')
axs[0, 0].set_xlabel('x [m]')
axs[0, 0].set_ylabel('y [m]')
axs[0, 0].grid(True, linestyle=':', alpha=0.7)
axs[0, 0].legend()
axs[0, 0].axis('equal')  # アスペクト比を1:1に保持

# --- 1行目右: 横偏差 e_2 - time ---
axs[0, 1].plot(df['time'], df['e_2'], color='darkorange', linewidth=2)
axs[0, 1].set_title('Lateral Error $e_2$ over Time')
axs[0, 1].set_xlabel('Time [s]')
axs[0, 1].set_ylabel('$e_2$ [m]')
axs[0, 1].grid(True, linestyle=':', alpha=0.7)

# --- 2行目左: 回転誤差 e_3 - time ---
axs[1, 0].plot(df['time'], df['e_3'], color='green', linewidth=2)
axs[1, 0].set_title('Orientation Error $e_3$ over Time')
axs[1, 0].set_xlabel('Time [s]')
axs[1, 0].set_ylabel('$e_3$ [rad]')
axs[1, 0].grid(True, linestyle=':', alpha=0.7)

# --- 2行目右: IOC/RLS control signals - time ---
axs[1, 1].plot(
    df['time'], df['phi_ioc'], color='purple', linewidth=2, label=r'$\phi_{ioc}$')
axs[1, 1].plot(
    df['time'], df['rls_prediction'], color='teal', linewidth=2,
    label='RLS prediction')
axs[1, 1].plot(
    df['time'], df['target_roll'], color='black', linestyle='--',
    linewidth=2, label=r'$\phi_{ioc}$ + RLS')
axs[1, 1].set_title('IOC and RLS Control Signals over Time')
axs[1, 1].set_xlabel('Time [s]')
axs[1, 1].set_ylabel('Control input')
axs[1, 1].grid(True, linestyle=':', alpha=0.7)
axs[1, 1].legend()

# --- 3行目左: human angle and inverted target roll - time ---
if {'phi_hum', 'target_roll'}.issubset(df.columns):
    axs[2, 0].plot(df['time'], df['phi_hum'], label=r'$\phi_{hum}$')
    axs[2, 0].plot(
        df['time'], -df['target_roll'], linestyle='--', label='-target_roll')
    axs[2, 0].set_title('Human angle and inverted target roll')
    axs[2, 0].set_xlabel('Time [s]')
    axs[2, 0].set_ylabel('Value')
    axs[2, 0].grid(True, linestyle=':', alpha=0.7)
    axs[2, 0].legend()
else:
    axs[2, 0].text(
        0.5, 0.5, 'Required columns are not available in this CSV',
        ha='center', va='center')
    axs[2, 0].set_axis_off()

# --- 3行目右: RLS parameter changes - time ---
param_label_map = {
    'rls_theta_0': r'$\hat{\theta}_0$ ($e_2$)',
    'rls_theta_1': r'$\hat{\theta}_1$ ($e_3$)',
    'rls_theta_2': r'$\hat{\theta}_2$ ($e_2|e_2|$)',
    'rls_theta_3': r'$\hat{\theta}_3$ ($e_3|e_3|$)',
    'rls_theta_4': r'$\hat{\theta}_4$ ($e_2|e_3|$)',
    'rls_theta_5': r'$\hat{\theta}_5$ ($|e_2|e_3$)',
    'rls_theta_6': r'$\hat{\theta}_6$ (bias 1.0)',
    'rls_theta_7': r'$\hat{\theta}_7$ ($\rho$)',
    'rls_theta_8': r'$\hat{\theta}_8$ ($e_2(t-L)$)',
    'rls_theta_9': r'$\hat{\theta}_9$ ($e_3(t-L)$)',
    'rls_theta_e2': r'$\hat{\theta}_{e_2}$ ($e_2$)',
    'rls_theta_e3': r'$\hat{\theta}_{e_3}$ ($e_3$)',
    'rls_theta_e2_abs_e2': r'$\hat{\theta}_{e_2|e_2|}$ ($e_2|e_2|$)',
    'rls_theta_e3_abs_e3': r'$\hat{\theta}_{e_3|e_3|}$ ($e_3|e_3|$)',
    'rls_theta_e2_abs_e3': r'$\hat{\theta}_{e_2|e_3|}$ ($e_2|e_3|$)',
    'rls_theta_abs_e2_e3': r'$\hat{\theta}_{|e_2|e_3}$ ($|e_2|e_3$)',
    'rls_theta_bias': r'$\hat{\theta}_{\mathrm{bias}}$ (bias 1.0)',
    'rls_theta_rho': r'$\hat{\theta}_{\rho}$ ($\rho$)',
    'rls_theta_e2_delay': r'$\hat{\theta}_{e_2(t-L)}$ ($e_2(t-L)$)',
    'rls_theta_e3_delay': r'$\hat{\theta}_{e_3(t-L)}$ ($e_3(t-L)$)',
}

theta_columns = [column for column in df.columns if column.startswith('rls_theta_')]
if theta_columns:
    for column in theta_columns:
        display_label = param_label_map.get(column, column)
        axs[2, 1].plot(df['time'], df[column], label=display_label)
    axs[2, 1].set_title('RLS parameter changes')
    axs[2, 1].set_xlabel('Time [s]')
    axs[2, 1].set_ylabel(r'$\hat{\theta}$')
    axs[2, 1].grid(True, linestyle=':', alpha=0.7)
    axs[2, 1].legend(ncol=2, fontsize=10)
else:
    axs[2, 1].text(
        0.5, 0.5, 'RLS parameter columns are not available in this CSV',
        ha='center', va='center')
    axs[2, 1].set_axis_off()

# --- 4行目左: phi_hum - (-target_roll) - time ---
if {'phi_hum', 'target_roll'}.issubset(df.columns):
    diff_roll = df['phi_hum'] - (-df['target_roll'])
    axs[3, 0].plot(
        df['time'], diff_roll, color='crimson', linewidth=2,
        label=r'$\phi_{hum} - (-\mathrm{target\_roll})$')
    axs[3, 0].axhline(0, color='gray', linestyle=':', linewidth=1)
    axs[3, 0].set_title('Difference: phi_hum - (-target_roll)')
    axs[3, 0].set_xlabel('Time [s]')
    axs[3, 0].set_ylabel('Value')
    axs[3, 0].grid(True, linestyle=':', alpha=0.7)
    axs[3, 0].legend()
else:
    axs[3, 0].text(
        0.5, 0.5, 'Required columns are not available in this CSV',
        ha='center', va='center')
    axs[3, 0].set_axis_off()

# --- 4行目右: 非線形誤差項 e_2|e_2|, e_3|e_3|, e_2|e_3|, |e_2|e_3 - time ---
e2_abs_e2 = df['e_2_abs_e_2'] if 'e_2_abs_e_2' in df.columns else df['e_2'] * df['e_2'].abs()
e3_abs_e3 = df['e_3_abs_e_3'] if 'e_3_abs_e_3' in df.columns else df['e_3'] * df['e_3'].abs()
e2_abs_e3 = df['e_2_abs_e_3'] if 'e_2_abs_e_3' in df.columns else df['e_2'] * df['e_3'].abs()
abs_e2_e3 = df['abs_e_2_e_3'] if 'abs_e_2_e_3' in df.columns else df['e_2'].abs() * df['e_3']

axs[3, 1].plot(df['time'], e2_abs_e2, color='darkorange', linewidth=2, label=r'$e_2|e_2|$')
axs[3, 1].plot(df['time'], e3_abs_e3, color='green', linestyle='--', linewidth=2, label=r'$e_3|e_3|$')
axs[3, 1].plot(df['time'], e2_abs_e3, color='royalblue', linewidth=2, label=r'$e_2|e_3|$')
axs[3, 1].plot(df['time'], abs_e2_e3, color='mediumorchid', linestyle='--', linewidth=2, label=r'$|e_2|e_3$')
if 'rho' in df.columns:
    axs[3, 1].plot(df['time'], df['rho'], color='black', linestyle=':', linewidth=2, label=r'$\rho$')
if 'e_2_delay' in df.columns:
    axs[3, 1].plot(df['time'], df['e_2_delay'], color='saddlebrown', linestyle='-.', linewidth=1.5, label=r'$e_2(t-L)$')
if 'e_3_delay' in df.columns:
    axs[3, 1].plot(df['time'], df['e_3_delay'], color='darkcyan', linestyle='-.', linewidth=1.5, label=r'$e_3(t-L)$')
axs[3, 1].axhline(0, color='gray', linestyle=':', linewidth=1)
axs[3, 1].set_title('Nonlinear Error Terms over Time')
axs[3, 1].set_xlabel('Time [s]')
axs[3, 1].set_ylabel('Value')
axs[3, 1].grid(True, linestyle=':', alpha=0.7)
axs[3, 1].legend(ncol=2)

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
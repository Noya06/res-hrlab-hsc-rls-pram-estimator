from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# ---------------------------------------------------------
# 1. データ読み込み
# ---------------------------------------------------------
workspace_dir = Path(__file__).resolve().parents[3]
# プロット対象のCSV指定
filename = workspace_dir / 'data' / 'まともに動かず_result_20261117_001637_steps10000.csv'
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
    _, theta_path = calc_path_y_and_angle(x[i-1])
    psi = theta_path + e3
    
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
    'font.size': 12,
    'axes.titlesize': 12,
    'axes.labelsize': 12,
    'xtick.labelsize': 10,
    'ytick.labelsize': 10,
    'legend.fontsize': 10
})

# =========================================================
# 1枚目: 軌道・追従誤差・制御信号 (2行 x 2列, 16:9)
# =========================================================
fig1, axs1 = plt.subplots(2, 2, figsize=(16, 9))

# --- 1行目左: x-y 座標 (機体の動き vs 正解Path) ---
axs1[0, 0].plot(x, y, label='Vehicle Trajectory', color='blue', linewidth=2)
axs1[0, 0].plot(x, y_path, label='Reference Path', color='red', linestyle='--', linewidth=2)
axs1[0, 0].set_title('Vehicle Trajectory vs Reference Path')
axs1[0, 0].set_xlabel('x [m]')
axs1[0, 0].set_ylabel('y [m]')
axs1[0, 0].grid(True, linestyle=':', alpha=0.7)
axs1[0, 0].legend()
axs1[0, 0].axis('equal')

# --- 1行目右: 横偏差 e_2 - time ---
axs1[0, 1].plot(df['time'], df['e_2'], color='darkorange', linewidth=2)
axs1[0, 1].set_title('Lateral Error $e_2$ over Time')
axs1[0, 1].set_xlabel('Time [s]')
axs1[0, 1].set_ylabel('$e_2$ [m]')
axs1[0, 1].grid(True, linestyle=':', alpha=0.7)

# --- 2行目左: 回転誤差 e_3 - time ---
axs1[1, 0].plot(df['time'], df['e_3'], color='green', linewidth=2)
axs1[1, 0].set_title('Orientation Error $e_3$ over Time')
axs1[1, 0].set_xlabel('Time [s]')
axs1[1, 0].set_ylabel('$e_3$ [rad]')
axs1[1, 0].grid(True, linestyle=':', alpha=0.7)

# --- 2行目右: IOC/RLS control signals - time ---
axs1[1, 1].plot(
    df['time'], df['phi_ioc'], color='purple', linewidth=2, label=r'$\phi_{ioc}$')
axs1[1, 1].plot(
    df['time'], df['rls_prediction'], color='teal', linewidth=2,
    label='RLS prediction')
axs1[1, 1].plot(
    df['time'], df['target_roll'], color='black', linestyle='--',
    linewidth=2, label=r'$\phi_{ioc}$ + RLS')
axs1[1, 1].set_title('IOC and RLS Control Signals over Time')
axs1[1, 1].set_xlabel('Time [s]')
axs1[1, 1].set_ylabel('Control input')
axs1[1, 1].set_ylim(-1, 1)
axs1[1, 1].grid(True, linestyle=':', alpha=0.7)
axs1[1, 1].legend()

plt.tight_layout()
fig1.subplots_adjust(hspace=0.32, wspace=0.22)

# =========================================================
# 2枚目: 人間入力比較 & RLS詳細 (2行 x 2列, 16:9)
# =========================================================
fig2, axs2 = plt.subplots(2, 2, figsize=(16, 9))

# --- 1行目左: human angle and inverted target roll - time ---
if {'phi_hum', 'target_roll'}.issubset(df.columns):
    axs2[0, 0].plot(df['time'], df['phi_hum'], label=r'$\phi_{hum}$')
    axs2[0, 0].plot(
        df['time'], -df['target_roll'], linestyle='--', label='-target_roll')
    axs2[0, 0].set_title('Human angle and inverted target roll')
    axs2[0, 0].set_xlabel('Time [s]')
    axs2[0, 0].set_ylabel('Value')
    axs2[0, 0].grid(True, linestyle=':', alpha=0.7)
    axs2[0, 0].legend()
else:
    axs2[0, 0].text(
        0.5, 0.5, 'Required columns are not available in this CSV',
        ha='center', va='center')
    axs2[0, 0].set_axis_off()

# --- 1行目右: RLS parameter changes - time ---
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
        axs2[0, 1].plot(df['time'], df[column], label=display_label)
    axs2[0, 1].set_title('RLS parameter changes')
    axs2[0, 1].set_xlabel('Time [s]')
    axs2[0, 1].set_ylabel(r'$\hat{\theta}$')
    axs2[0, 1].grid(True, linestyle=':', alpha=0.7)
    axs2[0, 1].legend(ncol=2, fontsize=8.5)
else:
    axs2[0, 1].text(
        0.5, 0.5, 'RLS parameter columns are not available in this CSV',
        ha='center', va='center')
    axs2[0, 1].set_axis_off()

# --- 2行目左: 予測誤差 (Innovation) と RMSE ---
has_innovation = 'rls_innovation' in df.columns
has_rmse = 'rls_rmse' in df.columns

if has_innovation or has_rmse:
    if has_rmse:
        axs2[1, 0].plot(
            df['time'], df['rls_rmse'], color='blue', linewidth=2,
            label=r'Online Cumulative RMSE ($\sqrt{\frac{1}{N}\sum (y-\xi^T\theta)^2}$)')
    if has_innovation:
        window_size = 200 if len(df) >= 200 else max(1, len(df))
        moving_rmse = np.sqrt(
            (df['rls_innovation'] ** 2).rolling(window=window_size, min_periods=1).mean()
        )
        axs2[1, 0].plot(
            df['time'], moving_rmse, color='purple', linestyle='--', linewidth=1.5,
            label=f'Moving RMSE (window={window_size})')
    axs2[1, 0].set_title('RLS Prediction Error RMSE over Time')
    axs2[1, 0].set_xlabel('Time [s]')
    axs2[1, 0].set_ylabel('RMSE [rad]')
    axs2[1, 0].set_ylim(0, 1)
    axs2[1, 0].grid(True, linestyle=':', alpha=0.7)
    axs2[1, 0].legend(fontsize=9)
else:
    axs2[1, 0].text(
        0.5, 0.5, 'RLS error columns not available in this CSV',
        ha='center', va='center')
    axs2[1, 0].set_axis_off()

# --- 2行目右: 共分散行列の対角成分 P_ii の個別推移 ---
p_diag_columns = [column for column in df.columns if column.startswith('rls_P_')]
if p_diag_columns:
    for column in p_diag_columns:
        theta_key = 'rls_theta_' + column[len('rls_P_'):]
        theta_label = param_label_map.get(theta_key, column)
        p_label = theta_label.replace(r'\hat{\theta}', 'P') if r'\hat{\theta}' in theta_label else column
        vals = np.maximum(df[column].to_numpy(dtype=float), 1e-12)
        axs2[1, 1].semilogy(df['time'], vals, label=p_label)
    axs2[1, 1].set_title('RLS Covariance Diagonal Elements $P_{ii}$ (Log scale)')
    axs2[1, 1].set_xlabel('Time [s]')
    axs2[1, 1].set_ylabel('$P_{ii}$ (log scale)')
    axs2[1, 1].grid(True, which='both', linestyle=':', alpha=0.7)
    axs2[1, 1].legend(ncol=2, fontsize=8.5)
else:
    axs2[1, 1].text(
        0.5, 0.5, 'RLS covariance diagonal columns are not available in this CSV',
        ha='center', va='center')
    axs2[1, 1].set_axis_off()

plt.tight_layout()
fig2.subplots_adjust(hspace=0.32, wspace=0.22)

# ---------------------------------------------------------
# 4. ソースファイル名に応じた画像保存名を作成 (2枚)
# ---------------------------------------------------------
output_image_path_1 = filename.parent / f"result_plots_{filename.stem}_1.png"
output_image_path_2 = filename.parent / f"result_plots_{filename.stem}_2.png"

# 画像ファイルとして保存
fig1.savefig(output_image_path_1, dpi=300, bbox_inches='tight')
fig2.savefig(output_image_path_2, dpi=300, bbox_inches='tight')
print(f"グラフ画像(1/2)を保存しました: {output_image_path_1}")
print(f"グラフ画像(2/2)を保存しました: {output_image_path_2}")

# 画面表示
plt.show()
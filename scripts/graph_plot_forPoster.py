from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# ---------------------------------------------------------
# 1. データ読み込みと出力ディレクトリの作成
# ---------------------------------------------------------
workspace_dir = Path(__file__).resolve().parents[3]
# プロット対象のCSV指定
filename = workspace_dir / 'data' / 'まともに動かず_result_20261117_001637_steps10000.csv'
df = pd.read_csv(filename, comment='#')

# 保存先ディレクトリ（CSVファイル名に基づいたフォルダを自動作成）
output_dir = filename.parent / f"plots_{filename.stem}"
output_dir.mkdir(parents=True, exist_ok=True)

# 時間刻み dt の計算
dt = np.diff(df['time']).mean()

# ---------------------------------------------------------
# 2. 運動学に基づく機体位置 (x, y) および 正解Path の算出
# ---------------------------------------------------------
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
    dy_dx = (main_amplitude * k1 * np.cos(dz * k1)
             + sub_amplitude * k2 * np.cos(dz * k2))
    theta_p = np.arctan(dy_dx)
    return y_p, theta_p

x = np.zeros(len(df))
y = np.zeros(len(df))

y_init, _ = calc_path_y_and_angle(x[0])
y[0] = y_init + (df.loc[0, 'e_2'] if 'e_2' in df.columns else 0.0)

for i in range(1, len(df)):
    v_xi = df.loc[i, 'v_xi']
    e3 = df.loc[i, 'e_3']

    _, theta_path = calc_path_y_and_angle(x[i - 1])
    psi = theta_path + e3

    dx = v_xi * np.cos(psi) * dt
    dy = v_xi * np.sin(psi) * dt

    x[i] = x[i - 1] + dx
    y[i] = y[i - 1] + dy

y_path, _ = calc_path_y_and_angle(x)

# ---------------------------------------------------------
# 3. グラフ描画用の共通設定・関数定義
# ---------------------------------------------------------
plt.rcParams.update({
    'font.size': 13,
    'axes.titlesize': 14,
    'axes.labelsize': 13,
    'xtick.labelsize': 11,
    'ytick.labelsize': 11,
    'legend.fontsize': 11
})

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

def plot_trajectory(ax):
    """1. 走行経路 (Vehicle Trajectory vs Reference Path)"""
    ax.plot(x, y, label='Vehicle Trajectory', color='blue', linewidth=2)
    ax.plot(x, y_path, label='Reference Path', color='red', linestyle='--', linewidth=2)
    ax.set_title('Vehicle Trajectory vs Reference Path')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.grid(True, linestyle=':', alpha=0.7)
    ax.legend()
    ax.axis('equal')

def plot_rmse(ax):
    """2. 瞬時予測誤差 (Innovation) の移動RMSE"""
    if 'rls_innovation' in df.columns:
        window_size = 200 if len(df) >= 200 else max(1, len(df))
        rmse_series = np.sqrt(
            (df['rls_innovation'] ** 2).rolling(window=window_size, min_periods=1).mean()
        )
        ax.plot(
            df['time'], rmse_series, color='purple', linewidth=2,
            label=f'Innovation Moving RMSE (window={window_size})'
        )
        ax.set_title('RLS Innovation Moving RMSE over Time')
        ax.set_xlabel('Time [s]')
        ax.set_ylabel('RMSE [rad]')
        ax.set_ylim(0, 1)
        ax.grid(True, linestyle=':', alpha=0.7)
        ax.legend()
    else:
        ax.text(
            0.5, 0.5, 'rls_innovation column is not available in this CSV',
            ha='center', va='center'
        )
        ax.set_axis_off()

def plot_parameters(ax):
    """3. RLS各パラメータの推移"""
    theta_columns = [col for col in df.columns if col.startswith('rls_theta_')]
    if theta_columns:
        for col in theta_columns:
            display_label = param_label_map.get(col, col)
            ax.plot(df['time'], df[col], label=display_label, linewidth=1.8)
        ax.set_title('RLS Parameter Changes')
        ax.set_xlabel('Time [s]')
        ax.set_ylabel(r'$\hat{\theta}$')
        ax.grid(True, linestyle=':', alpha=0.7)
        ax.legend(ncol=2, fontsize=10)
    else:
        ax.text(
            0.5, 0.5, 'RLS parameter columns are not available in this CSV',
            ha='center', va='center'
        )
        ax.set_axis_off()

# ---------------------------------------------------------
# 4. 4:3 と 16:9 の画像を生成してディレクトリに一括保存
# ---------------------------------------------------------
aspect_ratios = {
    '4x3': (12, 9),
    '16x9': (16, 9),
}

graphs = [
    ('trajectory', plot_trajectory),
    ('rmse', plot_rmse),
    ('parameters', plot_parameters),
]

for graph_name, plot_func in graphs:
    for ratio_name, figsize in aspect_ratios.items():
        fig, ax = plt.subplots(figsize=figsize)
        plot_func(ax)
        plt.tight_layout()

        save_path = output_dir / f"{graph_name}_{ratio_name}.png"
        fig.savefig(save_path, dpi=300, bbox_inches='tight')
        plt.close(fig)  # メモリ解放のためクローズ
        print(f"保存完了: {save_path}")

print(f"\nすべての画像をディレクトリに保存しました: {output_dir}")
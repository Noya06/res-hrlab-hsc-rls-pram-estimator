from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# ---------------------------------------------------------
# 1. データ読み込みと出力ディレクトリの作成
# ---------------------------------------------------------
workspace_dir = Path(__file__).resolve().parents[3]
# プロット対象のCSV指定（ファイルが存在しない場合は最新のCSVを自動選択）
filename = workspace_dir / 'data' / 'hum_result_20261119_174407_steps10000.csv'
if not filename.exists():
    candidates = sorted((workspace_dir / 'data').glob('*.csv'))
    if candidates:
        filename = candidates[-1]

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
# 3. グラフ描画用の共通設定・関数定義（ポスター用：文字と凡例をさらに拡大）
# ---------------------------------------------------------
plt.rcParams.update({
    'font.size': 28,
    'axes.labelsize': 34,
    'xtick.labelsize': 28,
    'ytick.labelsize': 28,
    'legend.fontsize': 26,
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

def calc_trajectory_from_csv(csv_path: Path):
    """CSVから運動学に基づく機体位置 (x, y, 最初の50s) を算出する"""
    df_item = pd.read_csv(csv_path, comment='#')
    dt_item = np.diff(df_item['time']).mean()
    time_elapsed = df_item['time'] - df_item['time'].iloc[0]
    mask = time_elapsed <= 50.0

    x_arr = np.zeros(len(df_item))
    y_arr = np.zeros(len(df_item))
    y_init_item, _ = calc_path_y_and_angle(x_arr[0])
    y_arr[0] = y_init_item + (df_item.loc[0, 'e_2'] if 'e_2' in df_item.columns else 0.0)

    for i in range(1, len(df_item)):
        v_xi = df_item.loc[i, 'v_xi'] if 'v_xi' in df_item.columns else 0.0
        e3 = df_item.loc[i, 'e_3'] if 'e_3' in df_item.columns else 0.0
        _, theta_p = calc_path_y_and_angle(x_arr[i - 1])
        psi = theta_p + e3
        dx = v_xi * np.cos(psi) * dt_item
        dy = v_xi * np.sin(psi) * dt_item
        x_arr[i] = x_arr[i - 1] + dx
        y_arr[i] = y_arr[i - 1] + dy

    return x_arr[mask], y_arr[mask]

def get_path_label_and_color(stem: str) -> tuple[str, str]:
    """CSVファイル名から凡例ラベルと色を取得"""
    if 'no_result' in stem:
        return 'Base Controller', 'black'
    elif 'tuned' in stem:
        return 'Tuned Controller', '#009B9F'
    elif 'hum' in stem:
        return 'Human', '#006699'
    return stem, 'purple'

def plot_trajectory(ax):
    """1. 走行経路 (Vehicle Trajectory vs Reference Path, はじめの50s分)"""
    mask_50s = (df['time'] - df['time'].iloc[0]) <= 50.0
    x_50s = x[mask_50s]
    y_50s = y[mask_50s]
    y_path_50s = y_path[mask_50s]

    label, color = get_path_label_and_color(filename.stem)
    ax.plot(x_50s, y_path_50s, label='Reference Path', color='gray', linestyle='--', linewidth=3.5, zorder=1)
    ax.plot(x_50s, y_50s, label=label, color=color, linewidth=3.5, zorder=2)

    # グラフタイトルはなし
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.grid(True, linestyle=':', alpha=0.7)
    ax.legend(fontsize=26)
    ax.axis('equal')

def plot_combined_trajectory(ax):
    """2. すべてのCSVの走行経路を合成プロット (はじめの50s分)"""
    csv_files = sorted((workspace_dir / 'data').glob('*.csv'))

    ref_plotted = False
    for i, csv_file in enumerate(csv_files):
        try:
            x_item, y_item = calc_trajectory_from_csv(csv_file)
            if len(x_item) == 0:
                continue

            if not ref_plotted:
                y_ref, _ = calc_path_y_and_angle(x_item)
                ax.plot(x_item, y_ref, label='Reference Path', color='gray', linestyle='--', linewidth=3.5, zorder=1)
                ref_plotted = True

            label, color = get_path_label_and_color(csv_file.stem)
            ax.plot(x_item, y_item, label=label, color=color, linewidth=3.5, alpha=0.85, zorder=2 + i)
        except Exception as e:
            print(f"[Warning] 合成プロット失敗 ({csv_file.name}): {e}")

    # グラフタイトルはなし
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.grid(True, linestyle=':', alpha=0.7)
    ax.legend(fontsize=24, loc='best')
    ax.axis('equal')

def plot_rmse(ax):
    """3. 予測誤差の累積RMSE"""
    has_rmse = 'rls_rmse' in df.columns
    has_innovation = 'rls_innovation' in df.columns

    if has_rmse or has_innovation:
        if has_rmse:
            rmse_series = df['rls_rmse'].copy()
        else:
            cumsum = (df['rls_innovation'] ** 2).cumsum()
            count = np.arange(1, len(df) + 1)
            rmse_series = np.sqrt(cumsum / count)

        # パラメータが固定化された以降はRMSEも固定値として保持
        theta_columns = [col for col in df.columns if col.startswith('rls_theta_')]
        if theta_columns:
            diff = df[theta_columns].diff().abs().sum(axis=1)
            changes = diff > 1e-9
            if changes.any():
                last_update_idx = changes[changes].index[-1]
                rmse_series.iloc[last_update_idx:] = rmse_series.iloc[last_update_idx]

        ax.plot(
            df['time'], rmse_series, color='blue', linewidth=3.5,
            label=r'Cumulative RMSE ($\sqrt{\frac{1}{N}\sum (y-\xi^T\theta)^2}$)'
        )
        # グラフタイトルはなし
        ax.set_xlabel('Time [s]')
        ax.set_ylabel('RMSE [rad]')
        ax.set_xlim(0, 60)
        ax.set_ylim(0, 1)
        ax.grid(True, linestyle=':', alpha=0.7)
        ax.legend(fontsize=24)
    else:
        ax.text(
            0.5, 0.5, 'RLS error columns are not available in this CSV',
            ha='center', va='center', fontsize=24
        )
        ax.set_axis_off()

def plot_parameters(ax):
    """4. RLS各パラメータの推移"""
    theta_columns = [col for col in df.columns if col.startswith('rls_theta_')]
    if theta_columns:
        for col in theta_columns:
            display_label = param_label_map.get(col, col)
            ax.plot(df['time'], df[col], label=display_label, linewidth=2.8)
        # グラフタイトルはなし
        ax.set_xlabel('Time [s]')
        ax.set_ylabel(r'$\hat{\theta}$')
        ax.set_xlim(0, 60)
        ax.grid(True, linestyle=':', alpha=0.7)
        ax.legend(loc='upper right', ncol=2, fontsize=21)
    else:
        ax.text(
            0.5, 0.5, 'RLS parameter columns are not available in this CSV',
            ha='center', va='center', fontsize=24
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
    ('combined_trajectory', plot_combined_trajectory),
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
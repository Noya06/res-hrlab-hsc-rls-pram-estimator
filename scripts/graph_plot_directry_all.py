#!/usr/bin/env python3
"""
/data/datafiles ディレクトリ内のすべてのCSVファイルを一括でグラフ化し、
画像(PNG)として保存するスクリプト。
"""

import argparse
from pathlib import Path
import sys

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# Unityの指定パラメータ設定 (SplinePathGenerator.cs と同じ計算式)
START_Z = -1.0
MAIN_AMPLITUDE = 1.0
MAIN_WAVELENGTH = 20.0
SUB_AMPLITUDE = 1.0
SUB_WAVELENGTH = 10.0

K1 = 2.0 * np.pi / MAIN_WAVELENGTH
K2 = 2.0 * np.pi / SUB_WAVELENGTH


def calc_path_y_and_angle(x_val):
    dz = x_val - START_Z
    y_p = MAIN_AMPLITUDE * np.sin(dz * K1) + SUB_AMPLITUDE * np.sin(dz * K2)
    dy_dx = (MAIN_AMPLITUDE * K1 * np.cos(dz * K1)
             + SUB_AMPLITUDE * K2 * np.cos(dz * K2))
    theta_p = np.arctan(dy_dx)
    return y_p, theta_p


PARAM_LABEL_MAP = {
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


def plot_single_csv(csv_path: Path, output_image_path: Path = None, show: bool = False) -> bool:
    """単一のCSVファイルをグラフ化して保存する"""
    try:
        df = pd.read_csv(csv_path, comment='#')
    except Exception as e:
        print(f"[Error] CSV読み込み失敗: {csv_path.name} ({e})")
        return False

    if 'time' not in df.columns or len(df) < 2:
        print(f"[Skip] 有効なデータ行または time 列がありません: {csv_path.name}")
        return False

    dt = np.diff(df['time']).mean()

    # 運動学に基づく機体位置 (x, y) および 正解Path の算出
    x = np.zeros(len(df))
    y = np.zeros(len(df))
    y_init, _ = calc_path_y_and_angle(x[0])
    y[0] = y_init + (df.loc[0, 'e_2'] if 'e_2' in df.columns else 0.0)

    for i in range(1, len(df)):
        v_xi = df.loc[i, 'v_xi'] if 'v_xi' in df.columns else 0.0
        e3 = df.loc[i, 'e_3'] if 'e_3' in df.columns else 0.0
        _, theta_path = calc_path_y_and_angle(x[i - 1])
        psi = theta_path + e3
        dx = v_xi * np.cos(psi) * dt
        dy = v_xi * np.sin(psi) * dt
        x[i] = x[i - 1] + dx
        y[i] = y[i - 1] + dy

    y_path, _ = calc_path_y_and_angle(x)

    # グラフ描画設定
    plt.rcParams.update({
        'font.size': 12,
        'axes.titlesize': 11,
        'axes.labelsize': 12,
        'xtick.labelsize': 10,
        'ytick.labelsize': 10,
        'legend.fontsize': 10
    })

    fig, axs = plt.subplots(5, 2, figsize=(15, 25))
    fig.suptitle(f"Dataset: {csv_path.name}", fontsize=14, y=0.995)

    # 1行目左: Vehicle Trajectory
    axs[0, 0].plot(x, y, label='Vehicle Trajectory', color='blue', linewidth=2)
    axs[0, 0].plot(x, y_path, label='Reference Path', color='red', linestyle='--', linewidth=2)
    axs[0, 0].set_title('Vehicle Trajectory vs Reference Path')
    axs[0, 0].set_xlabel('x [m]')
    axs[0, 0].set_ylabel('y [m]')
    axs[0, 0].grid(True, linestyle=':', alpha=0.7)
    axs[0, 0].legend()
    axs[0, 0].axis('equal')

    # 1行目右: e_2
    if 'e_2' in df.columns:
        axs[0, 1].plot(df['time'], df['e_2'], color='darkorange', linewidth=2)
        axs[0, 1].set_title('Lateral Error $e_2$ over Time')
        axs[0, 1].set_xlabel('Time [s]')
        axs[0, 1].set_ylabel('$e_2$ [m]')
        axs[0, 1].grid(True, linestyle=':', alpha=0.7)
    else:
        axs[0, 1].set_axis_off()

    # 2行目左: e_3
    if 'e_3' in df.columns:
        axs[1, 0].plot(df['time'], df['e_3'], color='green', linewidth=2)
        axs[1, 0].set_title('Orientation Error $e_3$ over Time')
        axs[1, 0].set_xlabel('Time [s]')
        axs[1, 0].set_ylabel('$e_3$ [rad]')
        axs[1, 0].grid(True, linestyle=':', alpha=0.7)
    else:
        axs[1, 0].set_axis_off()

    # 2行目右: IOC/RLS signals
    has_ioc_rls = False
    if 'phi_ioc' in df.columns:
        axs[1, 1].plot(df['time'], df['phi_ioc'], color='purple', linewidth=2, label=r'$\phi_{ioc}$')
        has_ioc_rls = True
    if 'rls_prediction' in df.columns:
        axs[1, 1].plot(df['time'], df['rls_prediction'], color='teal', linewidth=2, label='RLS prediction')
        has_ioc_rls = True
    if 'target_roll' in df.columns:
        axs[1, 1].plot(df['time'], df['target_roll'], color='black', linestyle='--', linewidth=2, label=r'$\phi_{ioc}$ + RLS')
        has_ioc_rls = True

    if has_ioc_rls:
        axs[1, 1].set_title('IOC and RLS Control Signals over Time')
        axs[1, 1].set_xlabel('Time [s]')
        axs[1, 1].set_ylabel('Control input')
        axs[1, 1].grid(True, linestyle=':', alpha=0.7)
        axs[1, 1].legend()
    else:
        axs[1, 1].set_axis_off()

    # 3行目左: phi_hum & target_roll
    if {'phi_hum', 'target_roll'}.issubset(df.columns):
        axs[2, 0].plot(df['time'], df['phi_hum'], label=r'$\phi_{hum}$')
        axs[2, 0].plot(df['time'], df['target_roll'], linestyle='--', label=r'$\mathrm{target\_roll}$')
        axs[2, 0].set_title('Human angle and target roll')
        axs[2, 0].set_xlabel('Time [s]')
        axs[2, 0].set_ylabel('Value')
        axs[2, 0].grid(True, linestyle=':', alpha=0.7)
        axs[2, 0].legend()
    else:
        axs[2, 0].text(0.5, 0.5, 'Required columns not found', ha='center', va='center')
        axs[2, 0].set_axis_off()

    # 3行目右: RLS parameter changes
    theta_columns = [col for col in df.columns if col.startswith('rls_theta_')]
    if theta_columns:
        for col in theta_columns:
            display_label = PARAM_LABEL_MAP.get(col, col)
            axs[2, 1].plot(df['time'], df[col], label=display_label)
        axs[2, 1].set_title('RLS parameter changes')
        axs[2, 1].set_xlabel('Time [s]')
        axs[2, 1].set_ylabel(r'$\hat{\theta}$')
        axs[2, 1].grid(True, linestyle=':', alpha=0.7)
        axs[2, 1].legend(ncol=2, fontsize=9)
    else:
        axs[2, 1].text(0.5, 0.5, 'RLS parameters not found', ha='center', va='center')
        axs[2, 1].set_axis_off()

    # 4行目左: phi_hum - target_roll
    if {'phi_hum', 'target_roll'}.issubset(df.columns):
        diff_roll = df['phi_hum'] - df['target_roll']
        axs[3, 0].plot(df['time'], diff_roll, color='crimson', linewidth=2, label=r'$\phi_{hum} - \mathrm{target\_roll}$')
        axs[3, 0].axhline(0, color='gray', linestyle=':', linewidth=1)
        axs[3, 0].set_title('Difference: phi_hum - target_roll')
        axs[3, 0].set_xlabel('Time [s]')
        axs[3, 0].set_ylabel('Value')
        axs[3, 0].grid(True, linestyle=':', alpha=0.7)
        axs[3, 0].legend()
    else:
        axs[3, 0].text(0.5, 0.5, 'Required columns not found', ha='center', va='center')
        axs[3, 0].set_axis_off()

    # 4行目右: 非線形誤差項
    has_nl = False
    if 'e_2' in df.columns and 'e_3' in df.columns:
        e2_abs_e2 = df['e_2_abs_e_2'] if 'e_2_abs_e_2' in df.columns else df['e_2'] * df['e_2'].abs()
        e3_abs_e3 = df['e_3_abs_e_3'] if 'e_3_abs_e_3' in df.columns else df['e_3'] * df['e_3'].abs()
        e2_abs_e3 = df['e_2_abs_e_3'] if 'e_2_abs_e_3' in df.columns else df['e_2'] * df['e_3'].abs()
        abs_e2_e3 = df['abs_e_2_e_3'] if 'abs_e_2_e_3' in df.columns else df['e_2'].abs() * df['e_3']

        axs[3, 1].plot(df['time'], e2_abs_e2, color='darkorange', linewidth=2, label=r'$e_2|e_2|$')
        axs[3, 1].plot(df['time'], e3_abs_e3, color='green', linestyle='--', linewidth=2, label=r'$e_3|e_3|$')
        axs[3, 1].plot(df['time'], e2_abs_e3, color='royalblue', linewidth=2, label=r'$e_2|e_3|$')
        axs[3, 1].plot(df['time'], abs_e2_e3, color='mediumorchid', linestyle='--', linewidth=2, label=r'$|e_2|e_3$')
        has_nl = True

    if 'rho' in df.columns:
        axs[3, 1].plot(df['time'], df['rho'], color='black', linestyle=':', linewidth=2, label=r'$\rho$')
        has_nl = True
    if 'e_2_delay' in df.columns:
        axs[3, 1].plot(df['time'], df['e_2_delay'], color='saddlebrown', linestyle='-.', linewidth=1.5, label=r'$e_2(t-L)$')
        has_nl = True
    if 'e_3_delay' in df.columns:
        axs[3, 1].plot(df['time'], df['e_3_delay'], color='darkcyan', linestyle='-.', linewidth=1.5, label=r'$e_3(t-L)$')
        has_nl = True

    if has_nl:
        axs[3, 1].axhline(0, color='gray', linestyle=':', linewidth=1)
        axs[3, 1].set_title('Nonlinear / Delayed Error Terms over Time')
        axs[3, 1].set_xlabel('Time [s]')
        axs[3, 1].set_ylabel('Value')
        axs[3, 1].grid(True, linestyle=':', alpha=0.7)
        axs[3, 1].legend(ncol=2)
    else:
        axs[3, 1].set_axis_off()

    # --- 5行目左: RLS 共分散トレース Tr(P) & コスト J - time ---
    has_p_cost = False
    if 'rls_trace_P' in df.columns:
        axs[4, 0].semilogy(df['time'], df['rls_trace_P'], color='blue', linewidth=2, label=r'$\mathrm{Tr}(P)$ (Uncertainty)')
        has_p_cost = True
    if 'rls_cost' in df.columns:
        axs[4, 0].semilogy(df['time'], df['rls_cost'], color='purple', linestyle='--', linewidth=2, label=r'Cost $J(t)$')
        has_p_cost = True

    if has_p_cost:
        axs[4, 0].set_title('RLS Covariance Trace $\mathrm{Tr}(P)$ and Cost $J(t)$ (Log scale)')
        axs[4, 0].set_xlabel('Time [s]')
        axs[4, 0].set_ylabel('Value (log scale)')
        axs[4, 0].grid(True, which="both", linestyle=':', alpha=0.7)
        axs[4, 0].legend()
    else:
        axs[4, 0].text(
            0.5, 0.5, 'RLS trace_P / cost columns not available',
            ha='center', va='center')
        axs[4, 0].set_axis_off()

    # --- 5行目右: RLS カルマンゲインノルム ||K|| - time ---
    if 'rls_gain_norm' in df.columns:
        axs[4, 1].plot(df['time'], df['rls_gain_norm'], color='crimson', linewidth=2, label=r'$\|K\|$ (Learning Sensitivity)')
        axs[4, 1].set_title('RLS Kalman Gain Norm $\|K\|$ over Time')
        axs[4, 1].set_xlabel('Time [s]')
        axs[4, 1].set_ylabel(r'$\|K\|$')
        axs[4, 1].grid(True, linestyle=':', alpha=0.7)
        axs[4, 1].legend()
    else:
        axs[4, 1].text(
            0.5, 0.5, 'RLS gain_norm column not available',
            ha='center', va='center')
        axs[4, 1].set_axis_off()

    plt.tight_layout()
    fig.subplots_adjust(top=0.96, hspace=0.35)

    if output_image_path is None:
        output_image_path = csv_path.parent / f"result_plots_{csv_path.stem}.png"

    plt.savefig(output_image_path, dpi=300, bbox_inches='tight')
    print(f"[Success] 保存完了: {output_image_path}")

    if show:
        plt.show()
    else:
        plt.close(fig)

    return True


def main():
    parser = argparse.ArgumentParser(
        description="datafiles ディレクトリ内のすべてのCSVファイルをグラフ化して保存します"
    )
    workspace_dir = Path(__file__).resolve().parents[3]
    default_dir = workspace_dir / 'data' / 'datafiles'

    parser.add_argument(
        '--dir',
        type=Path,
        default=default_dir,
        help=f"対象ディレクトリ (デフォルト: {default_dir})"
    )
    parser.add_argument(
        '--show',
        action='store_true',
        help="グラフ生成時にウィンドウ表示も行うか"
    )

    args = parser.parse_args()
    target_dir = args.dir

    if not target_dir.exists() or not target_dir.is_dir():
        print(f"[Error] 指定されたディレクトリが存在しません: {target_dir}")
        sys.exit(1)

    csv_files = sorted(list(target_dir.glob('*.csv')))
    if not csv_files:
        print(f"[Info] 対象ディレクトリ内にCSVファイルが見つかりませんでした: {target_dir}")
        return

    print(f"=== 合計 {len(csv_files)} 件のCSVファイルを処理します ===")
    print(f"ディレクトリ: {target_dir}")

    success_count = 0
    for idx, csv_path in enumerate(csv_files, start=1):
        print(f"\n[{idx}/{len(csv_files)}] 処理中: {csv_path.name}")
        if plot_single_csv(csv_path, show=args.show):
            success_count += 1

    print("\n" + "=" * 50)
    print(f"=== 完了: {success_count} / {len(csv_files)} 件の画像を生成しました ===")
    print("=" * 50)


if __name__ == '__main__':
    main()

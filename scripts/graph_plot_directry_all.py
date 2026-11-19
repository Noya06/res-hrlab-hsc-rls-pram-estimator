#!/usr/bin/env python3
"""
/data/datafiles ディレクトリ内のすべてのCSVファイルを一括でグラフ化し、
4グラフずつ2枚の画像(PNG)として保存するスクリプト。
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
    """単一のCSVファイルを4グラフずつ2枚の画像にして保存する"""
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
        'font.size': 11,
        'axes.titlesize': 11,
        'axes.labelsize': 11,
        'xtick.labelsize': 10,
        'ytick.labelsize': 10,
        'legend.fontsize': 9.5
    })

    # =========================================================
    # 1枚目: 軌道・追従誤差・制御信号 (2行 x 2列, 16:9)
    # =========================================================
    fig1, axs1 = plt.subplots(2, 2, figsize=(16, 9))
    fig1.suptitle(f"Dataset: {csv_path.name} (1/2: Trajectory & Control)", fontsize=13, y=0.98)

    # 1行目左: Vehicle Trajectory
    axs1[0, 0].plot(x, y, label='Vehicle Trajectory', color='blue', linewidth=2)
    axs1[0, 0].plot(x, y_path, label='Reference Path', color='red', linestyle='--', linewidth=2)
    axs1[0, 0].set_title('Vehicle Trajectory vs Reference Path')
    axs1[0, 0].set_xlabel('x [m]')
    axs1[0, 0].set_ylabel('y [m]')
    axs1[0, 0].grid(True, linestyle=':', alpha=0.7)
    axs1[0, 0].legend()
    axs1[0, 0].axis('equal')

    # 1行目右: e_2
    if 'e_2' in df.columns:
        axs1[0, 1].plot(df['time'], df['e_2'], color='darkorange', linewidth=2)
        axs1[0, 1].set_title('Lateral Error $e_2$ over Time')
        axs1[0, 1].set_xlabel('Time [s]')
        axs1[0, 1].set_ylabel('$e_2$ [m]')
        axs1[0, 1].grid(True, linestyle=':', alpha=0.7)
    else:
        axs1[0, 1].set_axis_off()

    # 2行目左: e_3
    if 'e_3' in df.columns:
        axs1[1, 0].plot(df['time'], df['e_3'], color='green', linewidth=2)
        axs1[1, 0].set_title('Orientation Error $e_3$ over Time')
        axs1[1, 0].set_xlabel('Time [s]')
        axs1[1, 0].set_ylabel('$e_3$ [rad]')
        axs1[1, 0].grid(True, linestyle=':', alpha=0.7)
    else:
        axs1[1, 0].set_axis_off()

    # 2行目右: IOC/RLS signals
    has_ioc_rls = False
    if 'phi_ioc' in df.columns:
        axs1[1, 1].plot(df['time'], df['phi_ioc'], color='purple', linewidth=2, label=r'$\phi_{ioc}$')
        has_ioc_rls = True
    if 'rls_prediction' in df.columns:
        axs1[1, 1].plot(df['time'], df['rls_prediction'], color='teal', linewidth=2, label='RLS prediction')
        has_ioc_rls = True
    if 'target_roll' in df.columns:
        axs1[1, 1].plot(df['time'], df['target_roll'], color='black', linestyle='--', linewidth=2, label=r'$\phi_{ioc}$ + RLS')
        has_ioc_rls = True

    if has_ioc_rls:
        axs1[1, 1].set_title('IOC and RLS Control Signals over Time')
        axs1[1, 1].set_xlabel('Time [s]')
        axs1[1, 1].set_ylabel('Control input')
        axs1[1, 1].set_ylim(-1, 1)
        axs1[1, 1].grid(True, linestyle=':', alpha=0.7)
        axs1[1, 1].legend()
    else:
        axs1[1, 1].set_axis_off()

    plt.tight_layout()
    fig1.subplots_adjust(top=0.91, hspace=0.32, wspace=0.22)

    # =========================================================
    # 2枚目: 人間入力比較 & RLS詳細 (2行 x 2列, 16:9)
    # =========================================================
    fig2, axs2 = plt.subplots(2, 2, figsize=(16, 9))
    fig2.suptitle(f"Dataset: {csv_path.name} (2/2: Human Roll & RLS Diagnostics)", fontsize=13, y=0.98)

    # 1行目左: human angle and inverted target roll
    if {'phi_hum', 'target_roll'}.issubset(df.columns):
        axs2[0, 0].plot(df['time'], df['phi_hum'], label=r'$\phi_{hum}$')
        axs2[0, 0].plot(df['time'], -df['target_roll'], linestyle='--', label='-target_roll')
        axs2[0, 0].set_title('Human angle and inverted target roll')
        axs2[0, 0].set_xlabel('Time [s]')
        axs2[0, 0].set_ylabel('Value')
        axs2[0, 0].grid(True, linestyle=':', alpha=0.7)
        axs2[0, 0].legend()
    else:
        axs2[0, 0].text(0.5, 0.5, 'Required columns not found', ha='center', va='center')
        axs2[0, 0].set_axis_off()

    # 1行目右: RLS parameter changes
    theta_columns = [col for col in df.columns if col.startswith('rls_theta_')]
    if theta_columns:
        for col in theta_columns:
            display_label = PARAM_LABEL_MAP.get(col, col)
            axs2[0, 1].plot(df['time'], df[col], label=display_label)
        axs2[0, 1].set_title('RLS parameter changes')
        axs2[0, 1].set_xlabel('Time [s]')
        axs2[0, 1].set_ylabel(r'$\hat{\theta}$')
        axs2[0, 1].grid(True, linestyle=':', alpha=0.7)
        axs2[0, 1].legend(ncol=2, fontsize=8.5)
    else:
        axs2[0, 1].text(0.5, 0.5, 'RLS parameters not found', ha='center', va='center')
        axs2[0, 1].set_axis_off()

    # 2行目左: 予測誤差 (Innovation) と RMSE
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
            0.5, 0.5, 'RLS error columns not available',
            ha='center', va='center')
        axs2[1, 0].set_axis_off()

    # 2行目右: 共分散行列の対角成分 P_ii の個別推移
    p_diag_columns = [col for col in df.columns if col.startswith('rls_P_')]
    if p_diag_columns:
        for col in p_diag_columns:
            theta_key = 'rls_theta_' + col[len('rls_P_'):]
            theta_label = PARAM_LABEL_MAP.get(theta_key, col)
            p_label = theta_label.replace(r'\hat{\theta}', 'P') if r'\hat{\theta}' in theta_label else col
            vals = np.maximum(df[col].to_numpy(dtype=float), 1e-12)
            axs2[1, 1].semilogy(df['time'], vals, label=p_label)
        axs2[1, 1].set_title('RLS Covariance Diagonal Elements $P_{ii}$ (Log scale)')
        axs2[1, 1].set_xlabel('Time [s]')
        axs2[1, 1].set_ylabel('$P_{ii}$ (log scale)')
        axs2[1, 1].grid(True, which='both', linestyle=':', alpha=0.7)
        axs2[1, 1].legend(ncol=2, fontsize=8.5)
    else:
        axs2[1, 1].text(
            0.5, 0.5, 'RLS covariance diagonal columns not available',
            ha='center', va='center')
        axs2[1, 1].set_axis_off()

    plt.tight_layout()
    fig2.subplots_adjust(top=0.91, hspace=0.32, wspace=0.22)

    # 保存パスの決定
    if output_image_path is None:
        base_dir = csv_path.parent
        base_stem = f"result_plots_{csv_path.stem}"
        suffix = ".png"
    else:
        base_dir = output_image_path.parent
        base_stem = output_image_path.stem
        suffix = output_image_path.suffix or ".png"

    out_path_1 = base_dir / f"{base_stem}_1{suffix}"
    out_path_2 = base_dir / f"{base_stem}_2{suffix}"

    fig1.savefig(out_path_1, dpi=300, bbox_inches='tight')
    fig2.savefig(out_path_2, dpi=300, bbox_inches='tight')
    print(f"[Success] 保存完了 (1/2): {out_path_1}")
    print(f"[Success] 保存完了 (2/2): {out_path_2}")

    if show:
        plt.show()
    else:
        plt.close(fig1)
        plt.close(fig2)

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
    print(f"=== 完了: {success_count} / {len(csv_files)} 件のCSVについて各2枚の画像を生成しました ===")
    print("=" * 50)


if __name__ == '__main__':
    main()
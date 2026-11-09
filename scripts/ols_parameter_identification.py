#!/usr/bin/env python3
"""Identify configured k2 and k3 with batch least squares.

The ROS2 controller calculates

    phi_com_y = k2 * e2 + k3 * e3

and this script estimates [k2, k3] directly from the recorded
``phi_com_y`` column.
"""

from __future__ import annotations

import argparse
import csv
from datetime import datetime
from pathlib import Path
import re
import numpy as np


# コードから使用する入力CSVを変更する場合は、このファイル名を変更する。
# __file__基準にすることで、実行時のカレントディレクトリに依存しない。
DEFAULT_INPUT_CSV = (
    Path(__file__).resolve().parents[3] / "data" / "ref_10.0_1.0.csv"
)


def read_error_csv(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Read e2/e3 and both control signals from a ROS2 logger CSV."""
    with path.open(newline="", encoding="utf-8") as file:
        rows = [row for row in csv.reader(file) if row and not row[0].lstrip().startswith("#")]

    if not rows:
        raise ValueError(f"CSVにデータがありません: {path}")

    header = [column.strip() for column in rows[0]]
    indices = {}
    for name in ("e2", "e_2", "e3", "e_3"):
        if name in header:
            indices[name] = header.index(name)

    e2_index = indices.get("e2", indices.get("e_2"))
    e3_index = indices.get("e3", indices.get("e_3"))
    u_hum_index = next(
        (header.index(name) for name in ("phi_hum_y", "U_hum", "u_hum") if name in header),
        None,
    )
    u_hsc_index = next(
        (header.index(name) for name in ("phi_com_y", "U_hsc", "u_hsc") if name in header),
        None,
    )
    if e2_index is None or e3_index is None or u_hsc_index is None or u_hum_index is None:
        raise ValueError(
            "CSVには e2/e_2、e3/e_3、phi_com_y/U_hsc、phi_hum_y/U_hum の列が必要です。"
        )

    try:
        values = np.asarray(
            [
                [
                    float(row[e2_index]),
                    float(row[e3_index]),
                    float(row[u_hsc_index]),
                    float(row[u_hum_index]),
                ]
                for row in rows[1:]
            ],
            dtype=float,
        )
    except (IndexError, ValueError) as error:
        raise ValueError(f"e2/e3列の数値を読み取れません: {path}") from error

    if values.size == 0:
        raise ValueError(f"CSVにデータ行がありません: {path}")
    if not np.all(np.isfinite(values)):
        raise ValueError("e2/e3に有限でない値が含まれています。")
    return values[:, 0], values[:, 1], values[:, 2], values[:, 3]


def read_true_parameters_from_filename(path: Path) -> tuple[float, float]:
    """Extract k2 and k3 from a filename in the form ref_K2_K3.csv."""
    match = re.fullmatch(r"ref_([^_]+)_([^_]+)\.csv", path.name)
    if match is None:
        raise ValueError(
            f"CSVファイル名は ref_K2_K3.csv の形式にしてください: {path.name}"
        )

    try:
        return float(match.group(1)), float(match.group(2))
    except ValueError as error:
        raise ValueError(
            f"CSVファイル名から正解パラメータを数値として読み取れません: {path.name}"
        ) from error


def generate_demo_errors(steps: int) -> tuple[np.ndarray, np.ndarray]:
    """Create independent, sufficiently varying errors for local verification."""
    time = np.arange(1, steps + 1, dtype=float)
    e2 = 0.8 * np.sin(0.071 * time) + 0.002 * time
    e3 = 0.6 * np.cos(0.113 * time + 0.4) - 0.001 * time
    return e2, e3


def solve_ols(design_matrix: np.ndarray, target: np.ndarray) -> np.ndarray:
    """Solve (X^T X)^-1 X^T Y while checking identifiability."""
    normal_matrix = design_matrix.T @ design_matrix
    if np.linalg.matrix_rank(normal_matrix) < design_matrix.shape[1]:
        raise ValueError("X^T Xが特異です。e2/e3が独立して変動しているか確認してください。")
    # solve() is numerically preferable to explicitly constructing an inverse.
    return np.linalg.solve(normal_matrix, design_matrix.T @ target)


def write_result_csv(
    path: Path,
    source: Path,
    k2_true: float,
    k3_true: float,
    k2_hat: float,
    k3_hat: float,
    e2: np.ndarray,
    e3: np.ndarray,
    observed: np.ndarray,
    prediction: np.ndarray,
) -> None:
    """Write the final parameter estimates and per-sample predictions."""
    path.parent.mkdir(parents=True, exist_ok=True)
    residual = observed - prediction
    rmse = np.sqrt(np.mean(residual ** 2))
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.writer(file)
        writer.writerow(["# source_csv", str(source)])
        writer.writerow(["# k2_true", f"{k2_true:.12g}"])
        writer.writerow(["# k3_true", f"{k3_true:.12g}"])
        writer.writerow(["# k2_estimated", f"{k2_hat:.12g}"])
        writer.writerow(["# k3_estimated", f"{k3_hat:.12g}"])
        writer.writerow(["# k2_error", f"{k2_hat - k2_true:.12g}"])
        writer.writerow(["# k3_error", f"{k3_hat - k3_true:.12g}"])
        writer.writerow(["# rmse", f"{rmse:.12g}"])
        writer.writerow([])
        writer.writerow(
            ["step", "e2", "e3", "phi_com_y_observed", "phi_com_y_predicted", "residual"]
        )
        for step, values in enumerate(
            zip(e2, e3, observed, prediction, residual), start=1
        ):
            writer.writerow([step, *[f"{value:.12g}" for value in values]])


def plot_fit(y: np.ndarray, prediction: np.ndarray, path: Path) -> None:
    """Save an optional measured/generated-vs-predicted fit plot."""
    try:
        import matplotlib.pyplot as plt
    except ImportError as error:
        raise RuntimeError("グラフには matplotlib が必要です。") from error

    path.parent.mkdir(parents=True, exist_ok=True)
    figure, axis = plt.subplots(figsize=(10, 4))
    axis.plot(y, label="y(t): generated", linewidth=1.5)
    axis.plot(prediction, "--", label="y_hat(t): OLS prediction", linewidth=1.2)
    axis.set_xlabel("step")
    axis.set_ylabel("y")
    axis.set_title("OLS parameter identification fit")
    axis.grid(True, alpha=0.3)
    axis.legend()
    figure.tight_layout()
    figure.savefig(path, dpi=150)
    plt.close(figure)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--input-csv",
        type=Path,
        default=DEFAULT_INPUT_CSV,
        help=(
            "ROS2 loggerが出力したe2/e3/phi_com_y/phi_hum_y CSV。"
            f"既定値: {DEFAULT_INPUT_CSV}"
        ),
    )
    parser.add_argument("--steps", type=int, default=500, help="デモデータのステップ数")
    parser.add_argument("--kp", type=float, default=1.0, help="P制御ゲイン Kp")
    parser.add_argument(
        "--output-csv",
        type=Path,
        help=(
            "最終的な推定パラメータと予測値の出力先。"
            "省略時は入力CSVと実行時刻から自動生成します。"
        ),
    )
    parser.add_argument(
        "--plot",
        type=Path,
        help="指定時にフィッティンググラフを保存するPNGパス",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.steps <= 0:
        raise ValueError("--stepsは正の整数で指定してください。")

    e2, e3, u_hsc, _ = read_error_csv(args.input_csv)
    source = str(args.input_csv)
    k2_true, k3_true = read_true_parameters_from_filename(args.input_csv)
    output_csv = args.output_csv
    if output_csv is None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        output_csv = (
            args.input_csv.parent
            / f"{args.input_csv.stem}_parameter_prediction_{timestamp}.csv"
        )
    # 設定された制御ゲインを検証するため、phi_com_yを目的変数にする。
    y = u_hsc

    design_matrix = np.column_stack((e2, e3))
    theta_hat = solve_ols(design_matrix, y)
    prediction = design_matrix @ theta_hat
    errors = theta_hat - np.array([k2_true, k3_true])

    write_result_csv(
        output_csv,
        args.input_csv,
        k2_true,
        k3_true,
        theta_hat[0],
        theta_hat[1],
        e2,
        e3,
        y,
        prediction,
    )
    if args.plot is not None:
        plot_fit(y, prediction, args.plot)

    print(f"データソース: {source}")
    print(f"サンプル数 N: {len(y)}")
    print(f"k2_true = {k2_true:.12g}, k2_hat = {theta_hat[0]:.12g}, 誤差 = {errors[0]:.3e}")
    print(f"k3_true = {k3_true:.12g}, k3_hat = {theta_hat[1]:.12g}, 誤差 = {errors[1]:.3e}")
    print(f"RMSE(phi_com_y - prediction): {np.sqrt(np.mean((y - prediction) ** 2)):.3e}")
    print(f"生成結果CSV: {output_csv}")
    if args.plot is not None:
        print(f"フィッティンググラフ: {args.plot}")


if __name__ == "__main__":
    main()

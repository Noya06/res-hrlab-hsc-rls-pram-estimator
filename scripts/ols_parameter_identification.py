#!/usr/bin/env python3
"""Generate validation data and identify k2, k3 with batch least squares.

The Unity simulation supplies e2 and e3.  This script then applies the
known human-operation model

    U_hum = U_hsc + k2_true * e2 + k3_true * e3

and estimates [k2, k3] from y = U_hum - U_hsc.
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
import numpy as np


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


def write_generated_csv(
    path: Path,
    e2: np.ndarray,
    e3: np.ndarray,
    e1: np.ndarray,
    u_hsc: np.ndarray,
    u_hum: np.ndarray,
    y: np.ndarray,
    prediction: np.ndarray,
) -> None:
    """Persist the complete X/Y data used by the OLS verification."""
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.writer(file)
        writer.writerow(["step", "e1", "e2", "e3", "U_hsc", "U_hum", "y", "y_prediction"])
        for step, values in enumerate(zip(e1, e2, e3, u_hsc, u_hum, y, prediction), start=1):
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
        help="ROS2 loggerが出力したe2/e3/phi_com_y/phi_hum_y CSV。省略時はデモデータを生成します。",
    )
    parser.add_argument("--steps", type=int, default=500, help="デモデータのステップ数")
    parser.add_argument("--kp", type=float, default=1.0, help="P制御ゲイン Kp")
    parser.add_argument("--k2-true", type=float, default=0.5, help="正解パラメータ k2*")
    parser.add_argument("--k3-true", type=float, default=-0.3, help="正解パラメータ k3*")
    parser.add_argument(
        "--output-csv",
        type=Path,
        default=Path("data/ols_identification.csv"),
        help="生成したX/Yと予測値の出力先",
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

    if args.input_csv is None:
        e2, e3 = generate_demo_errors(args.steps)
        source = "デモ生成"
    else:
        e2, e3, u_hsc, u_hum = read_error_csv(args.input_csv)
        source = str(args.input_csv)

    if args.input_csv is None:
        e1 = np.zeros_like(e2)
        u_hsc = args.kp * e1
        u_hum = u_hsc + args.k2_true * e2 + args.k3_true * e3
    else:
        # ROS2側で計算・記録したU_hsc/U_humを使い、PythonではOLSだけを行う。
        e1 = np.zeros_like(e2)
    y = u_hum - u_hsc

    design_matrix = np.column_stack((e2, e3))
    theta_hat = solve_ols(design_matrix, y)
    prediction = design_matrix @ theta_hat
    errors = theta_hat - np.array([args.k2_true, args.k3_true])

    write_generated_csv(args.output_csv, e2, e3, e1, u_hsc, u_hum, y, prediction)
    if args.plot is not None:
        plot_fit(y, prediction, args.plot)

    print(f"データソース: {source}")
    print(f"サンプル数 N: {len(y)}")
    print(f"k2_true = {args.k2_true:.12g}, k2_hat = {theta_hat[0]:.12g}, 誤差 = {errors[0]:.3e}")
    print(f"k3_true = {args.k3_true:.12g}, k3_hat = {theta_hat[1]:.12g}, 誤差 = {errors[1]:.3e}")
    print(f"RMSE(y - y_hat): {np.sqrt(np.mean((y - prediction) ** 2)):.3e}")
    print(f"生成データCSV: {args.output_csv}")
    if args.plot is not None:
        print(f"フィッティンググラフ: {args.plot}")


if __name__ == "__main__":
    main()

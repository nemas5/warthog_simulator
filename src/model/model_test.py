import sys
import argparse
import importlib.util
import os
from pathlib import Path

import torch
import numpy as np
import pandas as pd
import matplotlib

if not os.environ.get("DISPLAY") or importlib.util.find_spec("tkinter") is None:
    matplotlib.use("Agg")

import matplotlib.pyplot as plt
from torchdiffeq import odeint
from sklearn.metrics import mean_absolute_error

MODEL_DIR = Path(__file__).resolve().parent
PACKAGE_DIR = MODEL_DIR.parent
WORKSPACE_DIR = PACKAGE_DIR.parent

sys.path.insert(0, str(MODEL_DIR / "PSPSO" / "build"))
import pspso


class ODEFunc(torch.nn.Module):
    def __init__(self, hidden=16):
        super().__init__()

        self.net = torch.nn.Sequential(
            torch.nn.Linear(4, hidden),
            torch.nn.Tanh(),
            torch.nn.Linear(hidden, 2)
        )

    def forward(self, t, y):
        state_derivative = self.net(y)
        control_derivative = torch.zeros_like(y[:, 2:])
        return torch.cat([state_derivative, control_derivative], dim=-1)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("basis_name")
    parser.add_argument("dataset_name")
    return parser.parse_args()


def is_state_dict(value) -> bool:
    return isinstance(value, dict) and all(torch.is_tensor(v) for v in value.values())


def load_model_from_state_dict(state_dict) -> ODEFunc:
    model = ODEFunc()
    model.load_state_dict(state_dict)
    model.eval()
    return model


def load_checkpoint(model_file: Path):
    try:
        return torch.load(model_file, map_location="cpu", weights_only=True)
    except TypeError:
        return torch.load(model_file, map_location="cpu")


def load_basis_models(basis_name: str):
    basis_dir = MODEL_DIR / "neural_models" / basis_name
    model_files = sorted(
        p for p in basis_dir.iterdir()
        if p.is_file() and p.suffix in {".pt", ".pth"}
    )

    models = []
    for model_file in model_files:
        checkpoint = load_checkpoint(model_file)
        if is_state_dict(checkpoint):
            models.append((model_file.stem, load_model_from_state_dict(checkpoint)))
        else:
            for key, value in checkpoint.items():
                if is_state_dict(value):
                    name = f"{model_file.stem}:{key}"
                    models.append((name, load_model_from_state_dict(value)))

    if not models:
        raise RuntimeError(f"No models loaded from {basis_dir}")

    return models


def dataset_path(dataset_name: str) -> Path:
    name = Path(dataset_name).name
    if not name.endswith(".csv"):
        name = f"{name}.csv"
    return WORKSPACE_DIR / "datasets" / name


def show_or_save(fig, filename: str):
    if matplotlib.get_backend().lower() == "agg":
        output_dir = MODEL_DIR / "results"
        output_dir.mkdir(exist_ok=True)
        output_path = output_dir / filename
        fig.savefig(output_path, dpi=150)
        print(f"Saved plot: {output_path}")
        plt.close(fig)
    else:
        plt.show()


@torch.no_grad()
def get_basis_output(model: ODEFunc, df_slice: pd.DataFrame) -> np.ndarray:
    """
    Для каждой строки df_slice с полями
    odom_vx, odom_wz, cmd_vx, cmd_wz, dt
    интегрирует ODE на [0, dt] и возвращает предсказанное следующее состояние.
    """
    outputs = []
    for _, row in df_slice.iterrows():
        y0 = torch.tensor([[
            row["odom_vx"],
            row["odom_wz"],
            row["cmd_vx"],
            row["cmd_wz"],
        ]], dtype=torch.float32)

        t = torch.tensor([0.0, float(row["dt"])], dtype=torch.float32)
        traj = odeint(model, y0, t)          # shape: [2, 1, 4]
        y_next = traj[-1, :, :2].squeeze(0).cpu().numpy()  # [2]
        outputs.append(y_next)

    return np.vstack(outputs)                # shape: (N, 2)


args = parse_args()
models = load_basis_models(args.basis_name)
print("Loaded models:")
for model_idx, (model_name, _) in enumerate(models):
    print(f"  {model_idx}: {model_name}")

df = pd.read_csv(dataset_path(args.dataset_name))

df_cur  = df.iloc[:-1].reset_index(drop=True)
df_next = df.iloc[1:].reset_index(drop=True)

f_target = df_next[["odom_vx", "odom_wz"]].values  # shape (N, 2)
basis_outputs = [get_basis_output(model, df_cur) for _, model in models]
basis_values = np.stack(basis_outputs, axis=1)      # shape: (N, models, 2)


subswarms   = 5
particles   = 10
cognitive   = 2
social      = 2
perturb     = 0.002
max_ops     = 50
dimensions  = len(models) # по одному коэффициенту на модель базиса
start_dyn_i = 0           # индекс стартовой базисной функции (не критично здесь)

# NOTE: два независимых экземпляра — чтобы каналы vx и wz не мешали друг другу
pso_vx = pspso.PSO(subswarms, particles, cognitive, social,
                   perturb, max_ops, dimensions, start_dyn_i)
pso_wz = pspso.PSO(subswarms, particles, cognitive, social,
                   perturb, max_ops, dimensions, start_dyn_i)

N = len(df_cur)
pred       = np.zeros((N, 2))   # [vx_pred_next, wz_pred_next]
coeffs_vx  = np.zeros((N, dimensions))
coeffs_wz  = np.zeros((N, dimensions))

for i in range(N):
    # Базисные значения для текущего шага
    basis_vx = basis_values[i, :, 0].astype(float).tolist()
    basis_wz = basis_values[i, :, 1].astype(float).tolist()

    real_next_vx = float(f_target[i, 0])
    real_next_wz = float(f_target[i, 1])

    # CHANGED: явно приводим к обычным python float / list,
    # чтобы pybind11 без сюрпризов сделал std::vector<double> [file:7]
    alpha_vx = pso_vx.iterate(basis_vx, real_next_vx)
    alpha_wz = pso_wz.iterate(basis_wz, real_next_wz)

    alpha_vx = np.asarray(alpha_vx, dtype=float)
    alpha_wz = np.asarray(alpha_wz, dtype=float)

    coeffs_vx[i, :] = alpha_vx
    coeffs_wz[i, :] = alpha_wz

    # Линейная комбинация базисных функций (точно как в LS-примере)
    pred_vx = float(np.dot(alpha_vx, basis_vx))
    pred_wz = float(np.dot(alpha_wz, basis_wz))

    pred[i, 0] = pred_vx
    pred[i, 1] = pred_wz

true_vx = f_target[:, 0]
true_wz = f_target[:, 1]
pred_vx = pred[:, 0]
pred_wz = pred[:, 1]

mae_vx = mean_absolute_error(true_vx, pred_vx)
mae_wz = mean_absolute_error(true_wz, pred_wz)
print(f"MAE vx = {mae_vx:.4f}")
print(f"MAE wz = {mae_wz:.4f}")

# Время из stamp (наносекунды -> секунды)
t = ((df_cur["stamp"] - df_cur["stamp"].iloc[0]) * 1e-9).to_numpy()
cmd_vx = df_cur["cmd_vx"].to_numpy()
cmd_wz = df_cur["cmd_wz"].to_numpy()

err_vx = pred_vx - true_vx
err_wz = pred_wz - true_wz
abs_err_vx = np.abs(err_vx)
abs_err_wz = np.abs(err_wz)

fig, axs = plt.subplots(3, 1, figsize=(14, 10), sharex=True)

# 1) vx real vs predicted + control
axs[0].plot(t, true_vx, label="vx real", color="black")
axs[0].plot(t, pred_vx, label="vx predicted", color="orange", linestyle="--")
axs[0].plot(t, cmd_vx, label="cmd_vx", color="purple", linestyle=":")
axs[0].set_ylabel("vx [m/s]")
axs[0].legend()
axs[0].grid(True)

# 2) wz real vs predicted + control
axs[1].plot(t, true_wz, label="wz real", color="black")
axs[1].plot(t, pred_wz, label="wz predicted", color="blue", linestyle="--")
axs[1].plot(t, cmd_wz, label="cmd_wz", color="brown", linestyle=":")
axs[1].set_ylabel("wz [rad/s]")
axs[1].legend()
axs[1].grid(True)

# 3) |errors|
axs[2].plot(t, abs_err_vx, label="|vx error|", color="red")
axs[2].plot(t, abs_err_wz, label="|wz error|", color="green")
axs[2].set_xlabel("time [s]")
axs[2].set_ylabel("abs error")
axs[2].legend()
axs[2].grid(True)

plt.tight_layout()
show_or_save(fig, "model_test_prediction.png")

fig = plt.figure(figsize=(12, 4))
plt.plot(t, true_vx, label="vx real next", color="black")
for model_idx, (model_name, _) in enumerate(models):
    plt.plot(t, basis_outputs[model_idx][:, 0], label=f"{model_idx}: {model_name}")
plt.legend()
plt.grid(True)
plt.tight_layout()
show_or_save(fig, "model_test_basis_vx.png")

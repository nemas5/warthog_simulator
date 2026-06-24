import sys
from pathlib import Path

import torch
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from torchdiffeq import odeint
from sklearn.metrics import mean_absolute_error

sys.path.insert(0, str(Path(__file__).resolve().parent / "PSPSO" / "build"))
import pspso

class ODEFunc(torch.nn.Module):
    def __init__(self, hidden=64):
        super().__init__()
        self.net = torch.nn.Sequential(
            torch.nn.Linear(4, hidden), torch.nn.Tanh(),
            torch.nn.Linear(hidden, hidden), torch.nn.Tanh(),
            torch.nn.Linear(hidden, 2)
        )
        self.ctrl = None

    def forward(self, t, y):
        # y: [batch, 2], ctrl: [batch, 2]
        return self.net(torch.cat([y, self.ctrl], dim=-1))


ode_grass = ODEFunc()
ode_mud   = ODEFunc()

ckpt = torch.load("../neural_models/neural_ode_checkpoint.pth", map_location="cpu")
ode_grass.load_state_dict(ckpt["model_grass"])
ode_mud.load_state_dict(ckpt["model_mud"])
ode_grass.eval()
ode_mud.eval()

df = pd.read_csv("../data_samples/inference_test_data.csv")
df = df.iloc[:len(df) // 2].reset_index(drop=True)

df_cur  = df.iloc[:-1].reset_index(drop=True)
df_next = df.iloc[1:].reset_index(drop=True)

f_target = df_next[["odom_vx", "odom_wz"]].values  # shape (N, 2)

@torch.no_grad()
def get_basis_output(model: ODEFunc, df_slice: pd.DataFrame) -> np.ndarray:
    """
    Для каждой строки df_slice с полями
    odom_vx, odom_wz, cmd_vx, cmd_wz, dt
    интегрирует ODE на [0, dt] и возвращает предсказанное следующее состояние.
    """
    outputs = []
    for _, row in df_slice.iterrows():
        y0   = torch.tensor([[row["odom_vx"], row["odom_wz"]]], dtype=torch.float32)
        ctrl = torch.tensor([[row["cmd_vx"],  row["cmd_wz"]]], dtype=torch.float32)
        model.ctrl = ctrl

        t = torch.tensor([0.0, float(row["dt"])], dtype=torch.float32)
        traj = odeint(model, y0, t)          # shape: [2, 1, 2]
        y_next = traj[-1].squeeze(0).cpu().numpy()  # [2]
        outputs.append(y_next)

    return np.vstack(outputs)                # shape: (N, 2)


g1 = get_basis_output(ode_grass, df_cur)     # grass: [vx_next, wz_next]
g2 = get_basis_output(ode_mud,   df_cur)     # mud:   [vx_next, wz_next]


subswarms   = 5
particles   = 10
cognitive   = 2
social      = 2
perturb     = 0.002
max_ops     = 50
dimensions  = 2           # ДВА коэффициента (для g1 и g2) [file:4]
start_dyn_i = 0           # индекс стартовой базисной функции (не критично здесь)

# NOTE: два независимых экземпляра — чтобы каналы vx и wz не мешали друг другу
pso_vx = pspso.PSO(subswarms, particles, cognitive, social,
                   perturb, max_ops, dimensions, start_dyn_i)
pso_wz = pspso.PSO(subswarms, particles, cognitive, social,
                   perturb, max_ops, dimensions, start_dyn_i)

N = len(df_cur)
pred       = np.zeros((N, 2))   # [vx_pred_next, wz_pred_next]
coeffs_vx  = np.zeros((N, 2))
coeffs_wz  = np.zeros((N, 2))

for i in range(N):
    # Базисные значения для текущего шага
    basis_vx = [float(g1[i, 0]), float(g2[i, 0])]  # grass/mud для vx_next
    basis_wz = [float(g1[i, 1]), float(g2[i, 1])]  # grass/mud для wz_next

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
t = (df_cur["stamp"] - df_cur["stamp"].iloc[0]) * 1e-9

err_vx = pred_vx - true_vx
err_wz = pred_wz - true_wz
abs_err_vx = np.abs(err_vx)
abs_err_wz = np.abs(err_wz)

fig, axs = plt.subplots(3, 1, figsize=(14, 10), sharex=True)

# 1) vx real vs predicted + control
axs[0].plot(t, true_vx, label="vx real", color="black")
axs[0].plot(t, pred_vx, label="vx predicted", color="orange", linestyle="--")
axs[0].plot(t, df_cur["cmd_vx"], label="cmd_vx", color="purple", linestyle=":")
axs[0].set_ylabel("vx [m/s]")
axs[0].legend()
axs[0].grid(True)

# 2) wz real vs predicted + control
axs[1].plot(t, true_wz, label="wz real", color="black")
axs[1].plot(t, pred_wz, label="wz predicted", color="blue", linestyle="--")
axs[1].plot(t, df_cur["cmd_wz"], label="cmd_wz", color="brown", linestyle=":")
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
plt.show()

delta_real_vx = df_next["odom_vx"].to_numpy() - df_cur["odom_vx"].to_numpy()

plt.figure(figsize=(12, 4))
plt.plot(t, delta_real_vx, label="delta vx real")
plt.plot(t, g1[:, 0],   label="g1 vx_next (grass)")
plt.plot(t, g2[:, 0],   label="g2 vx_next (mud)")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()

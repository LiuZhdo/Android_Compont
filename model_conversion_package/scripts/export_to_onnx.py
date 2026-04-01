#!/usr/bin/env python3
"""
把当前目录里的 PyTorch 模型权重导出为 Android 更容易使用的 ONNX 文件。

使用前提:
1. 建议使用 Python 3.10 或 3.11
2. 安装依赖:
   pip install torch onnx onnxruntime

运行:
   python export_to_onnx.py

输出:
   model/adaptive_net_android.onnx
"""

from __future__ import annotations

from pathlib import Path

from torch_zip_checkpoint_loader import load_checkpoint

PACKAGE_ROOT = Path(__file__).resolve().parent.parent
INPUT_MODEL_DIR = PACKAGE_ROOT / "input_models"
OUTPUT_MODEL_DIR = PACKAGE_ROOT / "output_models"
WEIGHT_PATH = INPUT_MODEL_DIR / "best_adaptive_net_20260316_200459.pth"
CONFIG_PATH = INPUT_MODEL_DIR / "best_config_20260316_200459.pt"
OUTPUT_PATH = OUTPUT_MODEL_DIR / "adaptive_net_android.onnx"
FIXED_OUTPUT_PATH = OUTPUT_MODEL_DIR / "adaptive_net_android.fixed.onnx"


def main() -> None:
    try:
        import torch
        import torch.nn as nn
        import onnx
    except ImportError as exc:
        raise SystemExit(
            "没有找到 torch。请先用 Python 3.10/3.11 安装: pip install torch onnx onnxruntime"
        ) from exc

    if not WEIGHT_PATH.exists():
        raise SystemExit(f"没有找到权重文件: {WEIGHT_PATH}")
    if not CONFIG_PATH.exists():
        raise SystemExit(f"没有找到配置文件: {CONFIG_PATH}")

    OUTPUT_MODEL_DIR.mkdir(parents=True, exist_ok=True)

    bundle = load_checkpoint(CONFIG_PATH)
    config = bundle["config"]

    class AdaptiveNet(nn.Module):
        def __init__(self, cfg: dict) -> None:
            super().__init__()
            hidden_size = int(cfg["HIDDEN_SIZE"])
            lstm_layers = int(cfg["LSTM_LAYERS"])
            lstm_dropout = float(cfg["LSTM_DROPOUT"]) if lstm_layers > 1 else 0.0
            mlp_hidden = list(cfg["MLP_HIDDEN"])
            mlp_dropout = float(cfg["MLP_DROPOUT"])
            use_attention = bool(cfg["USE_ATTENTION"])

            self.use_attention = use_attention

            self.lstm = nn.LSTM(
                input_size=5,
                hidden_size=hidden_size,
                num_layers=lstm_layers,
                dropout=lstm_dropout,
                batch_first=True,
            )

            self.attention = nn.Sequential(
                nn.Linear(hidden_size, 32),
                nn.Tanh(),
                nn.Linear(32, 1),
            )

            self.scalar_net = nn.Sequential(
                nn.Linear(2, 32),
                nn.ReLU(),
                nn.Linear(32, 32),
                nn.ReLU(),
            )

            fused_size = hidden_size + 32
            self.mlp_backbone = nn.Sequential(
                nn.Linear(fused_size, mlp_hidden[0]),
                nn.BatchNorm1d(mlp_hidden[0]),
                nn.ReLU(),
                nn.Dropout(mlp_dropout),
                nn.Linear(mlp_hidden[0], mlp_hidden[1]),
                nn.BatchNorm1d(mlp_hidden[1]),
                nn.ReLU(),
                nn.Dropout(mlp_dropout),
            )
            self.residual_proj = nn.Linear(fused_size, mlp_hidden[1])
            self.output_head = nn.Linear(mlp_hidden[1], 4)

        def forward(self, time_features, scalar_features):
            lstm_out, _ = self.lstm(time_features)
            if self.use_attention:
                attn_scores = self.attention(lstm_out)
                attn_weights = torch.softmax(attn_scores, dim=1)
                time_embed = (attn_weights * lstm_out).sum(dim=1)
            else:
                time_embed = lstm_out[:, -1, :]

            scalar_embed = self.scalar_net(scalar_features)
            fused = torch.cat([time_embed, scalar_embed], dim=1)
            hidden = self.mlp_backbone(fused) + self.residual_proj(fused)
            return self.output_head(hidden)

    model = AdaptiveNet(config)
    state_dict = load_checkpoint(WEIGHT_PATH)
    model.load_state_dict(state_dict, strict=True)
    model.eval()

    seq_len = int(config["SEQ_LEN"])
    dummy_time = torch.randn(1, seq_len, 5, dtype=torch.float32)
    dummy_scalar = torch.randn(1, 2, dtype=torch.float32)

    # 如果这里两组不同输入给出完全相同输出，说明权重或模型结构仍然有问题。
    with torch.no_grad():
        probe_a = model(torch.zeros(1, seq_len, 5), torch.tensor([[1.0, 2.0]], dtype=torch.float32))
        probe_b = model(torch.ones(1, seq_len, 5), torch.tensor([[5.0, -3.0]], dtype=torch.float32))
        if torch.allclose(probe_a, probe_b):
            raise SystemExit(
                "导出终止: 当前加载出来的模型对不同输入输出完全相同，说明权重解析或模型结构仍有问题。"
            )

    torch.onnx.export(
        model,
        (dummy_time, dummy_scalar),
        OUTPUT_PATH.as_posix(),
        input_names=["time_features", "scalar_features"],
        output_names=["output"],
        dynamic_axes={
            "time_features": {0: "batch_size"},
            "scalar_features": {0: "batch_size"},
            "output": {0: "batch_size"},
        },
        opset_version=18,
        do_constant_folding=True,
        dynamo=False,
    )

    # 某些导出器会额外生成 .onnx.data 文件。这里强制合并成单文件，
    # 这样 Android 端只需要拷贝一个 .onnx 到 assets 即可。
    merged_model = onnx.load(OUTPUT_PATH.as_posix(), load_external_data=True)
    onnx.save_model(merged_model, OUTPUT_PATH.as_posix(), save_as_external_data=False)
    external_data_path = OUTPUT_PATH.with_suffix(OUTPUT_PATH.suffix + ".data")
    if external_data_path.exists():
        external_data_path.unlink()

    fixed_model = onnx.load(OUTPUT_PATH.as_posix())
    for value_info in list(fixed_model.graph.input) + list(fixed_model.graph.output):
        for dim in value_info.type.tensor_type.shape.dim:
            if dim.HasField("dim_param") and dim.dim_param == "batch_size":
                dim.ClearField("dim_param")
                dim.dim_value = 1
    onnx.save_model(fixed_model, FIXED_OUTPUT_PATH.as_posix(), save_as_external_data=False)

    print(f"导出完成: {OUTPUT_PATH}")
    print(f"固定输入形状版本: {FIXED_OUTPUT_PATH}")
    print("输入1: time_features, 形状类似 [1, 10, 5]")
    print("输入2: scalar_features, 形状类似 [1, 2]")
    print("输出 : output, 形状类似 [1, 4]")


if __name__ == "__main__":
    main()

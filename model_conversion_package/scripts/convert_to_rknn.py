#!/usr/bin/env python3
"""
用 RKNN Toolkit2 把当前 ONNX 模型转换成 Rockchip NPU 可用的 .rknn 文件。

推荐在 Docker 镜像 local/rknn-toolkit2:2.3.2-runtime 中运行。
"""

from __future__ import annotations

from pathlib import Path
import sys


PACKAGE_ROOT = Path(__file__).resolve().parent.parent
MODEL_DIR = PACKAGE_ROOT / "output_models"
ONNX_PATH = MODEL_DIR / "adaptive_net_android.fixed.onnx"
RKNN_PATH = MODEL_DIR / "adaptive_net_android.rknn"
TARGET_PLATFORM = "rk3568"


def main() -> int:
    try:
        from rknn.api import RKNN
    except Exception as exc:  # pragma: no cover
        print(f"无法导入 RKNN Toolkit2: {exc}", file=sys.stderr)
        return 1

    if not ONNX_PATH.exists():
        print(f"没有找到 ONNX 文件: {ONNX_PATH}", file=sys.stderr)
        return 1

    rknn = RKNN(verbose=True)

    try:
        print(f"正在配置 RKNN，目标平台: {TARGET_PLATFORM}")
        ret = rknn.config(
            target_platform=TARGET_PLATFORM,
            quantized_dtype="asymmetric_quantized-8",
            optimization_level=3,
        )
        if ret != 0:
            print(f"rknn.config 失败，返回值: {ret}", file=sys.stderr)
            return ret

        print(f"正在加载 ONNX: {ONNX_PATH}")
        ret = rknn.load_onnx(
            model=ONNX_PATH.as_posix(),
        )
        if ret != 0:
            print(f"rknn.load_onnx 失败，返回值: {ret}", file=sys.stderr)
            return ret

        print("正在构建 RKNN 模型，不做量化...")
        ret = rknn.build(do_quantization=False)
        if ret != 0:
            print(f"rknn.build 失败，返回值: {ret}", file=sys.stderr)
            return ret

        print(f"正在导出 RKNN 文件: {RKNN_PATH}")
        ret = rknn.export_rknn(RKNN_PATH.as_posix())
        if ret != 0:
            print(f"rknn.export_rknn 失败，返回值: {ret}", file=sys.stderr)
            return ret

        print("转换成功")
        print(RKNN_PATH)
        return 0
    finally:
        rknn.release()


if __name__ == "__main__":
    raise SystemExit(main())

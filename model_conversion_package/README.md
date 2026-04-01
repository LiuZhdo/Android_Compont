# 模型转换独立包

这个目录是专门给“把当前 checkpoint 转成 ONNX / RKNN”的独立交付包。

适合直接交给别人使用，不依赖当前项目其它目录。

## 目录结构

- `input_models/`
  原始输入模型文件

- `output_models/`
  当前已经生成好的 ONNX / RKNN 输出文件

- `scripts/`
  转换脚本

- `docker/rknn/`
  RKNN Toolkit2 的 Docker 环境

- `docs/`
  中文方法说明

## 你最常用的文件

- `scripts/export_to_onnx.py`
  从 checkpoint 导出 ONNX

- `scripts/convert_to_rknn.py`
  从 ONNX 转换 RKNN

- `scripts/torch_zip_checkpoint_loader.py`
  自定义 checkpoint 读取器

- `output_models/adaptive_net_android.rknn`
  已生成好的 RKNN 模型

## 推荐使用方式

如果只想快速使用当前已有结果，直接拿:

- `output_models/adaptive_net_android.onnx`
- `output_models/adaptive_net_android.fixed.onnx`
- `output_models/adaptive_net_android.rknn`

如果想重新跑完整转换流程，优先看:

- `docs/使用方法_中文.md`

## 为什么这里需要自定义 checkpoint 读取器

当前 `.pth` 权重文件不能直接完全依赖标准 `torch.load` 来正确恢复大部分大矩阵权重。

因此:

- ONNX 导出脚本必须依赖 `torch_zip_checkpoint_loader.py`

否则会出现：

- 模型输出接近常量
- 后续 RKNN 转换失败或结果失真

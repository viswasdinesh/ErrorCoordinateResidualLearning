#!/usr/bin/env python3
"""export_residual_weights.py -- generate a header-only Eigen C++ port of the
trained `ecrl` residual model (models/ecrl_model.pt + ecrl_metadata.json) for
use in the live MPC controller.

This is a code generator, not a runtime dependency: it reads the plain
(spectral-norm-already-removed) TorchScript state_dict and the normalization
stats from the metadata JSON, and emits
include/ommpc_ros2/residual_model_ecrl.h -- static Eigen weight/bias matrices
plus one function, ommpc::residual::ecrlForward(). No libtorch/onnxruntime
dependency is introduced; the generated header only needs Eigen, already a
project dependency (see include/ommpc_ros2/nominal_dynamics.h/so3_math.h for
the same hand-port-and-parity-test convention this follows).

Regenerate (don't hand-edit the generated header) whenever the model is
retrained:
    conda run -n ecrl python3 scripts/export_residual_weights.py
"""
import argparse
import json
import os

import numpy as np
import torch

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

LAYER_KEYS = ["network.0", "network.2", "network.4"]  # Linear, Tanh, Linear, Tanh, Linear


def format_array_literal(values) -> str:
    # Wrap at a modest line width so generated files stay diffable/reviewable;
    # a single giant Eigen comma-initializer expression (`<<` chain) is not
    # used here on purpose -- for a 128x128 layer that's 16384 chained
    # `operator,` calls in one C++ statement, which was observed to exhaust
    # the compiler's resources. A brace-initialized flat array is the
    # standard, cheap-to-compile way to embed large static tables.
    tokens = [f"{v:.17g}" for v in values]
    lines = []
    for i in range(0, len(tokens), 8):
        lines.append("  " + ", ".join(tokens[i:i + 8]) + ",")
    return "\n".join(lines)


def format_vec(name: str, values) -> str:
    n = len(values)
    data = format_array_literal(values)
    return (f"static const double {name}_data[{n}] = {{\n{data}\n}};\n"
            f"static const Eigen::Map<const Eigen::VectorXd> {name}({name}_data, {n});")


def format_mat(name: str, mat: np.ndarray) -> str:
    rows, cols = mat.shape
    data = format_array_literal(mat.flatten())  # row-major flat: mat[i,j] at i*cols+j
    return (f"static const double {name}_data[{rows * cols}] = {{\n{data}\n}};\n"
            f"static const Eigen::Map<const Eigen::Matrix<double, {rows}, {cols}, Eigen::RowMajor>> "
            f"{name}({name}_data);")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--adapter", default="ecrl",
                     help="Adapter name (e.g. ecrl, ambient). Names the generated forward-pass "
                          "function ({adapter}Forward) and prefixes its weight symbols "
                          "(k{Adapter}W0, k{Adapter}InputMean, ...) so headers for multiple "
                          "adapters can be #included in the same translation unit without symbol "
                          "collisions. Also picks the defaults for --model_path/--metadata_path/"
                          "--out_header when not explicitly overridden.")
    ap.add_argument("--model_path", default=None,
                     help="Plain (non-normalizing) TorchScript model, spectral_norm already removed "
                          "-- e.g. models/ecrl_model.pt or knode_training/output/ecrl_knode_model.pt. "
                          "Default: models/{adapter}_model.pt")
    ap.add_argument("--metadata_path", default=None,
                     help="Matching metadata JSON (input/output mean/std). Default: models/{adapter}_metadata.json")
    ap.add_argument("--out_header", default=None,
                     help="Default: include/ommpc_ros2/residual_model_{adapter}.h")
    args = ap.parse_args()

    ADAPTER = args.adapter
    ADAPTER_CAP = ADAPTER[:1].upper() + ADAPTER[1:]  # e.g. "ecrl" -> "Ecrl", for symbol prefixes
    MODEL_PATH = args.model_path or os.path.join(SCRIPT_DIR, "..", "models", f"{ADAPTER}_model.pt")
    METADATA_PATH = args.metadata_path or os.path.join(SCRIPT_DIR, "..", "models", f"{ADAPTER}_metadata.json")
    OUT_HEADER = args.out_header or os.path.join(SCRIPT_DIR, "..", "include", "ommpc_ros2",
                                                  f"residual_model_{ADAPTER}.h")

    state_dict = torch.jit.load(MODEL_PATH).state_dict()
    with open(METADATA_PATH) as f:
        meta = json.load(f)

    in_dim, out_dim = meta["input_dim"], meta["output_dim"]
    assert out_dim == 9, f"unexpected output dim in {METADATA_PATH}: {out_dim} (every adapter regresses onto the same 9-D world-frame d_k)"

    W, B = "W", "B"  # per-layer names get an index suffix below; prefixed with the adapter below

    lines = []
    lines.append("// GENERATED FILE -- do not hand-edit.")
    lines.append("// Produced by scripts/export_residual_weights.py from:")
    lines.append(f"//   {os.path.relpath(MODEL_PATH, os.path.dirname(OUT_HEADER))}")
    lines.append(f"//   {os.path.relpath(METADATA_PATH, os.path.dirname(OUT_HEADER))}")
    lines.append("// Regenerate (do not edit by hand) if the model is retrained:")
    lines.append(f"//   conda run -n ecrl python3 scripts/export_residual_weights.py --adapter {ADAPTER}")
    lines.append("//")
    lines.append(f"// Hand-ported forward pass for models/{ADAPTER}_model.pt (spectral_norm already")
    lines.append("// removed at export time -- see train_residuals_v2.py's export step), mirroring")
    lines.append(f"// residuals.py's {in_dim}-D input / {out_dim}-D output layout for the '{ADAPTER}' adapter.")
    lines.append(f"// Architecture: Linear({in_dim},128) -> tanh -> Linear(128,128) -> tanh -> Linear(128,{out_dim}),")
    lines.append("// wrapped with external input normalization / output de-normalization matching")
    lines.append(f"// models/{ADAPTER}_metadata.json (the same stats baked into {ADAPTER}_model_normalized.pt).")
    lines.append("// Symbols are prefixed with the adapter name so this header can be #included")
    lines.append("// alongside other adapters' generated headers in the same translation unit.")
    lines.append("#pragma once")
    lines.append("")
    lines.append("#include <Eigen/Eigen>")
    lines.append("")
    lines.append("namespace ommpc")
    lines.append("{")
    lines.append("namespace residual")
    lines.append("{")
    lines.append("")

    for i, key in enumerate(LAYER_KEYS):
        w = state_dict[f"{key}.weight"].detach().numpy().astype(np.float64)  # [out, in], PyTorch y=x@W^T+b
        b = state_dict[f"{key}.bias"].detach().numpy().astype(np.float64)
        lines.append(format_mat(f"k{ADAPTER_CAP}{W}{i}", w))
        lines.append(format_vec(f"k{ADAPTER_CAP}{B}{i}", b))
        lines.append("")

    lines.append(format_vec(f"k{ADAPTER_CAP}InputMean", np.array(meta["input_mean"], dtype=np.float64)))
    lines.append(format_vec(f"k{ADAPTER_CAP}InputStd", np.array(meta["input_std"], dtype=np.float64)))
    lines.append(format_vec(f"k{ADAPTER_CAP}OutputMean", np.array(meta["output_mean"], dtype=np.float64)))
    lines.append(format_vec(f"k{ADAPTER_CAP}OutputStd", np.array(meta["output_std"], dtype=np.float64)))
    lines.append("")

    lines.append(f"// x: {in_dim}-D '{ADAPTER}' adapter input (residuals.py's feature order for this")
    lines.append(f"// adapter). Returns the {out_dim}-D world-frame residual d_k prediction")
    lines.append("// (residuals.py::compute_targets' d_0..d_8 order: pos x/y/z, vel x/y/z, attitude x/y/z).")
    lines.append(f"inline Eigen::VectorXd {ADAPTER}Forward(const Eigen::VectorXd &x)")
    lines.append("{")
    lines.append(f"  Eigen::VectorXd xn = (x - k{ADAPTER_CAP}InputMean).cwiseQuotient(k{ADAPTER_CAP}InputStd);")
    lines.append(f"  Eigen::VectorXd h0 = (k{ADAPTER_CAP}{W}0 * xn + k{ADAPTER_CAP}{B}0).array().tanh();")
    lines.append(f"  Eigen::VectorXd h1 = (k{ADAPTER_CAP}{W}1 * h0 + k{ADAPTER_CAP}{B}1).array().tanh();")
    lines.append(f"  Eigen::VectorXd yn = k{ADAPTER_CAP}{W}2 * h1 + k{ADAPTER_CAP}{B}2;")
    lines.append(f"  return yn.cwiseProduct(k{ADAPTER_CAP}OutputStd) + k{ADAPTER_CAP}OutputMean;")
    lines.append("}")
    lines.append("")
    lines.append("}  // namespace residual")
    lines.append("}  // namespace ommpc")
    lines.append("")

    os.makedirs(os.path.dirname(OUT_HEADER), exist_ok=True)
    with open(OUT_HEADER, "w") as f:
        f.write("\n".join(lines))

    print(f"Wrote {OUT_HEADER}")
    for i, key in enumerate(LAYER_KEYS):
        w = state_dict[f"{key}.weight"].shape
        print(f"  layer {i} ({key}): weight {tuple(w)}")


if __name__ == "__main__":
    main()

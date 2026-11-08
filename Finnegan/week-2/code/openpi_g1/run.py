"""Einstieg in openpi fuer G1 + BrainCo: registriert pi05_g1_brainco_lora und ruft openpis eigene Skripte auf.

Laeuft in der openpi-Umgebung, Aufruf aus dem Repo mit PY=~/dev/g1-ext/openpi/.venv/bin/python:
  $PY -m openpi_g1.run check                        Konfiguration laden, Ersatz-Policy einmal durch die Umrechnung schicken
  $PY -m openpi_g1.run stand-in [--port 8000]       Ersatz-Policy-Server ohne Gewichte (Leitungstest)
  $PY -m openpi_g1.run norm-stats [--max-frames N]  Normierungs-Statistik aus dem Datensatz (vor dem Training)
  $PY -m openpi_g1.run train --dry-run --exp-name X nur die aufgeloeste Trainingskonfiguration zeigen
  $PY -m openpi_g1.run train --exp-name X [...]     LoRA-Training mit openpis scripts/train.py - nur Linux + NVIDIA-GPU
  $PY -m openpi_g1.run serve --checkpoint DIR       trainierten Checkpoint mit openpis scripts/serve_policy.py bereitstellen
"""
import argparse
import json
import logging
import pathlib
import platform
import runpy
import sys

import numpy as np

from g1sim import action_space
from openpi_g1 import config


def openpi_dir():
    """Wurzel des openpi-Checkouts (dort liegen scripts/train.py usw.)."""
    import openpi

    return pathlib.Path(openpi.__file__).resolve().parents[2]


def run_openpi_script(name, argv):
    script = openpi_dir() / "scripts" / name
    sys.argv = [str(script), *argv]
    runpy.run_path(str(script), run_name="__main__")


def summary(train_config):
    data = train_config.data.create(train_config.assets_dirs, train_config.model)
    model = train_config.model
    return {
        "name": train_config.name,
        "model": {"pi05": model.pi05, "action_horizon": model.action_horizon, "action_dim": model.action_dim,
                  "paligemma": model.paligemma_variant, "action_expert": model.action_expert_variant},
        "base_weights": train_config.weight_loader.params_path,
        "dataset": data.repo_id,
        "norm_stats": str(train_config.assets_dirs / data.asset_id) if data.norm_stats else "fehlt noch (norm-stats)",
        "quantile_norm": data.use_quantile_norm,
        "prompt_from_task": data.prompt_from_task,
        "delta_actions": f"{sum(action_space.DELTA_MASK)} Armgelenke als Delta, Haende absolut",
        "batch_size": train_config.batch_size,
        "num_train_steps": train_config.num_train_steps,
        "ema_decay": train_config.ema_decay,
        "checkpoint_base_dir": train_config.checkpoint_base_dir,
    }


def check():
    """Konfiguration laden und die Ersatz-Policy einmal durch openpis komplette Umrechnung schicken."""
    from openpi_g1 import policy, stand_in

    train_config = config.register()
    print(json.dumps(summary(train_config), indent=2, ensure_ascii=False))
    actions = stand_in.create_policy(train_config).infer(policy.make_example())["actions"]
    error = float(np.abs(actions - action_space.READY_POSE).max())
    ok = actions.shape == (train_config.model.action_horizon, action_space.DIM) and error < 1e-4
    print(f"Ersatz-Policy: Aktionen {actions.shape}, groesste Abweichung von der Startpose {error:.1e}")
    print("KONFIGURATION OK" if ok else "KONFIGURATION FEHLERHAFT")
    return 0 if ok else 1


def norm_stats(max_frames=None):
    config.register()
    script = runpy.run_path(str(openpi_dir() / "scripts" / "compute_norm_stats.py"))
    script["main"](config.CONFIG_NAME, max_frames=max_frames)


def train(argv, dry_run=False):
    from openpi.training import config as _config

    config.register()
    if dry_run:
        sys.argv = ["train.py", config.CONFIG_NAME, *argv]
        train_config = _config.cli()
        print(json.dumps({**summary(train_config), "exp_name": train_config.exp_name,
                          "checkpoint_dir": str(train_config.checkpoint_dir)}, indent=2, ensure_ascii=False))
        return 0
    import jax

    if platform.system() != "Linux" or jax.default_backend() != "gpu":
        print(f"Training nur auf Linux mit NVIDIA-GPU (hier {platform.system()}, JAX-Backend {jax.default_backend()}) - "
              "siehe CLAUDE.md und PROGRESS.md Blocker 3. Konfiguration pruefen: train --dry-run --exp-name X")
        return 2
    run_openpi_script("train.py", [config.CONFIG_NAME, *argv])
    return 0


def serve(checkpoint, port):
    config.register()
    run_openpi_script("serve_policy.py", ["policy:checkpoint", f"--policy.config={config.CONFIG_NAME}",
                                          f"--policy.dir={checkpoint}", f"--port={port}"])


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    commands = parser.add_subparsers(dest="command", required=True)
    commands.add_parser("check", help="Konfiguration laden + Ersatz-Policy einmal fragen")
    stand_in_cmd = commands.add_parser("stand-in", help="Ersatz-Policy-Server ohne Gewichte")
    stand_in_cmd.add_argument("--host", default="127.0.0.1")
    stand_in_cmd.add_argument("--port", type=int, default=8000)
    norm_cmd = commands.add_parser("norm-stats", help="Normierungs-Statistik aus dem Datensatz")
    norm_cmd.add_argument("--max-frames", type=int)
    train_cmd = commands.add_parser("train", help="LoRA-Training (nur Linux + NVIDIA-GPU); weitere Optionen gehen an openpi")
    train_cmd.add_argument("--dry-run", action="store_true", help="nur Konfiguration aufloesen und zeigen")
    serve_cmd = commands.add_parser("serve", help="trainierten Checkpoint bereitstellen")
    serve_cmd.add_argument("--checkpoint", required=True)
    serve_cmd.add_argument("--port", type=int, default=8000)
    args, rest = parser.parse_known_args(argv)
    if rest and args.command != "train":
        parser.error(f"unbekannte Argumente: {rest}")

    logging.basicConfig(level=logging.INFO, force=True)
    if args.command == "check":
        return check()
    if args.command == "stand-in":
        from openpi_g1 import stand_in

        stand_in.serve(args.host, args.port)
    elif args.command == "norm-stats":
        norm_stats(args.max_frames)
    elif args.command == "train":
        return train(rest, dry_run=args.dry_run)
    elif args.command == "serve":
        serve(args.checkpoint, args.port)
    return 0


if __name__ == "__main__":
    sys.exit(main())

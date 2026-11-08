"""Rohaufnahmen (.npz, g1sim/episode.py) -> LeRobot-Datensatz v2.1 fuer openpi (Stufe 2.1).

Muster: openpi examples/libero/convert_libero_data_to_lerobot.py. Laeuft in der openpi-Umgebung (lerobot 0.1.0 = v2.1).
Ziel: $HF_LEROBOT_HOME/<repo_id> (Standard ~/.cache/huggingface/lerobot) - dort sucht openpis Datenlader.
Jede Episode wird vorher mit episode.validate() geprueft. Episoden mit success=False bleiben draussen, ausser mit
--include-failed (nur fuer Leitungstests, nie fuers Training).

    ~/dev/g1-ext/openpi/.venv/bin/python -m openpi_g1.convert runs/stufe2_0_openpi/*.npz [--repo-id ...] [--overwrite]
"""
import argparse
import json
import pathlib
import shutil
import sys

import lerobot.common.datasets.lerobot_dataset as lerobot_dataset

from g1sim import action_space, episode
from openpi_g1 import config

ROBOT_TYPE = "unitree_g1_brainco"


def features(ep):
    """LeRobot-Spalten: drei Bilder (jedes in seiner eigenen Groesse, die Kopfkamera ist 16:10), Zustand und Aktionen mit den
    Gelenknamen aus action_space."""
    images = {key: {"dtype": "image", "shape": (*ep[key].shape[1:3], 3), "names": ["height", "width", "channel"]}
              for key in episode.IMAGE_KEYS}
    vector = {"dtype": "float32", "shape": (action_space.DIM,), "names": list(action_space.NAMES)}
    return {**images, "state": dict(vector), "actions": dict(vector)}


def convert(paths, repo_id=config.REPO_ID, include_failed=False, overwrite=False):
    """Episoden in einen neuen Datensatz schreiben. Rueckgabe: Zusammenfassung (geschrieben, uebersprungen mit Grund)."""
    root = lerobot_dataset.HF_LEROBOT_HOME / repo_id
    if root.exists():
        if not overwrite:
            raise FileExistsError(f"{root} existiert schon - --overwrite ersetzt ihn")
        if not (root / "meta" / "info.json").is_file():
            raise RuntimeError(f"{root} ist kein LeRobot-Datensatz - wird nicht geloescht")
        shutil.rmtree(root)

    dataset, written, frames, skipped = None, [], 0, {}
    for path in sorted(map(pathlib.Path, paths)):
        ep = episode.load(path)
        problems = episode.validate(ep)
        if not ep["success"] and not include_failed:
            problems.append("nicht erfolgreich (success=False)")
        if dataset is not None and ep["fps"] != dataset.fps:
            problems.append(f"fps {ep['fps']} statt {dataset.fps}")
        if problems:
            skipped[str(path)] = problems
            continue
        if dataset is None:
            # image_writer_threads=0: Bilder synchron schreiben - langsamer, aber ohne Hintergrund-Threads
            dataset = lerobot_dataset.LeRobotDataset.create(
                repo_id=repo_id, fps=ep["fps"], robot_type=ROBOT_TYPE, features=features(ep), image_writer_threads=0)
        for t in range(len(ep["timestamp"])):
            dataset.add_frame({
                **{key: ep[key][t] for key in episode.IMAGE_KEYS},
                "state": ep["state"][t],
                "actions": ep["actions"][t],
                "task": ep["task"],
            })
        dataset.save_episode()
        written.append(str(path))
        frames += len(ep["timestamp"])
    return {"repo_id": repo_id, "root": str(root), "episodes": len(written), "frames": frames, "skipped": skipped}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("episodes", nargs="+", type=pathlib.Path)
    parser.add_argument("--repo-id", default=config.REPO_ID)
    parser.add_argument("--include-failed", action="store_true", help="auch success=False (nur Leitungstests)")
    parser.add_argument("--overwrite", action="store_true", help="vorhandenen Datensatz ersetzen")
    args = parser.parse_args(argv)
    result = convert(args.episodes, args.repo_id, include_failed=args.include_failed, overwrite=args.overwrite)
    print(json.dumps(result, indent=2, ensure_ascii=False))
    return 0 if result["episodes"] else 1


if __name__ == "__main__":
    sys.exit(main())

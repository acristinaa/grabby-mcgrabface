"""Level 1 Step 4 - policy server for the eval (82): a SmolVLA checkpoint or recorded demos, over g1sim/policy_wire.py.

Modes (one required):
  --replay <npz|dir> [...]   ReplayPredictor: serves the recorded actions of the episode named in reset and measures how far
                             each observation is from the recorded frame (the replay gate). No torch import; runs in either env.
  --checkpoint <dir>         SmolVLAPredictor: lerobot 0.6.1 SmolVLA (SmolVLA env). <dir> = .../pretrained_model or its parent;
                             checkpoints/last is resolved to its real step. One noise chunk per request, drawn from a CPU
                             generator re-seeded at every reset, so the same seed and observation give the same chunk on CPU.
The server binds 127.0.0.1 (--host to expose it), serves one client at a time and prints "LISTENING <port>" only after the
predictor is loaded (--port 0 = ephemeral port).
--self-test <npz>: checkpoint mode only, no port opened. Infers one recorded frame twice with the same noise seed
(policy_wire.noise_seed(stem)), checks shape (50, 27), finite, identical on CPU, and that a 400 x 640 image is rejected; exit 0/1.
Run1 rules while lerobot-train runs (STEP4-PLAN §0): no --device mps (refused unless --force-mps) and no checkpoint inside the
training's --output_dir (refused unless --allow-during-training, R4: not approved by the user; even then only complete
checkpoints whose newest file is older than 60 s). A refusal exits 2 before torch is imported or any checkpoint file is read.
    replay (sim env or SmolVLA env):
        ~/dev/g1-ext/.venv/bin/python scripts/81_smolvla_server.py \\
            --replay "/Volumes/T7/Robot Project SAP/DATA Collection/g1-demos/smoke_v1" --port 8765
    checkpoint on CPU (SmolVLA env):
        ~/dev/g1-ext/smolvla-test/.venv/bin/python scripts/81_smolvla_server.py \\
            --checkpoint ~/dev/g1-ext/smolvla-runs/smoke/checkpoints/000200/pretrained_model --device cpu --threads 4
"""
import argparse
import functools
import os
import pathlib
import sys
import time

import numpy as np

REPO = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

from g1sim import episode, policy_wire  # noqa: E402

HF_HUB_CACHE = pathlib.Path.home() / "dev" / "g1-ext" / "smolvla-test" / "hf_cache" / "hub"  # as train_smolvla.sh
IMAGE_FEATURE = "observation.images.camera1"
STATE_FEATURE = "observation.state"


def refuse(message):
    print(message, file=sys.stderr, flush=True)
    raise SystemExit(2)


class SmolVLAPredictor:
    """SmolVLA checkpoint behind the predictor interface of policy_wire.serve: metadata, reset(episode, seed), infer(obs, tick).
    Guards first, then the checkpoint files, then the torch/lerobot imports, then the load."""

    def __init__(self, checkpoint, device="cpu", threads=None, force_mps=False, allow_during_training=False):
        cmdlines = policy_wire.training_cmdlines()
        problem = policy_wire.device_problem(device, cmdlines, force_mps)
        if problem:
            refuse(problem)
        path = pathlib.Path(os.path.realpath(os.path.expanduser(str(checkpoint))))
        if path.name != "pretrained_model":
            path = pathlib.Path(os.path.realpath(path / "pretrained_model"))
        problem = policy_wire.checkpoint_problem(path, cmdlines, allow_during_training, time.time())
        if problem:
            refuse(problem)
        missing = [name for name in policy_wire.CHECKPOINT_FILES if not (path / name).is_file()]
        stats = sorted(path.glob("policy_preprocessor_step_*_normalizer_processor.safetensors"))
        if missing or not stats:
            refuse(f"checkpoint not found or incomplete: {path} (missing {missing or 'normalizer stats'})")

        os.environ.setdefault("HF_HUB_CACHE", str(HF_HUB_CACHE))  # the SmolVLM2 backbone comes from this cache, offline
        os.environ.setdefault("HF_HUB_OFFLINE", "1")
        import lerobot
        import torch
        from lerobot.configs.policies import PreTrainedConfig
        from lerobot.policies.factory import make_pre_post_processors
        from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy
        from lerobot.policies.utils import prepare_observation_for_inference
        from safetensors.numpy import load_file

        self.torch, self.prepare = torch, prepare_observation_for_inference
        if threads:
            torch.set_num_threads(threads)
        cfg = PreTrainedConfig.from_pretrained(path)
        # the config still lists smolvla_base's input_features (state [6], 3 cameras); the real layout is in the stats
        stat = load_file(stats[0])
        layout = {"type smolvla": cfg.type == "smolvla", f"chunk_size {policy_wire.CHUNK}": cfg.chunk_size == policy_wire.CHUNK,
                  f"{IMAGE_FEATURE} in image_features": IMAGE_FEATURE in cfg.image_features,
                  f"action output ({policy_wire.ACTION_DIM},)":
                      tuple(cfg.output_features["action"].shape) == (policy_wire.ACTION_DIM,)
                      and stat.get("action.mean", np.zeros(0)).shape == (policy_wire.ACTION_DIM,),
                  f"{STATE_FEATURE}.mean {policy_wire.ACTION_DIM} entries":
                      stat.get(f"{STATE_FEATURE}.mean", np.zeros(0)).shape == (policy_wire.ACTION_DIM,)}
        wrong = [name for name, ok in layout.items() if not ok]
        if wrong:
            refuse(f"checkpoint layout not as trained by train_smolvla.sh: {wrong} ({path})")
        cfg.device = device
        self.cfg, self.device = cfg, torch.device(device)
        start = time.perf_counter()
        self.policy = SmolVLAPolicy.from_pretrained(path, config=cfg)
        self.pre, self.post = make_pre_post_processors(
            cfg, pretrained_path=str(path), preprocessor_overrides={"device_processor": {"device": device}})
        self.load_s = time.perf_counter() - start
        step = path.parent.name
        self.metadata = {"mode": "smolvla", "chunk_size": cfg.chunk_size, "action_dim": policy_wire.ACTION_DIM,
                         "image_shape": list(policy_wire.IMAGE_SHAPE), "checkpoint": str(path),
                         "step": int(step) if step.isdigit() else step, "device": device, "source": str(path),
                         "threads": torch.get_num_threads(), "lerobot": lerobot.__version__}
        self.generator = torch.Generator().manual_seed(0)

    def reset(self, episode_stem, seed):
        self.generator = self.torch.Generator().manual_seed(int(seed))  # CPU generator: the same noise on every device
        self.policy.reset()

    def infer(self, obs, tick):
        batch = self.prepare({IMAGE_FEATURE: np.array(obs[policy_wire.IMAGE_KEY], copy=True),  # converted in place
                              STATE_FEATURE: np.array(obs[policy_wire.STATE_KEY], dtype=np.float32, copy=True)},
                             self.device, task=obs["prompt"])
        noise = self.torch.randn(1, self.cfg.chunk_size, self.cfg.max_action_dim, generator=self.generator)
        actions = self.policy.predict_action_chunk(self.pre(batch), noise=noise.to(self.device))
        return self.post(actions)[0].float().cpu().numpy().astype(np.float32), {}


def self_test(predictor, npz, frame, device, save_chunk):
    """One recorded frame through the predictor twice with the same seed. Returns True if every check passed."""
    ep = episode.load(npz)
    stem = pathlib.Path(npz).stem
    obs = episode.policy_observation({"head_image": ep["head_image"][frame], "state": ep["state"][frame]}, policy_wire.TASK)
    problems = policy_wire.check_observation(obs)
    print(f"observation from {stem} frame {frame}: {problems or 'valid'}; npz task == TASK: {ep['task'] == policy_wire.TASK}")
    seed = policy_wire.noise_seed(stem)
    predictor.reset(stem, seed)
    start = time.perf_counter()
    first, _ = predictor.infer(obs, frame)
    chunk_ms = (time.perf_counter() - start) * 1000
    predictor.reset(stem, seed)
    second, _ = predictor.infer(obs, frame)
    shape_ok = first.shape == (policy_wire.CHUNK, policy_wire.ACTION_DIM) and first.dtype == np.float32
    finite = bool(np.isfinite(first).all())
    same = bool(np.array_equal(first, second))
    rejection = policy_wire.check_observation({**obs, policy_wire.IMAGE_KEY: np.zeros((400, 640, 3), np.uint8)})
    print(f"load_s {predictor.load_s:.1f}  chunk_ms {chunk_ms:.0f}  device {device}  threads {predictor.metadata['threads']}")
    print(f"noise seed {seed} (noise_seed({stem!r}))")
    print(f"chunk {first.shape} {first.dtype}  finite {finite}  same-seed identical {same}  "
          f"max |diff| {float(np.abs(first - second).max()):.3g}")
    print(f"400x640 image rejected: {bool(rejection)} {rejection}")
    if save_chunk:
        save_chunk.parent.mkdir(parents=True, exist_ok=True)
        np.save(save_chunk, first)
        print(f"first chunk saved to {save_chunk}")
    passed = not problems and shape_ok and finite and (same or device != "cpu") and bool(rejection)
    print("SELF-TEST", "PASS" if passed else "FAIL")
    return passed


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--replay", nargs="+", type=pathlib.Path, help="recorded episodes (npz files and/or folders)")
    mode.add_argument("--checkpoint", type=pathlib.Path, help=".../pretrained_model or its parent")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=policy_wire.DEFAULT_PORT, help="0 = ephemeral port")
    parser.add_argument("--device", choices=("mps", "cpu", "cuda"), default="mps", help="refused while lerobot-train runs")
    parser.add_argument("--force-mps", action="store_true", help="use MPS even while lerobot-train runs")
    parser.add_argument("--allow-during-training", action="store_true",
                        help="read a checkpoint inside a running training's output dir (R4: only with the user's OK)")
    parser.add_argument("--threads", type=int, help="torch CPU threads")
    parser.add_argument("--verbose", action="store_true", help="one line per request and per client")
    parser.add_argument("--self-test", type=pathlib.Path, metavar="NPZ", help="checkpoint mode: test on one recorded frame")
    parser.add_argument("--self-test-frame", type=int, default=0)
    parser.add_argument("--save-chunk", type=pathlib.Path, help="--self-test: save the first chunk (.npy)")
    args = parser.parse_args()
    if args.self_test and not args.checkpoint:
        parser.error("--self-test needs --checkpoint")

    if args.replay:
        try:
            predictor = policy_wire.ReplayPredictor(args.replay)
        except (OSError, ValueError) as error:
            refuse(f"replay sources: {error}")
    else:
        predictor = SmolVLAPredictor(args.checkpoint, args.device, args.threads, args.force_mps, args.allow_during_training)
    if args.self_test:
        raise SystemExit(0 if self_test(predictor, args.self_test, args.self_test_frame, args.device, args.save_chunk) else 1)
    print(f"serving {predictor.metadata}", flush=True)
    try:
        policy_wire.serve(predictor, args.host, args.port, on_listen=functools.partial(print, flush=True), verbose=args.verbose)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()

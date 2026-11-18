"""Ersatz-Policy-Server: openpis echter Server und echte Umrechnungskette, nur ohne π₀.₅-Gewichte (Stufe 2.0).

Zweck: die Leitung Sim <-> Policy-Server auf dem Mac pruefen, bevor es trainierte Gewichte gibt. Die Kette ist dieselbe wie
beim Server mit Checkpoint (openpi policy_config.create_trained_policy) und laeuft durch openpis Policy-Klasse (JAX):
  Beobachtung -> G1BraincoInputs -> Normalize -> Bildgroesse/Tokenizer/Auffuellen -> Modell
             -> Unnormalize -> AbsoluteActions -> G1BraincoOutputs -> 26-D-Aktionen
Ersetzt ist nur das Modell: StandInModel antwortet fuer den ganzen Aktions-Chunk mit "Startpose halten"
(action_space.READY_POSE). Weil die Antwort bekannt ist, kann die Sim pruefen, dass die Umrechnung nichts verfaelscht.
Normierung: neutrale Statistik, solange es keinen Datensatz gibt. Kann nicht greifen und lernt nichts.
"""
import logging

import flax.nnx as nnx
import jax.numpy as jnp
import numpy as np
from openpi import transforms as _transforms
from openpi.policies import policy as _policy
from openpi.serving import websocket_policy_server
from openpi.shared import normalize

from g1sim import action_space
from openpi_g1 import config


class StandInModel(nnx.Module):
    """Statt π₀.₅: liefert die Modellausgabe, die nach openpis Ausgabe-Umrechnung genau `target` ergibt."""

    def __init__(self, target, norm_stats, delta_mask, action_horizon, action_dim):
        self.target = tuple(float(v) for v in target)
        self.delta_mask = tuple(bool(v) for v in delta_mask)
        self.state_quantiles = (tuple(norm_stats["state"].q01.tolist()), tuple(norm_stats["state"].q99.tolist()))
        self.action_quantiles = (tuple(norm_stats["actions"].q01.tolist()), tuple(norm_stats["actions"].q99.tolist()))
        self.action_horizon, self.action_dim = action_horizon, action_dim

    def sample_actions(self, rng, observation, **kwargs):
        dim = len(self.target)
        # Umkehrung von Normalize/Unnormalize (Quantile, openpi/transforms.py) und AbsoluteActions
        q01, q99 = (jnp.asarray(q) for q in self.state_quantiles)
        state = (observation.state[..., :dim] + 1.0) / 2.0 * (q99 - q01 + 1e-6) + q01
        delta = jnp.asarray(self.target) - jnp.where(jnp.asarray(self.delta_mask), state, 0.0)
        q01, q99 = (jnp.asarray(q) for q in self.action_quantiles)
        normed = (delta - q01) / (q99 - q01 + 1e-6) * 2.0 - 1.0
        actions = jnp.zeros((observation.state.shape[0], self.action_horizon, self.action_dim), dtype=jnp.float32)
        return actions.at[..., :dim].set(normed[:, None, :])


def neutral_norm_stats(dim=action_space.DIM):
    """Normierung, die nichts veraendert (Quantile -1/1). Echte Statistik kommt spaeter aus dem Datensatz (run.py norm-stats)."""
    one = np.ones(dim, dtype=np.float32)
    stats = normalize.NormStats(mean=np.zeros(dim, dtype=np.float32), std=one, q01=-one, q99=one)
    return {"state": stats, "actions": stats}


def create_policy(train_config=None, norm_stats=None, target=action_space.READY_POSE):
    """openpi-Policy mit echter Umrechnungskette und Ersatz-Modell."""
    train_config = train_config or config.register()
    data_config = train_config.data.create(train_config.assets_dirs, train_config.model)
    if not data_config.use_quantile_norm:
        raise ValueError("Das Ersatz-Modell kehrt die Quantil-Normierung von π₀.₅ um - andere Normierung nicht vorgesehen")
    norm_stats = norm_stats or neutral_norm_stats()
    model = StandInModel(target, norm_stats, action_space.DELTA_MASK, train_config.model.action_horizon,
                         train_config.model.action_dim)
    # Reihenfolge wie openpi policy_config.create_trained_policy
    return _policy.Policy(
        model,
        transforms=[
            _transforms.InjectDefaultPrompt(None),
            *data_config.data_transforms.inputs,
            _transforms.Normalize(norm_stats, use_quantiles=True),
            *data_config.model_transforms.inputs,
        ],
        output_transforms=[
            *data_config.model_transforms.outputs,
            _transforms.Unnormalize(norm_stats, use_quantiles=True),
            *data_config.data_transforms.outputs,
        ],
        metadata={**train_config.policy_metadata, "stand_in": True},
    )


def serve(host="127.0.0.1", port=8000):
    policy = create_policy()
    logging.info("Ersatz-Policy-Server (ohne Gewichte) auf %s:%d", host, port)
    websocket_policy_server.WebsocketPolicyServer(policy, host=host, port=port, metadata=policy.metadata).serve_forever()

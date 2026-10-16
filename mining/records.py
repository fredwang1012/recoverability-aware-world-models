"""
records.py - the three separate objects from PDF section 12.

  case_spec         INTENT.  What the construction is trying to show.  Never a label.
  execution_record  MEASURED.  Everything a run produced: margins, evidence, timestamps, events, provenance.
  learning_example  FEATURES ONLY.  What a model may see: the delivered observation history and the declared contract ids.
                    Hidden metadata (world id/mode, role name, blueprint, case id, intended outcome, future coordinates)
                    must never appear here; validate_records.hidden_leaks() enforces it.

evidence_class comes from contract.classify on the stored policy x world matrix R (None = unresolved/NaN, never 0).
"""
import hashlib
import json
import math
import subprocess
from dataclasses import dataclass, field, asdict
from typing import Optional

import numpy as np

import contract

SCHEMA_VERSION = "0.2"
EVIDENCE_SCOPES = ("analytic", "reference_model", "mock_replay", "carla_finite_replay")
HIDDEN_KEYS = {"world", "world_id", "world_mode", "mode", "role_name", "blueprint", "blueprint_id", "case_id",
               "intended_outcome", "future", "future_xy", "actor_mode", "split_group", "family_id", "varied_factor"}


def git_sha():
    try:
        return subprocess.check_output(["git", "rev-parse", "--short", "HEAD"], text=True,
                                       stderr=subprocess.DEVNULL).strip()
    except Exception:
        return "unknown"


def checksum(obj) -> str:
    return hashlib.sha256(json.dumps(obj, sort_keys=True, default=str).encode()).hexdigest()[:16]


@dataclass
class CaseSpec:
    case_id: str
    family_id: str
    split_group: str
    site_id: str
    intended_outcome: str                     # INTENT only
    contract: dict                            # observation_mode, world_set_id, policy_library_id, physics_profile_id, dt, eps
    construction: dict                        # sweep values, reserve, gap rule ...
    terminal: dict                            # mode, post_maneuver_seconds
    required_checks: list = field(default_factory=list)
    parent_family_id: Optional[str] = None
    varied_factor: Optional[str] = None
    status: str = "NOT_EXECUTED"
    schema_version: str = SCHEMA_VERSION


def _ser(x):
    """JSON-safe number: None stays None (unresolved), -inf -> '-inf', NaN -> None."""
    if x is None:
        return None
    if isinstance(x, float) and not math.isfinite(x):
        return None if math.isnan(x) else ("-inf" if x < 0 else "inf")
    return x


@dataclass
class ExecutionRecord:
    case_id: str
    family_id: str
    split_group: str
    backend: str                              # "carla" | "mock" | "reference"
    theta: dict
    policies: list                            # policy names, in row order of R
    worlds: list                              # world names, in column order of R
    R_raw: list                               # [policy][world] label-grade G, None = unresolved
    R_norm: list                              # same, search-grade G (finite) or None
    terminal_ok: list                         # [policy][world] Boolean or None
    margin_config: dict
    evidence_scope: str
    seed: int
    policy_descriptors: list = field(default_factory=list)       # [{name, version, params}]
    runtime: dict = field(default_factory=dict)                  # server build, client, map, map hash
    physics_profile_id: Optional[str] = None
    transforms: dict = field(default_factory=dict)               # resolved actor transforms + blueprint ids
    geometry: dict = field(default_factory=dict)                 # measured gap, footprints
    timestamps: dict = field(default_factory=dict)               # capture/delivery/decision/command/response
    events: dict = field(default_factory=dict)                   # collisions, road events
    traces: dict = field(default_factory=dict)                   # witness / refutation trace refs
    coverage: dict = field(default_factory=dict)                 # {"run": n, "total": n}
    termination_cause: str = "horizon"
    invalid_reasons: list = field(default_factory=list)
    parent_family_id: Optional[str] = None
    varied_factor: Optional[str] = None
    code_version: str = field(default_factory=git_sha)
    schema_version: str = SCHEMA_VERSION
    # derived by finalize()
    V: Optional[float] = None
    V_hind: Optional[float] = None
    info_gap: Optional[float] = None
    evidence_class: Optional[str] = None
    best_policy: Optional[str] = None
    checksum: Optional[str] = None

    def finalize(self, eps=0.0):
        """Derive V, V_hind, info_gap, evidence_class from R_raw via contract.classify; INVALID_RUN overrides."""
        R = np.array([[np.nan if x is None else x for x in row] for row in self.R_raw], float)
        c = contract.classify(R, eps)
        self.V = None if not np.isfinite(c["V"]) else float(c["V"])
        self.V_hind = None if not np.isfinite(c["V_hind"]) else float(c["V_hind"])
        self.info_gap = None if not np.isfinite(c["info_gap"]) else float(c["info_gap"])
        self.evidence_class = "INVALID_RUN" if self.invalid_reasons else c["evidence"]
        self.best_policy = None if c["best_policy"] is None else self.policies[c["best_policy"]]
        self.coverage = {"run": int((~np.isnan(R)).sum()), "total": int(R.size)}
        self.checksum = checksum({k: v for k, v in self.to_json().items() if k != "checksum"})
        return self

    def to_json(self):
        d = asdict(self)
        for k in ("R_raw", "R_norm"):
            d[k] = [[_ser(x) for x in row] for row in d[k]]
        return json.loads(json.dumps(d, default=str))


@dataclass
class LearningExample:
    """Features only.  `history` = delivered observations [{t_capture, t_available, obs}] plus applied controls."""
    example_id: str
    history: list
    applied_controls: list
    contract_ids: dict                        # policy_library_id, world_set_id (ids are allowed, world index is not)
    label: dict                               # TARGETS (masked R, V, evidence_class), kept apart from features
    schema_version: str = SCHEMA_VERSION

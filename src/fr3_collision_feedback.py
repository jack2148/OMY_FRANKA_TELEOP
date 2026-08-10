"""MuJoCo FR3 contact detection and haptic-feedback primitives.

This module deliberately has no ROS publisher/subscriber and no hardware
current command.  The future bridge can call :class:`Fr3CollisionMonitor`
inside the MuJoCo loop, then serialize ``CollisionFeedback`` to its transport.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Iterable

import numpy as np


@dataclass(frozen=True)
class CollisionConfig:
    """Contact filtering and force-reflection parameters."""

    wall_geom_names: frozenset[str] = frozenset({"fr3_front_wall_geom"})
    minimum_penetration: float = 0.0
    force_deadband: float = 0.5
    force_limit: float = 20.0
    resistance_gain: float = 1.0
    resistance_damping: float = 0.0


@dataclass
class ContactSample:
    """One filtered MuJoCo contact in world coordinates."""

    geom1: str
    geom2: str
    distance: float
    position: np.ndarray
    normal: np.ndarray
    force: np.ndarray
    torque: np.ndarray


@dataclass
class CollisionFeedback:
    """Bridge-safe feedback payload produced on every simulation tick."""

    collision: bool = False
    wall_contact: bool = False
    force_world: np.ndarray = field(default_factory=lambda: np.zeros(3))
    normal_world: np.ndarray = field(default_factory=lambda: np.zeros(3))
    resistance_force_world: np.ndarray = field(default_factory=lambda: np.zeros(3))
    contacts: list[ContactSample] = field(default_factory=list)

    def as_dict(self) -> dict[str, Any]:
        """Return a JSON/ROS-message-friendly representation."""

        return {
            "collision": self.collision,
            "wall_contact": self.wall_contact,
            "force_world": self.force_world.tolist(),
            "normal_world": self.normal_world.tolist(),
            "resistance_force_world": self.resistance_force_world.tolist(),
            "contacts": [
                {
                    "geom1": c.geom1,
                    "geom2": c.geom2,
                    "distance": c.distance,
                    "position": c.position.tolist(),
                    "normal": c.normal.tolist(),
                    "force": c.force.tolist(),
                }
                for c in self.contacts
            ],
        }


def _geom_name(model: Any, geom_id: int) -> str:
    return str(model.geom(geom_id).name or f"geom_{geom_id}")


def _world_force(contact: Any, wrench: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Convert MuJoCo contact-frame force/torque into world coordinates."""

    # MuJoCo stores the contact frame as three row vectors.  The transpose
    # maps a vector from the contact frame into world coordinates.
    frame = np.asarray(contact.frame, dtype=float).reshape(3, 3)
    return frame.T @ wrench[:3], frame.T @ wrench[3:]


class Fr3CollisionMonitor:
    """Extract wall contacts and calculate a bounded resistance force.

    ``update`` is intended to run immediately after ``mujoco.mj_step``.  It
    uses only MuJoCo state; a bridge can publish the returned dataclass later.
    """

    def __init__(self, config: CollisionConfig | None = None) -> None:
        self.config = config or CollisionConfig()

    def update(self, model: Any, data: Any) -> CollisionFeedback:
        import mujoco  # Imported lazily so serialization/tests need no MuJoCo.

        samples: list[ContactSample] = []
        total_force = np.zeros(3, dtype=float)
        total_normal = np.zeros(3, dtype=float)

        for index in range(int(data.ncon)):
            contact = data.contact[index]
            name1 = _geom_name(model, int(contact.geom1))
            name2 = _geom_name(model, int(contact.geom2))
            if not (
                name1 in self.config.wall_geom_names
                or name2 in self.config.wall_geom_names
            ):
                continue
            distance = float(contact.dist)
            if distance > self.config.minimum_penetration:
                continue

            wrench = np.zeros(6, dtype=float)
            mujoco.mj_contactForce(model, data, index, wrench)
            force, torque = _world_force(contact, wrench)
            norm = float(np.linalg.norm(force))
            if norm < self.config.force_deadband:
                continue

            normal = np.asarray(contact.frame, dtype=float).reshape(3, 3)[0]
            normal /= max(float(np.linalg.norm(normal)), 1e-12)
            sample = ContactSample(
                geom1=name1,
                geom2=name2,
                distance=distance,
                position=np.asarray(contact.pos, dtype=float).copy(),
                normal=normal.copy(),
                force=force.copy(),
                torque=torque.copy(),
            )
            samples.append(sample)
            total_force += force
            total_normal += normal * norm

        magnitude = float(np.linalg.norm(total_force))
        if magnitude > self.config.force_limit:
            total_force *= self.config.force_limit / magnitude
        normal_norm = float(np.linalg.norm(total_normal))
        if normal_norm > 1e-12:
            total_normal /= normal_norm

        # Apply resistance opposite to the force that FR3 receives from the
        # wall.  The bridge decides how this Cartesian vector is reflected to
        # OMY (for example J_omy.T @ resistance_force_world).
        resistance = -self.config.resistance_gain * total_force
        return CollisionFeedback(
            collision=bool(samples),
            wall_contact=bool(samples),
            force_world=total_force,
            normal_world=total_normal,
            resistance_force_world=resistance,
            contacts=samples,
        )


def cartesian_force_to_joint_torque(
    position_jacobian: np.ndarray,
    force_world: Iterable[float],
    *,
    torque_limit: float | None = None,
) -> np.ndarray:
    """Map a world-frame Cartesian force to joint torque via ``JᵀF``.

    This function only computes a bounded numerical command.  It intentionally
    does not send effort/current to a robot; the hardware controller remains a
    separate safety-reviewed bridge component.
    """

    jacobian = np.asarray(position_jacobian, dtype=float)
    if jacobian.ndim != 2 or jacobian.shape[0] != 3:
        raise ValueError("position_jacobian must have shape (3, n_joints)")
    torque = jacobian.T @ np.asarray(tuple(force_world), dtype=float)
    if torque_limit is not None:
        torque = np.clip(torque, -abs(torque_limit), abs(torque_limit))
    return torque


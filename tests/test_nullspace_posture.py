import numpy as np


def dls_pseudoinverse(jacobian, damping):
    task_dim = jacobian.shape[0]
    regularized = (
        jacobian @ jacobian.T
        + damping**2 * np.eye(task_dim)
    )
    return jacobian.T @ np.linalg.solve(
        regularized,
        np.eye(task_dim),
    )


def nullspace_velocity(
    jacobian,
    q_current,
    q_reference,
    gain,
    damping,
    enabled,
):
    if not enabled:
        return np.zeros_like(q_current)

    jacobian_pinv = dls_pseudoinverse(jacobian, damping)
    projector = np.eye(q_current.shape[0]) - jacobian_pinv @ jacobian
    return projector @ (gain * (q_reference - q_current))


def test_nullspace_posture_shape_and_task_leakage():
    jacobian = np.array([
        [1.0, 0.2, 0.0, 0.1, 0.0, 0.0, 0.3],
        [0.0, 1.0, 0.1, 0.0, 0.2, 0.0, 0.0],
        [0.1, 0.0, 1.0, 0.0, 0.0, 0.2, 0.1],
    ])
    q_current = np.arange(1, 8, dtype=float) * 0.1
    q_reference = np.zeros(7)
    qdot_null = nullspace_velocity(
        jacobian,
        q_current,
        q_reference,
        gain=0.1,
        damping=0.05,
        enabled=True,
    )

    assert jacobian.shape == (3, 7)
    assert qdot_null.shape == (7,)
    assert np.linalg.norm(jacobian @ qdot_null) < (
        0.01 * np.linalg.norm(qdot_null)
    )


def test_disabled_nullspace_is_zero():
    jacobian = np.eye(3, 7)
    q_current = np.ones(7)
    q_reference = np.zeros(7)
    qdot_task = np.arange(1, 8, dtype=float)
    qdot_null = nullspace_velocity(
        jacobian,
        q_current,
        q_reference,
        gain=0.1,
        damping=0.05,
        enabled=False,
    )

    assert np.array_equal(qdot_null, np.zeros_like(qdot_task))
    assert np.array_equal(qdot_task + qdot_null, qdot_task)

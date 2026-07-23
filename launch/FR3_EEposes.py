#!/usr/bin/python3

from pathlib import Path

import mujoco


MODEL_PATH = (
    Path(__file__).resolve().parents[1]
    / "mujoco_menagerie"
    / "franka_fr3"
    / "scene.xml"
)

model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
data = mujoco.MjData(model)

home_id = mujoco.mj_name2id(
    model,
    mujoco.mjtObj.mjOBJ_KEY,
    "home",
)

if home_id < 0:
    raise ValueError("home keyframe not found")

mujoco.mj_resetDataKeyframe(model, data, home_id)
mujoco.mj_forward(model, data)

fr3_ee_site_id = mujoco.mj_name2id(
    model,
    mujoco.mjtObj.mjOBJ_SITE,
    "attachment_site",
)

if fr3_ee_site_id < 0:
    raise ValueError("attachment_site not found")

p_fr3_0 = data.site_xpos[fr3_ee_site_id].copy()
R_fr3_0 = data.site_xmat[fr3_ee_site_id].reshape(3, 3).copy()

fr3_home = (p_fr3_0, R_fr3_0)
print("FR3 initial position:", p_fr3_0)
print("FR3 initial rotation:\n", R_fr3_0)
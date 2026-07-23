import mujoco
import numpy as np

model = mujoco.MjModel.from_xml_path("omy.xml")
data = mujoco.MjData(model)

# XML keyframe의 home configuration 적용
home_id = mujoco.mj_name2id(
    model,
    mujoco.mjtObj.mjOBJ_KEY,
    "home",
)

if home_id < 0:
    raise ValueError("home keyframe not found")

mujoco.mj_resetDataKeyframe(model, data, home_id)
mujoco.mj_forward(model, data)

omy_ee_site_id = mujoco.mj_name2id(
    model,
    mujoco.mjtObj.mjOBJ_SITE,
    "omy_ee_site",
)

if omy_ee_site_id < 0:
    raise ValueError("omy_ee_site not found")

p_omy_0 = data.site_xpos[omy_ee_site_id].copy()
R_omy_0 = data.site_xmat[omy_ee_site_id].reshape(3, 3).copy()

omy_home = (p_omy_0, R_omy_0)

print("OMY initial position:", p_omy_0)
print("OMY initial rotation:\n", R_omy_0)
import numpy as np
import pinocchio as pin
from marsdog_control.control.wbc import WholeBodyController, WbcConfig
from marsdog_control.control.nmpc_reduced_model import default_urdf_path

wbc = WholeBodyController(default_urdf_path(), WbcConfig())
q = pin.neutral(wbc.model)
q[2] = 0.24 # base z
q[7:] = 0.0 # joints
pin.computeAllTerms(wbc.model, wbc.data, q, np.zeros(wbc.nv))
h = wbc.data.nle
print("h for calf:", h[6+3]) # fl_calf is joint 3

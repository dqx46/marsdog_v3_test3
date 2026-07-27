_FRONT_HIP_X = 0.2714
_REAR_HIP_X = -0.0417
_HALF_TRACK_FRONT = 0.040
_HALF_TRACK_REAR = 0.034

def get_hip_offsets(leg):
    if leg == 'fl': return _FRONT_HIP_X, _HALF_TRACK_FRONT
    if leg == 'fr': return _FRONT_HIP_X, -_HALF_TRACK_FRONT
    if leg == 'rl': return _REAR_HIP_X, _HALF_TRACK_REAR
    if leg == 'rr': return _REAR_HIP_X, -_HALF_TRACK_REAR

print(get_hip_offsets('fl'))

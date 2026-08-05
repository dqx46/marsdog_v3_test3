# Legacy Files

Flat-layout Python shims and old tool launchers live under
``legacy/mocap_to_real/``. Deploy resources (JSON / sounds / udev) stay in
root ``mocap_to_real/`` — see that README.

``marsdog_control.compat.ensure_legacy_path()`` puts ``legacy/mocap_to_real``
on ``sys.path`` so sunk modules that still do ``from joint_config import …``
resolve to the same objects as ``marsdog_control.config.joints``.

Prefer package entry points:

- ``python -m marsdog_control.apps.walk``
- ``python -m marsdog_control.apps.sim.sim_walk``
- ``python -m marsdog_control.apps.tools…``

Do not add new code under ``legacy/mocap_to_real/``. New tools go in
``src/marsdog_control/apps/tools/``.

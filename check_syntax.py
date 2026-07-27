import py_compile
try:
    py_compile.compile('/home/jetson/new_marsdog_v3/mocap_to_real/gait_controller.py', doraise=True)
    print("Syntax OK")
except Exception as e:
    print(e)

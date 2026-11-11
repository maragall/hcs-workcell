"""Demo: the plate goes from hotel slot A1 to the microscope and back.

    python demo.py
"""
from polar import Polar

arm = Polar()

arm.move("A1", "microscope")
arm.move("microscope", "A1")

arm.close()
print("READY")

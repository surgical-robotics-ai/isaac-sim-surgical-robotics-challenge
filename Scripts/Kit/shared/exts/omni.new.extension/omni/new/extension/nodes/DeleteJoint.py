"""
This is the implementation of the OGN node defined in DeleteJoint.ogn
"""

# Array or tuple values are accessed as numpy arrays so you probably need this import
import numpy
import omni
from omni.physx.scripts import utils
from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, Sdf, Gf, Tf

class DeleteJoint:
    """
         Deletes a joint dynamically during simulation
    """
    @staticmethod
    def compute(db) -> bool:
        """Compute the outputs from the current input"""

        try:
            # With the compute in a try block you can fail the compute by raising an exception
            if db.inputs.new_input:
                stage = omni.usd.get_context().get_stage()
                stage.RemovePrim(db.inputs.new_input)

        except Exception as error:
            # If anything causes your compute to fail report the error and return False
            db.log_error(str(error))
            return False

        # Even if inputs were edge cases like empty arrays, correct outputs mean success
        return True

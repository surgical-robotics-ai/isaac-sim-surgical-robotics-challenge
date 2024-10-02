"""
This is the implementation of the OGN node defined in CreateJoint.ogn
"""

# Array or tuple values are accessed as numpy arrays so you probably need this import
import numpy
import omni
from omni.physx.scripts import utils
from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, Sdf, Gf, Tf

class CreateJoint:
    """
         Creates a D6 joint dynamically while simulation is running
    """
    @staticmethod
    def compute(db) -> bool:
        """Compute the outputs from the current input"""

        try:
            # With the compute in a try block you can fail the compute by raising an exception
            stage = omni.usd.get_context().get_stage()
            from_prim_path = db.inputs.Body1
            to_prim_path = db.inputs.Body2

            from_prim = stage.GetPrimAtPath(from_prim_path)
            to_prim = stage.GetPrimAtPath(to_prim_path)

            prim = utils.createJoint(stage, "Fixed", from_prim, to_prim)
            temp = str(prim)
            temp1 = temp.split("<")[1]
            temp2 = temp1.split(">")[0]
            temp3 = temp2.split("/")[5]
            if temp3 == "FixedJoint":
                db.outputs.NewJoint = temp2
            else:
                stage.RemovePrim(temp2)
            
        except Exception as error:
            # If anything causes your compute to fail report the error and return False
            db.log_error(str(error))
            return False

        # Even if inputs were edge cases like empty arrays, correct outputs mean success
        return True

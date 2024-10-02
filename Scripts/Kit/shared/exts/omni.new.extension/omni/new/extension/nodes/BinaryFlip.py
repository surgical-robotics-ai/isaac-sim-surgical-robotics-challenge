"""
This is the implementation of the OGN node defined in BinaryFlip.ogn
"""

# Array or tuple values are accessed as numpy arrays so you probably need this import
import numpy


class BinaryFlip:
    """
         Binary Flip with input
    """
    @staticmethod
    def compute(db) -> bool:
        """Compute the outputs from the current input"""

        try:
            # With the compute in a try block you can fail the compute by raising an exception
            if db.inputs.Condition:
                db.inputs.Condition = False
                db.outputs.Condition = db.inputs.Condtion
            else:
                db.inputs.Condition = True
                db.outputs.Condition = db.inputs.Condition
        except Exception as error:
            # If anything causes your compute to fail report the error and return False
            db.log_error(str(error))
            return False

        # Even if inputs were edge cases like empty arrays, correct outputs mean success
        return True

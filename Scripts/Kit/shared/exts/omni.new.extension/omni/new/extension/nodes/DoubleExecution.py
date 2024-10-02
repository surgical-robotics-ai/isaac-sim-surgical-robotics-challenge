"""
This is the implementation of the OGN node defined in DoubleExecution.ogn
"""

# Array or tuple values are accessed as numpy arrays so you probably need this import
import numpy


class DoubleExecution:
    """
         Output execution for two different input execution nodes
    """
    @staticmethod
    def compute(db) -> bool:
        """Compute the outputs from the current input"""

        try:
            # With the compute in a try block you can fail the compute by raising an exception
            pass
        except Exception as error:
            # If anything causes your compute to fail report the error and return False
            db.log_error(str(error))
            return False

        # Even if inputs were edge cases like empty arrays, correct outputs mean success
        return True

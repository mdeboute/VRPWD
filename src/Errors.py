class ReferentialError(Exception):
    """ReferentialError class for error while choosing the refential in projected function"""

    def __init__(self, message):
        self.message = message

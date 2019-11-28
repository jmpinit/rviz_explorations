class UID32(object):
    def __init__(self):
        self.idsByNameSpace = {}

    def next(self, ns=None):
        if not ns in self.idsByNameSpace:
            self.idsByNameSpace[ns] = 0

        id = self.idsByNameSpace[ns]
        self.idsByNameSpace[ns] += 1
        return id

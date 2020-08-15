

class POI:
    def __init__(self, points):
        if isinstance(points, list):
            self.points = points
        else:
            self.points = [points]

    def __str__(self):
        return f"POI({self.points})"

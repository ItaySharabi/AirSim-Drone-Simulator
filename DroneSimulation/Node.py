class Waypoint:

    def __init__(self, pos, data):
        self.pos = pos
        self.data = data
        print(f'New waypoint: {self.__str__()}')

    def __str__(self):
        return f'Waypoint: [{self.pos}]'

    def __repr__(self):
        return f'Waypoint: [{self.pos}]'
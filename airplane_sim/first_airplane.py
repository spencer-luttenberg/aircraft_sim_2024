class Airplane:
    def __init__(self):
        self.lat = 36
        self.lon = 30
        self.alt = 0
    def print_position(self):
        print("Lat {} lon {} alt {}".format(self.lat, self.lon, self.alt))
        
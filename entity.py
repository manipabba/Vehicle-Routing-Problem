class Entity:
    def __init__(self):
        self.fitness = 0
        self.gnome = ""

    def __str__(self):
        return str(self.fitness) + ", " + self.gnome
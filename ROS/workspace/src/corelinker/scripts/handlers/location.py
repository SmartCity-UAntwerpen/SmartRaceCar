
class Location:

    def __init__(self, posx=0, posy=0, posz=0, orx=0, ory=0, orz=0, orw=0):
        self.posx = posx
        self.posy = posy
        self.posz = posz

        self.orx = orx
        self.ory = ory
        self.orz = orz
        self.orw = orw

    def set_position(self, posx, posy, posz):
        self.posx = posx
        self.posy = posy
        self.posz = posz

    def set_orientation(self, orx, ory, orz, orw):
        self.orx = orx
        self.ory = ory
        self.orz = orz
        self.orw = orw
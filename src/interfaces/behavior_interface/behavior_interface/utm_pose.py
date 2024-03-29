"""
Author: Jason Hughes
Date: July 2023
About: data type for utm coordinates
"""

class UtmPose():

    def __init__(self):

        self.easting_ = float()
        self.northing_ = float()
        self.altitude_ = float()
        self.zone_ = str()

    @property
    def easting(self):
        return self.easting_

    @easting.setter
    def easting(self, e):
        self.easting_ = e

    @property
    def northing(self):
        return self.northing_

    @northing.setter
    def northing(self, n):
        self.northing_ = n

    @property
    def altitude(self):
        return self.altitude_

    @altitude.setter
    def altitude(self, a):
        self.altitude_ = a
    
    @property
    def zone(self):
        return self.zone_

    @zone.setter
    def zone(self, z):
        self.zone_ = z

"""
Author: Jason Hughes
Date: May 2023
About: Data type for utm coordinates
"""

class UTMPose():

    def __init__(self):

        self.easting_ = float()
        self.northing_ = float()
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
    def zone(self):
        return self.zone_

    @zone.setter
    def zone(self, z):
        self.zone_ = z

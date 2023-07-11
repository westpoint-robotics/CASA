""" 
Author: Jason Hughes
Date: July 2023
About: Data type for global pose
"""

from behavior_interface.ll_utm_conversion import LLtoUTM, UTMtoLL

class GlobalPose():

    def __init__(self):
        self.latitude_ = float()
        self.longitude_ = float()
        self.altitude_ = float()

        self.easting_ = float()
        self.northing_ = float()
        self.zone_ = str()

    @property
    def latitude(self):
        return self.latitude_

    @latitude.setter
    def latitude(self, l):
        self.latitude_ = l

    @property
    def longitude(self):
        return self.longitude_

    @longitude.setter
    def longitude(self, l):
        self.longitude_ = l

    @property
    def altitude(self):
        return self.altitude_

    @altitude.setter
    def altitude(self, a):
        self.altitude_ = a

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

    def initializeFromNavSatFix(self, m):
        self.latitude_ = m.latitude
        self.longitude_ = m.longitude
        self.altitude_ = m.altitude

        (z, e, n) = LLtoUTM(23, m.latitude, m.longitude)

        self.easting_ = e
        self.northing_ = n
        self.zone_ = z

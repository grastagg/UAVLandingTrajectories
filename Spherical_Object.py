import numpy as np

class Spherical_Object:
  def __init__(self, center, radius):
    self.center_x = center[0]
    self.center_y = center[1]
    self.center_z = center[2]
    self.radius = radius

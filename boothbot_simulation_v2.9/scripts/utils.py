from __future__ import division, print_function

class CoordConverter(object):
    """
    Convert coords between maps and Gazebo world.
    """
    def __init__(self, map_width, map_height):
        """
        Initialize with map's size.

        Parameters
        ----------
        map_width: float
            The width of map

        map_height: float
            The height of map
        """
        # FIXME: Need a robust way to calculate origin point.
        # This two offset values only apply NVCC world.
        x_offset, y_offset = 2.5, 1.5
        self.map_world_origin_x = - map_width / 2.0 + x_offset
        self.map_world_origin_y = - map_height / 2.0 + y_offset

    def map_to_world(self, mx, my):
        """
        Convert map coords to Gazebo world coords.

        Parameters
        ----------
        mx: float
            X coord in map

        my: float
            Y coord in map
        
        Return
        ------
        (float, float):
            X and Y coords in Gazebo world
        """
        wx = self.map_world_origin_x + mx
        wy = self.map_world_origin_y + my
        return wx, wy
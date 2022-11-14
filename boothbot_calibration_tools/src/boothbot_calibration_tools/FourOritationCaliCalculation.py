import math
import numpy as np
###################
d = 0.04
a = -0.018
#GS to painter
l = -0.392


###################
# Calculator

zero_offset_radians = np.arctan2(-a/2,abs(l + d/2))

mark_offset = l - a / 2 / np.sin(zero_offset_radians)
current_mark_offset = l + mark_offset
print("mark_offset:",a / 2 / np.sin(zero_offset_radians))

print(" zero_offset_radians: ", zero_offset_radians)
print(" zero_offset_degree: ", zero_offset_radians * 180 / np.pi)
print("mark_offset: ", mark_offset)


import numpy as np

# CALCULATION SHOULD BE IN CENTIMETER
# !!CM!!

# Millimeters 
# 10  = 1cm
# 06  = 0.6cm
# 07  = 0.7cm
# 08  = 0.8cm
# 09  = 0.9cm
# 04  = 0.4cm
# 11  = 1.1cm
# 05  = 0.5cm
# 13  = 1.3cm
# 07  = 0.7cm
# NEED THE COORDINATES
#(0.5, -0.8)
#(-0.2, -0.7)
#(-0.2, -0.6)
#(-0.5, -0.7)
#(-0.8, -0.6)
#(-0.3, 0.1)
#(-0.4, 0.3)
#(-0.9, 0.5)
#(-1.0, 0.8)
#(-1.3, 1.1)

x = [0.5,-0.2, -0.2, -0.5, -0.8, -0.3, -0.4, -0.9, -1.0, -1.3]
y = [-0.8, -0.7, -0.6, -0.7, -0.6, 0.1, 0.3, 0.5, 0.8, 1.1]
print(np.cov([x,y]))
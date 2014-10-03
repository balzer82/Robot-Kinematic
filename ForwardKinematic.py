# -*- coding: utf-8 -*-
# <nbformat>3.0</nbformat>

# <codecell>

import numpy as np

# <codecell>

def calcT(y, p, r, t):
    
    T = np.eye(4)
    
    # from Degree to Radians
    y = y*np.pi/180.0
    p = p*np.pi/180.0
    r = r*np.pi/180.0
    
    cr = np.cos(r)
    sr = np.sin(r)
    cp = np.cos(p)
    sp = np.sin(p)
    cy = np.cos(y)
    sy = np.sin(y)
    
    Rr = np.matrix([[1.0, 0.0, 0.0],[0.0, cr, -sr],[0.0, sr, cr]])
    Rp = np.matrix([[cp, 0.0, sp],[0.0, 1.0, 0.0],[-sp, 0.0, cp]])
    Ry = np.matrix([[cy, -sy, 0.0],[sy, cy, 0.0],[0.0, 0.0, 1.0]])

    T[np.ix_([0,1,2],[0,1,2])] = Ry*Rp*Rr

    # Translationsteil
    T[np.ix_([0],[3])] = t
    
    return T

# <codecell>

conf = {}

# <codecell>

T = np.eye(4)
for i in range(5):
    T = calcT(0.0,0.0,0.0,i)
    print T

# <codecell>

endeffector = T*np.array([0.0, 0.0, 0.0, 1.0])
endeffector

# <codecell>

import sympy
import sympybotics

# <codecell>

rbtdef = sympybotics.RobotDef('Example Robot', # robot name
                            [(0, 0, 0, 0),  # list of tuples with Denavit-Hartenberg parameters
                            ( 'pi/2', 0, 0, 0)], # (alpha, a, d, theta)
                            dh_convention='standard' # either 'standard' or 'modified'
                            )

# <codecell>

rbtdef.frictionmodel = {'Coulomb', 'viscous'} # options are None or a combination of 'Coulomb', 'viscous' and 'offset'

# <codecell>

rbtdef.dynparms()

# <codecell>

rbt = sympybotics.RobotDynCode(rbtdef, verbose=True)

# <codecell>

rbt.geo.z

# <codecell>

rbt.kin.J[-1]

# <codecell>



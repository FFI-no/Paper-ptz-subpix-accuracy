# Copyright (c) 2025 Norwegian Defence Research Establishment (FFI)

import numpy as np


def get_r_number(lhs, rhs):
    cov = np.cov(lhs, rhs)

    return cov[1, 0]/np.sqrt(cov.diagonal().prod())

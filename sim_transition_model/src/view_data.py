#!/usr/bin/env python

import numpy as np
import time
import random
import pickle
import os.path
import matplotlib.pyplot as plt
from transition_experience import *




def main():
    texp = transition_experience(discrete=True)

    texp.save_to_file()

    # texp.divide_and_save(n = 750)

    texp.plot_data()

    return 1


if __name__=='__main__':
    main()

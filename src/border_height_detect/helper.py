import numpy as np
import matplotlib.pyplot as plt

uncut_plane = np.loadtxt("uncut_plane.txt")
standard_line = np.loadtxt("standard_line_file.txt")
ground_plane = np.loadtxt("ground_plane_file.txt")

mean_uncut_plane = np.mean(uncut_plane, 0)
mean_standard_line = np.mean(standard_line, 0)
mean_ground_plane = np.mean(ground_plane, 0)



pass







import numpy as np
import glob


bin_path = '/home/nika/Downloads/Merged-LDR-All_4/Merged-LDR-All/*.bin'

bin_path_list = glob.glob(bin_path)

for bin_path in bin_path_list:
    scan = (np.fromfile(bin_path, dtype=np.float32)).reshape(-1, 6)
    print(scan)
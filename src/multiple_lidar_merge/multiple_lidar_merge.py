import os
import argparse
import shutil
import glob
import csv

import yaml
import numpy as np
import pickle

class Merge:

    def __init__(self, data_path):

        print('----- initialization')

        self.dataDirPath = data_path + '/02_convert/2022-09-14_10-09-42_ADCV1-ADS-LC1/*/*/'
        self.mergeDirPath = data_path + '/03_merge/'
        self.ts_sync_csv_path = data_path + '2022-09-14_10-09-42_TS-Sync.csv'

        print('----- dataDirPath: ', self.dataDirPath)
        print('----- mergePath: ', self.mergeDirPath + '\n')

    def mergeLidar(self):
        
        if os.path.exists(self.mergeDirPath):
            shutil.rmtree(self.mergeDirPath)
        self.createFolder(self.mergeDirPath)

        dataset, total_frame = self.getDataset()

        sampling_cnt = 0
        i = 0
        
        total_sample_frame = 20
        sample_num = int(total_frame / total_sample_frame)

        mergeLidarDirPath = self.mergeDirPath + 'Merged-LDR-All/'
        self.createFolder(mergeLidarDirPath)

        for ar_view_ldr, fr_view_ldr_lower, fr_view_ldr_upper, near_view_ldr_left, near_view_ldr_rear, near_view_ldr_right, rr_view_ldr in dataset:
            
            if i % sample_num != 0:
                i += 1
                continue
            print(ar_view_ldr)
            ts = ar_view_ldr.split('/')[-1].split('_')[0]
            print(ts)


            print('----- Progress(current/total): ' + str(sampling_cnt) + '/' + str(total_sample_frame))

            scan_ar_view_ldr = (np.fromfile(ar_view_ldr, dtype=np.float32)).reshape(-1, 4).T
            scan_ar_view_ldr = np.concatenate((scan_ar_view_ldr, np.zeros((1, len(np.transpose(scan_ar_view_ldr))))))
            scan_ar_view_ldr = np.concatenate((scan_ar_view_ldr, np.ones((1, len(np.transpose(scan_ar_view_ldr)))) * 0)).T
            scan_fr_view_ldr_lower = (np.fromfile(fr_view_ldr_lower, dtype=np.float32)).reshape(-1, 4).T
            scan_fr_view_ldr_lower = np.concatenate((scan_fr_view_ldr_lower, np.zeros((1, len(np.transpose(scan_fr_view_ldr_lower))))))
            scan_fr_view_ldr_lower = np.concatenate((scan_fr_view_ldr_lower, np.ones((1, len(np.transpose(scan_fr_view_ldr_lower)))) * 1)).T
            scan_fr_view_ldr_upper = (np.fromfile(fr_view_ldr_upper, dtype=np.float32)).reshape(-1, 4).T
            scan_fr_view_ldr_upper = np.concatenate((scan_fr_view_ldr_upper, np.zeros((1, len(np.transpose(scan_fr_view_ldr_upper))))))
            scan_fr_view_ldr_upper = np.concatenate((scan_fr_view_ldr_upper, np.ones((1, len(np.transpose(scan_fr_view_ldr_upper)))) * 2)).T
            scan_near_view_ldr_left = (np.fromfile(near_view_ldr_left, dtype=np.float32)).reshape(-1, 4).T
            scan_near_view_ldr_left = np.concatenate((scan_near_view_ldr_left, np.zeros((1, len(np.transpose(scan_near_view_ldr_left))))))
            scan_near_view_ldr_left = np.concatenate((scan_near_view_ldr_left, np.ones((1, len(np.transpose(scan_near_view_ldr_left)))) * 3)).T
            scan_near_view_ldr_rear = (np.fromfile(near_view_ldr_rear, dtype=np.float32)).reshape(-1, 4).T
            scan_near_view_ldr_rear = np.concatenate((scan_near_view_ldr_rear, np.zeros((1, len(np.transpose(scan_near_view_ldr_rear))))))
            scan_near_view_ldr_rear = np.concatenate((scan_near_view_ldr_rear, np.ones((1, len(np.transpose(scan_near_view_ldr_rear)))) * 4)).T
            scan_near_view_ldr_right = (np.fromfile(near_view_ldr_right, dtype=np.float32)).reshape(-1, 4).T
            scan_near_view_ldr_right = np.concatenate((scan_near_view_ldr_right, np.zeros((1, len(np.transpose(scan_near_view_ldr_right))))))
            scan_near_view_ldr_right = np.concatenate((scan_near_view_ldr_right, np.ones((1, len(np.transpose(scan_near_view_ldr_right)))) * 5)).T
            scan_rr_view_ldr = (np.fromfile(rr_view_ldr, dtype=np.float32)).reshape(-1, 4).T
            scan_rr_view_ldr = np.concatenate((scan_rr_view_ldr, np.zeros((1, len(np.transpose(scan_rr_view_ldr))))))
            scan_rr_view_ldr = np.concatenate((scan_rr_view_ldr, np.ones((1, len(np.transpose(scan_rr_view_ldr)))) * 6)).T


            merged_ldr_all = np.vstack((scan_ar_view_ldr, scan_fr_view_ldr_lower))
            merged_ldr_all = np.vstack((merged_ldr_all, scan_fr_view_ldr_upper))
            merged_ldr_all = np.vstack((merged_ldr_all, scan_near_view_ldr_left))
            merged_ldr_all = np.vstack((merged_ldr_all, scan_near_view_ldr_rear))
            merged_ldr_all = np.vstack((merged_ldr_all, scan_near_view_ldr_right))
            merged_ldr_all = np.vstack((merged_ldr_all, scan_rr_view_ldr))
            print(merged_ldr_all)
            
            bin_merged_ldr_all = np.array(merged_ldr_all, dtype=np.float32).tobytes()


            bin_merged_ldr_all_file_path = f'{mergeLidarDirPath}{ts}_Merged-LDR-All.bin'
            with open(bin_merged_ldr_all_file_path, "wb") as f:
                f.write(bin_merged_ldr_all)

            sampling_cnt += 1
            i += 1
    
    def createFolder(self, directory):
        try:
            if not os.path.exists(directory):
                os.makedirs(directory)
        except OSError:
            print('Error: Creating directory. ' + directory)

    def getDataset(self):
        ar_view_ldr_dir_path = f'{self.dataDirPath}*AR-View-LDR.bin'
        fr_view_ldr_lower_dir_path = f'{self.dataDirPath}*FR-View-LDR-Lower.bin'
        fr_view_ldr_upper_dir_path = f'{self.dataDirPath}*FR-View-LDR-Upper.bin'
        near_view_ldr_left_dir_path = f'{self.dataDirPath}*Near-View-LDR-Left.bin'
        near_view_ldr_rear_dir_path = f'{self.dataDirPath}*Near-View-LDR-Rear.bin'
        near_view_ldr_right_dir_path = f'{self.dataDirPath}*Near-View-LDR-Right.bin'
        rr_view_ldr_dir_path = f'{self.dataDirPath}*RR-View-LDR.bin'
        ar_view_ldr_path_list = sorted(glob.glob(ar_view_ldr_dir_path))
        fr_view_ldr_lower_path_list = sorted(glob.glob(fr_view_ldr_lower_dir_path))
        fr_view_ldr_upper_path_list = sorted(glob.glob(fr_view_ldr_upper_dir_path))
        near_view_ldr_left_path_list = sorted(glob.glob(near_view_ldr_left_dir_path))
        near_view_ldr_rear_path_list = sorted(glob.glob(near_view_ldr_rear_dir_path))
        near_view_ldr_right_path_list = sorted(glob.glob(near_view_ldr_right_dir_path))
        rr_view_ldr_path_list = sorted(glob.glob(rr_view_ldr_dir_path))


        f = open(self.ts_sync_csv_path, 'r', encoding='utf-8')
        rdr = csv.reader(f)

        sync_ar_view_ldr_path_list = []
        sync_fr_view_ldr_lower_path_list = []
        sync_fr_view_ldr_upper_path_list = []
        sync_near_view_ldr_left_path_list = []
        sync_near_view_ldr_rear_path_list = []
        sync_near_view_ldr_right_path_list = []
        sync_rr_view_ldr_path_list = []
        for ind, line in enumerate(rdr):
            
            if ind == 0:
                continue

            ar_view_ldr_ts = line[1]
            fr_view_ldr_lower_ts = line[4]
            fr_view_ldr_upper_ts = line[5]
            near_view_ldr_left_ts = line[12]
            near_view_ldr_rear_ts = line[13]
            near_view_ldr_right_ts = line[14]
            rr_view_ldr_ts = line[18]

            ar_view_ldr_matching = [s for s in ar_view_ldr_path_list if ar_view_ldr_ts in s]
            fr_view_ldr_lower_matching = [s for s in fr_view_ldr_lower_path_list if fr_view_ldr_lower_ts in s]
            fr_view_ldr_upper_matching = [s for s in fr_view_ldr_upper_path_list if fr_view_ldr_upper_ts in s]
            near_view_ldr_left_matching = [s for s in near_view_ldr_left_path_list if near_view_ldr_left_ts in s]
            near_view_ldr_rear_matching = [s for s in near_view_ldr_rear_path_list if near_view_ldr_rear_ts in s]
            near_view_ldr_right_matching = [s for s in near_view_ldr_right_path_list if near_view_ldr_right_ts in s]
            rr_view_ldr_matching = [s for s in rr_view_ldr_path_list if rr_view_ldr_ts in s]
            sync_ar_view_ldr_path_list.append(ar_view_ldr_matching[0])
            sync_fr_view_ldr_lower_path_list.append(fr_view_ldr_lower_matching[0])
            sync_fr_view_ldr_upper_path_list.append(fr_view_ldr_upper_matching[0])
            sync_near_view_ldr_left_path_list.append(near_view_ldr_left_matching[0])
            sync_near_view_ldr_rear_path_list.append(near_view_ldr_rear_matching[0])
            sync_near_view_ldr_right_path_list.append(near_view_ldr_right_matching[0])
            sync_rr_view_ldr_path_list.append(rr_view_ldr_matching[0])

        f.close()

        total_frame = len(sync_ar_view_ldr_path_list)
        dataset = zip(sync_ar_view_ldr_path_list, sync_fr_view_ldr_lower_path_list, sync_fr_view_ldr_upper_path_list, sync_near_view_ldr_left_path_list, sync_near_view_ldr_rear_path_list, sync_near_view_ldr_right_path_list, sync_rr_view_ldr_path_list)
        return dataset, total_frame

def main():

    print('----- undistort projection start\n')

    parser = argparse.ArgumentParser()
    parser.add_argument('-i', '--data_path', type=str)
    args = parser.parse_args()

    if args.data_path == None:
        print('Please enter a data path.')
        return

    merge = Merge(args.data_path)
    merge.mergeLidar()

    print('----- Done')

if __name__ == "__main__":
	main()
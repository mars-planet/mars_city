"""
Reads and preprocesses data from one of the PAMAP2_Dataset files.
"""
from datetime import datetime

import pandas as pd
import numpy as np


def magnitude(row):
    """
    Calculates the magnitude of the body's acceleration vector in the row.
    """
    return (row[21] ** 2 + row[22] ** 2 + row[23] ** 2) ** 0.5


def read_data(path):
    """
    Reads the dataset, extracts acceleration and timestamp data
    and returns a Panda's DataFrame with it.
    Fields: index -> timestamp
            activityID
            hr -> heart rarte
            IMU_Chest_Magnitude -> chest accelerometer's magnitude vector
    """
    colum_names = ['timestamp', 'activityID', 'hr']
    colum_names += ['IMU_Hand' + str(x) for x in range(1, 18)]
    colum_names += ['IMU_Chest' + str(x) for x in range(1, 18)]
    colum_names += ['IMU_Foot' + str(x) for x in range(1, 18)]

    frame = pd.read_csv(path, sep=r'\s*', names=colum_names, header=None)
    frame['IMU_Chest_Magnitude'] = frame.apply(magnitude, axis=1)
    frame['IMU_Chest_x'] = frame['IMU_Chest1']
    frame['IMU_Chest_y'] = frame['IMU_Chest2']
    frame['IMU_Chest_z'] = frame['IMU_Chest3']
    frame = frame.loc[:, ['timestamp', 'activityID', 'hr',
                          'IMU_Chest_Magnitude',
                          'IMU_Chest_x', 'IMU_Chest_y', 'IMU_Chest_z']]

    now = datetime.now()
    frame['timestamp'] += (now - datetime(1970, 1, 1)).total_seconds()
    frame['timestamp'] = frame['timestamp'].apply(lambda x: x * 10 ** 9)
    frame['timestamp'] = frame['timestamp'].astype('datetime64[ns]')
    frame = frame.set_index('timestamp')

    return frame


def extract_hr_acc(dataframe):
    """
    Returns a more compact DataFrame with NAs forward filled.
    Fields: index -> timestamp
            activityID
            hr -> heart rarte
            acc -> chest accelerometer's magnitude vector
            ratio -> hr/acc
            ratio_log -> ln(ratio)
    """
    frame = pd.DataFrame({'hr': dataframe.hr,
                       'acc': dataframe.IMU_Chest_Magnitude,
                       'acc_x': dataframe.IMU_Chest_x,
                       'acc_y': dataframe.IMU_Chest_y,
                       'acc_z': dataframe.IMU_Chest_z},
                        index=dataframe.index)
    frame['ratio'] = frame.hr / frame.acc
    frame['ratio_log'] = np.log(frame.ratio)
    #fill NAs forward
    frame = frame.fillna(method='ffill')
    #fill NAs backwards (fill any NAs at the start of all series)
    frame = frame.fillna(method='bfill')

    #frame = frame.reset_index()
    return frame

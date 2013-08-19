from datetime import datetime

import pandas as pd
import numpy as np

def magnitude(row):
    return (row[21]**2 + row[22]**2 + row[23]**2)**.5


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
    colum_names += ['IMU_Hand'+str(x) for x in range(1,18)]
    colum_names += ['IMU_Chest'+str(x) for x in range(1,18)]
    colum_names += ['IMU_Foot'+str(x) for x in range(1,18)]

    df = pd.read_csv(path, sep='\s*', names=colum_names, header=None)
    df['IMU_Chest_Magnitude'] = df.apply(magnitude, axis=1)
    df['IMU_Chest_x'] = df['IMU_Chest1']
    df['IMU_Chest_y'] = df['IMU_Chest2']
    df['IMU_Chest_z'] = df['IMU_Chest3']
    df = df.loc[:, ['timestamp', 'activityID', 'hr', 'IMU_Chest_Magnitude',
                    'IMU_Chest_x', 'IMU_Chest_y', 'IMU_Chest_z']]

    now = datetime.now()
    df['timestamp'] += (now - datetime(1970,1,1)).total_seconds()
    df['timestamp'] = df['timestamp'].apply(lambda x: x*10**9)
    df['timestamp'] = df['timestamp'].astype('datetime64[ns]')
    df = df.set_index('timestamp')

    return df


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
    df = pd.DataFrame({'hr': dataframe.hr,
                       'acc': dataframe.IMU_Chest_Magnitude,
                       'acc_x': dataframe.IMU_Chest_x,
                       'acc_y': dataframe.IMU_Chest_y,
                       'acc_z': dataframe.IMU_Chest_z},
                        index=dataframe.index)
    df['ratio'] = df.hr/df.acc
    df['ratio_log'] = np.log(df.ratio)
    #fill NAs forward
    df = df.fillna(method='ffill')
    #fill NAs backwards (fill any NAs at the start of all series)
    df = df.fillna(method='bfill')

    #df = df.reset_index()
    return df

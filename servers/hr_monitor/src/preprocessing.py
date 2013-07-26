from pylab import sqrt
import pandas as pd
import numpy as np

def magnitude(row):
    return sqrt(row[21]**2 + row[22]**2 + row[23]**2)


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
    df = df.loc[:, ['timestamp', 'activityID', 'hr', 'IMU_Chest_Magnitude']]

    timestamps = df['timestamp']
    df['timestamp'] = timestamps.apply(lambda x: x*10**9)
    df['timestamp'] = timestamps.astype('datetime64[ns]')
    df = df.set_index('timestamp')

    df['activityID'] = df['activityID'].astype('string')

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
    df = pd.DataFrame({'hr': dataframe.hr.tolist(),
                       'acc': dataframe.IMU_Chest_Magnitude.tolist()},
                        index=dataframe.index)
    #fill NAs forward
    df = df.fillna(method='ffill')
    #fill NAs backwards (fill any NAs at the start of all series)
    df = df.fillna(method='bfill')
    df['ratio'] = df.hr/df.acc
    df['ratio_log'] = np.log(df.ratio)

    #df = df.reset_index()
    return df

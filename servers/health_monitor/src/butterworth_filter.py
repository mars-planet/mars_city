import scipy.signal as signal


def butterworth_filter(x, order=144, lowcut=0.5, highcut=100):
    b, a = signal.butter(order, [lowcut, highcut], 'band', True)
    filtered = signal.lfilter(a, b, x)

    return filtered

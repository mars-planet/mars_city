"""
Makes several plots of the data for better analysis.
"""
import matplotlib.pyplot as plt
import scipy.stats as st
from scipy.stats import norm
from numpy import linspace


def plot_distribution(style, name, bins, data):
    """
    Fits the distribution [name] to [data] and plots a histogram with [bins]
    and a line with [style]
    """
    print name
    try:
        name = name.strip()
        dist = st.__getattribute__(name)
        ratioparam = dist.fit(data)
        ratiofitted = dist.pdf(bins, *ratioparam)
        plt.plot(bins, ratiofitted, style, label=name)
        return (ratioparam, ratiofitted)
    except Exception as inst:
        print inst


def plot_everything(name, data, df_activities, plot_ratio=False):
    colors = {0: '#000000',
              1: '#00000f',
              2: '#0000f0',
              3: '#0000ff',
              4: '#000f00',
              5: '#000f0f',
              6: '#000ff0',
              7: '#000fff',
              9: '#00f000',
              10: '#00f00f',
              11: '#00f0f0',
              12: '#00f0ff',
              13: '#00ff00',
              16: '#00ff0f',
              17: '#00fff0',
              18: '#00ffff',
              19: '#0f0000',
              20: '#0f000f',
              24: '#0f00f0',
              }

    ratio_log = data.ratio_log

    plt.figure(name + ' fit')
    _, bins, _ = plt.hist(ratio_log, bins=1000,
                                range=(ratio_log.min(), ratio_log.max()),
                                normed=True, alpha=0.5)
    plot_distribution('c-', 'norm', bins, ratio_log)
    plt.legend(loc='best')

    plt.figure(name + ' historical')
    data.acc.plot(alpha=0.5)
    data.hr.plot(alpha=0.5)
    data.ratio_log.plot()
    if plot_ratio:
        data.ratio.plot()

    for name, group in df_activities.groupby('activityID'):
        xmin = min(group.index)
        xmax = max(group.index)
        plt.axvspan(xmin, xmax, facecolor=colors[int(name)], alpha=0.25)

    plt.legend(loc='best')

    plt.show()


def plot_assumption_free(scores, data, bins=50):
    """
    Plots the scores from the analysis using the assumption free algorithm.
    """
    plt.figure()
    plt.subplot(2, 1, 1)
    (data.acc / data.acc.max()).plot()
    (data.hr / data.hr.max()).plot()
    data.ratio_log.plot()
    plt.legend(loc='best')
    plt.subplot(2, 1, 2)
    plt.plot(data.index[:len(scores)], scores)

    scores = [x for x in scores if abs(x) > 10 ** -10]
    s_mean, sigma = norm.fit(scores)
    plt.figure()
    plt.hist(scores,  bins=50, normed=True)
    plt.plot(bins, norm.pdf(bins, loc=s_mean, scale=sigma))
    vlin = linspace(s_mean - 3 * sigma, s_mean + 3 * sigma, 13)
    step = int(256 / ((len(vlin) - 1) / 2))
    colors = linspace(0, 1, 256)[::step][:(len(vlin) - 1) / 2]
    colors = [(c, 0, 0) for c in colors]
    colors += [(1, 1, 1)]
    colors += [(0, c, 0) for c in reversed(colors)]
    plt.vlines(vlin.tolist()[1:], 0, 1, colors[1:])

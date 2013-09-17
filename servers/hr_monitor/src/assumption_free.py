"""
Calculates an anomaly score based on a discretized version of the input data.
It does this dividing the input data into two windows, a lead, representing
the current state, and a lag, representing the historic behavior, and then
comparing both.
"""
#
# Implemented following
# [Wei, Li, et al. "Assumption-Free Anomaly Detection in Time Series."
# SSDBM. Vol. 5. 2005.]
#

from __future__ import division, print_function
import string
from collections import deque, namedtuple
from itertools import islice

import numpy as np
from numpy import sqrt
from scipy.stats import norm


class AssumptionFreeAA(object):
    """
    Implements the anomaly scoring algorithm.
    """

    def __init__(self, word_size=10, window_factor=100,
                 lead_window_factor=3, lag_window_factor=30):
        """
        window_size = word_size * window_factor
        lead_window_size = lead_window_factor * window_size
        lag_window_size = lag_window_factor * window_size
        universe_size = lead_window_size + lag_window_size
        """
        self._word_size = word_size
        self._window_size = window_factor * word_size
        lead_window_size = lead_window_factor * self._window_size
        lag_window_size = lag_window_factor * self._window_size
        self.universe_size = lead_window_size + lag_window_size
        self._lead_window = deque(maxlen=lead_window_size)
        self._lag_window = deque(maxlen=lag_window_size)
        self._recursion_level = 2

    @classmethod
    def sax(cls, data, alphbt_size=4, word_size=10):
        """
        calculates the SAX discretization for the given data.
        Input:
            data:           sequential collection of datapoints.
                            It must be a numpy array of shape (n,)
            alphbt_size:    the size of the alphabet
            word_size:      size of each SAX word
        Output:
            List of words generated according to the SAX algorithm.
        Notes:
            based on https://gist.github.com/slnovak/912927
            by Stefan Novak
        """
        if not isinstance(data, np.ndarray):
            raise ValueError("data must be a numpy array")
        if len(data.shape) != 1:
            raise ValueError("data must be of shape (n,), not "
                             + "%s of shape %s" % (data.__class__, data.shape))
        if len(data) % word_size != 0:
            raise ValueError("len(data) must be divisible by word_size")
        # Scale data to have a mean of 0 and a standard deviation of 1.
        scaled_data = data - np.mean(data)
        std = scaled_data.std()
        if std != 0:
            scaled_data *= 1.0 / std
        # Calculate our breakpoint locations.
        breakpoints = norm.ppf(np.linspace(1 / alphbt_size,
                                           1 - 1 / alphbt_size,
                                           alphbt_size - 1))
        breakpoints = np.concatenate((breakpoints, np.array([np.Inf])))
        # Split the scaled_data into word_size pieces.
        scaled_data = np.array_split(scaled_data, word_size)
        # Calculate the mean for each section.
        section_means = [np.mean(section) for section in scaled_data]
        # Figure out which break each section is in
        # based on the section_means and calculated breakpoints.
        section_locations = [np.where(breakpoints > section_mean)[0][0]
                             for section_mean in section_means]
        # Convert the location into the corresponding letter.
        sax_phrases = ''.join([string.ascii_letters[ind]
                               for ind in section_locations])
        return sax_phrases

    @classmethod
    def dist(cls, mtrx_a, mtrx_b):
        r"""
        Returns \sum_{i=0}^{n} \sum_{j=0}^{n} (A_{ij}-B_{ij})^2
        """
        return np.power(mtrx_a - mtrx_b, 2).sum()

    @classmethod
    def build_combinations(cls, alphabet, combinations=None,
                            combination_len=2):
        """
        Returns all combination_len length combinations from the
        alphabet's characters.
        """
        if combinations is None or len(combinations) == 0:
            combinations = set(a for a in alphabet)
            combination_len -= 1
        if combination_len <= 0:
            return combinations
        else:
            new_combinations = set(c + a for a in alphabet
                                   for c in combinations)
            return cls.build_combinations(alphabet,
                                           new_combinations,
                                           combination_len - 1)

    @classmethod
    def count_substr(cls, stack, needle):
        """
        Counts occurrences of needle in stack.
        Example: in aaaa there are 3 occurrences of aa
        """
        count = 0
        for i in range(len(stack) - len(needle) + 1):
            if stack[i: i + len(needle)] == needle:
                count += 1
        return count

    @classmethod
    def count_frequencies(cls, words, alphabet, subword_len=2):
        """
        Builds a dictionary of frequencies, looking in the list words,
        of subwords of length subword_len.
        """
        combinations = cls.build_combinations(alphabet, set(), subword_len)
        freqs = {c: 0.0 for c in combinations}
        for word in words:
            for key in freqs:
                freqs[key] += cls.count_substr(word, key)
        return freqs

    @classmethod
    def build_bitmap(cls, freqs, norm_factor=0):
        """
        Builds a bitmap of size (len(freqs), len(freqs)).
        Each cell in the bitmap is proportional to the frequency of a subword.
        """
        matrix_size = sqrt(len(freqs))
        bitmap = np.zeros(shape=(matrix_size, matrix_size), dtype=np.float64)
        ordered_keys = sorted(freqs.keys())
        i = 0
        j = 0
        for key in ordered_keys:
            j = j % matrix_size
            if norm_factor != 0:
                bitmap[i, j] = freqs[key] / norm_factor
            else:
                bitmap[i, j] = freqs[key]
            j += 1
            if j == matrix_size:
                i += 1
        return bitmap

    @classmethod
    def get_words(cls, window, feature_size, word_size):
        """
        Splits window in feature_size sized chunks, calculates
        their SAX representation and returns a list with those representations.
        """
        words = []
        factor = int(len(window) / feature_size)
        for i in range(factor):
            init, end = i * feature_size, (i + 1) * feature_size
            window_slice = list(islice(window, init, end))
            word = cls.sax(data=np.array(window_slice), word_size=word_size)
            words.append(word)
        return words

    def detect(self, datapoints):
        """
        Calculates the datapoints' anomaly score according to:
        http://alumni.cs.ucr.edu/~ratana/SSDBM05.pdf
        """
        analysis_result = []
        for datapoint in datapoints:
            if len(self._lead_window) == self._lead_window.maxlen:
                self._lag_window.append(self._lead_window.popleft())
            self._lead_window.append(datapoint)
            if (len(self._lead_window) == self._lead_window.maxlen
                    and len(self._lag_window) == self._lag_window.maxlen):
                lead_words = self.get_words(window=self._lead_window,
                                            feature_size=self._window_size,
                                            word_size=self._word_size)
                lag_words = self.get_words(self._lag_window,
                                           self._window_size,
                                           self._word_size)
                lead_freqs = self.count_frequencies(lead_words, 'abcd',
                                                    self._recursion_level)
                lag_freqs = self.count_frequencies(lag_words, 'abcd',
                                                   self._recursion_level)
                norm_factor = max(lead_freqs.values() + lag_freqs.values())
                lead_bitmap = self.build_bitmap(lead_freqs, norm_factor)
                lag_bitmap = self.build_bitmap(lag_freqs, norm_factor)
                score = self.dist(lead_bitmap, lag_bitmap)
                score /= lead_bitmap.shape[0] * lead_bitmap.shape[1]
                result = AssumptionFreeAA.Analysis(score=score,
                                                   bitmp1=lead_bitmap,
                                                   bitmp2=lag_bitmap)
                analysis_result.append(result)
        return analysis_result


AssumptionFreeAA.Analysis = namedtuple('Analysis',
                                       ['score', 'bitmp1', 'bitmp2'])

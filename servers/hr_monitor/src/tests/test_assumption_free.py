#! /usr/bin/env python
from __future__ import division, print_function

import sys
import unittest

import numpy as np

sys.path.append("../../")

from src.assumption_free import AssumptionFreeAA


class AssumptionFreeTests(unittest.TestCase):

    def test_init(self):
        eword_size = 10
        ewindow_factor = 100
        elead_window_factor = 3
        elag_window_factor = 30
        ewindow_size = ewindow_factor * eword_size
        elead_window_size = elead_window_factor * ewindow_size
        elag_window_size = elag_window_factor * ewindow_size
        euniverse_size = elead_window_size + elag_window_size

        inst = AssumptionFreeAA(word_size=eword_size,
                                window_factor=ewindow_factor,
                                lead_window_factor=elead_window_factor,
                                lag_window_factor=elag_window_factor)

        self.assertEqual(eword_size, inst._word_size)
        self.assertEqual(ewindow_size, inst._window_size)
        self.assertEqual(euniverse_size, inst.universe_size)
        self.assertEqual(elead_window_size, inst._lead_window.maxlen)
        self.assertEqual(elag_window_size, inst._lag_window.maxlen)
        self.assertEqual(elag_window_size, inst._lag_window.maxlen)

    def test_sax(self):
        data = np.array(range(10))
        expected = 'aaabbccddd'
        actual = AssumptionFreeAA.sax(data)
        self.assertItemsEqual(expected, actual,
                              'exp: %s; act: %s' % (expected, actual))

    def test_sax_wrong_data(self):
        data = range(10)
        self.assertRaises(ValueError, AssumptionFreeAA.sax, data)

    def test_sax_wrong_len(self):
        data = np.array(range(9))
        self.assertRaises(ValueError, AssumptionFreeAA.sax, data)

    def test_sax_wrong_shape(self):
        data = np.array([[1, 2], [3, 4]])
        self.assertRaises(ValueError, AssumptionFreeAA.sax, data)

    def test_sax_std0(self):
        data = np.array([1] * 10)
        expected = 'cccccccccc'
        actual = AssumptionFreeAA.sax(data)
        self.assertItemsEqual(expected, actual,
                              'exp: %s; act: %s' % (expected, actual))

    def test_dist(self):
        mtrx_a = np.array(range(10))
        mtrx_b = np.array(range(10, 20))
        expected = 1000
        actual = AssumptionFreeAA.dist(mtrx_a, mtrx_b)
        self.assertEqual(expected, actual,
                         'exp: %s; act: %s' % (expected, actual))

    def test_dist_equal(self):
        mtrx_a = np.array(range(10))
        mtrx_b = np.array(range(10))
        expected = 0
        actual = AssumptionFreeAA.dist(mtrx_a, mtrx_b)
        self.assertEqual(expected, actual,
                         'exp: %s; act: %s' % (expected, actual))

    def test_dist_wrong_dim(self):
        mtrx_a = np.array(range(9))
        mtrx_b = np.array(range(10))
        self.assertRaises(ValueError, AssumptionFreeAA.dist, mtrx_a, mtrx_b)

    def test_build_combinations(self):
        data = 'abcd'
        expected = set(['aa', 'ab', 'ac', 'ad', 'ba', 'bb', 'bc', 'bd',
                        'ca', 'cb', 'cc', 'cd', 'da', 'db', 'dc', 'dd'])
        actual = AssumptionFreeAA.build_combinations(data)
        self.assertSetEqual(expected, actual,
                            'exp: %s; act: %s' % (expected, actual))

    def test_count_substr(self):
        data = 'aabaaacaadaaa'
        expected = 6
        actual = AssumptionFreeAA.count_substr(data, 'aa')
        self.assertEqual(expected, actual,
                         'exp: %s; act: %s' % (expected, actual))

    def test_count_substr1(self):
        data = 'abacadaa'
        expected = 1
        actual = AssumptionFreeAA.count_substr(data, 'aa')
        self.assertEqual(expected, actual,
                         'exp: %s; act: %s' % (expected, actual))

    def test_count_substr0(self):
        data = 'abacada'
        expected = 0
        actual = AssumptionFreeAA.count_substr(data, 'aa')
        self.assertEqual(expected, actual,
                         'exp: %s; act: %s' % (expected, actual))

    def test_count_frequencies(self):
        data = ['aabbaaacc', 'aadaaaccc']
        alphabet = 'abcd'
        expected = {'aa': 6, 'ab': 1, 'ac': 2, 'ad': 1,
                    'ba': 1, 'bb': 1, 'bc': 0, 'bd': 0,
                    'ca': 0, 'cb': 0, 'cc': 3, 'cd': 0,
                    'da': 1, 'db': 0, 'dc': 0, 'dd': 0}
        actual = AssumptionFreeAA.count_frequencies(data, alphabet)
        self.assertDictEqual(expected, actual,
                             'exp: %s; act: %s' % (expected, actual))

    def test_build_bitmap(self):
        data = {'aa': 6, 'ab': 1, 'ac': 2, 'ad': 1,
                'ba': 1, 'bb': 1, 'bc': 0, 'bd': 0,
                'ca': 0, 'cb': 0, 'cc': 3, 'cd': 0,
                'da': 1, 'db': 0, 'dc': 0, 'dd': 0}
        norm_factor = 10
        expected = np.array([[0.6, 0.3, 0.2, 0.1],
                             [0.1, 0.1, 0.1, 0.1],
                             [0, 0, 0, 0],
                             [0, 0, 0, 0]])
        actual = AssumptionFreeAA.build_bitmap(data, norm_factor)
        self.assertItemsEqual(expected.flatten(), actual.flatten(),
                             'exp: %s; act: %s' % (expected, actual))

    def test_build_bitmap_null_map(self):
        data = {'aa': 0, 'ab': 0, 'ac': 0, 'ad': 0,
                'ba': 0, 'bb': 0, 'bc': 0, 'bd': 0,
                'ca': 0, 'cb': 0, 'cc': 0, 'cd': 0,
                'da': 0, 'db': 0, 'dc': 0, 'dd': 0}
        norm_factor = 10
        expected = np.array([[0, 0, 0, 0],
                             [0, 0, 0, 0],
                             [0, 0, 0, 0],
                             [0, 0, 0, 0]])
        actual = AssumptionFreeAA.build_bitmap(data, norm_factor)
        self.assertItemsEqual(expected.flatten(), actual.flatten(),
                             'exp: %s; act: %s' % (expected, actual))

    def test_build_bitmap_norm_factor_null(self):
        data = {'aa': 6, 'ab': 1, 'ac': 2, 'ad': 1,
                'ba': 1, 'bb': 1, 'bc': 0, 'bd': 0,
                'ca': 0, 'cb': 0, 'cc': 3, 'cd': 0,
                'da': 1, 'db': 0, 'dc': 0, 'dd': 0}
        norm_factor = 0
        expected = np.array([[6, 3, 2, 1],
                             [1, 1, 1, 1],
                             [0, 0, 0, 0],
                             [0, 0, 0, 0]])
        actual = AssumptionFreeAA.build_bitmap(data, norm_factor)
        self.assertItemsEqual(expected.flatten(), actual.flatten(),
                             'exp: %s; act: %s' % (expected, actual))

    def test_build_bitmap_null_map_norm_factor_null(self):
        data = {'aa': 0, 'ab': 0, 'ac': 0, 'ad': 0,
                'ba': 0, 'bb': 0, 'bc': 0, 'bd': 0,
                'ca': 0, 'cb': 0, 'cc': 0, 'cd': 0,
                'da': 0, 'db': 0, 'dc': 0, 'dd': 0}
        norm_factor = 0
        expected = np.array([[0, 0, 0, 0],
                             [0, 0, 0, 0],
                             [0, 0, 0, 0],
                             [0, 0, 0, 0]])
        actual = AssumptionFreeAA.build_bitmap(data, norm_factor)
        self.assertItemsEqual(expected.flatten(), actual.flatten(),
                             'exp: %s; act: %s' % (expected, actual))

    def test_get_words(self):
        data = np.array([range(20)] * 100).flatten()
        feature_size = 20
        word_size = 10
        expected = ['aaabbccddd'] * 100
        actual = AssumptionFreeAA.get_words(data, feature_size, word_size)
        self.assertItemsEqual(expected, actual,
                              'exp: %s;\nact: %s' % (expected, actual))

    def test_detect(self):
        eword_size = 2
        ewindow_factor = 2
        elead_window_factor = 2
        elag_window_factor = 2

        inst = AssumptionFreeAA(word_size=eword_size,
                                window_factor=ewindow_factor,
                                lead_window_factor=elead_window_factor,
                                lag_window_factor=elag_window_factor)

        data = np.array([1, 2, 2, 2, 4, 7, 8, 9, 1, 9, 5, 6, 7, 3, 2, 6])
        escore = 0.125
        ebitmp1 = np.array([[0.,  0.,  0.,  0.],
                            [0.,  0.,  1.,  0.],
                            [0.,  1.,  0.,  0.],
                            [0.,  0.,  0.,  0.]])
        ebitmp2 = np.array([[0.,  0.,  0.,  1.],
                            [0.,  0.,  1.,  0.],
                            [0.,  0.,  0.,  0.],
                            [0.,  0.,  0.,  0.]])
        actual = inst.detect(data)[0]
        self.assertEqual(escore, actual.score,
                         'exp: %s; act: %s' % (escore, actual.score))
        self.assertItemsEqual(ebitmp1[0], actual.bitmp1[0],
                         'exp: %s; act: %s' % (ebitmp1[0], actual.bitmp1[0]))
        self.assertItemsEqual(ebitmp1[1], actual.bitmp1[1],
                         'exp: %s; act: %s' % (ebitmp1[1], actual.bitmp1[1]))
        self.assertItemsEqual(ebitmp2[0], actual.bitmp2[0],
                         'exp: %s; act: %s' % (ebitmp2[0], actual.bitmp2[0]))
        self.assertItemsEqual(ebitmp2[1], actual.bitmp2[1],
                         'exp: %s; act: %s' % (ebitmp2[1], actual.bitmp2[1]))

    def test_detect_not_enough_elements(self):
        eword_size = 2
        ewindow_factor = 2
        elead_window_factor = 2
        elag_window_factor = 2

        inst = AssumptionFreeAA(word_size=eword_size,
                                window_factor=ewindow_factor,
                                lead_window_factor=elead_window_factor,
                                lag_window_factor=elag_window_factor)

        data = np.array([1, 2, 2, 2, 4, 7, 8, 9, 1, 9])
        elen_score = 0
        actual = inst.detect(data)
        self.assertEqual(elen_score, len(actual),
                         'exp: %s; act: %s' % (elen_score, len(actual)))


if __name__ == '__main__':
    unittest.main()

import unittest
import numpy as np

def test_matrix(self: unittest.TestCase, size:int, cl):
    elements = list(np.arange(size*size, dtype=float) + 1)
    matrix = cl(*elements)

    self.assertEqual(matrix(1,2), memoryview(matrix)[1,2])
    self.assertTrue(memoryview(matrix).c_contiguous)
    # Have to cast through bytes for some reason...
    aslist = memoryview(matrix).tolist()
    self.assertEqual(np.array(elements).reshape((size,size)).tolist(), aslist)
    
    v = np.array(matrix, copy=False)
    self.assertEqual(matrix(0,1), 2)
    v[0,1] = 5
    self.assertEqual(matrix(0,1), 5) # can mutate origina matrix

def test_vector(self: unittest.TestCase, size:int, cl):
    elements = list(np.arange(size, dtype=float) + 1)
    vector = cl(*elements)

    self.assertEqual(vector[1], memoryview(vector)[1])
    self.assertTrue(memoryview(vector).c_contiguous)
    # Have to cast through bytes for some reason...
    aslist = memoryview(vector).tolist()
    self.assertEqual(elements, aslist)
    
    v = np.array(vector, copy=False)
    self.assertEqual(vector[1], 2)
    v[1] = 5
    self.assertEqual(vector[1], 5) # can mutate original vector

    self.assertEqual(elements, list(elements))


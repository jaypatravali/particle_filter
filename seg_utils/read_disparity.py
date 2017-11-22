import numpy as np
import re
import sys
from PIL import Image
from PIL import ImageOps
import cv2
import scipy
from scipy import ndimage

'''
Reads Disparity Images stored as PFM files
'''
def load_pfm(file):
  color = None
  width = None
  height = None
  scale = None
  endian = None

  header = file.readline().rstrip()
  if header == 'PF':
    color = True
  elif header == 'Pf':
    color = False
  else:
    raise Exception('Not a PFM file.')

  dim_match = re.match(r'^(\d+)\s(\d+)\s$', file.readline())
  if dim_match:
    width, height = map(int, dim_match.groups())
  else:
    raise Exception('Malformed PFM header.')

  scale = float(file.readline().rstrip())
  if scale < 0: # little-endian
    endian = '<'
    scale = -scale
  else:
    endian = '>' # big-endian

  data = np.fromfile(file, endian + 'f')
  shape = (height, width, 3) if color else (height, width)
  return np.reshape(data, shape), scale

def read_disparity(file_name):
    file = open(file_name)
    img, sc = load_pfm(file)
    cv2.flip(img, 0, img)
    vis = (img * 255 / np.max(img)).astype('uint8')
    return img, vis

    # cv2.imwrite('38.png', img)
    # cv2.imshow('dst_rt', img)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()




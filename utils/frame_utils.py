import numpy as np
from os.path import *
from scipy.misc import imread
from scipy.misc import imresize
import flow_utils 

def read_gen(file_name):
    ext = splitext(file_name)[-1]
    if ext == '.png' or ext == '.jpeg' or ext == '.ppm' or ext == '.jpg':
        im = imread(file_name)
        h  = im.shape[0];
        w  = im.shape[1];
        if ((h%64!=0) or (w%64!=0)):
            im = im_rescale(im);
        if im.shape[2] > 3:
            return im[:,:,:3]
        else:
            return im
    elif ext == '.bin' or ext == '.raw':
        return np.load(file_name)
    elif ext == '.flo':
        flo = flow_utils.readFlow(file_name).astype(np.float32)
        h   = flo.shape[0];
        w   = flo.shape[1];
        h_new = 64*int(round(h/64));
        w_new = 64*int(round(w/64));
        if ((h%64!=0) or (w%64!=0)):
            flo = np.resize(flo, (h_new, w_new, 2))
            #print(flo.shape)
        return flo
    return []

def im_rescale(im):
    h = im.shape[0];
    w = im.shape[1];
    h_new = 64*int(round(h/64));
    w_new = 64*int(round(w/64));
    im_resized = imresize(im, (h_new, w_new));
    return im_resized



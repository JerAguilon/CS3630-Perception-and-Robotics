import numpy as np
import re

from imgaug import augmenters as iaa
from skimage import io
from scipy.misc import imsave

seq = iaa.Sequential([
    iaa.Crop(px=(0, 16)), # crop images from each side by 0 to 16px (randomly chosen)
    iaa.Affine(rotate=(-25, 25), shear=(-8, 8)),
])

def imread_convert(f):
    return io.imread(f).astype(np.uint8)

def load_data_from_folder(dir):
    # read all images into an image collection
    ic = io.ImageCollection(dir+"*.bmp", load_func=imread_convert)

    #create one large array of image data
    data = io.concatenate_images(ic)

    #extract labels from image names
    labels = np.array(ic.files)
    for i, f in enumerate(labels):
        m = re.search("_", f)
        labels[i] = f[len(dir):m.start()]

    return (data,labels)

(test_raw, test_labels) = load_data_from_folder('./test2/')
images_aug = seq.augment_images(test_raw)

i = 0
for item in images_aug:
    imsave('./test2/out/{}_test{}.bmp'.format(test_labels[i], i), item)
    i += 1

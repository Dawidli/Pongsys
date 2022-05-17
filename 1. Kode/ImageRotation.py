import cv2 as cv
import numpy as np

def trackcall(angle):

        image0 = rotateImage(image, angle)
        cv.ShowImage("imageRotation",image0);


def rotateImage(image, angle):

    image0 = image
    if hasattr(image, 'shape'):
        image_center = tuple(np.array(image.shape)/2)
        shape = tuple(image.shape)
    elif hasattr(image, 'width') and hasattr(image, 'height'):
        image_center = tuple(np.array((image.width/2, image.height/2)))
        shape = (image.width, image.height)
    else:
        raise Exception('Unable to acquire dimensions of image for type %s.') % (type(image),)
    rot_mat = cv2.getRotationMatrix2D(image_center, angle,1.0)
    image = np.asarray( image[:,:] )

    rotated_image = cv2.warpAffine(image, rot_mat, shape, flags=cv2.INTER_LINEAR)

    # Copy the rotated data back into the original image object.
    cv.SetData(image0, rotated_image.tostring())

    return image0

angle = 720

vc = cv2.VideoCapture(0)

cv.NamedWindow('imageRotation',cv.CV_WINDOW_NORMAL)
cv.NamedWindow('imageMeter',cv.CV_WINDOW_AUTOSIZE)
cv.CreateTrackbar("rotate","imageMeter",360,angle,trackcall)
#image = cv.LoadImage('C:/Users/Public/Pictures/Sample Pictures/Desert.jpg', cv.CV_LOAD_IMAGE_COLOR)
#image0 = rotateImage(image, angle)
if vc.isOpened():

        rval = image = vc.read()
else:
        rval = False

while rval:

        image = vc.read()
        trackcall(0)


key = cv.WaitKey(0)
if key == 27:

    cv.DestroyWindow('imageRotation')
    cv.DestroyWindow("imageMeter")

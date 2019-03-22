import cv2  # Not actually necessary if you just want to create an image.
import numpy as np


def generate_lookup_table(lookup_table, camera_matrix, distCoeffs):
    # Creates an artifical image of known pixel values
    blank_image = np.zeros((height,width,4), np.uint8)
    height, width = blank_image.shape[:2]
    for row in range(len(height)):
        for col in range(len(width)):
            blank_image[row][0] = row%256
            blank_image[row][1] = int(row/256)
            blank_image[row][2] = col%256
            blank_image[row][3] = int(col/256)
    dist_img = cv2.undistort(blank_image, camera_matrix, distCoeffs)

    for row in range(len(height)):
        for col in range(len(width)):
            orig_row_idx = dist_img

            if row == 0 or col == 0:
                orig_row_idx_0 = orig_row_idx
                orig_col_idx_0 = orig_col_idx
            else:
                if (abs(orig_row_idx - orig_row_idx_0) > 6):
                    dist_img[row][col] = dist_img[row-1][col]
                if (abs(orig_col_idx - orig_col_idx_0) > 6):
                    dist_img[row][col] = dist_img[row][col-1]
                orig_row_idx_0 = orig_row_idx
                orig_col_idx_0 = orig_col_idx

    for col in range(len(height)):
        for row in range(len(width)):
            orig_row_idx = dist_img

            if row == 0 or col == 0:
                orig_row_idx_0 = orig_row_idx
                orig_col_idx_0 = orig_col_idx
            else:
                if (abs(orig_row_idx - orig_row_idx_0) > 6):
                    dist_img[row][col] = dist_img[row-1][col]
                if (abs(orig_col_idx - orig_col_idx_0) > 6):
                    dist_img[row][col]_row_idx
                orig_col_idx_0 = orig_col_idx

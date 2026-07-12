#!/usr/bin/env python3
import cv2
import numpy as np
import os

def main():
    resource_dir = "/home/yoon/yoon_urdf/src/yoon_urdf/resource"
    os.makedirs(resource_dir, exist_ok=True)
    
    square_size = 128
    # 8x6 squares
    width = square_size * 8
    height = square_size * 6
    
    img = np.zeros((height, width, 3), dtype=np.uint8)
    
    for i in range(6):
        for j in range(8):
            color = (255, 255, 255) if (i + j) % 2 == 0 else (0, 0, 0)
            img[i*square_size:(i+1)*square_size, j*square_size:(j+1)*square_size] = color
            
    # Add a white border (Quiet Zone) around the grid to help OpenCV detection
    border_img = cv2.copyMakeBorder(
        img, 
        top=64, bottom=64, left=64, right=64, 
        borderType=cv2.BORDER_CONSTANT, 
        value=[255, 255, 255]
    )
    
    path = os.path.join(resource_dir, "checkerboard.png")
    cv2.imwrite(path, border_img)
    print(f"SUCCESSfully generated high-contrast checkerboard image at {path}")

if __name__ == '__main__':
    main()

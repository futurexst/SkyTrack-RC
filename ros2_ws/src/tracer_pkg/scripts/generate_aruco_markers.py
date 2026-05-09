#!/usr/bin/env python3

import os
import cv2
import cv2.aruco as aruco


def create_marker(aruco_dict, marker_id, marker_size_px):
    if hasattr(aruco, "generateImageMarker"):
        return aruco.generateImageMarker(
            aruco_dict,
            marker_id,
            marker_size_px
        )

    if hasattr(aruco, "drawMarker"):
        return aruco.drawMarker(
            aruco_dict,
            marker_id,
            marker_size_px
        )

    raise RuntimeError("ArUco marker generation function not found.")


def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    pkg_dir = os.path.dirname(script_dir)

    out_dir = os.path.join(
        pkg_dir,
        'models',
        'lane_bot',
        'materials',
        'textures'
    )

    os.makedirs(out_dir, exist_ok=True)

    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

    marker_ids = [10, 15]
    marker_size_px = 800
    border_px = 160

    for marker_id in marker_ids:
        img = create_marker(aruco_dict, marker_id, marker_size_px)

        # 흰 quiet zone 추가
        img = cv2.copyMakeBorder(
            img,
            border_px,
            border_px,
            border_px,
            border_px,
            cv2.BORDER_CONSTANT,
            value=255
        )

        # 원본 저장
        out_path = os.path.join(out_dir, f'aruco_{marker_id}.png')
        cv2.imwrite(out_path, img)

        # Gazebo plane UV mirror 대응용 좌우 반전 저장
        img_flip_x = cv2.flip(img, 1)
        out_path_flip_x = os.path.join(out_dir, f'aruco_{marker_id}_flip_x.png')
        cv2.imwrite(out_path_flip_x, img_flip_x)

        # 혹시 상하 반전일 때 쓸 수 있게 상하 반전도 저장
        img_flip_y = cv2.flip(img, 0)
        out_path_flip_y = os.path.join(out_dir, f'aruco_{marker_id}_flip_y.png')
        cv2.imwrite(out_path_flip_y, img_flip_y)

        print(f'saved: {out_path}')
        print(f'saved: {out_path_flip_x}')
        print(f'saved: {out_path_flip_y}')


if __name__ == '__main__':
    main()

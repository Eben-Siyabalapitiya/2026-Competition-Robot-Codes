import argparse
import cv2
import numpy as np


def build(tag_id, px, quiet, label):
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    tag = cv2.aruco.generateImageMarker(d, tag_id, px)
    pad = quiet * px // 8
    sheet = np.full((px + 2 * pad, px + 2 * pad), 255, dtype=np.uint8)
    sheet[pad:pad + px, pad:pad + px] = tag
    if label:
        cv2.putText(sheet, f"tag36h11 id {tag_id}", (pad, pad - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, 0, 2)
    return sheet


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--id", type=int, default=0)
    ap.add_argument("--pixels", type=int, default=1200)
    ap.add_argument("--quiet", type=int, default=1)
    ap.add_argument("--no-label", action="store_true")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    sheet = build(args.id, args.pixels, args.quiet, not args.no_label)
    out = args.out or f"tag36h11_{args.id}.png"
    cv2.imwrite(out, sheet)
    print(f"wrote {out}  ({sheet.shape[1]}x{sheet.shape[0]} px)")
    print("print it at a size that leaves the white border intact")


if __name__ == "__main__":
    main()

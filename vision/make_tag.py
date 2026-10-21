import argparse
import cv2
import numpy as np
from PIL import Image


def build(tag_id, inches, dpi, quiet_ratio, label):
    side = int(round(inches * dpi))
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    tag = cv2.aruco.generateImageMarker(d, tag_id, side)

    pad = int(round(side * quiet_ratio))
    total = side + 2 * pad
    sheet = np.full((total, total), 255, dtype=np.uint8)
    sheet[pad:pad + side, pad:pad + side] = tag

    cv2.rectangle(sheet, (pad - 1, pad - 1), (pad + side, pad + side), 200, 1)

    if label:
        text = f"tag36h11  id {tag_id}  {inches}in"
        cv2.putText(sheet, text, (pad, max(18, pad - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX, dpi / 500.0, 120, max(1, dpi // 300))
    return sheet, side, total


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--id", type=int, default=0)
    ap.add_argument("--inches", type=float, default=5.0)
    ap.add_argument("--dpi", type=int, default=300)
    ap.add_argument("--quiet", type=float, default=0.25)
    ap.add_argument("--no-label", action="store_true")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    sheet, side, total = build(args.id, args.inches, args.dpi, args.quiet, not args.no_label)

    stem = args.out or f"tag36h11_id{args.id}_{args.inches:g}in"
    img = Image.fromarray(sheet)
    img.save(stem + ".png", dpi=(args.dpi, args.dpi))
    img.save(stem + ".pdf", resolution=args.dpi)

    print(f"{stem}.png / .pdf")
    print(f"  black square {args.inches:g} in ({side} px at {args.dpi} dpi)")
    print(f"  whole sheet  {total / args.dpi:.2f} in")
    print("  print at 100% / actual size, do not fit to page")


if __name__ == "__main__":
    main()

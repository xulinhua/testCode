# Changelog

## 0.1.4

- Trihedral ChArUco: **identical marker IDs on all three faces** (real-world same boards)
- Suite splits faces by coplanar RANSAC + PnP normals (not ID ranges)

## 0.1.3

- Fix trihedral ArUco **bit row 0 → low v** (was high v = 180° marker rotation; centers OK, corners wrong)
- Flip_u on both XZ and YZ faces (inside-corner view mirror)
- Default squares 8×8 (inner corners) to match suite `trihedral_oneshot.yaml`

## 0.1.2

- ChArUco / trihedral ChArUco use real OpenCV **DICT_4X4_250** markers (not random bits)
- Charuco polarity matches OpenCV 4.5: `(0,0)` black, markers on white squares
- Trihedral ChArUco face marker ID bases: XY=0, XZ=100, YZ=200

## 0.1.1

- Boards use solid geometry (chess / circles / ArUco-like / ChArUco / **trihedral 3-face**); textures no longer required for visibility
- UI strings English-only (avoid CJK mojibake in Isaac)

## 0.1.0

- New standalone extension `hs.calib.isaac_calib_sim`
- Single camera + switchable boards + orbit on Play + ROS2 RGB stream

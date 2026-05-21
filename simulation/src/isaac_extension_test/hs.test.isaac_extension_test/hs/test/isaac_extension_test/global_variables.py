# Copyright (c) 2024
from pxr import UsdPhysics

EXTENSION_TITLE = "Hs Test Isaac Extension Test"
EXTENSION_DESCRIPTION = "Standalone grasping scene data capture extension"

num_classes = 4
class_names = ["bwb", "sqm", "yida", "dingshuji"]

meshOptions = [
    UsdPhysics.Tokens.none,
    UsdPhysics.Tokens.meshSimplification,
    UsdPhysics.Tokens.convexHull,
    UsdPhysics.Tokens.convexDecomposition,
    UsdPhysics.Tokens.boundingSphere,
    UsdPhysics.Tokens.boundingCube,
]

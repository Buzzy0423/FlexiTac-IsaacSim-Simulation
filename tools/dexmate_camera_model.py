"""Centered, distortion-free nominal pinhole parameters for USD cameras."""

import math


def camera_parameters(spec=None):
    if spec is None:
        return {"resolution": [640, 480], "focal_length_mm": 24., "horizontal_aperture_mm": 36.,
                "vertical_aperture_mm": 27., "K": [[640 * 24 / 36, 0, 320],
                                                  [0, 480 * 24 / 27, 240], [0, 0, 1]],
                "status": "Legacy nominal pinhole, not calibrated"}
    width, height = spec["resolution"]
    diagonal = float(spec["diagonal_fov_deg"])
    focal = float(spec["focal_length_mm"])
    if (any(not isinstance(v, int) or v <= 0 for v in (width, height))
            or not 0 < diagonal < 180 or not math.isfinite(focal) or focal <= 0):
        raise ValueError("Invalid pinhole image dimensions, diagonal FOV or focal length")
    fpx = math.hypot(width, height) / (2 * math.tan(math.radians(diagonal) / 2))
    return {**spec, "horizontal_aperture_mm": focal * width / fpx,
            "vertical_aperture_mm": focal * height / fpx,
            "K": [[fpx, 0, width / 2], [0, fpx, height / 2], [0, 0, 1]],
            "distortion_model": "none (simulation approximation)", "distortion_coefficients": [0.] * 5}

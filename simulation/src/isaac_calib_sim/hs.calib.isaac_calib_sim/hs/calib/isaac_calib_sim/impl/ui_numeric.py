# -*- coding: utf-8 -*-
"""数值控件。"""

from __future__ import annotations

from typing import Callable, Optional, Tuple, Union

import omni.ui as ui

NumericEntry = Tuple[object, ui.Widget, float, float, float]


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def add_float_param(
    label: str,
    default: float,
    *,
    step: float = 0.01,
    fmt: str = "%.4f",
    min_val: float = -1e6,
    max_val: float = 1e6,
    on_changed: Optional[Callable[[], None]] = None,
) -> NumericEntry:
    model = ui.SimpleFloatModel(float(default))
    with ui.HStack(height=24):
        ui.Label(label, width=ui.Fraction(0.45), height=0)
        wheel_frame = ui.Frame(width=ui.Fraction(0.55), height=0)
        with wheel_frame:
            ui.FloatDrag(
                model=model,
                min=min_val,
                max=max_val,
                step=step,
                format=fmt,
                height=0,
            )

    def _wheel(x, y, _modifier):
        delta = y if y else x
        if not delta:
            return
        sign = 1.0 if delta > 0 else -1.0
        model.set_value(_clamp(model.get_value_as_float() + step * sign, min_val, max_val))
        if on_changed:
            on_changed()

    wheel_frame.set_mouse_wheel_fn(_wheel)
    if on_changed:
        model.add_value_changed_fn(lambda _m: on_changed())
    return model, wheel_frame, step, min_val, max_val


def add_int_param(
    label: str,
    default: int,
    *,
    step: int = 1,
    min_val: int = 1,
    max_val: int = 8192,
    on_changed: Optional[Callable[[], None]] = None,
) -> NumericEntry:
    model = ui.SimpleIntModel(int(default))
    with ui.HStack(height=24):
        ui.Label(label, width=ui.Fraction(0.45), height=0)
        wheel_frame = ui.Frame(width=ui.Fraction(0.55), height=0)
        with wheel_frame:
            ui.IntDrag(
                model=model,
                min=min_val,
                max=max_val,
                step=step,
                height=0,
            )

    def _wheel(x, y, _modifier):
        delta = y if y else x
        if not delta:
            return
        sign = 1 if delta > 0 else -1
        model.set_value(int(_clamp(model.get_value_as_int() + step * sign, min_val, max_val)))
        if on_changed:
            on_changed()

    wheel_frame.set_mouse_wheel_fn(_wheel)
    if on_changed:
        model.add_value_changed_fn(lambda _m: on_changed())
    return model, wheel_frame, float(step), float(min_val), float(max_val)


def read_entry_as_float(entry: Union[NumericEntry, object, None], default: float = 0.0) -> float:
    model = entry[0] if isinstance(entry, tuple) else entry
    if model is None:
        return default
    try:
        return float(model.get_value_as_float())
    except Exception:
        try:
            return float(model.get_value_as_int())
        except Exception:
            return default


def write_entry_float(entry: Union[NumericEntry, object, None], value: float) -> None:
    model = entry[0] if isinstance(entry, tuple) else entry
    if model is None:
        return
    try:
        model.set_value(float(value))
    except Exception:
        pass


def read_entry_as_int(entry: Union[NumericEntry, object, None], default: int = 0) -> int:
    model = entry[0] if isinstance(entry, tuple) else entry
    if model is None:
        return default
    try:
        return int(model.get_value_as_int())
    except Exception:
        try:
            return int(float(model.get_value_as_float()))
        except Exception:
            return default

# -*- coding: utf-8 -*-
"""数值控件：Frame 拦截滚轮，避免 ScrollingWindow 吞掉滚轮事件。"""

from __future__ import annotations

from typing import Callable, Optional, Tuple, Union

import omni.ui as ui

NumericModel = object
NumericEntry = Tuple[NumericModel, ui.Widget, float, float, float]  # model, wheel_frame, step, min, max

ROW_HEIGHT = 18
LABEL_FRAC = 0.50
VALUE_FRAC = 0.50
# 固定像素宽度，避免控件铺满整窗
COMPACT_LABEL_W = 52
COMPACT_VALUE_W = 62
COMPACT_COL_W = COMPACT_LABEL_W + COMPACT_VALUE_W + 4
# 模块内双列（min|max、平移|旋转）：加宽标签，列内左标签右对齐
DUAL_COL_LABEL_W = 84
DUAL_COL_VALUE_W = 58
DUAL_COL_W = DUAL_COL_LABEL_W + DUAL_COL_VALUE_W + 4
# 话题/路径行：窄标签，余宽给 StringField
TOPIC_LABEL_W = 56
# 复位关节等密集数值行：无标签，窄输入框
MINI_FLOAT_W = 40


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
    row_height: int = ROW_HEIGHT,
    label_frac: float = LABEL_FRAC,
    compact: bool = True,
    dual_col: bool = False,
) -> NumericEntry:
    """创建带滚轮步进的 FloatDrag 行。"""
    model = ui.SimpleFloatModel(float(default))

    if dual_col:
        row_w = DUAL_COL_W
        label_w, value_w = DUAL_COL_LABEL_W, DUAL_COL_VALUE_W
        label_align = ui.Alignment.RIGHT_CENTER
    elif compact:
        row_w = COMPACT_COL_W
        label_w, value_w = COMPACT_LABEL_W, COMPACT_VALUE_W
        label_align = ui.Alignment.LEFT_CENTER
    else:
        row_w = None
        label_w, value_w = ui.Fraction(label_frac), ui.Fraction(1.0 - label_frac)
        label_align = ui.Alignment.LEFT_CENTER

    with ui.HStack(height=row_height, width=row_w):
        ui.Label(label, width=label_w, height=0, alignment=label_align)
        wheel_frame = ui.Frame(width=value_w, height=0)
        with wheel_frame:
            ui.FloatDrag(
                model=model,
                min=min_val,
                max=max_val,
                step=step,
                format=fmt,
                height=0,
            )

    def _wheel(x, y, modifier):
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


def add_mini_float_param(
    default: float,
    *,
    step: float = 1.0,
    fmt: str = "%.1f",
    min_val: float = -360.0,
    max_val: float = 360.0,
    on_changed: Optional[Callable[[], None]] = None,
    row_height: int = ROW_HEIGHT,
    width: int = MINI_FLOAT_W,
) -> NumericEntry:
    """无标签窄 FloatDrag（用于一行排多个关节角）。"""
    model = ui.SimpleFloatModel(float(default))
    wheel_frame = ui.Frame(width=width, height=0)
    with wheel_frame:
        ui.FloatDrag(
            model=model,
            min=min_val,
            max=max_val,
            step=step,
            format=fmt,
            height=0,
        )

    def _wheel(x, y, modifier):
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
    row_height: int = ROW_HEIGHT,
    label_frac: float = LABEL_FRAC,
    compact: bool = True,
    dual_col: bool = False,
) -> NumericEntry:
    """创建带滚轮步进的 IntDrag 行。"""
    model = ui.SimpleIntModel(int(default))

    if dual_col:
        row_w = DUAL_COL_W
        label_w, value_w = DUAL_COL_LABEL_W, DUAL_COL_VALUE_W
        label_align = ui.Alignment.RIGHT_CENTER
    elif compact:
        row_w = COMPACT_COL_W
        label_w, value_w = COMPACT_LABEL_W, COMPACT_VALUE_W
        label_align = ui.Alignment.LEFT_CENTER
    else:
        row_w = None
        label_w, value_w = ui.Fraction(label_frac), ui.Fraction(1.0 - label_frac)
        label_align = ui.Alignment.LEFT_CENTER

    with ui.HStack(height=row_height, width=row_w):
        ui.Label(label, width=label_w, height=0, alignment=label_align)
        wheel_frame = ui.Frame(width=value_w, height=0)
        with wheel_frame:
            ui.IntDrag(
                model=model,
                min=min_val,
                max=max_val,
                step=step,
                height=0,
            )

    def _wheel(x, y, modifier):
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


def read_entry_as_float(entry: Union[NumericEntry, NumericModel, None], default: float = 0.0) -> float:
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


def read_entry_as_int(entry: Union[NumericEntry, NumericModel, None], default: int = 0) -> int:
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


def set_entry_value(entry: Union[NumericEntry, NumericModel, None], value: Union[float, int, str]) -> None:
    model = entry[0] if isinstance(entry, tuple) else entry
    if model is None:
        return
    try:
        model.set_value(float(value))
    except Exception:
        try:
            model.set_value(int(float(value)))
        except Exception:
            pass

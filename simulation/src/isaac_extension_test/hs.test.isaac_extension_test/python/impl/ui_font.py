# -*- coding: utf-8 -*-
"""为 omni.ui 解析可用 CJK 字体，避免中文显示为问号。"""

import os
from typing import Optional

_CJK_FONT_CACHE: Optional[str] = None


def resolve_cjk_font_path() -> Optional[str]:
    """返回系统 Noto/WenQuanYi 等 CJK 字体路径；找不到则返回 None。"""
    global _CJK_FONT_CACHE
    if _CJK_FONT_CACHE is not None:
        return _CJK_FONT_CACHE or None

    candidates = [
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
        "/usr/share/fonts/truetype/noto/NotoSansCJK-Regular.ttc",
        "/usr/share/fonts/noto-cjk/NotoSansCJK-Regular.ttc",
        "/usr/share/fonts/opentype/noto/NotoSansCJKsc-Regular.otf",
        "/usr/share/fonts/truetype/wqy/wqy-microhei.ttc",
        "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
        "/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf",
    ]
    for path in candidates:
        if os.path.isfile(path):
            _CJK_FONT_CACHE = path
            return path

    _CJK_FONT_CACHE = ""
    return None


def get_style_with_cjk():
    """在 Isaac 默认 style 上叠加 CJK 字体（若系统已安装）。"""
    from isaacsim.gui.components.ui_utils import get_style

    style = dict(get_style())
    font_path = resolve_cjk_font_path()
    if not font_path:
        return style

    # 不要改 ScrollingFrame / CollapsableFrame 相关 style，否则主面板可能空白。
    for widget in (
        "Label",
        "Button",
        "CheckBox",
        "ComboBox",
        "Field",
        "StringField",
    ):
        entry = dict(style.get(widget, {}))
        entry["font"] = font_path
        style[widget] = entry
    return style

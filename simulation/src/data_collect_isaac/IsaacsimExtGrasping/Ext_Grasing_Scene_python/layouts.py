import omni.ui as ui
from isaacsim.gui.components.ui_utils import multi_cb_builder, combo_cb_scrolling_frame_builder

class CategorySelector:
    """
    可复用的类别选择 UI 组件。

    当前主流程 ui_builder.py 使用了自定义 checkbox_dropdown_builder()，
    这个类保留为后续把类别选择独立成组件时使用。
    """

    def __init__(self):
        # key 为类别名，value 为勾选状态。
        self.category_states = {}  # 存储每个类别的勾选状态
        self.ui = None

    def on_category_toggled(self, category_name: str, checked: bool):
        """当某个类别被勾选/取消时调用"""
        self.category_states[category_name] = checked
        print(f"[UI] 类别 '{category_name}' 状态更新: {checked}")
        # 后续如果要让勾选状态直接控制场景显示/隐藏，可以在这里接入：
        # self.update_scene_visibility(category_name, checked)

    def build_ui(self, labels, default_states=None, panel_label="类别选择", tooltip="点击展开以选择要显示的类别"):
        """
        构建可折叠、可勾选类别的 UI 面板。

        Args:
            labels: 类别名列表。
            default_states: 每个类别默认是否勾选；None 表示全部勾选。
            panel_label: 折叠面板标题。
            tooltip: 鼠标悬停提示。
        """
        if default_states is None:
            default_states = [True] * len(labels)
        
        # 确保 default_states 和 labels 长度一致
        if len(default_states) != len(labels):
            raise ValueError("labels 和 default_states 长度必须一致")

        # 初始化状态
        for name, state in zip(labels, default_states):
            self.category_states[name] = state

        def build_checkbox_content():
            """在折叠面板内部构建复选框行"""
            # lambda 里用 name=name 捕获当前循环值，避免所有回调都绑定到最后一个类别。
            checkboxes = multi_cb_builder(
                labels=labels,
                default_vals=default_states,
                on_click_fns=[
                    lambda checked, name=name: self.on_category_toggled(name, checked)
                    for name in labels
                ],
                # 可选布局控制
                vertical=False,      # False: 横向排列 | True: 垂直排列
                spacing=8,           # 复选框之间间距
                tooltip_fns=[f"显示/隐藏 {name}" for name in labels]
            )
            return checkboxes

        # 创建可折叠 + 可启用的面板
        self.ui = combo_cb_scrolling_frame_builder(
            label=panel_label,
            tooltip=tooltip,
            checked=True,  # 默认展开且启用
            build_fn=build_checkbox_content,
            # 滚动条策略（重要：支持长列表）
            horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
            vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
            # 可选样式
            style={"background_color": 0xFF222222},  # 深色背景（可选）
            # 最小高度（防止折叠时太小）
            min_height=20,
        )
        return self.ui

__copyright__ = "Copyright (c) 2022-2024, NVIDIA CORPORATION. All rights reserved."
__license__ = """
NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
"""

import os
from typing import List

import carb
import numpy as np
from omni.syntheticdata.scripts.SyntheticData import SyntheticData
from omni.replicator.core.bindings._omni_replicator_core import Schema_omni_replicator_extinfo_1_0
from omni.replicator.core.scripts import functional as F
import warp as wp
from omni.replicator.core.scripts.annotators import AnnotatorRegistry
from omni.replicator.core.scripts.backends import BackendDispatch
from omni.replicator.core.scripts.utils import skeleton_data_utils
from omni.replicator.core.scripts.writers import Writer, WriterRegistry
from omni.replicator.core.scripts.writers_default.tools import colorize_distance, colorize_normals
from omni.replicator.core.scripts.backends.disk import DiskBackend
import open3d as o3d
__version__ = "0.0.2"


def lxSavePCD(
    path: str,
    pointcloud_data,
    rgb_data,
    backend
    ):
    """
    将 Replicator 点云 annotator 输出保存为 PCD 文件。

    Replicator 默认更偏向保存 npy/json/png，这里用 open3d 额外生成 lx_data_*.pcd，
    便于后续直接用 PCL、Open3D、CloudCompare 等工具查看或处理点云。
    """
    # warp.array 是 Isaac/Replicator 常见 GPU/CPU 数组类型；保存前统一转为 numpy。
    if isinstance(pointcloud_data, wp.array):
        pointcloud_data = pointcloud_data.numpy()
    if isinstance(rgb_data, wp.array):
        rgb_data = rgb_data.numpy()[:, :3]
    else:
        rgb_data = rgb_data[:, :3]
    # 点和颜色必须一一对应，否则 open3d 生成的点云颜色会错位。
    assert rgb_data.shape[0] == pointcloud_data.shape[0]
    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(pointcloud_data.astype(np.float64))
    pc.colors = o3d.utility.Vector3dVector(rgb_data.astype(np.float64) / 255)
    # BackendDispatch 负责输出根目录；这里拼出 data_log/<时间戳>/lx_data_XXXX.pcd。
    full_path = os.path.join(backend.output_dir, path)
    # path = "/home/jedzhang/omni.replicator_out/" + path
    carb.log_info("PCD PATH: {}".format(full_path))
    o3d.io.write_point_cloud(full_path, pc)


class LxWriter(Writer):
    """自定义 Replicator Writer。

    这个类基于 NVIDIA BasicWriter 的结构，按需启用 Replicator annotator，
    并在每帧 write() 时把 annotator 数据写成 PNG/NPY/JSON/PCD。

    本项目 collect_data() 默认启用：
        rgb=True
        distance_to_camera=True
        semantic_segmentation=True
        pointcloud=True

    最重要的定制点是 _write_pointcloud() 中额外调用 lxSavePCD()，生成 lx_data_*.pcd。

    Args:
        output_dir:
            Output directory string that indicates the directory to save the results.
        s3_bucket:
            The S3 Bucket name to write to. If not provided, disk backend will be used instead. Default: ``None``.
            This backend requires that AWS credentials are set up in ``~/.aws/credentials``.
            See https://boto3.amazonaws.com/v1/documentation/api/latest/guide/quickstart.html#configuration
        s3_region:
            If provided, this is the region the S3 bucket will be set to. Default: ``us-east-1``
        s3_endpoint:
            If provided, this endpoint URL will be used instead of the default.
        semantic_types:
            List of semantic types to consider when filtering annotator data. Default: ``["class"]``
        rgb:
            Boolean value that indicates whether the ``rgb``/``LdrColor`` annotator will be activated
            and the data will be written or not. Default: ``False``.
        bounding_box_2d_tight:
            Boolean value that indicates whether the ``bounding_box_2d_tight`` annotator will be activated
            and the data will be written or not. Default: ``False``.
        bounding_box_2d_loose:
            Boolean value that indicates whether the ``bounding_box_2d_loose`` annotator will be activated
            and the data will be written or not. Default: ``False``.
        semantic_segmentation:
            Boolean value that indicates whether the ``semantic_segmentation`` annotator will be activated
            and the data will be written or not. Default: ``False``.
        instance_id_segmentation:
            Boolean value that indicates whether the ``instance_id_segmentation`` annotator will be activated
            and the data will be written or not. Default: ``False``.
        instance_segmentation:
            Boolean value that indicates whether the ``instance_segmentation`` annotator will be activated
            and the data will be written or not. Default: ``False``.
        distance_to_camera:
            Boolean value that indicates whether the ``distance_to_camera`` annotator will be activated
            and the data will be written or not. Default: ``False``.
        distance_to_image_plane:
            Boolean value that indicates whether the ``distance_to_image_plane`` annotator will be activated
            and the data will be written or not. Default: ``False``.
        bounding_box_3d:
            Boolean value that indicates whether the ``bounding_box_3d`` annotator will be activated
            and the data will be written or not. Default: ``False``.
        occlusion:
            Boolean value that indicates whether the ``occlusion`` annotator will be activated
            and the data will be written or not. Default: ``False``.
        normals:
            Boolean value that indicates whether the ``normals`` annotator will be activated
            and the data will be written or not. Default: ``False``.
        motion_vectors:
            Boolean value that indicates whether the ``motion_vectors`` annotator will be activated
            and the data will be written or not. Default: ``False``.
        camera_params:
            Boolean value that indicates whether the ``camera_params`` annotator will be activated
            and the data will be written or not. Default: ``False``.
        pointcloud:
            Boolean value that indicates whether the ``pointcloud`` annotator will be activated
            and the data will be written or not. Default: ``False``.
        pointcloud_include_unlabelled:
            If ``True``, pointcloud annotator will capture any prim in the camera's perspective, not matter if it has
            semantics or not. If ``False``, only prims with semantics will be captured.
            Defaults to ``False``.
        image_output_format:
            String that indicates the format of saved RGB images. Default: ``"png"``
        colorize_semantic_segmentation:
            If ``True``, semantic segmentation is converted to an image where semantic IDs are mapped to colors
            and saved as a uint8 4 channel PNG image. If ``False``, the output is saved as a ``uint32`` PNG image.
            Defaults to ``True``.
        colorize_instance_id_segmentation:
            If ``True``, instance id segmentation is converted to an image where instance IDs are mapped to colors.
            and saved as a uint8 4 channel PNG image. If ``False``, the output is saved as a ``uint32`` PNG image.
            Defaults to ``True``.
        colorize_instance_segmentation:
            If ``True``, instance segmentation is converted to an image where instance are mapped to colors.
            and saved as a uint8 4 channel PNG image. If ``False``, the output is saved as a ``uint32`` PNG image.
            Defaults to ``True``.
        colorize_depth:
            If ``True``, will output an additional PNG image for depth for visualization
            Defaults to ``False``.
        frame_padding:
            Pad the frame number with leading zeroes.  Default: ``4``
        semantic_filter_predicate:
            A string specifying a semantic filter predicate as a disjunctive normal form of semantic type, labels.

            Examples :
                "typeA : labelA & !labelB | labelC , typeB: labelA ; typeC: labelD"
                "typeA : * ; * : labelA"
        use_common_output_dir:
            If ``True``, output for each annotator coming from multiple render products are saved under a common directory
            with the render product as the filename prefix (eg. <render_product_name>_<annotator_name>_<sequence>.<format>).
            If ``False``, multiple render product outputs are placed into their own directory
            (eg. <render_product_name>/<annotator_name>_<sequence>.<format>). Setting is ignored if using the writer with
            a single render product. Defaults to ``False``.


    Example:
        >>> import omni.replicator.core as rep
        >>> import carb
        >>> camera = rep.create.camera()
        >>> render_product = rep.create.render_product(camera, (1024, 1024))
        >>> writer = rep.WriterRegistry.get("BasicWriter")
        >>> tmp_dir = carb.tokens.get_tokens_interface().resolve("${temp}/rgb")
        >>> writer.initialize(output_dir=tmp_dir, rgb=True)
        >>> writer.attach([render_product])
        >>> rep.orchestrator.run()
    """

    def __init__(
        self,
        output_dir: str,
        s3_bucket: str = None,
        s3_region: str = None,
        s3_endpoint: str = None,
        semantic_types: List[str] = None,
        rgb: bool = True,
        bounding_box_2d_tight: bool = False,
        bounding_box_2d_loose: bool = False,
        semantic_segmentation: bool = False,
        instance_id_segmentation: bool = True,
        instance_segmentation: bool = False,
        distance_to_camera: bool = False,
        distance_to_image_plane: bool = False,
        bounding_box_3d: bool = False,
        occlusion: bool = False,
        normals: bool = False,
        motion_vectors: bool = False,
        camera_params: bool = False,
        pointcloud: bool = True,
        pointcloud_include_unlabelled: bool = False,
        image_output_format: str = "png",
        colorize_semantic_segmentation: bool = True,
        colorize_instance_id_segmentation: bool = True,
        colorize_instance_segmentation: bool = True,
        colorize_depth: bool = False,
        skeleton_data: bool = False,
        frame_padding: int = 4,
        semantic_filter_predicate: str = None,
        use_common_output_dir: bool = False,
    ):
        # 输出目录由 ui_builder.collector_folder_prep() 创建，一般是 data_log/<时间戳>/。
        self._output_dir = output_dir
        # annotator 模式表示按每个 annotator 分发数据；保留 BasicWriter 的数据组织方式。
        self.data_structure = "annotator"
        self.use_common_output_dir = use_common_output_dir
        if s3_bucket:
            self._backend = BackendDispatch(
                output_dir=output_dir,  # Maintains previous behaviour
                key_prefix=output_dir,
                bucket=s3_bucket,
                region=s3_region,
                endpoint_url=s3_endpoint,
            )
        else:
            self._backend = BackendDispatch(output_dir=output_dir)
        self.backend = self._backend
        # _frame_id 控制输出文件编号，默认生成 0000、0001 这样的四位编号。
        self._frame_id = 0
        self._sequence_id = 0
        self._image_output_format = image_output_format
        self._output_data_format = {}
        self.annotators = []
        self.version = __version__
        self._frame_padding = frame_padding
        self._telemetry = Schema_omni_replicator_extinfo_1_0()

        self.colorize_semantic_segmentation = colorize_semantic_segmentation
        self.colorize_instance_id_segmentation = colorize_instance_id_segmentation
        self.colorize_instance_segmentation = colorize_instance_segmentation
        self.colorize_depth = colorize_depth

        # 指定语义过滤类型。本项目用 semantic_types=["select_classes"]，
        # 因为 ui_builder 中给地面/桌子/物体添加的语义类型就是 select_classes。
        if semantic_types is not None:
            if semantic_filter_predicate is None:
                # semantic_filter_predicate = ":*; ".join(semantic_types) + ":*"
                semantic_filter_predicate = "; ".join(f"{t}:*" for t in semantic_types)
            else:
                raise ValueError(
                    "`semantic_types` and `semantic_filter_predicate` are mutually exclusive. Please choose only one."
                )
        elif semantic_filter_predicate is None:
            semantic_filter_predicate = "class:*"

        # 设置全局语义过滤表达式；只有匹配的 Prim 会进入语义/实例映射结果。
        if semantic_filter_predicate is not None:
            SyntheticData.Get().set_instance_mapping_semantic_filter(semantic_filter_predicate)

        # 以下根据初始化参数注册 annotator。只有注册的 annotator 才会在 write() 收到数据。
        # RGB
        if rgb:
            self.annotators.append(AnnotatorRegistry.get_annotator("rgb"))

        # Bounding Box 2D
        if bounding_box_2d_tight:
            self.annotators.append("bounding_box_2d_tight_fast")

        if bounding_box_2d_loose:
            self.annotators.append("bounding_box_2d_loose_fast")

        # Semantic Segmentation
        if semantic_segmentation:
            self.annotators.append(
                AnnotatorRegistry.get_annotator(
                    "semantic_segmentation", init_params={"colorize": colorize_semantic_segmentation}
                )
            )

        # Instance Segmentation
        if instance_id_segmentation:
            self.annotators.append(
                AnnotatorRegistry.get_annotator(
                    "instance_id_segmentation_fast", init_params={"colorize": colorize_instance_id_segmentation}
                )
            )

        # Instance Segmentation
        if instance_segmentation:
            self.annotators.append(
                AnnotatorRegistry.get_annotator(
                    "instance_segmentation_fast", init_params={"colorize": colorize_instance_segmentation}
                )
            )

        # Depth
        if distance_to_camera:
            self.annotators.append(AnnotatorRegistry.get_annotator("distance_to_camera"))

        if distance_to_image_plane:
            self.annotators.append(AnnotatorRegistry.get_annotator("distance_to_image_plane"))

        # Bounding Box 3D
        if bounding_box_3d:
            self.annotators.append("bounding_box_3d_fast")

        # Motion Vectors
        if motion_vectors:
            self.annotators.append(AnnotatorRegistry.get_annotator("motion_vectors"))

        # Occlusion
        if occlusion:
            self.annotators.append(AnnotatorRegistry.get_annotator("occlusion"))

        # Normals
        if normals:
            self.annotators.append(AnnotatorRegistry.get_annotator("normals"))

        # Camera Params
        if camera_params:
            self.annotators.append(AnnotatorRegistry.get_annotator("camera_params"))

        # Pointcloud
        if pointcloud:
            self.annotators.append(
                AnnotatorRegistry.get_annotator(
                    "pointcloud", init_params={"includeUnlabelled": pointcloud_include_unlabelled}
                )
            )

        # Skeleton Data
        if skeleton_data:
            self.annotators.append(
                AnnotatorRegistry.get_annotator("skeleton_data", init_params={"useSkelJoints": False})
            )

        backend_type = "S3" if s3_bucket else "Disk"
        self._telemetry.basicwriter_sendEvent(
            self.version,
            backend_type,
            rgb,
            bounding_box_2d_tight,
            bounding_box_2d_loose,
            semantic_segmentation,
            instance_id_segmentation,
            instance_segmentation,
            distance_to_camera,
            distance_to_image_plane,
            bounding_box_3d,
            occlusion,
            normals,
            motion_vectors,
            camera_params,
            pointcloud,
            pointcloud_include_unlabelled,
            image_output_format,
            colorize_semantic_segmentation,
            colorize_instance_id_segmentation,
            colorize_instance_segmentation,
            skeleton_data,
        )
        print(f"Annotators: {self.annotators}")

    def write(self, data: dict):
        """每帧写入回调。

        Replicator orchestrator 每推进一帧，OgnWriter 节点会调用本函数一次。
        data["annotators"] 中包含当前帧所有已注册 annotator 的结果。

        Args:
            data: 当前帧 annotator 数据字典。
        """
        # 如果使用 on_time 触发器，BasicWriter 会用 sequence_id 区分不同时间序列。
        # _ = data

        sequence_id = ""
        for trigger_name, call_count in data["trigger_outputs"].items():
            if "on_time" in trigger_name:
                sequence_id = f"{call_count}_{sequence_id}"
        if sequence_id != self._sequence_id:
            self._frame_id = 0
            self._sequence_id = sequence_id

        for annotator_name, annotator_data in data["annotators"].items():
            # Replicator 的 fast annotator 名称带 _fast，输出文件名中去掉这个后缀更易读。
            if annotator_name.endswith("_fast"):
                annotator_name = annotator_name[:-5]

            is_multi_rp = len(annotator_data) > 1
            for render_product_name, anno_rp_data in annotator_data.items():
                # 多 render_product 时需要把不同相机/渲染产品的输出目录区分开。
                if is_multi_rp:
                    if self.use_common_output_dir:
                        output_path = (
                            os.path.join(annotator_name, render_product_name) + "_"
                        )  # Add render product as prefix
                    else:
                        output_path = (
                            os.path.join(render_product_name, annotator_name) + os.path.sep
                        )  # Legacy behaviour
                else:
                    output_path = ""

                if annotator_name == "rgb" or annotator_name.startswith("Aug"):
                    self._write_rgb(anno_rp_data, output_path)

                elif annotator_name == "normals":
                    self._write_normals(anno_rp_data, output_path)

                elif annotator_name == "distance_to_camera":
                    self._write_distance_to_camera(anno_rp_data, output_path)

                elif annotator_name == "distance_to_image_plane":
                    self._write_distance_to_image_plane(anno_rp_data, output_path)

                elif annotator_name.startswith("semantic_segmentation"):
                    self._write_semantic_segmentation(anno_rp_data, output_path)

                elif annotator_name.startswith("instance_id_segmentation"):
                    self._write_instance_id_segmentation(anno_rp_data, output_path)

                elif annotator_name.startswith("instance_segmentation"):
                    self._write_instance_segmentation(anno_rp_data, output_path)

                elif annotator_name.startswith("motion_vectors"):
                    self._write_motion_vectors(anno_rp_data, output_path)

                elif annotator_name.startswith("occlusion"):
                    self._write_occlusion(anno_rp_data, output_path)

                elif annotator_name.startswith("bounding_box_3d"):
                    self._write_bounding_box_data(anno_rp_data, "3d", output_path)

                elif annotator_name.startswith("bounding_box_2d_loose"):
                    self._write_bounding_box_data(anno_rp_data, "2d_loose", output_path)

                elif annotator_name.startswith("bounding_box_2d_tight"):
                    self._write_bounding_box_data(anno_rp_data, "2d_tight", output_path)

                elif annotator_name.startswith("camera_params"):
                    self._write_camera_params(anno_rp_data, output_path)

                elif annotator_name.startswith("pointcloud"):
                    self._write_pointcloud(anno_rp_data, output_path)

                elif annotator_name.startswith("skeleton_data"):
                    self._write_skeleton(anno_rp_data, output_path)

                elif annotator_name not in ["camera", "resolution"]:
                    carb.log_warn(f"Unknown {annotator_name=}")
        self._frame_id += 1

    def _write_rgb(self, anno_rp_data: dict, output_path: str):
        """保存 RGB 图像：rgb_0000.png。"""
        file_path = (
            f"{output_path}rgb_{self._sequence_id}{self._frame_id:0{self._frame_padding}}.{self._image_output_format}"
        )
        self._backend.schedule(F.write_image, data=anno_rp_data["data"], path=file_path)

    def _write_normals(self, anno_rp_data: dict, output_path: str):
        """保存法线可视化图像：normals_0000.png。"""
        normals_data = anno_rp_data["data"]
        file_path = f"{output_path}normals_{self._sequence_id}{self._frame_id:0{self._frame_padding}}.png"
        colorized_normals_data = colorize_normals(normals_data)
        self._backend.schedule(F.write_image, data=colorized_normals_data, path=file_path)

    def _write_distance_to_camera(self, anno_rp_data: dict, output_path: str):
        """保存到相机光心的距离：distance_to_camera_0000.npy。"""
        dist_to_cam_data = anno_rp_data["data"]
        file_path = f"{output_path}distance_to_camera_{self._sequence_id}{self._frame_id:0{self._frame_padding}}.npy"
        self._backend.schedule(F.write_np, data=dist_to_cam_data, path=file_path)
        if self.colorize_depth:
            file_path = (
                f"{output_path}distance_to_camera_{self._sequence_id}{self._frame_id:0{self._frame_padding}}.png"
            )
            self._backend.schedule(
                F.write_image, data=colorize_distance(dist_to_cam_data, near=None, far=None), path=file_path
            )

    def _write_distance_to_image_plane(self, anno_rp_data: dict, output_path: str):
        """保存到图像平面的距离：distance_to_image_plane_0000.npy。"""
        dis_to_img_plane_data = anno_rp_data["data"]
        file_path = (
            f"{output_path}distance_to_image_plane_{self._sequence_id}{self._frame_id:0{self._frame_padding}}.npy"
        )
        self._backend.schedule(F.write_np, data=dis_to_img_plane_data, path=file_path)
        if self.colorize_depth:
            file_path = (
                f"{output_path}distance_to_image_plane_{self._sequence_id}{self._frame_id:0{self._frame_padding}}.png"
            )
            self._backend.schedule(
                F.write_image, data=colorize_distance(dis_to_img_plane_data, near=None, far=None), path=file_path
            )

    def _write_semantic_segmentation(self, anno_rp_data: dict, output_path: str):
        """保存语义分割图和语义 ID 到标签的映射 JSON。"""
        semantic_seg_data = anno_rp_data["data"]
        height, width = semantic_seg_data.shape[:2]

        file_path = f"{output_path}semantic_segmentation_{self._sequence_id}{self._frame_id:0{self._frame_padding}}.png"
        if self.colorize_semantic_segmentation:
            semantic_seg_data = semantic_seg_data.view(np.uint8).reshape(height, width, -1)
            self._backend.schedule(F.write_image, data=semantic_seg_data, path=file_path)
        else:
            semantic_seg_data = semantic_seg_data.view(np.uint32).reshape(height, width)
            self._backend.schedule(F.write_image, data=semantic_seg_data, path=file_path)

        id_to_labels = anno_rp_data["idToLabels"]
        file_path = (
            f"{output_path}semantic_segmentation_labels_{self._sequence_id}{self._frame_id:0{self._frame_padding}}.json"
        )
        self._backend.schedule(F.write_json, data={str(k): v for k, v in id_to_labels.items()}, path=file_path)


    def _write_instance_id_segmentation(self, anno_rp_data: dict, output_path: str):
        """保存实例 ID 分割图和实例 ID 到标签的映射 JSON。"""
        instance_seg_data = anno_rp_data["data"]
        height, width = instance_seg_data.shape[:2]

        file_path = (
            f"{output_path}instance_id_segmentation_{self._sequence_id}{self._frame_id:0{self._frame_padding}}.png"
        )
        if self.colorize_instance_id_segmentation:
            instance_seg_data = instance_seg_data.view(np.uint8).reshape(height, width, -1)
            self._backend.schedule(F.write_image, data=instance_seg_data, path=file_path)
        else:
            instance_seg_data = instance_seg_data.view(np.uint32).reshape(height, width)
            self._backend.schedule(F.write_image, data=instance_seg_data, path=file_path)

        id_to_labels = anno_rp_data["idToLabels"]

        file_path = f"{output_path}instance_id_segmentation_mapping_{self._sequence_id}{self._frame_id:0{self._frame_padding}}.json"
        self._backend.schedule(F.write_json, data={str(k): v for k, v in id_to_labels.items()}, path=file_path)
        # _ = 1

    def _write_instance_segmentation(self, anno_rp_data: dict, output_path: str):
        """保存实例分割图、标签映射和语义映射。"""
        instance_seg_data = anno_rp_data["data"]
        height, width = instance_seg_data.shape[:2]

        file_path = f"{output_path}instance_segmentation_{self._sequence_id}{self._frame_id:0{self._frame_padding}}.png"
        if self.colorize_instance_segmentation:
            instance_seg_data = instance_seg_data.view(np.uint8).reshape(height, width, -1)
            self._backend.schedule(F.write_image, data=instance_seg_data, path=file_path)
        else:
            instance_seg_data = instance_seg_data.view(np.uint32).reshape(height, width)
            self._backend.schedule(F.write_image, data=instance_seg_data, path=file_path)

        id_to_labels = anno_rp_data["idToLabels"]
        file_path = f"{output_path}instance_segmentation_mapping_{self._sequence_id}{self._frame_id:0{self._frame_padding}}.json"
        self._backend.schedule(F.write_json, data={str(k): v for k, v in id_to_labels.items()}, path=file_path)

        id_to_semantics = anno_rp_data["idToSemantics"]
        file_path = f"{output_path}instance_segmentation_semantics_mapping_{self._sequence_id}{self._frame_id:0{self._frame_padding}}.json"
        self._backend.schedule(F.write_json, data={str(k): v for k, v in id_to_semantics.items()}, path=file_path)

    def _write_motion_vectors(self, anno_rp_data: dict, output_path: str):
        """保存运动矢量 NPY。"""
        motion_vec_data = anno_rp_data["data"]
        file_path = f"{output_path}motion_vectors_{self._sequence_id}{self._frame_id:0{self._frame_padding}}.npy"
        self._backend.schedule(F.write_np, data=motion_vec_data, path=file_path)

    def _write_occlusion(self, anno_rp_data: dict, output_path: str):
        """保存遮挡比例 NPY。"""
        occlusion_data = anno_rp_data["data"]

        file_path = f"{output_path}occlusion_{self._sequence_id}{self._frame_id:0{self._frame_padding}}.npy"
        self._backend.schedule(F.write_np, data=occlusion_data, path=file_path)

    def _write_bounding_box_data(self, anno_rp_data: dict, bbox_type: str, output_path: str):
        """保存 2D/3D bounding box、标签映射和 Prim 路径。"""
        bbox_data = anno_rp_data["data"]
        id_to_labels = anno_rp_data["idToLabels"]
        prim_paths = anno_rp_data["primPaths"]

        file_path = (
            f"{output_path}bounding_box_{bbox_type}_{self._sequence_id}{self._frame_id:0{self._frame_padding}}.npy"
        )
        self._backend.schedule(F.write_np, data=bbox_data, path=file_path)

        labels_file_path = f"{output_path}bounding_box_{bbox_type}_labels_{self._sequence_id}{self._frame_id:0{self._frame_padding}}.json"
        self._backend.schedule(F.write_json, data=id_to_labels, path=labels_file_path)

        labels_file_path = f"{output_path}bounding_box_{bbox_type}_prim_paths_{self._sequence_id}{self._frame_id:0{self._frame_padding}}.json"
        self._backend.schedule(F.write_json, data=prim_paths, path=labels_file_path)

    def _write_camera_params(self, anno_rp_data: dict, output_path: str):
        """保存 Replicator 相机参数 JSON。"""
        camera_data = anno_rp_data
        serializable_data = {}

        for key, val in camera_data.items():
            if isinstance(val, np.ndarray):
                serializable_data[key] = val.tolist()
            else:
                serializable_data[key] = val

        file_path = f"{output_path}camera_params_{self._sequence_id}{self._frame_id:0{self._frame_padding}}.json"
        self._backend.schedule(F.write_json, data=serializable_data, path=file_path)

    def _write_pointcloud(self, anno_rp_data: dict, output_path: str):
        """
        保存点云相关输出。

        输出：
            pointcloud_*.npy: 点 XYZ
            pointcloud_rgb_*.npy: 每个点 RGB
            pointcloud_normals_*.npy: 每个点法线
            pointcloud_semantic_*.npy: 每个点语义 ID
            pointcloud_instance_*.npy: 每个点实例 ID
            lx_data_*.pcd: Open3D 写出的带颜色点云
        """
        pointcloud_data = anno_rp_data["data"]
        pointcloud_rgb = anno_rp_data["pointRgb"].reshape(-1, 4)
        pointcloud_normals = anno_rp_data["pointNormals"].reshape(-1, 4)
        pointcloud_semantic = anno_rp_data["pointSemantic"]
        pointcloud_instance = anno_rp_data["pointInstance"]

        file_path = f"{output_path}pointcloud_{self._sequence_id}{self._frame_id:0{self._frame_padding}}.npy"
        self._backend.schedule(F.write_np, data=pointcloud_data, path=file_path)

        rgb_file_path = f"{output_path}pointcloud_rgb_{self._sequence_id}{self._frame_id:0{self._frame_padding}}.npy"
        self._backend.schedule(F.write_np, data=pointcloud_rgb, path=rgb_file_path)

        normals_file_path = (
            f"{output_path}pointcloud_normals_{self._sequence_id}{self._frame_id:0{self._frame_padding}}.npy"
        )
        self._backend.schedule(F.write_np, data=pointcloud_normals, path=normals_file_path)

        semantic_file_path = (
            f"{output_path}pointcloud_semantic_{self._sequence_id}{self._frame_id:0{self._frame_padding}}.npy"
        )
        self._backend.schedule(F.write_np, data=pointcloud_semantic, path=semantic_file_path)

        instance_file_path = (
            f"{output_path}pointcloud_instance_{self._sequence_id}{self._frame_id:0{self._frame_padding}}.npy"
        )
        lx_file_path = (
            f"{output_path}lx_data_{self._sequence_id}{self._frame_id:0{self._frame_padding}}.pcd"
        )
        self._backend.schedule(F.write_np, data=pointcloud_instance, path=instance_file_path)
        # carb.log_info('{} {}'.format(lx_file_path, file_path))
        self._backend.schedule(lxSavePCD, path=lx_file_path, pointcloud_data=pointcloud_data,
                               rgb_data=pointcloud_rgb, backend=self._backend)

    def _write_skeleton(self, anno_rp_data: dict, output_path: str):
        """保存骨骼数据。当前抓取数据采集流程默认不启用。"""
        # "skeletonData" is deprecated
        # skeleton = json.loads(anno_rp_data["skeletonData"])

        skeleton_dict = {}

        skel_name = anno_rp_data["skelName"]
        skel_path = anno_rp_data["skelPath"]
        asset_path = anno_rp_data["assetPath"]
        animation_variant = anno_rp_data["animationVariant"]
        skeleton_parents = skeleton_data_utils.get_skeleton_parents(
            anno_rp_data["numSkeletons"], anno_rp_data["skeletonParents"], anno_rp_data["skeletonParentsSizes"]
        )
        rest_global_translations = skeleton_data_utils.get_rest_global_translations(
            anno_rp_data["numSkeletons"],
            anno_rp_data["restGlobalTranslations"],
            anno_rp_data["restGlobalTranslationsSizes"],
        )
        rest_local_translations = skeleton_data_utils.get_rest_local_translations(
            anno_rp_data["numSkeletons"],
            anno_rp_data["restLocalTranslations"],
            anno_rp_data["restLocalTranslationsSizes"],
        )
        rest_local_rotations = skeleton_data_utils.get_rest_local_rotations(
            anno_rp_data["numSkeletons"],
            anno_rp_data["restLocalRotations"],
            anno_rp_data["restLocalRotationsSizes"],
        )
        global_translations = skeleton_data_utils.get_global_translations(
            anno_rp_data["numSkeletons"],
            anno_rp_data["globalTranslations"],
            anno_rp_data["globalTranslationsSizes"],
        )
        local_rotations = skeleton_data_utils.get_local_rotations(
            anno_rp_data["numSkeletons"], anno_rp_data["localRotations"], anno_rp_data["localRotationsSizes"]
        )
        translations_2d = skeleton_data_utils.get_translations_2d(
            anno_rp_data["numSkeletons"], anno_rp_data["translations2d"], anno_rp_data["translations2dSizes"]
        )
        skeleton_joints = skeleton_data_utils.get_skeleton_joints(anno_rp_data["skeletonJoints"])
        joint_occlusions = skeleton_data_utils.get_joint_occlusions(
            anno_rp_data["numSkeletons"], anno_rp_data["jointOcclusions"], anno_rp_data["jointOcclusionsSizes"]
        )
        occlusion_types = skeleton_data_utils.get_occlusion_types(
            anno_rp_data["numSkeletons"], anno_rp_data["occlusionTypes"], anno_rp_data["occlusionTypesSizes"]
        )
        in_view = anno_rp_data["inView"]

        for skel_num in range(anno_rp_data["numSkeletons"]):
            skeleton_dict[f"skeleton_{skel_num}"] = {}
            skeleton_dict[f"skeleton_{skel_num}"]["skel_name"] = skel_name[skel_num]
            skeleton_dict[f"skeleton_{skel_num}"]["skel_path"] = skel_path[skel_num]
            skeleton_dict[f"skeleton_{skel_num}"]["asset_path"] = asset_path[skel_num]
            skeleton_dict[f"skeleton_{skel_num}"]["animation_variant"] = animation_variant[skel_num]
            skeleton_dict[f"skeleton_{skel_num}"]["skeleton_parents"] = (
                skeleton_parents[skel_num].tolist() if skeleton_parents else []
            )
            skeleton_dict[f"skeleton_{skel_num}"]["rest_global_translations"] = (
                rest_global_translations[skel_num].tolist() if rest_global_translations else []
            )
            skeleton_dict[f"skeleton_{skel_num}"]["rest_local_translations"] = (
                rest_local_translations[skel_num].tolist() if rest_local_translations else []
            )
            skeleton_dict[f"skeleton_{skel_num}"]["rest_local_rotations"] = (
                rest_local_rotations[skel_num].tolist() if rest_local_rotations else []
            )
            skeleton_dict[f"skeleton_{skel_num}"]["global_translations"] = (
                global_translations[skel_num].tolist() if global_translations else []
            )
            skeleton_dict[f"skeleton_{skel_num}"]["local_rotations"] = (
                local_rotations[skel_num].tolist() if local_rotations else []
            )
            skeleton_dict[f"skeleton_{skel_num}"]["translations_2d"] = (
                translations_2d[skel_num].tolist() if translations_2d else []
            )
            skeleton_dict[f"skeleton_{skel_num}"]["skeleton_joints"] = (
                skeleton_joints[skel_num] if skeleton_joints else []
            )
            skeleton_dict[f"skeleton_{skel_num}"]["joint_occlusions"] = (
                joint_occlusions[skel_num].tolist() if joint_occlusions else []
            )
            skeleton_dict[f"skeleton_{skel_num}"]["occlusion_types"] = (
                occlusion_types[skel_num] if occlusion_types else []
            )
            skeleton_dict[f"skeleton_{skel_num}"]["in_view"] = bool(in_view[skel_num]) if in_view.any() else False

        file_path = f"{output_path}skeleton_{self._sequence_id}{self._frame_id:0{self._frame_padding}}.json"

        self._backend.schedule(F.write_json, data=skeleton_dict, path=file_path)


# 注册到 Replicator WriterRegistry 后，ui_builder 中才能通过
# rep.WriterRegistry.get("LxWriter") 获取这个 Writer。
WriterRegistry.register(LxWriter)

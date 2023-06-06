# Copyright © 2022 Digital Agency. All rights reserved.
"""
|  Requirements: Python 3.9+.

|  円柱の空間ID取得モジュール
"""
import math
import itertools
from skspatial.objects import Point, Points, Line, Vector, Plane

from logging import getLogger
from SpatialId import LOGGER_NAME
from SpatialId.common.exception import SpatialIdError
from SpatialId.common.object.enum import Point_Option
from SpatialId.shape.line import (
    get_spatial_id_on_axis_ids,
    get_spatial_ids_on_line,
    generate_x_voxel_plane,
    generate_y_voxel_plane,
    generate_z_voxel_plane,
    detect_collision,
    get_voxel_id_to_spatial_id
)
from SpatialId.shape.point import (
    get_point_on_spatial_id,
    get_spatial_ids_on_points,
    convert_projected_point_list_to_point_list,
    convert_point_list_to_projected_point_list
)
from SpatialId.operated.shifting_spatial_id import get_shifting_spatial_id
from SpatialId.common.object.point import Point as SpatialPoint
from SpatialId.common.object.point import Projected_Point
from SpatialId.shape.bullet.capsule_bullet import CapsuleBullet
from SpatialId.shape.bullet.cylinder_bullet import CylinderBullet
from SpatialId.shape.bullet.sphere_bullet import SphereBullet

from SpatialId.convert import in_to_internal, internal_to_out

spatial_log = getLogger(LOGGER_NAME).getChild(__name__)

# CRSのデフォルト値(WGS84)のEPSGコード
__CRS__ = 4326
# 直交座標系のEPSGコード
__ORTH_CRS__ = 3857
# 浮動小数点誤差
__MINIMA__ = 1e-10
# 半径の下限値
__RADIUS_MINIMA__ = 1e-8


def _convert_spatial_point(point: Point) -> SpatialPoint:
    """ PointオブジェクトからSpatialPointオブジェクトへの変換関数

    :param point: Pointオブジェクト
    :type  point: skspatial.objects.Point

    :returns: SpatialPointオブジェクト
    :rtype:   SpatialId.common.object.point.Point
    """
    return SpatialPoint(point[0], point[1], point[2])


def _convert_spatial_projection_point(point: Point) -> Projected_Point:
    """ PointオブジェクトからProjected_Pointオブジェクトへの変換関数

    :param point: Pointオブジェクト
    :type  point: skspatial.objects.Point

    :returns: Projected_Pointオブジェクト
    :rtype:   SpatialId.common.object.point.Projected_Point
    """
    return Projected_Point(x=point[0], y=point[1], alt=point[2])


class Rectangular:
    """ 始点・終点の2面が正方形の直方体の空間ID

    :var _start_point: 直方体の始点。（高度も補正済みのWebメルカトル座標系）
    :var _end_point: 直方体の終点。（高度も補正済みのWebメルカトル座標系）
    :var _radius: 正方形の半径(単位:m)現実世界の長さ
    :var _all_cross_points: 全交点
    :var _all_spatial_ids: 全空間ID
    :var _height: 図形の長さ
    :var _unit_axis_vector: 軸方向の単位ベクトル
    :var _radius_orth_vector: 軸に対する半径長さの直交ベクトル
    :var _contact_point: 直交ベクトルと図形の交点
    :var _radius_normal_vector: 軸方向のベクトルと直交ベクトルの外積
    :var _factor: 始点座標においてのWebメルカトルの距離の倍率
    """
    def __init__(
        self, start_point: Point, end_point: Point, radius: float,
        h_zoom: int, v_zoom: int, factor: float
    ) -> None:
        """ コンストラクタ

        :param start_point: 直方体の始点。
        :type  start_point: skspatial.objects.Point
        :param end_point: 直方体の終点。
        :type  end_point: skspatial.objects.Point
        :param radius: 正方形の半径(単位:m)
        :type  radius: float
        :param h_zoom: 水平方向の精度レベル
        :type  h_zoom: int
        :param v_zoom: 垂直方向の精度レベル
        :type  v_zoom: int
        """
        # 【直交座標空間】
        self._start_point = start_point
        self._end_point = end_point
        self._radius = radius

        # 分解能取得
        self._v_zoom = v_zoom
        self._h_zoom = h_zoom

        # 全交点リスト
        self._all_cross_points = list()

        # 空間ID
        self._all_spatial_ids = set()

        # 現実距離→Webメルカトル換算係数
        self._factor = factor


    def _init_axis(self) -> None:
        """ 図形を構成する軸・ベクトルを初期化
        """
        # 【直交座標空間】軸ベクトル
        axis_vector = Vector.from_points(self._start_point, self._end_point)
        # 縦の長さ（Webメルカトル長さ）
        self._height = axis_vector.norm()
        x_vec, y_vec, z_vec = axis_vector

        # 開始点の直行ベクトルを一つ求める
        # 直行ベクトル
        start_orth_vector = None
        if abs(y_vec) < __MINIMA__:
            start_orth_vector = Vector([0, 1, 0])
        elif (abs(z_vec) < __MINIMA__):
            start_orth_vector = Vector([0, 0, 1])
        else:
            start_orth_vector = Vector([0, -z_vec, y_vec])

        # 直交ベクトルをWebメルカトル距離の半径の大きさにする。
        self._radius_orth_vector = self._radius * self._factor * start_orth_vector.unit()

        # 単位軸ベクトル
        self._unit_axis_vector = axis_vector.unit()

        # 円との接点
        self._contact_point = self._radius_orth_vector + self._start_point

        # 軸ベクトルと直交ベクトルの法線ベクトル(Webメルカトル距離)
        self._radius_normal_vector = \
            self._radius * self._factor * self._radius_orth_vector.cross(
                self._unit_axis_vector).unit()

    def _calc_rectangular_apex(self) -> None:
        """ 直方体の頂点を取得し、境界面との交点を取得
        """
        # 【直交座標空間】直方体の頂点
        apex_points = [0] * 8
        apex_points[0] = self._contact_point + self._radius_normal_vector
        apex_points[1] = self._contact_point - self._radius_normal_vector
        apex_points[2] = apex_points[0] - 2 * self._radius_orth_vector
        apex_points[3] = apex_points[1] - 2 * self._radius_orth_vector
        apex_points[4] = apex_points[0] + self._height * self._unit_axis_vector
        apex_points[5] = apex_points[1] + self._height * self._unit_axis_vector
        apex_points[6] = apex_points[2] + self._height * self._unit_axis_vector
        apex_points[7] = apex_points[3] + self._height * self._unit_axis_vector

        # Z成分のみWebメルカトル長さ→mに戻す
        for p in apex_points:
            p[2] /= self._factor

        spatial_log.debug("直方体の頂点: %s", apex_points)

        # 【直交座標空間】直方体の辺
        rect_lines = [
            Line.from_points(apex_points[0], apex_points[1]),
            Line.from_points(apex_points[0], apex_points[4]),
            Line.from_points(apex_points[1], apex_points[5]),
            Line.from_points(apex_points[4], apex_points[5]),
            Line.from_points(apex_points[2], apex_points[3]),
            Line.from_points(apex_points[2], apex_points[6]),
            Line.from_points(apex_points[6], apex_points[7]),
            Line.from_points(apex_points[3], apex_points[7]),
            Line.from_points(apex_points[0], apex_points[2]),
            Line.from_points(apex_points[1], apex_points[3]),
            Line.from_points(apex_points[4], apex_points[6]),
            Line.from_points(apex_points[5], apex_points[7])
        ]
        spatial_log.debug("直方体の辺: %s", rect_lines)

        # 【直交座標空間】直方体の各辺に対して空間IDを取得する
        for rect_line in rect_lines:
            edge_start = _convert_spatial_projection_point(rect_line.point)
            edge_end = _convert_spatial_projection_point(rect_line.to_point())
            # 【緯度経度空間】直方体の各辺の頂点を緯度経度に変換
            edge_wgs84_start, edge_wgs84_end = \
                convert_projected_point_list_to_point_list(
                    [edge_start, edge_end],
                    __ORTH_CRS__
                )
            edge_start.lon = edge_wgs84_start.lon
            edge_start.lat = edge_wgs84_start.lat
            edge_end.lon = edge_wgs84_end.lon
            edge_end.lat = edge_wgs84_end.lat
            self._all_spatial_ids \
                |= set(get_spatial_ids_on_line(
                     edge_start,
                     edge_end,
                     self._h_zoom, self._v_zoom, __CRS__)
                )

        spatial_log.debug(
            "直方体の辺に対して空間ID: %s",
            self._all_spatial_ids
        )

        # 【直交座標空間】直方体頂点の空間IDを取得
        edge_point_list = [
            _convert_spatial_projection_point(apex_point)
            for apex_point in apex_points
        ]
        edge_wgs84_list = convert_projected_point_list_to_point_list(
            edge_point_list, __ORTH_CRS__
        )
        edge_point_spatial_ids = get_spatial_ids_on_points(
            edge_wgs84_list, self._h_zoom, self._v_zoom
        )
        spatial_log.debug(
            "直方体頂点の空間ID: %s",
            edge_point_spatial_ids
        )

        edge_point_spatial_indexes = [
            get_voxel_id_to_spatial_id(spatial_id)
            for spatial_id in edge_point_spatial_ids
        ]

        # 【直交座標空間】直方体の各頂点の最大経度・最小経度を取得
        rect_x_index_list = [
            spatial_index[0] for spatial_index in edge_point_spatial_indexes
        ]
        min_rect_x_index = min(rect_x_index_list)
        max_rect_x_index = max(rect_x_index_list)
        spatial_log.debug(
            "直方体頂点空間IDのX成分範囲: %s～%s",
            min_rect_x_index,
            max_rect_x_index
        )

        # 【直交座標空間】直方体の各頂点の最大緯度・最小緯度を取得
        rect_y_index_list = [
            spatial_index[1] for spatial_index in edge_point_spatial_indexes
        ]
        min_rect_y_index = min(rect_y_index_list)
        max_rect_y_index = max(rect_y_index_list)
        spatial_log.debug(
            "直方体頂点空間IDのY成分範囲: %s～%s",
            min_rect_y_index,
            max_rect_y_index
        )

        # 【直交座標空間】直方体の各頂点の最大高さ・最小高さを取得
        rect_z_index_list = [
            spatial_index[2] for spatial_index in edge_point_spatial_indexes
        ]
        min_rect_z_index = min(rect_z_index_list)
        max_rect_z_index = max(rect_z_index_list)
        spatial_log.debug(
            "直方体頂点空間IDのZ成分範囲: %s～%s",
            min_rect_z_index,
            max_rect_z_index
        )

        # 【直交座標空間】Xボクセル境界面のイテレータ取得
        x_voxel_plane_itr = generate_x_voxel_plane(
            min_rect_x_index,
            max_rect_x_index,
            self._h_zoom,
            self._v_zoom
        )
        # ループ用に経度ボクセル境界面のイテレータをリスト化
        x_voxel_planes = list(x_voxel_plane_itr)

        # 【緯度経度空間⇒直交座標空間】緯度ボクセル境界面のイテレータ取得
        y_voxel_plane_itr = generate_y_voxel_plane(
            min_rect_y_index,
            max_rect_y_index,
            self._h_zoom,
            self._v_zoom
        )
        # ループ用に緯度ボクセル境界面のイテレータをリスト化
        y_voxel_planes = list(y_voxel_plane_itr)

        # 【直交座標空間】高さボクセル境界面のイテレータ取得
        z_voxel_plane_itr = generate_z_voxel_plane(
            min_rect_z_index,
            max_rect_z_index,
            self._h_zoom,
            self._v_zoom
        )
        # ループ用に高さボクセル境界面のイテレータをリスト化
        z_voxel_planes = list(z_voxel_plane_itr)

        # X境界面との交点を取得
        self._get_voxel_plane_cross_point(
            rect_lines,
            x_voxel_planes,
            y_voxel_planes,
            z_voxel_planes,
            2
        )
        # Y境界面との交点を取得
        self._get_voxel_plane_cross_point(
            rect_lines,
            y_voxel_planes,
            z_voxel_planes,
            x_voxel_planes,
            0
        )
        # Z境界面との交点を取得
        self._get_voxel_plane_cross_point(
            rect_lines,
            z_voxel_planes,
            x_voxel_planes,
            y_voxel_planes,
            1
        )

    def _get_voxel_plane_cross_point(
        self,
        rect_lines: list[Line],
        first_voxel_planes: list[Plane],
        second_voxel_planes: list[Plane],
        third_voxel_planes: list[Plane],
        axis: int
    ) -> None:
        """ 境界面との交点取得

        :param rect_lines: 直方体の辺全て。
        :type  rect_lines: list[skspatial.objects.line.Line]
        :param first_voxel_planes: 第一ボクセル境界面
        :type  first_voxel_planes: list[skspatial.objects.plane.Plane]
        :param second_voxel_planes: 第二ボクセル境界面
        :type  second_voxel_planes: list[skspatial.objects.plane.Plane]
        :param third_voxel_planes: 第三ボクセル境界面
        :type  third_voxel_planes: list[skspatial.objects.plane.Plane]
        :param axis: 処理時に基準とするXYZ軸(X=0, Y=1, Z=2)
        :type  axis: int
        """
        # 【直交座標空間】第一ボクセル境界面毎に処理
        for first_plane in first_voxel_planes:

            # 【直交座標空間】第一ボクセル境界面に対する辺の交点を求める
            plane_collision_points = list()
            for rect_line in rect_lines:

                # 【直交座標空間】第一ボクセル境界と辺の交点の座標を取得
                collision_point = detect_collision(first_plane, rect_line)
                # 辺と第一ボクセル境界が衝突しない場合はスキップ
                if collision_point is None:
                    continue

                plane_collision_points.append(collision_point)

            # 接触しない場合はスキップ
            if len(plane_collision_points) == 0:
                continue

            # 交点をユニークにする
            unique_plane_collision_points \
                = Points(plane_collision_points).unique()

            # 第一ボクセル境界への交点が1もしくは2の場合は辺の空間IDとして取得出来ているので、処理をスキップ
            if len(unique_plane_collision_points) <= 2:
                continue

            # 【直交座標空間】全交点リストに追加
            for unique_plane_collision_point in unique_plane_collision_points:
                spatial_log.debug(
                    "第一ボクセル境界への交点: %s",
                    unique_plane_collision_point
                )
                self._all_cross_points.append(
                    Point(unique_plane_collision_point)
                )

            # 第一ボクセル境界への交点を結んだ交線リストを作成
            plane_collision_lines = [
                Line.from_points(start_point, end_point)
                for start_point, end_point
                in itertools.combinations(unique_plane_collision_points, 2)
            ]
            spatial_log.debug(
                "第一ボクセル境界への交点を結んだ交線リスト: %s",
                plane_collision_lines
            )

            # 交線ごとの交点辞書を作成（キーは交線リストのインデックス）
            collision_line_cross_points = dict()
            for index, _ in enumerate(plane_collision_lines):
                collision_line_cross_points[index] = list()

            # 【直交座標空間】第二ボクセル境界面毎に処理
            second_collision_lines = list()
            for second_plane in second_voxel_planes:

                second_collision_points = list()
                # 交線に対する第二ボクセル境界面の交点を取得
                for index, plane_collision_line in \
                        enumerate(plane_collision_lines):

                    # 【直交座標空間】第二ボクセル境界と辺の交点の座標を取得
                    collision_point = detect_collision(
                        second_plane, plane_collision_line
                    )
                    # 辺と第二ボクセル境界が衝突しない場合はスキップ
                    if collision_point is None:
                        continue

                    second_collision_points.append(collision_point)
                    collision_line_cross_points[index].append(collision_point)

                # 接触しない場合はスキップ
                if len(second_collision_points) == 0:
                    continue

                # 交点をユニークにする
                unique_second_collision_points \
                    = Points(second_collision_points).unique()

                # 接触点が1の場合は接触点は辺なので取得済みとしてスキップ
                if len(unique_second_collision_points) <= 1:
                    continue

                # 【直交座標空間】全交点リストに追加
                for unique_second_collision_point \
                        in unique_second_collision_points:
                    spatial_log.debug(
                        "第二ボクセル境界への交点: %s",
                        unique_second_collision_point
                    )
                    self._all_cross_points.append(
                        Point(unique_second_collision_point)
                    )

                # 第二ボクセル交線を取得
                second_max_collision_point = \
                    max(
                        unique_second_collision_points,
                        key=lambda point: point[axis]
                    )
                second_min_collision_point = \
                    min(
                        unique_second_collision_points,
                        key=lambda point: point[axis]
                    )
                second_collision_lines.append(
                    Line.from_points(
                        second_max_collision_point,
                        second_min_collision_point
                    )
                )

            # 【直交座標空間】第三ボクセル境界面毎に処理
            for third_plane in third_voxel_planes:

                third_collision_points = list()
                for second_collision_line in second_collision_lines:

                    # 【直交座標空間】第三ボクセル境界と第二ボクセル交線の交点の座標を取得
                    collision_point = detect_collision(
                        third_plane, second_collision_line
                    )
                    # 第二ボクセル交線と第三ボクセル境界が衝突しない場合はスキップ
                    if collision_point is None:
                        continue

                    spatial_log.debug(
                        "第三ボクセル境界と第二ボクセル交線の交点: %s",
                        collision_point
                    )
                    third_collision_points.append(collision_point)
                    # 【直交座標空間】全交点リストに追加
                    self._all_cross_points.append(collision_point)

                # 接触しない場合はスキップ
                if len(third_collision_points) == 0:
                    continue

                # 第一交線に対する第三ボクセル境界面の交点を取得
                for index, plane_collision_line in \
                        enumerate(plane_collision_lines):

                    # 【直交座標空間】第三ボクセル境界と第一ボクセル交線の交点の座標を取得
                    collision_point = detect_collision(
                        third_plane, plane_collision_line
                    )
                    # 辺と経度ボクセル境界が衝突しない場合はスキップ
                    if collision_point is None:
                        continue

                    spatial_log.debug(
                        "第三ボクセル境界と第一ボクセル交線の交点の交点: %s",
                        collision_point
                    )
                    collision_line_cross_points[index].append(collision_point)
                    # 【直交座標空間】全交点リストに追加
                    self._all_cross_points.append(collision_point)

            for index, cross_points in collision_line_cross_points.items():
                # 第一ボクセル境界面上の線の方向ベクトル
                plane_collision_direction \
                    = plane_collision_lines[index].direction

                # 並び替え時の軸
                sort_axis = None
                # 線の方向ベクトルのX成分がある場合はX軸
                if abs(plane_collision_direction[0]) > __MINIMA__:
                    sort_axis = 0

                # 線の方向ベクトルのY成分がある場合はY軸
                elif abs(plane_collision_direction[1]) > __MINIMA__:
                    sort_axis = 1

                # 線の方向ベクトルのz成分がある場合はZ軸
                else:
                    sort_axis = 2

                sort_cross_points = sorted(
                    cross_points,
                    key=lambda val: val[sort_axis]
                )
                for index, _ in enumerate(sort_cross_points[:-1]):
                    # 第一ボクセル交線の第二ボクセル境界・第三ボクセル境界との交点の中点取得
                    middle_cross_point = \
                        (sort_cross_points[index]
                            + sort_cross_points[index+1]) / 2
                    spatial_log.debug(
                        "第一ボクセル交線の第二ボクセル境界・"
                        + "第三ボクセル境界との交点の中点: %s",
                        middle_cross_point
                    )
                    # 【直交座標空間】全交点リストに追加
                    self._all_cross_points.append(middle_cross_point)

    def _calc_cross_spatial_ids(self) -> None:
        """ 交点の空間IDを取得
        """

        # 【直交座標空間⇒緯度経度空間】交点の座標
        all_cross_project_points = [
            _convert_spatial_projection_point(cross_point)
            for cross_point
            in self._all_cross_points
        ]
        cross_wgs84_points = \
            convert_projected_point_list_to_point_list(
                all_cross_project_points, __ORTH_CRS__
            )
        spatial_log.debug(
            "交点の座標(直交座標): %s",
            all_cross_project_points
        )
        spatial_log.debug(
            "交点の座標(地理座標): %s",
            cross_wgs84_points
        )

        # 空間ID
        cross_spatial_ids = set(get_spatial_ids_on_points(
                cross_wgs84_points, self._h_zoom, self._v_zoom))
        spatial_log.debug(
            "交点の空間IDを取得: %s",
            cross_spatial_ids
        )
        self._all_spatial_ids |= cross_spatial_ids

    def calc_spatial_ids(self) -> None:
        """ 空間IDの取得実行
        """
        # 軸の初期化
        self._init_axis()
        # 直方体の境界面に対する交点取得
        self._calc_rectangular_apex()
        # 交点の空間ID取得
        self._calc_cross_spatial_ids()

    def get_spatial_ids(self) -> set[str]:
        """ 空間IDを返却

        :returns: 空間IDのセット
        :rtype:   set[str]
        """
        return self._all_spatial_ids


class Capsule(Rectangular):
    """ カプセル・円柱・球の空間ID

    :var _is_sphere: 球であるかの判定。True: 球 / False: 円柱 or カプセル
    :var _start_point: 円柱・カプセル・球の始点。（高度も補正済みのWebメルカトル座標系）
    :var _end_point: 円柱・カプセル・球の終点。（高度も補正済みのWebメルカトル座標系）
    :var _radius: 円柱の半径(単位:m) 現実世界の長さ
    :var _all_cross_points: 全交点
    :var _all_spatial_ids: 全空間ID
    :var _height: 図形の長さ
    :var _unit_axis_vector: 軸方向の単位ベクトル
    :var _radius_orth_vector: 軸に対する半径長さの直交ベクトル
    :var _contact_point: 直交ベクトルと図形の交点
    :var _radius_normal_vector: 軸方向のベクトルと直交ベクトルの外積
    :var _is_capsule: カプセルであるかを示す。True: カプセル / False: 円柱
    :var _is_precision: 衝突判定実施オプション。True: 衝突判定実施 / False: 衝突判定スキップ
    :var _include_spatial_ids: 内部判定空間ID
    :var _factor: 始点座標においてのWebメルカトルの距離の倍率
    """

    def __init__(
        self,
        start_point: Point,
        end_point: Point,
        radius: float,
        h_zoom: int,
        v_zoom: int,
        is_capsule: bool,
        is_precision: bool,
        factor: float
    ) -> None:
        """ コンストラクタ

        :param start_point: 円柱・カプセル・球の始点。
        :type  start_point: skspatial.objects.Point
        :param end_point: 円柱・カプセル・球の終点。
        :type  end_point: skspatial.objects.Point
        :param radius: 円柱の半径(単位:m)
        :type  radius: float
        :param h_zoom: 水平方向の精度レベル
        :type  h_zoom: int
        :param v_zoom: 垂直方向の精度レベル
        :type  v_zoom: int
        :param is_capsule: 始点、終点が球状であるかを示す。True: カプセル / False: 円柱
        :type  is_capsule: bool
        :param is_precision: 衝突判定実施オプション。True: 衝突判定実施 / False: 衝突判定スキップ
        :type  is_precision: bool
        """
        super().__init__(start_point, end_point, radius, h_zoom, v_zoom, factor)

        self._is_sphere = False
        if Points([start_point]).is_close(end_point):
            self._bullet_object = SphereBullet(radius * factor, start_point)
            self._is_sphere = True
        elif is_capsule:
            self._bullet_object = \
                CapsuleBullet(radius * factor, start_point, end_point)
        else:
            self._bullet_object = \
                CylinderBullet(radius * factor, start_point, end_point)

        self._is_capsule = is_capsule
        self._is_precision = is_precision 
        self._include_spatial_ids = set()

    def _init_axis(self) -> None:
        """ 図形を構成する軸・ベクトルを初期化
        """
        # 【直交座標空間】軸ベクトル
        axis_vector = Vector.from_points(self._start_point, self._end_point)
        # 縦の長さ（Webメルカトル長さ）
        self._height = axis_vector.norm()
        # カプセル/球の場合
        if self._is_capsule or self._is_sphere:
            self._height += 2 * self._radius * self._factor
        x_vec, y_vec, z_vec = axis_vector

        # 開始点の直行ベクトルを一つ求める
        # 直行ベクトル
        start_orth_vector = None
        if abs(y_vec) < __MINIMA__ or self._is_sphere:
            start_orth_vector = Vector([0, 1, 0])
        elif (abs(z_vec) < __MINIMA__):
            start_orth_vector = Vector([0, 0, 1])
        else:
            start_orth_vector = Vector([0, -z_vec, y_vec])

        # 直交ベクトルを半径にWebメルカトル座標系の係数をかけた大きさにする。
        self._radius_orth_vector = self._radius * self._factor * start_orth_vector.unit()

        # 単位軸ベクトル
        if self._is_sphere:
            self._unit_axis_vector = Vector([1, 0, 0])
        else:
            self._unit_axis_vector = axis_vector.unit()

        # 円柱は軸ベクトルを零ベクトルとする
        if not self._is_capsule and not self._is_sphere:
            radius_axis_vector = Vector([0, 0, 0])
        # 球・カプセルは軸ベクトルを半径にWebメルカトル座標系の係数をかけた大きさにする。
        else:
            radius_axis_vector = self._radius * self._factor * self._unit_axis_vector

        # 円との接点
        self._contact_point = \
            (self._radius_orth_vector - radius_axis_vector) \
            + self._start_point

        # 軸ベクトルと直交ベクトルの法線ベクトル(大きさは半径にWebメルカトル座標系の係数をかけた値)
        self._radius_normal_vector = \
            self._radius * self._factor \
            * self._radius_orth_vector.cross(self._unit_axis_vector).unit()

    def _approximate_distance_radius_points(
        self,
        spatial_id: str
    ) -> set[str]:
        """
        概算距離内の空間ID取得
        :param spatial_id: 概算距離の中心とする空間ID
        :type  spatial_id: str

        :returns:  概算距離内の空間ID
        :rtypes:   set[str]
        """
        approximate_distance_radius_points = set()
        for x_index in range(
                -self._approximate_voxel_num,
                self._approximate_voxel_num + 1):
            y_range = self._approximate_voxel_num - abs(x_index)
            for y_index in range(-y_range, y_range+1):
                z_range = y_range - abs(y_index)
                for z_index in range(-z_range, z_range+1):

                    # 概算距離内のボクセル数を上限に空間IDをシフト
                    approximate_id = get_shifting_spatial_id(
                        spatial_id, x_index, y_index, z_index
                    )
                    approximate_distance_radius_points.add(approximate_id)

        return approximate_distance_radius_points

    def _calc_valid_spatial_ids(self) -> None:
        """ 直方体の空間IDから実際の図形分の有効な空間IDを取得
        """
        # 【直交座標空間】対角線分の単位ボクセルの距離(概算距離)
        base_spatial_id = get_spatial_id_on_axis_ids(
            0,
            0 if self._h_zoom == 0 else 2 ** (self._h_zoom - 1),
            0,
            self._h_zoom,
            self._v_zoom
        )
        unit_voxel_vector = self._calc_unit_voxel_vector(base_spatial_id) # 基準ボクセルのWebメルカトルベクトル
        # 【直交座標空間】始点から対角線分の単位ボクセルの座標
        approximate_distance = unit_voxel_vector.norm()
        # 【直交座標空間】概算ボクセル数（概算距離が半径にWebメルカトル座標系の係数をかけた値内のボクセル数）
        self._approximate_voxel_num = \
            math.floor(self._radius * self._factor / approximate_distance)
        spatial_log.debug(
            "対角線分の単位ボクセルの距離(概算距離): %s",
            unit_voxel_vector
        )
        spatial_log.debug(
            "概算ボクセル数（概算距離が半径内のボクセル数）: %s",
            self._approximate_voxel_num
        )

        # 【直交座標空間⇒緯度経度空間】始点終の座標
        start_end_project_points = [
            _convert_spatial_projection_point(self._start_point),
            _convert_spatial_projection_point(self._end_point)
        ]
        start_end_project_points[0].alt /= self._factor # 高度をWebメルカトル長さ→mに変換
        start_end_project_points[1].alt /= self._factor # 高度をWebメルカトル長さ→mに変換
        
        start_end_wgs84_points = \
            convert_projected_point_list_to_point_list(
                start_end_project_points, __ORTH_CRS__
            )
        start_end_project_points[0].lon = start_end_wgs84_points[0].lon
        start_end_project_points[0].lat = start_end_wgs84_points[0].lat
        start_end_project_points[1].lon = start_end_wgs84_points[1].lon
        start_end_project_points[1].lat = start_end_wgs84_points[1].lat
        # 【直交座標空間】始点終点間の空間ID取得
        # 円柱・カプセルの場合
        if not self._is_sphere:
            line_spatial_ids \
                = get_spatial_ids_on_line(
                    start_end_project_points[0],
                    start_end_project_points[1],
                    self._h_zoom,
                    self._v_zoom,
                    __CRS__
                )
            spatial_log.debug(
                "円柱・カプセルの場合、始点終点間の空間ID取得: %s",
                line_spatial_ids
            )

        # 球の場合
        else:
            line_spatial_ids = get_spatial_ids_on_points(
                start_end_wgs84_points, self._h_zoom, self._v_zoom
            )
            spatial_log.debug(
                "球の場合、接続点の空間ID取得: %s",
                line_spatial_ids
            )

        # 【直交空間】円柱の空間ID簡易取得
        for line_spatial_id in line_spatial_ids:
            approximate_distance_radius_spatial_ids \
                = self._approximate_distance_radius_points(line_spatial_id)
            spatial_log.debug(
                "円柱の空間ID簡易取得(起点): %s, ",
                line_spatial_id
            )
            spatial_log.debug(
                "円柱の空間ID簡易取得(結果): %s, ",
                approximate_distance_radius_spatial_ids
            )
            self._include_spatial_ids \
                |= approximate_distance_radius_spatial_ids \
                & self._all_spatial_ids

        # PyBulletと衝突判定を行う空間ID
        exclude_spatial_ids = \
            self._all_spatial_ids - self._include_spatial_ids
        spatial_log.debug(
            "PyBulletと衝突判定を行う空間ID: %s",
            exclude_spatial_ids
        )

        lat_dict = dict()
        for exclude_spatial_id in exclude_spatial_ids:
            lat = get_voxel_id_to_spatial_id(exclude_spatial_id)[1]
            # 【直交座標空間】ボクセルの中心からの頂点までのベクトル
            if lat in lat_dict.keys():
                half_extent = lat_dict[lat]
            else:
                half_extent = self._calc_unit_voxel_vector(exclude_spatial_id) / 2
                lat_dict[lat] = half_extent
            # 【直交座標空間】ボクセルの中心座標(Z座標をWebメルカトル長さに補正する)
            center_point = \
                get_point_on_spatial_id(
                    exclude_spatial_id, Point_Option.CENTER, __ORTH_CRS__
                )[0]
            center_orth_point \
                = Point([center_point.x, center_point.y, center_point.alt * self._factor])
            spatial_log.debug(
                "ボクセルの中心座標: %s, %s, %s",
                center_orth_point[0], center_orth_point[1], center_orth_point[2]
            )
            # PyBulletと衝突判定
            if self._bullet_object.is_collide_voxcel(
                    center_orth_point, half_extent):
                spatial_log.debug(
                    "PyBulletと衝突: %s", exclude_spatial_id
                )
                self._include_spatial_ids.add(exclude_spatial_id)

    def _calc_unit_voxel_vector(self, spatial_id) -> Vector:
        """概算距離用の単位ボクセルベクトルを算出（Z成分をWebメルカトル基準に補正）

        :args: 空間ID
        :type: str

        :returns: 単位ボクセルベクトル
        :rtype: Vector
        """
        # 引数の空間IDの頂点座標を取得
        points = get_point_on_spatial_id(
            spatial_id, Point_Option.VERTEX, __ORTH_CRS__
        )
        # 各XYZ成分座標から単位ボクセルベクトルを算出
        point_x_max = max([point.x for point in points])
        point_x_min = min([point.x for point in points])
        point_y_max = max([point.y for point in points])
        point_y_min = min([point.y for point in points])
        point_z_max = max([point.alt for point in points])
        point_z_min = min([point.alt for point in points])
        return Vector([
            math.floor((point_x_max - point_x_min) * 10 ** 6) / 10 ** 6,
            math.floor((point_y_max - point_y_min) * 10 ** 6) / 10 ** 6,
            math.floor((point_z_max - point_z_min) * 10 ** 6) * self._factor / 10 ** 6
        ])

    def shave_sphere(self, direction_point) -> None:
        """ 球の空間IDから中心から終点方向へ半径分延ばした立方体分の空間IDを削る

        :param directtion_point: 終点方向
        :type  directtion_point: skspatial.objects.Point
        """
        # 【直交座標空間】単位軸ベクトル
        unit_axis_vector = \
            Vector.from_points(self._start_point, direction_point).unit()
        # 【直交座標空間】直方体の終点
        direction_end_point = \
            self._radius * unit_axis_vector * self._factor + self._start_point

        # 中心から終点方向へ半径分延ばした立方体分の空間IDを取得
        except_rectangular = Rectangular(
            self._start_point,
            direction_end_point,
            self._radius,
            self._h_zoom,
            self._v_zoom,
            self._factor
        )
        except_rectangular.calc_spatial_ids()

        spatial_log.debug(
            "中心から半径分延ばした立方体分の空間IDを取得: %s",
            except_rectangular.get_spatial_ids()
        )

        self._include_spatial_ids = \
            self._include_spatial_ids - except_rectangular.get_spatial_ids()

    def calc_spatial_ids(self) -> None:
        """ 空間IDを計算取得
        """
        super().calc_spatial_ids()

        if self._is_precision:
            # 交点の有効な空間ID取得
            self._calc_valid_spatial_ids()

        # 衝突判定割愛
        else:
            self._include_spatial_ids = self._all_spatial_ids

    def get_spatial_ids(self) -> set[str]:
        """ 空間ID取得

        :returns: 空間IDのセット
        :rtype: set[str]
        """
        return self._include_spatial_ids

def get_spatial_ids_on_cylinders(
        center: list[Point],
        radius: float,
        h_zoom: int,
        v_zoom: int,
        crs: int = __CRS__,
        is_capsule: bool = False,
        is_precision: bool =True
) -> list[str]:
    """
    | 円柱を複数つなげた経路が通る空間IDを取得する。
    | 円柱間の接続面は球状とする。
    | ドローンの経路や地中埋設配管が通る経路を空間IDで表現する際に使用する。

    :param center: 円柱の中心の接続点。Pointを複数指定するリスト。
                   2つ目の接続点は1つ目の円柱の終点となるが、2つ目の円柱の始点にもなる。
    :type  center: list[SpatialId.common.object.point.Point]
    :param radius: 円柱の半径(単位:m)
    :type  radius: float
    :param h_zoom: 水平方向の精度レベル
    :type  h_zoom: int
    :param v_zoom: 垂直方向の精度レベル
    :type  v_zoom: int
    :param crs: 座標参照系
    :type  crs: int
    :param is_capsule: 始点、終点が球状であるかを示す。True: カプセル / False: 円柱
    :type  is_capsule: bool
    :param is_precision: 衝突判定実施オプション。True: 衝突判定実施 / False: 衝突判定スキップ
    :type  is_precision: bool

    :returns: 円柱を複数つなげた経路が通る空間IDのリスト
    :rtype:  list[str]

    :raises SpatialIdError: 入力チェック結果が不正の場合のエラー
    :raises SpatialIdError: 座標の値が不正の場合、もしくは円柱の半径が0以下の場合、エラー
    :raises SpatialIdError: 入力されたEPSGコードが存在しない場合、エラー
    :raises SpatialIdError: 入力されたEPSGコードが地理座標系ではない場合、エラー
    :注意: 経度180度をまたがる点をまたがる円柱は空間ID化できない。
    """
    try:
        # 入力引数チェック
        _validate_args_type(center, radius, h_zoom, v_zoom, crs, is_capsule, is_precision)

        # 半径が0以下の場合は例外を投げる
        if radius <= __RADIUS_MINIMA__:
            spatial_log.debug("半径が0以下")
            raise SpatialIdError("INPUT_VALUE_ERROR")

        # 接続点数が0の場合は空配列を返却
        elif len(center) == 0:
            spatial_log.debug("接続点数が0個")
            return list()

        # 開始点の緯度から現実距離→Webメルカトル距離への変換係数を算出
        factor = 1 / math.cos(math.radians(center[0].lat))

        spatial_ids = set()
        # 接続点の球
        sphere = None
        # 接続点数
        connect_point_num = 1
        # 入力の接続点数-1の数だけループ
        for index, _ in enumerate(center[:-1]):
            start = center[index]
            end = center[index + 1]
            spatial_log.debug(
                "接続点: %s～%s", connect_point_num, connect_point_num + 1
            )
            spatial_log.debug(
                "始点(地理座標系): %s, %s, %s",
                start.lon, start.lat, start.alt
            )
            spatial_log.debug(
                "終点(地理座標系): %s, %s, %s",
                end.lon, end.lat, end.alt
            )

            # 【直交座標空間】始点終点の座標
            start_crs_point, end_crs_point \
                = convert_point_list_to_projected_point_list(
                    [start, end], __ORTH_CRS__, crs
                )
            start_orth_point = Point([
                start_crs_point.x,
                start_crs_point.y,
                start_crs_point.alt * factor
            ])
            end_orth_point = Point([
                end_crs_point.x,
                end_crs_point.y,
                end_crs_point.alt * factor
            ])
            spatial_log.debug(
                "始点(投影座標系): %s, %s, %s",
                start_orth_point[0],
                start_orth_point[1],
                start_orth_point[2]
            )
            spatial_log.debug(
                "終点(投影座標系): %s, %s, %s",
                end_orth_point[0],
                end_orth_point[1],
                end_orth_point[2]
            )

            # 始点終点の長さ(現実距離)
            axis = Vector.from_points(
                start_orth_point, end_orth_point
            )
            height = axis.norm() / factor

            # 同じ座標が連続する場合は次の座標へスキップ
            try:
                axis.unit()
            except ValueError:
                spatial_log.debug(
                    "同一座標の接続点が連続しているためスキップ"
                )
                continue

            # 接続点数をインクリメント
            connect_point_num += 1

            if sphere is not None:
                # 円柱の接続点で後方の円柱の長さが半径より短い場合
                if height < radius:
                    # 接続点の球の空間IDより円柱の始点から終点方向へ半径距離延ばした直方体の空間IDを除外する
                    sphere.shave_sphere(end_orth_point)

                shave_sphere_spatial_ids = sphere.get_spatial_ids()
                spatial_ids |= shave_sphere_spatial_ids
                spatial_log.debug(
                    "接続点の空間ID: %s",
                    shave_sphere_spatial_ids
                )

            # 【直交座標空間】始点終点からカプセルの空間ID取得
            capsule = Capsule(
                start_orth_point,
                end_orth_point,
                radius,
                h_zoom,
                v_zoom,
                is_capsule,
                is_precision,
                factor
            )
            capsule.calc_spatial_ids()
            capsule_spatial_ids = capsule.get_spatial_ids()
            spatial_ids |= capsule_spatial_ids
            spatial_log.debug(
                "接続点間の空間ID: %s",
                capsule_spatial_ids
            )

            # 終点もしくはカプセルの場合は接続点の空間IDは取得しない
            if index == (len(center[:-1]) - 1) or is_capsule:
                continue

            # 【直交座標空間】接続点の球の空間ID取得
            sphere = Capsule(
                end_orth_point,
                end_orth_point,
                radius,
                h_zoom,
                v_zoom,
                is_capsule,
                is_precision,
                factor
            )
            sphere.calc_spatial_ids()

            # 円柱の接続点で前方の円柱の長さが半径より短い場合
            if height < radius:
                # 接続点の球の空間IDより円柱の終点から始点方向へ半径距離延ばした直方体の空間IDを除外する
                sphere.shave_sphere(start_orth_point)

        # 接続点数が1の場合
        if connect_point_num == 1:
            spatial_log.debug("接続点数が1個")

            # 【直交座標空間】中心の座標
            center_crs_point \
                = convert_point_list_to_projected_point_list(
                    [center[0]], __ORTH_CRS__, crs
                )[0]
            center_orth_point = Point([
                center_crs_point.x,
                center_crs_point.y,
                center_crs_point.alt * factor
            ])
            spatial_log.debug(
                "中心座標(地理座標系): %s, %s, %s",
                center[0].lon,
                center[0].lat,
                center[0].alt
            )
            spatial_log.debug(
                "中心座標(投影座標系): %s, %s, %s",
                center_orth_point[0],
                center_orth_point[1],
                center_orth_point[2]
            )

            # 【直交座標空間】接続点の球の空間ID取得
            sphere = Capsule(
                center_orth_point,
                center_orth_point,
                radius,
                h_zoom,
                v_zoom,
                is_capsule,
                is_precision,
                factor
            )
            sphere.calc_spatial_ids()

            spatial_ids = sphere.get_spatial_ids()
            spatial_log.debug("球の空間ID: %s", spatial_ids)

        return list(spatial_ids)

    except SpatialIdError:
        # 内部処理で検出済みの例外を再送
        raise

    except Exception as e:
        # その他例外を投げる
        raise SpatialIdError("OTHER_ERROR") from e

def f_get_spatial_ids_on_cylinders( center: list[Point],
        radius: float,
        zoom: int,
        crs: int = __CRS__,
        is_capsule: bool = False,
        is_precision: bool =True
) -> list[str]:
    """
    | 円柱を複数つなげた経路が通る空間IDを取得する。
    | 円柱間の接続面は球状とする。
    | ドローンの経路や地中埋設配管が通る経路を空間IDで表現する際に使用する。

    | f_"がついてないAPIとの違いは以下：
    | 入力の空間ID精度レベルを水平・垂直方向共に統一
    | 出力の空間IDの並びは仕様通りの記載。
    | z/f/x/y :z{ズームレベル}/f{高さの位置}/x{経度方向の位置}/y{緯度方向の位置}
    | 参照：https://github.com/unvt/zfxy-spec
    
    :param center: 円柱の中心の接続点。Pointを複数指定するリスト。
                   2つ目の接続点は1つ目の円柱の終点となるが、2つ目の円柱の始点にもなる。
    :type  center: list[SpatialId.common.object.point.Point]
    :param radius: 円柱の半径(単位:m)
    :type  radius: float
    :param zoom: 空間IDの精度レベル
    :type  zoom: int
    :param crs: 座標参照系
    :type  crs: int
    :param is_capsule: 始点、終点が球状であるかを示す。True: カプセル / False: 円柱
    :type  is_capsule: bool
    :param is_precision: 衝突判定実施オプション。True: 衝突判定実施 / False: 衝突判定スキップ
    :type  is_precision: bool

    :returns: 円柱を複数つなげた経路が通る空間IDのリスト
    :rtype:  list[str]

    :raises SpatialIdError: 入力チェック結果が不正の場合のエラー
    :raises SpatialIdError: 座標の値が不正の場合、もしくは円柱の半径が0以下の場合、エラー
    :raises SpatialIdError: 入力されたEPSGコードが存在しない場合、エラー
    :raises SpatialIdError: 入力されたEPSGコードが地理座標系ではない場合、エラー
    :注意: 経度180度をまたがる点をまたがる円柱は空間ID化できない。
    """
    out = get_spatial_ids_on_cylinders(center, radius, zoom, zoom, crs, is_capsule, is_precision)
    return internal_to_out(out)


def _validate_args_type(
    center: list[Point],
    radius: float,
    h_zoom: int,
    v_zoom: int,
    crs: int,
    is_capsule: bool,
    is_precision: bool
) -> None:
    """
    入力引数の型チェック

    :param center: 円柱の中心の接続点。Pointを複数指定するリスト。
                   2つ目の接続点は1つ目の円柱の終点となるが、2つ目の円柱の始点にもなる。
    :type  center: list[SpatialId.common.object.point.Point]
    :param radius: 円柱の半径(単位:m)
    :type  radius: float
    :param h_zoom: 水平方向の精度レベル
    :type  h_zoom: int
    :param v_zoom: 垂直方向の精度レベル
    :type  v_zoom: int
    :param crs: 座標参照系
    :type  crs: int
    :param is_capsule: 始点、終点が球状であるかを示す。True: カプセル / False: 円柱
    :type  is_capsule: bool
    :param is_precision: 衝突判定実施オプション。True: 衝突判定実施 / False: 衝突判定スキップ
    :type  is_precision: bool

    :raise SpatialIdError: 入力チェック結果が不正の場合のエラー
    """
    error_flag = False

    if type(center) is not list:
        error_flag = True

    else:
        for point in center:
            if type(point) is not SpatialPoint:
                error_flag = True
                break

    if type(radius) not in [int, float]:
        error_flag = True

    if type(h_zoom) is not int:
        error_flag = True

    if type(v_zoom) is not int:
        error_flag = True

    if type(crs) is not int:
        error_flag = True

    if type(is_capsule) is not bool:
        error_flag = True

    if type(is_precision) is not bool:
        error_flag = True

    if error_flag is True:
        raise SpatialIdError("INPUT_VALUE_ERROR")

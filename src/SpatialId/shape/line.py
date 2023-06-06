# Copyright © 2022 Digital Agency. All rights reserved.
"""線分を対象に空間IDを取得するための名前空間
"""
import math
import numpy as np

from logging import getLogger
from skspatial.objects import Point, Line, Vector, Plane

from SpatialId import LOGGER_NAME
from SpatialId.common.object.enum import Point_Option
from SpatialId.common.object.point import Point as SpatialPoint
from SpatialId.common.object.point import Projected_Point
from SpatialId.shape.point import (
        get_spatial_ids_on_points,
        get_point_on_spatial_id,
        convert_projected_point_list_to_point_list,
        convert_point_list_to_projected_point_list)

# CRSのデフォルト値のEPSGコード
SPATIAL_ID_CRS = 3857

# DEBUGログ用のロガーインスタンス
spatial_logger = getLogger(LOGGER_NAME).getChild(__name__)


def get_spatial_ids_on_line(
        start: Projected_Point,
        end: Projected_Point,
        h_zoom: int,
        v_zoom: int,
        crs: int
) -> list[str]:
    """指定範囲の空間ID変換(線分)を取得する。

    :param start: 始点
    :type start: SpatialId.common.object.point.Point
    :param end: 終点
    :type end: SpatialId.common.object.point.Point

    :param h_zoom: 垂直方向の精度レベル
    :type h_zoom: int
    :param v_zoom: 水平方向の精度レベル
    :type v_zoom: int
    :param crs: 座標参照系
    :type crs: int

    :returns: 空間ID集合
    :rtype: list[string]

    :raise SpatialIdError: 入力チェックエラー
    :raise SpatialIdError: 入力されたEPSGコードが存在しない場合、エラー
    :raise SpatialIdError: 入力されたEPSGコードが地理座標系ではない場合、エラー
    """
    spatial_logger.debug("[START] 空間ID変換(線分)")
    spatial_logger.debug(
        "始点(地理座標系)=(%s, %s, %s)",
        start.lon, start.lat, start.alt)
    spatial_logger.debug(
        "終点(地理座標系)=(%s, %s, %s)",
        end.lon, end.lat, end.alt)

    # 始点・終点の空間IDを格納
    start_end_spatial_id_list = get_spatial_ids_on_points(
            [start, end], h_zoom, v_zoom)
    spatial_logger.debug("始点の空間ID=%s", start_end_spatial_id_list[0])
    spatial_logger.debug("終点の空間ID=%s", start_end_spatial_id_list[1])

    # 戻り値格納用 - 始点・終点の空間IDを格納
    spatial_ids = set(start_end_spatial_id_list)

    # 始点・終点が同じ座標かチェック
    if start == end:
        spatial_logger.debug("始点終点が同一")
        spatial_logger.debug("線分範囲の空間ID=%s", spatial_ids)
        spatial_logger.debug("[END] 空間ID変換(線分)")

        # 同じ座標の場合、 始点・終点の空間IDを返却
        return list(spatial_ids)

    # 【直交座標空間】交点及び交点間の中点(Point)リスト初期化
    cross_points_list = []

    # 【直交座標空間】取得対象外の座標を格納する集合
    exclude_point_set = set()

    # 空間ID利用の投影座標系を格納
    spatial_logger.debug(
        "始点(投影座標系)=(%s, %s, %s)",
        start.x,
        start.y,
        start.alt)
    spatial_logger.debug(
        "終点(投影座標系)=(%s, %s, %s)",
        end.x,
        end.y,
        end.alt)

    # 【直交座標空間】始点終点のx, y, z座標
    start_x, start_y, start_z = (
        start.x,
        start.y,
        start.alt
    )

    end_x, end_y, end_z = (
        end.x,
        end.y,
        end.alt
    )

    # 【直交座標空間】始点終点のPoint
    start_orth_point = Point((start_x, start_y, start_z))
    end_orth_point = Point((end_x, end_y, end_z))

    # 始点の空間IDから始点ボクセルIDを取得
    start_x_id, start_y_id, start_z_id = \
        get_voxel_id_to_spatial_id(start_end_spatial_id_list[0])

    # 終点の空間IDから終点ボクセルID取得
    end_x_id, end_y_id, end_z_id = \
        get_voxel_id_to_spatial_id(start_end_spatial_id_list[1])

    spatial_logger.debug(
        "始点(空間ID成分)=(%s, %s, %s)",
        start_x_id,
        start_y_id,
        start_z_id)
    spatial_logger.debug(
        "終点(空間ID成分)=(%s, %s, %s)",
        end_x_id,
        end_y_id,
        end_z_id)

    # 【直交座標空間】X軸ボクセル境界との交点取得開始
    # 【直交座標空間】X軸ボクセル境界面を取得
    x_voxel_plane_list = generate_x_voxel_plane(
            start_x_id,
            end_x_id,
            h_zoom,
            v_zoom
    )

    # 【直交座標空間】X軸ボクセル境界面との交点を取得
    cross_points_list += _get_plane_cross_points(
            x_voxel_plane_list,
            start_orth_point,
            end_orth_point,
            exclude_point_set
    )

    # 【直交座標空間】Y軸ボクセル境界との交点取得開始
    # 【直交座標空間】Y軸ボクセル境界面を取得
    y_voxel_plane_list = generate_y_voxel_plane(
            start_y_id,
            end_y_id,
            h_zoom,
            v_zoom
    )

    # 【直交座標空間】Y軸ボクセル境界面との交点を取得
    cross_points_list += _get_plane_cross_points(
            y_voxel_plane_list,
            start_orth_point,
            end_orth_point,
            exclude_point_set
    )

    # 【直交座標空間】Z軸ボクセル境界との交点取得開始
    # 【直交座標空間】Z軸ボクセル境界面を取得
    z_voxel_plane_list = generate_z_voxel_plane(
            start_z_id,
            end_z_id,
            h_zoom,
            v_zoom
    )

    # 【直交座標空間】Z軸ボクセル境界面との交点を取得
    cross_points_list += _get_plane_cross_points(
            z_voxel_plane_list,
            start_orth_point,
            end_orth_point,
            exclude_point_set
    )

    # 【直交座標空間】始点を含む、交点間の中点を取得する
    cross_points_list += _get_middle_points(
        start_orth_point, end_orth_point,
        cross_points_list, exclude_point_set
    )

    # 【直交座標空間】直交座標空間のデータクラスオブジェクトのリスト作成
    projected_cross_points_list = []

    for cross_points in cross_points_list:
        projected_cross_points_list.append(Projected_Point(
                cross_points[0], cross_points[1], cross_points[2]
                )
        )

    # 【直交座標空間】空間ID利用の座標系(EPSG:3857)座標系をWGS84の地理座標に変換したPointオブジェクト取得
    cross_spatial_points = \
        convert_projected_point_list_to_point_list(
                projected_cross_points_list,
                SPATIAL_ID_CRS)

    # 【緯度経度空間】境界面交点・交点間の中点座標を空間IDに変換
    cross_spatial_ids = get_spatial_ids_on_points(
            cross_spatial_points,
            h_zoom,
            v_zoom
    )
    spatial_logger.debug("交点リスト(直交座標)=%s", projected_cross_points_list)
    spatial_logger.debug("交点リスト(地理座標)=%s", cross_spatial_points)
    spatial_logger.debug("交点リスト(空間ID)=%s", cross_spatial_ids)

    spatial_ids |= set(cross_spatial_ids)

    spatial_logger.debug("線分範囲の空間ID=%s", spatial_ids)

    spatial_logger.debug("[END] 空間ID変換(線分)")

    return list(spatial_ids)


def detect_collision(plane: Plane, line: Line) -> Point:
    """
    :param plane: 衝突判定面
    :type plane: Plane
    :param line: 衝突判定線分
    :type line: Line

    :returns: 衝突座標(衝突箇所がない場合はNone)
    :rtype: Point
    """
    # 衝突位置
    collision_point = None

    # 面上の一点から辺の始点へのベクトル
    c0 = Vector.from_points(plane.point, line.point)

    # 進行方向と法線の内積
    direct_dot = np.dot(plane.normal, line.direction)
    # 進行方向と法線の内積が0
    # (進行方向が面に平行)の場合は衝突位置をNoneとして返却
    if math.fabs(direct_dot) < 1e-10:
        spatial_logger.debug("線分と衝突判定面が平行")
        return None

    # 衝突時間算出
    collision_time = -np.dot(plane.normal, c0) / direct_dot
    # 衝突時間が0～1の間にあれば衝突位置を算出
    if 0 <= collision_time and collision_time <= 1:
        collision_point = line.to_point(collision_time)

    return collision_point


def create_sequence_list(start: int, end: int) -> list[int]:
    """指定範囲の連番リストを作成

    開始値から終了値までの範囲の連番リストを返却する。
    - プラス方向に増加している場合は、開始値から終了値までの範囲で増分1の連番となる。
    - マイナス方向に増加している場合は、終了値から開始値までの範囲で増分1の連番となる。
    - 開始値と終了値が同一の場合は、開始値を格納した配列を返却する。

    :param start: 開始値
    :type start: int
    :param start: 終了値
    :type start: int

    :returns: 指定範囲の連番リスト
    :rtype: list[int]
    """
    num = end - start
    if num < 0:
        return range(end, start + 1, 1)
    elif num > 0:
        return range(start, end + 1, 1)
    else:
        return [start]


def generate_x_voxel_plane(
        start_x_id: int,
        end_x_id: int,
        h_zoom: int,
        v_zoom: int
) -> Plane:
    """X軸ボクセル境界を取得

    | 空間IDのX軸ボクセル境界面を返却する。

    :param start_x_id: 始点ボクセルのX成分ID
    :type start_x_id: int
    :param end_x_id: 終点ボクセルのX成分ID
    :type end_x_id: int
    :param h_zoom: 水平方向精度
    :type h_zoom: int
    :param v_zoom: 垂直方向精度
    :type v_zoom: int

    :returns: X軸ボクセル境界面
    :rtype: Plane

    :raise SpatialIdError: pyprojを用いた変換オブジェクトの生成に失敗
    :raise SpatialIdError: pyprojを用いた変換時にInfまたはNaNが発生した場合にエラー
    """
    spatial_logger.debug(
        "X境界面の取得(start_id=%s, end_id=%s)",
        start_x_id, end_x_id)

    # 【直交座標空間系】X境界線毎に処理
    for x_id in create_sequence_list(start_x_id, end_x_id)[1:]:
        # 【直交座標空間】X軸ボクセル境界面の単位法線ベクトル
        x_norm_vector = Vector([1, 0, 0])

        # 【直交座標空間】X軸空間ID取得
        spatial_id = get_spatial_id_on_axis_ids(x_id, 0, 0, h_zoom, v_zoom)

        # 【直交座標空間】境界面上のX座標取得
        x_point = get_x_voxel_plane_point_on_spatial_id(spatial_id)

        # 【直交座標空間】境界面上の座標を定義
        plane_orth_point = (x_point, 0, 0)

        spatial_logger.debug("X境界面上の空間ID=%s", spatial_id)
        spatial_logger.debug("X境界面上の座標=%s", plane_orth_point)

        # 【直交座標空間】X軸ボクセル境界面
        yield Plane(point=plane_orth_point, normal=x_norm_vector)


def generate_y_voxel_plane(
        start_y_id: int,
        end_y_id: int,
        h_zoom: int,
        v_zoom: int
) -> Plane:
    """Y軸ボクセル境界を取得

    | 空間IDのY軸ボクセル境界面を返却する。

    :param start_y_id: 始点ボクセルのY成分ID
    :type start_y_id: int
    :param end_y_id: 終点ボクセルのY成分ID
    :type end_y_id: int
    :param h_zoom: 水平方向精度
    :type h_zoom: int
    :param v_zoom: 垂直方向精度
    :type v_zoom: int

    :returns: Y軸ボクセル境界面
    :rtype: Plane

    :raise SpatialIdError: pyprojを用いた変換オブジェクトの生成に失敗
    :raise SpatialIdError: pyprojを用いた変換時にInfまたはNaNが発生した場合にエラー
    """
    spatial_logger.debug(
        "Y境界面の取得(start_id=%s, end_id=%s)",
        start_y_id, end_y_id)

    # 【直交座標空間系】Y境界線毎に処理
    for y_id in create_sequence_list(start_y_id, end_y_id)[1:]:
        # 【直交座標空間】Y軸ボクセル境界面の単位法線ベクトル
        y_norm_vector = Vector([0, 1, 0])

        # 【直交座標空間】Y軸空間ID取得
        spatial_id = get_spatial_id_on_axis_ids(0, y_id, 0, h_zoom, v_zoom)

        # 【直交座標空間】境界面上のY座標取得
        y_point = get_y_voxel_plane_point_on_spatial_id(spatial_id)

        # 【直交座標空間】境界面上のY座標取得
        plane_orth_point = (0, y_point, 0)

        spatial_logger.debug("Y境界面上の空間ID=%s", spatial_id)
        spatial_logger.debug("Y境界面上の座標=%s", plane_orth_point)

        # 【直交座標空間】Y軸ボクセル境界面
        yield Plane(point=plane_orth_point, normal=y_norm_vector)


def generate_z_voxel_plane(
        start_z_id: int,
        end_z_id: int,
        h_zoom: int,
        v_zoom: int
) -> Plane:
    """Z軸ボクセル境界を取得

    | 空間IDのZ軸ボクセル境界面を返却する。

    :param start_z_id: 始点ボクセルのZ成分ID
    :type start_z_id: int
    :param end_z_id: 終点ボクセルのZ成分ID
    :type end_z_id: int
    :param h_zoom: 水平方向精度
    :type h_zoom: int
    :param v_zoom: 垂直方向精度
    :type v_zoom: int

    :returns: Z軸ボクセル境界面
    :rtype: Plane

    :raise SpatialIdError: pyprojを用いた変換オブジェクトの生成に失敗
    :raise SpatialIdError: pyprojを用いた変換時にInfまたはNaNが発生した場合にエラー
    """
    spatial_logger.debug(
        "Z境界面の取得(start_id=%s, end_id=%s)",
        start_z_id, end_z_id)

    # 【直交座標空間】Z境界線毎に処理
    for z_id in create_sequence_list(start_z_id, end_z_id)[1:]:
        # 【直交座標空間】Z軸ボクセル境界面の単位法線ベクトル
        z_norm_vector = Vector([0, 0, 1])

        # 【直交座標空間】Z軸空間ID取得
        spatial_id = get_spatial_id_on_axis_ids(0, 0, z_id, h_zoom, v_zoom)

        # 【直交座標空間】境界面上のZ座標取得
        z_point = get_z_voxel_plane_point_on_spatial_id(spatial_id)

        # 【直交座標空間】境界面上のZ座標取得
        plane_orth_point = (0, 0, z_point)

        spatial_logger.debug("Z境界面上の空間ID=%s", spatial_id)
        spatial_logger.debug("Z境界面上の座標=%s", plane_orth_point)

        # 【直交座標空間】Z軸ボクセル境界面
        yield Plane(point=plane_orth_point, normal=z_norm_vector)


def get_x_voxel_plane_point_on_spatial_id(spatial_id: str) -> float:
    """空間IDからX方向ボクセル境界座標を取得

    :param spatial_id: 空間ID
    :type spatial_id: str

    :returns: X方向ボクセル境界座標
    :rtype: float
    """
    # 頂点座標を取得
    points = get_point_on_spatial_id(
        spatial_id, Point_Option.VERTEX, SPATIAL_ID_CRS
    )
    return min([point.x for point in points])


def get_y_voxel_plane_point_on_spatial_id(spatial_id: str) -> float:
    """空間IDからY方向ボクセル境界座標を取得

    :param spatial_id: 空間ID
    :type spatial_id: str

    :returns: Y方向ボクセル境界座標
    :rtype: float
    """
    # 頂点座標を取得
    points = get_point_on_spatial_id(
        spatial_id, Point_Option.VERTEX, SPATIAL_ID_CRS
    )
    return max([point.y for point in points])


def get_z_voxel_plane_point_on_spatial_id(spatial_id: str) -> float:
    """空間IDからZ方向ボクセル境界座標を取得

    :param spatial_id: 空間ID
    :type spatial_id: str

    :returns: Z方向ボクセル境界座標
    :rtype: float
    """
    # 頂点座標を取得
    points = get_point_on_spatial_id(
        spatial_id, Point_Option.VERTEX, SPATIAL_ID_CRS
    )
    return min([point.alt for point in points])


def get_spatial_id_on_axis_ids(
    x_id: int, y_id: int, z_id: int, h_zoom: int, v_zoom: int
) -> str:
    """軸IDから空間ID取得

    :param x_id: X軸ボクセルID
    :type x_id: int
    :param y_id: Y軸ボクセルID
    :type y_id: int
    :param z_id: Z軸ボクセルID
    :type z_id: int
    :param h_zoom: 水平方向精度
    :type h_zoom: int
    :param v_zoom: 垂直方向精度
    :type v_zoom: int

    :return: 空間ID
    :rtype: str
    """
    return '{}/{}/{}/{}/{}'.format(h_zoom, x_id, y_id, v_zoom, z_id)


def get_voxel_id_to_spatial_id(spatial_id: str) -> tuple[int, int, int]:
    """空間IDからボクセル成分ID取得

    :param spatial_id: 取得するボクセルの空間ID
    :type spatial_id: str

    :return: (xインデックス, yインデックス, vインデックス)
    :rtype: tuple
    """
    spatial_id_split = spatial_id.split('/')
    return (
            int(spatial_id_split[1]),
            int(spatial_id_split[2]),
            int(spatial_id_split[4]))


def _get_plane_cross_points(
        plane_list: list,
        start_orth_point: Point,
        end_orth_point: Point,
        exclude_point_set: set
) -> list[Point]:
    """
    |  始点、終点を結んだ線分と、境界面との交点座標を取得し返却する。
    |  取得対象外の座標が指定されていた場合、その座標は戻り値から除外する。

    :param plane_list: 境界面を格納したリスト
    :type plane_list: list[Point]
    :param start_orth_point: 線分の始点
    :type start_orth_point: Point
    :param end_orth_point: 線分の終点
    :type end_orth_point: Point
    :param exclude_point_list: 取得対象外の座標を格納する集合
    :type exclude_point_list: set[str]

    :return: 線分と境界面の交点を格納したリスト
    :rtype: list[Point]
    """
    # 【直交座標空間】戻り値格納用
    plane_cross_point_list = []

    # 【直交座標空間】始点終点の線
    line = Line.from_points(start_orth_point, end_orth_point)

    # 【直交座標空間】ボクセル境界毎に処理
    for plane in plane_list:
        # 【直交座標空間】ボクセル境界と線の交点の座標を取得
        cross_point = detect_collision(plane, line)

        if cross_point is None:
            # 線分と境界面の交点が存在しない場合はスキップ
            spatial_logger.debug("線分と境界面との交点なし")
            continue

        spatial_logger.debug(
            "ボクセル境界と線の交点=%s",
            tuple(cross_point))

        # 【直交座標空間】交点を経度・緯度ボクセル境界で取得済みの交点として取得済みの場合はスキップ
        cross_point_str = _convert_point_to_str(cross_point)
        if cross_point_str in exclude_point_set:
            continue

        # 【直交座標空間】交点を経度ボクセル境界で取得済みの交点として登録
        exclude_point_set.add(cross_point_str)

        # 【直交座標空間】交点を交点座標リストに格納
        plane_cross_point_list.append(cross_point)

    return plane_cross_point_list


def _convert_point_to_str(point: Point) -> str:
    """点座標文字列変換

    :param point: 点座標
    :type point: Point

    :returns: 点座標文字列
    :rtype: string
    """
    return "{0}_{1}_{2}".format(point[0], point[1], point[2])


def _get_middle_points(
    start_point: Point, end_point: Point,
    cross_points_list: list[Point],
    exclude_point_set: set
) -> list[Point]:
    """中点座標の取得

    :param start_point: 始点
    :type start_point: Point
    :param end_point: 終点
    :type end_point: Point
    :param cross_points_list: 交点座標リスト
    :type cross_points_list: list[Point]
    :param exclude_point_list: 取得対象外の座標を格納する集合
    :type exclude_point_list: set[str]

    :returns: 交点間の中点座標リスト
    :rtype: list[Point]
    """

    # どの軸でソートするかを確認
    line = Line.from_points(start_point, end_point)
    if abs(line.direction[0]) > 0:
        sort_axis = 0
    elif abs(line.direction[1]) > 0:
        sort_axis = 1
    else:
        sort_axis = 2

    spatial_logger.debug(
        "交点座標をソートする軸=%s", sort_axis)

    # 始点、終点を交点との中点も合わせて取得
    start_point_str = _convert_point_to_str(start_point)
    if start_point_str not in exclude_point_set:
        cross_points_list.append(start_point)
    end_point_str = _convert_point_to_str(end_point)
    if end_point_str not in exclude_point_set:
        cross_points_list.append(end_point)

    # 交点座標をソート
    sort_cross_points = sorted(
        cross_points_list,
        key=lambda val: val[sort_axis]
    )

    # 交点の中点取得
    middle_cross_points = []
    for index, _ in enumerate(sort_cross_points[:-1]):
        middle_cross_point = \
            (sort_cross_points[index]
                + sort_cross_points[index + 1]) / 2
        middle_cross_points.append(middle_cross_point)
        spatial_logger.debug(
            "交点間の中点=%s",
            tuple(middle_cross_point))

    return middle_cross_points

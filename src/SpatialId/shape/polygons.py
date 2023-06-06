# Copyright © 2022 Digital Agency. All rights reserved.
"""
|  Requirements: Python 3.9+.

|  ポリゴンの空間ID取得モジュール
"""
import copy
import itertools

from collections import defaultdict, Counter
from logging import getLogger

import numpy as np
from skspatial.objects import Point, Points, Line, Plane, Vector, Triangle

from SpatialId import LOGGER_NAME
from SpatialId.common.exception import SpatialIdError
from SpatialId.common.object.point import Point as SpatialPoint
from SpatialId.common.object.point import Triangle as SpatialTriangle
from SpatialId.common.object.point import Projected_Triangle
from SpatialId.common.object.point import Projected_Point
from SpatialId.common.object.enum import Point_Option

from SpatialId.operated.shifting_spatial_id import get_shifting_spatial_id
from SpatialId.shape import get_CRS_OBJECT, get_transformer
from SpatialId.shape.point import (
        get_spatial_ids_on_points,
        convert_point_list_to_projected_point_list,
        convert_projected_point_list_to_point_list,
        get_point_on_spatial_id
        )
from SpatialId.shape.line import (
        get_spatial_ids_on_line,
        detect_collision,
        get_voxel_id_to_spatial_id,
        generate_x_voxel_plane,
        generate_y_voxel_plane,
        generate_z_voxel_plane
        )

from SpatialId.convert import in_to_internal, internal_to_out

# CRSのデフォルト値(WGS84)のEPSGコード
__CRS__ = 4326
# 直交座標系のEPSGコード
__ORTH_CRS__ = 3857

# 三角ポリゴン頂点のキー
__VERTEX_KEY__ = ["A", "B", "C"]
# 三角ポリゴンの辺のキー
__EDGE_KEY__ = ["a", "b", "c"]
# 浮動小数点誤差
__MINIMA__ = 1e-10
# 辺浮動小数点誤差
__EDGE_MINIMA__ = 1e-8

# DEBUGログ用のロガーインスタンス
spatial_logger = getLogger(LOGGER_NAME).getChild(__name__)

def get_spatial_ids_on_polygons(
        barrier_triangles: list[SpatialTriangle],
        space_triangles: list[SpatialTriangle],
        h_zoom: int,
        v_zoom: int,
        crs: int = __CRS__,
        needs_closed_checking: bool = True
) -> list[str]:
    """
    | 三角形ポリゴンの集合で表されるモデルが含まれる空間IDを取得する。
    | 三角形ポリゴンが接しない内部の空間IDも合わせて取得する。
    | 三角形ポリゴンに接しない内部のボクセルの内外判定は、Z方向に行う。

    | 三角形ポリゴンの集合で表されるモデルが閉塞していない場合でも、内外判定は行われるが、
    | Z軸上方向に三角形ポリゴンがない場合は外部として判定する。
    | 外部として判定された箇所の空間IDは返却される空間IDには含まれない。

    | 内部に空間IDとして取得しない空間がある場合は、
    | 除外する三角形ポリゴンの集合で表されるモデルとして定義する。

    | 三角形ポリゴンの集合で表されるモデルの空間IDを取得後、
    | 除外する三角形ポリゴンの集合で表されるモデルの空間IDを除外する。

    :param barrier_triangles: 空間IDを取得する三角形ポリゴンの集合で
                              表されるモデル。Triangleオブジェクトを組み合わせ、複数指定する。
    :type barrier_triangles: list[SpatialId.common.object.point.Triangle]
    :param space_triangles: 除外する三角形ポリゴンの集合で表されるモデル。
                            Triangleオブジェクトを組み合わせ、複数指定する。
    :type space_triangles: list[SpatialId.common.object.point.Triangle]
    :param h_zoom: 水平方向の精度レベル
    :type h_zoom: int
    :param v_zoom: 垂直方向の精度レベル
    :type v_zoom: int
    :param crs: 座標参照系
    :type crs: int
    :param needs_closed_checking: 閉塞チェックフラグ。Trueの場合はチェックを行う。
    :type needs_closed_checking: bool

    :return: 三角形ポリゴンの集合で表されるモデルが含まれる空間IDのリスト
    :rtype: list[string]

    :raise SpatialIdError: 入力チェック結果が不正の場合のエラー
    :raise SpatialIdError: 三角ポリゴンの3点が同一直線上に存在する場合
    :raise SpatialIdError: 三角形ポリゴンのモデルが閉塞していない場合
    :raise SpatialIdError: 入力されたEPSGコードが存在しない場合、エラー
    :raise SpatialIdError: 入力されたEPSGコードが地理座標系ではない場合、エラー

    :注意: 経度180度をまたがる辺を持つポリゴンは空間ID化できない。
    """
    try:
        spatial_logger.debug("ポリゴン処理開始")
        # 入力引数チェック
        _validate_args_type(
            barrier_triangles, space_triangles,
            h_zoom, v_zoom, crs, needs_closed_checking
        )

        # 空間ID取得対象の三角ポリゴンをチェック
        _valid_triangle_points(barrier_triangles)

        # 空間ID取得対象の三角ポリゴンをチェック
        _valid_triangle_points(space_triangles)

        # 三角ポリゴンが形成するモデルが閉塞しているかのチェック
        if needs_closed_checking:
            spatial_logger.debug(
                "三角ポリゴンが形成するモデルが閉塞しているかのチェック"
            )
            _valid_closed_polygons(barrier_triangles)
            _valid_closed_polygons(space_triangles)


        # 戻り値の空間IDを格納する変数
        valid_spatial_ids = set()

        if len(barrier_triangles) == 0:
            # 有効な範囲の三角ポリゴン指定なしの場合、空の配列を返す
            spatial_logger.debug(
                "有効な範囲の三角ポリゴン指定なし"
            )
            return list(valid_spatial_ids)
        # 有効な範囲の空間ID配列を取得
        valid_spatial_ids = _get_spatial_ids_on_polygons(
            barrier_triangles,
            h_zoom,
            v_zoom,
            crs
        )

        if len(space_triangles) > 0:
            spatial_logger.debug(
                "除外する三角形ポリゴンが指定あり"
            )
            # 除外する三角形ポリゴンが指定された場合、除外対象の空間IDを取得
            invalid_spatial_ids = _get_spatial_ids_on_polygons(
                space_triangles,
                h_zoom,
                v_zoom,
                crs
            )

            # 有効な範囲から、除外対象の空間IDを取り除く
            valid_spatial_ids = valid_spatial_ids - invalid_spatial_ids

        spatial_logger.debug("ポリゴン処理終了")

        # 空間IDのリストを返却
        return list(valid_spatial_ids)

    except SpatialIdError:
        # 内部処理で検出済みの例外を再送
        raise

    except Exception:
        # その他例外を投げる
        raise SpatialIdError('OTHER_ERROR')


def f_get_spatial_ids_on_polygons( barrier_triangles: list[SpatialTriangle],
        space_triangles: list[SpatialTriangle],
        zoom: int,
        crs: int = __CRS__,
        needs_closed_checking: bool = True) -> list[str]:
    """
    | 三角形ポリゴンの集合で表されるモデルが含まれる空間IDを取得する。
    | 三角形ポリゴンが接しない内部の空間IDも合わせて取得する。
    | 三角形ポリゴンに接しない内部のボクセルの内外判定は、Z方向に行う。

    | 三角形ポリゴンの集合で表されるモデルが閉塞していない場合でも、内外判定は行われるが、
    | Z軸上方向に三角形ポリゴンがない場合は外部として判定する。
    | 外部として判定された箇所の空間IDは返却される空間IDには含まれない。

    | 内部に空間IDとして取得しない空間がある場合は、
    | 除外する三角形ポリゴンの集合で表されるモデルとして定義する。

    | 三角形ポリゴンの集合で表されるモデルの空間IDを取得後、
    | 除外する三角形ポリゴンの集合で表されるモデルの空間IDを除外する。

    | f_"がついてないAPIとの違いは以下：
    | 入力の空間ID精度レベルを水平・垂直方向共に統一
    | 出力の空間IDの並びは仕様通りの記載。
    | z/f/x/y :z{ズームレベル}/f{高さの位置}/x{経度方向の位置}/y{緯度方向の位置}
    | 参照：https://github.com/unvt/zfxy-spec

    :param barrier_triangles: 空間IDを取得する三角形ポリゴンの集合で
                              表されるモデル。Triangleオブジェクトを組み合わせ、複数指定する。
    :type barrier_triangles: list[SpatialId.common.object.point.Triangle]
    :param space_triangles: 除外する三角形ポリゴンの集合で表されるモデル。
                            Triangleオブジェクトを組み合わせ、複数指定する。
    :type space_triangles: list[SpatialId.common.object.point.Triangle]
    :param zoom: 空間IDの精度レベル
    :type zoom: int
    :param crs: 座標参照系
    :type crs: int
    :param needs_closed_checking: 閉塞チェックフラグ。Trueの場合はチェックを行う。
    :type needs_closed_checking: bool

    :return: 三角形ポリゴンの集合で表されるモデルが含まれる空間IDのリスト
    :rtype: list[string]

    :raise SpatialIdError: 入力チェック結果が不正の場合のエラー
    :raise SpatialIdError: 三角ポリゴンの3点が同一直線上に存在する場合
    :raise SpatialIdError: 三角形ポリゴンのモデルが閉塞していない場合
    :raise SpatialIdError: 入力されたEPSGコードが存在しない場合、エラー
    :raise SpatialIdError: 入力されたEPSGコードが地理座標系ではない場合、エラー

    :注意: 経度180度をまたがる辺を持つポリゴンは空間ID化できない。
    """
    out = get_spatial_ids_on_polygons(barrier_triangles,space_triangles,zoom,zoom,crs,needs_closed_checking)
    return internal_to_out(out)


def _get_spatial_ids_on_polygons(
    triangles: list[SpatialTriangle],
    h_zoom: int,
    v_zoom: int,
    crs: int
) -> set[str]:
    """
    | 三角形ポリゴンの集合で表されるモデルが含まれる空間IDを取得する。

    :param triangles: 三角形ポリゴンの集合
    :type triangles: list[SpatialId.common.object.point.Triangle]
    :param h_zoom: 水平方向の精度レベル
    :type h_zoom: int
    :param v_zoom: 垂直方向の精度レベル
    :type v_zoom: int
    :param crs: 座標参照系
    :type crs: int

    :return: 空間ID集合
    :rtype: set[str]
    """
    spatial_ids = set()
    rectangular_triangles = list()

    for triangle in triangles:
        if not crs == __ORTH_CRS__ and get_CRS_OBJECT(crs).is_projected:
            # 入力のcrsが投影座標かつ3857以外の場合、3857に変換し、triangleを入れ替える
            p1_converted, p2_converted, p3_converted = get_transformer(
                crs, __ORTH_CRS__).transform(
                [triangle.p1.x, triangle.p1.y, triangle.p1.alt],
                [triangle.p2.x, triangle.p2.y, triangle.p2.alt],
                [triangle.p3.x, triangle.p3.y, triangle.p3.alt], errcheck=True)
            triangle = Projected_Triangle(
                    Projected_Point(
                        p1_converted[0], p1_converted[1], p1_converted[2]),
                    Projected_Point(
                        p2_converted[0], p2_converted[1], p2_converted[2]),
                    Projected_Point(
                        p3_converted[0], p3_converted[1], p3_converted[2])
            )
        # 三角ポリゴンの座標系を平面直角座標系に変換
        if get_CRS_OBJECT(crs).is_geographic:
            rectangular_crs_triangle = \
                _get_triangle_convert_rectangular_coordinate_crs(
                    triangle,
                    crs
                )
        else:
            rectangular_crs_triangle = Triangle(
                    Point([triangle.p1.x, triangle.p1.y, triangle.p1.alt]),
                    Point([triangle.p2.x, triangle.p2.y, triangle.p2.alt]),
                    Point([triangle.p3.x, triangle.p3.y, triangle.p3.alt])
                )

        rectangular_triangles.append(rectangular_crs_triangle)
        spatial_logger.debug(
            "三角ポリゴンの座標(地理座標): %s", triangle
        )
        spatial_logger.debug(
            "三角ポリゴンの座標(投影座標): %s", rectangular_crs_triangle
        )

        # 空間ID取得開始
        # 三角ポリゴンの各辺の空間IDを取得
        triangle_side_spatial_ids = _get_triangle_side_spatial_ids(
            triangle,
            rectangular_crs_triangle,
            h_zoom,
            v_zoom,
            crs
        )
        spatial_ids |= triangle_side_spatial_ids

        # 三角ポリゴンの面の空間ID取得
        triangle_plane_spatial_ids = _get_triangle_plane_spatial_ids(
            triangle,
            rectangular_crs_triangle,
            h_zoom,
            v_zoom,
            crs,
        )
        spatial_ids |= triangle_plane_spatial_ids

        spatial_logger.debug(
            "三角ポリゴンの各辺の空間ID: %s", triangle_side_spatial_ids
        )
        spatial_logger.debug(
            "三角ポリゴンの面の空間ID: %s", triangle_plane_spatial_ids
        )

    # 有効なボクセルを取得
    inner_spatial_ids = set()
    outer_spatial_ids = set()
    _get_inner_voxel(
        rectangular_triangles, spatial_ids,
        inner_spatial_ids, outer_spatial_ids
    )
    spatial_logger.debug(
        "有効なボクセルを取得前の空間ID: %s", spatial_ids
    )
    spatial_logger.debug(
        "有効なボクセルを取得した空間ID: %s", inner_spatial_ids
    )
    spatial_logger.debug(
        "無効なボクセルを取得した空間ID: %s", outer_spatial_ids
    )
    spatial_ids.update(inner_spatial_ids)

    return spatial_ids


def _valid_triangle_points(triangles: list[SpatialTriangle]):
    """
    |  引数で渡された三角ポリゴンの座標が面を形成するかチェックする。
    |  面を形成しない三角ポリゴンが含まれていた場合は例外を発生させる。


    :param triangles: 三角ポリゴンインスタンスが格納されたリスト
    :type triangles: list[SpatialId.common.object.point.Triangle]

    :raise: SpatialIdError: 三角ポリゴンの3点が同一直線上に存在する場合
    """
    # パラメータチェック
    for triangle in triangles:

        # 三角ポリゴンの座標取得
        p1 = triangle.p1
        p2 = triangle.p2
        p3 = triangle.p3

        try:
            # 三角ポリゴンオブジェクトの生成に成功するかを確認
            t_object = Triangle(
                point_a=Point([p1.lon, p1.lat, p1.alt]),
                point_b=Point([p2.lon, p2.lat, p2.alt]),
                point_c=Point([p3.lon, p3.lat, p3.alt])
            )

            # 桁数が多い場合はオブジェクト生成を通過するため、平行判定を行う
            vector_a_c = \
                Vector.from_points(t_object.point_a, t_object.point_b).unit()
            vector_a_b = \
                Vector.from_points(t_object.point_a, t_object.point_c).unit()

            if vector_a_b.cross(vector_a_c).norm() < __MINIMA__:
                # 3点が同一直線状にある場合、面を形成しないため例外発生
                raise SpatialIdError('POLYGON_POINT_COLLINEAR')

        except ValueError as e:

            # 3点が同一直線状にある場合、面を形成しないため例外発生
            raise SpatialIdError('POLYGON_POINT_COLLINEAR') from e


def _valid_closed_polygons(triangles: list[SpatialTriangle]):
    """
    |  引数で渡された三角ポリゴンが形成するモデルが閉塞しているかをチェックする。
    |  以下のいずれかを満たす場合は、閉塞していないものとする。
    |  - 頂点を共有する三角ポリゴンにおいて、共有する頂点以外の頂点を抽出したとき、同じ頂点が奇数個抽出された場合
    |  - 辺を共有するポリゴンが奇数個の場合

    :param triangles: 三角ポリゴンインスタンスが格納されたリスト
    :type triangles: list[SpatialId.common.object.point.Triangle]

    :raise SpatialIdError: 三角形ポリゴンのモデルが閉塞していない場合
    """

    # 三角ポリゴンが共有する頂点をキー、共有する頂点以外の頂点の出現回数を値
    point_to_polygons = defaultdict(lambda: Counter())
    # 三角ポリゴンが共有する辺をキー、辺を共有するポリゴンの数を値
    line_to_polygons = defaultdict(lambda: 0)

    for triangle in triangles:
        spatial_logger.debug("triangle=%s", triangle)
        # 三角ポリゴンの各座標を辞書のキーとできるようにtupleに変換
        points = []
        points.append(
            tuple((triangle.p1.lon, triangle.p1.lat, triangle.p1.alt))
        )
        points.append(
            tuple((triangle.p2.lon, triangle.p2.lat, triangle.p2.alt))
        )
        points.append(
            tuple((triangle.p3.lon, triangle.p3.lat, triangle.p3.alt))
        )

        # 順序を固定するためにソートする
        points.sort()

        # 三角ポリゴンが共有する頂点をキー、共有する頂点以外の頂点の出現回数を値
        point_to_polygons[points[0]] += Counter([points[1], points[2]])
        point_to_polygons[points[1]] += Counter([points[0], points[2]])
        point_to_polygons[points[2]] += Counter([points[0], points[1]])

        # 三角ポリゴンが共有する辺をキー、辺を共有するポリゴンの数を値
        for i, j in itertools.combinations(range(3), 2):
            line_points = (points[i], points[j])
            line_to_polygons[line_points] += 1

    # 頂点を共有する三角ポリゴンにおいて、共有する頂点以外の頂点を抽出したとき、同じ頂点が奇数個抽出された場合
    for counter in point_to_polygons.values():
        for _, apex_num in counter.items():
            spatial_logger.debug("apex_num=%s", apex_num)
            if apex_num % 2 == 1:
                raise SpatialIdError("POLYGON_NOT_CLOSED_MODEL")

    # 辺を共有するポリゴンが奇数個の場合
    for triangle_num in line_to_polygons.values():
        spatial_logger.debug("triangle_num=%s", triangle_num)
        if triangle_num % 2 == 1:
            raise SpatialIdError("POLYGON_NOT_CLOSED_MODEL")


def _get_triangle_convert_rectangular_coordinate_crs(
    triangle: SpatialTriangle,
    crs: int
) -> Triangle:
    """
    |  引数で渡された三角ポリゴンの座標系を、
    |  空間ID利用の座標系(WGS84)から座標計算で利用する平面直角座標系へ変換

    :param triangle: 座標系を変換する三角ポリゴン
    :type triangle: SpatialId.common.object.point.Triangle
    :param crs: 座標参照系
    :type  crs: int

    :return: 平面直角座標系に変換した三角ポリゴン
    :rtype: Triangle
    """
    # 【直交座標空間】三角ポリゴンの座標系を変換
    p1, p2, p3 = convert_point_list_to_projected_point_list(
        [triangle.p1, triangle.p2, triangle.p3],
        __ORTH_CRS__,
        crs
    )

    # 三角ポリゴンインスタンス作成
    return Triangle(
        Point([p1.x, p1.y, p1.alt]),
        Point([p2.x, p2.y, p2.alt]),
        Point([p3.x, p3.y, p3.alt]),
    )


def _get_point_object_on_triangle(
    triangle: SpatialTriangle,
    rectangular_crs_triangle: Triangle
) -> list[SpatialPoint]:
    """
    |  引数で入力された三角ポリゴンの頂点座標から、
    |  Pointインスタンスを作成し返却する。

    :param triangle: 三角ポリゴンのインスタンス
    :type triangle: SpatialId.common.object.point.Triangle
    :param rectangular_crs_triangle: 三角ポリゴンのインスタンス
    :type rectangular_crs_triangle: skspatial.objects.Triangle

    :return: 三角ポリゴンの頂点座標のPointインスタンス
    :rtype: list[SpatialId.common.object.point.Point]
    """
    # 三角ポリゴンの頂点座標群を格納
    point_a = triangle.p1
    point_b = triangle.p2
    point_c = triangle.p3

    apex_points = [
            Projected_Point(
                rectangular_crs_triangle.point_a[0],
                rectangular_crs_triangle.point_a[1],
                point_a.alt, point_a.lon, point_a.lat),
            Projected_Point(
                rectangular_crs_triangle.point_b[0],
                rectangular_crs_triangle.point_b[1],
                point_b.alt, point_b.lon, point_b.lat),
            Projected_Point(
                rectangular_crs_triangle.point_c[0],
                rectangular_crs_triangle.point_c[1],
                point_c.alt, point_c.lon, point_c.lat)
    ]

    return apex_points


def _get_triangle_side_spatial_ids(
        triangle: SpatialTriangle,
        rectangular_crs_triangle: Triangle,
        h_zoom: int,
        v_zoom: int,
        crs: int
) -> set[str]:
    """
    |  引数で入力された三角ポリゴンの辺の空間IDをリストで返却する。

    :param triangle: 三角ポリゴンのインスタンス
    :type triangle: SpatialId.common.object.point.Triangle
    :param rectangular_crs_triangle: 三角ポリゴンのインスタンス
    :type rectangular_crs_triangle: skspatial.objects.Triangle
    :param h_zoom: 水平方向の精度レベル
    :type h_zoom: int
    :param v_zoom: 垂直方向の精度レベル
    :type v_zoom: int
    :param crs: ユーザ指定の座標系
    :tyep crs: int

    :return: 頂点座標の空間IDの集合
    :rtype: set
    """
    # 戻り値格納用
    spatial_ids = set()

    # 三角ポリゴンの頂点座標のPointインスタンス取得
    apex_points = _get_point_object_on_triangle(
        triangle,rectangular_crs_triangle)

    # 三角ポリゴンの1-2点間の辺上の空間ID取得
    spatial_ids |= set(get_spatial_ids_on_line(
            apex_points[0],
            apex_points[1],
            h_zoom,
            v_zoom,
            crs))

    # 三角ポリゴンの2-3点間の辺上の空間ID取得
    spatial_ids |= set(get_spatial_ids_on_line(
            apex_points[1],
            apex_points[2],
            h_zoom,
            v_zoom,
            crs))

    # 三角ポリゴンの3-1点間の辺上の空間ID取得
    spatial_ids |= set(get_spatial_ids_on_line(
            apex_points[2],
            apex_points[0],
            h_zoom,
            v_zoom,
            crs))

    return spatial_ids


def _get_triangle_plane_spatial_ids(
        triangle: SpatialTriangle,
        rectangular_triangle: Triangle,
        h_zoom: int,
        v_zoom: int,
        crs: int
) -> set[str]:
    """
    |  ボクセル境界面と三角ポリゴンの面との交点間の空間IDを取得し返却する。

    :param triangle: 経度緯度空間の三角ポリゴンのインスタンス
    :type triangle: SpatialId.common.object.point.Triangle
    :param triangle: 平面直角座標の三角ポリゴンのインスタンス
    :type rectangular_triangle: Triangle
    :param h_zoom: 水平方向の精度レベル
    :type h_zoom: int
    :param v_zoom: 垂直方向の精度レベル
    :type v_zoom: int
    :param crs: ユーザ指定の座標系
    :tyep crs: int

    :return: 境界面と線の交差点の空間IDを格納した集合
    :rtype: set
    """
    # 戻り値格納用
    spatial_ids = set()

    # 【経度緯度空間】三角ポリゴンの頂点座標を配列に格納
    triangle_points = [
            SpatialPoint(triangle.p1.lon, triangle.p1.lat, triangle.p1.alt),
            SpatialPoint(triangle.p2.lon, triangle.p2.lat, triangle.p2.alt),
            SpatialPoint(triangle.p3.lon, triangle.p3.lat, triangle.p3.alt)
    ]
    triangle_spatial_ids = get_spatial_ids_on_points(
        triangle_points,
        h_zoom,
        v_zoom,
    )

    # 空間IDの成分取得
    triangle_point_axis_ids = [
        get_voxel_id_to_spatial_id(spatial_id)
        for spatial_id
        in triangle_spatial_ids
    ]

    # X境界面上の空間ID取得
    x_max_id = max(triangle_point_axis_ids, key=lambda x: x[0])[0]
    x_min_id = min(triangle_point_axis_ids, key=lambda x: x[0])[0]
    # X軸ボクセル境界面を取得
    x_voxel_plane_list = generate_x_voxel_plane(
            x_min_id,
            x_max_id,
            h_zoom,
            v_zoom
    )
    # Xボクセル境界面との交点取得
    for x_plane in x_voxel_plane_list:
        # 境界面と三角ポリゴン面の交点取得
        spatial_ids |= _get_cross_points_of_voxel_plane(
                    x_plane,
                    rectangular_triangle,
                    h_zoom,
                    v_zoom,
                    crs)

    # Y境界面上の空間ID取得
    y_max_id = max(triangle_point_axis_ids, key=lambda x: x[1])[1]
    y_min_id = min(triangle_point_axis_ids, key=lambda x: x[1])[1]
    # Y軸ボクセル境界面を取得
    y_voxel_plane_list = generate_y_voxel_plane(
            y_min_id,
            y_max_id,
            h_zoom,
            v_zoom
    )
    # Yボクセル境界面との交点取得
    for y_plane in y_voxel_plane_list:
        # 境界面と三角ポリゴン面の交点取得
        spatial_ids |= _get_cross_points_of_voxel_plane(
                    y_plane,
                    rectangular_triangle,
                    h_zoom,
                    v_zoom,
                    crs)

    # Z境界面上の空間ID取得
    z_max_id = max(triangle_point_axis_ids, key=lambda x: x[2])[2]
    z_min_id = min(triangle_point_axis_ids, key=lambda x: x[2])[2]
    # Z軸ボクセル境界面を取得
    z_voxel_plane_list = generate_z_voxel_plane(
            z_min_id,
            z_max_id,
            h_zoom,
            v_zoom
    )
    # Zボクセル境界面との交点取得
    for z_plane in z_voxel_plane_list:
        # 境界面と三角ポリゴン面の交点取得
        spatial_ids |= _get_cross_points_of_voxel_plane(
                    z_plane,
                    rectangular_triangle,
                    h_zoom,
                    v_zoom,
                    crs)

    return spatial_ids


def _get_cross_points_of_voxel_plane(
        plane: Plane,
        triangle: Triangle,
        h_zoom: int,
        v_zoom: int,
        crs: int
) -> set[str]:
    """
    |  引数で入力された三角ポリゴンの頂点から線分(平面直角座標空間)を取得し、
    |  線分と衝突判定面が交差する空間IDの集合を返却する。
    |  衝突箇所がない場合、空の集合が返却される。

    :param plane: 衝突判定面
    :type plane: Plane
    :param triangle: 三角ポリゴン
    :type triangle: Triangle

    :return: 境界面と線分が交差する空間IDの集合
    :rtype: set[str]
    """
    # 戻り値格納用
    spatial_ids = set()

    # 同じ境界面でとれた2点を始点と終点に設定
    start_point = []
    end_point = []

    # 【平面直角座標空間】三角ポリゴンの線を取得
    line_ab = Line.from_points(
            triangle.point_a,
            triangle.point_b)

    line_bc = Line.from_points(
            triangle.point_b,
            triangle.point_c)

    line_ca = Line.from_points(
            triangle.point_c,
            triangle.point_a)

    # 【平面直角座標空間】経度ボクセル境界と線の交点の座標を取得
    cross_point_ab = detect_collision(plane, line_ab)
    cross_point_bc = detect_collision(plane, line_bc)
    cross_point_ca = detect_collision(plane, line_ca)

    # 【平面直角座標空間】始点終点格納
    if cross_point_ab is not None:
        # 同じ境界面でとれた始点
        start_point = cross_point_ab

    if cross_point_bc is not None:
        if len(start_point) == 0:
            start_point = cross_point_bc
        elif not (start_point == cross_point_bc).all():
            # 同じ境界面でとれた終点
            end_point = cross_point_bc

    if cross_point_ca is not None:
        if end_point == [] and (not (start_point == cross_point_ca).all()):
            end_point = cross_point_ca

    # 始点と終点が取得できた場合
    if len(end_point) != 0:

        # 【経度緯度空間】経度緯度の座標系に変換
        start_spatial_point, end_spatial_point = \
            convert_projected_point_list_to_point_list([
                Projected_Point(
                    start_point[0], start_point[1], start_point[2]
                ),
                Projected_Point(
                    end_point[0], end_point[1], end_point[2]
                )],
                __ORTH_CRS__
            )

        # 【経度緯度空間】境界面と三角ポリゴンが交差する点の空間ID格納
        spatial_ids |= set(get_spatial_ids_on_line(
                    Projected_Point(start_point[0], start_point[1], start_spatial_point.alt, start_spatial_point.lon, start_spatial_point.lat),
                    Projected_Point(end_point[0], end_point[1], end_spatial_point.alt, end_spatial_point.lon, end_spatial_point.lat),
                    h_zoom,
                    v_zoom,
                    crs))

    return spatial_ids


def _make_right_rotation_line(
    start_line: list[Point],
    lines: list[Line]
) -> tuple[list[Point], list[Line]]:
    """
    |  線を開始線から右周りで繋がるように連結する
    | 再帰を含めた最大呼び出し回数はlinesの要素数分
    | 最大呼び出し回数になる場合は開始線から全ての線リストが右回りに連結できる場合

    :param start_line: 開始線
    :type  start_line: list[Point]
    :param lines: 線のリスト
    :type  lines: list[Line]

    :returns: 連結した点配列
    :rtype: list[Point]]
    :returns: 連結した点配列を除いた余った線の配列
    :rtype: list[Line]
    """
    # 開始線の終点から開始（右回り）
    end_point = start_line[-1]
    # 右回りの次の線があるかの判定
    has_next = False

    # 線のリストの要素ごとに処理
    for index, line in enumerate(lines):

        # 線の始点・終点毎に確認
        for t in [0, 1]:
            # 線の端点が開始線の終点と一致する場合
            if end_point.distance_point(line.to_point(t)) < __MINIMA__:
                # 線のもう片方の端点を開始線の終点の後ろに追加
                next_line = [line.to_point(1-t)]
                start_line += next_line
                # 判定した線を線のリストから除外
                lines.pop(index)
                has_next = True
                break

        # 線の端点が開始線の終点と一致しない場合は処理継続
        else:
            continue

        # 線の端点が開始線の終点と一致する場合は線のリストの要素の確認を終了
        break

    # 次の線がある場合
    if has_next:
        # 開始線から右回りで線が繋がりきるまで再帰的に処理を行う
        return _make_right_rotation_line(start_line, lines)

    # 次の線が無い場合
    else:
        # 開始線から右回りで線が繋がりきったとして結果を返却
        return start_line, lines


def _make_left_rotation_line(
    start_line: list[Point],
    lines: list[list[Point]]
) -> tuple[list[Point], list[list[Point]]]:
    """
    |  線を開始線から左周りで繋がるように連結する
    | 再帰を含めた最大呼び出し回数はlinesの要素数分
    | 最大呼び出し回数になる場合は開始線から全ての線リストが左回りに連結できる場合

    :param start_line: 開始線
    :type  start_line: list[Point]
    :param lines: 線のリスト
    :type  lines: list[list[Point]]

    :returns: 連結した点配列
    :rtype: list[Point]
    :returns: 連結した点配列を除いた余った線の配列
    :rtype: list[list[Point]]
    """

    # 開始線の始点から開始（左回り）
    end_point = start_line[0]
    # 左回りの次の線があるかの判定
    has_next = False

    # 線のリストの要素ごとに処理
    for index, line in enumerate(lines):

        # 線の始点・終点毎に確認
        for t in [0, 1]:
            # 線の端点が開始線の終点と一致する場合
            if end_point.distance_point(line.to_point(t)) < __MINIMA__:
                # 線のもう片方の端点を開始線の終点の後ろに追加
                next_line = [line.to_point(1-t)]
                start_line = next_line + start_line
                # 判定した線を線のリストから除外
                lines.pop(index)
                has_next = True
                break

        # 線の端点が開始線の終点と一致しない場合は処理継続
        else:
            continue

        # 線の端点が開始線の終点と一致する場合は線のリストの要素の確認を終了
        break

    # 次の線がある場合
    if has_next:
        # 開始線から左回りで線が繋がりきるまで再帰的に処理を行う
        return _make_left_rotation_line(start_line, lines)

    # 次の線が無い場合
    else:
        # 開始線から左回りで線が繋がりきったとして結果を返却
        return start_line, lines


def _make_line_group(lines: list[Line]) -> list[list[Point]]:
    """
    |  線を点が繋がるように連結してグループ化する
    | 再帰を含めた最大呼び出し回数はlinesの要素数分
    | 最大呼び出し回数になる場合は全ての線が別グループとなる場合

    :param lines: 線のリスト
    :type  lines: list[Line]

    :returns: 連結してグループ化した点配列のリスト
    :rtype: list[list[Point]]
    """

    # 線のリストが空の場合は空リストを返却
    if len(lines) == 0:
        return list()

    line_group = list()
    # 線のリストの先頭を開始線、残りを繋げる候補の線として右回りに連結
    start_line = [lines[0].point, lines[0].to_point()]
    start_line, lines = _make_right_rotation_line(start_line, lines[1:])
    # 右回りに連結済みの線を開始線、残りを繋げる候補の線として左回りに連結
    start_line, lines = _make_left_rotation_line(start_line, lines)
    # 左右に連結してグループ化した点配列を返却値に追加
    line_group.append(start_line)

    # 繋げる候補の線が残っている場合
    if len(lines) != 0:
        # 繋げる候補の線で再帰的に処理を呼び出し
        # 繋げる候補の線は1グループ分の線が抜かれた線のリスト
        line_group += _make_line_group(lines)

    # 繋げる候補の線が残っていない場合は返却値を返却
    return line_group


def _is_inside(vertex_point_list: list[Point], target_point: Point) -> bool:
    """
    |  判定対象点が多角形の内部であるかの判定
    | 判定対象点が図形の頂点・辺上に存在する場合はこのメソッドの呼び出し元で考慮しているため、
    | 上記パターンはこのメソッドでは想定しない

    :param vertex_point_list: 多角形の頂点（2次元）
    :type  vertex_point_list: list[Point]
    :param target_point: 判定座標（2次元）
    :type  target_point: Point

    :returns: 判定対象点が多角形の内部であるかの判定結果
    :rtype: bool
    """

    # 射影の頂点が全て同一線上にある場合は外部として判定する
    if Points(vertex_point_list).are_collinear():
        return False

    # 角度
    theta = 0
    for i, vertex_point in enumerate(vertex_point_list):

        # 確認対象の点から多角形の点へのベクトル1
        v1 = Vector.from_points(target_point, vertex_point)

        # 確認対象の点から多角形の点へのベクトル2
        if (i+1 == len(vertex_point_list)):
            v2 = Vector.from_points(target_point, vertex_point_list[0])
        else:
            v2 = Vector.from_points(target_point, vertex_point_list[i+1])

        # ベクトル1,2の角度を算出（ラジアン単位）
        angle_radian = v1.angle_signed(v2)
        # 算出した角度を加算
        theta += angle_radian

    # 多角形の各頂点を巡回した最終的な角度が1周以上していれば内部として判定する
    return not abs(np.degrees(theta)) < __MINIMA__


def _grouping_triangle(
    cross_point_set: set[str],
    union_triangle_indexes: set[int],
    edge_triangle_group: dict[str, set[int]],
    vertex_triangle_group: dict[str, set[int]]
):
    """
    |  三角ポリゴンを共有するグループをまとめる

    :param cross_point_set: 衝突点の集合
    :type  cross_point_set: set[str]
    :param union_triangle_indexes: 共有グループとなる三角ポリゴン
    :type  union_triangle_indexes: set[int]
    :param edge_triangle_group: 辺の三角ポリゴン辞書
    :type  edge_triangle_group: dict[str, set[int]]
    :param vertex_triangle_group: 頂点の三角ポリゴン辞書
    :type  vertex_triangle_group: dict[str, set[int]]
    """
    # 共有する三角ポリゴンを持つ衝突点があるかの有無
    has_some_cross_point = False

    # 辺の衝突点に対する三角ポリゴングループに関して、共有する三角ポリゴンがあるかチェック
    for check_point_str, edege_triangle_indexes in \
            edge_triangle_group.items():
        # 既に共有していると判定済みの衝突点は除く
        if check_point_str in cross_point_set:
            continue

        # 共有する三角ポリゴンがある場合
        if union_triangle_indexes & edege_triangle_indexes:
            # 共有三角ポリゴングループを更新
            union_triangle_indexes |= edege_triangle_indexes
            # 共有判定済みの衝突点に衝突点を追加
            cross_point_set.add(check_point_str)
            # 共有する三角ポリゴンを持つ衝突点があると判定
            has_some_cross_point = True

    # 頂点の衝突点に対する三角ポリゴングループに関して、共有する三角ポリゴンがあるかチェック
    for check_point_str, vertex_triangle_indexes in \
            vertex_triangle_group.items():
        # 既に共有していると判定済みの衝突点は除く
        if check_point_str in cross_point_set:
            continue

        # 共有する三角ポリゴンがある場合
        if union_triangle_indexes & vertex_triangle_indexes:
            # 共有三角ポリゴングループを更新
            union_triangle_indexes |= vertex_triangle_indexes
            # 共有判定済みの衝突点に衝突点を追加
            cross_point_set.add(check_point_str)
            # 共有する三角ポリゴンを持つ衝突点があると判定
            has_some_cross_point = True

    # 共有する三角ポリゴンを持つ衝突点無くなるまで再帰的に処理を実行
    if has_some_cross_point:
        _grouping_triangle(
            cross_point_set, union_triangle_indexes,
            edge_triangle_group, vertex_triangle_group
        )


def _change_point_str(point: Point) -> str:
    """
    |  引数で入力された座標を文字列に変換する

    :param point: 座標
    :type  point: Point

    :returns: 文字列変換した座標
    :rtype: str
    """
    return "{:.10f}_{:.10f}_{:.10f}".format(point[0], point[1], point[2])


def _change_line_str(line: Line) -> str:
    """
    |  引数で入力された線を文字列に変換する

    :param line: 線
    :type  line: Line

    :returns: 文字列変換した線
    :rtype: str
    """
    line_strs = [
        _change_point_str(line.point),
        _change_point_str(line.to_point())
    ]
    return "{}_{}".format(max(line_strs), min(line_strs))


def _count_line_side(target_line: Line, target_point: Point) -> (int, int):
    """
    |  線への左右判定

    :param target_line: 左右判定の対象の辺（2次元）
    :type  target_line: Line
    :param target_vertex: 左右判定の対象の点（2次元）
    :type  target_vertex: Point

    :returns: 左判定回数
    :rtype: int
    :returns: 右判定回数
    :rtype: int
    """
    # 左判定回数
    left_count = 0
    # 右判定回数
    right_count = 0
    # 直線Lに対して判定対象の頂点が左右どちらにあるかカウントする
    side = target_line.side_point(target_point)
    # -1の場合、左判定
    if side == -1:
        left_count = 1
    # 1の場合、右判定
    elif side == 1:
        right_count = 1

    return (left_count, right_count)

def _count_edge_side(
    xy_cross_edge: Line,
    group_triangle_indexes: set[int],
    rectangular_triangles: list[Triangle]
) -> (int, int):
    """
    |  辺に対する左右判定

    :param xy_cross_edge: 衝突点が通過する辺（2次元）
    :type  xy_cross_edge: Line
    :param group_triangle_indexes: 同グループの三角ポリゴンのrectangular_triangles内のインデックス
    :type  group_triangle_indexes: set[int]
    :param rectangular_triangles: 全三角ポリゴン
    :type  rectangular_triangles: list[Triangle]

    :returns: 左判定回数
    :rtype: int
    :returns: 右判定回数
    :rtype: int
    """
    # 左判定回数
    left_count = 0
    # 右判定回数
    right_count = 0

    # 辺に対してグループ化された三角ポリゴンのインデックス
    for group_triangle_index in group_triangle_indexes:
        cross_triangle = rectangular_triangles[group_triangle_index]
        # 同グループの三角ポリゴンの頂点を取得
        for vertex_key in __VERTEX_KEY__:
            vertex = cross_triangle.point(vertex_key)
            # 頂点をXY平面に射影
            xy_vertex = [vertex[0], vertex[1]]

            # 辺に対して判定対象の頂点が左右どちらにあるかカウントする
            left, right = _count_line_side(xy_cross_edge, xy_vertex)
            left_count += left
            right_count += right

    return left_count, right_count


def _search_judge_vertex(
    index: int,
    xy_front_start_point: Point,
    xy_vertex_line: Line,
    share_front_edges: list[Line],
    share_back_edges: list[Line],
    no_cross_triangle_edges: list[Line]
) -> Point:
    """
    |  XY上の判定対象頂点を検索取得

    :param index: 処理を開始する衝突点の前にある三角ポリゴンの衝突点を含まない辺のインデックス
    :type  index: int
    :param xy_front_start_point: 処理を開始する衝突点の前にある三角ポリゴンの衝突点を含まない辺の端点（2次元）
    :type  xy_front_start_point: Point
    :param xy_vertex_line: 衝突点が通過する直線L（2次元）
    :type  xy_vertex_line: Line
    :param share_front_edges: 衝突点の前にある三角ポリゴンの衝突点を含まない辺（2次元）
    :type  share_front_edges: list[Line]
    :param share_back_edges: 衝突点の後にある三角ポリゴンの衝突点を含まない辺（2次元）
    :type  share_back_edges: list[Line]
    :param no_cross_triangle_edges: 衝突点の前後判定が出来なかった三角ポリゴンの衝突点を含まない辺（2次元）
    :type  no_cross_triangle_edges: list[Line]

    :returns: XY上の判定対象頂点(2次元)
    :rtype: Point
    """
    # 判定対象頂点
    target_vertex = None

    copy_no_cross_triangle_edges = copy.copy(no_cross_triangle_edges)

    # 交点CXを通る辺から連結される辺を辿る
    while(True):

        # 衝突点の後にある三角ポリゴンの衝突点を含まない辺を確認
        for share_back_edge in share_back_edges:
            # 衝突点の後にある三角ポリゴンの衝突点を含まない辺が開始点を端点に持つ場合
            if share_back_edge.point.distance_point(
                    xy_front_start_point) < __MINIMA__ \
                or share_back_edge.to_point().distance_point(
                    xy_front_start_point) < __MINIMA__:

                # 直線Lの開始頂点と終了頂点は除外
                if xy_vertex_line.point.distance_point(
                        xy_front_start_point) < __MINIMA__ \
                    or xy_vertex_line.to_point().distance_point(
                        xy_front_start_point) < __MINIMA__:
                    continue

                target_vertex = xy_front_start_point

        if target_vertex is not None:
            break

        # 他の交点CXがＬ上で交点である頂点を挟んで反対にない場合かの判定
        has_cross_front = False
        for front_index, share_front_edge in enumerate(share_front_edges):
            # 自身は除く
            if front_index == index:
                continue

            #  他の交点CXがＬ上で交点である頂点を挟んで反対で無いか確認
            for t in [0, 1]:
                if share_front_edge.to_point(t).distance_point(
                        xy_front_start_point) < __MINIMA__:
                    has_cross_front = True
                    break

            else:
                continue

            # 判定完了時はループを抜ける
            break

        # 他の交点CXがＬ上で交点である頂点を挟んで反対にない場合
        if has_cross_front:
            break

        # 交点CXを通る辺から連結される辺を連結
        has_connect_line = False
        for index, no_cross_triangle_edge \
                in enumerate(copy_no_cross_triangle_edges):
            for t in [0, 1]:
                if no_cross_triangle_edge.to_point(t).distance_point(
                        xy_front_start_point) < __MINIMA__:
                    # 開始点を頂点を共有する辺のもう一端の頂点とする
                    xy_front_start_point \
                        = no_cross_triangle_edge.to_point(1 - t)
                    # チェック対象の辺から除く
                    copy_no_cross_triangle_edges.pop(index)
                    has_connect_line = True
                    break

        # 交点に達しなかった
        if not has_connect_line:
            break

    return target_vertex


def _count_vertex_side(
    cross_point_infos: list[tuple[Point, set[int]]],
    xy_cross_line: Line,
    rectangular_triangles: list[Triangle]
) -> (int, int):
    """
    |  頂点に対する左右判定

    :param cross_point_infos: 衝突点（3次元）と同グループの三角ポリゴンの
                            rectangular_triangles内のインデックスのタプル配列
    :type  cross_point_infos: list[tuple[Point, set[int]]]
    :param xy_cross_line: 衝突点が通過する直線L（2次元）
    :type  xy_cross_line: Line
    :param rectangular_triangles: 全三角ポリゴン
    :type  rectangular_triangles: list[Triangle]

    :returns: 左判定回数
    :rtype: int
    :returns: 右判定回数
    :rtype: int
    """
    # 左判定回数
    left_count = 0
    # 右判定回数
    right_count = 0

    # 衝突点の前にある三角ポリゴンの衝突点を含まない辺
    share_front_edges = list()
    # 衝突点の後にある三角ポリゴンの衝突点を含まない辺
    share_back_edges = list()
    # 衝突点の前後にあるか判定できない三角ポリゴンの衝突点を含まない辺
    no_cross_triangle_edges = list()

    spatial_logger.debug("## 頂点に対する左右判定 ##")
    spatial_logger.debug("直線L：%s", xy_cross_line)

    for cross_point, group_triangle_indexes in cross_point_infos:

        spatial_logger.debug("頂点：%s", cross_point)
        # 頂点に対してグループ化された三角ポリゴンのインデックス
        for group_triangle_index in group_triangle_indexes:

            # 交点CXを求める三角ポリゴン
            check_triangle = rectangular_triangles[group_triangle_index]

            # 三角ポリゴンで衝突点が含まれない辺(2次元)
            xy_check_edge = None
            for vertex_key in __VERTEX_KEY__:
                if check_triangle.point(vertex_key).distance_point(
                        cross_point) < __MINIMA__:
                    check_edge = check_triangle.line(vertex_key.lower())
                    xy_start_edge = \
                        Point([check_edge.point[0], check_edge.point[1]])
                    xy_end_edge = \
                        Point([
                            check_edge.to_point()[0],
                            check_edge.to_point()[1]
                        ])
                    xy_check_edge = \
                        Line.from_points(xy_start_edge, xy_end_edge)
                    break

            # 三角ポリゴンで衝突点が含まれない辺(2次元)が1つに定まらない場合は対象の三角ポリゴンをスキップ
            # 上記は実際の処理上は発生しないルート
            else:
                continue

            # 交点CXが無い場合はスキップ
            if xy_cross_line.side_point(xy_check_edge.point) \
                    * xy_cross_line.side_point(xy_check_edge.to_point()) == 1:
                no_cross_triangle_edges.append(xy_check_edge)
                continue

            # 交点CX(2次元)
            try:
                cx_cross_point = xy_check_edge.intersect_line(xy_cross_line)

            # 直線Lに平行な辺は衝突点の前後にあるか判定できない三角ポリゴンの衝突点を含まない辺とする
            except ValueError:
                no_cross_triangle_edges.append(xy_check_edge)
                continue

            # 交点CXの前後比較成分
            cx_cross_point_value = None
            # 衝突点の前後比較成分
            cross_point_value = None

            # 直線LがX軸に平行な場合はY軸を前後判定の基準にする
            if abs(xy_cross_line.direction[0]) < __MINIMA__:
                cx_cross_point_value = cx_cross_point[1]
                cross_point_value = cross_point[1]

            # 直線LがX軸に平行でない場合
            else:
                cx_cross_point_value = cx_cross_point[0]
                cross_point_value = cross_point[0]

            # 交点CXの前後判定
            if cx_cross_point_value > cross_point_value:
                share_front_edges.append(xy_check_edge)

            elif cx_cross_point_value < cross_point_value:
                share_back_edges.append(xy_check_edge)

    spatial_logger.debug("前に交点CXがある辺: %s", share_front_edges)
    spatial_logger.debug("後に交点CXがある辺: %s", share_back_edges)
    spatial_logger.debug("前後に交点CXがない辺: %s", no_cross_triangle_edges)

    # 交点CXが前後に存在しない場合
    if len(share_front_edges) == 0 or len(share_back_edges) == 0:
        return (left_count, right_count)

    # 衝突点の前にある三角ポリゴンの衝突点を含まない辺から辿る
    target_vertexes = list()
    # 対象判定頂点が見つからない場合は別の辺から辿る
    for front_index, share_front_edge in enumerate(share_front_edges):
        # 右方向・左方向それぞれ見つかるまでつなげる
        for t in [0, 1]:
            xy_front_start_point = share_front_edge.to_point(t)
            # 直線Lに対して対象判定頂点を求める
            target_vertex = _search_judge_vertex(
                front_index,
                xy_front_start_point,
                xy_cross_line,
                share_front_edges,
                share_back_edges,
                no_cross_triangle_edges
            )

            if target_vertex is not None:
                target_vertexes.append(target_vertex)

    # 対象判定頂点がある場合
    for target_vertex in target_vertexes:

        # 直線Lに対して判定対象の頂点が左右どちらにあるかカウントする
        left, right = _count_line_side(xy_cross_line, target_vertex)
        left_count += left
        right_count += right

    return (left_count, right_count)


def _check_cross_one_edge(
    xy_cross_edge: Line,
    group_triangle_indexes: list[int],
    rectangular_triangles: list[Triangle]
) -> bool:
    """
    |  1辺との衝突判定

    :param xy_cross_edge: 衝突点が通過する辺（2次元）
    :type  xy_cross_edge: Line
    :param group_triangle_indexes: 同グループの三角ポリゴンのrectangular_triangles内のインデックス
    :type  group_triangle_indexes: set[int]
    :param rectangular_triangles: 全三角ポリゴン
    :type  rectangular_triangles: list[Triangle]

    :returns: 衝突判定結果(True: 内部、False: 外部)
    :rtype: bool
    """
    # 衝突判定結果
    collision_result = None

    left_count, right_count = _count_edge_side(
        xy_cross_edge, group_triangle_indexes, rectangular_triangles
    )

    if left_count > 0 and right_count > 0:
        spatial_logger.debug("1辺の接点射影内部と判定されました")
        collision_result = True
    else:
        spatial_logger.debug("1辺の接点射影外部と判定されました")
        collision_result = False

    return collision_result


def _check_cross_two_edges(
    xy_cross_edge: Line,
    max_group_triangle_indexes: list[int],
    min_group_triangle_indexes: list[int],
    rectangular_triangles: list[Triangle]
) -> bool:
    """
    |  2辺との衝突判定

    :param xy_cross_edge: 衝突点が通過する辺（2次元）
    :type  xy_cross_edge: Line
    :param max_group_triangle_indexes: 共通の三角ポリゴンがあるグループ群の中で
        衝突点の高さが最大の三角ポリゴングループのrectangular_triangles内のインデックス
    :type  max_group_triangle_indexes: set[int]
    :param min_group_triangle_indexes: 共通の三角ポリゴンがあるグループ群の中で
        衝突点の高さが最小の三角ポリゴングループのrectangular_triangles内のインデックス
    :type  min_group_triangle_indexes: awt[int]
    :param rectangular_triangles: 全三角ポリゴン
    :type  rectangular_triangles: list[Triangle]

    :returns: 衝突判定結果(True: 内部、False: 外部)
    :rtype: bool
    """
    # 衝突判定結果
    collision_result = None

    # 左判定回数
    all_left_count = 0
    # 右判定回数
    all_right_count = 0
    # 衝突点の高さが最大/最小の三角ポリゴングループで内部判定
    for group_triangle_indexes in \
            [max_group_triangle_indexes, min_group_triangle_indexes]:
        left_count, right_count = _count_edge_side(
            xy_cross_edge, group_triangle_indexes, rectangular_triangles
        )
        # 左判定回数
        all_left_count += left_count
        # 右判定回数
        all_right_count += right_count

    if all_left_count > 0 and all_right_count > 0:
        spatial_logger.debug("2辺の接点射影内部と判定されました")
        collision_result = True
    else:
        spatial_logger.debug("2辺の接点射影外部と判定されました")
        collision_result = False

    return collision_result


def _check_cross_vertex_and_edge(
    cross_point: Point,
    xy_cross_edge: Line,
    vertex_group_triangle_indexes: list[int],
    edge_group_triangle_indexes: list[int],
    rectangular_triangles: list[Triangle]
) -> bool:
    """
    |  1頂点1辺との衝突判定

    :param cross_point: 衝突点（3次元）
    :type  cross_point: Point
    :param xy_cross_edge: 衝突点が通過する辺（2次元）
    :type  xy_cross_edge: Line
    :param vertex_group_triangle_indexes: 共通の三角ポリゴンがあるグループ群の中で
        衝突点が頂点の三角ポリゴングループのrectangular_triangles内のインデックス
    :type  vertex_group_triangle_indexes: list[int]
    :param edge_group_triangle_indexes: 共通の三角ポリゴンがあるグループ群の中で
        衝突点がが辺の三角ポリゴングループのrectangular_triangles内のインデックス
    :type  edge_group_triangle_indexes: list[int]
    :param rectangular_triangles: 全三角ポリゴン
    :type  rectangular_triangles: list[Triangle]

    :returns: 衝突判定結果(True: 内部、False: 外部)
    :rtype: bool
    """
    # 衝突判定結果
    collision_result = None

    # 辺との左右判定
    left_count, right_count = _count_edge_side(
        xy_cross_edge, edge_group_triangle_indexes, rectangular_triangles
    )
    # 頂点との左右判定
    vertex_left_count, verrtex_right_count = _count_vertex_side(
        [(cross_point, vertex_group_triangle_indexes)],
        xy_cross_edge,
        rectangular_triangles
    )
    # 左右判定
    left_count += vertex_left_count
    right_count += verrtex_right_count

    if left_count > 0 and right_count > 0:
        spatial_logger.debug("1頂点1辺の接点射影内部と判定されました")
        collision_result = True
    else:
        spatial_logger.debug("1頂点1辺の接点射影外部と判定されました")
        collision_result = False

    return collision_result


def _check_cross_two_vertexes(
    max_cross_point: Point,
    min_cross_point: Point,
    xy_cross_edge: Line,
    max_group_triangle_indexes: set[int],
    min_group_triangle_indexes: set[int],
    rectangular_triangles: list[Triangle]
) -> bool:
    """
    |  2頂点との衝突判定

    :param max_cross_point: 最大高さの衝突点
    :type  max_cross_point: Point
    :param min_cross_point: 最小高さの衝突点
    :type  min_cross_point: Point
    :param xy_cross_edge: 衝突点が通過する辺（2次元）
    :type  xy_cross_edge: Line
    :param max_group_triangle_indexes: 共通の三角ポリゴンがあるグループ群の中で
        衝突点の高さが最大の三角ポリゴングループのrectangular_triangles内のインデックス
    :type  max_group_triangle_indexes: set[int]
    :param min_group_triangle_indexes: 共通の三角ポリゴンがあるグループ群の中で
        衝突点の高さが最小の三角ポリゴングループのrectangular_triangles内のインデックス
    :type  min_group_triangle_indexes: set[int]
    :param rectangular_triangles: 全三角ポリゴン
    :type  rectangular_triangles: list[Triangle]

    :returns: 衝突判定結果(True: 内部、False: 外部)
    :rtype: bool
    """
    # 衝突判定結果
    collision_result = None

    cross_point_infos = [
        tuple((max_cross_point, max_group_triangle_indexes)),
        tuple((min_cross_point, min_group_triangle_indexes))
    ]
    # 頂点との左右判定
    vertex_left_count, vertex_right_count = _count_vertex_side(
        cross_point_infos, xy_cross_edge,
        rectangular_triangles
    )

    if vertex_left_count > 0 and vertex_right_count > 0:
        spatial_logger.debug("2頂点の接点射影内部と判定されました")
        collision_result = True
    else:
        spatial_logger.debug("2頂点の接点射影外部と判定されました")
        collision_result = False

    return collision_result


def _check_cross_one_vertex(
    cross_point: Point,
    group_triangle_indexes: set[int],
    rectangular_triangles: list[Triangle]
) -> bool:
    """
    |  1頂点との衝突判定

    :param cross_point: 衝突点（3次元）
    :type  cross_point: Point
    :param group_triangle_indexes: 衝突点の三角ポリゴングループのrectangular_triangles内のインデックス
    :type  group_triangle_indexes: set[int]
    :param rectangular_triangles: 全三角ポリゴン
    :type  rectangular_triangles: list[Triangle]

    :returns: 衝突判定結果(True: 内部、False: 外部)
    :rtype: bool
    """
    # 衝突判定結果
    collision_result = None

    # 頂点のXY射影
    xy_cross_vertex = Point([cross_point[0], cross_point[1]])
    # 頂点を通る三角ポリゴンの接触点以外の辺のリスト
    xy_not_cross_edges = list()

    for group_triangle_index in group_triangle_indexes:
        # 衝突点の三角ポリゴン
        cross_triangle = rectangular_triangles[group_triangle_index]
        for vertex_key in __VERTEX_KEY__:
            vertex = cross_triangle.point(vertex_key)
            if cross_point.distance_point(vertex) < __MINIMA__:

                # 衝突点がない辺
                not_cross_edge = cross_triangle.line(vertex_key.lower())
                xy_not_cross_edge_start = Point([
                    not_cross_edge.point[0],
                    not_cross_edge.point[1]
                ])
                xy_not_cross_edge_end = Point([
                    not_cross_edge.to_point()[0],
                    not_cross_edge.to_point()[1]
                ])

                if xy_not_cross_edge_start.distance_point(
                        xy_not_cross_edge_end) < __MINIMA__:
                    continue

                # 衝突点がない辺のXY射影
                xy_not_cross_edge = Line.from_points(
                    xy_not_cross_edge_start, xy_not_cross_edge_end
                )
                xy_not_cross_edges.append(xy_not_cross_edge)

    # 頂点を通る三角ポリゴンの接触点以外の辺を連結した図形
    rect_line_group = _make_line_group(xy_not_cross_edges)

    if _is_inside(rect_line_group[0], xy_cross_vertex):
        spatial_logger.debug("1頂点の接点射影内部と判定されました")
        collision_result = True
    else:
        spatial_logger.debug("1頂点の接点射影外部と判定されました")
        collision_result = False

    return collision_result


def _is_same_line(line_a: Line, line_b: Line) -> bool:
    """
    |  線分が同一であるかの判定

    :param line_a: 辺A
    :type  line_a: Line
    :param line_b: 辺B
    :type  line_b: Line

    :returns: 辺が同一：True 辺が同一でない：False
    :rtype: bool
    """

    # 辺Aの端点
    line_a_start = line_a.point
    line_a_end = line_a.to_point()
    # 辺Bの端点
    line_b_start = line_b.point
    line_b_end = line_b.to_point()

    # 辺の端点の一致を確認
    is_pos_line = line_a_start.distance_point(line_b_start) < __MINIMA__ \
        and line_a_end.distance_point(line_b_end) < __MINIMA__
    is_neg_line = line_a_start.distance_point(line_b_end) < __MINIMA__ \
        and line_a_end.distance_point(line_b_start) < __MINIMA__

    # どちらかの組で一致すればTrue
    return is_pos_line or is_neg_line


def _make_triangle_group(
    cross_point_cordinate_result: list[Point, str, str],
    vertex_dict: dict[str, Point],
    edge_list: list[str],
    edge_line_triangle_group: dict[str, (Line, set[int])],
    vertex_triangle_group: dict[str, set[int]],
    cross_group_index: dict[str, int],
    triangle_index: int,
    rectangular_triangles: list[Triangle]
) -> None:
    """
    |  三角ポリゴングループ作成・更新

    :param cross_point_cordinate_result: 衝突点情報（3次元）
    :type  cross_point_cordinate_result: list[Point, str, str]
    :param vertex_dict: 衝突頂点辞書
    :type  vertex_dict: dict[str, Point]
    :param edge_list: 衝突辺リスト
    :type  edge_list: list[str]
    :param edge_line_triangle_group: 衝突点の辺の線情報と三角ポリゴングループの
                                  rectangular_triangles内のインデックス
    :type  edge_line_triangle_group: dict[str, (Line, set[int])]
    :param vertex_triangle_group: 衝突点の頂点の三角ポリゴングループの
                                    rectangular_triangles内のインデックス
    :type  vertex_triangle_group: dict[str, set[int]]
    :param cross_group_index: 衝突点の三角ポリゴンのインデックス
    :type  cross_group_index: dict[str, int]
    :param triangle_index: 衝突点の三角ポリゴンのインデックス
    :type  triangle_index: int
    :param rectangular_triangles: 全三角ポリゴン
    :type  rectangular_triangles: list[Triangle]
    """
    cross_point, edge_key, vertex_key = cross_point_cordinate_result
    # 交差点を文字列に変換
    cross_point_str = _change_point_str(cross_point)
    # 判定三角ポリゴン
    rectangular_triangle = rectangular_triangles[triangle_index]

    # 事前に処理を行った頂点を共有しない辺・頂点のグループ数分グループ番号をずらす
    start_offset = 1
    if cross_point_str in cross_group_index.keys():
        start_offset = cross_group_index[cross_point_str]

    # 辺と交差する場合
    if edge_key is not None:

        # 衝突辺
        cross_edge = rectangular_triangle.line(edge_key)
        cross_edge_str = _change_line_str(cross_edge)

        # 衝突辺リストに衝突辺が登録済みの場合は三角ポリゴンがグルーピング済みなのでスキップ
        if cross_edge_str in edge_list:
            spatial_logger.debug(
                "衝突辺辞書に衝突点が登録済み"
            )
            return

        # 衝突辺をリストに保存
        edge_list.append(cross_edge_str)

        # 衝突辺の両端の点を取得
        point_a = cross_edge.point
        point_b = cross_edge.to_point()
        # グループ対象辺リスト
        line_group = list()
        # 頂点Aを含まない辺で頂点Bを含む辺を持つ三角ポリゴンの辞書
        exp_triangle = dict()
        # グループ対象の三角ポリゴンを決定
        for share_triangle_index, share_triangle \
                in enumerate(rectangular_triangles):
            for vertex in __VERTEX_KEY__:
                # 頂点Aを共有する三角ポリゴン
                if share_triangle.point(vertex).distance_point(point_a) \
                        < __MINIMA__:
                    # 頂点Aを含まない辺
                    share_line = share_triangle.line(vertex.lower())
                    # 頂点Aを含まない辺が頂点Bを含まない辺であるかを確認
                    distance_a = share_line.point.distance_point(point_b)
                    distance_b = share_line.to_point().distance_point(point_b)

                    # 頂点Aを含まない辺が頂点Bを含まない辺である場合
                    if not distance_a < __MINIMA__ \
                            and not distance_b < __MINIMA__:
                        # グループ対象辺リストへ追加
                        line_group.append(share_line)

                    # 頂点Aを含まない辺で頂点Bを含む辺である場合
                    else:
                        # 頂点Aを含まない辺で頂点Bを含む辺を持つ三角ポリゴンの辞書に追加
                        exp_triangle[share_triangle_index] = share_line

        # グループ対象辺リストを繋がる辺同士でグループ化する
        # 要素数は辺ABを共有するオブジェクトの個数
        connect_line_group = _make_line_group(line_group)

        # 次回処理での衝突点のグループ番号を更新
        cross_group_index[cross_point_str] = \
            start_offset + len(connect_line_group)

        # グループ対象辺リストをグルーピングしたグループごとに処理
        # 事前に処理を行った頂点を共有しない辺・頂点のグループ数分グループ番号をずらす
        for index, connect_line \
                in enumerate(connect_line_group, start=start_offset):

            # 頂点Aを含まない辺で頂点Bを含む辺を持つ三角ポリゴンのインデックス
            group_cross_point_str = cross_point_str + "%" + str(index)

            # 連絡先がない頂点と頂点Bを持つ辺を持つ三角ポリゴン
            for share_triangle_index, exp_line in exp_triangle.items():

                # グループ対象辺リストを繋けた結果の端と頂点Bを繋げた辺と
                # 頂点Aを含まない辺で頂点Bを含む辺が一致するか確認する
                # 繋げた結果の始端の辺と一致
                start_line = Line.from_points(connect_line[0], point_b)
                is_line_a = _is_same_line(exp_line, start_line)

                # 繋げた結果の終端の辺と一致
                end_line = Line.from_points(connect_line[-1], point_b)
                is_line_b = _is_same_line(exp_line, end_line)

                # 上記の辺が一致しない場合はスキップ
                if not is_line_a and not is_line_b:
                    continue

                # 頂点Aを含まない辺で頂点Bを含む辺を持つ三角ポリゴンのインデックスを
                # グループに所属する三角ポリゴンとして登録
                if group_cross_point_str in edge_line_triangle_group.keys():
                    edge_line_triangle_group[group_cross_point_str][1].add(
                        share_triangle_index
                    )
                else:
                    edge_line_triangle_group[group_cross_point_str] = (
                        cross_edge,
                        {
                            share_triangle_index
                        }
                    )

    elif vertex_key is not None:

        # 衝突頂点辞書に衝突点が登録済みの場合は三角ポリゴンがグルーピング済みなのでスキップ
        if cross_point_str in vertex_dict.keys():
            spatial_logger.debug(
                "衝突頂点辞書に衝突点が登録済み"
            )
            return

        # 衝突頂点を辞書に保存
        vertex_dict[cross_point_str] = cross_point

        line_group = list()
        exp_triangle = dict()
        for check_triangle_index, check_triangle \
                in enumerate(rectangular_triangles):
            for vertex in __VERTEX_KEY__:
                # 頂点Aを共有する三角ポリゴン
                if check_triangle.point(vertex).distance_point(cross_point) \
                        < __MINIMA__:
                    # 頂点Aを含まない辺
                    check_line = check_triangle.line(vertex.lower())
                    line_group.append(check_line)
                    exp_triangle[check_triangle_index] = check_line

        # 頂点Aを含まない辺を互いに繋がる辺同士でグルーピング
        # 要素数は頂点Aを共有するオブジェクトの個数
        connect_line_group = _make_line_group(line_group)

        # 次回処理での衝突点のグループ番号を更新
        cross_group_index[cross_point_str] = \
            start_offset + len(connect_line_group)

        for group_index, connect_line \
                in enumerate(connect_line_group, start=start_offset):

            # グループ化した辺を含む三角ポリゴンのインデックス
            group_cross_point_str = cross_point_str + "%" + str(group_index)

            # グループ内の辺がどの三角ポリゴンの辺か確認する
            for index, _ in enumerate(connect_line[:-1]):
                line = Line.from_points(
                    connect_line[index],
                    connect_line[index+1]
                )
                for share_triangle_index, exp_line in exp_triangle.items():

                    # グループ化した辺を含む三角ポリゴンを選択する
                    if not _is_same_line(exp_line, line):
                        continue

                    # グループ化した辺を含む三角ポリゴンのインデックスを
                    # グループに所属する三角ポリゴンとして登録
                    if group_cross_point_str in vertex_triangle_group.keys():
                        vertex_triangle_group[group_cross_point_str].add(
                            share_triangle_index
                        )
                    else:
                        vertex_triangle_group[group_cross_point_str] = {
                            share_triangle_index
                        }


def _get_inner_voxel(
    rectangular_triangles: list[Triangle],
    spatial_ids: list[str],
    inner_spatial_ids: set[str],
    outer_spatial_ids: set[str]
) -> None:
    """
    |  内部ボクセル取得

    :param rectangular_triangles: 全三角ポリゴン
    :type  rectangular_triangles: list[Triangle]
    :param spatial_ids: ボクセルの空間ID
    :type  spatial_ids: list[str]
    :param inner_spatial_ids: 内部のボクセル空間ID
    :type  inner_spatial_ids: set[str]
    :param outer_spatial_ids: 外部のボクセル空間ID
    :type  outer_spatial_ids: set[str]
    """

    # 空間IDの成分を取得
    split_spatial_ids = [
        get_voxel_id_to_spatial_id(spatial_id) for spatial_id in spatial_ids
    ]

    # 三角形ポリゴンが重なっている空間IDの最大の高さ
    alt_max_index = max(
        split_spatial_ids, key=lambda spatial_id: spatial_id[2]
    )[2]
    spatial_logger.debug(
        "三角形ポリゴンが重なっている空間IDの最大の高さ: %s", alt_max_index
    )

    # 空間ID毎に処理
    for spatial_id in spatial_ids:

        # 【緯度経度空間】内外判定対象空間ID
        check_spatial_id = get_shifting_spatial_id(spatial_id, 0, 0, 1)
        spatial_logger.debug("内外判定対象空間ID: %s", check_spatial_id)

        # 内外判定対象空間IDが三角ポリゴンの空間IDの場合はスキップ
        if check_spatial_id in spatial_ids:
            spatial_logger.debug(
                "内外判定対象空間IDが三角ポリゴンの空間IDの場合はスキップ"
            )
            continue

        # 内外判定対象空間IDから内部判定を行う空間ID
        check_spatial_ids = set()
        is_outer = False
        while(True):

            # 空間IDの高さ成分を取得
            alt_index = get_voxel_id_to_spatial_id(check_spatial_id)[2]

            # 内外判定対象空間IDが内判定済みの場合
            if check_spatial_id in inner_spatial_ids:
                inner_spatial_ids.update(check_spatial_ids)
                is_outer = True
                spatial_logger.debug(
                    "内外判定対象空間IDが内判定済み: %s", check_spatial_ids
                )
                break

            # 内外判定対象空間IDが外判定済みの場合
            elif check_spatial_id in outer_spatial_ids:
                outer_spatial_ids.update(check_spatial_ids)
                is_outer = True
                spatial_logger.debug(
                    "内外判定対象空間IDが外判定済み: %s", check_spatial_ids
                )
                break

            # 三角形ポリゴンが重なっている空間IDの最大の高さを超過
            elif alt_index > alt_max_index:
                # そこまでSpatialId/shape/polygons.pyの空間IDは外部判定
                outer_spatial_ids.update(check_spatial_ids)
                is_outer = True
                spatial_logger.debug(
                    "空間IDの最大の高さを超過: %s", check_spatial_ids
                )
                break

            # 三角ポリゴンが重なっている空間IDに衝突
            elif check_spatial_id in spatial_ids:
                # 内外判定
                spatial_logger.debug(
                    "三角ポリゴンが重なっている空間IDに衝突: %s",
                    check_spatial_ids
                )
                check_spatial_id \
                    = get_shifting_spatial_id(check_spatial_id, 0, 0, -1)
                break

            check_spatial_ids.add(check_spatial_id)
            check_spatial_id \
                = get_shifting_spatial_id(check_spatial_id, 0, 0, 1)
            spatial_logger.debug(
                "内外判定対象空間IDをZ方向に+1: %s", check_spatial_id
            )

        # 三角ポリゴンが重なる空間IDがない場合は次の空間IDの処理へ移る
        if is_outer:
            continue

        spatial_logger.debug(
            "内外判定対象の空間ID: %s", check_spatial_ids
        )

        # 内外判定
        # 【直交座標空間】始点ボクセルの中心座標
        start_point = get_point_on_spatial_id(
            check_spatial_id, Point_Option.CENTER, __ORTH_CRS__
        )[0]
        start_orth_point = \
            Point([start_point.x, start_point.y, start_point.alt])

        spatial_logger.debug(
            "始点ボクセルの中心座標(空間ID): %s", check_spatial_id
        )
        spatial_logger.debug(
            "始点ボクセルの中心座標(投影座標): %s", start_orth_point
        )

        # ボクセルに対するグループ
        vertex_dict = dict()
        # 衝突辺の文字列リスト
        edge_list = list()
        # 辺の線と三角ポリゴングループ（キー：衝突点の文字列にグループ番号を加えた値、値：辺の線情報と三角ポリゴンのインデックスのタプル）
        edge_line_triangle_group = dict()
        # 頂点の三角ポリゴングループ（キー：衝突点の文字列にグループ番号を加えた値、値：三角ポリゴンのインデックス）
        vertex_triangle_group = dict()
        # 衝突点に対するグループ数+1
        cross_group_index = dict()
        # 処理済みの衝突点集合
        judgment_completed_cross_points = set()
        # 交差判定結果リスト
        cross_point_cordinate_results_list = list()

        # 衝突回数
        collision_count = 0
        # 【直交座標空間】三角ポリゴン毎にレイとの衝突判定
        for triangle_index, rectangular_triangle \
                in enumerate(rectangular_triangles):
            spatial_logger.debug(
                "三角ポリゴン毎にレイとの衝突判定: %s, %s",
                triangle_index, rectangular_triangle
            )

            # 交差判定
            cross_point_cordinate_results = \
                _get_cross_point_cordinate(
                    rectangular_triangle,
                    start_orth_point
                )
            spatial_logger.debug(
                "交差判定の結果: %s", cross_point_cordinate_results
            )

            # グループ作成実施(グループ作成出来ない場合は衝突判定・内部判定済み)
            for cross_point_cordinate_result in cross_point_cordinate_results:
                _make_triangle_group(
                    cross_point_cordinate_result,
                    vertex_dict,
                    edge_list,
                    edge_line_triangle_group,
                    vertex_triangle_group,
                    cross_group_index,
                    triangle_index,
                    rectangular_triangles
                )

                spatial_logger.debug(
                    "衝突点情報: %s",
                    cross_point_cordinate_result
                )
                spatial_logger.debug(
                    "衝突頂点辞書: %s",
                    vertex_dict
                )
                spatial_logger.debug(
                    "衝突辺リスト: %s",
                    edge_list
                )
                spatial_logger.debug(
                    "衝突点の頂点の三角ポリゴングループ: %s",
                    vertex_triangle_group
                )
                spatial_logger.debug(
                    "衝突点の辺の線と三角ポリゴングループ: %s",
                    edge_line_triangle_group
                )

            if len(cross_point_cordinate_results) > 0:
                cross_point_cordinate_results_list.append(
                    cross_point_cordinate_results
                )

        spatial_logger.debug(
            "衝突点の結果を元に内部判定を実施開始: %s", check_spatial_id
        )
        spatial_logger.debug(
            "衝突頂点辞書: %s",
            vertex_dict
        )
        spatial_logger.debug(
            "衝突辺リスト: %s",
            edge_list
        )
        spatial_logger.debug(
            "衝突点の頂点の三角ポリゴングループ: %s",
            vertex_triangle_group
        )
        spatial_logger.debug(
            "衝突点の辺の線と三角ポリゴングループ: %s",
            edge_line_triangle_group
        )
        spatial_logger.debug(
            "衝突点に対するグループ数+1: %s",
            cross_group_index
        )
        # 衝突点の結果を元に内部判定を実施
        for cross_point_cordinate_results \
                in cross_point_cordinate_results_list:

            spatial_logger.debug(
                "衝突点情報: %s",
                cross_point_cordinate_results
            )

            # 面と衝突する場合
            if len(cross_point_cordinate_results) == 1 \
                    and cross_point_cordinate_results[0][1] is None \
                    and cross_point_cordinate_results[0][2] is None:
                spatial_logger.debug(
                    "面と衝突"
                )
                inside_results = [True]

            else:
                # 辺頂点に衝突した場合の内部判定
                inside_results = \
                    _cross_check_edge_vertex(
                        vertex_triangle_group,
                        edge_line_triangle_group,
                        vertex_dict,
                        rectangular_triangles,
                        judgment_completed_cross_points
                    )

                spatial_logger.debug(
                    "辺頂点に衝突した場合の内部判定: %s",
                    inside_results
                )

            for inside_result in inside_results:
                if inside_result:
                    collision_count += 1

            spatial_logger.debug(
                "衝突回数: %s",
                collision_count
            )

        # 衝突回数が奇数の場合は内側判定
        if collision_count % 2 == 1:
            spatial_logger.debug(
                "衝突回数が奇数の場合は内側判定",
            )
            inner_spatial_ids.update(check_spatial_ids)

        # 衝突回数が偶数の場合は外側判定
        else:
            spatial_logger.debug(
                "衝突回数が偶数の場合は外側判定",
            )
            outer_spatial_ids.update(check_spatial_ids)

    return inner_spatial_ids


def _get_share_triangle_index(
    max_z_point: str,
    vertex_triangle_group: dict[str, set[int]],
) -> int:
    """
    |  最大頂点共有三角ポリゴン取得

    :param max_z_point: 最大頂点
    :type  max_z_point: str
    :param vertex_triangle_group: 頂点三角ポリゴングループ
    :type  vertex_triangle_group: dict[str, set[int]]

    :returns: 共有三角ポリゴンインデックス
    :rtype:   int
    """
    # 最大頂点グループ三角ポリゴンインデックス
    max_group_triangle_indexes = vertex_triangle_group[max_z_point]
    # 共有三角ポリゴンインデックス
    share_triangle_index = None
    for max_group_triangle_index in max_group_triangle_indexes:
        for key, value in vertex_triangle_group.items():

            # 自身は除く
            if key == max_z_point:
                continue

            # 頂点三角ポリゴングループの中で最大頂点グループ三角ポリゴンを持つものがいた場合
            if max_group_triangle_index in value:
                share_triangle_index = max_group_triangle_index
                break

        # 最大頂点グループ三角ポリゴンのインデクスを共有する三角ポリゴンが無い場合は
        # 他の最大頂点グループ三角ポリゴンのインデクスへ移動
        else:
            continue

        break

    return share_triangle_index


def _cross_check_edge_vertex(
    vertex_triangle_group: dict[str, set[int]],
    edge_line_triangle_group: dict[str, (Line, set[int])],
    vertex_dict: dict[str, Point],
    rectangular_triangles: list[Triangle],
    judgment_completed_cross_points: set[str]
) -> list[bool]:
    """
    |  辺・頂点の内部判定

    :param vertex_triangle_group: 頂点三角ポリゴングループ
    :type  vertex_triangle_group: dict[str, set[int]]
    :param edge_line_triangle_group: 辺三角ポリゴングループ
    :type  edge_line_triangle_group: dict[str, (Line, set[int])]
    :param vertex_dict: 衝突頂点辞書
    :type  vertex_dict: dict[str, Point]
    :param rectangular_triangles: 全三角ポリゴン情報
    :type rectangular_triangles: list[Triangle]
    :param judgment_completed_cross_points: 処理済みの衝突点集合
    :type judgment_completed_cross_points: set[str]

    :returns: 内部判定結果
    :rtype:   list[bool]
    """

    # 内部判定
    inside_results = list()

    # 辺文字列と三角ポリゴンの辞書を作成
    edge_triangle_group = {
        key: value[1] for key, value in edge_line_triangle_group.items()
    }

    triangle_group = vertex_triangle_group | edge_triangle_group

    # 衝突判定
    for group_cross_point_str, triangle_indexes in triangle_group.items():
        spatial_logger.debug(
            "衝突判定: %s, %s", group_cross_point_str, triangle_indexes
        )

        # 判定済みは飛ばす
        if group_cross_point_str in judgment_completed_cross_points:
            spatial_logger.debug(
                "処理済みの衝突点のためスキップ"
            )
            continue

        # 辺と衝突するか判定
        is_edge = None
        if group_cross_point_str in edge_triangle_group:
            is_edge = True

        cross_point_str, _ = group_cross_point_str.split("%")

        # 共有する三角ポリゴンをもつグルーピング済み衝突点
        cross_point_set = {group_cross_point_str}
        # 共有する三角ポリゴンのインデックス
        union_triangle_indexes = triangle_indexes
        # 共有する三角ポリゴンを持つグループを一つにまとめる
        _grouping_triangle(
            cross_point_set, union_triangle_indexes,
            edge_triangle_group, vertex_triangle_group
        )

        # 判定済み衝突点に追加
        judgment_completed_cross_points |= cross_point_set

        # 1頂点との衝突
        # 頂点に衝突した三角ポリゴンのグループに対して、グループに所属する三角ポリゴンが他のグループに所属しない場合
        if len(cross_point_set) == 1 and not is_edge:
            spatial_logger.debug(
                "1頂点と衝突"
            )

            # 接触点である頂点
            cross_point = vertex_dict[cross_point_str]

            # 頂点に対してグループ化された三角ポリゴンのインデックス
            group_triangle_indexes = triangle_indexes

            # 判定
            inside_result = _check_cross_one_vertex(
                cross_point, group_triangle_indexes, rectangular_triangles
            )

        # 一辺との衝突
        # 辺に衝突した三角ポリゴンのグループに対して、グループに所属する三角ポリゴンが他のグループに所属しない場合
        elif len(cross_point_set) == 1 and is_edge:
            spatial_logger.debug(
                "1辺と衝突"
            )
            # 衝突点がある辺
            cross_edge = edge_line_triangle_group[group_cross_point_str][0]

            xy_cross_edge_start = Point([
                cross_edge.point[0], cross_edge.point[1]
            ])
            xy_cross_edge_end = Point([
                cross_edge.to_point()[0], cross_edge.to_point()[1]
            ])
            # 衝突点が通過する辺のXY射影
            xy_cross_edge = Line.from_points(
                xy_cross_edge_start, xy_cross_edge_end
            )
            # 衝突点がある辺に対してグループ化された三角ポリゴンのインデックス
            group_triangle_indexes = triangle_indexes

            # 判定結果
            inside_result = _check_cross_one_edge(
                xy_cross_edge, group_triangle_indexes, rectangular_triangles
            )

        # 2辺・2頂点・1頂点1辺との衝突
        else:

            spatial_logger.debug(
                "2辺・2頂点・1頂点1辺と衝突"
            )

            # 最大・最小の衝突点を算出する
            cross_points = list(cross_point_set)
            max_z_point = max(
                cross_points,
                key=lambda x: float(x.split("%")[0].split("_")[2])
            )
            min_z_point = min(
                cross_points,
                key=lambda x: float(x.split("%")[0].split("_")[2])
            )

            spatial_logger.debug(
                "最大衝突点: %s", max_z_point
            )
            spatial_logger.debug(
                "最小衝突点: %s", min_z_point
            )

            # 最大・最小の衝突点が辺で衝突するか頂点で衝突するか判定する
            is_max_edge = max_z_point in edge_triangle_group.keys()
            is_min_edge = min_z_point in edge_triangle_group.keys()

            # 2辺との衝突
            if is_max_edge and is_min_edge:

                # 衝突点がある辺
                cross_edge = edge_line_triangle_group[max_z_point][0]
                xy_cross_edge_start = Point([
                    cross_edge.point[0], cross_edge.point[1]
                ])
                xy_cross_edge_end = Point([
                    cross_edge.to_point()[0], cross_edge.to_point()[1]
                ])
                # 衝突点が通過する辺のXY射影
                xy_cross_edge = Line.from_points(
                    xy_cross_edge_start, xy_cross_edge_end
                )

                # 辺に対してグループ化された三角ポリゴンのインデックス
                max_group_triangle_indexes = edge_triangle_group[max_z_point]
                min_group_triangle_indexes = edge_triangle_group[min_z_point]

                # 判定
                inside_result = _check_cross_two_edges(
                    xy_cross_edge,
                    max_group_triangle_indexes,
                    min_group_triangle_indexes,
                    rectangular_triangles
                )

            # 1頂点1辺との衝突
            elif is_max_edge or is_min_edge:
                if is_max_edge:
                    vertex_cross_point_str, _ = min_z_point.split("%")
                    edge_cross_point_str = max_z_point
                    # 辺・頂点に対してグループ化された三角ポリゴンのインデックス
                    vertex_triangle_indexes = \
                        vertex_triangle_group[min_z_point]
                    edge_triangle_indexes = edge_triangle_group[max_z_point]
                else:
                    vertex_cross_point_str, _ = max_z_point.split("%")
                    edge_cross_point_str = min_z_point
                    # 辺・頂点に対してグループ化された三角ポリゴンのインデックス
                    vertex_triangle_indexes = \
                        vertex_triangle_group[max_z_point]
                    edge_triangle_indexes = edge_triangle_group[min_z_point]

                # 衝突点がある頂点
                cross_point = vertex_dict[vertex_cross_point_str]

                # 衝突点がある辺
                cross_edge = edge_line_triangle_group[edge_cross_point_str][0]
                xy_cross_edge_start = Point([
                    cross_edge.point[0], cross_edge.point[1]
                ])
                xy_cross_edge_end = Point([
                    cross_edge.to_point()[0], cross_edge.to_point()[1]
                ])
                # 衝突点が通過する辺のXY射影
                xy_cross_edge = Line.from_points(
                    xy_cross_edge_start, xy_cross_edge_end
                )

                # 判定
                inside_result = _check_cross_vertex_and_edge(
                    cross_point,
                    xy_cross_edge,
                    vertex_triangle_indexes,
                    edge_triangle_indexes,
                    rectangular_triangles
                )

            # 2頂点との衝突
            else:
                max_cross_point_str, _ = max_z_point.split("%")
                min_cross_point_str, _ = min_z_point.split("%")

                # 衝突点がある頂点
                max_cross_point = vertex_dict[max_cross_point_str]
                min_cross_point = vertex_dict[min_cross_point_str]

                # 頂点に対してグループ化された三角ポリゴンのインデックス
                max_group_triangle_indexes = vertex_triangle_group[max_z_point]
                min_group_triangle_indexes = vertex_triangle_group[min_z_point]

                # 頂点に衝突した三角ポリゴンで複数グループに含まれる三角ポリゴン
                share_triangle_index = \
                    _get_share_triangle_index(
                        max_z_point,
                        vertex_triangle_group
                    )

                # 複数グループに含まれる三角ポリゴンが無い場合
                if share_triangle_index is None:
                    inside_results.append(False)
                    continue

                share_triangle = rectangular_triangles[share_triangle_index]

                # 複数グループに含まれる三角ポリゴンの頂点をXY平面に射影
                xy_vertexs = list()
                for vertex_key in __VERTEX_KEY__:
                    vertex = share_triangle.point(vertex_key)
                    xy_vertex = Point([vertex[0], vertex[1]])
                    xy_vertexs.append(xy_vertex)

                # 直線L
                xy_vertex_line = None
                if xy_vertexs[0].distance_point(xy_vertexs[1]) < __MINIMA__:
                    xy_vertex_line = Line.from_points(
                        xy_vertexs[0], xy_vertexs[2]
                    )

                else:
                    xy_vertex_line = Line.from_points(
                        xy_vertexs[0], xy_vertexs[1]
                    )

                # 判定
                inside_result = _check_cross_two_vertexes(
                    max_cross_point,
                    min_cross_point,
                    xy_vertex_line,
                    max_group_triangle_indexes,
                    min_group_triangle_indexes,
                    rectangular_triangles
                )

        # 内外判定結果格納
        inside_results.append(inside_result)

    return inside_results


def _get_cross_point_cordinate(
    rectangular_triangle, start_orth_point
) -> list[tuple[Point, str, str]]:
    """ 【直交座標空間】
    | ボクセルの中心点からZ軸方向へのレイと三角ポリゴンの接触点を算出

    :param rectangular_triangle: 三角ポリゴン
    :type  rectangular_triangle: Triangle
    :param start_orth_point: ボクセルの中心点
    :type  start_orth_point: Point

    :returns: 以下の成分を持つタプルのリスト
              衝突点、
              衝突点がある辺のキー（辺に衝突しない場合はNone）、
              衝突点がある頂点のキー（頂点に衝突しない場合はNone）
    :rtype:   list[tuple[Point, str, str]]
    """
    # ボクセルの中心点からZ軸方向へのレイ
    ray_line = Line(start_orth_point, Vector([0, 0, 1]))
    # 三角ポリゴンが属する平面
    triangle_plane = Plane(
        rectangular_triangle.point_a, rectangular_triangle.normal()
    )
    return_value = list()

    # Z軸方向へのレイと三角ポリゴンが属する平面が水平の場合
    # ※交差点の数は0～2個
    if abs(np.dot(rectangular_triangle.normal(), ray_line.direction)) \
            < __MINIMA__:

        for key in __EDGE_KEY__:
            edge_key = None
            vertex_key = None
            # 三角ポリゴンの辺
            edge = rectangular_triangle.line(key)

            # 辺とレイの交差点を取得
            try:
                edge_cross_point = edge.intersect_line(ray_line)
                if edge_cross_point[2] < start_orth_point[2]:
                    continue

            # 辺とレイが平行な場合は次の辺の処理へ移行
            # 辺とレイが重なる場合は他の辺の接点として、交差点は取得可能
            except ValueError:
                continue

            # 辺とレイの交差点が三角ポリゴンの範囲内であるかを確認
            edge_length = edge.direction.norm()
            edge_cross_length1 = \
                Vector.from_points(edge_cross_point, edge.point).norm()
            edge_cross_length2 = \
                Vector.from_points(edge_cross_point, edge.to_point()).norm()

            # 交点までの長さの差がある場合は辺上に無いと判定する
            if abs(edge_length - edge_cross_length1 - edge_cross_length2) \
                    > __EDGE_MINIMA__:
                continue

            edge_key = key

            # 交差点が三角ポリゴンの頂点であるかを確認
            for vertex in __VERTEX_KEY__:
                if rectangular_triangle.point(vertex).distance_point(
                        edge_cross_point) < __MINIMA__:
                    edge_key = None
                    vertex_key = vertex
                    break

            # 同じ交差点は除外する
            for value in return_value:
                if value[0].distance_point(edge_cross_point) < __MINIMA__:
                    break

            else:
                return_value.append((edge_cross_point, edge_key, vertex_key))

    # Z軸方向へのレイと三角ポリゴンが属する平面が水平でない場合
    else:
        # 三角ポリゴンとZ方向のレイとの交点を求める
        collision_point = triangle_plane.intersect_line(ray_line)
        if collision_point[2] < start_orth_point[2]:
            return []

        # レイの開始点(XY平面)
        ray_start_point = Point([start_orth_point[0], start_orth_point[1]])

        # 頂点との衝突確認
        vertex_key = None
        edge_key = None
        for vertex in __VERTEX_KEY__:
            exp_vetex = rectangular_triangle.point(vertex)
            xy_exp_vetex = Point([exp_vetex[0], exp_vetex[1]])
            # XY平面上の距離で確認
            if xy_exp_vetex.distance_point(ray_start_point) < __MINIMA__:
                # 衝突点を実際の頂点座標に補正
                collision_point = exp_vetex
                vertex_key = vertex
                break

        # 頂点に衝突する場合
        if vertex_key is not None:
            return [(collision_point, edge_key, vertex_key)]

        # 辺との衝突確認
        for edge in __EDGE_KEY__:
            exp_edge = rectangular_triangle.line(edge)
            exp_point = exp_edge.point
            exp_direction = exp_edge.direction
            xy_exp_edge = Line(
                point=Point([exp_point[0], exp_point[1]]),
                direction=Vector([exp_direction[0], exp_direction[1]])
            )

            # XY平面上の距離で確認
            edge_length = xy_exp_edge.direction.norm()
            edge_cross_length1 = \
                Vector.from_points(xy_exp_edge.point, ray_start_point).norm()
            edge_cross_length2 = \
                Vector.from_points(
                    xy_exp_edge.to_point(),
                    ray_start_point
                ).norm()

            if abs(edge_length - edge_cross_length1 - edge_cross_length2) \
                    < __EDGE_MINIMA__:
                edge_key = edge
                break

        # 辺に衝突する場合
        if edge_key is not None:
            return [(collision_point, edge_key, vertex_key)]

        # 各辺
        edge1 = Vector.from_points(
            rectangular_triangle.point_b,
            rectangular_triangle.point_a
        )
        edge2 = Vector.from_points(
            rectangular_triangle.point_c,
            rectangular_triangle.point_b
        )
        edge3 = Vector.from_points(
            rectangular_triangle.point_a,
            rectangular_triangle.point_c
        )

        # 各頂点から衝突点
        collision_apex1 = Vector.from_points(
            collision_point,
            rectangular_triangle.point_b
        )
        collision_apex2 = Vector.from_points(
            collision_point,
            rectangular_triangle.point_c
        )
        collision_apex3 = Vector.from_points(
            collision_point,
            rectangular_triangle.point_a
        )

        c1 = edge1.cross(collision_apex1)
        c2 = edge2.cross(collision_apex2)
        c3 = edge3.cross(collision_apex3)

        if np.dot(c1, c2) > 0 and np.dot(c1, c3) > 0:
            # 【直交座標空間】三角形上の点の座標
            cross_point = collision_point

            return_value.append((cross_point, edge_key, vertex_key))

    return return_value


def _validate_args_type(
    barrier_triangles: list[SpatialTriangle],
    space_triangles: list[SpatialTriangle],
    h_zoom: int,
    v_zoom: int,
    crs: int = __CRS__,
    needs_closed_checking: bool = True
) -> None:
    """
    入力引数の型チェック

    :param barrier_triangles: 空間IDを取得する三角形ポリゴンの集合で
                              表されるモデル。Triangleオブジェクトを組み合わせ、複数指定する。
    :type barrier_triangles: list[SpatialId.common.object.point.Triangle]
    :param space_triangles: 除外する三角形ポリゴンの集合で表されるモデル。
                            Triangleオブジェクトを組み合わせ、複数指定する。
    :type space_triangles: list[SpatialId.common.object.point.Triangle]
    :param h_zoom: 水平方向の精度レベル
    :type h_zoom: int
    :param v_zoom: 垂直方向の精度レベル
    :type v_zoom: int
    :param crs: 座標参照系
    :type crs: int
    :param needs_closed_checking: 閉塞チェックフラグ。Trueの場合はチェックを行う。
    :type needs_closed_checking: bool

    :raise SpatialIdError: 入力チェック結果が不正の場合のエラー
    """
    error_flag = False

    if type(barrier_triangles) is not list:
        error_flag = True

    else:
        for triangle in barrier_triangles:
            if type(triangle) is not SpatialTriangle and \
                type(triangle) is not Projected_Triangle:
                error_flag = True
                break

    if type(space_triangles) is not list:
        error_flag = True

    else:
        for triangle in space_triangles:
            if type(triangle) is not SpatialTriangle and \
                type(triangle) is not Projected_Triangle:
                error_flag = True
                break

    if type(h_zoom) is not int:
        error_flag = True

    if type(v_zoom) is not int:
        error_flag = True

    if type(crs) is not int:
        error_flag = True

    if type(needs_closed_checking) is not bool:
        error_flag = True

    if error_flag is True:
        raise SpatialIdError("INPUT_VALUE_ERROR")

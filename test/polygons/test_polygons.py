#!/bin/env python3
# -*- coding: utf-8 -*-

import pytest

import numpy as np
from skspatial.objects import Point, Plane, Line, Vector, Triangle

from SpatialId.common.object.enum import Point_Option
from SpatialId.common.exception import SpatialIdError
from SpatialId.common.object.point import Point as SpatialPoint
from SpatialId.common.object.point import Triangle as SpatialTriangle
from SpatialId.common.object.point import Projected_Triangle as SpatialProjectedTriangle
from SpatialId.common.object.point import Projected_Point
from SpatialId.shape.point import (
        get_point_on_spatial_id,
        convert_projected_point_list_to_point_list,
        convert_point_list_to_projected_point_list)
from SpatialId.shape.line import get_spatial_ids_on_line
from SpatialId.shape.polygons import (
        f_get_spatial_ids_on_polygons,
        get_spatial_ids_on_polygons,
        _get_spatial_ids_on_polygons,
        _valid_triangle_points,
        _valid_closed_polygons,
        _get_triangle_convert_rectangular_coordinate_crs,
        _get_point_object_on_triangle,
        _get_triangle_side_spatial_ids,
        _get_triangle_plane_spatial_ids,
        _get_cross_points_of_voxel_plane,
        _make_right_rotation_line,
        _make_left_rotation_line,
        _make_line_group,
        _is_inside,
        _grouping_triangle,
        _change_point_str,
        _count_edge_side,
        _search_judge_vertex,
        _count_vertex_side,
        _check_cross_one_edge,
        _check_cross_two_edges,
        _check_cross_vertex_and_edge,
        _check_cross_two_vertexes,
        _check_cross_one_vertex,
        _make_triangle_group,
        _get_inner_voxel,
        _get_share_triangle_index,
        _cross_check_edge_vertex,
        _get_cross_point_cordinate,
        _validate_args_type)

# CRSのデフォルト値のEPSGコード
PROJECTED_CRS = 3857
GEOGRAPHIC_CRS = 4326
MINIMA = 1e-10


@pytest.fixture
def rectangular_model_zoom10_3x3x3():
    """
    3x3x3のボクセル空間にある立方体のモデルデータ
    モデルを構成するポリゴン（SpatialTriangle）の配列、空間IDの配列を戻り値として返す。
    空間IDは水平、垂直方向精度が10で算出されたものを返す。
    モデルの所属する空間IDは"10/901/1/10/1" ～ "10/903/3/10/3"
    """
    # ポリゴン配列
    triangles = [
        SpatialTriangle(p1=SpatialPoint(
                lon=137.5, lat=85.0, alt=49152.0),
                p2=SpatialPoint(lon=137.0, lat=85.0, alt=49152.0),
                p3=SpatialPoint(lon=137.5, lat=84.94, alt=49152.0)),
        SpatialTriangle(
                p1=SpatialPoint(lon=137.0, lat=85.0, alt=49152.0),
                p2=SpatialPoint(lon=137.5, lat=84.94, alt=49152.0),
                p3=SpatialPoint(lon=137.0, lat=84.94, alt=49152.0)),
        SpatialTriangle(
                p1=SpatialPoint(lon=137.5, lat=85.0, alt=114688.0),
                p2=SpatialPoint(lon=137.0, lat=85.0, alt=114688.0),
                p3=SpatialPoint(lon=137.5, lat=84.94, alt=114688.0)),
        SpatialTriangle(
                p1=SpatialPoint(lon=137.0, lat=85.0, alt=114688.0),
                p2=SpatialPoint(lon=137.5, lat=84.94, alt=114688.0),
                p3=SpatialPoint(lon=137.0, lat=84.94, alt=114688.0)),
        SpatialTriangle(
                p1=SpatialPoint(lon=137.5, lat=85.0, alt=49152.0),
                p2=SpatialPoint(lon=137.5, lat=85.0, alt=114688.0),
                p3=SpatialPoint(lon=137.0, lat=85.0, alt=49152.0)),
        SpatialTriangle(
                p1=SpatialPoint(lon=137.5, lat=85.0, alt=114688.0),
                p2=SpatialPoint(lon=137.0, lat=85.0, alt=49152.0),
                p3=SpatialPoint(lon=137.0, lat=85.0, alt=114688.0)),
        SpatialTriangle(
                p1=SpatialPoint(lon=137.0, lat=85.0, alt=49152.0),
                p2=SpatialPoint(lon=137.0, lat=85.0, alt=114688.0),
                p3=SpatialPoint(lon=137.0, lat=84.94, alt=49152.0)),
        SpatialTriangle(
                p1=SpatialPoint(lon=137.0, lat=85.0, alt=114688.0),
                p2=SpatialPoint(lon=137.0, lat=84.94, alt=49152.0),
                p3=SpatialPoint(lon=137.0, lat=84.94, alt=114688.0)),
        SpatialTriangle(
                p1=SpatialPoint(lon=137.0, lat=84.94, alt=49152.0),
                p2=SpatialPoint(lon=137.0, lat=84.94, alt=114688.0),
                p3=SpatialPoint(lon=137.5, lat=84.94, alt=49152.0)),
        SpatialTriangle(
                p1=SpatialPoint(lon=137.0, lat=84.94, alt=114688.0),
                p2=SpatialPoint(lon=137.5, lat=84.94, alt=49152.0),
                p3=SpatialPoint(lon=137.5, lat=84.94, alt=114688.0)),
        SpatialTriangle(
                p1=SpatialPoint(lon=137.5, lat=84.94, alt=49152.0),
                p2=SpatialPoint(lon=137.5, lat=84.94, alt=114688.0),
                p3=SpatialPoint(lon=137.5, lat=85.0, alt=49152.0)),
        SpatialTriangle(
                p1=SpatialPoint(lon=137.5, lat=84.94, alt=114688.0),
                p2=SpatialPoint(lon=137.5, lat=85.0, alt=49152.0),
                p3=SpatialPoint(lon=137.5, lat=85.0, alt=114688.0))]

    # 空間ID配列
    spatial_id = [
        '10/901/1/10/1',
        '10/901/1/10/2',
        '10/901/1/10/3',
        '10/901/2/10/1',
        '10/901/2/10/2',
        '10/901/2/10/3',
        '10/901/3/10/1',
        '10/901/3/10/2',
        '10/901/3/10/3',
        '10/902/1/10/1',
        '10/902/1/10/2',
        '10/902/1/10/3',
        '10/902/2/10/1',
        '10/902/2/10/2',
        '10/902/2/10/3',
        '10/902/3/10/1',
        '10/902/3/10/2',
        '10/902/3/10/3',
        '10/903/1/10/1',
        '10/903/1/10/2',
        '10/903/1/10/3',
        '10/903/2/10/1',
        '10/903/2/10/2',
        '10/903/2/10/3',
        '10/903/3/10/1',
        '10/903/3/10/2',
        '10/903/3/10/3']

    return [triangles, spatial_id]

@pytest.fixture
def rectangular_model_zoom10_3x3x3_f():
    """
    3x3x3のボクセル空間にある立方体のモデルデータ
    モデルを構成するポリゴン（SpatialTriangle）の配列、空間IDの配列を戻り値として返す。
    空間IDは水平、垂直方向精度が10で算出されたものを返す。
    モデルの所属する空間IDは"10/901/1/10/1" ～ "10/903/3/10/3"
    """
    # ポリゴン配列
    triangles = [
        SpatialTriangle(p1=SpatialPoint(
                lon=137.5, lat=85.0, alt=49152.0),
                p2=SpatialPoint(lon=137.0, lat=85.0, alt=49152.0),
                p3=SpatialPoint(lon=137.5, lat=84.94, alt=49152.0)),
        SpatialTriangle(
                p1=SpatialPoint(lon=137.0, lat=85.0, alt=49152.0),
                p2=SpatialPoint(lon=137.5, lat=84.94, alt=49152.0),
                p3=SpatialPoint(lon=137.0, lat=84.94, alt=49152.0)),
        SpatialTriangle(
                p1=SpatialPoint(lon=137.5, lat=85.0, alt=114688.0),
                p2=SpatialPoint(lon=137.0, lat=85.0, alt=114688.0),
                p3=SpatialPoint(lon=137.5, lat=84.94, alt=114688.0)),
        SpatialTriangle(
                p1=SpatialPoint(lon=137.0, lat=85.0, alt=114688.0),
                p2=SpatialPoint(lon=137.5, lat=84.94, alt=114688.0),
                p3=SpatialPoint(lon=137.0, lat=84.94, alt=114688.0)),
        SpatialTriangle(
                p1=SpatialPoint(lon=137.5, lat=85.0, alt=49152.0),
                p2=SpatialPoint(lon=137.5, lat=85.0, alt=114688.0),
                p3=SpatialPoint(lon=137.0, lat=85.0, alt=49152.0)),
        SpatialTriangle(
                p1=SpatialPoint(lon=137.5, lat=85.0, alt=114688.0),
                p2=SpatialPoint(lon=137.0, lat=85.0, alt=49152.0),
                p3=SpatialPoint(lon=137.0, lat=85.0, alt=114688.0)),
        SpatialTriangle(
                p1=SpatialPoint(lon=137.0, lat=85.0, alt=49152.0),
                p2=SpatialPoint(lon=137.0, lat=85.0, alt=114688.0),
                p3=SpatialPoint(lon=137.0, lat=84.94, alt=49152.0)),
        SpatialTriangle(
                p1=SpatialPoint(lon=137.0, lat=85.0, alt=114688.0),
                p2=SpatialPoint(lon=137.0, lat=84.94, alt=49152.0),
                p3=SpatialPoint(lon=137.0, lat=84.94, alt=114688.0)),
        SpatialTriangle(
                p1=SpatialPoint(lon=137.0, lat=84.94, alt=49152.0),
                p2=SpatialPoint(lon=137.0, lat=84.94, alt=114688.0),
                p3=SpatialPoint(lon=137.5, lat=84.94, alt=49152.0)),
        SpatialTriangle(
                p1=SpatialPoint(lon=137.0, lat=84.94, alt=114688.0),
                p2=SpatialPoint(lon=137.5, lat=84.94, alt=49152.0),
                p3=SpatialPoint(lon=137.5, lat=84.94, alt=114688.0)),
        SpatialTriangle(
                p1=SpatialPoint(lon=137.5, lat=84.94, alt=49152.0),
                p2=SpatialPoint(lon=137.5, lat=84.94, alt=114688.0),
                p3=SpatialPoint(lon=137.5, lat=85.0, alt=49152.0)),
        SpatialTriangle(
                p1=SpatialPoint(lon=137.5, lat=84.94, alt=114688.0),
                p2=SpatialPoint(lon=137.5, lat=85.0, alt=49152.0),
                p3=SpatialPoint(lon=137.5, lat=85.0, alt=114688.0))]

    # 空間ID配列
    spatial_id = ['10/1/901/1', '10/2/901/1', '10/3/901/1', '10/1/901/2', '10/2/901/2', '10/3/901/2', '10/1/901/3', '10/2/901/3', '10/3/901/3', '10/1/902/1', '10/2/902/1', '10/3/902/1', '10/1/902/2', '10/2/902/2', '10/3/902/2', '10/1/902/3', '10/2/902/3', '10/3/902/3', '10/1/903/1', '10/2/903/1', '10/3/903/1', '10/1/903/2', '10/2/903/2', '10/3/903/2', '10/1/903/3', '10/2/903/3', '10/3/903/3']

    return [triangles, spatial_id]


@pytest.fixture
def rectangular_model_zoom10_3x3x4():
    """
    3x3x4のボクセル空間にある立方体のモデルデータ
    モデルを構成するポリゴン（SpatialTriangle）の配列、空間IDの配列を戻り値として返す。
    空間IDは水平、垂直方向精度が10で算出されたものを返す。
    モデルの所属する空間IDは"10/903/3/10/1" ～ "10/905/5/10/4"
    """
    # ポリゴン配列
    triangles = [
        SpatialTriangle(
                p1=SpatialPoint(lon=138.35, lat=84.935, alt=49152.0),
                p2=SpatialPoint(lon=137.64, lat=84.935, alt=49152.0),
                p3=SpatialPoint(lon=138.35, lat=84.885, alt=49152.0)),
        SpatialTriangle(
                p1=SpatialPoint(lon=137.64, lat=84.935, alt=49152.0),
                p2=SpatialPoint(lon=138.35, lat=84.885, alt=49152.0),
                p3=SpatialPoint(lon=137.64, lat=84.885, alt=49152.0)),
        SpatialTriangle(
                p1=SpatialPoint(lon=138.35, lat=84.935, alt=147456.0),
                p2=SpatialPoint(lon=137.64, lat=84.935, alt=147456.0),
                p3=SpatialPoint(lon=138.35, lat=84.885, alt=147456.0)),
        SpatialTriangle(
                p1=SpatialPoint(lon=137.64, lat=84.935, alt=147456.0),
                p2=SpatialPoint(lon=138.35, lat=84.885, alt=147456.0),
                p3=SpatialPoint(lon=137.64, lat=84.885, alt=147456.0)),
        SpatialTriangle(
                p1=SpatialPoint(lon=138.35, lat=84.935, alt=49152.0),
                p2=SpatialPoint(lon=138.35, lat=84.935, alt=147456.0),
                p3=SpatialPoint(lon=137.64, lat=84.935, alt=49152.0)),
        SpatialTriangle(
                p1=SpatialPoint(lon=138.35, lat=84.935, alt=147456.0),
                p2=SpatialPoint(lon=137.64, lat=84.935, alt=49152.0),
                p3=SpatialPoint(lon=137.64, lat=84.935, alt=147456.0)),
        SpatialTriangle(
                p1=SpatialPoint(lon=137.64, lat=84.935, alt=49152.0),
                p2=SpatialPoint(lon=137.64, lat=84.935, alt=147456.0),
                p3=SpatialPoint(lon=137.64, lat=84.885, alt=49152.0)),
        SpatialTriangle(
                p1=SpatialPoint(lon=137.64, lat=84.935, alt=147456.0),
                p2=SpatialPoint(lon=137.64, lat=84.885, alt=49152.0),
                p3=SpatialPoint(lon=137.64, lat=84.885, alt=147456.0)),
        SpatialTriangle(
                p1=SpatialPoint(lon=137.64, lat=84.885, alt=49152.0),
                p2=SpatialPoint(lon=137.64, lat=84.885, alt=147456.0),
                p3=SpatialPoint(lon=138.35, lat=84.885, alt=49152.0)),
        SpatialTriangle(
                p1=SpatialPoint(lon=137.64, lat=84.885, alt=147456.0),
                p2=SpatialPoint(lon=138.35, lat=84.885, alt=49152.0),
                p3=SpatialPoint(lon=138.35, lat=84.885, alt=147456.0)),
        SpatialTriangle(
                p1=SpatialPoint(lon=138.35, lat=84.885, alt=49152.0),
                p2=SpatialPoint(lon=138.35, lat=84.885, alt=147456.0),
                p3=SpatialPoint(lon=138.35, lat=84.935, alt=49152.0)),
        SpatialTriangle(
                p1=SpatialPoint(lon=138.35, lat=84.885, alt=147456.0),
                p2=SpatialPoint(lon=138.35, lat=84.935, alt=49152.0),
                p3=SpatialPoint(lon=138.35, lat=84.935, alt=147456.0))]

    # 空間ID配列
    spatial_id = [
        '10/903/3/10/1',
        '10/903/3/10/2',
        '10/903/3/10/3',
        '10/903/3/10/4',
        '10/904/3/10/1',
        '10/904/3/10/2',
        '10/904/3/10/3',
        '10/904/3/10/4',
        '10/905/3/10/1',
        '10/905/3/10/2',
        '10/905/3/10/3',
        '10/905/3/10/4',
        '10/903/4/10/1',
        '10/903/4/10/2',
        '10/903/4/10/3',
        '10/903/4/10/4',
        '10/904/4/10/1',
        '10/904/4/10/2',
        '10/904/4/10/3',
        '10/904/4/10/4',
        '10/905/4/10/1',
        '10/905/4/10/2',
        '10/905/4/10/3',
        '10/905/4/10/4',
        '10/903/5/10/1',
        '10/903/5/10/2',
        '10/903/5/10/3',
        '10/903/5/10/4',
        '10/904/5/10/1',
        '10/904/5/10/2',
        '10/904/5/10/3',
        '10/904/5/10/4',
        '10/905/5/10/1',
        '10/905/5/10/2',
        '10/905/5/10/3',
        '10/905/5/10/4']

    return [triangles, spatial_id]


def test_get_spatial_ids_on_polygons_01(rectangular_model_zoom10_3x3x3):
    """三角ポリゴン空間ID取得、除外対象ポリゴン無し
    + 試験詳細
      - 入力条件
        - 取得対象ポリゴンに閉塞したポリゴンを入力
        - 除外対象ポリゴンは空の配列
      - 確認内容
        - 空間IDが取得できること
    """
    # 試験データ定義
    # 水平/垂直方向精度
    h_zoom = 10
    v_zoom = 10

    # 取得対象ポリゴン
    barrier_triangles = rectangular_model_zoom10_3x3x3[0]

    # 期待値定義
    # 取得対象ポリゴンの空間ID
    param_assert = rectangular_model_zoom10_3x3x3[1]

    # 試験実施
    # ポリゴン空間ID取得
    param_return = get_spatial_ids_on_polygons(
            barrier_triangles,
            [],
            h_zoom,
            v_zoom,
            GEOGRAPHIC_CRS)

    # 空間IDの要素数が期待値と等しいことを確認
    assert len(param_assert) == len(param_return)

    # 空間IDの値が期待値と等しいことを確認
    for param in param_return:
        assert (param in param_assert) is True

def test_f_get_spatial_ids_on_polygons_01(rectangular_model_zoom10_3x3x3_f):
    """三角ポリゴン空間ID取得、除外対象ポリゴン無し
    + 試験詳細
      - 入力条件
        - 取得対象ポリゴンに閉塞したポリゴンを入力
        - 除外対象ポリゴンは空の配列
      - 確認内容
        - 空間IDが取得できること
    """
    # 試験データ定義
    # 水平/垂直方向精度
    h_zoom = 10
    v_zoom = 10

    # 取得対象ポリゴン
    barrier_triangles = rectangular_model_zoom10_3x3x3_f[0]

    # 期待値定義
    # 取得対象ポリゴンの空間ID
    param_assert = rectangular_model_zoom10_3x3x3_f[1]

    # 試験実施
    # ポリゴン空間ID取得
    param_return = f_get_spatial_ids_on_polygons(
            barrier_triangles,
            [],
            h_zoom,
            GEOGRAPHIC_CRS)

    # 空間IDの要素数が期待値と等しいことを確認
    assert len(param_assert) == len(param_return)

    # 空間IDの値が期待値と等しいことを確認
    for param in param_return:
        assert (param in param_assert) is True

def test_get_spatial_ids_on_polygons_02(
        rectangular_model_zoom10_3x3x3,
        rectangular_model_zoom10_3x3x4
):
    """三角ポリゴン空間ID取得、除外対象ポリゴン有り
    + 試験詳細
      - 入力条件
        - 取得対象ポリゴンに閉塞したポリゴンを入力
        - 除外対象ポリゴンは取得対象ポリゴンの一部に重なるポリゴン
      - 確認内容
        - 空間IDが取得できること
        - 除外対象ポリゴンの空間IDが含まれていないこと
    """
    # 試験データ定義
    # 水平/垂直方向精度
    h_zoom = 10
    v_zoom = 10

    # 取得対象ポリゴン
    barrier_triangles = rectangular_model_zoom10_3x3x3[0]

    # 除外対象ポリゴン
    space_triangles = rectangular_model_zoom10_3x3x4[0]

    # 期待値定義
    # 取得対象ポリゴンの空間ID
    param_assert = []

    for spatial_id in rectangular_model_zoom10_3x3x3[1]:
        if spatial_id not in rectangular_model_zoom10_3x3x4[1]:
            # 取得対象の空間IDが、除外対象の空間IDに含まれない場合期待値に追加
            param_assert.append(spatial_id)

    # 試験実施
    # ポリゴン空間ID取得
    param_return = get_spatial_ids_on_polygons(
            barrier_triangles,
            space_triangles,
            h_zoom,
            v_zoom,
            GEOGRAPHIC_CRS)

    # 空間IDの要素数が期待値と等しいことを確認
    assert len(param_assert) == len(param_return)

    # 空間IDの値が期待値と等しいことを確認
    for param in param_return:
        assert (param in param_assert) is True


def test_get_spatial_ids_on_polygons_03():
    """三角ポリゴン空間ID取得、取得対象ポリゴンが空
    + 試験詳細
      - 入力条件
        - 取得対象ポリゴンに空の配列を指定
      - 確認内容
        - 空のリストが返却されること
    """
    # 試験データ定義
    # 水平/垂直方向精度
    h_zoom = 10
    v_zoom = 10

    # 取得対象ポリゴン
    barrier_triangles = []

    # 期待値定義
    # 取得対象ポリゴンの空間ID
    param_assert = []

    # 試験実施
    # ポリゴン空間ID取得
    param_return = get_spatial_ids_on_polygons(
            barrier_triangles,
            [],
            h_zoom,
            v_zoom,
            GEOGRAPHIC_CRS)

    # 空間IDの要素数が期待値と等しいことを確認
    assert len(param_assert) == len(param_return)


def test_get_spatial_ids_on_polygons_04(rectangular_model_zoom10_3x3x3):
    """三角ポリゴン空間ID取得、入力したEPSGコードが投影座標
    + 試験詳細
      - 入力条件
        - EPSGコードに投影座標系を入力
      - 確認内容
        - 座標変換失敗の例外が発生すること
    """
    # 試験データ定義
    # 水平/垂直方向精度
    h_zoom = 10
    v_zoom = 10

    # 取得対象ポリゴン
    barrier_triangles = rectangular_model_zoom10_3x3x3[0]

    # 期待値定義
    # 例外取得
    exception_assert = SpatialIdError('OTHER_ERROR')

    # 試験実施
    # 例外発生確認
    with pytest.raises(SpatialIdError) as e:
        # ポリゴン空間ID取得
        get_spatial_ids_on_polygons(
                barrier_triangles,
                [],
                h_zoom,
                v_zoom,
                PROJECTED_CRS)

    # 例外メッセージ確認
    assert str(exception_assert) == str(e.value)


def test_get_spatial_ids_on_polygons_05(
        rectangular_model_zoom10_3x3x3,
        mocker
):
    """三角ポリゴン空間ID取得、入力したEPSGコードが投影座標
    + 試験詳細
       - 試験条件
         - 通常発生しない例外パターンのためモックを、使用して例外を発生させる
       - 確認内容
         - その他例外が発生すること
    """
    # 試験データ定義
    # 水平/垂直方向精度
    h_zoom = 10
    v_zoom = 10

    # 取得対象ポリゴン
    barrier_triangles = rectangular_model_zoom10_3x3x3[0]

    # 期待値定義
    # 例外取得
    exception_assert = SpatialIdError('OTHER_ERROR')

    mocker.patch(
            'SpatialId.shape.polygons._get_spatial_ids_on_polygons').\
        side_effect = Exception('予期せぬ例外が発生')

    # 試験実施
    # 例外発生確認
    with pytest.raises(SpatialIdError) as e:
        # ポリゴン空間ID取得
        get_spatial_ids_on_polygons(
                barrier_triangles,
                [],
                h_zoom,
                v_zoom,
                GEOGRAPHIC_CRS)

    # 例外メッセージ確認
    assert str(exception_assert) == str(e.value)


def test_get_spatial_ids_on_polygons_06(rectangular_model_zoom10_3x3x3):
    """三角ポリゴン空間ID取得、ポリゴン閉塞チェックOFF
    + 試験詳細
      - 入力条件
        - 閉塞チェックフラグOFF
      - 確認内容
        - 空間IDが取得できること
    """
    # 試験データ定義
    # 水平/垂直方向精度
    h_zoom = 10
    v_zoom = 10

    # 取得対象ポリゴン
    barrier_triangles = rectangular_model_zoom10_3x3x3[0]

    # 期待値定義
    # 取得対象ポリゴンの空間ID
    param_assert = rectangular_model_zoom10_3x3x3[1]

    # 試験実施
    # ポリゴン空間ID取得
    param_return = get_spatial_ids_on_polygons(
            barrier_triangles,
            [],
            h_zoom,
            v_zoom,
            GEOGRAPHIC_CRS,
            False)

    # 空間IDの要素数が期待値と等しいことを確認
    assert len(param_assert) == len(param_return)

    # 空間IDの値が期待値と等しいことを確認
    for param in param_return:
        assert (param in param_assert) is True


def test_get_spatial_ids_on_polygons_07(rectangular_model_zoom10_3x3x3):
    """三角ポリゴン空間ID取得失敗
    + 試験詳細
      - 入力条件
        - 精度に不正な値を指定
      - 確認内容
        - 入力引数チェックエラーが発生すること
    """
    # 試験データ定義
    # 水平/垂直方向精度
    h_zoom = '10'
    v_zoom = 10

    # 取得対象ポリゴン
    barrier_triangles = rectangular_model_zoom10_3x3x3[0]

    # 期待値定義
    # 例外取得
    exception_assert = SpatialIdError('INPUT_VALUE_ERROR')

    # 試験実施
    # 例外発生確認
    with pytest.raises(SpatialIdError) as e:
        # ポリゴン空間ID取得
        get_spatial_ids_on_polygons(
                barrier_triangles,
                [],
                h_zoom,
                v_zoom,
                PROJECTED_CRS)

    # 例外メッセージ確認
    assert str(exception_assert) == str(e.value)


def test__get_spatial_ids_on_polygons_01(rectangular_model_zoom10_3x3x3):
    """三角ポリゴン空間ID取得
    + 試験詳細
      - 入力条件
        - 取得対象ポリゴンに閉塞したポリゴンを入力
      - 確認内容
        - 空間IDが取得できること
    """
    # 試験データ定義
    # 水平/垂直方向精度
    h_zoom = 10
    v_zoom = 10

    # 取得対象ポリゴン
    triangles = rectangular_model_zoom10_3x3x3[0]

    # 期待値定義
    # 取得対象ポリゴンの空間ID
    param_assert = set(rectangular_model_zoom10_3x3x3[1])

    # 試験実施
    # ポリゴン空間ID取得
    param_return = _get_spatial_ids_on_polygons(
            triangles,
            h_zoom,
            v_zoom,
            GEOGRAPHIC_CRS)

    # 空間IDの要素数が期待値と等しいことを確認
    assert len(param_assert) == len(param_return)

    # 空間IDの値が期待値と等しいことを確認
    for param in param_return:
        assert (param in param_assert) is True


def test__get_spatial_ids_on_polygons_02():
    """三角ポリゴン空間ID取得
    + 試験詳細
      - 入力条件
        - 取得対象ポリゴンに閉塞したポリゴンを入力
        - 入力のcrsが投影座標かつ3857以外
      - 確認内容
        - 空間IDが取得できること
    """
    # 試験データ定義
    # 水平/垂直方向精度
    h_zoom = 10
    v_zoom = 10

    # 入力CRS
    crs = 6677

    # 取得対象ポリゴン(地理座標)
    triangles = [
        SpatialTriangle(
          SpatialPoint(140.349999, 36.935001, 12),
          SpatialPoint(140.350000, 36.935000, 12),
          SpatialPoint(140.350001, 36.935001, 12),
        ),
        SpatialTriangle(
          SpatialPoint(140.349999, 36.935001, 12),
          SpatialPoint(140.350000, 36.935001, 13),
          SpatialPoint(140.350001, 36.935001, 12),
        ),
        SpatialTriangle(
          SpatialPoint(140.350000, 36.935000, 12),
          SpatialPoint(140.350000, 36.935001, 13),
          SpatialPoint(140.350001, 36.935001, 12),
        ),
          SpatialTriangle(
          SpatialPoint(140.349999, 36.935001, 12),
          SpatialPoint(140.350000, 36.935001, 13),
          SpatialPoint(140.350000, 36.935000, 12),
        ),
    ]

    # 作業用の地理座標のリスト
    geographic_point_list = []
    # 試験対象メソッドに引数として渡す投影座標ポリゴンのリスト
    projected_triangles = []

    # 三角ポリゴンのリストの要素を投影座標に変換
    for triangle in triangles:
      geographic_point_01 = SpatialPoint(
        lon=triangle.p1.lon,
        lat=triangle.p1.lat,
        alt=triangle.p1.alt
      )
      geographic_point_02 = SpatialPoint(
        lon=triangle.p2.lon,
        lat=triangle.p2.lat,
        alt=triangle.p2.alt
      )
      geographic_point_03 = SpatialPoint(
        lon=triangle.p3.lon,
        lat=triangle.p3.lat,
        alt=triangle.p3.alt
      )
      # 三角ポリゴンの各頂点の座標を地理座標リストに追加
      geographic_point_list.append(geographic_point_01)
      geographic_point_list.append(geographic_point_02)
      geographic_point_list.append(geographic_point_03)
      # 地理座標から投影座標に変換
      projected_points = \
          convert_point_list_to_projected_point_list(
                  geographic_point_list, crs, GEOGRAPHIC_CRS)
      # 作業用の地理座標リストの要素を削除
      geographic_point_list.clear()
      # ポリゴン(投影座標)を作成し、リストへ追加
      projected_triangle = SpatialProjectedTriangle(
        projected_points[0],
        projected_points[1],
        projected_points[2]
      )
      projected_triangles.append(projected_triangle)

    # 試験実施
    # ポリゴン空間ID取得
    param_return = _get_spatial_ids_on_polygons(
            projected_triangles,
            h_zoom,
            v_zoom,
            crs)

    # 入力から空間IDの予測が難しいため空間IDが取得できることのみチェック
    assert len(param_return) != 0


def test__validate_args_type_01():
    """入力チェックメソッド、入力チェックOK
    + 試験詳細
      - 入力条件
        - barrier_triangles: list[SpatialTriangle]
        - space_triangles: list[SpatialTriangle]
        - h_zoom: int
        - v_zoom: int
        - crs: int
        - needs_closed_checking: bool
      - 確認内容
        - 例外が発生しないこと
    """
    # 試験データ定義
    # 三角ポリゴンの頂点を定義
    triangle_01 = SpatialTriangle(
            SpatialPoint(140.0, 39.0, 10.0),
            SpatialPoint(141.0, 40.0, 11.0),
            SpatialPoint(142.0, 41.0, 12.0))
    triangle_02 = SpatialTriangle(
            SpatialPoint(130.0, 49.0, 20.0),
            SpatialPoint(131.0, 50.0, 21.0),
            SpatialPoint(132.0, 51.0, 22.0))

    # 取得対象、除外対象の三角ポリゴン配列を定義
    barrier_triangles = [triangle_01, triangle_02]
    space_triangles = [triangle_01, triangle_02]

    # 試験実施
    param_return = _validate_args_type(
            barrier_triangles,
            space_triangles,
            1,
            1,
            4326,
            True)

    assert None is param_return


def test__validate_args_type_02():
    """入力チェックメソッド、入力チェックNG、barrier_trianglesの型不正
    + 試験詳細
      - 入力条件
        - barrier_triangles: int
      - 確認内容
        - 入力値チェックエラーの例外が発生すること
    """
    # 試験データ定義
    # 三角ポリゴンの頂点を定義
    triangle_01 = SpatialTriangle(
            SpatialPoint(140.0, 39.0, 10.0),
            SpatialPoint(141.0, 40.0, 11.0),
            SpatialPoint(142.0, 41.0, 12.0))
    triangle_02 = SpatialTriangle(
            SpatialPoint(130.0, 49.0, 20.0),
            SpatialPoint(131.0, 50.0, 21.0),
            SpatialPoint(132.0, 51.0, 22.0))

    # 取得対象、除外対象の三角ポリゴン配列を定義
    # 第1引数用の変数にはint型を格納
    barrier_triangles = 10
    space_triangles = [triangle_01, triangle_02]

    # 期待値定義
    # 例外取得
    exception_assert = SpatialIdError('INPUT_VALUE_ERROR')

    # 試験実施
    # 例外発生確認
    with pytest.raises(SpatialIdError) as e:
        _validate_args_type(
            barrier_triangles,
            space_triangles,
            1,
            1,
            4326,
            True)

    assert str(exception_assert) == str(e.value)


def test__validate_args_type_03():
    """入力チェックメソッド、入力チェックNG、barrier_trianglesの配列内の型不正
    + 試験詳細
      - 入力条件
        - barrier_triangles: list[int]
      - 確認内容
        - 入力値チェックエラーの例外が発生すること
    """
    # 試験データ定義
    # 三角ポリゴンの頂点を定義
    triangle_01 = SpatialTriangle(
            SpatialPoint(140.0, 39.0, 10.0),
            SpatialPoint(141.0, 40.0, 11.0),
            SpatialPoint(142.0, 41.0, 12.0))
    triangle_02 = SpatialTriangle(
            SpatialPoint(130.0, 49.0, 20.0),
            SpatialPoint(131.0, 50.0, 21.0),
            SpatialPoint(132.0, 51.0, 22.0))

    # 取得対象、除外対象の三角ポリゴン配列を定義
    # 第1引数用の変数にはint型を格納
    barrier_triangles = [10, 10]
    space_triangles = [triangle_01, triangle_02]

    # 期待値定義
    # 例外取得
    exception_assert = SpatialIdError('INPUT_VALUE_ERROR')

    # 試験実施
    # 例外発生確認
    with pytest.raises(SpatialIdError) as e:
        _validate_args_type(
            barrier_triangles,
            space_triangles,
            1,
            1,
            4326,
            True)

    assert str(exception_assert) == str(e.value)


def test__validate_args_type_04():
    """入力チェックメソッド、入力チェックNG、space_trianglesの型不正
    + 試験詳細
      - 入力条件
        - space_triangles: int
      - 確認内容
        - 入力値チェックエラーの例外が発生すること
    """
    # 試験データ定義
    # 三角ポリゴンの頂点を定義
    triangle_01 = SpatialTriangle(
            SpatialPoint(140.0, 39.0, 10.0),
            SpatialPoint(141.0, 40.0, 11.0),
            SpatialPoint(142.0, 41.0, 12.0))
    triangle_02 = SpatialTriangle(
            SpatialPoint(130.0, 49.0, 20.0),
            SpatialPoint(131.0, 50.0, 21.0),
            SpatialPoint(132.0, 51.0, 22.0))

    # 取得対象、除外対象の三角ポリゴン配列を定義
    # 第1引数用の変数にはint型を格納
    barrier_triangles = [triangle_01, triangle_02]
    space_triangles = 10

    # 期待値定義
    # 例外取得
    exception_assert = SpatialIdError('INPUT_VALUE_ERROR')

    # 試験実施
    # 例外発生確認
    with pytest.raises(SpatialIdError) as e:
        _validate_args_type(
            barrier_triangles,
            space_triangles,
            1,
            1,
            4326,
            True)

    assert str(exception_assert) == str(e.value)


def test__validate_args_type_05():
    """入力チェックメソッド、入力チェックNG、space_trianglesの配列内の型不正
    + 試験詳細
      - 入力条件
        - space_triangles: list[int]
      - 確認内容
        - 入力値チェックエラーの例外が発生すること
    """
    # 試験データ定義
    # 三角ポリゴンの頂点を定義
    triangle_01 = SpatialTriangle(
            SpatialPoint(140.0, 39.0, 10.0),
            SpatialPoint(141.0, 40.0, 11.0),
            SpatialPoint(142.0, 41.0, 12.0))
    triangle_02 = SpatialTriangle(
            SpatialPoint(130.0, 49.0, 20.0),
            SpatialPoint(131.0, 50.0, 21.0),
            SpatialPoint(132.0, 51.0, 22.0))

    # 取得対象、除外対象の三角ポリゴン配列を定義
    # 第1引数用の変数にはint型を格納
    barrier_triangles = [triangle_01, triangle_02]
    space_triangles = [10, 10]

    # 期待値定義
    # 例外取得
    exception_assert = SpatialIdError('INPUT_VALUE_ERROR')

    # 試験実施
    # 例外発生確認
    with pytest.raises(SpatialIdError) as e:
        _validate_args_type(
            barrier_triangles,
            space_triangles,
            1,
            1,
            4326,
            True)

    assert str(exception_assert) == str(e.value)


def test__validate_args_type_06():
    """入力チェックメソッド、入力チェックNG、h_zoomの型不正
    + 試験詳細
      - 入力条件
        - h_zoom: str
      - 確認内容
        - 入力値チェックエラーの例外が発生すること
    """
    # 試験データ定義
    # 三角ポリゴンの頂点を定義
    triangle_01 = SpatialTriangle(
            SpatialPoint(140.0, 39.0, 10.0),
            SpatialPoint(141.0, 40.0, 11.0),
            SpatialPoint(142.0, 41.0, 12.0))
    triangle_02 = SpatialTriangle(
            SpatialPoint(130.0, 49.0, 20.0),
            SpatialPoint(131.0, 50.0, 21.0),
            SpatialPoint(132.0, 51.0, 22.0))

    # 取得対象、除外対象の三角ポリゴン配列を定義
    # 第1引数用の変数にはint型を格納
    barrier_triangles = [triangle_01, triangle_02]
    space_triangles = [triangle_01, triangle_02]

    # 期待値定義
    # 例外取得
    exception_assert = SpatialIdError('INPUT_VALUE_ERROR')

    # 試験実施
    # 例外発生確認
    with pytest.raises(SpatialIdError) as e:
        _validate_args_type(
            barrier_triangles,
            space_triangles,
            "1",
            1,
            4326,
            True)

    assert str(exception_assert) == str(e.value)


def test__validate_args_type_07():
    """入力チェックメソッド、入力チェックNG、v_zoomの型不正
    + 試験詳細
      - 入力条件
        - v_zoom: str
      - 確認内容
        - 入力値チェックエラーの例外が発生すること
    """
    # 試験データ定義
    # 三角ポリゴンの頂点を定義
    triangle_01 = SpatialTriangle(
            SpatialPoint(140.0, 39.0, 10.0),
            SpatialPoint(141.0, 40.0, 11.0),
            SpatialPoint(142.0, 41.0, 12.0))
    triangle_02 = SpatialTriangle(
            SpatialPoint(130.0, 49.0, 20.0),
            SpatialPoint(131.0, 50.0, 21.0),
            SpatialPoint(132.0, 51.0, 22.0))

    # 取得対象、除外対象の三角ポリゴン配列を定義
    # 第1引数用の変数にはint型を格納
    barrier_triangles = [triangle_01, triangle_02]
    space_triangles = [triangle_01, triangle_02]

    # 期待値定義
    # 例外取得
    exception_assert = SpatialIdError('INPUT_VALUE_ERROR')

    # 試験実施
    # 例外発生確認
    with pytest.raises(SpatialIdError) as e:
        _validate_args_type(
            barrier_triangles,
            space_triangles,
            1,
            "1",
            4326,
            True)

    assert str(exception_assert) == str(e.value)


def test__validate_args_type_08():
    """入力チェックメソッド、入力チェックNG、crsの型不正
    + 試験詳細
      - 入力条件
        - crs: str
      - 確認内容
        - 入力値チェックエラーの例外が発生すること
    """
    # 試験データ定義
    # 三角ポリゴンの頂点を定義
    triangle_01 = SpatialTriangle(
            SpatialPoint(140.0, 39.0, 10.0),
            SpatialPoint(141.0, 40.0, 11.0),
            SpatialPoint(142.0, 41.0, 12.0))
    triangle_02 = SpatialTriangle(
            SpatialPoint(130.0, 49.0, 20.0),
            SpatialPoint(131.0, 50.0, 21.0),
            SpatialPoint(132.0, 51.0, 22.0))

    # 取得対象、除外対象の三角ポリゴン配列を定義
    # 第1引数用の変数にはint型を格納
    barrier_triangles = [triangle_01, triangle_02]
    space_triangles = [triangle_01, triangle_02]

    # 期待値定義
    # 例外取得
    exception_assert = SpatialIdError('INPUT_VALUE_ERROR')

    # 試験実施
    # 例外発生確認
    with pytest.raises(SpatialIdError) as e:
        _validate_args_type(
            barrier_triangles,
            space_triangles,
            1,
            1,
            "4326",
            True)

    assert str(exception_assert) == str(e.value)


def test__validate_args_type_09():
    """入力チェックメソッド、入力チェックNG、needs_closed_checkingの型不正
    + 試験詳細
      - 入力条件
        - needs_closed_checking: str
      - 確認内容
        - 入力値チェックエラーの例外が発生すること
    """
    # 試験データ定義
    # 三角ポリゴンの頂点を定義
    triangle_01 = SpatialTriangle(
            SpatialPoint(140.0, 39.0, 10.0),
            SpatialPoint(141.0, 40.0, 11.0),
            SpatialPoint(142.0, 41.0, 12.0))
    triangle_02 = SpatialTriangle(
            SpatialPoint(130.0, 49.0, 20.0),
            SpatialPoint(131.0, 50.0, 21.0),
            SpatialPoint(132.0, 51.0, 22.0))

    # 取得対象、除外対象の三角ポリゴン配列を定義
    # 第1引数用の変数にはint型を格納
    barrier_triangles = [triangle_01, triangle_02]
    space_triangles = [triangle_01, triangle_02]

    # 期待値定義
    # 例外取得
    exception_assert = SpatialIdError('INPUT_VALUE_ERROR')

    # 試験実施
    # 例外発生確認
    with pytest.raises(SpatialIdError) as e:
        _validate_args_type(
            barrier_triangles,
            space_triangles,
            1,
            1,
            4326,
            "True")

    assert str(exception_assert) == str(e.value)


def test__change_point_str_01():
    """Pointに格納された座標を文字列に変換
    """
    # 試験用データ定義
    point = [178.051685, 42.060433, 10.5]

    # 期待値定義
    param_assert = '_'.join(
      ["{:.10f}".format(point[0]),
       "{:.10f}".format(point[1]),
       "{:.10f}".format(point[2])])

    # 試験実施
    # 戻り値取得
    param_return = _change_point_str(Point(point))

    # XYZ座標が'_'で結合された文字列が返却されること確認
    assert param_assert == param_return


def test__get_point_object_on_triangle_01():
    """三角ポリゴンの頂点からPointオブジェクトを生成
    """
    # 試験用データ定義
    point_01 = Projected_Point(0.0, 0.0, lon=139.01, lat=45.01, alt=100)
    point_02 = Projected_Point(0.0, 0.0, lon=139.11, lat=45.11, alt=200)
    point_03 = Projected_Point(0.0, 0.0, lon=139.21, lat=45.21, alt=300)

    triangle = SpatialProjectedTriangle(point_01, point_02, point_03)
    rectanglar_triangle = _get_triangle_convert_rectangular_coordinate_crs(
      triangle, 4326)
    # 期待値定義
    param_assert = [point_01, point_02, point_03]

    # 試験実施
    # 戻り値取得
    param_return = _get_point_object_on_triangle(triangle,rectanglar_triangle)

    # 戻り値確認
    
    for cnt in range(len(param_return)):
        assert param_assert[cnt].lon == param_return[cnt].lon
        assert param_assert[cnt].lat == param_return[cnt].lat
        assert param_assert[cnt].alt == param_return[cnt].alt


def test__get_triangle_convert_rectangular_coordinate_crs_01():
    """三角ポリゴンの座標系を空間ID利用の座標系(WGS84)から座標計算で利用する平面直角座標系へ変換
    + 試験詳細
      - 入力条件
        - WGS84の座標を持つ三角ポリゴン
        - 参照座標系にはWGS84(4326)を指定
      - 確認内容
        - 平面直角座標を持つ三角ポリゴンが取得できること
    """
    # 試験用データ定義
    point_01 = SpatialPoint(139.01, 45.01, 100)
    point_02 = SpatialPoint(139.11, 45.11, 200)
    point_03 = SpatialPoint(139.21, 45.21, 300)

    # 期待値定義
    geographic_point_list = [point_01, point_02, point_03]

    projected_point_list = \
        convert_point_list_to_projected_point_list(
                geographic_point_list, PROJECTED_CRS, GEOGRAPHIC_CRS)

    point_a = projected_point_list[0]
    point_b = projected_point_list[1]
    point_c = projected_point_list[2]

    param_assert = Triangle(
            Point([point_a.x, point_a.y, point_a.alt]),
            Point([point_b.x, point_b.y, point_b.alt]),
            Point([point_c.x, point_c.y, point_c.alt]))

    # 試験実施
    # 戻り値取得
    geographic_triangle = SpatialTriangle(
            point_01,
            point_02,
            point_03)

    param_return = \
        _get_triangle_convert_rectangular_coordinate_crs(
                geographic_triangle, GEOGRAPHIC_CRS)

    # 戻り値確認
    assert (param_assert.point_a == param_return.point_a).all()
    assert (param_assert.point_b == param_return.point_b).all()
    assert (param_assert.point_c == param_return.point_c).all()


def test__get_triangle_convert_rectangular_coordinate_crs_02():
    """三角ポリゴンの座標系をユーザ指定の地理座標系から座標計算で利用する平面直角座標系へ変換
    + 試験詳細
      - 入力条件
        - 参照座標系JGD2000の座標を持つ三角ポリゴン
        - 参照座標系にはJGD2000(4612)を指定
      - 確認内容
        - 平面直角座標を持つ三角ポリゴンが取得できること
    """
    # 試験用データ定義
    point_01 = SpatialPoint(139.01, 45.01, 100)
    point_02 = SpatialPoint(139.11, 45.11, 200)
    point_03 = SpatialPoint(139.21, 45.21, 300)

    input_crs = 4612

    # 期待値定義
    geographic_point_list = [point_01, point_02, point_03]

    projected_point_list = \
        convert_point_list_to_projected_point_list(
                geographic_point_list, PROJECTED_CRS, input_crs)

    point_a = projected_point_list[0]
    point_b = projected_point_list[1]
    point_c = projected_point_list[2]

    param_assert = Triangle(
            Point([point_a.x, point_a.y, point_a.alt]),
            Point([point_b.x, point_b.y, point_b.alt]),
            Point([point_c.x, point_c.y, point_c.alt]))

    # 試験実施
    # 戻り値取得
    geographic_triangle = SpatialTriangle(
            point_01,
            point_02,
            point_03)

    param_return = \
        _get_triangle_convert_rectangular_coordinate_crs(
                geographic_triangle, input_crs)

    # 戻り値確認
    assert (param_assert.point_a == param_return.point_a).all()
    assert (param_assert.point_b == param_return.point_b).all()
    assert (param_assert.point_c == param_return.point_c).all()


def test__valid_triangle_points_01():
    """三角ポリゴンの面形成チェック、面を形成する三角ポリゴン
    + 試験詳細
      - 入力条件
        - 面を形成する三角ポリゴン
      - 確認内容
        - チェックOKとなり例外は発生しないこと
    """
    # 試験データ定義
    point_01 = SpatialPoint(139.01, 45.01, 100)
    point_02 = SpatialPoint(139.11, 45.11, 200)
    point_03 = SpatialPoint(139.21, 45.21, 301)

    point_04 = SpatialPoint(140.21, 45.21, 301)
    point_05 = SpatialPoint(141.21, 45.21, 301)
    point_06 = SpatialPoint(142.21, 45.21, 311)

    triangle_01 = SpatialTriangle(point_01, point_02, point_03)
    triangle_02 = SpatialTriangle(point_04, point_05, point_06)

    triangles = [triangle_01, triangle_02]

    # 試験実施
    # 例外が発生しないことを確認
    _valid_triangle_points(triangles)


def test__valid_triangle_points_02():
    """三角ポリゴンの面形成チェック、面を形成しない三角ポリゴン
    + 試験詳細
      - 入力条件
        - 三頂点が同一直線上に存在するポリゴン
      - 確認内容
        - チェックNGとなり例外が発生すること
    """
    # 試験データ定義
    point_01 = SpatialPoint(139.01, 45.01, 100)
    point_02 = SpatialPoint(139.11, 45.11, 200)
    point_03 = SpatialPoint(139.21, 45.21, 301)

    point_04 = SpatialPoint(140.21, 45.21, 301)
    point_05 = SpatialPoint(141.21, 45.21, 301)
    point_06 = SpatialPoint(142.21, 45.21, 301)

    triangle_01 = SpatialTriangle(point_01, point_02, point_03)
    triangle_02 = SpatialTriangle(point_04, point_05, point_06)

    triangles = [triangle_01, triangle_02]

    # 期待値定義
    # 例外取得
    exception_assert = SpatialIdError('POLYGON_POINT_COLLINEAR')

    # 試験実施
    # 例外発生確認
    with pytest.raises(SpatialIdError) as e:
        _valid_triangle_points(triangles)

    assert str(exception_assert) == str(e.value)


def test__valid_triangle_points_03():
    """三角ポリゴンの面形成チェック、面を形成しない三角ポリゴン
    + 試験詳細
      - 入力条件
        - 三角ポリゴンオブジェクトの生成に成功
        - 三頂点が同一直線上に存在するポリゴン
      - 確認内容
        - 直交ベクトルのノルムチェックでNGとなり例外が発生すること
    """
    # 試験データ定義
    point_01 = SpatialPoint(139.01, 45.01, 100)
    point_02 = SpatialPoint(139.11, 45.11, 200)
    point_03 = SpatialPoint(139.21, 45.21, 301)

    point_04 = SpatialPoint(140.21, 45.21, 301.00000000001)
    point_05 = SpatialPoint(141.21, 45.21, 301)
    point_06 = SpatialPoint(142.21, 45.21, 301)

    triangle_01 = SpatialTriangle(point_01, point_02, point_03)
    triangle_02 = SpatialTriangle(point_04, point_05, point_06)

    triangles = [triangle_01, triangle_02]

    # 期待値定義
    # 例外取得
    exception_assert = SpatialIdError('POLYGON_POINT_COLLINEAR')

    # 試験実施
    # 例外発生確認
    with pytest.raises(SpatialIdError) as e:
        _valid_triangle_points(triangles)

    assert str(exception_assert) == str(e.value)


def test__get_triangle_side_spatial_ids_01():
    """三角ポリゴンの辺の空間ID取得、座標系にデフォルトの地理座標系(WGS84)を指定
    + 試験詳細
      - 入力条件
        - 地理座標系(WGS84)の座標を格納した三角ポリゴンを引数に指定
      - 確認内容
        - 辺の空間IDが返却されること
    """
    # 試験データ定義
    point_01 = Projected_Point(
      15474522.4, 5623095.92, lon=139.01, lat=45.01, alt=100.0)
    point_02 = Projected_Point(
      15596973.9, 5638855.39, lon=140.11, lat=45.11, alt=200.0)
    point_03 = Projected_Point(
      15719425.3, 5654642.50, lon=141.21, lat=45.21, alt=301.0)
    triangle_01 = SpatialProjectedTriangle(point_01, point_02, point_03)
    h_zoom = 10
    v_zoom = 10

    # 期待値定義
    param_assert = set()
    param_assert |= set(get_spatial_ids_on_line(
            point_01, point_02, h_zoom, v_zoom, GEOGRAPHIC_CRS))
    param_assert |= set(get_spatial_ids_on_line(
            point_02, point_03, h_zoom, v_zoom, GEOGRAPHIC_CRS))
    param_assert |= set(get_spatial_ids_on_line(
            point_03, point_01, h_zoom, v_zoom, GEOGRAPHIC_CRS))

    rectanglar_triangle = _get_triangle_convert_rectangular_coordinate_crs(
      triangle_01, GEOGRAPHIC_CRS)
    # 試験実施
    # 戻り値取得
    param_return = _get_triangle_side_spatial_ids(
            triangle_01, rectanglar_triangle, h_zoom, v_zoom, GEOGRAPHIC_CRS)

    assert len(list(param_assert)) == len(param_return)

    for param in param_return:
        assert True == (param in param_assert)


def test__get_triangle_side_spatial_ids_02():
    """三角ポリゴンの辺の空間ID取得、座標系にユーザ入力の地理座標系を指定
    + 試験詳細
      - 入力条件
        - 地理座標系(JGD2000)の座標を格納した三角ポリゴンを引数に指定
      - 確認内容
        - 辺の空間IDが返却されること
    """
    # 試験データ定義
    point_01 = Projected_Point(
      15474522.4, 5623095.92, lon=139.01, lat=45.01, alt=100.0)
    point_02 = Projected_Point(
      15596973.9, 5638855.39, lon=140.11, lat=45.11, alt=200.0)
    point_03 = Projected_Point(
      15719425.3, 5654642.50, lon=141.21, lat=45.21, alt=301.0)

    triangle_01 = SpatialProjectedTriangle(point_01, point_02, point_03)
    h_zoom = 10
    v_zoom = 10
    input_crs = 4612

    rectanglar_triangle = _get_triangle_convert_rectangular_coordinate_crs(
      triangle_01, 4612)

    # 期待値定義
    param_assert = set()
    param_assert |= set(get_spatial_ids_on_line(
            point_01, point_02, h_zoom, v_zoom, input_crs))
    param_assert |= set(get_spatial_ids_on_line(
            point_02, point_03, h_zoom, v_zoom, input_crs))
    param_assert |= set(get_spatial_ids_on_line(
            point_03, point_01, h_zoom, v_zoom, input_crs))

    # 試験実施
    # 戻り値取得
    param_return = _get_triangle_side_spatial_ids(
            triangle_01, rectanglar_triangle, h_zoom, v_zoom, input_crs)

    assert len(list(param_assert)) == len(param_return)

    for param in param_return:
        assert True == (param in param_assert)


def test__get_cross_points_of_voxel_plane_01():
    """三角ポリゴンと境界面の交点取得、線分AB, ACと交点あり
    + 試験詳細
      - 入力条件
        - 三角ポリゴンの頂点をA, B, Cとしたとき、AB, ACと交差する境界面を入力
      - 確認内容
        - 境界面との交点の空間IDが取得できること
    """
    # 試験データ
    # 三角ポリゴン頂点座標
    point_a = Point([0.0, 0.0, 0.0])
    point_b = Point([300.0, 600.0, 0.0])
    point_c = Point([600.0, 0.0, 0.0])

    # 水平/垂直方向精度
    h_zoom = 20
    v_zoom = 20

    # 三角ポリゴン
    triangle = Triangle(point_a, point_b, point_c)

    # 衝突判定面
    cross_plane = Plane(point=(100, 0.0, 0.0), normal=Vector([1, 0, 0]))

    # 期待値定義
    # 境界面交点
    projected_cross_point = [
            Projected_Point(x=100.0, y=0.0, alt=0.0),
            Projected_Point(x=100.0, y=200.0, alt=0.0)]

    # 境界面交点をXYに変換
    geographic_cross_point = \
        convert_projected_point_list_to_point_list(
                projected_cross_point, PROJECTED_CRS)
    projected_cross_point[0].lon = geographic_cross_point[0].lon
    projected_cross_point[1].lon = geographic_cross_point[1].lon
    projected_cross_point[0].lat = geographic_cross_point[0].lat
    projected_cross_point[1].lat = geographic_cross_point[1].lat

    param_assert = \
        set(get_spatial_ids_on_line(
                projected_cross_point[0],
                projected_cross_point[1],
                h_zoom,
                v_zoom,
                GEOGRAPHIC_CRS))

    # 試験実施
    # 戻り値取得
    param_return = _get_cross_points_of_voxel_plane(
            cross_plane, triangle, h_zoom, v_zoom, GEOGRAPHIC_CRS)

    # 戻り値確認
    assert param_assert == param_return


def test__get_cross_points_of_voxel_plane_02():
    """三角ポリゴンと境界面の交点取得、線分AB, BCと交点あり
    + 試験詳細
      - 入力条件
        - 三角ポリゴンの頂点をA, B, Cとしたとき、AB, BCと交差する境界面を入力
      - 確認内容
        - 境界面との交点の空間IDが取得できること
    """
    # 試験データ
    # 三角ポリゴン頂点座標
    point_a = Point([0.0, 0.0, 0.0])
    point_b = Point([300.0, 600.0, 0.0])
    point_c = Point([600.0, 0.0, 0.0])

    # 水平/垂直方向精度
    h_zoom = 20
    v_zoom = 20

    # 三角ポリゴン
    triangle = Triangle(point_a, point_b, point_c)

    # 衝突判定面
    cross_plane = Plane(point=(400, 0.0, 0.0), normal=Vector([1, 0, 0]))

    # 期待値定義
    # 境界面交点
    projected_cross_point = [
            Projected_Point(x=400.0, y=0.0, alt=0.0),
            Projected_Point(x=400.0, y=400.0, alt=0.0)]

    # 境界面交点を緯度経度に変換
    geographic_cross_point = \
        convert_projected_point_list_to_point_list(
                projected_cross_point, PROJECTED_CRS)
    projected_cross_point[0].lon = geographic_cross_point[0].lon
    projected_cross_point[1].lon = geographic_cross_point[1].lon
    projected_cross_point[0].lat = geographic_cross_point[0].lat
    projected_cross_point[1].lat = geographic_cross_point[1].lat

    param_assert = \
        set(get_spatial_ids_on_line(
                projected_cross_point[0],
                projected_cross_point[1],
                h_zoom,
                v_zoom,
                GEOGRAPHIC_CRS))

    # 試験実施
    # 戻り値取得
    param_return = _get_cross_points_of_voxel_plane(
            cross_plane, triangle, h_zoom, v_zoom, GEOGRAPHIC_CRS)

    # 戻り値確認
    assert param_assert == param_return


def test__get_cross_points_of_voxel_plane_03():
    """三角ポリゴンと境界面の交点取得、線分AC, BCと交点あり
    + 試験詳細
      - 入力条件
        - 三角ポリゴンの頂点をA, B, Cとしたとき、AC, BCと交差する境界面を入力
      - 確認内容
        - 境界面との交点の空間IDが取得できること
    """
    # 試験データ
    # 三角ポリゴン頂点座標
    point_a = Point([0.0, 0.0, 0.0])
    point_b = Point([300.0, 600.0, 0.0])
    point_c = Point([600.0, 0.0, 0.0])

    # 水平/垂直方向精度
    h_zoom = 20
    v_zoom = 20

    # 三角ポリゴン
    triangle = Triangle(point_a, point_b, point_c)

    # 衝突判定面
    cross_plane = Plane(point=(0.0, 200.0, 0.0), normal=Vector([0, 1, 0]))

    # 期待値定義
    # 境界面交点
    projected_cross_point = [
            Projected_Point(x=100.0, y=200.0, alt=0.0),
            Projected_Point(x=500.0, y=200.0, alt=0.0)]

    # 境界面交点を緯度経度に変換
    geographic_cross_point = \
        convert_projected_point_list_to_point_list(
                projected_cross_point, PROJECTED_CRS)

    projected_cross_point[0].lon = geographic_cross_point[0].lon
    projected_cross_point[1].lon = geographic_cross_point[1].lon
    projected_cross_point[0].lat = geographic_cross_point[0].lat
    projected_cross_point[1].lat = geographic_cross_point[1].lat

    param_assert = \
        set(get_spatial_ids_on_line(
                projected_cross_point[0],
                projected_cross_point[1],
                h_zoom,
                v_zoom,
                GEOGRAPHIC_CRS))

    # 試験実施
    # 戻り値取得
    param_return = _get_cross_points_of_voxel_plane(
            cross_plane, triangle, h_zoom, v_zoom, GEOGRAPHIC_CRS)

    # 戻り値確認
    assert param_assert == param_return


def test__get_cross_points_of_voxel_plane_04():
    """三角ポリゴンと境界面の交点取得、境界面との衝突無し
    + 試験詳細
      - 入力条件
        - 三角ポリゴンのいずれの辺とも衝突しない境界面を入力
      - 確認内容
        - 空の集合が返却されること
    """
    # 試験データ
    # 三角ポリゴン頂点座標
    point_a = Point([0.0, 0.0, 0.0])
    point_b = Point([300.0, 600.0, 0.0])
    point_c = Point([600.0, 0.0, 0.0])

    # 水平/垂直方向精度
    h_zoom = 20
    v_zoom = 20

    # 三角ポリゴン
    triangle = Triangle(point_a, point_b, point_c)

    # 衝突判定面
    cross_plane = Plane(point=(601, 0.0, 0.0), normal=Vector([1, 0, 0]))

    # 期待値定義
    param_assert = set()

    # 試験実施
    # 戻り値取得
    param_return = _get_cross_points_of_voxel_plane(
            cross_plane, triangle, h_zoom, v_zoom, GEOGRAPHIC_CRS)

    # 戻り値確認
    assert param_assert == param_return


def test__get_cross_points_of_voxel_plane_05():
    """三角ポリゴンと境界面の交点取得、頂点Aと交差する境界面
    + 試験詳細
      - 入力条件
        - 三角ポリゴンの頂点Aと交差する境界面を入力
      - 確認内容
        - 空の集合が返却されること
    """
    # 試験データ
    # 三角ポリゴン頂点座標
    point_a = Point([0.0, 0.0, 0.0])
    point_b = Point([300.0, 600.0, 0.0])
    point_c = Point([600.0, 0.0, 0.0])

    # 水平/垂直方向精度
    h_zoom = 20
    v_zoom = 20

    # 三角ポリゴン
    triangle = Triangle(point_a, point_b, point_c)

    # 衝突判定面
    cross_plane = Plane(point=(0.0, 0.0, 0.0), normal=Vector([1, 0, 0]))

    # 期待値定義
    param_assert = set()

    # 試験実施
    # 戻り値取得
    param_return = _get_cross_points_of_voxel_plane(
            cross_plane, triangle, h_zoom, v_zoom, GEOGRAPHIC_CRS)

    # 戻り値確認
    assert param_assert == param_return


def test__get_cross_points_of_voxel_plane_06():
    """三角ポリゴンと境界面の交点取得、頂点Bと交差する境界面
    + 試験詳細
      - 入力条件
        - 三角ポリゴンの頂点Bと交差する境界面を入力
      - 確認内容
        - 空の集合が返却されること
    """
    # 試験データ
    # 三角ポリゴン頂点座標
    point_a = Point([0.0, 0.0, 0.0])
    point_b = Point([300.0, 600.0, 0.0])
    point_c = Point([600.0, 0.0, 0.0])

    # 水平/垂直方向精度
    h_zoom = 20
    v_zoom = 20

    # 三角ポリゴン
    triangle = Triangle(point_a, point_b, point_c)

    # 衝突判定面
    cross_plane = Plane(point=(0.0, 600.0, 0.0), normal=Vector([0, 1, 0]))

    # 期待値定義
    param_assert = set()

    # 試験実施
    # 戻り値取得
    param_return = _get_cross_points_of_voxel_plane(
            cross_plane, triangle, h_zoom, v_zoom, GEOGRAPHIC_CRS)

    # 戻り値確認
    assert param_assert == param_return


def test__get_cross_points_of_voxel_plane_07():
    """三角ポリゴンと境界面の交点取得、頂点Cと交差する境界面
    + 試験詳細
      - 入力条件
        - 三角ポリゴンの頂点Cと交差する境界面を入力
      - 確認内容
        - 空の集合が返却されること
    """
    # 試験データ
    # 三角ポリゴン頂点座標
    point_a = Point([0.0, 0.0, 0.0])
    point_b = Point([300.0, 600.0, 0.0])
    point_c = Point([600.0, 0.0, 0.0])

    # 水平/垂直方向精度
    h_zoom = 20
    v_zoom = 20

    # 三角ポリゴン
    triangle = Triangle(point_a, point_b, point_c)

    # 衝突判定面
    cross_plane = Plane(point=(600.0, 0.0, 0.0), normal=Vector([1, 0, 0]))

    # 期待値定義
    param_assert = set()

    # 試験実施
    # 戻り値取得
    param_return = _get_cross_points_of_voxel_plane(
            cross_plane, triangle, h_zoom, v_zoom, GEOGRAPHIC_CRS)

    # 戻り値確認
    assert param_assert == param_return


def test__get_cross_points_of_voxel_plane_08():
    """三角ポリゴンと境界面の交点取得、辺と同一平面上となる境界面
    + 試験詳細
      - 入力条件
        - 三角ポリゴンの辺と同一平面上となる境界面を入力
      - 確認内容
        - 交差した辺上の空間IDが返却されること
    """
    # 試験データ
    # 三角ポリゴン頂点座標
    point_a = Point([0.0, 0.0, 0.0])
    point_b = Point([300.0, 600.0, 0.0])
    point_c = Point([600.0, 0.0, 0.0])

    # 水平/垂直方向精度
    h_zoom = 20
    v_zoom = 20

    # 三角ポリゴン
    triangle = Triangle(point_a, point_b, point_c)

    # 衝突判定面
    cross_plane = Plane(point=(0.0, 0.0, 0.0), normal=Vector([0, 1, 0]))

    # 境界面交点
    projected_cross_point = [
            Projected_Point(x=0.0, y=0.0, alt=0.0),
            Projected_Point(x=600.0, y=0.0, alt=0.0)]

    # 境界面交点を緯度経度に変換
    geographic_cross_point = \
        convert_projected_point_list_to_point_list(
                projected_cross_point, PROJECTED_CRS)
    projected_cross_point[0].lon = geographic_cross_point[0].lon
    projected_cross_point[1].lon = geographic_cross_point[1].lon
    projected_cross_point[0].lat = geographic_cross_point[0].lat
    projected_cross_point[1].lat = geographic_cross_point[1].lat

    param_assert = \
        set(get_spatial_ids_on_line(
                projected_cross_point[0],
                projected_cross_point[1],
                h_zoom,
                v_zoom,
                GEOGRAPHIC_CRS))

    # 試験実施
    # 戻り値取得
    param_return = _get_cross_points_of_voxel_plane(
            cross_plane, triangle, h_zoom, v_zoom, GEOGRAPHIC_CRS)

    # 戻り値確認
    assert param_assert == param_return


def test__valid_closed_polygons_01():
    """モデルの閉塞チェック、閉塞チェックOK
    + 試験詳細
      - 入力条件
        - 閉塞しているモデルとなるポリゴンのリストを入力
      - 確認内容
        - 例外が発生しないこと
    """
    # Pointの座標を格納したリストを定義
    point_01 = SpatialPoint(149.05, 35.06, 150.5)
    point_02 = SpatialPoint(148.05, 35.06, 200.5)
    point_03 = SpatialPoint(151.05, 35.06, 200.5)
    point_04 = SpatialPoint(151.05, 40.06, 200.5)
    point_05 = SpatialPoint(148.05, 40.06, 200.5)
    point_06 = SpatialPoint(148.05, 35.06, 200.5)
    point_07 = SpatialPoint(149.05, 35.06, 100.5)

    # 閉塞しているポリゴンモデルを生成
    triangle_01 = SpatialTriangle(point_01, point_02, point_03)
    triangle_02 = SpatialTriangle(point_01, point_03, point_04)
    triangle_03 = SpatialTriangle(point_01, point_04, point_05)
    triangle_04 = SpatialTriangle(point_01, point_05, point_06)
    triangle_05 = SpatialTriangle(point_07, point_02, point_03)
    triangle_06 = SpatialTriangle(point_07, point_03, point_04)
    triangle_07 = SpatialTriangle(point_07, point_04, point_05)
    triangle_08 = SpatialTriangle(point_07, point_05, point_06)
    triangle_list = [
            triangle_01,
            triangle_02,
            triangle_03,
            triangle_04,
            triangle_05,
            triangle_06,
            triangle_07,
            triangle_08]

    # 試験実施
    # 例外が発生しないことを確認
    _valid_closed_polygons(triangle_list)


def test__valid_closed_polygons_02():
    """モデルの閉塞チェック、閉塞チェックNG、共有する頂点以外の頂点奇数個抽出
    + 試験詳細
      - 入力条件
        - 頂点を共有する三角ポリゴンにおいて、共有する頂点以外の頂点を抽出したとき、同じ頂点が奇数個抽出された場合
        - 8面体の1面が存在しないモデルを入力
      - 確認内容
        - ポリゴン閉塞チェックNGの例外が発生すること
    """
    # Pointの座標を格納したリストを定義
    point_01 = SpatialPoint(149.05, 35.06, 150.5)
    point_02 = SpatialPoint(148.05, 35.06, 200.5)
    point_03 = SpatialPoint(151.05, 35.06, 200.5)
    point_04 = SpatialPoint(151.05, 40.06, 200.5)
    point_05 = SpatialPoint(148.05, 40.06, 200.5)
    point_06 = SpatialPoint(148.05, 35.06, 200.5)
    point_07 = SpatialPoint(149.05, 35.06, 100.5)

    # 閉塞しているポリゴンモデルを生成
    triangle_01 = SpatialTriangle(point_01, point_02, point_03)
    triangle_02 = SpatialTriangle(point_01, point_03, point_04)
    triangle_03 = SpatialTriangle(point_01, point_04, point_05)
    triangle_04 = SpatialTriangle(point_01, point_05, point_06)
    triangle_05 = SpatialTriangle(point_07, point_02, point_03)
    triangle_06 = SpatialTriangle(point_07, point_03, point_04)
    triangle_07 = SpatialTriangle(point_07, point_04, point_05)
    triangle_list = [
            triangle_01,
            triangle_02,
            triangle_03,
            triangle_04,
            triangle_05,
            triangle_06,
            triangle_07]

    # 期待値取得
    # 例外を取得
    exception_assert = SpatialIdError('POLYGON_NOT_CLOSED_MODEL')

    # 試験実施
    # 例外発生を確認
    with pytest.raises(SpatialIdError) as e:
        _valid_closed_polygons(triangle_list)

    assert str(exception_assert) == str(e.value)


def test__valid_closed_polygons_03():
    """モデルの閉塞チェック、閉塞チェックNG、辺を共有するポリゴンが複数の場合
    + 試験詳細
      - 入力条件
        - 頂点を共有する三角ポリゴンにおいて、辺を共有するポリゴンが奇数個の場合
        - 8面体の内部に1辺を共有する三角ポリゴンが存在するモデルを入力
      - 確認内容
        - ポリゴン閉塞チェックNGの例外が発生すること
    """
    # Pointの座標を格納したリストを定義
    point_01 = SpatialPoint(149.05, 35.06, 150.5)
    point_02 = SpatialPoint(148.05, 35.06, 200.5)
    point_03 = SpatialPoint(151.05, 35.06, 200.5)
    point_04 = SpatialPoint(151.05, 40.06, 200.5)
    point_05 = SpatialPoint(148.05, 40.06, 200.5)
    point_06 = SpatialPoint(148.05, 35.06, 200.5)
    point_07 = SpatialPoint(149.05, 35.06, 100.5)
    point_08 = SpatialPoint(149.05, 35.06, 150.5)

    # 閉塞しているモデルの面となるポリゴン
    triangle_01 = SpatialTriangle(point_01, point_02, point_03)
    triangle_02 = SpatialTriangle(point_01, point_03, point_04)
    triangle_03 = SpatialTriangle(point_01, point_04, point_05)
    triangle_04 = SpatialTriangle(point_01, point_05, point_06)
    triangle_05 = SpatialTriangle(point_07, point_02, point_03)
    triangle_06 = SpatialTriangle(point_07, point_03, point_04)
    triangle_07 = SpatialTriangle(point_07, point_04, point_05)
    triangle_08 = SpatialTriangle(point_07, point_05, point_06)

    # モデル内部で1編を共有するポリゴン
    triangle_09 = SpatialTriangle(point_01, point_02, point_08)

    # 閉塞しているポリゴンモデルを生成
    triangle_list = [
            triangle_01,
            triangle_02,
            triangle_03,
            triangle_04,
            triangle_05,
            triangle_06,
            triangle_07,
            triangle_08,
            triangle_09]

    # 期待値取得
    # 例外を取得
    exception_assert = SpatialIdError('POLYGON_NOT_CLOSED_MODEL')

    # 試験実施
    # 例外発生を確認
    with pytest.raises(SpatialIdError) as e:
        _valid_closed_polygons(triangle_list)

    assert str(exception_assert) == str(e.value)


def test__get_triangle_plane_spatial_ids_01():
    """ボクセル境界面と三角ポリゴンの面との交点間の空間IDを取得、X, Y境界面との交点あり
    + 試験詳細
      - 入力条件
        - X, Y境界面との交点が発生する三角ポリゴンを入力
      - 確認詳細
        - ポリゴンの面と境界面の交点間の空間IDが返却されること
    """
    # 試験データ定義
    # 水平、垂直方向精度
    h_zoom = 10
    v_zoom = 10

    # 三角ポリゴン頂点の空間ID
    apex_spatial_id = [
            '10/1/1/10/1',
            '10/4/7/10/1',
            '10/7/1/10/1']

    # 地理座標系の頂点座標を取得
    geographic_point_a = get_point_on_spatial_id(
            apex_spatial_id[0], Point_Option.CENTER)[0]
    geographic_point_b = get_point_on_spatial_id(
            apex_spatial_id[1], Point_Option.CENTER)[0]
    geographic_point_c = get_point_on_spatial_id(
            apex_spatial_id[2], Point_Option.CENTER)[0]

    # 投影座標系の頂点座標を取得
    geographic_point_list = [
            geographic_point_a, geographic_point_b, geographic_point_c]
    projected_point_list = convert_point_list_to_projected_point_list(
            geographic_point_list, PROJECTED_CRS)
    skspatial_point_a = Point([
            projected_point_list[0].x,
            projected_point_list[0].y,
            projected_point_list[0].alt])
    skspatial_point_b = Point([
            projected_point_list[1].x,
            projected_point_list[1].y,
            projected_point_list[1].alt])
    skspatial_point_c = Point([
            projected_point_list[2].x,
            projected_point_list[2].y,
            projected_point_list[2].alt])

    # 地理座標系の三角ポリゴン
    geographic_triangle = SpatialTriangle(
            geographic_point_a,
            geographic_point_b,
            geographic_point_c)

    # 投影座標系の三角ポリゴン
    projected_triangle = Triangle(
            skspatial_point_a,
            skspatial_point_b,
            skspatial_point_c)

    # 期待値定義
    param_assert = set([
            '10/1/2/10/1',
            '10/2/1/10/1',
            '10/2/2/10/1',
            '10/2/3/10/1',
            '10/2/4/10/1',
            '10/3/1/10/1',
            '10/3/2/10/1',
            '10/3/3/10/1',
            '10/3/4/10/1',
            '10/3/5/10/1',
            '10/3/6/10/1',
            '10/4/1/10/1',
            '10/4/2/10/1',
            '10/4/3/10/1',
            '10/4/4/10/1',
            '10/4/5/10/1',
            '10/4/6/10/1',
            '10/4/7/10/1',
            '10/5/1/10/1',
            '10/5/2/10/1',
            '10/5/3/10/1',
            '10/5/4/10/1',
            '10/5/5/10/1',
            '10/5/6/10/1',
            '10/6/1/10/1',
            '10/6/2/10/1',
            '10/6/3/10/1',
            '10/6/4/10/1',
            '10/7/1/10/1',
            '10/7/2/10/1'])

    # 試験実施
    # 戻り値取得
    param_return = _get_triangle_plane_spatial_ids(
            geographic_triangle,
            projected_triangle,
            h_zoom,
            v_zoom,
            GEOGRAPHIC_CRS)

    # 戻り値比較
    # 空間IDの要素数が期待値と等しいことを確認
    assert len(param_assert) == len(param_return)

    # 期待値の空間IDが全て含まれていることを確認
    for param in param_return:
        assert (param in param_assert) is True


def test__get_triangle_plane_spatial_ids_02():
    """ボクセル境界面と三角ポリゴンの面との交点間の空間IDを取得、X, Z境界面との交点あり
    + 試験詳細
      - 入力条件
        - X, Z境界面との交点が発生する三角ポリゴンを入力
      - 確認詳細
        - ポリゴンの面と境界面の交点間の空間IDが返却されること
    """
    # 試験データ定義
    # 水平、垂直方向精度
    h_zoom = 10
    v_zoom = 10

    # 三角ポリゴン頂点の空間ID
    apex_spatial_id = [
            '10/1/1/10/1',
            '10/4/1/10/7',
            '10/7/1/10/1']

    # 地理座標系の頂点座標を取得
    geographic_point_a = get_point_on_spatial_id(
            apex_spatial_id[0], Point_Option.CENTER)[0]
    geographic_point_b = get_point_on_spatial_id(
            apex_spatial_id[1], Point_Option.CENTER)[0]
    geographic_point_c = get_point_on_spatial_id(
            apex_spatial_id[2], Point_Option.CENTER)[0]
    geographic_point_list = [
            geographic_point_a, geographic_point_b, geographic_point_c]

    # 投影座標系の頂点座標を取得
    projected_point_list = convert_point_list_to_projected_point_list(
            geographic_point_list, PROJECTED_CRS)
    skspatial_point_a = Point([
            projected_point_list[0].x,
            projected_point_list[0].y,
            projected_point_list[0].alt])
    skspatial_point_b = Point([
            projected_point_list[1].x,
            projected_point_list[1].y,
            projected_point_list[1].alt])
    skspatial_point_c = Point([
            projected_point_list[2].x,
            projected_point_list[2].y,
            projected_point_list[2].alt])

    # 地理座標系の三角ポリゴン
    geographic_triangle = SpatialTriangle(
            geographic_point_a,
            geographic_point_b,
            geographic_point_c)

    # 投影座標系の三角ポリゴン
    projected_triangle = Triangle(
            skspatial_point_a,
            skspatial_point_b,
            skspatial_point_c)

    # 期待値定義
    param_assert = set([
            '10/1/1/10/2',
            '10/2/1/10/1',
            '10/2/1/10/2',
            '10/2/1/10/3',
            '10/2/1/10/4',
            '10/3/1/10/1',
            '10/3/1/10/2',
            '10/3/1/10/3',
            '10/3/1/10/4',
            '10/3/1/10/5',
            '10/3/1/10/6',
            '10/4/1/10/1',
            '10/4/1/10/2',
            '10/4/1/10/3',
            '10/4/1/10/4',
            '10/4/1/10/5',
            '10/4/1/10/6',
            '10/4/1/10/7',
            '10/5/1/10/1',
            '10/5/1/10/2',
            '10/5/1/10/3',
            '10/5/1/10/4',
            '10/5/1/10/5',
            '10/5/1/10/6',
            '10/6/1/10/1',
            '10/6/1/10/2',
            '10/6/1/10/3',
            '10/6/1/10/4',
            '10/7/1/10/1',
            '10/7/1/10/2'])

    # 試験実施
    # 戻り値取得
    param_return = _get_triangle_plane_spatial_ids(
            geographic_triangle,
            projected_triangle,
            h_zoom,
            v_zoom,
            GEOGRAPHIC_CRS)

    # 戻り値比較
    # 空間IDの要素数が期待値と等しいことを確認
    assert len(param_assert) == len(param_return)

    # 期待値の空間IDが全て含まれていることを確認
    for param in param_return:
        assert (param in param_assert) is True


def test__get_triangle_plane_spatial_ids_03():
    """ボクセル境界面と三角ポリゴンの面との交点間の空間IDを取得、Y, Z境界面との交点あり
    + 試験詳細
      - 入力条件
        - Y, Z境界面との交点が発生する三角ポリゴンを入力
      - 確認詳細
        - ポリゴンの面と境界面の交点間の空間IDが返却されること
    """
    # 試験データ定義
    # 水平、垂直方向精度
    h_zoom = 10
    v_zoom = 10

    # 三角ポリゴン頂点の空間ID
    apex_spatial_id = [
            '10/1/1/10/1',
            '10/1/4/10/7',
            '10/1/7/10/1']

    # 地理座標系の頂点座標を取得
    geographic_point_a = get_point_on_spatial_id(
            apex_spatial_id[0], Point_Option.CENTER)[0]
    geographic_point_b = get_point_on_spatial_id(
            apex_spatial_id[1], Point_Option.CENTER)[0]
    geographic_point_c = get_point_on_spatial_id(
            apex_spatial_id[2], Point_Option.CENTER)[0]
    geographic_point_list = [
            geographic_point_a, geographic_point_b, geographic_point_c]

    # 投影座標系の頂点座標を取得
    projected_point_list = convert_point_list_to_projected_point_list(
            geographic_point_list, PROJECTED_CRS)
    skspatial_point_a = Point([
            projected_point_list[0].x,
            projected_point_list[0].y,
            projected_point_list[0].alt])
    skspatial_point_b = Point([
            projected_point_list[1].x,
            projected_point_list[1].y,
            projected_point_list[1].alt])
    skspatial_point_c = Point([
            projected_point_list[2].x,
            projected_point_list[2].y,
            projected_point_list[2].alt])

    # 地理座標系の三角ポリゴン
    geographic_triangle = SpatialTriangle(
            geographic_point_a,
            geographic_point_b,
            geographic_point_c)

    # 投影座標系の三角ポリゴン
    projected_triangle = Triangle(
            skspatial_point_a,
            skspatial_point_b,
            skspatial_point_c)

    # 期待値定義
    param_assert = set([
            '10/1/1/10/2',
            '10/1/2/10/1',
            '10/1/2/10/2',
            '10/1/2/10/3',
            '10/1/2/10/4',
            '10/1/3/10/1',
            '10/1/3/10/2',
            '10/1/3/10/3',
            '10/1/3/10/4',
            '10/1/3/10/5',
            '10/1/3/10/6',
            '10/1/4/10/1',
            '10/1/4/10/2',
            '10/1/4/10/3',
            '10/1/4/10/4',
            '10/1/4/10/5',
            '10/1/4/10/6',
            '10/1/4/10/7',
            '10/1/5/10/1',
            '10/1/5/10/2',
            '10/1/5/10/3',
            '10/1/5/10/4',
            '10/1/5/10/5',
            '10/1/5/10/6',
            '10/1/6/10/1',
            '10/1/6/10/2',
            '10/1/6/10/3',
            '10/1/6/10/4',
            '10/1/7/10/1',
            '10/1/7/10/2'])

    # 試験実施
    # 戻り値取得
    param_return = _get_triangle_plane_spatial_ids(
            geographic_triangle,
            projected_triangle,
            h_zoom,
            v_zoom,
            GEOGRAPHIC_CRS)

    # 戻り値比較
    # 空間IDの要素数が期待値と等しいことを確認
    assert len(param_assert) == len(param_return)

    # 期待値の空間IDが全て含まれていることを確認
    for param in param_return:
        assert (param in param_assert) is True


def test__get_triangle_plane_spatial_ids_04():
    """ボクセル境界面と三角ポリゴンの面との交点間の空間IDを取得、X, Y, Z境界面との交点あり
    + 試験詳細
      - 入力条件
        - X, Y, Z境界面との交点が発生する三角ポリゴンを入力
      - 確認詳細
        - ポリゴンの面と境界面の交点間の空間IDが返却されること
    """
    # 試験データ定義
    # 水平、垂直方向精度
    h_zoom = 10
    v_zoom = 10

    # 三角ポリゴン頂点の空間ID
    apex_spatial_id = [
            '10/1/1/10/1',
            '10/4/7/10/4',
            '10/7/1/10/1']

    # 地理座標系の頂点座標を取得
    geographic_point_a = get_point_on_spatial_id(
            apex_spatial_id[0], Point_Option.CENTER)[0]
    geographic_point_b = get_point_on_spatial_id(
            apex_spatial_id[1], Point_Option.CENTER)[0]
    geographic_point_c = get_point_on_spatial_id(
            apex_spatial_id[2], Point_Option.CENTER)[0]

    # 投影座標系の頂点座標を取得
    geographic_point_list = [
            geographic_point_a, geographic_point_b, geographic_point_c]
    projected_point_list = convert_point_list_to_projected_point_list(
            geographic_point_list, PROJECTED_CRS)
    skspatial_point_a = Point([
            projected_point_list[0].x,
            projected_point_list[0].y,
            projected_point_list[0].alt])
    skspatial_point_b = Point([
            projected_point_list[1].x,
            projected_point_list[1].y,
            projected_point_list[1].alt])
    skspatial_point_c = Point([
            projected_point_list[2].x,
            projected_point_list[2].y,
            projected_point_list[2].alt])

    # 地理座標系の三角ポリゴン
    geographic_triangle = SpatialTriangle(
            geographic_point_a,
            geographic_point_b,
            geographic_point_c)

    # 投影座標系の三角ポリゴン
    projected_triangle = Triangle(
            skspatial_point_a,
            skspatial_point_b,
            skspatial_point_c)

    # 期待値定義
    param_assert = set([
            '10/2/1/10/1',
            '10/3/1/10/1',
            '10/4/1/10/1',
            '10/5/1/10/1',
            '10/6/1/10/1',
            '10/7/1/10/1',
            '10/1/2/10/1',
            '10/2/2/10/1',
            '10/3/2/10/1',
            '10/4/2/10/1',
            '10/5/2/10/1',
            '10/6/2/10/1',
            '10/7/2/10/1',
            '10/2/2/10/2',
            '10/3/2/10/2',
            '10/4/2/10/2',
            '10/5/2/10/2',
            '10/6/2/10/2',
            '10/7/2/10/2',
            '10/2/3/10/2',
            '10/3/3/10/2',
            '10/4/3/10/2',
            '10/5/3/10/2',
            '10/6/3/10/2',
            '10/2/4/10/2',
            '10/3/4/10/2',
            '10/4/4/10/2',
            '10/5/4/10/2',
            '10/6/4/10/2',
            '10/3/4/10/3',
            '10/4/4/10/3',
            '10/5/4/10/3',
            '10/6/4/10/3',
            '10/3/5/10/3',
            '10/4/5/10/3',
            '10/5/5/10/3',
            '10/3/6/10/3',
            '10/4/6/10/3',
            '10/5/6/10/3',
            '10/4/6/10/4',
            '10/4/7/10/4'])

    # 試験実施
    # 戻り値取得
    param_return = _get_triangle_plane_spatial_ids(
            geographic_triangle,
            projected_triangle,
            h_zoom,
            v_zoom,
            GEOGRAPHIC_CRS)

    # 戻り値比較
    # 空間IDの要素数が期待値と等しいことを確認
    assert len(param_assert) == len(param_return)

    # 期待値の空間IDが全て含まれていることを確認
    for param in param_return:
        assert (param in param_assert) is True


def test__get_triangle_plane_spatial_ids_05():
    """ボクセル境界面と三角ポリゴンの面との交点間の空間IDを取得、X, Y, Z境界面との交点あり、頂点座標入れ替えパターン1
    + 試験詳細
      - 入力条件
        - X, Y, Z境界面との交点が発生する三角ポリゴンを入力
        - 頂点の入力順を、試験済みのテストケースと異なる順番に設定
      - 確認詳細
        - ポリゴンの面と境界面の交点間の空間IDが返却されること
    """
    # 試験データ定義
    # 水平、垂直方向精度
    h_zoom = 10
    v_zoom = 10

    # 三角ポリゴン頂点の空間ID
    apex_spatial_id = [
            '10/1/1/10/1',
            '10/4/7/10/4',
            '10/7/1/10/1']

    # 地理座標系の頂点座標を取得
    geographic_point_a = get_point_on_spatial_id(
            apex_spatial_id[0], Point_Option.CENTER)[0]
    geographic_point_b = get_point_on_spatial_id(
            apex_spatial_id[1], Point_Option.CENTER)[0]
    geographic_point_c = get_point_on_spatial_id(
            apex_spatial_id[2], Point_Option.CENTER)[0]

    # 投影座標系の頂点座標を取得
    geographic_point_list = [
            geographic_point_a, geographic_point_b, geographic_point_c]
    projected_point_list = convert_point_list_to_projected_point_list(
            geographic_point_list, PROJECTED_CRS)
    skspatial_point_a = Point([
            projected_point_list[0].x,
            projected_point_list[0].y,
            projected_point_list[0].alt])
    skspatial_point_b = Point([
            projected_point_list[1].x,
            projected_point_list[1].y,
            projected_point_list[1].alt])
    skspatial_point_c = Point([
            projected_point_list[2].x,
            projected_point_list[2].y,
            projected_point_list[2].alt])

    # 地理座標系の三角ポリゴン - 座標の入力順を変更
    geographic_triangle = SpatialTriangle(
            geographic_point_b,
            geographic_point_c,
            geographic_point_a)

    # 投影座標系の三角ポリゴン - 座標の入力順を変更
    projected_triangle = Triangle(
            skspatial_point_b,
            skspatial_point_c,
            skspatial_point_a)

    # 期待値定義
    param_assert = set([
            '10/2/1/10/1',
            '10/3/1/10/1',
            '10/4/1/10/1',
            '10/5/1/10/1',
            '10/6/1/10/1',
            '10/7/1/10/1',
            '10/1/2/10/1',
            '10/2/2/10/1',
            '10/3/2/10/1',
            '10/4/2/10/1',
            '10/5/2/10/1',
            '10/6/2/10/1',
            '10/7/2/10/1',
            '10/2/2/10/2',
            '10/3/2/10/2',
            '10/4/2/10/2',
            '10/5/2/10/2',
            '10/6/2/10/2',
            '10/7/2/10/2',
            '10/2/3/10/2',
            '10/3/3/10/2',
            '10/4/3/10/2',
            '10/5/3/10/2',
            '10/6/3/10/2',
            '10/2/4/10/2',
            '10/3/4/10/2',
            '10/4/4/10/2',
            '10/5/4/10/2',
            '10/6/4/10/2',
            '10/3/4/10/3',
            '10/4/4/10/3',
            '10/5/4/10/3',
            '10/6/4/10/3',
            '10/3/5/10/3',
            '10/4/5/10/3',
            '10/5/5/10/3',
            '10/3/6/10/3',
            '10/4/6/10/3',
            '10/5/6/10/3',
            '10/4/6/10/4',
            '10/4/7/10/4'])

    # 試験実施
    # 戻り値取得
    param_return = _get_triangle_plane_spatial_ids(
            geographic_triangle,
            projected_triangle,
            h_zoom,
            v_zoom,
            GEOGRAPHIC_CRS)

    # 戻り値比較
    # 空間IDの要素数が期待値と等しいことを確認
    assert len(param_assert) == len(param_return)

    # 期待値の空間IDが全て含まれていることを確認
    for param in param_return:
        assert (param in param_assert) is True


def test__get_triangle_plane_spatial_ids_06():
    """ボクセル境界面と三角ポリゴンの面との交点間の空間IDを取得、1ボクセルに収まる三角ポリゴン、ボクセル中点、頂点座標
    + 試験詳細
      - 入力条件
        - 1ボクセル内に収まるポリゴンを入力
        - ボクセルの中点座標と頂点の座標から作られるポリゴンを入力
      - 確認詳細
        - ポリゴンが収まっているボクセルの空間IDが返却されること
    """
    # 試験データ定義
    # 水平、垂直方向精度
    h_zoom = 10
    v_zoom = 10

    # 三角ポリゴン頂点の空間ID
    apex_spatial_id = '10/1/1/10/1'

    # 地理座標系の頂点座標を取得
    geographic_point_a = get_point_on_spatial_id(
            apex_spatial_id, Point_Option.CENTER)[0]
    geographic_point_b = get_point_on_spatial_id(
            apex_spatial_id, Point_Option.VERTEX)[0]
    geographic_point_c = get_point_on_spatial_id(
            apex_spatial_id, Point_Option.VERTEX)[1]
    geographic_point_list = [
            geographic_point_a, geographic_point_b, geographic_point_c]

    # 投影座標系の頂点座標を取得
    projected_point_list = convert_point_list_to_projected_point_list(
            geographic_point_list, PROJECTED_CRS)
    skspatial_point_a = Point([
            projected_point_list[0].x,
            projected_point_list[0].y,
            projected_point_list[0].alt])
    skspatial_point_b = Point([
            projected_point_list[1].x,
            projected_point_list[1].y,
            projected_point_list[1].alt])
    skspatial_point_c = Point([
            projected_point_list[2].x,
            projected_point_list[2].y,
            projected_point_list[2].alt])

    # 地理座標系の三角ポリゴン
    geographic_triangle = SpatialTriangle(
            geographic_point_a,
            geographic_point_b,
            geographic_point_c)

    # 投影座標系の三角ポリゴン
    projected_triangle = Triangle(
            skspatial_point_a,
            skspatial_point_b,
            skspatial_point_c)

    # 期待値定義
    param_assert = set()

    # 試験実施
    # 戻り値取得
    param_return = _get_triangle_plane_spatial_ids(
            geographic_triangle,
            projected_triangle,
            h_zoom,
            v_zoom,
            GEOGRAPHIC_CRS)

    # 戻り値比較
    # 空間IDの要素数が期待値と等しいことを確認
    assert len(param_assert) == len(param_return)

    # 期待値の空間IDが全て含まれていることを確認
    for param in param_return:
        assert (param in param_assert) is True


def test__get_triangle_plane_spatial_ids_07():
    """ボクセル境界面と三角ポリゴンの面との交点間の空間IDを取得、1ボクセルに収まる三角ポリゴン、ボクセル中点、中点付近座標
    + 試験詳細
      - 入力条件
        - 1ボクセル内に収まるポリゴンを入力
        - ボクセルの中点座標と中点近辺の座標から作られるポリゴンを入力
      - 確認詳細
        - 空の集合が返却されること
    """
    # 試験データ定義
    # 水平、垂直方向精度
    h_zoom = 3
    v_zoom = 3

    # 三角ポリゴン頂点の空間ID
    apex_spatial_id = '3/1/1/3/1'

    # 地理座標系の頂点座標を取得 - ボクセル中点取得
    geographic_point_a = get_point_on_spatial_id(
            apex_spatial_id, Point_Option.CENTER)[0]

    # 1ボクセル内に収まる座標とするため、中点座標を元に座標を定義
    geographic_point_b = SpatialPoint(
            geographic_point_a.lon - 5,
            geographic_point_a.lat - 2,
            geographic_point_a.alt - 500000)
    geographic_point_c = SpatialPoint(
            geographic_point_a.lon - 5,
            geographic_point_a.lat + 2,
            geographic_point_a.alt - 500000)
    geographic_point_list = [
            geographic_point_a, geographic_point_b, geographic_point_c]

    # 投影座標系の頂点座標を取得
    projected_point_list = convert_point_list_to_projected_point_list(
            geographic_point_list, PROJECTED_CRS)
    skspatial_point_a = Point([
            projected_point_list[0].x,
            projected_point_list[0].y,
            projected_point_list[0].alt])
    skspatial_point_b = Point([
            projected_point_list[1].x,
            projected_point_list[1].y,
            projected_point_list[1].alt])
    skspatial_point_c = Point([
            projected_point_list[2].x,
            projected_point_list[2].y,
            projected_point_list[2].alt])

    # 地理座標系の三角ポリゴン
    geographic_triangle = SpatialTriangle(
            geographic_point_a,
            geographic_point_b,
            geographic_point_c)

    # 投影座標系の三角ポリゴン
    projected_triangle = Triangle(
            skspatial_point_a,
            skspatial_point_b,
            skspatial_point_c)

    # 期待値定義
    param_assert = set()

    # 試験実施
    # 戻り値取得
    param_return = _get_triangle_plane_spatial_ids(
            geographic_triangle,
            projected_triangle,
            h_zoom,
            v_zoom,
            GEOGRAPHIC_CRS)

    # 戻り値比較
    # 空間IDの要素数が期待値と等しいことを確認
    assert len(param_assert) == len(param_return)

    # 期待値の空間IDが全て含まれていることを確認
    for param in param_return:
        assert (param in param_assert) is True


def test__make_right_rotation_line_01():
    """線の右回転連結メソッド（線の連結無し）
    + 試験詳細
      - 入力条件
        - 線のリストに以下の線を指定する
          - 開始線と連結しない線
          - 開始線から左回りで連結する線
      - 確認内容
        - 入力した値がそのまま返却されること
    """
    # 試験データ定義
    # 開始線
    start_line = [Point([0, 0]), Point([1, 0])]
    # 連結対象の線のリスト
    lines = [
        Line.from_points([2, 1], [1, 2]),
        Line.from_points([0, 0], [-1, 1])
    ]

    # 試験実施
    result_start_line, result_lines = _make_right_rotation_line(start_line, lines)

    # 戻り値確認
    assert result_start_line == start_line
    assert result_lines == lines


def test__make_right_rotation_line_02():
    """線の右回転連結メソッド（始点が一致する線の連結）
    + 試験詳細
      - 入力条件
        - 線のリストに以下の線を指定する
          - 開始線と連結しない線
          - 開始線から左回りで連結する線
          - 始点が開始線から右回りで連結する線
      - 確認内容
        - 想定した結果が返却されること
    """
    # 試験データ定義
    # 開始線
    start_line = [Point([0, 0]), Point([1, 0])]
    # 連結対象の線のリスト
    lines = [
        Line.from_points([2, 1], [1, 2]),
        Line.from_points([0, 0], [-1, 1]),
        Line.from_points([1, 0], [1, 1]),
    ]

    # 期待値の定義
    exp_start_line = [Point([0., 0.]), Point([1., 0.]), Point([1., 1.])]
    exp_lines = [
        Line.from_points([2, 1], [1, 2]),
        Line.from_points([0, 0], [-1, 1]),
    ]

    # 試験実施
    result_start_line, result_lines = _make_right_rotation_line(start_line, lines)

    # 戻り値確認
    assert (np.array(result_start_line) == np.array(exp_start_line)).all()
    for result_line, exp_line in zip(result_lines, exp_lines):
        assert (result_line.direction == exp_line.direction).all()
        assert (result_line.point == exp_line.point).all()


def test__make_right_rotation_line_03():
    """線の右回転連結メソッド（終点が一致する線の連結）
    + 試験詳細
      - 入力条件
        - 線のリストに以下の線を指定する
          - 開始線と連結しない線
          - 開始線から左回りで連結する線
          - 終点が開始線から右回りで連結する線
      - 確認内容
        - 想定した結果が返却されること
    """
    # 試験データ定義
    # 開始線
    start_line = [Point([0, 0]), Point([1, 0])]
    # 連結対象の線のリスト
    lines = [
        Line.from_points([2, 1], [1, 2]),
        Line.from_points([0, 0], [-1, 1]),
        Line.from_points([1, 1], [1, 0]),
    ]

    # 期待値の定義
    exp_start_line = [Point([0., 0.]), Point([1., 0.]), Point([1., 1.])]
    exp_lines = [
        Line.from_points([2., 1.], [1., 2.]),
        Line.from_points([0., 0.], [-1., 1.]),
    ]

    # 試験実施
    result_start_line, result_lines = _make_right_rotation_line(start_line, lines)

    # 戻り値確認
    assert (np.array(result_start_line) == np.array(exp_start_line)).all()
    for result_line, exp_line in zip(result_lines, exp_lines):
        assert (result_line.direction == exp_line.direction).all()
        assert (result_line.point == exp_line.point).all()


def test__make_right_rotation_line_04():
    """線の右回転連結メソッド（複数の線の連結）
    + 試験詳細
      - 入力条件
        - 開始線として複数の線を連結済みの結果とする
        - 線のリストに以下の線を指定する
          - 開始線と連結しない線
          - 開始線から左回りで連結する線
          - 始点が開始線から右回りで連結する線
          - 上記の線と連結する線
      - 確認内容
        - 想定した結果が返却されること
    """
    # 試験データ定義
    # 開始線
    start_line = [Point([0, 0]), Point([0.5, -1]), Point([1, 0])]
    # 連結対象の線のリスト
    lines = [
        Line.from_points([2, 1], [1, 2]),
        Line.from_points([0, 0], [-1, 1]),
        Line.from_points([1, 0], [1, 1]),
        Line.from_points([1, 1], [1.5, 1]),
    ]

    # 期待値の定義
    exp_start_line = [
        Point([0., 0.]),
        Point([0.5, -1.]),
        Point([1., 0.]),
        Point([1., 1.]),
        Point([1.5, 1.])
    ]
    exp_lines = [
        Line.from_points([2., 1.], [1., 2.]),
        Line.from_points([0., 0.], [-1., 1.]),
    ]

    # 試験実施
    result_start_line, result_lines = _make_right_rotation_line(start_line, lines)

    # 戻り値確認
    assert (np.array(result_start_line) == np.array(exp_start_line)).all()
    for result_line, exp_line in zip(result_lines, exp_lines):
        assert (result_line.direction == exp_line.direction).all()
        assert (result_line.point == exp_line.point).all()


def test__make_left_rotation_line_01():
    """線の左回転連結メソッド（線の連結無し）
    + 試験詳細
      - 入力条件
        - 線のリストに以下の線を指定する
          - 開始線と連結しない線
          - 開始線から右回りで連結する線
      - 確認内容
        - 入力した値がそのまま返却されること
    """
    # 試験データ定義
    # 開始線
    start_line = [Point([1, 0]), Point([0, 0])]
    # 連結対象の線のリスト
    lines = [
        Line.from_points([2, 1], [1, 2]),
        Line.from_points([-1, 1], [0, 0])
    ]

    # 試験実施
    result_start_line, result_lines = _make_left_rotation_line(start_line, lines)

    # 戻り値確認
    assert result_start_line == start_line
    assert result_lines == lines


def test__make_left_rotation_line_02():
    """線の左回転連結メソッド（始点が一致する線の連結）
    + 試験詳細
      - 入力条件
        - 線のリストに以下の線を指定する
          - 開始線と連結しない線
          - 開始線から右回りで連結する線
          - 始点が開始線から左回りで連結する線
      - 確認内容
        - 想定した結果が返却されること
    """
    # 試験データ定義
    # 開始線
    start_line = [Point([1, 0]), Point([0, 0])]
    # 連結対象の線のリスト
    lines = [
        Line.from_points([2, 1], [1, 2]),
        Line.from_points([0, 0], [-1, 1]),
        Line.from_points([1, 0], [1, 1]),
    ]

    # 期待値の定義
    exp_start_line = [Point([1., 1.]), Point([1., 0.]), Point([0., 0.])]
    exp_lines = [
        Line.from_points([2, 1], [1, 2]),
        Line.from_points([0, 0], [-1, 1]),
    ]

    # 試験実施
    result_start_line, result_lines = _make_left_rotation_line(start_line, lines)

    # 戻り値確認
    assert (np.array(result_start_line) == np.array(exp_start_line)).all()
    for result_line, exp_line in zip(result_lines, exp_lines):
        assert (result_line.direction == exp_line.direction).all()
        assert (result_line.point == exp_line.point).all()


def test__make_left_rotation_line_03():
    """線の左回転連結メソッド（終点が一致する線の連結）
    + 試験詳細
      - 入力条件
        - 線のリストに以下の線を指定する
          - 開始線と連結しない線
          - 開始線から右回りで連結する線
          - 終点が開始線から左回りで連結する線
      - 確認内容
        - 想定した結果が返却されること
    """
    # 試験データ定義
    # 開始線
    start_line = [Point([1, 0]), Point([0, 0])]
    # 連結対象の線のリスト
    lines = [
        Line.from_points([2, 1], [1, 2]),
        Line.from_points([0, 0], [-1, 1]),
        Line.from_points([1, 1], [1, 0]),
    ]

    # 期待値の定義
    exp_start_line = [Point([1., 1.]), Point([1., 0.]), Point([0., 0.])]
    exp_lines = [
        Line.from_points([2., 1.], [1., 2.]),
        Line.from_points([0., 0.], [-1., 1.]),
    ]

    # 試験実施
    result_start_line, result_lines = _make_left_rotation_line(start_line, lines)

    # 戻り値確認
    assert (np.array(result_start_line) == np.array(exp_start_line)).all()
    for result_line, exp_line in zip(result_lines, exp_lines):
        assert (result_line.direction == exp_line.direction).all()
        assert (result_line.point == exp_line.point).all()

def test__make_left_rotation_line_04():
    """線の左回転連結メソッド（複数の線の連結）
    + 試験詳細
      - 入力条件
        - 開始線として複数の線を連結済みの結果とする
        - 線のリストに以下の線を指定する
          - 開始線と連結しない線
          - 開始線から右回りで連結する線
          - 始点が開始線から左回りで連結する線
          - 上記の線と連結する線
      - 確認内容
        - 想定した結果が返却されること
    """
    # 試験データ定義
    # 開始線
    start_line = [Point([1, 0]), Point([0.5, -1]), Point([0, 0])]
    # 連結対象の線のリスト
    lines = [
        Line.from_points([2, 1], [1, 2]),
        Line.from_points([0, 0], [-1, 1]),
        Line.from_points([1, 0], [1, 1]),
        Line.from_points([1, 1], [1.5, 1]),
    ]

    # 期待値の定義
    exp_start_line = [
        Point([1.5, 1.]),
        Point([1., 1.]),
        Point([1., 0.]),
        Point([0.5, -1.]),
        Point([0., 0.]),
    ]
    exp_lines = [
        Line.from_points([2., 1.], [1., 2.]),
        Line.from_points([0., 0.], [-1., 1.]),
    ]

    # 試験実施
    result_start_line, result_lines = _make_left_rotation_line(start_line, lines)

    # 戻り値確認
    assert (np.array(result_start_line) == np.array(exp_start_line)).all()
    for result_line, exp_line in zip(result_lines, exp_lines):
        assert (result_line.direction == exp_line.direction).all()
        assert (result_line.point == exp_line.point).all()


def test__make_line_group_01():
    """線のグループ化メソッド（単一グループ）
    + 試験詳細
      - 入力条件
        - 線のリストに一グループとして連結可能な線を指定する
      - 確認内容
        - 想定した結果が返却されること
    """
    # 連結対象の線のリスト
    lines = [
        Line.from_points([0, 0], [-1, 1]),
        Line.from_points([1, 0], [0.5, -1]),
        Line.from_points([1, 0], [1, 1]),
        Line.from_points([0, 0], [0.5, -1]),
        Line.from_points([1, 1], [1.5, 1]),
    ]

    # 期待値の定義
    exp_lines = [
        Point([1.5, 1.]),
        Point([1., 1.]),
        Point([1., 0.]),
        Point([0.5, -1.]),
        Point([0., 0.]),
    ]

    # 試験実施
    result_lines = _make_line_group(lines)

    # 戻り値確認
    assert len(result_lines) == 1
    for result_point, exp_point in zip(result_lines[0], exp_lines):
        assert (result_point == exp_point).all()


def test__make_line_group_02():
    """線のグループ化メソッド（複数グループ）
    + 試験詳細
      - 入力条件
        - 線のリストに複数グループの連結可能な線を指定する
      - 確認内容
        - 想定した結果が返却されること
    """
    # 連結対象の線のリスト
    lines = [
        Line.from_points([0, 0], [-1, 1]),
        Line.from_points([3, 3], [4, 4]),
        Line.from_points([1, 0], [0.5, -1]),
        Line.from_points([1, 0], [1, 1]),
        Line.from_points([3, 5], [4, 4]),
        Line.from_points([3, 5], [3, 3]),
        Line.from_points([0, 0], [0.5, -1]),
        Line.from_points([1, 1], [1.5, 1]),
    ]

    # 期待値の定義
    exp_lines_1 = [
        Point([1.5, 1.]),
        Point([1., 1.]),
        Point([1., 0.]),
        Point([0.5, -1.]),
        Point([0., 0.]),
    ]
    exp_lines_2 = [
        Point([3., 3.]),
        Point([4., 4.]),
        Point([3., 5.]),
    ]

    # 試験実施
    result_lines = _make_line_group(lines)

    # 戻り値確認
    assert len(result_lines) == 2
    for result_point, exp_point in zip(result_lines[0], exp_lines_1):
        assert (result_point == exp_point).all()
    for result_point, exp_point in zip(result_lines[1], exp_lines_2):
        assert (result_point == exp_point).all()


def test__make_line_group_03():
    """線のグループ化メソッド（線のリストが空）
    + 試験詳細
      - 入力条件
        - 線のリストに空のリストを指定する
      - 確認内容
        - 空のリストが返却されること
    """
    # 連結対象の線のリスト
    lines = []

    # 試験実施
    result_lines = _make_line_group(lines)

    # 戻り値確認
    assert len(result_lines) == 0


def test__is_inside_01():
    """多角形の内部判定メソッド（多角形が直線）
    + 試験詳細
      - 入力条件
        - 多角形の線が全て同一直線上にあるように設定する
      - 確認内容
        - Falseが返却されること
    """
    # 多角形の頂点のリスト
    vertex_point_list = [
        Point([3., 3.]),
        Point([4., 4.]),
        Point([3.5, 3.5]),
    ]
    # 判定座標
    target_point = Point([3., 3.])

    # 試験実施
    result = _is_inside(vertex_point_list, target_point)

    # 戻り値確認
    assert not result


def test__is_inside_02():
    """多角形の内部判定メソッド（判定座標が頂点上）
    MEMO 頂点や辺の座標を指定したときの結果は未定義のため、結果の確認は不要とする。
         チェック対象の座標(頂点、辺)が事前に三角形の頂点や辺にないことを確認して使うこと。
    + 試験詳細
      - 入力条件
        - 多角形の線が全て同一直線上にないように設定する
        - 判定座標を多角形の頂点に設定する
      - 確認内容
        - Trueが返却されること
    """
    # 多角形の頂点のリスト
    vertex_point_list = [
        Point([3., 3.]),
        Point([4., 4.]),
        Point([3., 5.]),
    ]
    # 判定座標
    target_point = Point([3., 3.])


    # 試験実施
    result = _is_inside(vertex_point_list, target_point)


def test__is_inside_03():
    """多角形の内部判定メソッド（判定座標が辺上）
    MEMO 頂点や辺の座標を指定したときの結果は未定義のため、結果の確認は不要とする。
         チェック対象の座標(頂点、辺)が事前に三角形の頂点や辺にないことを確認して使うこと。
    + 試験詳細
      - 入力条件
        - 多角形の線が全て同一直線上にないように設定する
        - 判定座標を多角形の辺上に設定する
      - 確認内容
        - Trueが返却されること
    """
    # 多角形の頂点のリスト
    vertex_point_list = [
        Point([3., 3.]),
        Point([4., 4.]),
        Point([3., 5.]),
    ]
    # 判定座標
    target_point = Point([3.5, 3.5])

    # 試験実施
    result = _is_inside(vertex_point_list, target_point)


def test__is_inside_04():
    """多角形の内部判定メソッド（判定座標が内部）
    + 試験詳細
      - 入力条件
        - 多角形の線が全て同一直線上にないように設定する
        - 判定座標を多角形の内部に設定する
      - 確認内容
        - Trueが返却されること
    """
    # 多角形の頂点のリスト
    vertex_point_list = [
        Point([3., 3.]),
        Point([4., 4.]),
        Point([3., 5.]),
    ]
    # 判定座標
    target_point = Point([3.5, 4.])


    # 試験実施
    result = _is_inside(vertex_point_list, target_point)

    # 戻り値確認
    assert result

def test__is_inside_05():
    """多角形の内部判定メソッド（判定座標が外部）
    + 試験詳細
      - 入力条件
        - 多角形の線が全て同一直線上にないように設定する
        - 判定座標を多角形の外部に設定する
      - 確認内容
        - Trueが返却されること
    """
    # 多角形の頂点のリスト
    vertex_point_list = [
        Point([3., 3.]),
        Point([4., 4.]),
        Point([3., 5.]),
    ]
    # 判定座標
    target_point = Point([2.5, 3.5])

    # 試験実施
    result = _is_inside(vertex_point_list, target_point)

    # 戻り値確認
    assert not result

def test__grouping_triangle_01():
    """三角ポリゴンのグルーピングメソッド（辺のグルーピング）
    + 試験詳細
      - 入力条件
        - 辺の三角ポリゴン辞書をグルーピングする
      - 確認内容
        - 想定した結果が返却されること
    """
    # 衝突点の集合
    cross_point_set = {"3.0_3.0_1.0%0"}
    # 共有グループとなる三角ポリゴン
    union_triangle_indexes = {0, 1}
    # 辺の三角ポリゴン辞書
    edge_triangle_group = {
        "3.0_3.0_1.0%0": {0, 1},
        "3.0_3.0_4.0%0": {3, 4},
        "3.0_3.0_4.0%1": {1, 2}
    }
    # 頂点の三角ポリゴン辞書
    vertex_triangle_group = {}

    ## 期待値
    # 衝突点の集合
    cross_point_set_assert = {"3.0_3.0_1.0%0", "3.0_3.0_4.0%1"}
    union_triangle_indexes_assert = {0, 1, 2}

    # 試験実施
    _grouping_triangle(
        cross_point_set,
        union_triangle_indexes,
        edge_triangle_group,
        vertex_triangle_group,
    )

    # 戻り値確認
    assert cross_point_set == cross_point_set_assert
    assert union_triangle_indexes == union_triangle_indexes_assert


def test__grouping_triangle_02():
    """三角ポリゴンのグルーピングメソッド（頂点のグルーピング）
    + 試験詳細
      - 入力条件
        - 頂点の三角ポリゴン辞書をグルーピングする
      - 確認内容
        - 想定した結果が返却されること
    """
    # 衝突点の集合
    cross_point_set = {"3.0_3.0_1.0%0"}
    # 共有グループとなる三角ポリゴン
    union_triangle_indexes = {0, 1, 2}
    # 辺の三角ポリゴン辞書
    edge_triangle_group = {}
    # 頂点の三角ポリゴン辞書
    vertex_triangle_group = {
        "3.0_3.0_1.0%0": {0, 1, 2},
        "3.0_3.0_4.0%0": {3, 4, 5},
        "3.0_3.0_4.0%1": {1, 6, 7}
    }

    ## 期待値
    # 衝突点の集合
    cross_point_set_assert = {"3.0_3.0_1.0%0", "3.0_3.0_4.0%1"}
    union_triangle_indexes_assert = {0, 1, 2, 6, 7}

    # 試験実施
    _grouping_triangle(
        cross_point_set,
        union_triangle_indexes,
        edge_triangle_group,
        vertex_triangle_group,
    )

    # 戻り値確認
    assert cross_point_set == cross_point_set_assert
    assert union_triangle_indexes == union_triangle_indexes_assert


def test__grouping_triangle_03():
    """三角ポリゴンのグルーピングメソッド（頂点・辺のグルーピング）
    + 試験詳細
      - 入力条件
        - 頂点と辺の三角ポリゴン辞書をグルーピングする
      - 確認内容
        - 想定した結果が返却されること
    """
    # 衝突点の集合
    cross_point_set = {"3.0_3.0_1.0%0"}
    # 共有グループとなる三角ポリゴン
    union_triangle_indexes = {0, 1, 2}
    # 辺の三角ポリゴン辞書
    edge_triangle_group = {
        "3.0_3.0_6.0%0": {9, 10},
        "3.0_3.0_6.0%1": {8, 7}
    }
    # 頂点の三角ポリゴン辞書
    vertex_triangle_group = {
        "3.0_3.0_1.0%0": {0, 1, 2},
        "3.0_3.0_4.0%0": {3, 4, 5},
        "3.0_3.0_4.0%1": {1, 6, 7}
    }

    ## 期待値
    # 衝突点の集合
    cross_point_set_assert = {"3.0_3.0_1.0%0", "3.0_3.0_4.0%1", "3.0_3.0_6.0%1"}
    union_triangle_indexes_assert = {0, 1, 2, 6, 7, 8}

    # 試験実施
    _grouping_triangle(
        cross_point_set,
        union_triangle_indexes,
        edge_triangle_group,
        vertex_triangle_group,
    )

    # 戻り値確認
    assert cross_point_set == cross_point_set_assert
    assert union_triangle_indexes == union_triangle_indexes_assert


def test__count_edge_side_01():
    """辺の左右対称メソッド(辺の右に三角ポリゴンが存在)
    + 試験詳細
      - 入力条件
        - 座標を3次元座標で指定する
        - 同グループの三角ポリゴンを辺の右になるように配置する
      - 確認内容
        - 想定する値が返却されること
    """
    # 辺
    xy_cross_edge = Line.from_points([3., 3.], [4., 4.])
    # 同グループの三角ポリゴンのrectangular_triangles内のインデックス
    group_triangle_indexes = {0}
    # 全三角ポリゴン
    rectangular_triangles = [
        Triangle([3., 3., 0.], [4., 4., 1.], [3.5, 2., 2.])
    ]

    # 試験実施
    left_count, right_count = _count_edge_side(xy_cross_edge, group_triangle_indexes, rectangular_triangles)

    ## 期待値
    # 左判定回数
    left_count_assert = 0
    # 右判定回数
    right_count_assert = 1

    # 戻り値確認
    assert left_count == left_count_assert
    assert right_count == right_count_assert


def test__count_edge_side_02():
    """辺の左右対称メソッド(辺の左に三角ポリゴンが存在)
    + 試験詳細
      - 入力条件
        - 座標を3次元座標で指定する
        - 同グループの三角ポリゴンを辺の左になるように配置する
      - 確認内容
        - 想定する値が返却されること
    """
    # 辺
    xy_cross_edge = Line.from_points([3., 3.], [4., 4.])
    # 同グループの三角ポリゴンのrectangular_triangles内のインデックス
    group_triangle_indexes = {1}
    # 全三角ポリゴン
    rectangular_triangles = [
        Triangle([3., 3., 0.], [4., 4., 1.], [3.5, 2., 2.]),
        Triangle([3., 3., 0.], [4., 4., 1.], [3.5, 4., 2.])
    ]

    # 試験実施
    left_count, right_count = _count_edge_side(xy_cross_edge, group_triangle_indexes, rectangular_triangles)

    ## 期待値
    # 左判定回数
    left_count_assert = 1
    # 右判定回数
    right_count_assert = 0

    # 戻り値確認
    assert left_count == left_count_assert
    assert right_count == right_count_assert

def test__count_edge_side_03():
    """辺の左右対称メソッド(辺に垂直に三角ポリゴンが存在)
    + 試験詳細
      - 入力条件
        - 座標を3次元座標で指定する
        - 同グループの三角ポリゴンを辺に垂直になるように配置する
      - 確認内容
        - 想定する値が返却されること
    """
    # 辺
    xy_cross_edge = Line.from_points([3., 3.], [4., 4.])
    # 同グループの三角ポリゴンのrectangular_triangles内のインデックス
    group_triangle_indexes = {0}
    # 全三角ポリゴン
    rectangular_triangles = [
        Triangle([3., 3., 0.], [4., 4., 1.], [3.5, 3.5, 2.]),
    ]

    # 試験実施
    left_count, right_count = _count_edge_side(xy_cross_edge, group_triangle_indexes, rectangular_triangles)

    ## 期待値
    # 左判定回数
    left_count_assert = 0
    # 右判定回数
    right_count_assert = 0

    # 戻り値確認
    assert left_count == left_count_assert
    assert right_count == right_count_assert


def test__search_judge_vertex_01():
    """XY上の判定対象頂点を検索取得 (前方の交点CXを通る辺から連結される辺が後方の交点CXを通る辺の場合)
    + 試験詳細
      - 入力条件
        - 前方の交点CXを通る辺と連結する辺に後方の交点CXを通る辺を指定する
      - 確認内容
        - 前方の交点CXを通る辺と連結する後方の交点CXを通る辺の連結点が返却されること
    """
    # 処理を開始する衝突点の前にある三角ポリゴンの衝突点を含まない辺のインデックス
    index = 0
    # 衝突点が通過する直線L（2次元）
    xy_vertex_line = Line.from_points([4, 4], [5, 2])
    # 処理を開始する衝突点の前にある三角ポリゴンの衝突点を含まない辺の端点（2次元）
    xy_front_start_point = Point([5, 5])
    # 衝突点の前にある三角ポリゴンの衝突点を含まない辺（2次元）
    share_front_edges = [
        Line.from_points([3, 3.5], [5, 5]),
    ]
    # 衝突点の後にある三角ポリゴンの衝突点を含まない辺
    share_back_edges = [
        Line.from_points([3.5, 3], [5, 2]),
        Line.from_points([5, 5], [5, 2]),
    ]
    #  衝突点の前後判定が出来なかった三角ポリゴンの衝突点を含まない辺（2次元）
    no_cross_triangle_edges = [
        Line.from_points([3, 3.5], [3.5, 3]),
    ]

    # 試験実施
    target_vertex = _search_judge_vertex(
        index,
        xy_front_start_point,
        xy_vertex_line,
        share_front_edges,
        share_back_edges,
        no_cross_triangle_edges
    )

    ## 期待値
    # 判定対象点
    target_vertex_assert = Point([5, 5])

    # 戻り値確認
    assert (target_vertex == target_vertex_assert).all()


def test__search_judge_vertex_02():
    """XY上の判定対象頂点を検索取得
    | 前方の交点CXを通る辺から連結される辺が交点CXを通らない辺
    | 連結される辺が交点CXを通らない辺が後方の交点CXを通る辺と連結する
    + 試験詳細
      - 入力条件
        - 前方の交点CXを通る辺と連結する交点CXを通らない辺を指定する
        - 上記の連結する交点CXを通らない辺と連結する後方の交点CXを通る辺を指定する
      - 確認内容
        - 交点CXを通らない辺と連結する後方の交点CXを通る辺の連結点が返却されること
    """
    # 処理を開始する衝突点の前にある三角ポリゴンの衝突点を含まない辺のインデックス
    index = 0
    # 衝突点が通過する直線L（2次元）
    xy_vertex_line = Line.from_points([4, 4], [5, 2])
    # 処理を開始する衝突点の前にある三角ポリゴンの衝突点を含まない辺の端点（2次元）
    xy_front_start_point = Point([3, 3.5])
    # 衝突点の前にある三角ポリゴンの衝突点を含まない辺（2次元）
    share_front_edges = [
        Line.from_points([3, 3.5], [5, 5]),
    ]
    # 衝突点の後にある三角ポリゴンの衝突点を含まない辺
    share_back_edges = [
        Line.from_points([3.5, 3], [5, 2]),
        Line.from_points([5, 5], [5, 2]),
    ]
    #  衝突点の前後判定が出来なかった三角ポリゴンの衝突点を含まない辺（2次元）
    no_cross_triangle_edges = [
        Line.from_points([3, 3.5], [3.5, 3]),
    ]

    # 試験実施
    target_vertex = _search_judge_vertex(
        index,
        xy_front_start_point,
        xy_vertex_line,
        share_front_edges,
        share_back_edges,
        no_cross_triangle_edges
    )

    ## 期待値
    # 判定対象点
    target_vertex_assert = Point([3.5, 3])

    # 戻り値確認
    assert (target_vertex == target_vertex_assert).all()


def test__search_judge_vertex_03():
    """XY上の判定対象頂点を検索取得
    | 前方の交点CXを通る辺から連結される辺が交点CXを通らない辺
    | 上記の交点CXを通らない辺が後方の交点CXを通る辺と連結しない
    + 試験詳細
      - 入力条件
        - 前方の交点CXを通る辺と連結する交点CXを通らない辺を指定する
        - 上記の連結する交点CXを通らない辺と連結する後方の交点CXを通る辺を指定しない
      - 確認内容
        - Noneが返却されること
    """
    # 処理を開始する衝突点の前にある三角ポリゴンの衝突点を含まない辺のインデックス
    index = 0
    # 衝突点が通過する直線L（2次元）
    xy_vertex_line = Line.from_points([4, 4], [5, 2])
    # 処理を開始する衝突点の前にある三角ポリゴンの衝突点を含まない辺の端点（2次元）
    xy_front_start_point = Point([3, 3.5])
    # 衝突点の前にある三角ポリゴンの衝突点を含まない辺（2次元）
    share_front_edges = [
        Line.from_points([3, 3.5], [5, 5]),
    ]
    # 衝突点の後にある三角ポリゴンの衝突点を含まない辺
    share_back_edges = [
        Line.from_points([5, 5], [5, 2]),
    ]
    #  衝突点の前後判定が出来なかった三角ポリゴンの衝突点を含まない辺（2次元）
    no_cross_triangle_edges = [
        Line.from_points([3, 3.5], [3.5, 3]),
        Line.from_points([0, 0], [-1, -1]),
    ]

    # 試験実施
    target_vertex = _search_judge_vertex(
        index,
        xy_front_start_point,
        xy_vertex_line,
        share_front_edges,
        share_back_edges,
        no_cross_triangle_edges
    )

    # 戻り値確認
    assert target_vertex is None


def test__search_judge_vertex_04():
    """XY上の判定対象頂点を検索取得
    | 前方の交点CXを通る辺から連結される辺が前方の交点CXを通る辺の場合
    + 試験詳細
      - 入力条件
        - 前方の交点CXを通る辺と連結する前方の交点CXを通る辺を指定する
      - 確認内容
        - Noneが返却されること
    """
    # 処理を開始する衝突点の前にある三角ポリゴンの衝突点を含まない辺のインデックス
    index = 0
    # 衝突点が通過する直線L（2次元）
    xy_vertex_line = Line.from_points([4, 4], [5, 2])
    # 処理を開始する衝突点の前にある三角ポリゴンの衝突点を含まない辺の端点（2次元）
    xy_front_start_point = Point([4, 7])
    # 衝突点の前にある三角ポリゴンの衝突点を含まない辺（2次元）
    share_front_edges = [
        Line.from_points([3.5, 3.5], [4, 7]),
        Line.from_points([4, 7], [2, 5]),
    ]
    # 衝突点の後にある三角ポリゴンの衝突点を含まない辺
    share_back_edges = [
        Line.from_points([6, 4], [3, 2]),
        Line.from_points([6, 4], [3.5, 3.5]),
    ]
    #  衝突点の前後判定が出来なかった三角ポリゴンの衝突点を含まない辺（2次元）
    no_cross_triangle_edges = [
        Line.from_points([2, 5], [3, 2]),
    ]

    # 試験実施
    target_vertex = _search_judge_vertex(
        index,
        xy_front_start_point,
        xy_vertex_line,
        share_front_edges,
        share_back_edges,
        no_cross_triangle_edges
    )

    # 戻り値確認
    assert target_vertex is None


def test__search_judge_vertex_05():
    """XY上の判定対象頂点を検索取得 (前方の交点CXを通る辺から連結される辺が後方の交点CXを通る辺の場合)
    | 前方の交点CXを通る辺と連結する後方の交点CXを通る辺の連結点が辺と端点と一致する
    + 試験詳細
      - 入力条件
        - 前方の交点CXを通る辺と連結する辺に後方の交点CXを通る辺を指定する
        - 前方の交点CXを通る辺と連結する後方の交点CXを通る辺の連結点を衝突点が通過する辺の始点とする
      - 確認内容
        - Noneが返却されること
    """
    # 処理を開始する衝突点の前にある三角ポリゴンの衝突点を含まない辺のインデックス
    index = 0
    # 衝突点が通過する直線L（2次元）
    xy_vertex_line = Line.from_points([4, 4], [5, 2])
    # 処理を開始する衝突点の前にある三角ポリゴンの衝突点を含まない辺の端点（2次元）
    xy_front_start_point = Point([5, 2])
    # 衝突点の前にある三角ポリゴンの衝突点を含まない辺（2次元）
    share_front_edges = [
        Line.from_points([4.5, 3], [5, 2]),
    ]
    # 衝突点の後にある三角ポリゴンの衝突点を含まない辺
    share_back_edges = [
        Line.from_points([3.5, 3], [5, 2]),
    ]
    #  衝突点の前後判定が出来なかった三角ポリゴンの衝突点を含まない辺（2次元）
    no_cross_triangle_edges = [
    ]

    # 試験実施
    target_vertex = _search_judge_vertex(
        index,
        xy_front_start_point,
        xy_vertex_line,
        share_front_edges,
        share_back_edges,
        no_cross_triangle_edges
    )

    ## 期待値
    # 戻り値確認
    assert target_vertex is None


def test__search_judge_vertex_06():
    """XY上の判定対象頂点を検索取得
    | 前方の交点CXを通る辺から連結される辺が交点CXを通らない辺の場合
    | 上記の交点CXを通らない辺が別の前方の交点CXを通る辺に連結する場合
    + 試験詳細
      - 入力条件
        - 前方の交点CXを通る辺と連結する交点CXを通らない辺を指定する
        - 記の交点CXを通らない辺が別の前方の交点CXを通る辺を指定する
      - 確認内容
        - Noneが返却されること
    """
    # 処理を開始する衝突点の前にある三角ポリゴンの衝突点を含まない辺のインデックス
    index = 0
    # 衝突点が通過する直線L（2次元）
    xy_vertex_line = Line.from_points([4, 4], [5, 2])
    # 処理を開始する衝突点の前にある三角ポリゴンの衝突点を含まない辺の端点（2次元）
    xy_front_start_point = Point([4, 7])
    # 衝突点の前にある三角ポリゴンの衝突点を含まない辺（2次元）
    share_front_edges = [
        Line.from_points([3.5, 3.5], [4, 7]),
        Line.from_points([4, 6], [2, 5]),
    ]
    # 衝突点の後にある三角ポリゴンの衝突点を含まない辺
    share_back_edges = [
        Line.from_points([6, 4], [3, 2]),
        Line.from_points([6, 4], [3.5, 3.5]),
    ]
    #  衝突点の前後判定が出来なかった三角ポリゴンの衝突点を含まない辺（2次元）
    no_cross_triangle_edges = [
        Line.from_points([4, 6], [4, 7]),
        Line.from_points([2, 5], [3, 2]),
    ]

    # 試験実施
    target_vertex = _search_judge_vertex(
        index,
        xy_front_start_point,
        xy_vertex_line,
        share_front_edges,
        share_back_edges,
        no_cross_triangle_edges
    )

    ## 期待値
    # 戻り値確認
    assert target_vertex is None


def test__count_vertex_side_01():
    """頂点に対する左右判定(判定対象点が左に存在)
    + 試験詳細
      - 入力条件
        - 衝突点に対して左と判定される三角ポリゴンを指定する
      - 確認内容
        - 左判定回数1が返却されること
    """
    # 衝突点（3次元）
    cross_point = Point([3., 3., 6.])
    #  同グループの三角ポリゴンのrectangular_triangles内のインデックス
    group_triangle_indexes = {0, 1, 2}
    # 衝突点（3次元）と同グループの三角ポリゴンのrectangular_triangles内のインデックスのタプル配列
    cross_point_infos = [(cross_point, group_triangle_indexes)]
    # 衝突点が通過する直線L（2次元）
    xy_cross_line = Line.from_points([1., 1.], [5., 5.])
    # 全三角ポリゴン
    rectangular_triangles = [
        Triangle([1., 1., 1.], [5., 5., 1.], [3., 3., 6.]),
        Triangle([3., 3., 6.], [5., 5., 1.], [3., 7., 1.]),
        Triangle([3., 3., 6.], [3., 7., 1.], [1., 1., 1.])
    ]

    ## 期待値
    # 左判定回数
    left_count_assert = 1
    # 右判定回数
    right_count_assert = 0

    # 試験実施
    left_count, right_count = _count_vertex_side(
        cross_point_infos,
        xy_cross_line,
        rectangular_triangles
    )

    assert left_count == left_count_assert
    assert right_count == right_count_assert


def test__count_vertex_side_02():
    """頂点に対する左右判定(判定対象点が右に存在)
    + 試験詳細
      - 入力条件
        - 衝突点に対して右と判定される三角ポリゴンを指定する
      - 確認内容
        - 右判定回数1が返却されること
    """
    # 衝突点（3次元）
    cross_point = Point([3., 3., 6.])
    #  同グループの三角ポリゴンのrectangular_triangles内のインデックス
    group_triangle_indexes = {0, 1, 2}
    # 衝突点（3次元）と同グループの三角ポリゴンのrectangular_triangles内のインデックスのタプル配列
    cross_point_infos = [(cross_point, group_triangle_indexes)]
    # 衝突点が通過する直線L（2次元）
    xy_cross_line = Line.from_points([1., 1.], [5., 5.])
    # 全三角ポリゴン
    rectangular_triangles = [
        Triangle([3., 3., 6.], [5., 5., 1.], [1., 1., 1.]),
        Triangle([3., 3., 6.], [5., 5., 1.], [3., 1., 1.]),
        Triangle([3., 3., 6.], [3., 1., 1.], [1., 1., 1.])
    ]

    ## 期待値
    # 左判定回数
    left_count_assert = 0
    # 右判定回数
    right_count_assert = 1

    # 試験実施
    left_count, right_count = _count_vertex_side(
        cross_point_infos,
        xy_cross_line,
        rectangular_triangles
    )

    assert left_count == left_count_assert
    assert right_count == right_count_assert


def test__count_vertex_side_03():
    """頂点に対する左右判定(交点CXがない直線Lに平行でない辺が存在)
    + 試験詳細
      - 入力条件
        - 衝突点に対して交点CXがない直線Lに平行でない三角ポリゴンを指定する
        - 上記を含めて衝突点に対して右と判定される三角ポリゴンを指定する
      - 確認内容
        - 左判定回数1が返却されること
    """
    # 衝突点（3次元）
    cross_point = Point([3., 3., 6.])
    #  同グループの三角ポリゴンのrectangular_triangles内のインデックス
    group_triangle_indexes = {0, 1, 2, 3}
    # 衝突点（3次元）と同グループの三角ポリゴンのrectangular_triangles内のインデックスのタプル配列
    cross_point_infos = [(cross_point, group_triangle_indexes)]
    # 衝突点が通過する直線L（2次元）
    xy_cross_line = Line.from_points([1., 1.], [5., 5.])
    # 全三角ポリゴン
    rectangular_triangles = [
        Triangle([3., 3., 6.], [5., 5., 1.], [1., 1., 1.]),
        Triangle([3., 3., 6.], [5., 5., 1.], [4., 6., 1.]),
        Triangle([3., 3., 6.], [4., 6., 1.], [3., 7., 1.]),
        Triangle([3., 3., 6.], [3., 7., 1.], [1., 1., 1.])
    ]

    ## 期待値
    # 左判定回数
    left_count_assert = 1
    # 右判定回数
    right_count_assert = 0

    # 試験実施
    left_count, right_count = _count_vertex_side(
        cross_point_infos,
        xy_cross_line,
        rectangular_triangles
    )

    assert left_count == left_count_assert
    assert right_count == right_count_assert


def test__count_vertex_side_04():
    """頂点に対する左右判定(直線Lに平行な辺が存在)
    + 試験詳細
      - 入力条件
        - 衝突点に対して直線Lに平行な辺を持つ三角ポリゴンを指定する
        - 上記を含めて衝突点に対して右と判定される三角ポリゴンを指定する
      - 確認内容
        - 左判定回数1が返却されること
    """
    # 衝突点（3次元）
    cross_point = Point([3., 3., 6.])
    #  同グループの三角ポリゴンのrectangular_triangles内のインデックス
    group_triangle_indexes = {0, 1, 2, 3}
    # 衝突点（3次元）と同グループの三角ポリゴンのrectangular_triangles内のインデックスのタプル配列
    cross_point_infos = [(cross_point, group_triangle_indexes)]
    # 衝突点が通過する直線L（2次元）
    xy_cross_line = Line.from_points([1., 1.], [5., 5.])
    # 全三角ポリゴン
    rectangular_triangles = [
        Triangle([3., 3., 6.], [5., 5., 1.], [1., 1., 1.]),
        Triangle([3., 3., 6.], [5., 5., 1.], [3., 7., 1.]),
        Triangle([3., 3., 6.], [3., 7., 1.], [4., 8., 1.]),
        Triangle([3., 3., 6.], [4., 8., 1.], [1., 1., 1.])
    ]

    ## 期待値
    # 左判定回数
    left_count_assert = 1
    # 右判定回数
    right_count_assert = 0

    # 試験実施
    left_count, right_count = _count_vertex_side(
        cross_point_infos,
        xy_cross_line,
        rectangular_triangles
    )

    assert left_count == left_count_assert
    assert right_count == right_count_assert


def test__count_vertex_side_05():
    """頂点に対する左右判定(直線Lの後だけに交点CXが存在)
    + 試験詳細
      - 入力条件
        - 衝突点に対して直線Lの後だけに交点CXが存在する辺を持つ三角ポリゴンを指定する
      - 確認内容
        - 左判定回数・右判定回数0が返却されること
    """
    # 衝突点（3次元）
    cross_point = Point([3., 3., 6.])
    #  同グループの三角ポリゴンのrectangular_triangles内のインデックス
    group_triangle_indexes = {0, 1, 2}
    # 衝突点（3次元）と同グループの三角ポリゴンのrectangular_triangles内のインデックスのタプル配列
    cross_point_infos = [(cross_point, group_triangle_indexes)]
    # 衝突点が通過する直線L（2次元）
    xy_cross_line = Line.from_points([3., 3.], [5., 5.])
    # 全三角ポリゴン
    rectangular_triangles = [
        Triangle([3., 3., 6.], [5., 5., 6.], [5., 1., 6.]),
        Triangle([3., 3., 6.], [3., 3., 1.], [5., 5., 6.]),
        Triangle([3., 3., 6.], [3., 3., 1.], [5., 1., 6.]),
    ]

    ## 期待値
    # 左判定回数
    left_count_assert = 0
    # 右判定回数
    right_count_assert = 0

    # 試験実施
    left_count, right_count = _count_vertex_side(
        cross_point_infos,
        xy_cross_line,
        rectangular_triangles
    )

    assert left_count == left_count_assert
    assert right_count == right_count_assert


def test__count_vertex_side_06():
    """頂点に対する左右判定(前後判定の線が連結しない)
    | 上記パターンは実際の運用時は発生しない
    + 試験詳細
      - 入力条件
        - 衝突点に対して直線Lに前後判定の線が連結しない辺を持つ三角ポリゴンを指定する
      - 確認内容
        - 左判定回数・右判定回数0が返却されること
    """
    # 衝突点（3次元）
    cross_point = Point([3., 3., 6.])
    #  同グループの三角ポリゴンのrectangular_triangles内のインデックス
    group_triangle_indexes = {0, 1, 3}
    # 衝突点（3次元）と同グループの三角ポリゴンのrectangular_triangles内のインデックスのタプル配列
    cross_point_infos = [(cross_point, group_triangle_indexes)]
    # 衝突点が通過する直線L（2次元）
    xy_cross_line = Line.from_points([1., 1.], [5., 5.])
    # 全三角ポリゴン
    rectangular_triangles = [
        Triangle([3., 3., 6.], [5., 5., 1.], [1., 1., 1.]),
        Triangle([3., 3., 6.], [5., 5., 1.], [3., 7., 1.]),
        Triangle([3., 3., 6.], [3., 7., 1.], [4., 8., 1.]),
        Triangle([3., 3., 6.], [4., 8., 1.], [1., 1., 1.])
    ]

    ## 期待値
    # 左判定回数
    left_count_assert = 0
    # 右判定回数
    right_count_assert = 0

    # 試験実施
    left_count, right_count = _count_vertex_side(
        cross_point_infos,
        xy_cross_line,
        rectangular_triangles
    )

    assert left_count == left_count_assert
    assert right_count == right_count_assert


def test__count_vertex_side_07():
    """頂点に対する左右判定(衝突点が含まれない辺が1つに定まらない)
    + 試験詳細
      - 入力条件
        - 衝突点に対して衝突点が含まれない辺が1つに定まらない三角ポリゴンを指定する
      - 確認内容
        - 左判定回数・右判定回数0が返却されること
    """
    # 衝突点（3次元）
    cross_point = Point([3., 3., 6.])
    #  同グループの三角ポリゴンのrectangular_triangles内のインデックス
    group_triangle_indexes = {0, 1, 2, 3}
    # 衝突点（3次元）と同グループの三角ポリゴンのrectangular_triangles内のインデックスのタプル配列
    cross_point_infos = [(cross_point, group_triangle_indexes)]
    # 衝突点が通過する直線L（2次元）
    xy_cross_line = Line.from_points([1., 1.], [5., 5.])
    # 全三角ポリゴン
    rectangular_triangles = [
        Triangle([3., 3., 6.], [5., 5., 1.], [1., 1., 1.]),
        Triangle([3., 3., 7.], [5., 5., 1.], [3., 7., 1.]),
        Triangle([3., 3., 6.], [3., 7., 1.], [4., 8., 1.]),
        Triangle([3., 3., 6.], [4., 8., 1.], [1., 1., 1.])
    ]

    ## 期待値
    # 左判定回数
    left_count_assert = 0
    # 右判定回数
    right_count_assert = 0

    # 試験実施
    left_count, right_count = _count_vertex_side(
        cross_point_infos,
        xy_cross_line,
        rectangular_triangles
    )

    assert left_count == left_count_assert
    assert right_count == right_count_assert


def test__count_vertex_side_08():
    """頂点に対する左右判定(直線LがX軸に平行)
    + 試験詳細
      - 入力条件
        - 衝突点に対して左と判定される三角ポリゴンを指定する
        - 直線LがX軸に平行になるように指定する
      - 確認内容
        - 左判定回数1が返却されること
    """
    # 衝突点（3次元）
    cross_point = Point([3., 3., 6.])
    #  同グループの三角ポリゴンのrectangular_triangles内のインデックス
    group_triangle_indexes = {0, 1, 2}
    # 衝突点（3次元）と同グループの三角ポリゴンのrectangular_triangles内のインデックスのタプル配列
    cross_point_infos = [(cross_point, group_triangle_indexes)]
    # 衝突点が通過する直線L（2次元）
    xy_cross_line = Line.from_points([3., 1.], [3., 5.])
    # 全三角ポリゴン
    rectangular_triangles = [
        Triangle([3., 3., 6.], [3., 5., 1.], [3., 1., 6.]),
        Triangle([3., 3., 6.], [3., 5., 1.], [1., 3., 1.]),
        Triangle([3., 3., 6.], [1., 3., 1.], [3., 1., 1.])
    ]

    ## 期待値
    # 左判定回数
    left_count_assert = 1
    # 右判定回数
    right_count_assert = 0

    # 試験実施
    left_count, right_count = _count_vertex_side(
        cross_point_infos,
        xy_cross_line,
        rectangular_triangles
    )

    assert left_count == left_count_assert
    assert right_count == right_count_assert


def test__count_vertex_side_09():
    """頂点に対する左右判定(直線LがY軸に平行)
    + 試験詳細
      - 入力条件
        - 衝突点に対して左と判定される三角ポリゴンを指定する
        - 直線LがY軸に平行になるように指定する
      - 確認内容
        - 左判定回数1が返却されること
    """
    # 衝突点（3次元）
    cross_point = Point([3., 3., 6.])
    #  同グループの三角ポリゴンのrectangular_triangles内のインデックス
    group_triangle_indexes = {0, 1, 2}
    # 衝突点（3次元）と同グループの三角ポリゴンのrectangular_triangles内のインデックスのタプル配列
    cross_point_infos = [(cross_point, group_triangle_indexes)]
    # 衝突点が通過する直線L（2次元）
    xy_cross_line = Line.from_points([1., 3.], [5., 3.])
    # 全三角ポリゴン
    rectangular_triangles = [
        Triangle([3., 3., 6.], [5., 3., 1.], [1., 3., 6.]),
        Triangle([3., 3., 6.], [5., 3., 1.], [3., 5., 1.]),
        Triangle([3., 3., 6.], [3., 5., 1.], [1., 3., 1.])
    ]

    ## 期待値
    # 左判定回数
    left_count_assert = 1
    # 右判定回数
    right_count_assert = 0

    # 試験実施
    left_count, right_count = _count_vertex_side(
        cross_point_infos,
        xy_cross_line,
        rectangular_triangles
    )

    assert left_count == left_count_assert
    assert right_count == right_count_assert


def test__count_vertex_side_10():
    """頂点に対する左右判定(直線Lの左右に判定対象点存在)
    + 試験詳細
      - 入力条件
        - 衝突点に対して左右両方と判定される三角ポリゴンを指定する
      - 確認内容
        - 左判定回数・右判定回数1が返却されること
    """
    # 衝突点（3次元）
    cross_point = Point([3., 3., 6.])
    #  同グループの三角ポリゴンのrectangular_triangles内のインデックス
    group_triangle_indexes = {0, 1, 2, 3, 4}
    # 衝突点（3次元）と同グループの三角ポリゴンのrectangular_triangles内のインデックスのタプル配列
    cross_point_infos = [(cross_point, group_triangle_indexes)]
    # 衝突点が通過する直線L（2次元）
    xy_cross_line = Line.from_points([3., 3.], [5., 5.])
    # 全三角ポリゴン
    rectangular_triangles = [
        Triangle([3., 3., 6.], [1., 3., 6.], [5., 5., 6.]),
        Triangle([3., 3., 6.], [1., 3., 6.], [5., 1., 6.]),
        Triangle([3., 3., 6.], [4., 4., 10.], [5., 5., 6.]),
        Triangle([3., 3., 6.], [4., 2., 10.], [5., 1., 6.]),
        Triangle([3., 3., 6.], [4., 4., 10.], [4., 2., 10.]),
    ]

    ## 期待値
    # 左判定回数
    left_count_assert = 1
    # 右判定回数
    right_count_assert = 1

    # 試験実施
    left_count, right_count = _count_vertex_side(
        cross_point_infos,
        xy_cross_line,
        rectangular_triangles
    )

    assert left_count == left_count_assert
    assert right_count == right_count_assert


def test__count_vertex_side_11():
    """頂点に対する左右判定(衝突点が複数)
    + 試験詳細
      - 入力条件
        - 衝突点に対して左右両方と判定される三角ポリゴンを指定する
        - 衝突点を複数指定する
      - 確認内容
        - 左判定回数・右判定回数1が返却されること
    """
    # 衝突点（3次元）
    cross_point1 = Point([3., 3., 6.])
    cross_point2 = Point([3., 3., 1.])
    #  同グループの三角ポリゴンのrectangular_triangles内のインデックス
    group_triangle_indexes1 = {0, 1, 3, 4}
    group_triangle_indexes2 = {0, 1, 2}
    # 衝突点（3次元）と同グループの三角ポリゴンのrectangular_triangles内のインデックスのタプル配列
    cross_point_infos = [
        (cross_point1, group_triangle_indexes1),
        (cross_point2, group_triangle_indexes2),
    ]
    # 衝突点が通過する直線L（2次元）
    xy_cross_line = Line.from_points([3., 3.], [5., 5.])
    # 全三角ポリゴン
    rectangular_triangles = [
        Triangle([3., 3., 6.], [3., 3, 1.], [5., 5., 6.]),
        Triangle([3., 3., 6.], [3., 3, 1.], [5., 1., 6.]),
        Triangle([3., 3., 1.], [5., 1., 6.], [5., 5., 6.]),
        Triangle([3., 3., 6.], [1., 3., 6.], [5., 1., 6.]),
        Triangle([3., 3., 6.], [1., 3., 6.], [5., 5., 6.]),
    ]

    ## 期待値
    # 左判定回数
    left_count_assert = 1
    # 右判定回数
    right_count_assert = 1

    # 試験実施
    left_count, right_count = _count_vertex_side(
        cross_point_infos,
        xy_cross_line,
        rectangular_triangles
    )

    assert left_count == left_count_assert
    assert right_count == right_count_assert


def test__check_cross_one_edge_01():
    """1辺との衝突判定(衝突内部)
    + 試験詳細
      - 入力条件
        - 衝突内部と判定される辺と三角ポリゴンを指定する
      - 確認内容
        - Trueが返却されること
    """
    # 衝突点が通過する辺（2次元）
    xy_cross_edge = Line.from_points([3, 3], [5, 5])
    # 同グループの三角ポリゴンのrectangular_triangles内のインデックス
    group_triangle_indexes = {0, 1}
    # 全三角ポリゴン
    rectangular_triangles = [
        Triangle([3, 3, 1], [5, 5, 2], [4, 5, 3]),
        Triangle([3, 3, 1], [5, 5, 2], [4, 2, 3]),
    ]

    # 試験実施
    collision_result  = _check_cross_one_edge(
        xy_cross_edge,
        group_triangle_indexes,
        rectangular_triangles,
    )

    # 戻り値確認
    assert collision_result


def test__check_cross_one_edge_02():
    """1辺との衝突判定(衝突外部)
    + 試験詳細
      - 入力条件
        - 衝突外部と判定される辺と三角ポリゴンを指定する
      - 確認内容
        - Falseが返却されること
    """
    # 衝突点が通過する辺（2次元）
    xy_cross_edge = Line.from_points([3, 3], [5, 5])
    # 同グループの三角ポリゴンのrectangular_triangles内のインデックス
    group_triangle_indexes = {0, 1}
    # 全三角ポリゴン
    rectangular_triangles = [
        Triangle([3, 3, 1], [5, 5, 2], [4, 5, 3]),
        Triangle([3, 3, 1], [5, 5, 2], [4, 6, 7]),
    ]

    # 試験実施
    collision_result  = _check_cross_one_edge(
        xy_cross_edge,
        group_triangle_indexes,
        rectangular_triangles,
    )

    # 戻り値確認
    assert not collision_result


def test__check_cross_two_edges_01():
    """2辺との衝突判定(衝突内部)
    + 試験詳細
      - 入力条件
        - 衝突内部と判定される辺と三角ポリゴンを指定する
        - 三角ポリゴンのグループを3つ用意する
      - 確認内容
        - Trueが返却されること
    """
    # 衝突点が通過する辺（2次元）
    xy_cross_edge = Line.from_points([3, 3], [5, 5])
    # 共通の三角ポリゴンがあるグループ群の中で衝突点の高さが最大の三角ポリゴングループのrectangular_triangles内のインデックス
    max_group_triangle_indexes = {0, 1}
    # 共通の三角ポリゴンがあるグループ群の中で衝突点の高さが最小の三角ポリゴングループのrectangular_triangles内のインデックス
    min_group_triangle_indexes = {2, 3}
    # 全三角ポリゴン
    rectangular_triangles = [
        Triangle([3, 3, 1], [5, 5, 2], [4, 5, 3]),
        Triangle([3, 3, 1], [5, 5, 2], [4, 4, 3]),
        Triangle([3, 3, 1], [3, 3, 4], [4, 4, 3]),
        Triangle([4, 4, 3], [3, 3, 4], [4, 2, 7]),
    ]

    # 試験実施
    collision_result  = _check_cross_two_edges(
        xy_cross_edge,
        max_group_triangle_indexes,
        min_group_triangle_indexes,
        rectangular_triangles,
    )

    # 戻り値確認
    assert collision_result


def test__check_cross_two_edges_02():
    """2辺との衝突判定(衝突外部)
    + 試験詳細
      - 入力条件
        - 衝突外部と判定される辺と三角ポリゴンを指定する
        - 三角ポリゴンのグループを3つ用意する
      - 確認内容
        - Falseが返却されること
    """
    # 衝突点が通過する辺（2次元）
    xy_cross_edge = Line.from_points([3, 3], [5, 5])
    # 共通の三角ポリゴンがあるグループ群の中で衝突点の高さが最大の三角ポリゴングループのrectangular_triangles内のインデックス
    max_group_triangle_indexes = {0, 1}
    # 共通の三角ポリゴンがあるグループ群の中で衝突点の高さが最小の三角ポリゴングループのrectangular_triangles内のインデックス
    min_group_triangle_indexes = {2, 3}
    # 全三角ポリゴン
    rectangular_triangles = [
        Triangle([3, 3, 1], [5, 5, 2], [4, 5, 3]),
        Triangle([3, 3, 1], [5, 5, 2], [4, 4, 3]),
        Triangle([3, 3, 1], [3, 3, 4], [4, 4, 3]),
        Triangle([4, 4, 3], [3, 3, 4], [4, 6, 7]),
    ]

    # 試験実施
    collision_result  = _check_cross_two_edges(
        xy_cross_edge,
        max_group_triangle_indexes,
        min_group_triangle_indexes,
        rectangular_triangles,
    )

    # 戻り値確認
    assert not collision_result


def test__check_cross_vertex_and_edge_01():
    """1辺1頂点との衝突判定(衝突内部)
    + 試験詳細
      - 入力条件
        - 衝突内部と判定される辺・頂点と三角ポリゴンを指定する
      - 確認内容
        - Trueが返却されること
    """
    # 衝突点（3次元）
    cross_point = Point([3., 3., 6.])
    # 衝突点が通過する辺（2次元）
    xy_cross_edge = Line.from_points([1, 1], [5, 5])
    # 共通の三角ポリゴンがあるグループ群の中で衝突点が頂点の三角ポリゴングループのrectangular_triangles内のインデックス
    vertex_group_triangle_indexes = {0, 1, 2}
    # 共通の三角ポリゴンがあるグループ群の中で衝突点がが辺の三角ポリゴングループのrectangular_triangles内のインデックス
    edge_group_triangle_indexes = {0, 3}
    # 全三角ポリゴン
    rectangular_triangles = [
        Triangle([3., 3., 6.], [5., 5., 2.], [1., 1., 2.]),
        Triangle([3., 3., 6.], [5., 5., 2.], [5., 3., 1.]),
        Triangle([3., 3., 6.], [1., 1., 2.], [5., 3., 1.]),
        Triangle([5., 5., 2.], [1., 1., 2.], [5., 7., 1.]),
    ]

    # 試験実施
    collision_result  = _check_cross_vertex_and_edge(
        cross_point,
        xy_cross_edge,
        vertex_group_triangle_indexes,
        edge_group_triangle_indexes,
        rectangular_triangles,
    )

    # 戻り値確認
    assert collision_result


def test__check_cross_vertex_and_edge_02():
    """1辺1頂点との衝突判定(衝突外部)
    + 試験詳細
      - 入力条件
        - 衝突外部と判定される辺・頂点と三角ポリゴンを指定する
      - 確認内容
        - Falseが返却されること
    """
    # 衝突点（3次元）
    cross_point = Point([3., 3., 6.])
    # 衝突点が通過する辺（2次元）
    xy_cross_edge = Line.from_points([1, 1], [5, 5])
    # 共通の三角ポリゴンがあるグループ群の中で衝突点が頂点の三角ポリゴングループのrectangular_triangles内のインデックス
    vertex_group_triangle_indexes = {0, 1, 2}
    # 共通の三角ポリゴンがあるグループ群の中で衝突点がが辺の三角ポリゴングループのrectangular_triangles内のインデックス
    edge_group_triangle_indexes = {0, 3}
    # 全三角ポリゴン
    rectangular_triangles = [
        Triangle([3., 3., 6.], [5., 5., 2.], [1., 1., 2.]),
        Triangle([3., 3., 6.], [5., 5., 2.], [5., 3., 1.]),
        Triangle([3., 3., 6.], [1., 1., 2.], [5., 3., 1.]),
        Triangle([5., 5., 2.], [1., 1., 2.], [5., 3., 1.]),
    ]

    # 試験実施
    collision_result  = _check_cross_vertex_and_edge(
        cross_point,
        xy_cross_edge,
        vertex_group_triangle_indexes,
        edge_group_triangle_indexes,
        rectangular_triangles,
    )

    # 戻り値確認
    assert not collision_result


def test__check_cross_two_vertexes_01():
    """2頂点との衝突判定(衝突内部)
    + 試験詳細
      - 入力条件
        - 衝突内部と判定される2頂点と三角ポリゴンを指定する
      - 確認内容
        - Trueが返却されること
    """
    # 最大高さの衝突点
    max_cross_point = Point([3., 3., 6.])
    # 最小高さの衝突点
    min_cross_point = Point([3., 3., 1.])
    # 衝突点が通過する辺（2次元）
    xy_cross_edge = Line.from_points([3, 3], [5, 5])
    # 共通の三角ポリゴンがあるグループ群の中で衝突点の高さが最大の三角ポリゴングループのrectangular_triangles内のインデックス
    max_group_triangle_indexes = {0, 1, 2, 3, 4, 5, 6}
    # 共通の三角ポリゴンがあるグループ群の中で衝突点の高さが最小の三角ポリゴングループのrectangular_triangles内のインデックス
    min_group_triangle_indexes = {0, 1, 7, 8}
    # 全三角ポリゴン
    rectangular_triangles = [
        Triangle([3., 3., 6.], [3., 3., 1.], [5., 5., 6.]),
        Triangle([3., 3., 6.], [3., 3., 1.], [5., 1., 6.]),
        Triangle([3., 3., 6.], [1., 3., 6.], [5., 5., 6.]),
        Triangle([3., 3., 6.], [1., 3., 6.], [5., 1., 6.]),
        Triangle([3., 3., 6.], [4., 4., 10.], [5., 5., 6.]),
        Triangle([3., 3., 6.], [4., 2., 10.], [5., 1., 6.]),
        Triangle([3., 3., 6.], [4., 4., 10.], [4., 2., 10.]),
        Triangle([1., 3., 6.], [3., 3., 1.], [5., 5., 6.]),
        Triangle([1., 3., 6.], [3., 3., 1.], [5., 1., 6.]),
    ]

    # 試験実施
    collision_result  = _check_cross_two_vertexes(
        max_cross_point,
        min_cross_point,
        xy_cross_edge,
        max_group_triangle_indexes,
        min_group_triangle_indexes,
        rectangular_triangles,
    )

    # 戻り値確認
    assert collision_result


def test__check_cross_two_vertexes_02():
    """2頂点との衝突判定(衝突外部)
    + 試験詳細
      - 入力条件
        - 衝突外部と判定される2頂点と三角ポリゴンを指定する
      - 確認内容
        - Falseが返却されること
    """
    # 最大高さの衝突点
    max_cross_point = Point([3., 3., 6.])
    # 最小高さの衝突点
    min_cross_point = Point([3., 3., 1.])
    # 衝突点が通過する辺（2次元）
    xy_cross_edge = Line.from_points([3, 3], [5, 5])
    # 共通の三角ポリゴンがあるグループ群の中で衝突点の高さが最大の三角ポリゴングループのrectangular_triangles内のインデックス
    max_group_triangle_indexes = {0, 1, 2, 3, 4, 5, 6}
    # 共通の三角ポリゴンがあるグループ群の中で衝突点の高さが最小の三角ポリゴングループのrectangular_triangles内のインデックス
    min_group_triangle_indexes = {0, 1, 7, 8}
    # 全三角ポリゴン
    rectangular_triangles = [
        Triangle([3., 3., 6.], [3., 3., 1.], [5., 5., 6.]),
        Triangle([3., 3., 6.], [3., 3., 1.], [5., 1., 6.]),
        Triangle([3., 3., 6.], [1., 3., 6.], [5., 5., 6.]),
        Triangle([3., 3., 6.], [1., 3., 6.], [5., 1., 6.]),
        Triangle([3., 3., 6.], [2., 4., 10.], [5., 5., 6.]),
        Triangle([3., 3., 6.], [2., 2., 10.], [5., 1., 6.]),
        Triangle([3., 3., 6.], [2., 4., 10.], [2., 2., 10.]),
        Triangle([1., 3., 6.], [3., 3., 1.], [5., 5., 6.]),
        Triangle([1., 3., 6.], [3., 3., 1.], [5., 1., 6.]),
    ]

    # 試験実施
    collision_result  = _check_cross_two_vertexes(
        max_cross_point,
        min_cross_point,
        xy_cross_edge,
        max_group_triangle_indexes,
        min_group_triangle_indexes,
        rectangular_triangles,
    )

    # 戻り値確認
    assert not collision_result


def test__check_cross_vertex_and_edge_02():
    """1辺1頂点との衝突判定(衝突外部)
    + 試験詳細
      - 入力条件
        - 衝突外部と判定される辺・頂点と三角ポリゴンを指定する
      - 確認内容
        - Falseが返却されること
    """
    # 衝突点（3次元）
    cross_point = Point([3., 3., 6.])
    # 衝突点が通過する辺（2次元）
    xy_cross_edge = Line.from_points([1, 1], [5, 5])
    # 共通の三角ポリゴンがあるグループ群の中で衝突点が頂点の三角ポリゴングループのrectangular_triangles内のインデックス
    vertex_group_triangle_indexes = {0, 1, 2}
    # 共通の三角ポリゴンがあるグループ群の中で衝突点がが辺の三角ポリゴングループのrectangular_triangles内のインデックス
    edge_group_triangle_indexes = {0, 3}
    # 全三角ポリゴン
    rectangular_triangles = [
        Triangle([3., 3., 6.], [5., 5., 2.], [1., 1., 2.]),
        Triangle([3., 3., 6.], [5., 5., 2.], [5., 3., 1.]),
        Triangle([3., 3., 6.], [1., 1., 2.], [5., 3., 1.]),
        Triangle([5., 5., 2.], [1., 1., 2.], [5., 3., 1.]),
    ]

    # 試験実施
    collision_result  = _check_cross_vertex_and_edge(
        cross_point,
        xy_cross_edge,
        vertex_group_triangle_indexes,
        edge_group_triangle_indexes,
        rectangular_triangles,
    )


def test__check_cross_one_vertex_01():
    """1頂点との衝突判定(衝突内部)
    + 試験詳細
      - 入力条件
        - 衝突内部と判定される1頂点と三角ポリゴンを指定する
      - 確認内容
        - Trueが返却されること
    """
    # 衝突点（3次元）
    cross_point = Point([3., 2., 6.])
    # 衝突点の三角ポリゴングループのrectangular_triangles内のインデックス
    group_triangle_indexes = {0, 1, 2}
    # 全三角ポリゴン
    rectangular_triangles = [
        Triangle([3., 2., 6.], [5., 5., 2.], [5., 1., 2.]),
        Triangle([3., 2., 6.], [5., 5., 2.], [1., 1., 1.]),
        Triangle([3., 2., 6.], [1., 1., 1.], [5., 1., 2.]),
    ]

    # 試験実施
    collision_result  = _check_cross_one_vertex(
        cross_point,
        group_triangle_indexes,
        rectangular_triangles,
    )

    # 戻り値確認
    assert collision_result


def test__check_cross_one_vertex_02():
    """1頂点との衝突判定(衝突外部)
    + 試験詳細
      - 入力条件
        - 衝突外部と判定される1頂点と三角ポリゴンを指定する
      - 確認内容
        - Falseが返却されること
    """
    # 衝突点（3次元）
    cross_point = Point([3., 4., 6.])
    # 衝突点の三角ポリゴングループのrectangular_triangles内のインデックス
    group_triangle_indexes = {0, 1, 2}
    # 全三角ポリゴン
    rectangular_triangles = [
        Triangle([3., 4., 6.], [5., 5., 2.], [5., 1., 2.]),
        Triangle([3., 4., 6.], [5., 5., 2.], [1., 1., 1.]),
        Triangle([3., 4., 6.], [1., 1., 1.], [5., 1., 2.]),
    ]

    # 試験実施
    collision_result  = _check_cross_one_vertex(
        cross_point,
        group_triangle_indexes,
        rectangular_triangles,
    )

    # 戻り値確認
    assert not collision_result


def test__check_cross_one_vertex_03():
    """1頂点との衝突判定(衝突外部)
    + 試験詳細
      - 入力条件
        - 衝突点ではない辺の長さが極めて小さい三角ポリゴン
      - 確認内容
        - Falseが返却されること
    """
    # 衝突点（3次元）
    cross_point = Point([3., 4., 6.])
    # 衝突点の三角ポリゴングループのrectangular_triangles内のインデックス
    group_triangle_indexes = {0, 1, 2, 3}
    # 全三角ポリゴン
    rectangular_triangles = [
        Triangle([3., 4., 6.], [5., 5., 2.], [5., 1., 2.]),
        Triangle([3., 4., 6.], [5., 5., 2.], [1., 1., 1.]),
        Triangle([3., 4., 6.], [1., 1., 1.], [5., 1., 2.]),
        Triangle([3., 4., 6.], [3., 4. + 1e-11, 6.], [3., 4., 6. + 1e-11]),
    ]

    # 試験実施
    collision_result  = _check_cross_one_vertex(
        cross_point,
        group_triangle_indexes,
        rectangular_triangles,
    )

    # 戻り値確認
    assert not collision_result


def test__make_triangle_group_01():
    """ 三角ポリゴングループ作成・更新 (頂点のグループ)
    + 試験詳細
      - 入力条件
        - 頂点の三角ポリゴンの三角ポリゴン2グループを入力に渡す
      - 確認内容
        - 頂点の三角ポリゴングループが返却されること
    """
    # 衝突点（3次元）
    cross_point_cordinate_result = (Point([3., 3., 6.]), None, "a")
    # 衝突頂点辞書
    vertex_dict = dict()
    # 衝突辺辞書
    edge_dict = dict()
    # 衝突点の辺の三角ポリゴングループのrectangular_triangles内のインデックス
    edge_triangle_indexes = dict()
    # 衝突点の頂点の三角ポリゴングループのrectangular_triangles内のインデックス
    vertex_triangle_indexes = dict()

    cross_group_index = dict()
    # 衝突点の三角ポリゴンのインデックス
    triangle_index = 0
    # 全三角ポリゴン
    rectangular_triangles = [
        Triangle([3., 3., 6.], [5., 5., 2.], [5., 1., 2.]),
        Triangle([3., 3., 6.], [5., 5., 2.], [1., 1., 1.]),
        Triangle([3., 3., 6.], [1., 1., 1.], [5., 1., 2.]),
        Triangle([3., 3., 6.], [5., 5., 10.], [5., 1., 10.]),
        Triangle([3., 3., 6.], [5., 5., 10.], [1., 1., 11.]),
        Triangle([3., 3., 6.], [1., 1., 11.], [5., 1., 10.]),
    ]

    ## 期待値
    # 衝突頂点辞書
    vertex_dict_assert = {"3.0000000000_3.0000000000_6.0000000000": Point([3., 3., 6.])}
    # 衝突辺辞書
    edge_dict_assert = dict()
    edge_triangle_indexes_assert = dict()
    vertex_triangle_indexes_assert = {
        "3.0000000000_3.0000000000_6.0000000000%1": {0, 1, 2},
        "3.0000000000_3.0000000000_6.0000000000%2": {3, 4, 5}
    }

    # 試験実施
    _make_triangle_group(
        cross_point_cordinate_result,
        vertex_dict,
        edge_dict,
        edge_triangle_indexes,
        vertex_triangle_indexes,
        cross_group_index,
        triangle_index,
        rectangular_triangles,
    )

    ## 戻り値確認
    assert len(vertex_dict) == len(vertex_dict_assert)
    for key in vertex_dict:
        assert (vertex_dict[key] == vertex_dict_assert[key]).all()
    assert len(edge_dict) == len(edge_dict_assert)
    for key in edge_dict:
        assert (edge_dict[key].point == edge_dict_assert[key].point).all()
        assert (edge_dict[key].direction == edge_dict_assert[key].direction).all()
    assert edge_triangle_indexes == edge_triangle_indexes_assert
    assert vertex_triangle_indexes == vertex_triangle_indexes_assert


def test__make_triangle_group_02():
    """ 三角ポリゴングループ作成・更新 (登録済み頂点のグループ)
    + 試験詳細
      - 入力条件
        - 衝突点を頂点とする
        - 頂点の三角ポリゴンの三角ポリゴン2グループを入力に渡す
        - 上記三角ポリゴン2グループが既にグループ化済みとする
      - 確認内容
        - 頂点の三角ポリゴングループが更新されずにが返却されること
    """
    # 衝突点（3次元）
    cross_point_cordinate_result = (Point([3., 3., 6.]), None, "a")
    # 衝突頂点辞書
    vertex_dict = {"3.0000000000_3.0000000000_6.0000000000": Point([3., 3., 6.])}
    # 衝突辺辞書
    edge_dict = dict()
    # 衝突点の辺の三角ポリゴングループのrectangular_triangles内のインデックス
    edge_triangle_indexes = dict()
    # 衝突点の頂点の三角ポリゴングループのrectangular_triangles内のインデックス
    vertex_triangle_indexes = {
        "3.0_3.0_6.0%0": {0, 1, 2},
        "3.0_3.0_6.0%1": {3, 4, 5}
    }
    cross_group_index=dict()
    # 衝突点の三角ポリゴンのインデックス
    triangle_index = 1
    # 全三角ポリゴン
    rectangular_triangles = [
        Triangle([3., 3., 6.], [5., 5., 2.], [5., 1., 2.]),
        Triangle([3., 3., 6.], [5., 5., 2.], [1., 1., 1.]),
        Triangle([3., 3., 6.], [1., 1., 1.], [5., 1., 2.]),
        Triangle([3., 3., 6.], [5., 5., 10.], [5., 1., 10.]),
        Triangle([3., 3., 6.], [5., 5., 10.], [1., 1., 11.]),
        Triangle([3., 3., 6.], [1., 1., 11.], [5., 1., 10.]),
    ]

    ## 期待値
    # 衝突頂点辞書
    vertex_dict_assert = {"3.0000000000_3.0000000000_6.0000000000": Point([3., 3., 6.])}
    # 衝突辺辞書
    edge_dict_assert = dict()
    edge_triangle_indexes_assert = dict()
    vertex_triangle_indexes_assert = {
        "3.0_3.0_6.0%0": {0, 1, 2},
        "3.0_3.0_6.0%1": {3, 4, 5}
    }

    # 試験実施
    _make_triangle_group(
        cross_point_cordinate_result,
        vertex_dict,
        edge_dict,
        edge_triangle_indexes,
        vertex_triangle_indexes,
        cross_group_index,
        triangle_index,
        rectangular_triangles,
    )

    ## 戻り値確認
    assert len(vertex_dict) == len(vertex_dict_assert)
    for key in vertex_dict:
        assert (vertex_dict[key] == vertex_dict_assert[key]).all()
    assert len(edge_dict) == len(edge_dict_assert)
    for key in edge_dict:
        assert (edge_dict[key].point == edge_dict_assert[key].point).all()
        assert (edge_dict[key].direction == edge_dict_assert[key].direction).all()
    assert edge_triangle_indexes == edge_triangle_indexes_assert
    assert vertex_triangle_indexes == vertex_triangle_indexes_assert


def test__make_triangle_group_03():
    """ 三角ポリゴングループ作成・更新 (面のグループ)
    + 試験詳細
      - 入力条件
        - 衝突点を面とする
        - 頂点の三角ポリゴンの三角ポリゴン2グループを入力に渡す
      - 確認内容
        - 三角ポリゴングループが更新されずに返却されること
    """
    # 衝突点（3次元）
    cross_point_cordinate_result = (Point([3., 2., 6.]), None, None)
    # 衝突頂点辞書
    vertex_dict = dict()
    # 衝突辺辞書
    edge_dict = dict()
    # 衝突点の辺の三角ポリゴングループのrectangular_triangles内のインデックス
    edge_triangle_indexes = dict()
    # 衝突点の頂点の三角ポリゴングループのrectangular_triangles内のインデックス
    vertex_triangle_indexes = dict()
    cross_group_index = dict()
    # 衝突点の三角ポリゴンのインデックス
    triangle_index = 0
    # 全三角ポリゴン
    rectangular_triangles = [
        Triangle([3., 3., 6.], [5., 5., 2.], [5., 1., 2.]),
        Triangle([3., 3., 6.], [5., 5., 2.], [1., 1., 1.]),
        Triangle([3., 3., 6.], [1., 1., 1.], [5., 1., 2.]),
        Triangle([3., 3., 6.], [5., 5., 10.], [5., 1., 10.]),
        Triangle([3., 3., 6.], [5., 5., 10.], [1., 1., 11.]),
        Triangle([3., 3., 6.], [1., 1., 11.], [5., 1., 10.]),
    ]

    ## 期待値
    # 衝突頂点辞書
    vertex_dict_assert = dict()
    # 衝突辺辞書
    edge_dict_assert = dict()
    edge_triangle_indexes_assert = dict()
    vertex_triangle_indexes_assert = dict()

    # 試験実施
    _make_triangle_group(
        cross_point_cordinate_result,
        vertex_dict,
        edge_dict,
        edge_triangle_indexes,
        vertex_triangle_indexes,
        cross_group_index,
        triangle_index,
        rectangular_triangles,
    )

    ## 戻り値確認
    assert len(vertex_dict) == len(vertex_dict_assert)
    for key in vertex_dict:
        assert (vertex_dict[key] == vertex_dict_assert[key]).all()
    assert len(edge_dict) == len(edge_dict_assert)
    for key in edge_dict:
        assert (edge_dict[key].point == edge_dict_assert[key].point).all()
        assert (edge_dict[key].direction == edge_dict_assert[key].direction).all()
    assert edge_triangle_indexes == edge_triangle_indexes_assert
    assert vertex_triangle_indexes == vertex_triangle_indexes_assert


def test__make_triangle_group_04():
    """ 三角ポリゴングループ作成・更新 (辺のグループ)
    + 試験詳細
      - 入力条件
        - 辺の三角ポリゴンの三角ポリゴン2グループを入力に渡す
      - 確認内容
        - 辺の三角ポリゴングループが返却されること
    """
    # 衝突点（3次元）
    cross_point_cordinate_result = (Point([3., 3., 6.]), "c", None)
    # 衝突頂点辞書
    vertex_dict = dict()
    # 衝突辺辞書
    edge_list = list()
    # 衝突点の辺の三角ポリゴングループのrectangular_triangles内のインデックス
    edge_triangle_indexes = dict()
    # 衝突点の頂点の三角ポリゴングループのrectangular_triangles内のインデックス
    vertex_triangle_indexes = dict()
    cross_group_index = dict()
    # 衝突点の三角ポリゴンのインデックス
    triangle_index = 0
    # 全三角ポリゴン
    rectangular_triangles = [
        Triangle([1., 1., 2.], [5., 5., 2.], [3., 1., 6.]),
        Triangle([1., 1., 2.], [5., 5., 2.], [4., 3., 10.]),
        Triangle([1., 1., 2.], [4., 3., 10.], [3., 1., 6.]),
        Triangle([3., 1., 6.], [5., 5., 2.], [4., 3., 10.]),
        Triangle([1., 1., 2.], [5., 5., 2.], [3., 7., -1.]),
        Triangle([1., 1., 2.], [5., 5., 2.], [2., 9., -2.]),
        Triangle([1., 1., 2.], [2., 9., -2.], [3., 7., -1.]),
        Triangle([3., 7., -1.], [5., 5., 2.], [2., 9., -2.]),
    ]

    ## 期待値
    # 衝突頂点辞書
    vertex_dict_assert = dict()
    # 衝突辺辞書
    edge_dict_assert = ['5.0000000000_5.0000000000_2.0000000000_1.0000000000_1.0000000000_2.0000000000']
    edge_triangle_indexes_assert = {
        "3.0000000000_3.0000000000_6.0000000000%1": (Line(point=Point([1., 1., 2.]), direction=Vector([4., 4., 0.])), {0, 1}),
        "3.0000000000_3.0000000000_6.0000000000%2": (Line(point=Point([1., 1., 2.]), direction=Vector([4., 4., 0.])), {4, 5})
    }
    vertex_triangle_indexes_assert = dict()

    # 試験実施
    _make_triangle_group(
        cross_point_cordinate_result,
        vertex_dict,
        edge_list,
        edge_triangle_indexes,
        vertex_triangle_indexes,
        cross_group_index,
        triangle_index,
        rectangular_triangles,
    )

    ## 戻り値確認
    assert len(vertex_dict) == len(vertex_dict_assert)
    for key in vertex_dict:
        assert (vertex_dict[key] == vertex_dict_assert[key]).all()
    assert len(edge_list) == len(edge_dict_assert)
    assert edge_list == edge_dict_assert
    assert len(edge_triangle_indexes) == len(edge_triangle_indexes_assert)

    for key in edge_triangle_indexes:
        assert (edge_triangle_indexes[key][0].point == edge_triangle_indexes_assert[key][0].point).all
        assert (edge_triangle_indexes[key][0].direction == edge_triangle_indexes_assert[key][0].direction).all
        assert edge_triangle_indexes[key][1] == edge_triangle_indexes_assert[key][1]
    assert vertex_triangle_indexes == vertex_triangle_indexes_assert


def test__make_triangle_group_05():
    """ 三角ポリゴングループ作成・更新 (辺のグループ)
    + 試験詳細
      - 入力条件
        - 辺の三角ポリゴンの三角ポリゴン2グループを入力に渡す
        - 上記三角ポリゴン2グループが既にグループ化済みとする
      - 確認内容
        - 辺の三角ポリゴングループが更新されずに返却されること
    """
    # 衝突点（3次元）
    cross_point_cordinate_result = (Point([3., 3., 6.]), "c", None)
    # 衝突頂点辞書
    vertex_dict = dict()
    # 衝突辺辞書
    edge_dict = ['5.0000000000_5.0000000000_2.0000000000_1.0000000000_1.0000000000_2.0000000000']
    # 衝突点の辺の三角ポリゴングループのrectangular_triangles内のインデックス
    edge_triangle_indexes = {
        "3.0_3.0_6.0%0": {0, 1},
        "3.0_3.0_6.0%1": {4, 5}
    }
    # 衝突点の頂点の三角ポリゴングループのrectangular_triangles内のインデックス
    vertex_triangle_indexes = dict()
    cross_group_index = dict()
    # 衝突点の三角ポリゴンのインデックス
    triangle_index = 0
    # 全三角ポリゴン
    rectangular_triangles = [
        Triangle([1., 1., 2.], [5., 5., 2.], [3., 1., 6.]),
        Triangle([1., 1., 2.], [5., 5., 2.], [4., 3., 10.]),
        Triangle([1., 1., 2.], [4., 3., 10.], [3., 1., 6.]),
        Triangle([3., 1., 6.], [5., 5., 2.], [4., 3., 10.]),
        Triangle([1., 1., 2.], [5., 5., 2.], [3., 7., -1.]),
        Triangle([1., 1., 2.], [5., 5., 2.], [2., 9., -2.]),
        Triangle([1., 1., 2.], [2., 9., -2.], [3., 7., -1.]),
        Triangle([3., 7., -1.], [5., 5., 2.], [2., 9., -2.]),
    ]

    ## 期待値
    # 衝突頂点辞書
    vertex_dict_assert = dict()
    # 衝突辺辞書
    edge_dict_assert = ['5.0000000000_5.0000000000_2.0000000000_1.0000000000_1.0000000000_2.0000000000']
    edge_triangle_indexes_assert = {
        "3.0_3.0_6.0%0": {0, 1},
        "3.0_3.0_6.0%1": {4, 5}
    }
    vertex_triangle_indexes_assert = dict()

    # 試験実施
    _make_triangle_group(
        cross_point_cordinate_result,
        vertex_dict,
        edge_dict,
        edge_triangle_indexes,
        vertex_triangle_indexes,
        cross_group_index,
        triangle_index,
        rectangular_triangles,
    )

    ## 戻り値確認
    assert len(vertex_dict) == len(vertex_dict_assert)
    for key in vertex_dict:
        assert (vertex_dict[key] == vertex_dict_assert[key]).all()
    assert len(edge_dict) == len(edge_dict_assert)
    assert edge_dict == edge_dict_assert
    assert edge_triangle_indexes == edge_triangle_indexes_assert
    assert vertex_triangle_indexes == vertex_triangle_indexes_assert


def test__make_triangle_group_07():
    """ 三角ポリゴングループ作成・更新 (辺のグループ)
    + 試験詳細
      - 入力条件
        - 辺の三角ポリゴンの三角ポリゴン2グループを入力に渡す
        - 上記三角ポリゴン2グループが既にグループ化済みとする
        - 事前に処理を行った頂点が存在
      - 確認内容
        - 辺の三角ポリゴングループが更新されずに返却されること
    """
    # 衝突点（3次元）
    cross_point_cordinate_result = (Point([3., 3., 6.]), None, "a")
    # 衝突頂点辞書
    vertex_dict = dict()
    # 衝突辺辞書
    edge_dict = dict()
    # 衝突点の辺の三角ポリゴングループのrectangular_triangles内のインデックス
    edge_triangle_indexes = dict()
    # 衝突点の頂点の三角ポリゴングループのrectangular_triangles内のインデックス
    vertex_triangle_indexes = dict()
    # 事前に処理を行った頂点が存在
    cross_group_index = {"3.0000000000_3.0000000000_6.0000000000": 6}
    # 衝突点の三角ポリゴンのインデックス
    triangle_index = 0
    # 全三角ポリゴン
    rectangular_triangles = [
        Triangle([3., 3., 6.], [5., 5., 2.], [5., 1., 2.]),
        Triangle([3., 3., 6.], [5., 5., 2.], [1., 1., 1.]),
        Triangle([3., 3., 6.], [1., 1., 1.], [5., 1., 2.]),
        Triangle([3., 3., 6.], [5., 5., 10.], [5., 1., 10.]),
        Triangle([3., 3., 6.], [5., 5., 10.], [1., 1., 11.]),
        Triangle([3., 3., 6.], [1., 1., 11.], [5., 1., 10.]),
    ]

    ## 期待値
    # 衝突頂点辞書
    vertex_dict_assert = {"3.0000000000_3.0000000000_6.0000000000": Point([3., 3., 6.])}
    # 衝突辺辞書
    edge_dict_assert = dict()
    edge_triangle_indexes_assert = dict()
    vertex_triangle_indexes_assert = {
        "3.0000000000_3.0000000000_6.0000000000%6": {0, 1, 2},
        "3.0000000000_3.0000000000_6.0000000000%7": {3, 4, 5}
    }

    # 試験実施
    _make_triangle_group(
        cross_point_cordinate_result,
        vertex_dict,
        edge_dict,
        edge_triangle_indexes,
        vertex_triangle_indexes,
        cross_group_index,
        triangle_index,
        rectangular_triangles,
    )

    ## 戻り値確認
    assert len(vertex_dict) == len(vertex_dict_assert)
    for key in vertex_dict:
        assert (vertex_dict[key] == vertex_dict_assert[key]).all()
    assert len(edge_dict) == len(edge_dict_assert)
    for key in edge_dict:
        assert (edge_dict[key].point == edge_dict_assert[key].point).all()
        assert (edge_dict[key].direction == edge_dict_assert[key].direction).all()
    assert edge_triangle_indexes == edge_triangle_indexes_assert
    assert vertex_triangle_indexes == vertex_triangle_indexes_assert


def test__get_inner_voxel_01():
    """   内部ボクセル取得 (内外判定対象空間IDが空間IDの最大の高さを超過)
    + 試験詳細
      - 入力条件
        - 内外判定対象空間IDが空間IDの最大の高さを超過となる空間IDを指定
      - 確認内容
        - 空集合が返却されること
    """
    # 全三角ポリゴン
    rectangular_triangles = []
    # ボクセルの空間ID
    spatial_ids = [
        "25/30/60/25/40",
    ]

    inner_spatial_ids = set()
    outer_spatial_ids = set()
    # 試験実施
    result_spatial_ids = _get_inner_voxel(
        rectangular_triangles,
        spatial_ids,
        inner_spatial_ids,
        outer_spatial_ids
    )

    # 期待値
    inner_spatial_ids_assert = set()
    outer_spatial_ids_assert = set()

    # 結果確認
    assert inner_spatial_ids_assert == inner_spatial_ids
    assert outer_spatial_ids_assert == outer_spatial_ids


def test__get_inner_voxel_02():
    """   内部ボクセル取得 (内外判定対象空間IDが三角ポリゴンの空間IDになる)
    + 試験詳細
      - 入力条件
        - 内外判定対象空間IDが三角ポリゴンの空間IDとなる空間IDを指定
      - 確認内容
        - 空集合が返却されること
    """
    # 全三角ポリゴン 
    rectangular_triangles = []
    # ボクセルの空間ID 
    spatial_ids = [
        "25/30/60/25/40",
        "25/30/60/25/41"
    ]

    inner_spatial_ids = set()
    outer_spatial_ids = set()
    # 試験実施
    result_spatial_ids = _get_inner_voxel(
        rectangular_triangles,
        spatial_ids,
        inner_spatial_ids,
        outer_spatial_ids
    )

    # 期待値
    inner_spatial_ids_assert = set()
    outer_spatial_ids_assert = set()

    # 結果確認
    assert inner_spatial_ids_assert == inner_spatial_ids
    assert outer_spatial_ids_assert == outer_spatial_ids

def test__get_inner_voxel_03():
    """   内部ボクセル取得 (内外判定対象空間IDが内判定済みになる)
    + 試験詳細
      - 入力条件
        - 走査範囲が内部判定済みの空間IDを渡す
      - 確認内容
        - 内部判定済みの空間IDのみが返却されること
    """
    # 全三角ポリゴン 
    rectangular_triangles = []
    # ボクセルの空間ID 
    spatial_ids = [
        "25/30/60/25/40",
        "25/30/60/25/42"
    ]

    inner_spatial_ids = set(["25/30/60/25/41"])
    outer_spatial_ids = set()
    # 試験実施
    result_spatial_ids = _get_inner_voxel(
        rectangular_triangles,
        spatial_ids,
        inner_spatial_ids,
        outer_spatial_ids
    )

    # 期待値
    inner_spatial_ids_assert = set(["25/30/60/25/41"])
    outer_spatial_ids_assert = set()

    # 結果確認
    assert inner_spatial_ids_assert == inner_spatial_ids
    assert outer_spatial_ids_assert == outer_spatial_ids

def test__get_inner_voxel_04():
    """   内部ボクセル取得 (内外判定対象空間IDが外部定済みになる)
    + 試験詳細
      - 入力条件
        - 走査範囲が外部判定済みの空間IDを渡す
      - 確認内容
        - 外部判定済みの空間IDのみが返却されること
    """
    # 全三角ポリゴン 
    rectangular_triangles = []
    # ボクセルの空間ID 
    spatial_ids = [
        "25/30/60/25/40",
        "25/30/60/25/42"
    ]

    inner_spatial_ids = set()
    outer_spatial_ids = set(["25/30/60/25/41"])
    # 試験実施
    result_spatial_ids = _get_inner_voxel(
        rectangular_triangles,
        spatial_ids,
        inner_spatial_ids,
        outer_spatial_ids
    )

    # 期待値
    inner_spatial_ids_assert = set()
    outer_spatial_ids_assert = set(["25/30/60/25/41"])

    # 結果確認
    assert inner_spatial_ids_assert == inner_spatial_ids
    assert outer_spatial_ids_assert == outer_spatial_ids


def test__get_inner_voxel_05():
    """   内部ボクセル取得 (内外判定対象空間IDが三角ポリゴンの重なっている空間IDに衝突、内部の空間IDあり)
    + 試験詳細
      - 入力条件
        - 内外判定対象空間IDが三角ポリゴンの重なっている空間IDに衝突となる空間IDを指定
        - 内部判定される空間IDが存在するように指定
      - 確認内容
        - 内部に判定された空間IDが返却されること
    """
    # 全三角ポリゴン 
    point_a = get_point_on_spatial_id(
            "25/10/10/25/1", Point_Option.CENTER, PROJECTED_CRS)[0]
    point_b = get_point_on_spatial_id(
            "25/12/14/25/1", Point_Option.CENTER, PROJECTED_CRS)[0]
    point_c = get_point_on_spatial_id(
            "25/14/10/25/1", Point_Option.CENTER, PROJECTED_CRS)[0]
    point_d = get_point_on_spatial_id(
            "25/12/12/25/15", Point_Option.CENTER, PROJECTED_CRS)[0]
    rec_point_a = [point_a.x, point_a.y, point_a.alt]
    rec_point_b = [point_b.x, point_b.y, point_b.alt]
    rec_point_c = [point_c.x, point_c.y, point_c.alt]
    rec_point_d = [point_d.x, point_d.y, point_d.alt]
    rectangular_triangles = [
        Triangle(rec_point_a, rec_point_b, rec_point_c),
        Triangle(rec_point_a, rec_point_b, rec_point_d),
        Triangle(rec_point_a, rec_point_c, rec_point_d),
        Triangle(rec_point_b, rec_point_c, rec_point_d),
    ]
    # ボクセルの空間ID 
    spatial_ids = [
        "25/12/11/25/1",
        "25/12/11/25/4",
        "25/12/12/25/15"
    ]

    # 試験実施
    inner_spatial_ids = set()
    outer_spatial_ids = set()
    result_spatial_ids = _get_inner_voxel(
        rectangular_triangles,
        spatial_ids,
        inner_spatial_ids,
        outer_spatial_ids
    )

    # 期待値
    inner_spatial_ids_assert = set(["25/12/11/25/2", "25/12/11/25/3"])
    outer_spatial_ids_assert = set([
        "25/12/11/25/5",
        "25/12/11/25/6",
        "25/12/11/25/7",
        "25/12/11/25/8",
        "25/12/11/25/9",
        "25/12/11/25/10",
        "25/12/11/25/11",
        "25/12/11/25/12",
        "25/12/11/25/13",
        "25/12/11/25/14",
        "25/12/11/25/15",
    ])

    # 結果確認
    assert inner_spatial_ids_assert == inner_spatial_ids
    assert outer_spatial_ids_assert == outer_spatial_ids

def test__get_inner_voxel_06():
    """   内部ボクセル取得 (内外判定対象空間IDが三角ポリゴンの重なっている空間IDに衝突、内部の空間IDなし)
    + 試験詳細
      - 入力条件
        - 内外判定対象空間IDが三角ポリゴンの重なっている空間IDに衝突となる空間IDを指定
        - 外部判定される空間IDが存在するように指定
      - 確認内容
        - 内部に判定された空間IDが返却されるないこと
    """
    # 全三角ポリゴン 
    point_a = get_point_on_spatial_id(
            "25/10/10/25/1", Point_Option.CENTER, PROJECTED_CRS)[0]
    point_b = get_point_on_spatial_id(
            "25/12/14/25/1", Point_Option.CENTER, PROJECTED_CRS)[0]
    point_c = get_point_on_spatial_id(
            "25/14/10/25/1", Point_Option.CENTER, PROJECTED_CRS)[0]
    point_d = get_point_on_spatial_id(
            "25/12/12/25/15", Point_Option.CENTER, PROJECTED_CRS)[0]
    # 外部判定させるためのポリゴン(閉塞していないモデル)
    point_a_b = get_point_on_spatial_id(
            "25/10/10/25/16", Point_Option.CENTER, PROJECTED_CRS)[0]
    point_b_b = get_point_on_spatial_id(
            "25/12/14/25/16", Point_Option.CENTER, PROJECTED_CRS)[0]
    point_c_b = get_point_on_spatial_id(
            "25/14/10/25/16", Point_Option.CENTER, PROJECTED_CRS)[0]
    rec_point_a = [point_a.x, point_a.y, point_a.alt]
    rec_point_b = [point_b.x, point_b.y, point_b.alt]
    rec_point_c = [point_c.x, point_c.y, point_c.alt]
    rec_point_d = [point_d.x, point_d.y, point_d.alt]
    rec_point_a_b = [point_a_b.x, point_a_b.y, point_a_b.alt]
    rec_point_b_b = [point_b_b.x, point_b_b.y, point_b_b.alt]
    rec_point_c_b = [point_c_b.x, point_c_b.y, point_c_b.alt]
    rectangular_triangles = [
        Triangle(rec_point_a, rec_point_b, rec_point_c),
        Triangle(rec_point_a, rec_point_b, rec_point_d),
        Triangle(rec_point_a, rec_point_c, rec_point_d),
        Triangle(rec_point_b, rec_point_c, rec_point_d),
        Triangle(rec_point_a_b, rec_point_b_b, rec_point_c_b),
    ]
    # ボクセルの空間ID 
    spatial_ids = [
        "25/12/11/25/1",
        "25/12/11/25/4",
        "25/12/12/25/16"
    ]

    # 試験実施
    inner_spatial_ids = set()
    outer_spatial_ids = set()
    result_spatial_ids = _get_inner_voxel(
        rectangular_triangles,
        spatial_ids,
        inner_spatial_ids,
        outer_spatial_ids
    )

    # 期待値
    inner_spatial_ids_assert = set()
    outer_spatial_ids_assert = set([
        "25/12/11/25/2",
        "25/12/11/25/3",
        "25/12/11/25/5",
        "25/12/11/25/6",
        "25/12/11/25/7",
        "25/12/11/25/8",
        "25/12/11/25/9",
        "25/12/11/25/10",
        "25/12/11/25/11",
        "25/12/11/25/12",
        "25/12/11/25/13",
        "25/12/11/25/14",
        "25/12/11/25/15",
        "25/12/11/25/16",
    ])

    # 結果確認
    assert inner_spatial_ids_assert == inner_spatial_ids
    assert outer_spatial_ids_assert == outer_spatial_ids

def test__get_inner_voxel_07():
    """   内部ボクセル取得 (内外判定対象空間IDが三角ポリゴンの重なっている空間IDに衝突、辺頂点に衝突した場合の外側判定)
    + 試験詳細
      - 入力条件
        - 内外判定対象空間IDが三角ポリゴンの重なっている空間IDに衝突となる空間IDを指定
        - 辺頂点に衝突した場合の外側に判定されるようにする
      - 確認内容
        - 内部に判定された空間IDが返却されるないこと
    """
    # 全三角ポリゴン 
    point_a = get_point_on_spatial_id(
            "25/10/10/25/1", Point_Option.CENTER, PROJECTED_CRS)[0]
    point_b = get_point_on_spatial_id(
            "25/12/14/25/1", Point_Option.CENTER, PROJECTED_CRS)[0]
    point_c = get_point_on_spatial_id(
            "25/14/10/25/1", Point_Option.CENTER, PROJECTED_CRS)[0]
    point_d = get_point_on_spatial_id(
            "25/12/12/25/15", Point_Option.CENTER, PROJECTED_CRS)[0]
    # 外部判定させるためのポリゴン(閉塞していないモデル)
    point_a_b = get_point_on_spatial_id(
            "25/10/10/25/16", Point_Option.CENTER, PROJECTED_CRS)[0]
    point_b_b = get_point_on_spatial_id(
            "25/12/14/25/16", Point_Option.CENTER, PROJECTED_CRS)[0]
    point_c_b = get_point_on_spatial_id(
            "25/14/10/25/16", Point_Option.CENTER, PROJECTED_CRS)[0]
    rec_point_a = [point_a.x, point_a.y, point_a.alt]
    rec_point_b = [point_b.x, point_b.y, point_b.alt]
    rec_point_c = [point_c.x, point_c.y, point_c.alt]
    rec_point_d = [point_d.x, point_d.y, point_d.alt]
    rec_point_a_b = [point_a_b.x, point_a_b.y, point_a_b.alt]
    rec_point_b_b = [point_b_b.x, point_b_b.y, point_b_b.alt]
    rec_point_c_b = [point_c_b.x, point_c_b.y, point_c_b.alt]
    rectangular_triangles = [
        Triangle(rec_point_a, rec_point_b, rec_point_c),
        Triangle(rec_point_a, rec_point_b, rec_point_d),
        Triangle(rec_point_a, rec_point_c, rec_point_d),
        Triangle(rec_point_b, rec_point_c, rec_point_d),
        Triangle(rec_point_a_b, rec_point_b_b, rec_point_c_b),
    ]
    # ボクセルの空間ID 
    spatial_ids = [
        "25/10/10/25/1",
        "25/10/10/25/15",
        "25/10/10/25/16"
    ]

    # 試験実施
    inner_spatial_ids = set()
    outer_spatial_ids = set()
    result_spatial_ids = _get_inner_voxel(
        rectangular_triangles,
        spatial_ids,
        inner_spatial_ids,
        outer_spatial_ids
    )

    # 期待値
    inner_spatial_ids_assert = set()
    outer_spatial_ids_assert = set([
        "25/10/10/25/2",
        "25/10/10/25/3",
        "25/10/10/25/4",
        "25/10/10/25/5",
        "25/10/10/25/6",
        "25/10/10/25/7",
        "25/10/10/25/8",
        "25/10/10/25/9",
        "25/10/10/25/10",
        "25/10/10/25/11",
        "25/10/10/25/12",
        "25/10/10/25/13",
        "25/10/10/25/14",
    ])

    # 結果確認
    assert inner_spatial_ids_assert == inner_spatial_ids
    assert outer_spatial_ids_assert == outer_spatial_ids


def test__get_share_triangle_index_01():
    """  最大頂点共有三角ポリゴン取得(取得成功)
    + 試験詳細
      - 入力条件
        - 最大頂点共有三角ポリゴンと共有する三角ポリゴンを持つ三角ポリゴングループを指定する
      - 確認内容
        - 最大頂点共有三角ポリゴンのキーが返却されること
    """
    # 最大頂点
    max_z_point = "0.0_0.0_0.0%0"
    # 頂点三角ポリゴングループ
    vertex_triangle_group = {
        "0.0_0.0_0.0%0": {0, 1, 2},
        "0.0_0.0_3.0%0": {3, 4, 5},
        "0.0_0.0_5.0%0": {2, 6, 7},
    }

    ## 期待値
    # 共有三角ポリゴンインデックス
    share_triangle_index_assert = 2

    # 試験実施
    share_triangle_index = _get_share_triangle_index(max_z_point, vertex_triangle_group)

    # 結果確認
    assert share_triangle_index_assert == share_triangle_index


def test__get_share_triangle_index_02():
    """  最大頂点共有三角ポリゴン取得(取得失敗)
    + 試験詳細
      - 入力条件
        - 最大頂点共有三角ポリゴンと共有する三角ポリゴンを持たない三角ポリゴングループを指定する
      - 確認内容
        - Noneが返却されること
    """
    # 最大頂点
    max_z_point = "0.0_0.0_0.0%0"
    # 頂点三角ポリゴングループ
    vertex_triangle_group = {
        "0.0_0.0_0.0%0": {0, 1, 2},
        "0.0_0.0_3.0%0": {3, 4, 5},
        "0.0_0.0_5.0%0": {28, 6, 7},
    }

    ## 期待値
    # 共有三角ポリゴンインデックス
    share_triangle_index_assert = None

    # 試験実施
    share_triangle_index = _get_share_triangle_index(max_z_point, vertex_triangle_group)

    # 結果確認
    assert share_triangle_index_assert == share_triangle_index


def test__cross_check_edge_vertex_01():
    """  辺・頂点の内部判定 (面)
    + 試験詳細
      - 入力条件
        - 面に衝突した場合の衝突点情報を指定する
      - 確認内容
        - Trueのみの配列が返却されること
    """
    # 頂点三角ポリゴン辞書
    vertex_triangle_group = {
      "0.0_0.0_8.0%0": {0 ,1, 3, 4, 5}
    }
    # 辺三角ポリゴン辞書
    # dict[str, (Line, set[int])]
    edge_line_triangle_group = dict()
   #         "-1.0000000000_1.0000000000_2.0000000000%0": {7, 9}

    # 頂点辞書
    vertex_dict = {
        "0.0_0.0_6.0": Point([0., 0., 6.]),
        "0.0_0.0_7.0": Point([0., 0., 7.]),
        "0.0_0.0_8.0": Point([0., 0., 8.])
    }

    # 全三角ポリゴン
    rectangular_triangles = [
        Triangle([0., 0., 8.], [3., 3., 6.], [0., 0., 6.]),
        Triangle([0., 0., 8.], [3., -3., 6.], [0., 0., 6.]),
        Triangle([0., 0., 6.], [3., 3., 6.], [3., -3., 6.]),
        Triangle([0., 0., 8.], [3., 3., 6.], [-3., 3., 6.]),
        Triangle([0., 0., 8.], [-3., -3., 6.], [3., -3., 6.]),
        Triangle([0., 0., 8.], [-3., -3., 6.], [-3., 3., 6.]),
        Triangle([0., 0., 0.], [3., 3., 6.], [3., -3., 6.]),
        Triangle([0., 0., 0.], [3., 3., 6.], [-3., 3., 6.]),
        Triangle([0., 0., 0.], [-3., -3., 6.], [3., -3., 6.]),
        Triangle([0., 0., 0.], [-3., -3., 6.], [-3., 3., 6.]),
    ]
    judgment_completed_cross_points = set()

    # 試験実施
    inside_results  = _cross_check_edge_vertex(
        vertex_triangle_group,
        edge_line_triangle_group,
        vertex_dict,
        rectangular_triangles,
        judgment_completed_cross_points
    )

    ## 戻り値確認
    assert len(inside_results) == 1
    assert inside_results == [True]


def test__cross_check_edge_vertex_02():
    """  辺・頂点の内部判定 (内部判定頂点)
    + 試験詳細
      - 入力条件
        - 内部と判定される頂点に衝突した場合の衝突点情報を指定する
      - 確認内容
        - Trueのみの配列が返却されること
    """
    # 頂点三角ポリゴン辞書
    vertex_triangle_group = {
        "0.0_0.0_0.0%0": {6 ,7, 8, 9}
    }
    # 辺三角ポリゴン辞書
    edge_line_triangle_group = {}
    # 頂点辞書
    vertex_dict = {"0.0_0.0_0.0": Point([0., 0., 0.])}

    # 全三角ポリゴン
    rectangular_triangles = [
        Triangle([0., 0., 8.], [3., 3., 6.], [0., 0., 6.]),
        Triangle([0., 0., 8.], [3., -3., 6.], [0., 0., 6.]),
        Triangle([0., 0., 6.], [3., 3., 6.], [3., -3., 6.]),
        Triangle([0., 0., 8.], [3., 3., 6.], [-3., 3., 6.]),
        Triangle([0., 0., 8.], [-3., -3., 6.], [3., -3., 6.]),
        Triangle([0., 0., 8.], [-3., -3., 6.], [-3., 3., 6.]),
        Triangle([0., 0., 0.], [3., 3., 6.], [3., -3., 6.]),
        Triangle([0., 0., 0.], [3., 3., 6.], [-3., 3., 6.]),
        Triangle([0., 0., 0.], [-3., -3., 6.], [3., -3., 6.]),
        Triangle([0., 0., 0.], [-3., -3., 6.], [-3., 3., 6.]),
    ]
    judgment_completed_cross_points = set()

    # 試験実施
    inside_results = _cross_check_edge_vertex(
        vertex_triangle_group,
        edge_line_triangle_group,
        vertex_dict,
        rectangular_triangles,
        judgment_completed_cross_points
    )

    ## 戻り値確認
    assert len(inside_results) == 1
    assert inside_results == [True]


def test__cross_check_edge_vertex_03():
    """  辺・頂点の内部判定 (内部判定辺)
    + 試験詳細
      - 入力条件
        - 内部と判定される辺に衝突した場合の衝突点情報を指定する
      - 確認内容
        - Trueのみの配列が返却されること
    """
    # 頂点三角ポリゴン辞書
    vertex_triangle_group = {}
    # 辺三角ポリゴン辞書
    edge_line_triangle_group = {
        "-1.0_1.0_2.0%0": (Line.from_points([0., 0., 0.], [-3., 3., 6.]),set({7, 9}))
    }
    # 頂点辞書
    vertex_dict = {}
    # 全三角ポリゴン
    rectangular_triangles = [
        Triangle([0., 0., 8.], [3., 3., 6.], [0., 0., 6.]),
        Triangle([0., 0., 8.], [3., -3., 6.], [0., 0., 6.]),
        Triangle([0., 0., 6.], [3., 3., 6.], [3., -3., 6.]),
        Triangle([0., 0., 8.], [3., 3., 6.], [-3., 3., 6.]),
        Triangle([0., 0., 8.], [-3., -3., 6.], [3., -3., 6.]),
        Triangle([0., 0., 8.], [-3., -3., 6.], [-3., 3., 6.]),
        Triangle([0., 0., 0.], [3., 3., 6.], [3., -3., 6.]),
        Triangle([0., 0., 0.], [3., 3., 6.], [-3., 3., 6.]),
        Triangle([0., 0., 0.], [-3., -3., 6.], [3., -3., 6.]),
        Triangle([0., 0., 0.], [-3., -3., 6.], [-3., 3., 6.]),
    ]

    judgment_completed_cross_points = set()
    # 試験実施
    inside_results = _cross_check_edge_vertex(
        vertex_triangle_group,
        edge_line_triangle_group,
        vertex_dict,
        rectangular_triangles,
        judgment_completed_cross_points
    )

    ## 戻り値確認
    assert len(inside_results) == 1
    assert inside_results == [True]


def test__cross_check_edge_vertex_04():
    """  辺・頂点の内部判定 (内部判定2辺)
    + 試験詳細
      - 入力条件
        - 内部と判定される2辺に衝突した場合の衝突点情報を指定する
      - 確認内容
        - Trueのみの配列が返却されること
    """
    # 頂点三角ポリゴン辞書
    vertex_triangle_group = {}
    # 辺三角ポリゴン辞書
    edge_line_triangle_group = {
        "1.5_1.5_6.0%0":(Line.from_points([0., 0., 6.], [3., 3., 6.]), {2, 0}),
        "1.5_1.5_7.0%0": (Line.from_points([0., 0., 8.], [3., 3., 6.]),{0, 3})
    }
    # 頂点辞書
    vertex_dict = {}

    # 全三角ポリゴン
    rectangular_triangles = [
        Triangle([0., 0., 8.], [3., 3., 6.], [0., 0., 6.]),
        Triangle([0., 0., 8.], [3., -3., 6.], [0., 0., 6.]),
        Triangle([0., 0., 6.], [3., 3., 6.], [3., -3., 6.]),
        Triangle([0., 0., 8.], [3., 3., 6.], [-3., 3., 6.]),
        Triangle([0., 0., 8.], [-3., -3., 6.], [3., -3., 6.]),
        Triangle([0., 0., 8.], [-3., -3., 6.], [-3., 3., 6.]),
        Triangle([0., 0., 0.], [3., 3., 6.], [3., -3., 6.]),
        Triangle([0., 0., 0.], [3., 3., 6.], [-3., 3., 6.]),
        Triangle([0., 0., 0.], [-3., -3., 6.], [3., -3., 6.]),
        Triangle([0., 0., 0.], [-3., -3., 6.], [-3., 3., 6.]),
    ]
    judgment_completed_cross_points = set()

    # 試験実施
    inside_results = _cross_check_edge_vertex(
        vertex_triangle_group,
        edge_line_triangle_group,
        vertex_dict,
        rectangular_triangles,
        judgment_completed_cross_points
    )

    ## 戻り値確認
    assert len(inside_results) == 1
    assert inside_results == [True]


def test__cross_check_edge_vertex_05():
    """  辺・頂点の内部判定 (内部判定2頂点)
    + 試験詳細
      - 入力条件
        - 内部と判定される2頂点に衝突した場合の衝突点情報を指定する
      - 確認内容
        - Trueのみの配列が返却されること
    """
    # 頂点三角ポリゴン辞書
    vertex_triangle_group = {
        "0.0_0.0_8.0%0": {0, 1, 3, 4, 5},
        "0.0_0.0_6.0%0": {0, 1, 2}
    }
    # 辺三角ポリゴン辞書
    edge_line_triangle_group = {}
    # 頂点辞書
    vertex_dict = {
        "0.0_0.0_6.0": Point([0., 0., 6.]),
        "0.0_0.0_8.0": Point([0., 0., 8.])
    }

    # 全三角ポリゴン
    rectangular_triangles = [
        Triangle([0., 0., 8.], [3., 3., 6.], [0., 0., 6.]),
        Triangle([0., 0., 8.], [3., -3., 6.], [0., 0., 6.]),
        Triangle([0., 0., 6.], [3., 3., 6.], [3., -3., 6.]),
        Triangle([0., 0., 8.], [3., 3., 6.], [-3., 3., 6.]),
        Triangle([0., 0., 8.], [-3., -3., 6.], [3., -3., 6.]),
        Triangle([0., 0., 8.], [-3., -3., 6.], [-3., 3., 6.]),
        Triangle([0., 0., 0.], [3., 3., 6.], [3., -3., 6.]),
        Triangle([0., 0., 0.], [3., 3., 6.], [-3., 3., 6.]),
        Triangle([0., 0., 0.], [-3., -3., 6.], [3., -3., 6.]),
        Triangle([0., 0., 0.], [-3., -3., 6.], [-3., 3., 6.]),
    ]
    judgment_completed_cross_points = set()

    # 試験実施
    inside_results = _cross_check_edge_vertex(
        vertex_triangle_group,
        edge_line_triangle_group,
        vertex_dict,
        rectangular_triangles,
        judgment_completed_cross_points
    )

    ## 戻り値確認
    assert len(inside_results) == 1
    assert inside_results == [True]


def test__cross_check_edge_vertex_06():
    """  辺・頂点の内部判定 (内部判定1辺1頂点)
    + 試験詳細
      - 入力条件
        - 内部と判定される1辺1頂点に衝突した場合の衝突点情報を指定する
        - 頂点の衝突点の方がZ成分を大きくする
      - 確認内容
        - Trueのみの配列が返却されること
    """
    # 頂点三角ポリゴン辞書
    vertex_triangle_group = {
        "1.0_1.0_8.0%0": {0, 1, 3, 4, 5},
    }
    # 辺三角ポリゴン辞書
    edge_line_triangle_group = {
        "1.0_1.0_6.0%0": (Line.from_points([0., 0., 6.], [3., 3., 6.]),{0, 2})
    }
    # 頂点辞書
    vertex_dict = {
        "1.0_1.0_8.0": Point([1., 1., 8.])
    }

    # 全三角ポリゴン
    rectangular_triangles = [
        Triangle([1., 1., 8.], [3., 3., 6.], [0., 0., 6.]),
        Triangle([1., 1., 8.], [3., -3., 6.], [0., 0., 6.]),
        Triangle([0., 0., 6.], [3., 3., 6.], [3., -3., 6.]),
        Triangle([1., 1., 8.], [3., 3., 6.], [-3., 3., 6.]),
        Triangle([1., 1., 8.], [-3., -3., 6.], [3., -3., 6.]),
        Triangle([1., 1., 8.], [-3., -3., 6.], [-3., 3., 6.]),
        Triangle([0., 0., 0.], [3., 3., 6.], [3., -3., 6.]),
        Triangle([0., 0., 0.], [3., 3., 6.], [-3., 3., 6.]),
        Triangle([0., 0., 0.], [-3., -3., 6.], [3., -3., 6.]),
        Triangle([0., 0., 0.], [-3., -3., 6.], [-3., 3., 6.]),
    ]
    judgment_completed_cross_points = set()

    # 試験実施
    inside_results = _cross_check_edge_vertex(
        vertex_triangle_group,
        edge_line_triangle_group,
        vertex_dict,
        rectangular_triangles,
        judgment_completed_cross_points
    )

    ## 戻り値確認
    assert len(inside_results) == 1
    assert inside_results == [True]


def test__cross_check_edge_vertex_07():
    """  辺・頂点の内部判定 (内部判定1辺1頂点)
    + 試験詳細
      - 入力条件
        - 内部と判定される1辺1頂点に衝突した場合の衝突点情報を指定する
        - 辺の衝突点の方がZ成分を大きくする
      - 確認内容
        - Trueのみの配列が返却されること
    """
    # 頂点三角ポリゴン辞書
    vertex_triangle_group = {
        "1.0_1.0_-8.0%0": {0, 1, 3, 4, 5},
    }
    # 辺三角ポリゴン辞書
    edge_line_triangle_group = {
        "1.0_1.0_-6.0%0": (Line.from_points([0., 0., -6.], [3., 3., -6.]),{0, 2})
    }
    # 頂点辞書
    vertex_dict = {
        "1.0_1.0_-8.0": Point([1., 1., -8.])
    }
    # 全三角ポリゴン
    rectangular_triangles = [
        Triangle([1., 1., -8.], [3., 3., -6.], [0., 0., -6.]),
        Triangle([1., 1., -8.], [3., -3., -6.], [0., 0., -6.]),
        Triangle([0., 0., -6.], [3., 3., -6.], [3., -3., -6.]),
        Triangle([1., 1., -8.], [3., 3., -6.], [-3., 3., -6.]),
        Triangle([1., 1., -8.], [-3., -3., -6.], [3., -3., -6.]),
        Triangle([1., 1., -8.], [-3., -3., -6.], [-3., 3., -6.]),
        Triangle([0., 0., -0.], [3., 3., -6.], [3., -3., -6.]),
        Triangle([0., 0., -0.], [3., 3., -6.], [-3., 3., -6.]),
        Triangle([0., 0., -0.], [-3., -3., -6.], [3., -3., -6.]),
        Triangle([0., 0., -0.], [-3., -3., -6.], [-3., 3., -6.]),
    ]

    judgment_completed_cross_points = set()

    # 試験実施
    inside_results = _cross_check_edge_vertex(
        vertex_triangle_group,
        edge_line_triangle_group,
        vertex_dict,
        rectangular_triangles,
        judgment_completed_cross_points
    )

    ## 戻り値確認
    assert len(inside_results) == 1
    assert inside_results == [True]


def test__cross_check_edge_vertex_08(mocker):
    """  辺・頂点の内部判定 (内部判定2頂点で共有三角ポリゴン無し)
    | 下記ルートは基本的に不通過であるため、モックを用いて試験を行う
    + 試験詳細
      - 入力条件
        - 内部と判定される2頂点に衝突した場合の衝突点情報を指定する
      - 確認内容
        - Falseのみの配列が返却されること
    """
    # モック定義
    mocker.patch('SpatialId.shape.polygons._get_share_triangle_index', return_value=None)

    # 頂点三角ポリゴン辞書
    vertex_triangle_group = {
        "0.0_0.0_8.0%0": {0, 1, 3, 4, 5},
        "0.0_0.0_6.0%0": {0, 1, 2}
    }
    # 辺三角ポリゴン辞書
    edge_list_triangle_group = {}
    # 頂点辞書
    vertex_dict = {
        "0.0_0.0_6.0": Point([0., 0., 6.]),
        "0.0_0.0_8.0": Point([0., 0., 8.])
    }

    # 全三角ポリゴン
    rectangular_triangles = [
        Triangle([0., 0., 8.], [3., 3., 6.], [0., 0., 6.]),
        Triangle([0., 0., 8.], [3., -3., 6.], [0., 0., 6.]),
        Triangle([0., 0., 6.], [3., 3., 6.], [3., -3., 6.]),
        Triangle([0., 0., 8.], [3., 3., 6.], [-3., 3., 6.]),
        Triangle([0., 0., 8.], [-3., -3., 6.], [3., -3., 6.]),
        Triangle([0., 0., 8.], [-3., -3., 6.], [-3., 3., 6.]),
        Triangle([0., 0., 0.], [3., 3., 6.], [3., -3., 6.]),
        Triangle([0., 0., 0.], [3., 3., 6.], [-3., 3., 6.]),
        Triangle([0., 0., 0.], [-3., -3., 6.], [3., -3., 6.]),
        Triangle([0., 0., 0.], [-3., -3., 6.], [-3., 3., 6.]),
    ]

    judgment_completed_cross_points = set()

    # 試験実施
    inside_results = _cross_check_edge_vertex(
        vertex_triangle_group,
        edge_list_triangle_group,
        vertex_dict,
        rectangular_triangles,
        judgment_completed_cross_points
    )

    ## 戻り値確認
    assert len(inside_results) == 1
    assert inside_results == [False]


def test__cross_check_edge_vertex_09():
    """  辺・頂点の内部判定 (内部判定2頂点&2頂点連続)
    + 試験詳細
      - 入力条件
        - 内部と判定される2頂点に衝突した場合の衝突点情報を指定する
        - 内部と判定される2頂点が2つの2頂点の連続で成り立つ入力
      - 確認内容
        - Trueのみの配列が返却されること
    """
    # 頂点三角ポリゴン辞書
    vertex_triangle_group = {
        "0.0_0.0_6.0%0": {2, 3, 4},
        "0.0_0.0_7.0%0": {0, 1, 3, 4, 5},
        "0.0_0.0_8.0%0": {0, 1, 5, 6, 7},
    }
    # 辺三角ポリゴン辞書
    edge_list_triangle_group = {}
    # 頂点辞書
    vertex_dict = {
        "0.0_0.0_6.0": Point([0., 0., 6.]),
        "0.0_0.0_7.0": Point([0., 0., 7.]),
        "0.0_0.0_8.0": Point([0., 0., 8.])
    }
    # 全三角ポリゴン
    rectangular_triangles = [
        Triangle([0., 0., 8.], [3., 3., 6.], [0., 0., 7.]),
        Triangle([0., 0., 8.], [3., -3., 6.], [0., 0., 7.]),
        Triangle([0., 0., 7.], [3., 3., 6.], [0., 0., 6.]),
        Triangle([0., 0., 7.], [3., -3., 6.], [0., 0., 6.]),
        Triangle([0., 0., 6.], [3., 3., 6.], [3., -3., 6.]),
        Triangle([0., 0., 8.], [3., 3., 6.], [-3., 3., 6.]),
        Triangle([0., 0., 8.], [-3., -3., 6.], [3., -3., 6.]),
        Triangle([0., 0., 8.], [-3., -3., 6.], [-3., 3., 6.]),
        Triangle([0., 0., 0.], [3., 3., 6.], [3., -3., 6.]),
        Triangle([0., 0., 0.], [3., 3., 6.], [-3., 3., 6.]),
        Triangle([0., 0., 0.], [-3., -3., 6.], [3., -3., 6.]),
        Triangle([0., 0., 0.], [-3., -3., 6.], [-3., 3., 6.]),
    ]

    judgment_completed_cross_points = set()

    # 試験実施
    inside_results = _cross_check_edge_vertex(
        vertex_triangle_group,
        edge_list_triangle_group,
        vertex_dict,
        rectangular_triangles,
        judgment_completed_cross_points
    )

    ## 戻り値確認
    assert len(inside_results) == 1
    assert inside_results == [True]


def test__cross_check_edge_vertex_10():
    """  辺・頂点の内部判定 (内部判定2頂点)
    + 試験詳細
      - 入力条件
        - 内部と判定される2頂点に衝突した場合の衝突点情報を指定する
        - 頂点のXY座標が同一とみなされる座標をポリゴンの頂点に指定する
      - 確認内容
        - Trueのみの配列が返却されること
    """
    # 頂点三角ポリゴン辞書
    vertex_triangle_group = {
        "0.0_0.0_8.0%0": {0, 1, 3, 4, 5},
        "0.0_0.0_6.0%0": {0, 1, 2}
    }
    # 辺三角ポリゴン辞書
    edge_line_triangle_group = {}
    # 頂点辞書
    vertex_dict = {
        "0.0_0.0_6.0": Point([0., 0., 6.]),
        "0.0_0.0_8.0": Point([0., 0., 8.])
    }

    # 全三角ポリゴン
    rectangular_triangles = [
        Triangle([0., 0., 8.], [0., 0., 6.], [3., 3., 6.]),
        Triangle([0., 0., 8.], [3., -3., 6.], [0., 0., 6.]),
        Triangle([0., 0., 6.], [3., 3., 6.], [3., -3., 6.]),
        Triangle([0., 0., 8.], [3., 3., 6.], [-3., 3., 6.]),
        Triangle([0., 0., 8.], [-3., -3., 6.], [3., -3., 6.]),
        Triangle([0., 0., 8.], [-3., -3., 6.], [-3., 3., 6.]),
        Triangle([0., 0., 0.], [3., 3., 6.], [3., -3., 6.]),
        Triangle([0., 0., 0.], [3., 3., 6.], [-3., 3., 6.]),
        Triangle([0., 0., 0.], [-3., -3., 6.], [3., -3., 6.]),
        Triangle([0., 0., 0.], [-3., -3., 6.], [-3., 3., 6.]),
    ]
    judgment_completed_cross_points = set()

    # 試験実施
    inside_results = _cross_check_edge_vertex(
        vertex_triangle_group,
        edge_line_triangle_group,
        vertex_dict,
        rectangular_triangles,
        judgment_completed_cross_points
    )

    ## 戻り値確認
    assert len(inside_results) == 1
    assert inside_results == [True]


def test__get_cross_point_cordinate_01():
    """ ボクセルの中心点からZ軸方向へのレイと三角ポリゴンの接触点を算出(面衝突)
    + 試験詳細
      - 入力条件
        - ボクセルの中心点からZ軸方向へのレイと面で接触する三角ポリゴンを指定
      - 確認内容
        - 接触点が返却されること
        - 衝突点がある辺のキーがNoneであること
        - 衝突点がある頂点のキーがNoneであること
    """
    # 三角ポリゴン
    rectangular_triangle = Triangle([3, 3, 1], [5, 5, 2], [4, 5, 3])
    # ボクセルの中心点
    start_orth_point = Point([4, 4.5, 0])

    ## 期待値
    # 衝突点
    cross_point_assert = Point([4, 4.5, 2.25])
    # 衝突点がある辺のキー
    edge_key_assert = None
    # 衝突点がある頂点のキー
    vertex_key_assert = None

    # 試験実施
    return_value  = _get_cross_point_cordinate(
        rectangular_triangle,
        start_orth_point
    )

    # 戻り値確認
    assert len(return_value) == 1

    cross_point, edge_key, vertex_key = return_value[0]
    assert (cross_point_assert == cross_point).all()
    assert edge_key_assert == edge_key
    assert vertex_key_assert == vertex_key


def test__get_cross_point_cordinate_02():
    """ ボクセルの中心点からZ軸方向へのレイと三角ポリゴンの接触点を算出(非衝突)
    + 試験詳細
      - 入力条件
        - ボクセルの中心点からZ軸方向へのレイと接触しない三角ポリゴンを指定
      - 確認内容
        - 空リストが返却されること
    """
    # 三角ポリゴン
    rectangular_triangle = Triangle([3, 3, 1], [5, 5, 2], [4, 5, 3])
    # ボクセルの中心点
    start_orth_point = Point([4, 5.5, 0])


    # 試験実施
    return_value  = _get_cross_point_cordinate(
        rectangular_triangle,
        start_orth_point
    )

    # 戻り値確認
    assert len(return_value) == 0


def test__get_cross_point_cordinate_03():
    """ ボクセルの中心点からZ軸方向へのレイと三角ポリゴンの接触点を算出(1頂点と衝突)
    + 試験詳細
      - 入力条件
        - ボクセルの中心点からZ軸方向へのレイと1頂点接触する三角ポリゴンを指定
      - 確認内容
        - 接触点が返却されること
        - 衝突点がある辺のキーがNoneであること
        - 衝突点がある頂点のキーがNoneでないこと
    """
    # 三角ポリゴン
    rectangular_triangle = Triangle([3, 3, 1], [5, 5, 2], [4, 5, 3])
    # ボクセルの中心点
    start_orth_point = Point([4, 5, 0])

    ## 期待値
    # 衝突点
    cross_point_assert = Point([4, 5, 3])
    # 衝突点がある辺のキー
    edge_key_assert = None
    # 衝突点がある頂点のキー
    vertex_key_assert = "C"

    # 試験実施
    return_value  = _get_cross_point_cordinate(
        rectangular_triangle,
        start_orth_point
    )

    # 戻り値確認
    assert len(return_value) == 1

    cross_point, edge_key, vertex_key = return_value[0]
    assert (cross_point_assert == cross_point).all()
    assert edge_key_assert == edge_key
    assert vertex_key_assert == vertex_key


def test__get_cross_point_cordinate_04():
    """ ボクセルの中心点からZ軸方向へのレイと三角ポリゴンの接触点を算出(1辺と衝突)
    + 試験詳細
      - 入力条件
        - ボクセルの中心点からZ軸方向へのレイと1辺接触する三角ポリゴンを指定
      - 確認内容
        - 接触点が返却されること
        - 衝突点がある辺のキーがNoneでないこと
        - 衝突点がある頂点のキーがNoneであること
    """
    # 三角ポリゴン
    rectangular_triangle = Triangle([3, 3, 1], [5, 5, 2], [4, 5, 3])
    # ボクセルの中心点
    start_orth_point = Point([4, 4, 0])

    ## 期待値
    # 衝突点
    cross_point_assert = Point([4, 4, 1.5])
    # 衝突点がある辺のキー
    edge_key_assert = "c"
    # 衝突点がある頂点のキー
    vertex_key_assert = None

    # 試験実施
    return_value  = _get_cross_point_cordinate(
        rectangular_triangle,
        start_orth_point
    )

    # 戻り値確認
    assert len(return_value) == 1

    cross_point, edge_key, vertex_key = return_value[0]
    assert (cross_point_assert == cross_point).all()
    assert edge_key_assert == edge_key
    assert vertex_key_assert == vertex_key


def test__get_cross_point_cordinate_05():
    """ ボクセルの中心点からZ軸方向へのレイと三角ポリゴンの接触点を算出
    | 三角ポリゴンがZ軸に水平
    | レイとの接触なし
    + 試験詳細
      - 入力条件
        - Z軸に水平な三角ポリゴンを指定する
        - ボクセルの中心点からZ軸方向へのレイと接触しない三角ポリゴンを指定
      - 確認内容
        - 空リストが返却されること
    """
    # 三角ポリゴン
    rectangular_triangle = Triangle([3, 3, 1], [5, 5, 2], [4, 4, 3])
    # ボクセルの中心点
    start_orth_point = Point([2, 2, 0])


    # 試験実施
    return_value  = _get_cross_point_cordinate(
        rectangular_triangle,
        start_orth_point
    )

    # 戻り値確認
    assert len(return_value) == 0

def test__get_cross_point_cordinate_06():
    """ ボクセルの中心点からZ軸方向へのレイと三角ポリゴンの接触点を算出
    | 三角ポリゴンがZ軸に水平
    | レイと頂点で接触
    + 試験詳細
      - 入力条件
        - Z軸に水平な三角ポリゴンを指定する
        - ボクセルの中心点からZ軸方向へのレイと頂点で接触する三角ポリゴンを指定
    - 確認内容
        - リストの要素数が1であること
        - 接触点が返却されること
        - 衝突点がある辺のキーがNoneであること
        - 衝突点がある頂点のキーがNoneでないこと
    """
    # 三角ポリゴン
    rectangular_triangle = Triangle([3, 3, 1], [5, 5, 2], [4, 4, 3])
    # ボクセルの中心点
    start_orth_point = Point([3, 3, 0])

    ## 期待値
    # 衝突点
    cross_point_assert = Point([3., 3., 1.])
    # 衝突点がある辺のキー
    edge_key_assert = None
    # 衝突点がある頂点のキー
    vertex_key_assert = "A"

    # 試験実施
    return_value  = _get_cross_point_cordinate(
        rectangular_triangle,
        start_orth_point
    )

    # 戻り値確認
    assert len(return_value) == 1

    cross_point, edge_key, vertex_key = return_value[0]
    assert cross_point_assert.distance_point(cross_point) < MINIMA
    assert edge_key_assert == edge_key
    assert vertex_key_assert == vertex_key


def test__get_cross_point_cordinate_07():
    """ ボクセルの中心点からZ軸方向へのレイと三角ポリゴンの接触点を算出
    | 三角ポリゴンがZ軸に水平
    | レイと2辺が接触
    + 試験詳細
      - 入力条件
        - Z軸に水平な三角ポリゴンを指定する
        - ボクセルの中心点からZ軸方向へのレイと2辺で接触する三角ポリゴンを指定
    - 確認内容
        - リストの要素数が2であること
        - 要素の両方接触点が返却されること
        - 要素の両方衝突点がある辺のキーがNoneでないこと
        - 要素の両方衝突点がある頂点のキーがNoneであること
    """
    # 三角ポリゴン
    rectangular_triangle = Triangle([3, 3, 1], [5, 5, 2], [4, 4, 3])
    # ボクセルの中心点
    start_orth_point = Point([3.5, 3.5, 0])

    ## 期待値
    # 衝突点
    cross_point_assert1 = Point([3.5, 3.5, 2.])
    cross_point_assert2 = Point([3.5, 3.5, 1.25])
    # 衝突点がある辺のキー
    edge_key_assert1 = "b"
    edge_key_assert2 = "c"
    # 衝突点がある頂点のキー
    vertex_key_assert1 = None
    vertex_key_assert2 = None

    # 試験実施
    return_value  = _get_cross_point_cordinate(
        rectangular_triangle,
        start_orth_point
    )

    # 戻り値確認
    assert len(return_value) == 2

    cross_point, edge_key, vertex_key = return_value[0]
    assert cross_point_assert1.distance_point(cross_point) < MINIMA
    assert edge_key_assert1 == edge_key
    assert vertex_key_assert1 == vertex_key

    cross_point, edge_key, vertex_key = return_value[1]
    assert cross_point_assert2.distance_point(cross_point) < MINIMA
    assert edge_key_assert2 == edge_key
    assert vertex_key_assert2 == vertex_key


def test__get_cross_point_cordinate_08():
    """ ボクセルの中心点からZ軸方向へのレイと三角ポリゴンの接触点を算出
    | 三角ポリゴンがZ軸に水平
    | レイと1辺1頂点が接触
    + 試験詳細
      - 入力条件
        - Z軸に水平な三角ポリゴンを指定する
        - ボクセルの中心点からZ軸方向へのレイと1辺1頂点で接触する三角ポリゴンを指定
    - 確認内容
        - リストの要素数が2であること
        - 辺と頂点の接触点が返却されること
        - 辺側の衝突点がある辺のキーがNoneでないこと
        - 頂点側の衝突点がある頂点のキーがNoneでないこと
    """
    # 三角ポリゴン
    rectangular_triangle = Triangle([3, 3, 1], [5, 5, 2], [4, 4, 3])
    # ボクセルの中心点
    start_orth_point = Point([4, 4, 0])

    ## 期待値
    # 衝突点
    cross_point_assert1 = Point([4., 4., 3.])
    cross_point_assert2 = Point([4., 4., 1.5])
    # 衝突点がある辺のキー
    edge_key_assert1 = None
    edge_key_assert2 = "c"
    # 衝突点がある頂点のキー
    vertex_key_assert1 = "C"
    vertex_key_assert2 = None

    # 試験実施
    return_value  = _get_cross_point_cordinate(
        rectangular_triangle,
        start_orth_point
    )

    # 戻り値確認
    assert len(return_value) == 2

    cross_point, edge_key, vertex_key = return_value[0]
    assert cross_point_assert1.distance_point(cross_point) < MINIMA
    assert edge_key_assert1 == edge_key
    assert vertex_key_assert1 == vertex_key

    cross_point, edge_key, vertex_key = return_value[1]
    assert cross_point_assert2.distance_point(cross_point) < MINIMA
    assert edge_key_assert2 == edge_key
    assert vertex_key_assert2 == vertex_key


def test__get_cross_point_cordinate_09():
    """ ボクセルの中心点からZ軸方向へのレイと三角ポリゴンの接触点を算出
    | 三角ポリゴンがZ軸に水平
    | レイと辺が重複して接触
    + 試験詳細
      - 入力条件
        - Z軸に水平な三角ポリゴンを指定する
        - ボクセルの中心点からZ軸方向へのレイと辺が重複して接触する三角ポリゴンを指定
    - 確認内容
        - リストの要素数が2であること
        - 頂点の接触点が返却されること
        - 衝突点がある辺のキーがNoneであること
        - 頂点側の衝突点がある頂点のキーがNoneでないこと
    """
    # 三角ポリゴン
    rectangular_triangle = Triangle([3, 3, 1], [5, 5, 2], [3, 3, 3])
    # ボクセルの中心点
    start_orth_point = Point([3, 3, 0])

    ## 期待値
    # 衝突点
    cross_point_assert1 = Point([3., 3., 3.])
    cross_point_assert2 = Point([3., 3., 1.])
    # 衝突点がある辺のキー
    edge_key_assert1 = None
    edge_key_assert2 = None
    # 衝突点がある頂点のキー
    vertex_key_assert1 = "C"
    vertex_key_assert2 = "A"

    # 試験実施
    return_value  = _get_cross_point_cordinate(
        rectangular_triangle,
        start_orth_point
    )

    # 戻り値確認
    assert len(return_value) == 2

    cross_point, edge_key, vertex_key = return_value[0]
    assert cross_point_assert1.distance_point(cross_point) < MINIMA
    assert edge_key_assert1 == edge_key
    assert vertex_key_assert1 == vertex_key

    cross_point, edge_key, vertex_key = return_value[1]
    assert cross_point_assert2.distance_point(cross_point) < MINIMA
    assert edge_key_assert2 == edge_key
    assert vertex_key_assert2 == vertex_key


def test__get_cross_point_cordinate_10():
    """ ボクセルの中心点からZ軸方向へのレイと三角ポリゴンの接触点を算出
    | 三角ポリゴンがZ軸に水平
    | レイと垂直に交わる辺が三角ポリゴンに存在
    | 垂直に交わる辺とレイが延長線上で交差する
    + 試験詳細
      - 入力条件
        - Z軸に水平な三角ポリゴンを指定する
        - ボクセルの中心点からZ軸方向へのレイと垂直に交わる辺が三角ポリゴンに存在し、接触しない値を指定
    - 確認内容
        - リストの要素数が0であること
    """
    # 三角ポリゴン
    rectangular_triangle = Triangle([3, 3, 1], [5, 5, 3], [3, 3, 3])
    # ボクセルの中心点
    start_orth_point = Point([6, 6, 0])

    # 試験実施
    return_value  = _get_cross_point_cordinate(
        rectangular_triangle,
        start_orth_point
    )

    # 戻り値確認
    assert len(return_value) == 0

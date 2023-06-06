#!/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import pytest

from skspatial.objects import Point, Plane, Vector, Line

from SpatialId.common.object.enum import Point_Option
from SpatialId.common.exception import SpatialIdError
from SpatialId.common.object.point import Point as SpatialPoint, Projected_Point
from SpatialId.common.object.point import (
        Projected_Point as SpatialProjected_Point)
from SpatialId.shape.point import (
        get_spatial_ids_on_points,
        get_point_on_spatial_id,
        convert_projected_point_list_to_point_list,
        convert_point_list_to_projected_point_list)
from SpatialId.shape.line import (
        get_spatial_ids_on_line,
        detect_collision,
        create_sequence_list,
        generate_x_voxel_plane,
        generate_y_voxel_plane,
        generate_z_voxel_plane,
        get_x_voxel_plane_point_on_spatial_id,
        get_y_voxel_plane_point_on_spatial_id,
        get_z_voxel_plane_point_on_spatial_id,
        get_spatial_id_on_axis_ids,
        get_voxel_id_to_spatial_id,
        _get_plane_cross_points,
        _convert_point_to_str,
        _get_middle_points
        )

# CRSのデフォルト値のEPSGコード
PROJECTED_CRS = 3857
GEOGRAPHIC_CRS = 4326


def test__convert_point_to_str_01():
    """点座標文字列変換
    """
    # 試験用データの定義
    # Point
    point = Point((140.08785504, 36.10377479, 6.6))

    # 期待値の定義
    # 文字列変換後の座標
    param_assert = '140.08785504_36.10377479_6.6'

    # 試験実施
    # 空間ID取得
    param_return = _convert_point_to_str(point)

    # 戻り値確認
    assert param_assert == param_assert


def test_get_voxel_id_to_spatial_id_01():
    """空間IDからボクセル成分を取得、正常な空間IDを入力
    """
    # 試験用データの定義
    # voxcel id の定義パターン1 - 水平、垂直方向精度1
    spatial_id_01 = "1/1/0/1/1"

    # voxcel id の定義パターン2 - 水平精度10、垂直方向精度8
    spatial_id_02 = "10/91/40/8/0"

    # 期待値の定義
    # パターン1に対応するインデックス
    param_assert_01 = (1, 0, 1)

    # パターン1に対応するインデックス
    param_assert_02 = (91, 40, 0)

    # 試験実施
    # 戻り値取得
    param_return_01 = get_voxel_id_to_spatial_id(spatial_id_01)
    param_return_02 = get_voxel_id_to_spatial_id(spatial_id_02)

    # 戻り値確認
    assert param_assert_01 == param_return_01
    assert param_assert_02 == param_return_02


def test_get_spatial_id_on_axis_ids_01():
    """軸IDから空間ID取得
    """
    # 試験用データの定義
    # 軸IDを定義
    x_id = 2
    y_id = 1
    z_id = 3

    # 水平、垂直方向精度を定義
    h_zoom = 1
    v_zoom = 2

    # 期待値の定義
    # 空間IDを定義
    param_assert = "/".join(map(str, [h_zoom, x_id, y_id, v_zoom, z_id]))

    # 試験実施
    # 戻り値取得
    param_return = get_spatial_id_on_axis_ids(x_id, y_id, z_id, h_zoom, v_zoom)

    # 戻り値確認
    assert param_assert == param_return


def test_get_z_voxel_plane_point_on_spatial_id_01():
    """空間IDからZ方向ボクセル境界座標を取得、正常な空間IDを入力
    """
    # 試験用データの定義
    # 座標1, 水平/垂直方向精度
    point_01 = SpatialPoint(140.08785504, 36.10377479, 26.6)
    h_zoom = 2
    v_zoom = 1

    # 座標1の空間IDを取得
    points = [point_01]
    spatial_ids_set = set(get_spatial_ids_on_points(points, h_zoom, v_zoom))
    spatial_ids_list = list(spatial_ids_set)
    spatial_id_01 = spatial_ids_list[0]

    # 空間IDの頂点座標を取得
    spatial_id_points = get_point_on_spatial_id(
        spatial_id_01, Point_Option.VERTEX, PROJECTED_CRS)

    # 期待値の定義
    # Z方向ボクセルの境界座標取得
    param_assert = min([point.alt for point in spatial_id_points])

    # 試験実施
    # 戻り値取得
    param_return = get_z_voxel_plane_point_on_spatial_id(spatial_id_01)

    # 戻り値確認
    assert param_assert == param_return


def test_get_y_voxel_plane_point_on_spatial_id_01():
    """空間IDからY方向ボクセル境界座標を取得、正常な空間IDを入力
    """
    # 試験用データの定義
    # 座標1, 水平/垂直方向精度
    point_01 = SpatialPoint(140.08785504, 36.10377479, 26.6)
    h_zoom = 2
    v_zoom = 1

    # 座標1の空間IDを取得
    points = [point_01]
    spatial_ids_set = set(get_spatial_ids_on_points(points, h_zoom, v_zoom))
    spatial_ids_list = list(spatial_ids_set)
    spatial_id_01 = spatial_ids_list[0]

    # 空間IDの頂点座標を取得
    spatial_id_points = get_point_on_spatial_id(
        spatial_id_01, Point_Option.VERTEX, PROJECTED_CRS)

    # 期待値の定義
    # Y方向ボクセルの境界座標取得
    param_assert = max([point.y for point in spatial_id_points])

    # 試験実施
    # 戻り値取得
    param_return = get_y_voxel_plane_point_on_spatial_id(spatial_id_01)

    # 戻り値確認
    assert param_assert == param_return


def test_get_x_voxel_plane_point_on_spatial_id_01():
    """空間IDからX方向ボクセル境界座標を取得、正常な空間IDを入力
    """
    # 試験用データの定義
    # 座標1, 水平/垂直方向精度
    point_01 = SpatialPoint(140.08785504, 36.10377479, 26.6)
    h_zoom = 2
    v_zoom = 1

    # 座標1の空間IDを取得
    points = [point_01]
    spatial_ids_set = set(get_spatial_ids_on_points(points, h_zoom, v_zoom))
    spatial_ids_list = list(spatial_ids_set)
    spatial_id_01 = spatial_ids_list[0]

    # 空間IDの頂点座標を取得
    spatial_id_points = get_point_on_spatial_id(
        spatial_id_01, Point_Option.VERTEX, PROJECTED_CRS)

    # 期待値の定義
    # X方向ボクセルの境界座標取得
    param_assert = min([point.x for point in spatial_id_points])

    # 試験実施
    # 戻り値取得
    param_return = get_x_voxel_plane_point_on_spatial_id(spatial_id_01)

    # 戻り値確認
    assert param_assert == param_return


def test_create_sequence_list_01():
    """指定範囲の連番リストを作成、始点ID < 終点ID となるIDを入力
    + 試験詳細
      - 入力条件
        - 始点ID < 終点ID となるIDを入力
      - 確認内容
        - 始点IDから終点ID迄の整数値が昇順で格納された配列が返却されること
    """
    # 試験用データの定義
    # 始点ID、終点IDを定義
    start_id = 1
    end_id = 10

    # 期待値の定義
    # 指定範囲の連番リストを定義
    param_assert = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]

    # 試験実施
    # 戻り値の取得
    param_return = create_sequence_list(start_id, end_id)

    # 戻り値の確認
    assert param_assert == list(param_return)


def test_create_sequence_list_02():
    """指定範囲の連番リストを作成、終点ID < 始点IDとなるIDを入力
    + 試験詳細
      - 入力条件
        - 終点ID < 始点IDとなるIDを入力
      - 確認内容
        - 終点IDから始点ID迄の整数値が昇順で格納された配列が返却されること
    """
    # 試験用データの定義
    # 始点ID、終点IDを定義
    start_id = 10
    end_id = 1

    # 期待値の定義
    # 指定範囲の連番リストを定義
    param_assert = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]

    # 試験実施
    # 戻り値の取得
    param_return = create_sequence_list(start_id, end_id)

    # 戻り値の確認
    assert param_assert == list(param_return)


def test_create_sequence_list_03():
    """指定範囲の連番リストを作成、終点ID == 始点IDとなるIDを入力
    + 試験詳細
      - 入力条件
        - 始点ID == 終点IDとなるIDを入力
      - 確認内容
        - 始点IDのみ格納さした配列が返却されること
    """
    # 試験用データの定義
    # 始点ID、終点IDを定義
    start_id = 10
    end_id = 10

    # 期待値の定義
    # 指定範囲の連番リストを定義
    param_assert = [10]

    # 試験実施
    # 戻り値の取得
    param_return = create_sequence_list(start_id, end_id)

    # 戻り値の確認
    assert param_assert == param_return


def test_generate_z_voxel_plane_01():
    """Z軸ボクセル境界取得、境界面の数が複数
    + 試験詳細
      - 入力条件
        - 境界面の数が2つとなる始点、終点を入力
        - 始点ID < 終点IDとなるIDを入力
      - 確認内容
        - 境界面が2つ取得されていること
        - 入力した始点、終点IDに対応した境界面となっていること
    """
    # 試験用データの定義
    # 始点、終点のZ軸IDを定義
    start_z_id = 1
    end_z_id = 3
    sequence_list = [1, 2, 3]

    # 水平/垂直方向精度を定義
    h_zoom = 1
    v_zoom = 1

    # 期待値の定義
    # Z軸ボクセル境界面の単位法線ベクトル
    z_norm_vector = Vector([0, 0, 1])
    param_assert_list = []

    # 境界面を生成
    for z_id in sequence_list[1:]:
        # 期待値用の空間ID定義
        id_element_list = [h_zoom, 0, 0, v_zoom, z_id]
        spatial_id = '/'.join(map(str, id_element_list))

        # 空間IDの頂点座標を取得
        spatial_id_points = get_point_on_spatial_id(
            spatial_id, Point_Option.VERTEX, PROJECTED_CRS)

        # Z方向ボクセルの境界座標取得
        z_point = min([point.alt for point in spatial_id_points])

        # 境界面上の座標を定義
        plane_point = (0, 0, z_point)

        param_assert_list.append(
                Plane(point=plane_point, normal=z_norm_vector))

    # 試験実施
    # 戻り値の取得
    param_return = generate_z_voxel_plane(
            start_z_id, end_z_id, h_zoom, v_zoom)

    # 境界面の確認
    loop_cnt = 0
    for param in param_return:
        assert (param_assert_list[loop_cnt].point == param.point).all()
        assert (param_assert_list[loop_cnt].normal == param.normal).all()
        loop_cnt += 1

    # 境界面の個数確認
    assert 2 == loop_cnt


def test_generate_z_voxel_plane_02():
    """Z軸ボクセル境界取得、境界面の数が1
    + 試験詳細
      - 入力条件
        - 境界面の数が1つとなる始点、終点を入力
      - 確認内容
        - 境界面が1つ取得されていること
        - 入力した始点、終点IDに対応した境界面となっていること
    """
    # 試験用データの定義
    # 始点、終点のZ軸IDを定義
    start_z_id = 11
    end_z_id = 12
    sequence_list = [11, 12]

    # 水平/垂直方向精度を定義
    h_zoom = 1
    v_zoom = 1

    # 期待値の定義
    # Z軸ボクセル境界面の単位法線ベクトル
    z_norm_vector = Vector([0, 0, 1])
    param_assert_list = []

    # 境界面を生成
    for z_id in sequence_list[1:]:
        # 期待値用の空間ID定義
        id_element_list = [h_zoom, 0, 0, v_zoom, z_id]
        spatial_id = '/'.join(map(str, id_element_list))

        # 空間IDの頂点座標を取得
        spatial_id_points = get_point_on_spatial_id(
            spatial_id, Point_Option.VERTEX, PROJECTED_CRS)

        # Z方向ボクセルの境界座標取得
        z_point = min([point.alt for point in spatial_id_points])

        # 境界面上の座標を定義
        plane_point = (0, 0, z_point)

        param_assert_list.append(
                Plane(point=plane_point, normal=z_norm_vector))

    # 試験実施
    # 戻り値の取得
    param_return = generate_z_voxel_plane(
            start_z_id, end_z_id, h_zoom, v_zoom)

    # 境界面の確認
    loop_cnt = 0
    for param in param_return:
        assert (param_assert_list[loop_cnt].point == param.point).all()
        assert (param_assert_list[loop_cnt].normal == param.normal).all()
        loop_cnt += 1

    # 境界面の個数確認
    assert 1 == loop_cnt


def test_generate_z_voxel_plane_03():
    """Z軸ボクセル境界取得、境界面の数が0
    + 試験詳細
      - 入力条件
        - 境界面の数が0となる始点、終点を入力
      - 確認内容
        - 境界面が取得されないこと
    """
    # 試験用データの定義
    # 始点、終点のZ軸IDを定義
    start_z_id = 11
    end_z_id = 11

    # 水平/垂直方向精度を定義
    h_zoom = 1
    v_zoom = 1

    # 期待値の定義
    # Z軸ボクセル境界面の単位法線ベクトル
    z_norm_vector = Vector([0, 0, 1])

    # 境界面の数は0なので空配列を設定
    param_assert_list = []

    # 試験実施
    # 戻り値の取得
    param_return = generate_z_voxel_plane(
            start_z_id, end_z_id, h_zoom, v_zoom)

    # 境界面の確認
    loop_cnt = 0
    for param in param_return:
        # 面の数は0のため、期待通りの動作であればループ処理に入らない
        loop_cnt += 1

    # 境界面の個数確認
    assert 0 == loop_cnt


def test_generate_z_voxel_plane_04():
    """Z軸ボクセル境界取得、終点ID < 始点IDとなるIDを入力
    + 試験詳細
      - 入力条件
        - 境界面の数が10となる始点、終点を入力
        - 終点ID < 始点IDとなるIDを入力
      - 確認内容
        - 境界面が10面取得されていること
        - 入力した始点、終点IDに対応した境界面となっていること
    """
    # 試験用データの定義
    # 始点、終点のZ軸IDを定義
    start_z_id = 20
    end_z_id = 10
    sequence_list = [10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20]

    # 水平/垂直方向精度を定義
    h_zoom = 1
    v_zoom = 1

    # 期待値の定義
    # Z軸ボクセル境界面の単位法線ベクトル
    z_norm_vector = Vector([0, 0, 1])
    param_assert_list = []

    # 境界面を生成
    for z_id in sequence_list[1:]:
        # 期待値用の空間ID定義
        id_element_list = [h_zoom, 0, 0, v_zoom, z_id]
        spatial_id = '/'.join(map(str, id_element_list))

        # 空間IDの頂点座標を取得
        spatial_id_points = get_point_on_spatial_id(
            spatial_id, Point_Option.VERTEX, PROJECTED_CRS)

        # Z方向ボクセルの境界座標取得
        z_point = min([point.alt for point in spatial_id_points])

        # 境界面上の座標を定義
        plane_point = (0, 0, z_point)

        param_assert_list.append(
                Plane(point=plane_point, normal=z_norm_vector))

    # 試験実施
    # 戻り値の取得
    param_return = generate_z_voxel_plane(
            start_z_id, end_z_id, h_zoom, v_zoom)

    # 境界面の確認
    loop_cnt = 0
    for param in param_return:
        assert (param_assert_list[loop_cnt].point == param.point).all()
        assert (param_assert_list[loop_cnt].normal == param.normal).all()
        loop_cnt += 1

    # 境界面の個数確認
    assert 10 == loop_cnt


def test_generate_y_voxel_plane_01():
    """Y軸ボクセル境界取得、境界面の数が複数
    + 試験詳細
      - 入力条件
        - 境界面の数が2つとなる始点、終点を入力
        - 始点ID < 終点ID となるIDを入力
      - 確認内容
        - 境界面が2つ取得されていること
        - 入力した始点、終点IDに対応した境界面となっていること
    """
    # 試験用データの定義
    # 始点、終点のY軸IDを定義
    start_y_id = 1
    end_y_id = 3
    sequence_list = [1, 2, 3]

    # 水平/垂直方向精度を定義
    h_zoom = 1
    v_zoom = 1

    # 期待値の定義
    # Y軸ボクセル境界面の単位法線ベクトル
    y_norm_vector = Vector([0, 1, 0])
    param_assert_list = []

    # 境界面を生成
    for y_id in sequence_list[1:]:
        # 期待値用の空間ID定義
        id_element_list = [h_zoom, 0, y_id, v_zoom, 0]
        spatial_id = '/'.join(map(str, id_element_list))

        # 空間IDの頂点座標を取得
        spatial_id_points = get_point_on_spatial_id(
            spatial_id, Point_Option.VERTEX, PROJECTED_CRS)

        # Y方向ボクセルの境界座標取得
        y_point = max([point.y for point in spatial_id_points])

        # 境界面上の座標を定義
        plane_point = (0, y_point, 0)

        param_assert_list.append(
                Plane(point=plane_point, normal=y_norm_vector))

    # 試験実施
    # 戻り値の取得
    param_return = generate_y_voxel_plane(
            start_y_id, end_y_id, h_zoom, v_zoom)

    # 境界面の確認
    loop_cnt = 0
    for param in param_return:
        assert (param_assert_list[loop_cnt].point == param.point).all()
        assert (param_assert_list[loop_cnt].normal == param.normal).all()
        loop_cnt += 1

    # 境界面の個数確認
    assert 2 == loop_cnt


def test_generate_y_voxel_plane_02():
    """Y軸ボクセル境界取得、境界面の数が1つ
    + 試験詳細
      - 入力条件
        - 境界面の数が1つとなる始点、終点を入力
      - 確認内容
        - 境界面が1つ取得されていること
        - 入力した始点、終点IDに対応した境界面となっていること
    """
    # 試験用データの定義
    # 始点、終点のY軸IDを定義
    start_y_id = 6
    end_y_id = 7
    sequence_list = [6, 7]

    # 水平/垂直方向精度を定義
    h_zoom = 1
    v_zoom = 1

    # 期待値の定義
    # Y軸ボクセル境界面の単位法線ベクトル
    y_norm_vector = Vector([0, 1, 0])
    param_assert_list = []

    # 境界面を生成
    for y_id in sequence_list[1:]:
        # 期待値用の空間ID定義
        id_element_list = [h_zoom, 0, y_id, v_zoom, 0]
        spatial_id = '/'.join(map(str, id_element_list))

        # 空間IDの頂点座標を取得
        spatial_id_points = get_point_on_spatial_id(
            spatial_id, Point_Option.VERTEX, PROJECTED_CRS)

        # Y方向ボクセルの境界座標取得
        y_point = max([point.y for point in spatial_id_points])

        # 境界面上の座標を定義
        plane_point = (0, y_point, 0)

        param_assert_list.append(
                Plane(point=plane_point, normal=y_norm_vector))

    # 試験実施
    # 戻り値の取得
    param_return = generate_y_voxel_plane(
            start_y_id, end_y_id, h_zoom, v_zoom)

    # 境界面の確認
    loop_cnt = 0
    for param in param_return:
        assert (param_assert_list[loop_cnt].point == param.point).all()
        assert (param_assert_list[loop_cnt].normal == param.normal).all()
        loop_cnt += 1

    # 境界面の個数確認
    assert 1 == loop_cnt


def test_generate_y_voxel_plane_03():
    """Y軸ボクセル境界取得、境界面の数が0
    + 試験詳細
      - 入力条件
        - 境界面の数が0となる始点、終点を入力
      - 確認内容
        - 境界面が取得されていないこと
    """
    # 試験用データの定義
    # 始点、終点のY軸IDを定義
    start_y_id = 6
    end_y_id = 6
    sequence_list = [6]

    # 水平/垂直方向精度を定義
    h_zoom = 1
    v_zoom = 1

    # 期待値の定義
    # Y軸ボクセル境界面の単位法線ベクトル
    y_norm_vector = Vector([0, 1, 0])

    # 境界面の数は0なので空配列を設定
    param_assert_list = []

    # 試験実施
    # 戻り値の取得
    param_return = generate_y_voxel_plane(
            start_y_id, end_y_id, h_zoom, v_zoom)

    # 境界面の確認
    loop_cnt = 0
    for param in param_return:
        # 面の数は0のため、期待通りの動作であればループ処理に入らない
        loop_cnt += 1

    # 境界面の個数確認
    assert 0 == loop_cnt


def test_generate_y_voxel_plane_04():
    """Y軸ボクセル境界取得、終点ID < 始点IDとなるIDを入力
    + 試験詳細
      - 入力条件
        - 境界面の数が10となる始点、終点を入力
        - 終点ID < 始点IDとなるIDを入力
      - 確認内容
        - 境界面が10面取得されていること
        - 入力した始点、終点IDに対応した境界面となっていること
    """
    # 試験用データの定義
    # 始点、終点のY軸IDを定義
    start_y_id = 20
    end_y_id = 10
    sequence_list = [10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20]

    # 水平/垂直方向精度を定義
    h_zoom = 1
    v_zoom = 1

    # 期待値の定義
    # Y軸ボクセル境界面の単位法線ベクトル
    y_norm_vector = Vector([0, 1, 0])
    param_assert_list = []

    # 境界面を生成
    for y_id in sequence_list[1:]:
        # 期待値用の空間ID定義
        id_element_list = [h_zoom, 0, y_id, v_zoom, 0]
        spatial_id = '/'.join(map(str, id_element_list))

        # 空間IDの頂点座標を取得
        spatial_id_points = get_point_on_spatial_id(
            spatial_id, Point_Option.VERTEX, PROJECTED_CRS)

        # Y方向ボクセルの境界座標取得
        y_point = max([point.y for point in spatial_id_points])

        # 境界面上の座標を定義
        plane_point = (0, y_point, 0)

        param_assert_list.append(
                Plane(point=plane_point, normal=y_norm_vector))

    # 試験実施
    # 戻り値の取得
    param_return = generate_y_voxel_plane(
            start_y_id, end_y_id, h_zoom, v_zoom)

    # 境界面の確認
    loop_cnt = 0
    for param in param_return:
        assert (param_assert_list[loop_cnt].point == param.point).all()
        assert (param_assert_list[loop_cnt].normal == param.normal).all()
        loop_cnt += 1

    # 境界面の個数確認
    assert 10 == loop_cnt


def test_generate_x_voxel_plane_01():
    """X軸ボクセル境界取得、境界面の数が複数
    + 試験詳細
      - 入力条件
        - 境界面の数が2つとなる始点、終点を入力
        - 始点ID < 終点IDとなるIDを入力
      - 確認内容
        - 境界面が2つ取得されていること
        - 入力した始点、終点IDに対応した境界面となっていること
    """
    # 試験用データの定義
    # 始点、終点のX軸IDを定義
    start_x_id = 1
    end_x_id = 3
    sequence_list = [1, 2, 3]

    # 水平/垂直方向精度を定義
    h_zoom = 1
    v_zoom = 1

    # 期待値の定義
    # X軸ボクセル境界面の単位法線ベクトル
    x_norm_vector = Vector([1, 0, 0])
    param_assert_list = []

    # 境界面を生成
    for x_id in sequence_list[1:]:
        # 期待値用の空間ID定義
        id_element_list = [h_zoom, x_id, 0, v_zoom, 0]
        spatial_id = '/'.join(map(str, id_element_list))

        # 空間IDの頂点座標を取得
        spatial_id_points = get_point_on_spatial_id(
            spatial_id, Point_Option.VERTEX, PROJECTED_CRS)

        # X方向ボクセルの境界座標取得
        x_point = min([point.x for point in spatial_id_points])

        # 境界面上の座標を定義
        plane_point = (x_point, 0, 0)

        param_assert_list.append(
                Plane(point=plane_point, normal=x_norm_vector))

    # 試験実施
    # 戻り値の取得
    param_return = generate_x_voxel_plane(
            start_x_id, end_x_id, h_zoom, v_zoom)

    # 境界面の確認
    loop_cnt = 0
    for param in param_return:
        assert (param_assert_list[loop_cnt].point == param.point).all()
        assert (param_assert_list[loop_cnt].normal == param.normal).all()
        loop_cnt += 1

    # 境界面の個数確認
    assert 2 == loop_cnt


def test_generate_x_voxel_plane_02():
    """X軸ボクセル境界取得、境界面の数が1つ
    + 試験詳細
      - 入力条件
        - 境界面の数が1つ以上となる始点、終点を入力
      - 確認内容
        - 境界面が1つ取得されていること
        - 入力した始点、終点IDに対応した境界面となっていること
    """
    # 試験用データの定義
    # 始点、終点のX軸IDを定義
    start_x_id = 6
    end_x_id = 7
    sequence_list = [6, 7]

    # 水平/垂直方向精度を定義
    h_zoom = 1
    v_zoom = 1

    # 期待値の定義
    # X軸ボクセル境界面の単位法線ベクトル
    x_norm_vector = Vector([1, 0, 0])
    param_assert_list = []

    # 境界面を生成
    for x_id in sequence_list[1:]:
        # 期待値用の空間ID定義
        id_element_list = [h_zoom, x_id, 0, v_zoom, 0]
        spatial_id = '/'.join(map(str, id_element_list))

        # 空間IDの頂点座標を取得
        spatial_id_points = get_point_on_spatial_id(
            spatial_id, Point_Option.VERTEX, PROJECTED_CRS)

        # X方向ボクセルの境界座標取得
        x_point = min([point.x for point in spatial_id_points])

        # 境界面上の座標を定義
        plane_point = (x_point, 0, 0)

        param_assert_list.append(
                Plane(point=plane_point, normal=x_norm_vector))

    # 試験実施
    # 戻り値の取得
    param_return = generate_x_voxel_plane(
            start_x_id, end_x_id, h_zoom, v_zoom)

    # 境界面の確認
    loop_cnt = 0
    for param in param_return:
        assert (param_assert_list[loop_cnt].point == param.point).all()
        assert (param_assert_list[loop_cnt].normal == param.normal).all()
        loop_cnt += 1

    # 境界面の個数確認
    assert 1 == loop_cnt


def test_generate_x_voxel_plane_03():
    """X軸ボクセル境界取得、境界面の数が0
    + 試験詳細
      - 入力条件
        - 境界面の数が0となる始点、終点を入力
      - 確認内容
        - 境界面が取得されないこと
    """
    # 試験用データの定義
    # 始点、終点のX軸IDを定義
    start_x_id = 6
    end_x_id = 6
    sequence_list = [6]

    # 水平/垂直方向精度を定義
    h_zoom = 1
    v_zoom = 1

    # 期待値の定義
    # X軸ボクセル境界面の単位法線ベクトル
    x_norm_vector = Vector([1, 0, 0])

    # 境界面の数は0なので空配列を設定
    param_assert_list = []

    # 試験実施
    # 戻り値の取得
    param_return = generate_x_voxel_plane(
            start_x_id, end_x_id, h_zoom, v_zoom)

    # 境界面の確認
    loop_cnt = 0
    for param in param_return:
        # 面の数は0のため、期待通りの動作であればループ処理に入らない
        loop_cnt += 1

    # 境界面の個数確認
    assert 0 == loop_cnt


def test_generate_x_voxel_plane_04():
    """X軸ボクセル境界取得、終点ID < 始点IDとなるIDを入力
    + 試験詳細
      - 入力条件
        - 境界面の数が10となる始点、終点を入力
        - 終点ID < 始点IDとなるIDを入力
      - 確認内容
        - 境界面が10面取得されていること
        - 入力した始点、終点IDに対応した境界面となっていること
    """
    # 試験用データの定義
    # 始点、終点のX軸IDを定義
    start_x_id = 20
    end_x_id = 10
    sequence_list = [10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20]

    # 水平/垂直方向精度を定義
    h_zoom = 1
    v_zoom = 1

    # 期待値の定義
    # X軸ボクセル境界面の単位法線ベクトル
    x_norm_vector = Vector([1, 0, 0])
    param_assert_list = []

    # 境界面を生成
    for x_id in sequence_list[1:]:
        # 期待値用の空間ID定義
        id_element_list = [h_zoom, x_id, 0, v_zoom, 0]
        spatial_id = '/'.join(map(str, id_element_list))

        # 空間IDの頂点座標を取得
        spatial_id_points = get_point_on_spatial_id(
            spatial_id, Point_Option.VERTEX, PROJECTED_CRS)

        # X方向ボクセルの境界座標取得
        x_point = min([point.x for point in spatial_id_points])

        # 境界面上の座標を定義
        plane_point = (x_point, 0, 0)

        param_assert_list.append(
                Plane(point=plane_point, normal=x_norm_vector))

    # 試験実施
    # 戻り値の取得
    param_return = generate_x_voxel_plane(
            start_x_id, end_x_id, h_zoom, v_zoom)

    # 境界面の確認
    loop_cnt = 0
    for param in param_return:
        assert (param_assert_list[loop_cnt].point == param.point).all()
        assert (param_assert_list[loop_cnt].normal == param.normal).all()
        loop_cnt += 1

    # 境界面の個数確認
    assert 10 == loop_cnt


def test_detect_collision_01():
    """線分と面の衝突判定、衝突箇所あり、X境界面
    + 試験詳細
      - 入力条件
        - 入力された面と衝突する線分
        - 境界面は、X軸ボクセル境界面
      - 確認内容
        - 面と線分が衝突した座標が取得できること
    """
    # 試験用データの定義
    # 始点, 終点
    start_point = Point([-0.5, 0.5, 0.5])
    end_point = Point([1.0, 1.0, 1.0])

    # 視点から終点の線分
    line = Line.from_points(start_point, end_point)

    # 境界面
    plane = Plane(point=(0.0, 0.0, 0.0), normal=Vector([1, 0, 0]))

    # 期待値の定義
    # 進行方向と法線の内積
    direct_dot = np.dot(plane.normal, line.direction)

    # 交点取得
    cross_point = Vector.from_points(plane.point, line.point)
    collision_time = -np.dot(plane.normal, cross_point) / direct_dot
    param_assert = line.to_point(collision_time)

    # 試験実施
    # 戻り値取得
    param_return = detect_collision(plane, line)

    # 戻り値確認
    assert (param_assert == param_return).all()


def test_detect_collision_02():
    """線分と面の衝突判定、衝突箇所なし、X境界面
    + 試験詳細
      - 入力条件
        - 入力された面と衝突しない線分
        - 境界面は、X軸ボクセル境界面
      - 確認内容
        - 戻り値はNone（衝突無し）であること
    """
    # 試験用データの定義
    # 始点, 終点
    start_point = Point([-0.5, 0.5, 0.5])
    end_point = Point([1.0, 1.0, 1.0])

    # 視点から終点の線分
    line = Line.from_points(start_point, end_point)

    # 境界面 - 線分と交差しない面
    plane = Plane(point=(2.0, 1.0, 1.0), normal=Vector([1, 0, 0]))

    # 期待値の定義
    param_assert = None

    # 試験実施
    # 戻り値取得
    param_return = detect_collision(plane, line)

    # 戻り値確認
    if type(param_return) == Point:
        assert (param_assert == param_return).all()
    else:
        assert param_assert == param_return


def test_detect_collision_03():
    """線分と面の衝突判定、線分と面が平行、X境界面
    + 試験詳細
      - 入力条件
        - 入力された面と衝突しない線分
        - 境界面は、X軸ボクセル境界面
      - 確認内容
        - 戻り値はNone（衝突無し）であること
    """
    # 試験用データの定義
    # 始点, 終点
    start_point = Point([1.0, 0.5, 0.5])
    end_point = Point([1.0, 1.0, 1.0])

    # 視点から終点の線分
    line = Line.from_points(start_point, end_point)

    # 境界面 - 線分と交差しない面
    plane = Plane(point=(2.0, 1.0, 1.0), normal=Vector([1, 0, 0]))

    # 期待値の定義
    param_assert = None

    # 試験実施
    # 戻り値取得
    param_return = detect_collision(plane, line)

    # 戻り値確認
    if type(param_return) == Point:
        assert (param_assert == param_return).all()
    else:
        assert param_assert == param_return


def test_detect_collision_04():
    """線分と面の衝突判定、衝突箇所あり、Y境界面
    + 試験詳細
      - 入力条件
        - 入力された面と衝突する線分
        - 境界面は、Y軸ボクセル境界面
      - 確認内容
        - 面と線分が衝突した座標が取得できること
    """
    # 試験用データの定義
    # 始点, 終点
    start_point = Point([0.5, -0.5, 0.5])
    end_point = Point([1.0, 1.0, 1.0])

    # 視点から終点の線分
    line = Line.from_points(start_point, end_point)

    # 境界面
    plane = Plane(point=(0.0, 0.0, 0.0), normal=Vector([0, 1, 0]))

    # 期待値の定義
    # 進行方向と法線の内積
    direct_dot = np.dot(plane.normal, line.direction)

    # 交点取得
    cross_point = Vector.from_points(plane.point, line.point)
    collision_time = -np.dot(plane.normal, cross_point) / direct_dot
    param_assert = line.to_point(collision_time)

    # 試験実施
    # 戻り値取得
    param_return = detect_collision(plane, line)

    # 戻り値確認
    assert (param_assert == param_return).all()


def test_detect_collision_05():
    """線分と面の衝突判定、衝突箇所なし、Y境界面
    + 試験詳細
      - 入力条件
        - 入力された面と衝突しない線分
        - 境界面は、Y軸ボクセル境界面
      - 確認内容
        - 戻り値はNone（衝突無し）であること
    """
    # 試験用データの定義
    # 始点, 終点
    start_point = Point([0.5, -0.5, 0.5])
    end_point = Point([1.0, 1.0, 1.0])

    # 視点から終点の線分
    line = Line.from_points(start_point, end_point)

    # 境界面 - 線分と交差しない面
    plane = Plane(point=(2.0, 1.5, 1.0), normal=Vector([0, 1, 0]))

    # 期待値の定義
    param_assert = None

    # 試験実施
    # 戻り値取得
    param_return = detect_collision(plane, line)

    # 戻り値確認
    if type(param_return) == Point:
        assert (param_assert == param_return).all()
    else:
        assert param_assert == param_return


def test_detect_collision_06():
    """線分と面の衝突判定、線分と面が平行、Y境界面
    + 試験詳細
      - 入力条件
        - 入力された面と衝突しない線分
        - 境界面は、Y軸ボクセル境界面
      - 確認内容
        - 戻り値はNone（衝突無し）であること
    """
    # 試験用データの定義
    # 始点, 終点
    start_point = Point([0.0, 1.0, 0.5])
    end_point = Point([1.0, 1.0, 1.0])

    # 視点から終点の線分
    line = Line.from_points(start_point, end_point)

    # 境界面 - 線分と交差しない面
    plane = Plane(point=(2.0, 2.0, 1.0), normal=Vector([0, 1, 0]))

    # 期待値の定義
    param_assert = None

    # 試験実施
    # 戻り値取得
    param_return = detect_collision(plane, line)

    # 戻り値確認
    if type(param_return) == Point:
        assert (param_assert == param_return).all()
    else:
        assert param_assert == param_return


def test_detect_collision_07():
    """線分と面の衝突判定、衝突箇所あり、Z境界面
    + 試験詳細
      - 入力条件
        - 入力された面と衝突する線分
        - 境界面は、Z軸ボクセル境界面
      - 確認内容
        - 面と線分が衝突した座標が取得できること
    """
    # 試験用データの定義
    # 始点, 終点
    start_point = Point([0.5, 0.5, -0.5])
    end_point = Point([1.0, 1.0, 1.0])

    # 視点から終点の線分
    line = Line.from_points(start_point, end_point)

    # 境界面
    plane = Plane(point=(0.0, 0.0, 0.0), normal=Vector([0, 0, 1]))

    # 期待値の定義
    # 進行方向と法線の内積
    direct_dot = np.dot(plane.normal, line.direction)

    # 交点取得
    cross_point = Vector.from_points(plane.point, line.point)
    collision_time = -np.dot(plane.normal, cross_point) / direct_dot
    param_assert = line.to_point(collision_time)

    # 試験実施
    # 戻り値取得
    param_return = detect_collision(plane, line)

    # 戻り値確認
    assert (param_assert == param_return).all()


def test_detect_collision_08():
    """線分と面の衝突判定、衝突箇所なし、Z境界面
    + 試験詳細
      - 入力条件
        - 入力された面と衝突しない線分
        - 境界面は、Z軸ボクセル境界面
      - 確認内容
        - 戻り値はNone（衝突無し）であること
    """
    # 試験用データの定義
    # 始点, 終点
    start_point = Point([0.5, -0.5, 0.5])
    end_point = Point([1.0, 1.0, 1.0])

    # 視点から終点の線分
    line = Line.from_points(start_point, end_point)

    # 境界面 - 線分と交差しない面
    plane = Plane(point=(2.0, 1.0, 0.0), normal=Vector([0, 0, 1]))

    # 期待値の定義
    param_assert = None

    # 試験実施
    # 戻り値取得
    param_return = detect_collision(plane, line)

    # 戻り値確認
    if type(param_return) == Point:
        assert (param_assert == param_return).all()
    else:
        assert param_assert == param_return


def test_detect_collision_09():
    """線分と面の衝突判定、線分と面が平行、Z境界面
    + 試験詳細
      - 入力条件
        - 入力された面と衝突しない線分
        - 境界面は、Z軸ボクセル境界面
      - 確認内容
        - 戻り値はNone（衝突無し）であること
    """
    # 試験用データの定義
    # 始点, 終点
    start_point = Point([0.0, 1.0, 1.0])
    end_point = Point([1.0, 1.0, 1.0])

    # 視点から終点の線分
    line = Line.from_points(start_point, end_point)

    # 境界面 - 線分と交差しない面
    plane = Plane(point=(2.0, 1.0, 2.0), normal=Vector([0, 0, 1]))

    # 期待値の定義
    param_assert = None

    # 試験実施
    # 戻り値取得
    param_return = detect_collision(plane, line)

    # 戻り値確認
    if type(param_return) == Point:
        assert (param_assert == param_return).all()
    else:
        assert param_assert == param_return


def test_detect_collision_10():
    """線分と面の衝突判定、線分の始点が境界面上
    + 試験詳細
      - 入力条件
        - 始点が境界面上に存在する線分
      - 確認内容
        - 面と線分が衝突した座標が取得できること
    """
    # 試験用データの定義
    # 始点, 終点
    start_point = Point([1.0, 1.0, 1.0])
    end_point = Point([2.0, 2.0, 2.0])

    # 視点から終点の線分
    line = Line.from_points(start_point, end_point)

    # 境界面
    plane = Plane(point=(1.0, 0.0, 0.0), normal=Vector([1, 0, 0]))

    # 期待値の定義
    # 進行方向と法線の内積
    direct_dot = np.dot(plane.normal, line.direction)

    # 交点取得
    cross_point = Vector.from_points(plane.point, line.point)
    collision_time = -np.dot(plane.normal, cross_point) / direct_dot
    param_assert = line.to_point(collision_time)

    # 試験実施
    # 戻り値取得
    param_return = detect_collision(plane, line)

    # 戻り値確認
    assert (param_assert == param_return).all()


def test_detect_collision_11():
    """線分と面の衝突判定、線分の終点が境界面上
    + 試験詳細
      - 入力条件
        - 終点が境界面上に存在する線分
      - 確認内容
        - 面と線分が衝突した座標が取得できること
    """
    # 試験用データの定義
    # 始点, 終点
    start_point = Point([-0.5, 0.5, 0.5])
    end_point = Point([1.0, 1.0, 1.0])

    # 視点から終点の線分
    line = Line.from_points(start_point, end_point)

    # 境界面
    plane = Plane(point=(1.0, 0.0, 0.0), normal=Vector([1, 0, 0]))

    # 期待値の定義
    # 進行方向と法線の内積
    direct_dot = np.dot(plane.normal, line.direction)

    # 交点取得
    cross_point = Vector.from_points(plane.point, line.point)
    collision_time = -np.dot(plane.normal, cross_point) / direct_dot
    param_assert = line.to_point(collision_time)

    # 試験実施
    # 戻り値取得
    param_return = detect_collision(plane, line)

    # 戻り値確認
    assert (param_assert == param_return).all()


def test__get_plane_cross_points_01():
    """始点、終点と境界面との交点取得、取得対象外の座標指定なし
    + 試験詳細
      - 入力条件
        - 境界面との交点が発生する始点、終点、境界面を入力
        - 交点から除外するPointのリストは入力なし
      - 確認内容
        - 境界面との交点が返却されること。
    """
    # 試験用データの定義
    # 境界面定義
    plane_list = []

    for cnt in range(5):
        plane_list.append(
                Plane(point=(cnt, 0.0, 0.0), normal=Vector([1, 0, 0])))

    # 始点、終点定義
    start_point = Point([0.0, 0.0, 0.0])
    end_point = Point([5.0, 5.0, 5.0])

    # 取得対象外のPointリスト
    exclude_point_set = set()

    # 期待値の定義
    # 線分と境界面の交点
    param_assert = [
            Point([0.0, 0.0, 0.0]),
            Point([1.0, 1.0, 1.0]),
            Point([2.0, 2.0, 2.0]),
            Point([3.0, 3.0, 3.0]),
            Point([4.0, 4.0, 4.0])]
    # 重複除去用の集合
    exclude_point_set_assert = set()
    for point in param_assert:
        exclude_point_set_assert.add("{0}_{1}_{2}".format(point[0], point[1], point[2]))

    # 試験実施
    # 戻り値取得
    param_return = _get_plane_cross_points(
            plane_list, start_point, end_point, exclude_point_set)

    # 戻り値確認
    assert len(param_assert) == len(param_return)
    assert exclude_point_set == exclude_point_set_assert

    for cnt in range(len(param_assert)):
        assert (param_assert[cnt] == param_return[cnt]).all()


def test__get_plane_cross_points_02():
    """始点、終点と境界面との交点取得、取得対象外の座標指定あり、対象外となる座標を含む
    + 試験詳細
      - 入力条件
        - 境界面との交点が発生する始点、終点、境界面を入力
        - 境界面との交点が取得対象外となるPointのリスト入力あり
      - 確認内容
        - 境界面との交点が返却されること。
        - 取得対象外としたPointが含まれていないこと
    """
    # 試験用データの定義
    # 境界面定義
    plane_list = []

    for cnt in range(5):
        plane_list.append(
                Plane(point=(cnt, 0.0, 0.0), normal=Vector([1, 0, 0])))

    # 始点、終点定義
    start_point = Point([0.0, 0.0, 0.0])
    end_point = Point([5.0, 5.0, 5.0])

    # 取得対象外とするPointリスト
    exclude_point_list = [
            Point([1.0, 1.0, 1.0]),
            Point([2.0, 2.0, 2.0]),
            Point([2.5, 2.0, 2.0]),
            Point([3.0, 3.0, 3.0]),
            Point([3.0, 3.5, 2.0]),
            Point([4.0, 4.0, 4.0])]
    exclude_point_set = set()
    for point in exclude_point_list:
        exclude_point_set.add("{0}_{1}_{2}".format(point[0], point[1], point[2]))

    # 期待値の定義
    param_assert = [
            Point([0.0, 0.0, 0.0])]
    # 重複除去用の集合
    exclude_point_set_assert = set()
    for point in exclude_point_list:
        exclude_point_set_assert.add("{0}_{1}_{2}".format(point[0], point[1], point[2]))
    for point in param_assert:
        exclude_point_set_assert.add("{0}_{1}_{2}".format(point[0], point[1], point[2]))

    # 試験実施
    # 戻り値取得
    param_return = _get_plane_cross_points(
            plane_list, start_point, end_point, exclude_point_set)

    # 戻り値確認
    assert len(param_assert) == len(param_return)
    assert exclude_point_set == exclude_point_set_assert

    for cnt in range(len(param_assert)):
        assert (param_assert[cnt] == param_return[cnt]).all()

def test__get_plane_cross_points_03():
    """始点、終点と境界面との交点取得、線と境界面が平行
    + 試験詳細
      - 入力条件
        - 線分が境界面との平行となる始点、終点、境界面を入力
      - 確認内容
        - 戻り値は空の配列（交点なし）であること
    """
    # 試験用データの定義
    # 境界面定義
    plane_list = []

    for cnt in range(5):
        plane_list.append(
                Plane(point=(cnt, 0.0, 0.0), normal=Vector([1, 0, 0])))

    # 始点、終点定義
    start_point = Point([0.0, 0.0, 0.0])
    end_point = Point([0.0, 5.0, 5.0])

    # 取得対象外のPointリスト
    exclude_point_list = []
    exclude_point_set = set()
    for point in exclude_point_list:
        exclude_point_set.add("{0}_{1}_{2}".format(point[0], point[1], point[2]))

    # 期待値の定義
    # 戻り値の要素数
    param_assert = 0

    # 試験実施
    # 戻り値取得
    param_return = _get_plane_cross_points(
            plane_list, start_point, end_point, exclude_point_set)

    # 戻り値確認
    assert param_assert == len(param_return)


def test__get_plane_cross_points_04():
    """始点、終点と境界面との交点取得、線と境界面の交点なし
    + 試験詳細
      - 入力条件
        - 線分が境界面との平行ではなく、境界面と交差しない始点、終点、境界面を入力
      - 確認内容
        - 戻り値は空の配列（交点なし）であること
    """
    # 試験用データの定義
    # 境界面定義
    plane_list = []

    for cnt in range(5):
        plane_list.append(
                Plane(point=(cnt, 0.0, 0.0), normal=Vector([1, 0, 0])))

    # 始点、終点定義
    start_point = Point([0.2, 0.0, 0.0])
    end_point = Point([0.5, 5.0, 5.0])

    # 取得対象外のPointリスト
    exclude_point_list = []
    exclude_point_set = set()
    for point in exclude_point_list:
        exclude_point_set.add("{0}_{1}_{2}".format(point[0], point[1], point[2]))

    # 期待値の定義
    # 戻り値の要素数
    param_assert = 0

    # 試験実施
    # 戻り値取得
    param_return = _get_plane_cross_points(
            plane_list, start_point, end_point, exclude_point_set)

    # 戻り値確認
    assert param_assert == len(param_return)


def test__get_plane_cross_points_05():
    """始点、終点と境界面との交点取得、始点、終点が境界面上にない
    + 試験詳細
      - 入力条件
        - 線分と境界面との交点が発生する始点、終点、境界面を入力
        - 線分の始点と終点は、境界面上に存在しない。
      - 確認内容
        - 線分と境界面との交点を格納したリストが返却されること
        - 戻り値には、始点、終点の線分範囲外の座標が含まれないこと
    """
    # 試験用データの定義
    # 境界面定義
    plane_list = []

    for cnt in range(5):
        plane_list.append(
                Plane(point=(cnt, 0.0, 0.0), normal=Vector([1, 0, 0])))

    # 始点、終点定義
    start_point = Point([-0.2, -0.2, -0.2])
    end_point = Point([4.2, 4.2, 4.2])

    # 取得対象外のPointリスト
    exclude_point_list = []
    exclude_point_set = set()
    for point in exclude_point_list:
        exclude_point_set.add("{0}_{1}_{2}".format(point[0], point[1], point[2]))

    # 期待値の定義
    # 交点リスト
    param_assert = [
            Point([0.0, 0.0, 0.0]),
            Point([1.0, 1.0, 1.0]),
            Point([2.0, 2.0, 2.0]),
            Point([3.0, 3.0, 3.0]),
            Point([4.0, 4.0, 4.0])]
    # 重複除去用の集合
    exclude_point_set_assert = set()
    for point in exclude_point_list:
        exclude_point_set_assert.add("{0}_{1}_{2}".format(point[0], point[1], point[2]))
    for point in param_assert:
        exclude_point_set_assert.add("{0}_{1}_{2}".format(point[0], point[1], point[2]))

    # 試験実施
    # 戻り値取得
    param_return = _get_plane_cross_points(
            plane_list, start_point, end_point, exclude_point_set)

    # 戻り値確認
    assert len(param_assert) == len(param_return)
    assert exclude_point_set == exclude_point_set_assert

    for cnt in range(len(param_assert)):
        assert (param_assert[cnt] == param_return[cnt]).all()


def test_get_spatial_ids_on_line_01():
    """指定範囲の空間ID変換(線分)取得、X境界と交差する線分
    + 試験詳細
      - 入力条件
        - X境界面との交点が発生する始点、終点
        - 水平方向精度：10
        - 垂直方向精度：10
        - CRS: 入力なし
      - 確認内容
        - 線分と境界面の交点の空間IDが取得できること
    """
    # 期待値の定義
    # 空間IDリスト定義
    param_assert = [
        '10/10/10/10/10',
        '10/11/10/10/10',
        '10/12/10/10/10',
        '10/13/10/10/10']

    # 試験用データの定義
    # 始点、終点の空間ID定義
    start_spatial_id = param_assert[0]
    end_spatial_id = param_assert[-1]

    # 始点、終点の定義 - 空間IDから座標を取得するライブラリを利用
    start_point = get_point_on_spatial_id(
            start_spatial_id, Point_Option.CENTER)[0]
    end_point = get_point_on_spatial_id(
            end_spatial_id, Point_Option.CENTER)[0]
    start_projection_point = get_point_on_spatial_id(
            start_spatial_id, Point_Option.CENTER, 3857)[0]
    end_projection_point = get_point_on_spatial_id(
            end_spatial_id, Point_Option.CENTER, 3857)[0]
    start_projection_point.lon = start_point.lon
    end_projection_point.lon = end_point.lon
    start_projection_point.lat = start_point.lat
    end_projection_point.lat = end_point.lat

    h_zoom = 10
    v_zoom = 10

    # 空間ID取得
    param_return = get_spatial_ids_on_line(
            start_projection_point, end_projection_point, h_zoom, v_zoom, GEOGRAPHIC_CRS)

    # 戻り値確認
    # 空間IDの要素数が期待値通りであること
    assert len(param_assert) == len(param_return)

    # 期待値通りの空間IDが返却されていること
    for param in param_return:
        assert True == (param in param_assert)


def test_get_spatial_ids_on_line_02():
    """指定範囲の空間ID変換(線分)取得、Y境界と交差する線分
    + 試験詳細
      - 入力条件
        - Y境界面との交点が発生する始点、終点
        - 水平方向精度：10
        - 垂直方向精度：10
        - CRS: 入力なし
      - 確認内容
        - 線分と境界面の交点の空間IDが取得できること
    """
    # 期待値の定義
    # 空間IDリスト定義
    param_assert = [
        '10/10/10/10/10',
        '10/10/11/10/10',
        '10/10/12/10/10',
        '10/10/13/10/10']

    # 試験用データの定義
    # 始点、終点の空間ID定義
    start_spatial_id = param_assert[0]
    end_spatial_id = param_assert[-1]

    # 始点、終点の定義 - 空間IDから座標を取得するライブラリを利用
    start_point = get_point_on_spatial_id(
            start_spatial_id, Point_Option.CENTER)[0]
    end_point = get_point_on_spatial_id(
            end_spatial_id, Point_Option.CENTER)[0]
    start_projection_point = get_point_on_spatial_id(
            start_spatial_id, Point_Option.CENTER, 3857)[0]
    end_projection_point = get_point_on_spatial_id(
            end_spatial_id, Point_Option.CENTER, 3857)[0]
    start_projection_point.lon = start_point.lon
    end_projection_point.lon = end_point.lon
    start_projection_point.lat = start_point.lat
    end_projection_point.lat = end_point.lat

    h_zoom = 10
    v_zoom = 10

    # 空間ID取得
    param_return = get_spatial_ids_on_line(
            start_projection_point, end_projection_point, h_zoom, v_zoom, GEOGRAPHIC_CRS)

    # 戻り値確認
    # 空間IDの要素数が期待値通りであること
    assert len(param_assert) == len(param_return)

    # 期待値通りの空間IDが返却されていること
    for param in param_return:
        assert True == (param in param_assert)


def test_get_spatial_ids_on_line_03():
    """指定範囲の空間ID変換(線分)取得、Z境界と交差する線分
    + 試験詳細
      - 入力条件
        - Z境界面との交点が発生する始点、終点
        - 水平方向精度：10
        - 垂直方向精度：10
        - CRS: 入力なし
      - 確認内容
        - 線分と境界面の交点の空間IDが取得できること
    """
    # 期待値の定義
    # 空間IDリスト定義
    param_assert = [
        '10/10/10/10/10',
        '10/10/10/10/11',
        '10/10/10/10/12',
        '10/10/10/10/13']

    # 試験用データの定義
    # 始点、終点の空間ID定義
    start_spatial_id = param_assert[0]
    end_spatial_id = param_assert[-1]

    # 始点、終点の定義 - 空間IDから座標を取得するライブラリを利用
    start_point = get_point_on_spatial_id(
            start_spatial_id, Point_Option.CENTER)[0]
    end_point = get_point_on_spatial_id(
            end_spatial_id, Point_Option.CENTER)[0]
    start_projection_point = get_point_on_spatial_id(
            start_spatial_id, Point_Option.CENTER, 3857)[0]
    end_projection_point = get_point_on_spatial_id(
            end_spatial_id, Point_Option.CENTER, 3857)[0]
    start_projection_point.lon = start_point.lon
    end_projection_point.lon = end_point.lon
    start_projection_point.lat = start_point.lat
    end_projection_point.lat = end_point.lat

    h_zoom = 10
    v_zoom = 10

    # 空間ID取得
    param_return = get_spatial_ids_on_line(
            start_projection_point, end_projection_point, h_zoom, v_zoom, GEOGRAPHIC_CRS)

    # 戻り値確認
    # 空間IDの要素数が期待値通りであること
    assert len(param_assert) == len(param_return)

    # 期待値通りの空間IDが返却されていること
    for param in param_return:
        assert True == (param in param_assert)


def test_get_spatial_ids_on_line_04():
    """指定範囲の空間ID変換(線分)取得、X, Y , Z境界と交差する線分
    + 試験詳細
      - 入力条件
        - X, Y, Z境界面との交点が発生する始点、終点
        - 水平方向精度：10
        - 垂直方向精度：10
        - CRS: 入力なし
      - 確認内容
        - 線分と境界面の交点の空間IDが取得できること
    """
    # 期待値の定義
    # 空間IDリスト定義
    param_assert = [
        '10/10/10/10/10',
        '10/11/10/10/10',
        '10/11/11/10/10',
        '10/11/11/10/11',
        '10/12/11/10/11']

    # 試験用データの定義
    # 始点、終点の定義
    start_point = SpatialPoint(lon=-176.30859375, lat=84.7222180279, alt=344064.0)
    end_point = SpatialPoint(lon=-175.60546875, lat=84.6897807505, alt=376832.0)
    start_projection_point, end_projection_point \
      = convert_point_list_to_projected_point_list(
        [start_point, end_point], PROJECTED_CRS, GEOGRAPHIC_CRS
      )

    h_zoom = 10
    v_zoom = 10

    # 空間ID取得
    param_return = get_spatial_ids_on_line(
            start_projection_point, end_projection_point, h_zoom, v_zoom, GEOGRAPHIC_CRS)

    # 戻り値確認
    # 空間IDの要素数が期待値通りであること
    assert len(param_assert) == len(param_return)

    # 期待値通りの空間IDが返却されていること
    for param in param_return:
        assert True == (param in param_assert)


def test_get_spatial_ids_on_line_05():
    """指定範囲の空間ID変換(線分)取得、水平方向精度下限
    + 試験詳細
      - 入力条件
        - 水平方向精度：1
        - 垂直方向精度：10
        - CRS: 入力なし
      - 確認内容
        - 線分と境界面の交点の空間IDが取得できること
    """
    # 期待値の定義
    # 空間IDリスト定義
    param_assert = [
        '1/1/1/10/10']

    # 試験用データの定義
    # 始点、終点の空間ID定義
    start_spatial_id = param_assert[0]
    end_spatial_id = param_assert[0]

    # 始点、終点の定義 - 空間IDから座標を取得するライブラリを利用
    start_point = get_point_on_spatial_id(
            start_spatial_id, Point_Option.CENTER)[0]
    end_point = get_point_on_spatial_id(
            end_spatial_id, Point_Option.VERTEX)[0]
    start_projection_point = get_point_on_spatial_id(
            start_spatial_id, Point_Option.CENTER, 3857)[0]
    end_projection_point = get_point_on_spatial_id(
            end_spatial_id, Point_Option.VERTEX, 3857)[0]
    start_projection_point.lon = start_point.lon
    end_projection_point.lon = end_point.lon
    start_projection_point.lat = start_point.lat
    end_projection_point.lat = end_point.lat
    h_zoom = 1
    v_zoom = 10

    # 空間ID取得
    param_return = get_spatial_ids_on_line(
            start_projection_point, end_projection_point, h_zoom, v_zoom, GEOGRAPHIC_CRS)

    # 戻り値確認
    # 空間IDの要素数が期待値通りであること
    assert len(param_assert) == len(param_return)

    # 期待値通りの空間IDが返却されていること
    for param in param_return:
        assert True == (param in param_assert)


# 2022/08/18 水平方向精度が大きい場合、空間IDから座標の変換において緯度方向が正常に取得できない問題が点群APIで発生中
# 以下のテストケースは一時的にコメントアウト、問題改修後に再度確認予定(#964#note-42 で確認依頼中)
#def test_get_spatial_ids_on_line_06():
#    """指定範囲の空間ID変換(線分)取得、水平方向精度上限
#    + 試験詳細
#      - 入力条件
#        - 水平方向精度：40
#        - 垂直方向精度：10
#        - CRS: 入力なし
#      - 確認内容
#        - 線分と境界面の交点の空間IDが取得できること
#    """
#    # 期待値の定義
#    # 空間IDリスト定義
#    param_assert = [
#        '40/10/10/10/10',
#        '40/11/10/10/10',
#        '40/12/10/10/10']
#
#    # 試験用データの定義
#    # 始点、終点の空間ID定義
#    start_spatial_id = param_assert[0]
#    end_spatial_id = param_assert[-1]
#
#    # 始点、終点の定義 - 空間IDから座標を取得するライブラリを利用
#    start_point = get_point_on_spatial_id(
#            start_spatial_id, Point_Option.CENTER)[0]
#    end_point = get_point_on_spatial_id(
#            end_spatial_id, Point_Option.CENTER)[0]
#    h_zoom = 40
#    v_zoom = 10
#
#    # 空間ID取得
#    param_return = get_spatial_ids_on_line(
#            start_point, end_point, h_zoom, v_zoom, GEOGRAPHIC_CRS)
#
#    # 戻り値確認
#    # 空間IDの要素数が期待値通りであること
#    assert len(param_assert) == len(param_return)
#
#    # 期待値通りの空間IDが返却されていること
#    for param in param_return:
#        assert True == (param in param_assert)


def test_get_spatial_ids_on_line_07():
    """指定範囲の空間ID変換(線分)取得、水平方向精度が閾値下限より小さい
    + 試験詳細
      - 入力条件
        - 水平方向精度：0
        - 垂直方向精度：1
      - 確認内容
        - 入力チェックエラーの例外が発生すること
    """
    # 試験用データの定義
    # 始点, 終点, 水平/垂直方向精度
    start_point = SpatialPoint(139.08785504, 36.10377479, 6.6)
    end_point = SpatialPoint(140.051685, 37.160433, 40000.5)
    h_zoom = -1
    v_zoom = 1

    # 期待値の定義
    # 例外取得
    exception_assert = SpatialIdError('INPUT_VALUE_ERROR')

    # 試験実施
    # 例外発生確認
    with pytest.raises(SpatialIdError) as e:
         # 空間ID取得
         get_spatial_ids_on_line(
                 start_point, end_point, h_zoom, v_zoom, GEOGRAPHIC_CRS)

    # 例外メッセージ確認
    assert str(exception_assert) == str(e.value)


def test_get_spatial_ids_on_line_08():
    """指定範囲の空間ID変換(線分)取得、水平方向精度が閾値上限より大きい
    + 試験詳細
      - 入力条件
        - 水平方向精度：41
        - 垂直方向精度：1
      - 確認内容
        - 入力チェックエラーの例外が発生すること
    """
    # 試験用データの定義
    # 始点, 終点, 水平/垂直方向精度
    start_point = SpatialPoint(139.08785504, 36.10377479, 6.6)
    end_point = SpatialPoint(140.051685, 37.160433, 40000.5)
    h_zoom = 41
    v_zoom = 1

    # 期待値の定義
    # 例外取得
    exception_assert = SpatialIdError('INPUT_VALUE_ERROR')

    # 試験実施
    # 例外発生確認
    with pytest.raises(SpatialIdError) as e:
         # 空間ID取得
         get_spatial_ids_on_line(
                 start_point, end_point, h_zoom, v_zoom, GEOGRAPHIC_CRS)

    # 例外メッセージ確認
    assert str(exception_assert) == str(e.value)


def test_get_spatial_ids_on_line_09():
    """指定範囲の空間ID変換(線分)取得、垂直方向精度下限
    + 試験詳細
      - 入力条件
        - 水平方向精度：10
        - 垂直方向精度：1
        - CRS: 入力なし
      - 確認内容
        - 線分と境界面の交点の空間IDが取得できること
    """
    # 期待値の定義
    # 空間IDリスト定義
    param_assert = [
        '10/10/10/1/10',
        '10/10/10/1/11',
        '10/10/10/1/12']

    # 試験用データの定義
    # 始点、終点の空間ID定義
    start_spatial_id = param_assert[0]
    end_spatial_id = param_assert[-1]

    # 始点、終点の定義 - 空間IDから座標を取得するライブラリを利用
    start_point = get_point_on_spatial_id(
            start_spatial_id, Point_Option.CENTER)[0]
    end_point = get_point_on_spatial_id(
            end_spatial_id, Point_Option.CENTER)[0]
    start_projection_point = get_point_on_spatial_id(
            start_spatial_id, Point_Option.CENTER, 3857)[0]
    end_projection_point = get_point_on_spatial_id(
            end_spatial_id, Point_Option.CENTER, 3857)[0]
    start_projection_point.lon = start_point.lon
    end_projection_point.lon = end_point.lon
    start_projection_point.lat = start_point.lat
    end_projection_point.lat = end_point.lat
    h_zoom = 10
    v_zoom = 1

    # 空間ID取得
    param_return = get_spatial_ids_on_line(
            start_projection_point, end_projection_point, h_zoom, v_zoom, GEOGRAPHIC_CRS)

    # 戻り値確認
    # 空間IDの要素数が期待値通りであること
    assert len(param_assert) == len(param_return)

    # 期待値通りの空間IDが返却されていること
    for param in param_return:
        assert True == (param in param_assert)


def test_get_spatial_ids_on_line_10():
    """指定範囲の空間ID変換(線分)取得、垂直方向精度上限
    + 試験詳細
      - 入力条件
        - 水平方向精度：10
        - 垂直方向精度：40
        - CRS: 入力なし
      - 確認内容
        - 線分と境界面の交点の空間IDが取得できること
    """
    # 期待値の定義
    # 空間IDリスト定義
    param_assert = [
        '10/10/10/35/10',
        '10/10/10/35/11',
        '10/10/10/35/12']

    # 試験用データの定義
    # 始点、終点の空間ID定義
    start_spatial_id = param_assert[0]
    end_spatial_id = param_assert[-1]

    # 始点、終点の定義 - 空間IDから座標を取得するライブラリを利用
    start_point = get_point_on_spatial_id(
            start_spatial_id, Point_Option.CENTER)[0]
    end_point = get_point_on_spatial_id(
            end_spatial_id, Point_Option.CENTER)[0]
    start_projection_point = get_point_on_spatial_id(
            start_spatial_id, Point_Option.CENTER, 3857)[0]
    end_projection_point = get_point_on_spatial_id(
            end_spatial_id, Point_Option.CENTER, 3857)[0]
    start_projection_point.lon = start_point.lon
    end_projection_point.lon = end_point.lon
    start_projection_point.lat = start_point.lat
    end_projection_point.lat = end_point.lat

    h_zoom = 10
    v_zoom = 35

    # 空間ID取得
    param_return = get_spatial_ids_on_line(
            start_projection_point, end_projection_point, h_zoom, v_zoom, GEOGRAPHIC_CRS)

    # 戻り値確認
    # 空間IDの要素数が期待値通りであること
    assert len(param_assert) == len(param_return)

    # 期待値通りの空間IDが返却されていること
    for param in param_return:
        assert True == (param in param_assert)


def test_get_spatial_ids_on_line_11():
    """指定範囲の空間ID変換(線分)取得、垂直方向精度が閾値下限より小さい
    + 試験詳細
      - 入力条件
        - 水平方向精度：1
        - 垂直方向精度：0
      - 確認内容
        - 入力チェックエラーの例外が発生すること
    """
    # 試験用データの定義
    # 始点, 終点, 水平/垂直方向精度
    start_point = SpatialPoint(139.08785504, 36.10377479, 6.6)
    end_point = SpatialPoint(140.051685, 37.160433, 40000.5)
    h_zoom = 1
    v_zoom = -1

    # 期待値の定義
    # 例外取得
    exception_assert = SpatialIdError('INPUT_VALUE_ERROR')

    # 試験実施
    # 例外発生確認
    with pytest.raises(SpatialIdError) as e:
         # 空間ID取得
         get_spatial_ids_on_line(
                 start_point, end_point, h_zoom, v_zoom, GEOGRAPHIC_CRS)

    # 例外メッセージ確認
    assert str(exception_assert) == str(e.value)


def test_get_spatial_ids_on_line_12():
    """指定範囲の空間ID変換(線分)取得、垂直方向精度が閾値上限より大きい
    + 試験詳細
      - 入力条件
        - 水平方向精度：1
        - 垂直方向精度：41
      - 確認内容
        - 入力チェックエラーの例外が発生すること
    """
    # 試験用データの定義
    # 始点, 終点, 水平/垂直方向精度
    start_point = SpatialPoint(139.08785504, 36.10377479, 6.6)
    end_point = SpatialPoint(140.051685, 37.160433, 40000.5)
    h_zoom = 1
    v_zoom = 41

    # 期待値の定義
    # 例外取得
    exception_assert = SpatialIdError('INPUT_VALUE_ERROR')

    # 試験実施
    # 例外発生確認
    with pytest.raises(SpatialIdError) as e:
         # 空間ID取得
         get_spatial_ids_on_line(
                 start_point, end_point, h_zoom, v_zoom, GEOGRAPHIC_CRS)

    # 例外メッセージ確認
    assert str(exception_assert) == str(e.value)


def test_get_spatial_ids_on_line_13():
    """指定範囲の空間ID変換(線分)取得、CRSに地理座標系（4612）を指定
    + 試験詳細
      - 入力条件
        - CRS: 地理座標系
      - 確認内容
        - 線分と境界面の交点の空間IDが取得できること
    """
    # 期待値の定義
    # 空間IDリスト定義
    param_assert = [
        '10/10/10/10/10',
        '10/11/10/10/10',
        '10/12/10/10/10',
        '10/13/10/10/10']

    # 試験用データの定義
    # 座標系の定義 (JGD2000)
    input_crs = 4612

    # 始点、終点の空間ID定義
    start_spatial_id = param_assert[0]
    end_spatial_id = param_assert[-1]

    # 始点、終点の定義 - 空間IDから座標を取得するライブラリを利用
    start_point = get_point_on_spatial_id(
            start_spatial_id, Point_Option.CENTER, input_crs)[0]
    end_point = get_point_on_spatial_id(
            end_spatial_id, Point_Option.CENTER, input_crs)[0]
    start_projection_point = get_point_on_spatial_id(
            start_spatial_id, Point_Option.CENTER, 3857)[0]
    end_projection_point = get_point_on_spatial_id(
            end_spatial_id, Point_Option.CENTER, 3857)[0]
    start_projection_point.lon = start_point.lon
    end_projection_point.lon = end_point.lon
    start_projection_point.lat = start_point.lat
    end_projection_point.lat = end_point.lat
    h_zoom = 10
    v_zoom = 10

    # 空間ID取得
    param_return = get_spatial_ids_on_line(
            start_projection_point, end_projection_point, h_zoom, v_zoom, input_crs)

    # 戻り値確認
    # 空間IDの要素数が期待値通りであること
    assert len(param_assert) == len(param_return)

    # 期待値通りの空間IDが返却されていること
    for param in param_return:
        assert True == (param in param_assert)

# get_spatial_ids_on_line内部の座標変換処理は不要になったため、削除
#def test_get_spatial_ids_on_line_14():
#    """指定範囲の空間ID変換(線分)取得、CRSに投影座標系を指定
#    + 試験詳細
#      - 入力条件
#        - CRS: 投影座標系
#      - 確認内容
#        - 座標変換失敗でエラーが発生すること
#    """
#    # 試験用データの定義
#    # 空間IDリスト定義
#    param_assert = [
#        '10/10/10/10/10',
#        '10/11/10/10/10',
#        '10/12/10/10/10',
#        '10/13/10/10/10']

#    # 座標系の定義 (投影座標系)
#    input_crs = PROJECTED_CRS

#    # 始点、終点の空間ID定義
#    start_spatial_id = param_assert[0]
#    end_spatial_id = param_assert[-1]

#    # 始点、終点の定義 - 空間IDから座標を取得するライブラリを利用
#    start_point = get_point_on_spatial_id(
#            start_spatial_id, Point_Option.CENTER)[0]
#    end_point = get_point_on_spatial_id(
#            end_spatial_id, Point_Option.CENTER)[0]
#    start_projection_point = get_point_on_spatial_id(
#            start_spatial_id, Point_Option.CENTER, 3857)[0]
#    end_projection_point = get_point_on_spatial_id(
#            end_spatial_id, Point_Option.CENTER, 3857)[0]
#    start_projection_point.lon = start_point.lon
#    end_projection_point.lon = end_point.lon
#    start_projection_point.lat = start_point.lat
#    end_projection_point.lat = end_point.lat
#    h_zoom = 10
#    v_zoom = 10

#    # 期待値の定義
#    # 例外取得
#    exception_assert = SpatialIdError('VALUE_CONVERT_ERROR')

#    # 試験実施
#    # 例外発生確認
#    with pytest.raises(SpatialIdError) as e:
#        # 空間ID取得
#        get_spatial_ids_on_line(
#            start_projection_point, end_projection_point, h_zoom, v_zoom, input_crs)

#    # 例外メッセージ確認
#    assert str(exception_assert) == str(e.value)

# get_spatial_ids_on_line内部の座標変換処理は不要になったため、削除
#def test_get_spatial_ids_on_line_15():
#    """指定範囲の空間ID変換(線分)取得、CRSに存在しない座標系を指定
#    + 試験詳細
#      - 入力条件
#        - CRS: 存在しない座標系
#      - 確認内容
#        - 座標変換失敗でエラーが発生すること
#    """
#    # 試験用データの定義
#    # 空間IDリスト定義
#    param_assert = [
#        '10/10/10/10/10',
#        '10/11/10/10/10',
#        '10/12/10/10/10',
#        '10/13/10/10/10']

#    # 座標系の定義 (投影座標系)
#    input_crs = 1

#    # 始点、終点の空間ID定義
#    start_spatial_id = param_assert[0]
#    end_spatial_id = param_assert[-1]

#    # 始点、終点の定義 - 空間IDから座標を取得するライブラリを利用
#    start_point = get_point_on_spatial_id(
#            start_spatial_id, Point_Option.CENTER)[0]
#    end_point = get_point_on_spatial_id(
#            end_spatial_id, Point_Option.CENTER)[0]
#    start_projection_point = get_point_on_spatial_id(
#            start_spatial_id, Point_Option.CENTER, 3857)[0]
#    end_projection_point = get_point_on_spatial_id(
#            end_spatial_id, Point_Option.CENTER, 3857)[0]
#    start_projection_point.lon = start_point.lon
#    end_projection_point.lon = end_point.lon
#    start_projection_point.lat = start_point.lat
#    end_projection_point.lat = end_point.lat
#    h_zoom = 10
#    v_zoom = 10

#    # 期待値の定義
#    # 例外取得
#    exception_assert = SpatialIdError('VALUE_CONVERT_ERROR')

#    # 試験実施
#    # 例外発生確認
#    with pytest.raises(SpatialIdError) as e:
#        # 空間ID取得
#        get_spatial_ids_on_line(
#            start_projection_point, end_projection_point, h_zoom, v_zoom, input_crs)

#    # 例外メッセージ確認
#    assert str(exception_assert) == str(e.value)


def test_get_spatial_ids_on_line_16():
    """指定範囲の空間ID変換(線分)取得、始点終点が同じ座標
    + 試験詳細
      - 入力条件
        - 始点終点が同じ座標
      - 確認内容
        - 始点終点の空間IDが取得できること
    """
    # 期待値の定義
    # 空間IDリスト定義
    param_assert = ['10/10/10/10/10']

    # 試験用データの定義
    # 始点、終点の空間ID定義
    start_spatial_id = param_assert[0]
    end_spatial_id = param_assert[0]

    # 始点、終点の定義 - 空間IDから座標を取得するライブラリを利用
    start_point = get_point_on_spatial_id(
            start_spatial_id, Point_Option.CENTER, GEOGRAPHIC_CRS)[0]
    end_point = get_point_on_spatial_id(
            end_spatial_id, Point_Option.CENTER, GEOGRAPHIC_CRS)[0]
    h_zoom = 10
    v_zoom = 10

    # 空間ID取得
    param_return = get_spatial_ids_on_line(
            start_point, end_point, h_zoom, v_zoom, GEOGRAPHIC_CRS)

    # 戻り値確認
    # 空間IDの要素数が期待値通りであること
    assert len(param_assert) == len(param_return)

    # 期待値通りの空間IDが返却されていること
    for param in param_return:
        assert True == (param in param_assert)


def test__get_middle_points_01():
    """始点、終点、交点間の中点を取得(全ての点の間に中点が存在)
    """
    # 準備
    start_point = Point([0.0, 0.0, 0.0])
    end_point = Point([5.0, 5.0, 5.0])
    cross_point_list = [
            Point([1.0, 1.0, 1.0]),
            Point([2.0, 2.0, 2.0]),
            Point([3.0, 3.0, 3.0]),
            Point([4.0, 4.0, 4.0])]
    exclude_point_set = set()
    for point in cross_point_list:
        exclude_point_set.add("{0}_{1}_{2}".format(point[0], point[1], point[2]))

    # 期待値
    param_assert = [
            Point([0.5, 0.5, 0.5]),
            Point([1.5, 1.5, 1.5]),
            Point([2.5, 2.5, 2.5]),
            Point([3.5, 3.5, 3.5]),
            Point([4.5, 4.5, 4.5])]

    # 実行
    param_return = _get_middle_points(start_point, end_point, cross_point_list, exclude_point_set)

    # 結果確認
    assert len(param_assert) == len(param_return)
    for cnt in range(len(param_assert)):
        assert (param_assert[cnt] == param_return[cnt]).all()


def test__get_middle_points_02():
    """始点、終点、交点間の中点を取得(始点、終点が交点と同一)
    """
    # 準備
    start_point = Point([1.0, 1.0, 1.0])
    end_point = Point([4.0, 4.0, 4.0])
    cross_point_list = [
            Point([1.0, 1.0, 1.0]),
            Point([2.0, 2.0, 2.0]),
            Point([3.0, 3.0, 3.0]),
            Point([4.0, 4.0, 4.0])]
    exclude_point_set = set()
    for point in cross_point_list:
        exclude_point_set.add("{0}_{1}_{2}".format(point[0], point[1], point[2]))

    # 期待値
    param_assert = [
            Point([1.5, 1.5, 1.5]),
            Point([2.5, 2.5, 2.5]),
            Point([3.5, 3.5, 3.5])]

    # 実行
    param_return = _get_middle_points(start_point, end_point, cross_point_list, exclude_point_set)

    # 結果確認
    assert len(param_assert) == len(param_return)
    for cnt in range(len(param_assert)):
        assert (param_assert[cnt] == param_return[cnt]).all()


def test__get_middle_points_03():
    """始点、終点、交点間の中点を取得(X軸と並行)
    """
    # 準備
    start_point = Point([0.0, 0.0, 0.0])
    end_point = Point([5.0, 0.0, 0.0])
    cross_point_list = [
            Point([1.0, 0.0, 0.0]),
            Point([2.0, 0.0, 0.0]),
            Point([3.0, 0.0, 0.0]),
            Point([4.0, 0.0, 0.0])]
    exclude_point_set = set()
    for point in cross_point_list:
        exclude_point_set.add("{0}_{1}_{2}".format(point[0], point[1], point[2]))

    # 期待値
    param_assert = [
            Point([0.5, 0.0, 0.0]),
            Point([1.5, 0.0, 0.0]),
            Point([2.5, 0.0, 0.0]),
            Point([3.5, 0.0, 0.0]),
            Point([4.5, 0.0, 0.0])]

    # 実行
    param_return = _get_middle_points(start_point, end_point, cross_point_list, exclude_point_set)

    # 結果確認
    assert len(param_assert) == len(param_return)
    for cnt in range(len(param_assert)):
        assert (param_assert[cnt] == param_return[cnt]).all()

def test__get_middle_points_04():
    """始点、終点、交点間の中点を取得(Y軸と並行)
    """
    # 準備
    start_point = Point([0.0, 0.0, 0.0])
    end_point = Point([0.0, 5.0, 0.0])
    cross_point_list = [
            Point([0.0, 1.0, 0.0]),
            Point([0.0, 2.0, 0.0]),
            Point([0.0, 3.0, 0.0]),
            Point([0.0, 4.0, 0.0])]
    exclude_point_set = set()
    for point in cross_point_list:
        exclude_point_set.add("{0}_{1}_{2}".format(point[0], point[1], point[2]))

    # 期待値
    param_assert = [
            Point([0.0, 0.5, 0.0]),
            Point([0.0, 1.5, 0.0]),
            Point([0.0, 2.5, 0.0]),
            Point([0.0, 3.5, 0.0]),
            Point([0.0, 4.5, 0.0])]

    # 実行
    param_return = _get_middle_points(start_point, end_point, cross_point_list, exclude_point_set)

    # 結果確認
    assert len(param_assert) == len(param_return)
    for cnt in range(len(param_assert)):
        assert (param_assert[cnt] == param_return[cnt]).all()


def test__get_middle_points_05():
    """始点、終点、交点間の中点を取得(Z軸と並行)
    """
    # 準備
    start_point = Point([0.0, 0.0, 0.0])
    end_point = Point([0.0, 0.0, 5.0])
    cross_point_list = [
            Point([0.0, 0.0, 1.0]),
            Point([0.0, 0.0, 2.0]),
            Point([0.0, 0.0, 3.0]),
            Point([0.0, 0.0, 4.0])]
    exclude_point_set = set()
    for point in cross_point_list:
        exclude_point_set.add("{0}_{1}_{2}".format(point[0], point[1], point[2]))

    # 期待値
    param_assert = [
            Point([0.0, 0.0, 0.5]),
            Point([0.0, 0.0, 1.5]),
            Point([0.0, 0.0, 2.5]),
            Point([0.0, 0.0, 3.5]),
            Point([0.0, 0.0, 4.5])]

    # 実行
    param_return = _get_middle_points(start_point, end_point, cross_point_list, exclude_point_set)

    # 結果確認
    assert len(param_assert) == len(param_return)
    for cnt in range(len(param_assert)):
        assert (param_assert[cnt] == param_return[cnt]).all()


def test__get_middle_points_06():
    """始点、終点、交点間の中点を取得(負の方向)
    """
    # 準備
    start_point = Point([5.0, 5.0, 5.0])
    end_point = Point([0.0, 0.0, 0.0])
    cross_point_list = [
            Point([1.0, 1.0, 1.0]),
            Point([2.0, 2.0, 2.0]),
            Point([3.0, 3.0, 3.0]),
            Point([4.0, 4.0, 4.0])]
    exclude_point_set = set()
    for point in cross_point_list:
        exclude_point_set.add("{0}_{1}_{2}".format(point[0], point[1], point[2]))

    # 期待値
    param_assert = [
            Point([0.5, 0.5, 0.5]),
            Point([1.5, 1.5, 1.5]),
            Point([2.5, 2.5, 2.5]),
            Point([3.5, 3.5, 3.5]),
            Point([4.5, 4.5, 4.5])]

    # 実行
    param_return = _get_middle_points(start_point, end_point, cross_point_list, exclude_point_set)

    # 結果確認
    assert len(param_assert) == len(param_return)
    for cnt in range(len(param_assert)):
        assert (param_assert[cnt] == param_return[cnt]).all()

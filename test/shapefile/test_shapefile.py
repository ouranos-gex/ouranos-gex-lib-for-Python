#!/bin/env python3
# -*- coding: utf-8 -*-

import os
import pytest
import zipfile
import shapefile as sf

from shapefile import TRIANGLE_STRIP, TRIANGLE_FAN, Shape, ShapeRecord

from SpatialId.common.exception import SpatialIdError
from SpatialId.common.object.point import Point as SpatialPoint
from SpatialId.common.object.point import Projected_Point
from SpatialId.common.object.point import Triangle as SpatialTriangle
from SpatialId.common.object.point import Projected_Triangle as SpatialProjectedTriangle
from SpatialId.shape.point import convert_projected_point_list_to_point_list
from SpatialId.shape.polygons import get_spatial_ids_on_polygons, f_get_spatial_ids_on_polygons
from SpatialId.io.shapefile import (
        read_shapefile,
        read_shapefile_bulk,
        read_shapefile_bulk_with_aggregation,
        f_read_shapefile,
        f_read_shapefile_bulk,
        f_read_shapefile_bulk_with_aggregation,
        _get_fields_data,
        _get_shapefile_spatial_id_and_record,
        _get_multipatch_data,
        _combine_x_y_z_point,
        _shaping_multipatch_parts,
        _shaping_triangle_strip,
        _shaping_triangle_fan,
        _get_triangle_objects,
        _validate_args_type)

# テスト用shapefile格納ディレクトリパス
SHAPEFILE_DIR = './test/data/SpatialId/io/test_shapefile/'


@pytest.fixture
def geographic_shape_point_data_01():
    """
    shapefile作成用のパーツデータを返却する。
    TRIANGLE_FANのデータ2つ分のデータが格納されている。
    全てのパーツを1つのレコードに格納したshapefileを作成した場合、
    そのレコードで作成されるモデルは閉塞状態となる。
    設定されている座標は地理座標系

    return: shapefile作成用のパーツデータ
    rtypr: list[list[list[tuple]], list[int]]
    """
    # Pointの座標を格納したリストを定義
    fan_point_01 = (139.05, 45.06, 150.5)
    fan_point_02 = (138.05, 45.06, 200.5)
    fan_point_03 = (141.05, 45.06, 200.5)
    fan_point_04 = (141.05, 50.06, 200.5)
    fan_point_05 = (138.05, 50.06, 200.5)
    fan_point_06 = (138.05, 45.06, 200.5)
    fan_point_07 = (139.05, 45.06, 100.5)

    # shapefile作成用の座標配列
    fan_point_list_01 = [
            fan_point_01,
            fan_point_02,
            fan_point_03,
            fan_point_04,
            fan_point_05,
            fan_point_06,
            ]
    fan_point_list_02 = [
            fan_point_07,
            fan_point_02,
            fan_point_03,
            fan_point_04,
            fan_point_05,
            fan_point_06,
            ]

    # パーツの座標群、タイプを戻り値として返却
    parts_list = [fan_point_list_01, fan_point_list_02]
    parts_types = [1, 1]

    return [parts_list, parts_types]


@pytest.fixture
def geographic_shape_point_data_02():
    """
    shapefile作成用のパーツデータを返却する。
    TRIANGLE_FANのデータ2つ分のデータが格納されている。
    全てのパーツを1つのレコードに格納したshapefileを作成した場合、
    そのレコードで作成されるモデルは閉塞状態となる。
    設定されている座標は地理座標系

    return: shapefile作成用のパーツデータ
    rtypr: list[list[list[tuple]], list[int]]
    """
    # Pointの座標を格納したリストを定義
    fan_point_01 = (149.05, 35.06, 150.5)
    fan_point_02 = (148.05, 35.06, 200.5)
    fan_point_03 = (151.05, 35.06, 200.5)
    fan_point_04 = (151.05, 40.06, 200.5)
    fan_point_05 = (148.05, 40.06, 200.5)
    fan_point_06 = (148.05, 35.06, 200.5)
    fan_point_07 = (149.05, 35.06, 100.5)

    # shapefile作成用の座標配列
    fan_point_list_01 = [
            fan_point_01,
            fan_point_02,
            fan_point_03,
            fan_point_04,
            fan_point_05,
            fan_point_06,
            ]
    fan_point_list_02 = [
            fan_point_07,
            fan_point_02,
            fan_point_03,
            fan_point_04,
            fan_point_05,
            fan_point_06,
            ]

    # パーツの座標群、タイプを戻り値として返却
    parts_list = [fan_point_list_01, fan_point_list_02]
    parts_types = [1, 1]

    return [parts_list, parts_types]


@pytest.fixture
def geographic_shape_point_data_03():
    """
    shapefile作成用のパーツデータを返却する。
    TRIANGLE_FANのデータ2つ分のデータが格納されている。
    全てのパーツを1つのレコードに格納したshapefileを作成した場合、
    そのレコードで作成されるモデルは閉塞状態となる。
    設定されている座標は地理座標系(精度レベル40向け)

    return: shapefile作成用のパーツデータ
    rtypr: list[list[list[tuple]], list[int]]
    """
    # Pointの座標を格納したリストを定義
    fan_point_01 = (139.05, 45.06, 150.5)
    fan_point_02 = (139.0499999999, 45.06, 150.5000000001)
    fan_point_03 = (139.0500000001, 45.06, 150.5000000001)
    fan_point_04 = (139.0500000001, 45.0600000001, 150.5000000001)
    fan_point_05 = (139.0499999999, 45.0600000001, 150.5000000001)
    fan_point_06 = (139.0499999999, 45.06, 150.5000000001)
    fan_point_07 = (139.05, 45.06, 150.4999999999)

    # shapefile作成用の座標配列
    fan_point_list_01 = [
            fan_point_01,
            fan_point_02,
            fan_point_03,
            fan_point_04,
            fan_point_05,
            fan_point_06,
            ]
    fan_point_list_02 = [
            fan_point_07,
            fan_point_02,
            fan_point_03,
            fan_point_04,
            fan_point_05,
            fan_point_06,
            ]

    # パーツの座標群、タイプを戻り値として返却
    parts_list = [fan_point_list_01, fan_point_list_02]
    parts_types = [1, 1]

    return [parts_list, parts_types]


@pytest.fixture
def geographic_shape_point_data_04():
    """
    shapefile作成用のパーツデータを返却する。
    TRIANGLE_FANのデータ2つ分のデータが格納されている。
    閉塞していないモデルの確認用データ。
    設定されている座標は地理座標系

    return: shapefile作成用のパーツデータ
    rtypr: list[list[list[tuple]], list[int]]
    """
    # Pointの座標を格納したリストを定義
    fan_point_01 = (139.05, 45.06, 150.5)
    fan_point_02 = (138.05, 45.06, 200.5)
    fan_point_03 = (141.05, 45.06, 200.5)
    fan_point_04 = (141.05, 50.06, 200.5)
    fan_point_05 = (138.05, 50.06, 200.5)
    fan_point_06 = (138.05, 45.06, 200.5)
    fan_point_07 = (139.05, 45.06, 100.5)

    # shapefile作成用の座標配列
    fan_point_list_01 = [
            fan_point_01,
            fan_point_02,
            fan_point_03,
            fan_point_04,
            fan_point_05,
            fan_point_06,
            ]
    fan_point_list_02 = [
            fan_point_07,
            fan_point_02,
            fan_point_03,
            fan_point_04,
            fan_point_05,
            ]

    # パーツの座標群、タイプを戻り値として返却
    parts_list = [fan_point_list_01, fan_point_list_02]
    parts_types = [1, 1]

    return [parts_list, parts_types]


@pytest.fixture
def projected_shape_point_data_01():
    """
    shapefile作成用のパーツデータを返却する。
    TRIANGLE_STRIPのデータが2つ、TRIANGLE_FANのデータ1つ分のデータが格納されている。
    全てのパーツを1つのレコードに格納したshapefileを作成した場合、
    そのレコードで作成されるモデルは閉塞状態となる。
    設定されている座標は投影座標系

    return: shapefile作成用のパーツデータ
    rtypr: list[list[list[tuple]], list[int]]
    """
    # Pointの座標を格納したリストを定義
    strip_point_01 = (0.0, 0.0, 0.0)
    strip_point_02 = (0.0, 0.0, 30000.0)
    strip_point_03 = (10005.0, 0.0, 0.0)
    strip_point_04 = (10005.0, 0.0, 30000.0)
    strip_point_05 = (10005.0, 10005.0, 0.0)
    strip_point_06 = (10005.0, 10005.0, 30000.0)
    strip_point_07 = (0.0, 10005.0, 0.0)
    strip_point_08 = (0.0, 10005.0, 30000.0)
    strip_point_09 = (0.0, 0.0, 0.0)
    strip_point_10 = (0.0, 0.0, 30000.0)
    fan_point_01 = (5002.5, 5002.5, 500000.0)
    fan_point_02 = (0.0, 0.0, 30000.0)
    fan_point_03 = (10005.0, 0.0, 30000.0)
    fan_point_04 = (10005.0, 10005.0, 30000.0)
    fan_point_05 = (0.0, 10005.0, 30000.0)
    fan_point_06 = (0.0, 0.0, 30000.0)

    # shapefile作成用の座標配列
    strip_point_list_01 = [
            strip_point_01,
            strip_point_02,
            strip_point_03,
            strip_point_04,
            strip_point_05,
            strip_point_06,
            strip_point_07,
            strip_point_08,
            strip_point_09,
            strip_point_10
            ]
    strip_point_list_02 = [
            strip_point_01,
            strip_point_03,
            strip_point_07,
            strip_point_05
            ]
    fan_point_list = [
            fan_point_01,
            fan_point_02,
            fan_point_03,
            fan_point_04,
            fan_point_05,
            fan_point_06,
            ]

    # パーツの座標群、タイプを戻り値として返却
    parts_list = [strip_point_list_01, strip_point_list_02, fan_point_list]
    parts_types = [0, 0, 1]

    return [parts_list, parts_types]


def test__validate_args_type_01():
    """入力引数の型チェック、チェック結果OK
    + 試験詳細
      - 入力条件
        - filepathにstr型の値を入力
        - encodingにstr型の値を入力
        - h_zoomにint型の値を入力
        - v_zoomにint型の値を入力
        - crsにint型の値を入力
      - 確認内容
        - 例外が発生しないこと
    """
    # 試験実施
    # 例外が発生しないことを確認
    param_return = _validate_args_type("test/dir", "utf-8", 1, 1, 4326, True)
    assert None is param_return


def test__validate_args_type_02():
    """入力引数の型チェック、filepath入力チェックNG
    + 試験詳細
      - 入力条件
        - filepathにint型の値を入力
      - 確認内容
        - 入力値チェックエラーの例外が発生すること
    """
    # 期待値定義
    # 例外を取得
    exception_assert = SpatialIdError('INPUT_VALUE_ERROR')

    # 試験実施
    # 例外発生を確認
    with pytest.raises(SpatialIdError) as e:
        _validate_args_type(100, "utf-8", 1, 1, 4326)

    assert str(exception_assert) == str(e.value)


def test__validate_args_type_03():
    """入力引数の型チェック、encoding入力チェックNG
    + 試験詳細
      - 入力条件
        - encodingにint型の値を入力
      - 確認内容
        - 入力値チェックエラーの例外が発生すること
    """
    # 期待値定義
    # 例外を取得
    exception_assert = SpatialIdError('INPUT_VALUE_ERROR')

    # 試験実施
    # 例外発生を確認
    with pytest.raises(SpatialIdError) as e:
        _validate_args_type("test/dir", 100, 1, 1, 4326)

    assert str(exception_assert) == str(e.value)


def test__validate_args_type_04():
    """入力引数の型チェック、h_zoom入力チェックNG
    + 試験詳細
      - 入力条件
        - h_zoomにstr型の値を入力
      - 確認内容
        - 入力値チェックエラーの例外が発生すること
    """
    # 期待値定義
    # 例外を取得
    exception_assert = SpatialIdError('INPUT_VALUE_ERROR')

    # 試験実施
    # 例外発生を確認
    with pytest.raises(SpatialIdError) as e:
        _validate_args_type("test/dir", "utf-8", "1", 1, 4326)

    assert str(exception_assert) == str(e.value)


def test__validate_args_type_05():
    """入力引数の型チェック、v_zoom入力チェックNG
    + 試験詳細
      - 入力条件
        - v_zoomにstr型の値を入力
      - 確認内容
        - 入力値チェックエラーの例外が発生すること
    """
    # 期待値定義
    # 例外を取得
    exception_assert = SpatialIdError('INPUT_VALUE_ERROR')

    # 試験実施
    # 例外発生を確認
    with pytest.raises(SpatialIdError) as e:
        _validate_args_type("test/dir", "utf-8", 1, "1", 4326)

    assert str(exception_assert) == str(e.value)


def test__validate_args_type_06():
    """入力引数の型チェック、crs入力チェックNG
    + 試験詳細
      - 入力条件
        - crsにstr型の値を入力
      - 確認内容
        - 入力値チェックエラーの例外が発生すること
    """
    # 期待値定義
    # 例外を取得
    exception_assert = SpatialIdError('INPUT_VALUE_ERROR')

    # 試験実施
    # 例外発生を確認
    with pytest.raises(SpatialIdError) as e:
        _validate_args_type("test/dir", "utf-8", 1, 1, "4326")

    assert str(exception_assert) == str(e.value)


def test__validate_args_type_07():
    """入力引数の型チェック、needs_closed_checking入力チェックNG
    + 試験詳細
      - 入力条件
        - needs_closed_checkingにstr型の値を入力
      - 確認内容
        - 入力値チェックエラーの例外が発生すること
    """
    # 期待値定義
    # 例外を取得
    exception_assert = SpatialIdError('INPUT_VALUE_ERROR')

    # 試験実施
    # 例外発生を確認
    with pytest.raises(SpatialIdError) as e:
        _validate_args_type("test/dir", "utf-8", 1, 1, 4326, "True")

    assert str(exception_assert) == str(e.value)


def test__get_triangle_objects_01():
    """Triangleオブジェクトが取得できること
    """
    # 試験用データの定義
    # shapefileに格納されたデータの座標系
    shapefile_crs = 4326

    # 期待値取得用0.0, 0.0, 変数
    spatial_point_01 = Projected_Point(0.0, 0.0, lon=139.051685, lat=45.060433, alt=150.5)
    spatial_point_02 = Projected_Point(0.0, 0.0, lon=140.051685, lat=46.060433, alt=151.5)
    spatial_point_03 = Projected_Point(0.0, 0.0, lon=141.051685, lat=47.060433, alt=152.5)
    spatial_point_04 = Projected_Point(0.0, 0.0, lon=139.051685, lat=45.060433, alt=151.5)
    spatial_point_05 = Projected_Point(0.0, 0.0, lon=140.051685, lat=46.060433, alt=152.5)
    spatial_point_06 = Projected_Point(0.0, 0.0, lon=141.051685, lat=47.060433, alt=153.5)

    # Triangleの座標を格納したリストを定義
    point_02 = Projected_Point(0.0, 0.0, lon=140.051685, lat=46.060433, alt=151.5)
    point_01 = Projected_Point(0.0, 0.0, lon=139.051685, lat=45.060433, alt=150.5)
    point_03 = Projected_Point(0.0, 0.0, lon=141.051685, lat=47.060433, alt=152.5)
    tri_parts_list_01 = [point_01, point_02, point_03]

    point_04 = Projected_Point(0.0, 0.0, lon=139.051685, lat=45.060433, alt=151.5)
    point_05 = Projected_Point(0.0, 0.0, lon=140.051685, lat=46.060433, alt=152.5)
    point_06 = Projected_Point(0.0, 0.0, lon=141.051685, lat=47.060433, alt=153.5)
    tri_parts_list_02 = [point_04, point_05, point_06]
    shapefile_tri_list = [tri_parts_list_01, tri_parts_list_02]

    # 期待値の取得
    # Triangleオブジェクト取得
    param_assert_01 = SpatialProjectedTriangle(
            spatial_point_01,
            spatial_point_02,
            spatial_point_03)
    param_assert_02 = SpatialProjectedTriangle(
            spatial_point_04,
            spatial_point_05,
            spatial_point_06)
    param_assert_list = [param_assert_01, param_assert_02]

    # 試験実施
    # 戻り値を取得
    param_return = _get_triangle_objects(shapefile_tri_list)

    # 戻り値確認
    assert 2 == len(param_return)
    for cnt in range(len(param_return)):
        assert param_assert_list[cnt].p1.lon == param_return[cnt].p1.lon
        assert param_assert_list[cnt].p1.lat == param_return[cnt].p1.lat
        assert param_assert_list[cnt].p1.alt == param_return[cnt].p1.alt
        assert param_assert_list[cnt].p2.lon == param_return[cnt].p2.lon
        assert param_assert_list[cnt].p2.lat == param_return[cnt].p2.lat
        assert param_assert_list[cnt].p2.alt == param_return[cnt].p2.alt
        assert param_assert_list[cnt].p3.lon == param_return[cnt].p3.lon
        assert param_assert_list[cnt].p3.lat == param_return[cnt].p3.lat
        assert param_assert_list[cnt].p3.alt == param_return[cnt].p3.alt


def test__get_triangle_objects_02():
    """引数の座標リストが空の場合
    + 試験詳細
      - 入力条件
        - 空の座標リストを渡す
      - 確認内容
        - 空のリストが返却されること
    """
    # 試験用データの定義
    # shapefileに格納されたデータの座標系
    shapefile_crs = 4326

    # Triangleの座標を格納したリストを定義
    shapefile_tri_list = []

    # 期待値の取得
    # 空のリストを定義
    param_assert = []

    # 試験実施
    # 戻り値を取得
    param_return = _get_triangle_objects(shapefile_tri_list)

    # 戻り値を確認
    assert param_assert == param_return


def test__shaping_triangle_fan_01():
    """TRIANGLE_FAN形式に成形された座標情報取得
    """
    # 試験用データの定義
    # TRIANGLE_FANを構成する座標
    point_01 = (2.5, 2.5, 5.0)
    point_02 = (0.0, 0.0, 3.0)
    point_03 = (5.0, 0.0, 3.0)
    point_04 = (5.0, 5.0, 3.0)
    point_05 = (0.0, 5.0, 3.0)
    point_06 = (0.0, 0.0, 3.0)
    point_list = [
            point_01,
            point_02,
            point_03,
            point_04,
            point_05,
            point_06]

    # 期待値の定義
    # TRIANGLE_FANの始点を定義
    start_point = point_01

    # TRIANGLE_FANの座標を格納した配列を定義
    triangle_01 = [start_point, point_02, point_03]
    triangle_02 = [start_point, point_03, point_04]
    triangle_03 = [start_point, point_04, point_05]
    triangle_04 = [start_point, point_05, point_06]
    param_assert = [triangle_01, triangle_02, triangle_03, triangle_04]

    # 試験実施
    # TRIANGLE_FAN形式に成形した座標を取得
    param_return = _shaping_triangle_fan(point_list)

    # 戻り値を確認
    assert param_assert == param_return


def test__shaping_triangle_fan_02():
    """TRIANGLE_FAN境界値試験、頂点3点の座標成形
    + 試験詳細
      - 入力条件
        - 頂点座標が3点
      - 確認内容
        - 頂点座標が3点の場合は、TRIANGLE_FANの座標情報が取得できること
    """
    # 試験用データの定義
    # TRIANGLE_FANを構成する座標
    point_01 = (2.5, 2.5, 5.0)
    point_02 = (0.0, 0.0, 3.0)
    point_03 = (5.0, 0.0, 3.0)
    point_list = [
            point_01,
            point_02,
            point_03]

    # 期待値の定義
    # TRIANGLE_FANの始点を定義
    start_point = point_01

    # TRIANGLE_FANの座標を格納した配列を定義
    triangle_01 = [start_point, point_02, point_03]
    param_assert = [triangle_01]

    # 試験実施
    # TRIANGLE_FAN形式に成形した座標を取得
    param_return = _shaping_triangle_fan(point_list)

    # 戻り値を確認
    assert param_assert == param_return


def test__shaping_triangle_fan_03():
    """TRIANGLE_FAN境界値試験、頂点2点の座標成形
    + 試験詳細
      - 入力条件
        - 頂点座標が2点以下
      - 確認内容
        - 頂点座標が2点以下の場合は、例外が発生すること
    """
    # 試験用データの定義
    # TRIANGLE_FANを構成する座標
    point_01 = (2.5, 2.5, 5.0)
    point_02 = (0.0, 0.0, 3.0)
    point_list = [point_01, point_02]

    # 期待値の取得
    # 例外取得
    exception_assert = SpatialIdError('SHAPEFILE_PARTS_POINTS_NUM_ERROR')

    # 試験実施
    # 例外発生確認
    with pytest.raises(SpatialIdError) as e:
        # 頂点座標が2点の座標リストを渡す
        _shaping_triangle_fan(point_list)

    # 例外メッセージ確認
    assert str(exception_assert) == str(e.value)


def test__shaping_triangle_strip_01():
    """TRIANGLE_STRIP形式に成形された座標情報取得
    """
    # 試験用データの定義
    # TRIANGLE_STRIPを構成する座標
    point_01 = (0.0, 0.0, 0.0)
    point_02 = (0.0, 0.0, 3.0)
    point_03 = (5.0, 0.0, 0.0)
    point_04 = (5.0, 0.0, 3.0)
    point_05 = (5.0, 5.0, 0.0)
    point_06 = (5.0, 5.0, 3.0)
    point_list = [
            point_01,
            point_02,
            point_03,
            point_04,
            point_05,
            point_06]

    # 期待値の定義
    # TRIANGLE_STRIPの座標を格納した配列を定義
    triangle_01 = [point_01, point_02, point_03]
    triangle_02 = [point_02, point_03, point_04]
    triangle_03 = [point_03, point_04, point_05]
    triangle_04 = [point_04, point_05, point_06]
    param_assert = [triangle_01, triangle_02, triangle_03, triangle_04]

    # 試験実施
    # TRIANGLE_STRIP形式に成形した座標を取得
    param_return = _shaping_triangle_strip(point_list)

    # 戻り値を確認
    assert param_assert == param_return


def test__shaping_triangle_strip_02():
    """TRIANGLE_STRIPの境界値試験、頂点3点の座標成形
    + 試験詳細
      - 入力条件
        - 頂点座標が3点
      - 確認内容
        - 頂点座標が3点の場合は、TRIANGLE_STRIPの座標情報が取得できること
    """
    # 試験用データの定義
    # TRIANGLE_STRIPを構成する座標
    point_01 = (0.0, 0.0, 0.0)
    point_02 = (0.0, 0.0, 3.0)
    point_03 = (5.0, 0.0, 0.0)
    point_list = [
            point_01,
            point_02,
            point_03]

    # 期待値の定義
    # TRIANGLE_STRIPの座標を格納した配列を定義
    triangle_01 = [point_01, point_02, point_03]
    param_assert = [triangle_01]

    # 試験実施
    # TRIANGLE_STRIP形式に成形した座標を取得
    param_return = _shaping_triangle_strip(point_list)

    # 戻り値を確認
    assert param_assert == param_return


def test__shaping_triangle_strip_03():
    """TRIANGLE_STRIPの境界値試験、頂点2点の座標成形
    + 試験詳細
      - 入力条件
        - 頂点座標が2点
      - 確認内容
        - 頂点座標が2点の場合は、例外が発生すること
    """
    # 試験用データの定義
    # TRIANGLE_STRIPを構成する座標
    point_01 = (0.0, 0.0, 0.0)
    point_02 = (5.0, 0.0, 0.0)
    point_list = [
            point_01,
            point_02]

    # 期待値の定義
    # 例外取得
    exception_assert = SpatialIdError('SHAPEFILE_PARTS_POINTS_NUM_ERROR')

    # 試験実施
    # 例外発生確認
    with pytest.raises(SpatialIdError) as e:
        # 頂点座標が2点の座標リストを渡す
        _shaping_triangle_strip(point_list)

    # 例外メッセージ確認
    assert str(exception_assert) == str(e.value)


def test__shaping_multipatch_parts_01():
    """partTypeがTRIANGLE_STRIPの座標成形
    + 試験詳細
      - 入力条件
        - partType: TRIANGLE_STRIP (0)
      - 確認内容
        - partType に 0 が指定された場合、TRIANGLE_STRIPの形式に成形された座標が取得できること
    """
    # 試験用データの定義
    # partTypeを定義
    part_type = TRIANGLE_STRIP

    # TRIANGLE_STRIPを構成する座標
    point_01 = (0.0, 0.0, 0.0)
    point_02 = (0.0, 0.0, 3.0)
    point_03 = (5.0, 0.0, 0.0)
    point_04 = (5.0, 0.0, 3.0)
    point_05 = (5.0, 5.0, 0.0)
    point_06 = (5.0, 5.0, 3.0)
    point_list = [
            point_01,
            point_02,
            point_03,
            point_04,
            point_05,
            point_06]

    # 期待値の定義
    # TRIANGLE_STRIPの座標を格納した配列を定義
    triangle_01 = [point_01, point_02, point_03]
    triangle_02 = [point_02, point_03, point_04]
    triangle_03 = [point_03, point_04, point_05]
    triangle_04 = [point_04, point_05, point_06]
    param_assert = [triangle_01, triangle_02, triangle_03, triangle_04]

    # 試験実施
    # TRIANGLE_STRIP形式に成形した座標を取得
    param_return = _shaping_multipatch_parts(point_list, part_type)

    # 戻り値の確認
    assert param_assert == param_return


def test__shaping_multipatch_parts_02():
    """partTypeがTRIANGLE_FANの座標成形
    + 試験詳細
      - 入力条件
        - partType: TRIANGLE_FAN (1)
      - 確認内容
        - partType に 1 が指定された場合、TRIANGLE_FANの形式に成形された座標が取得できること
    """
    # 試験用データの定義
    # partTypeを定義
    part_type = TRIANGLE_FAN

    # TRIANGLE_FANを構成する座標
    point_01 = (2.5, 2.5, 5.0)
    point_02 = (0.0, 0.0, 3.0)
    point_03 = (5.0, 0.0, 3.0)
    point_04 = (5.0, 5.0, 3.0)
    point_05 = (0.0, 5.0, 3.0)
    point_06 = (0.0, 0.0, 3.0)
    point_list = [
            point_01,
            point_02,
            point_03,
            point_04,
            point_05,
            point_06]

    # 期待値の定義
    # TRIANGLE_FANの始点を定義
    start_point = point_01

    # TRIANGLE_FANの座標を格納した配列を定義
    triangle_01 = [start_point, point_02, point_03]
    triangle_02 = [start_point, point_03, point_04]
    triangle_03 = [start_point, point_04, point_05]
    triangle_04 = [start_point, point_05, point_06]
    param_assert = [triangle_01, triangle_02, triangle_03, triangle_04]

    # 試験実施
    # TRIANGLE_FAN形式に成形した座標を取得
    param_return = _shaping_multipatch_parts(point_list, part_type)

    # 戻り値の確認
    assert param_assert == param_return


def test__shaping_multipatch_parts_03():
    """partTypeがサポート対象外の座標成形
    + 試験詳細
      - 入力条件
        - partType: 2 (サポート対象外)
      - 確認内容
        - partType にサポート対象外の数値が指定された場合、例外が発生すること。
    """
    # 試験用データの定義
    # サポート対象外のpartType(OUTER_RING)を定義
    part_type = 2

    # OUTER_RINGを構成する座標
    point_01 = (2.5, 2.5, 3.0)
    point_02 = (0.0, 0.0, 3.0)
    point_03 = (5.0, 0.0, 3.0)
    point_04 = (5.0, 5.0, 3.0)
    point_list = [
            point_01,
            point_02,
            point_03,
            point_04]

    # 期待値の取得
    # 例外取得
    exception_assert = SpatialIdError('SHAPEFILE_PARTETYPE_ERROR')

    # 試験実施
    # 例外発生確認
    with pytest.raises(SpatialIdError) as e:
        # サポート対象外のpartTypeを引数に渡す
        _shaping_multipatch_parts(point_list, part_type)

    # 例外メッセージ確認
    assert str(exception_assert) == str(e.value)


def test__combine_x_y_z_point_01():
    """x, y, z 座標を組み合わせた座標データ取得
    """
    # 試験用データの定義
    # xy, z 座標情報を定義
    point_xy = [(2.5, 2.5), (2.0, 1.0)]
    point_z = [5, 10]

    # 期待値の定義
    # xy, z 結合後の配列を定義
    param_assert = []

    for x_y, z in zip(point_xy, point_z):
        param_assert.append(Projected_Point(0,0,lon=x_y[0], lat=x_y[1], alt=z))

    # 試験実施
    # 戻り値の取得
    param_return = _combine_x_y_z_point(point_xy, point_z, 4326)

    # 戻り値の確認
    assert param_assert[0].lon == param_return[0].lon
    assert param_assert[0].lat == param_return[0].lat
    assert param_assert[0].alt == param_return[0].alt
    assert param_assert[1].lon == param_return[1].lon
    assert param_assert[1].lat == param_return[1].lat
    assert param_assert[1].alt == param_return[1].alt


def test__combine_x_y_z_point_02():
    """xy, z 座標のリストが空の場合
    + 試験詳細
      - 入力条件
        - xy座標, z座標のリストともに空
      - 確認内容
        - 空のリストが返却されること
    """
    # 試験用データの定義
    # xy, z 座標情報を定義
    point_xy = []
    point_z = []

    # 期待値の定義
    param_assert = []

    # 試験実施
    # 戻り値の取得
    param_return = _combine_x_y_z_point(point_xy, point_z, 4326)

    # 戻り値の確認
    assert param_assert == param_return


def test__combine_x_y_z_point_03():
    """xy, z の座標の要素数不一致の場合
    + 試験詳細
      - 入力条件
        - xy座標、z座標の要素数が一致していない
      - 確認内容
        - 要素数不一致の場合は例外が発生すること
    """
    # 試験用データの定義
    # xy, z 座標情報を定義 - 要素数不一致
    point_xy = [(2.5, 2.5), (2.0, 1.0)]
    point_z = [5]

    # 期待値の定義
    # 例外取得
    exception_assert = SpatialIdError('SHAPEFILE_POINTS_NUM_ERROR')

    # 試験実施
    # 例外発生確認
    with pytest.raises(SpatialIdError) as e:
        # 頂点座標が2点の座標リストを渡す
        _combine_x_y_z_point(point_xy, point_z, 4326)

    # 例外メッセージ確認
    assert str(exception_assert) == str(e.value)


def test__get_multipatch_data_01():
    """shapeTypeがMultipatchのレコード情報取得

    """
    # 試験用データの定義
    # shapefileに格納されたデータの座標系
    shapefile_crs = 4326

    # Pointの座標を格納したリストを定義
    point_01 = Projected_Point(0.0,0.0,lon=139.051685, lat=45.060433, alt=150.5)
    point_02 = Projected_Point(0.0,0.0,lon=140.051685, lat=46.060433, alt=151.5)
    point_03 = Projected_Point(0.0,0.0,lon=141.051685, lat=47.060433, alt=152.5)
    point_04 = Projected_Point(0.0,0.0,lon=139.051685, lat=45.060433, alt=151.5)
    point_list=[
        point_01,
        point_02,
        point_03,
        point_04
    ]
    # partTypeを定義
    part_type = [1]

    # partsを定義
    parts = [0]

    # 期待値の定義
    # Pointオブジェクトを定義
    spatial_point_list = []

    for projected_point in point_list:
        spatial_point_list.append(Projected_Point(
          projected_point.x,
          projected_point.y,
          projected_point.alt,
          projected_point.lon,
          projected_point.lat))

    # 始点を定義
    start_point = spatial_point_list[0]

    # TRIANGLE_FANのオブジェクト作成
    triangle_fan_01 = SpatialProjectedTriangle(
            start_point, spatial_point_list[1], spatial_point_list[2])
    triangle_fan_02 = SpatialProjectedTriangle(
            start_point, spatial_point_list[2], spatial_point_list[3])
    param_assert_list = [triangle_fan_01, triangle_fan_02]

    # 試験実施
    # 戻り値を取得
    param_return = _get_multipatch_data(
            point_list, parts, part_type)

    # 戻り値を確認
    assert 2 == len(param_return)
    for cnt in range(len(param_return)):
        assert param_assert_list[cnt].p1.lon == param_return[cnt].p1.lon
        assert param_assert_list[cnt].p1.lat == param_return[cnt].p1.lat
        assert param_assert_list[cnt].p1.alt == param_return[cnt].p1.alt
        assert param_assert_list[cnt].p2.lon == param_return[cnt].p2.lon
        assert param_assert_list[cnt].p2.lat == param_return[cnt].p2.lat
        assert param_assert_list[cnt].p2.alt == param_return[cnt].p2.alt
        assert param_assert_list[cnt].p3.lon == param_return[cnt].p3.lon
        assert param_assert_list[cnt].p3.lat == param_return[cnt].p3.lat
        assert param_assert_list[cnt].p3.alt == param_return[cnt].p3.alt


def test__get_multipatch_data_02():
    """parts数が2つ以上のMultipatchのレコード情報取
    + 試験詳細
      - 入力条件
        - partsの要素数が2つ以上
      - 確認内容
        - 全てのparts文のTriangleオブジェクトが返却されること
    """
    # 試験用データの定義
    # shapefileに格納されたデータの座標系
    shapefile_crs = 4326

    # Pointの座標を格納したリストを定義
    point_01 = Projected_Point(0.0,0.0,lon=139.051685, lat=45.060433, alt=150.5)
    point_02 = Projected_Point(0.0,0.0,lon=140.051685, lat=46.060433, alt=151.5)
    point_03 = Projected_Point(0.0,0.0,lon=141.051685, lat=47.060433, alt=152.5)
    point_04 = Projected_Point(0.0,0.0,lon=142.051685, lat=48.060433, alt=151.5)
    point_05 = Projected_Point(0.0,0.0,lon=139.151685, lat=45.160433, alt=140.5)
    point_06 = Projected_Point(0.0,0.0,lon=140.151685, lat=46.160433, alt=141.5)
    point_07 = Projected_Point(0.0,0.0,lon=141.151685, lat=47.160433, alt=142.5)
    point_08 = Projected_Point(0.0,0.0,lon=142.151685, lat=48.160433, alt=142.5)
    point_list = [
            point_01,
            point_02,
            point_03,
            point_04,
            point_05,
            point_06,
            point_07,
            point_08,
            ]

    # partTypeを定義
    part_type = [1, 0]

    # partsを定義
    parts = [0, 4]

    # 期待値の定義
    # Pointオブジェクトを定義
    spatial_point_list = []

    for projected_point in point_list:
        spatial_point_list.append(Projected_Point(
          projected_point.x,
          projected_point.y,
          projected_point.alt,
          projected_point.lon,
          projected_point.lat))

    # 始点を定義
    start_point = spatial_point_list[0]
    # TRIANGLE_FANのオブジェクト作成
    triangle_fan_01 = SpatialProjectedTriangle(
            start_point, spatial_point_list[1], spatial_point_list[2])
    triangle_fan_02 = SpatialProjectedTriangle(
            start_point, spatial_point_list[2], spatial_point_list[3])
    # TRIANGLE_STRIPのオブジェクト作成
    triangle_strip_01 = SpatialProjectedTriangle(
            spatial_point_list[4],
            spatial_point_list[5],
            spatial_point_list[6])
    triangle_strip_02 = SpatialProjectedTriangle(
            spatial_point_list[5],
            spatial_point_list[6],
            spatial_point_list[7])
    param_assert_list = [
            triangle_fan_01,
            triangle_fan_02,
            triangle_strip_01,
            triangle_strip_02]

    # 試験実施
    # 戻り値を取得
    param_return = _get_multipatch_data(
            point_list, parts, part_type)

    # 戻り値を確認
    assert 4 == len(param_return)

    for cnt in range(len(param_return)):
        assert param_assert_list[cnt].p1.lon == param_return[cnt].p1.lon
        assert param_assert_list[cnt].p1.lat == param_return[cnt].p1.lat
        assert param_assert_list[cnt].p1.alt == param_return[cnt].p1.alt
        assert param_assert_list[cnt].p2.lon == param_return[cnt].p2.lon
        assert param_assert_list[cnt].p2.lat == param_return[cnt].p2.lat
        assert param_assert_list[cnt].p2.alt == param_return[cnt].p2.alt
        assert param_assert_list[cnt].p3.lon == param_return[cnt].p3.lon
        assert param_assert_list[cnt].p3.lat == param_return[cnt].p3.lat
        assert param_assert_list[cnt].p3.alt == param_return[cnt].p3.alt


def test__get_multipatch_data_03():
    """pointsが空のMultipatchのレコード情報取得
    + 試験詳細
      - 入力条件
        - pointsの配列が空
      - 確認内容
        - pointsの配列が空の場合、例外が発生すること
    """
    # 試験用データの定義
    # shapefileに格納されたデータの座標系
    shapefile_crs = 4326

    # Pointの座標を格納したリストを定義
    point_list = []

    # partTypeを定義
    part_type = [1, 0]

    # partsを定義
    parts = [0, 4]

    # 期待値の定義
    # 例外取得
    exception_assert = SpatialIdError('SHAPEFILE_PARTS_POINTS_NUM_ERROR')

    # 試験実施
    # 例外発生確認
    with pytest.raises(SpatialIdError) as e:
        # 空の座標リストを渡す
        _get_multipatch_data(
                point_list, parts, part_type)

    # 例外メッセージ確認
    assert str(exception_assert) == str(e.value)


def test__get_multipatch_data_04():
    """partTypeが空のMultipatchのレコード情報取得
    + 試験詳細
      - 入力条件
        - part_typeの配列が空
      - 確認内容
        - part_typeが空の場合、例外が発生すること
    """
    # 試験用データの定義
    # shapefileに格納されたデータの座標系
    shapefile_crs = 4326

    # Pointの座標を格納したリストを定義
    point_01 = Projected_Point(0.0,0.0,lon=139.051685, lat=45.060433, alt=150.5)
    point_02 = Projected_Point(0.0,0.0,lon=140.051685, lat=46.060433, alt=151.5)
    point_03 = Projected_Point(0.0,0.0,lon=141.051685, lat=47.060433, alt=152.5)
    point_04 = Projected_Point(0.0,0.0,lon=142.051685, lat=48.060433, alt=151.5)
    point_list = [
            point_01,
            point_02,
            point_03,
            point_04]

    # partTypeを定義
    part_type = []

    # partsを定義
    parts = [0]

    # 期待値の定義
    # 例外取得
    exception_assert = SpatialIdError('SHAPEFILE_PART_TYPE_LIST_ERROR')

    # 試験実施
    # 例外発生確認
    with pytest.raises(SpatialIdError) as e:
        # 空の座標リストを渡す
        _get_multipatch_data(
                point_list, parts, part_type)

    # 例外メッセージ確認
    assert str(exception_assert) == str(e.value)


def test__get_multipatch_data_05():
    """partsがMultipatchのレコード情報取得
    + 試験詳細
      - 入力条件
        - partsの配列が空
      - 確認内容
        - partsが空の場合、例外が発生すること
    """
    # 試験用データの定義
    # shapefileに格納されたデータの座標系
    shapefile_crs = 4326

    # Pointの座標を格納したリストを定義
    point_01 = Projected_Point(0.0,0.0,lon=139.051685, lat=45.060433, alt=150.5)
    point_02 = Projected_Point(0.0,0.0,lon=140.051685, lat=46.060433, alt=151.5)
    point_03 = Projected_Point(0.0,0.0,lon=141.051685, lat=47.060433, alt=152.5)
    point_04 = Projected_Point(0.0,0.0,lon=142.051685, lat=48.060433, alt=151.5)
    point_list = [
            point_01,
            point_02,
            point_03,
            point_04]

    # partTypeを定義
    part_type = [1]

    # partsを定義
    parts = []

    # 期待値の定義
    # 例外取得
    exception_assert = SpatialIdError('SHAPEFILE_PARTS_LIST_ERROR')

    # 試験実施
    # 例外発生確認
    with pytest.raises(SpatialIdError) as e:
        # 空のパーツリストを渡す
        _get_multipatch_data(
                point_list, parts, part_type)

    # 例外メッセージ確認
    assert str(exception_assert) == str(e.value)


def test__get_shapefile_spatial_id_and_record_01(
        geographic_shape_point_data_01,
        geographic_shape_point_data_02
):
    """shapefileReaderから空間ID及びペイロードを取得、サポート対象のshapeType、水平/垂直方向精度に閾値下限を入力
    + 試験詳細
      - 入力条件
        - サポート対象のshapeTypeのshapefileを読み込み
        - shapefileレコード数が2レコード以上
        - 水平方向精度:1
        - 垂直方向精度:1
      - 確認内容
        - 空間ID、ペイロードが格納されたgeneratorが返却されること
    """
    # 試験用データの定義
    # 試験用shalefileパス
    filepath = SHAPEFILE_DIR + 'test__get_shapefile_spatial_id_and_record_01'

    # 水平、垂直方向精度を定義
    h_zoom = 1
    v_zoom = 1

    # shapefileに格納するデータの座標系
    shapefile_crs = 4326

    # shapefile作成用の座標配列取得
    fan_point_list_01 = geographic_shape_point_data_01[0][0]
    fan_point_type_01 = geographic_shape_point_data_01[1][0]
    fan_point_list_02 = geographic_shape_point_data_01[0][1]
    fan_point_type_02 = geographic_shape_point_data_01[1][1]

    fan_point_list_03 = geographic_shape_point_data_02[0][0]
    fan_point_type_03 = geographic_shape_point_data_02[1][0]
    fan_point_list_04 = geographic_shape_point_data_02[0][1]
    fan_point_type_04 = geographic_shape_point_data_02[1][1]

    # shapefile生成
    w = sf.Writer(filepath)
    w.field('name', 'C')
    w.field('type', 'C')
    w.multipatch(
            [fan_point_list_01, fan_point_list_02],
            partTypes=[TRIANGLE_FAN, TRIANGLE_FAN])
    w.record('record1', 'FAN')
    w.multipatch(
            [fan_point_list_03, fan_point_list_04],
            partTypes=[TRIANGLE_FAN, TRIANGLE_FAN])
    w.record('record2', 'FAN')
    w.close()

    # 期待値取得
    # ペイロードの期待値を定義
    shapefile_record_list = [['record1', 'FAN'], ['record2', 'FAN']]

    # TRIANGLE_FANのSpatialPointオブジェクトを生成
    spatial_fan_point_list_01 = _get_spatial_point_objects(fan_point_list_01)
    spatial_fan_point_list_02 = _get_spatial_point_objects(fan_point_list_02)
    spatial_fan_point_list_03 = _get_spatial_point_objects(fan_point_list_03)
    spatial_fan_point_list_04 = _get_spatial_point_objects(fan_point_list_04)

    # SpatialTriangleオブジェクト生成
    spatial_triangle_list_01 = \
        _shaping_point_to_triangle(
                spatial_fan_point_list_01, fan_point_type_01)
    spatial_triangle_list_02 = \
        _shaping_point_to_triangle(
                spatial_fan_point_list_02, fan_point_type_02)
    spatial_triangle_list_03 = \
        _shaping_point_to_triangle(
                spatial_fan_point_list_03, fan_point_type_03)
    spatial_triangle_list_04 = \
        _shaping_point_to_triangle(
                spatial_fan_point_list_04, fan_point_type_04)

    # 空間ID取得対象のSpatialTriangleを格納した配列を定義
    triangle_list_01 = spatial_triangle_list_01 + spatial_triangle_list_02
    triangle_list_02 = spatial_triangle_list_03 + spatial_triangle_list_04

    # 空間IDの期待値を定義
    shapefile_spatial_ids_01 = get_spatial_ids_on_polygons(
            triangle_list_01, [], h_zoom, v_zoom)
    shapefile_spatial_ids_02 = get_spatial_ids_on_polygons(
            triangle_list_02, [], h_zoom, v_zoom)
    shapefile_spatial_id_list = [
            shapefile_spatial_ids_01,
            shapefile_spatial_ids_02]

    # 試験実施
    # shapefile読み込み
    with sf.Reader(filepath) as sf_reader:
        # 戻り値取得
        param_return = _get_shapefile_spatial_id_and_record(
                sf_reader, h_zoom, v_zoom, shapefile_crs)

        # 戻り値確認
        loop_cnt = 0

        for ids, records in param_return:
            # APIから取得した空間IDは順不同のため比較前にソートし、格納された空間IDが一致していることを確認する。
            param_assert_ids = sorted(shapefile_spatial_id_list[loop_cnt])
            param_return_ids = sorted(ids)

            # 戻り値確認 - 空間ID, ペイロード
            assert param_assert_ids == param_return_ids
            assert shapefile_record_list[loop_cnt] == records

            loop_cnt += 1


def test__get_shapefile_spatial_id_and_record_02():
    """shapefileReaderから空間ID及びペイロードを取得、サポート対象外のshapeType
    + 試験詳細
      - 入力条件
        - サポート対象外のshapeTypeのshapefileを読み込み
      - 確認内容
        - サポート対象外のshapeTypeの場合例外が発生すること
    """
    # 試験用データの定義
    # 試験用shalefileパス
    filepath = SHAPEFILE_DIR + 'test__get_shapefile_spatial_id_and_record_02'

    # 水平、垂直方向精度を定義
    h_zoom = 1
    v_zoom = 1

    # shapefileに格納するデータの座標系
    shapefile_crs = 4326

    # Pointの座標を格納したリストを定義
    point_01 = (139.051685, 45.060433, 150.5)
    point_02 = (140.051685, 46.060433, 151.5)
    point_03 = (141.051685, 47.060433, 152.5)

    # shapefile作成用の座標配列
    point_list = [
            point_01,
            point_02,
            point_03
            ]

    # shapefile生成
    w = sf.Writer(filepath)
    w.field('name', 'C')
    w.field('type', 'C')
    w.multipointz(point_list)
    w.record('record1', 'MPOINTZ')
    w.close()

    # 期待値取得
    # 例外取得
    exception_assert = SpatialIdError('SHAPEFILE_SHAPETYPE_ERROR')

    # 試験実施
    # shapefile読み込み
    with sf.Reader(filepath) as sf_reader:
        # 戻り値取得
        param_return = _get_shapefile_spatial_id_and_record(
                sf_reader, h_zoom, v_zoom, shapefile_crs)

        # 例外発生確認
        with pytest.raises(SpatialIdError) as e:
            # Generator内での例外発生を検知するため、Generatorの要素を参照
            param_return.__next__()

        # 例外メッセージ確認
        assert str(exception_assert) == str(e.value)


def test__get_shapefile_spatial_id_and_record_03(
        geographic_shape_point_data_01
):
    """shapefileReaderから空間ID及びペイロードを取得、水平方向精度に閾値下限より小さい値を入力
    + 試験詳細
      - 入力条件
        - 水平方向精度：0
      - 確認内容
        - ポリゴンの空間ID取得メソッドで例外が発生すること
    """
    # 試験用データの定義
    # 試験用shalefileパス
    filepath = SHAPEFILE_DIR + 'test__get_shapefile_spatial_id_and_record_03'

    # 水平、垂直方向精度を定義
    h_zoom = -1
    v_zoom = 1

    # shapefileに格納するデータの座標系
    shapefile_crs = 4326

    # shapefile作成用の座標配列取得
    fan_point_list_01 = geographic_shape_point_data_01[0][0]
    fan_point_list_02 = geographic_shape_point_data_01[0][1]

    # shapefile生成
    w = sf.Writer(filepath)
    w.field('name', 'C')
    w.field('type', 'C')
    w.multipatch(
            [fan_point_list_01, fan_point_list_02],
            partTypes=[TRIANGLE_FAN, TRIANGLE_FAN])
    w.record('record1', 'FAN')
    w.close()

    # 期待値取得
    # 例外取得
    exception_assert = SpatialIdError('INPUT_VALUE_ERROR')

    # 試験実施
    # shapefile読み込み
    with sf.Reader(filepath) as sf_reader:

        # 戻り値取得
        param_return = _get_shapefile_spatial_id_and_record(
                sf_reader, h_zoom, v_zoom, shapefile_crs)

        # 例外発生確認
        with pytest.raises(SpatialIdError) as e:
            # Generator内での例外発生を検知するため、Generatorの要素を参照
            param_return.__next__()

        # 例外メッセージ確認
        assert str(exception_assert) == str(e.value)


def test__get_shapefile_spatial_id_and_record_04(
        geographic_shape_point_data_03
):
    """shapefileReaderから空間ID及びペイロードを取得、水平方向精度に閾値上限を入力
    + 試験詳細
      - 入力条件
        - 水平方向精度：35
      - 確認内容
        - 空間ID、ペイロードが格納されたgeneratorが返却されること
    """
    # 試験用データの定義
    # 試験用shalefileパス
    filepath = SHAPEFILE_DIR + 'test__get_shapefile_spatial_id_and_record_04'

    # 水平、垂直方向精度を定義
    h_zoom = 35
    v_zoom = 1

    # shapefileに格納するデータの座標系
    shapefile_crs = 4326

    # shapefile作成用の座標配列取得
    fan_point_list_01 = geographic_shape_point_data_03[0][0]
    fan_point_type_01 = geographic_shape_point_data_03[1][0]
    fan_point_list_02 = geographic_shape_point_data_03[0][1]
    fan_point_type_02 = geographic_shape_point_data_03[1][1]

    # shapefile生成
    w = sf.Writer(filepath)
    w.field('name', 'C')
    w.field('type', 'C')
    w.multipatch(
            [fan_point_list_01, fan_point_list_02],
            partTypes=[TRIANGLE_FAN, TRIANGLE_FAN])
    w.record('record1', 'FAN')
    w.close()

    # 期待値取得
    # ペイロードの期待値を定義
    shapefile_record_list = [['record1', 'FAN']]

    # TRIANGLE_FANのSpatialPointオブジェクトを生成
    spatial_fan_point_list_01 = _get_spatial_point_objects(fan_point_list_01)
    spatial_fan_point_list_02 = _get_spatial_point_objects(fan_point_list_02)

    # SpatialTriangleオブジェクト生成
    spatial_triangle_list_01 = \
        _shaping_point_to_triangle(
                spatial_fan_point_list_01, fan_point_type_01)
    spatial_triangle_list_02 = \
        _shaping_point_to_triangle(
                spatial_fan_point_list_02, fan_point_type_02)

    # 空間ID取得対象のSpatialTriangleを格納した配列を定義
    triangle_list = spatial_triangle_list_01 + spatial_triangle_list_02

    # 空間IDの期待値を定義
    shapefile_spatial_id_list = [get_spatial_ids_on_polygons(
              triangle_list, [], h_zoom, v_zoom)]

    # 試験実施
    # shapefile読み込み
    with sf.Reader(filepath) as sf_reader:
        # 戻り値取得
        param_return = _get_shapefile_spatial_id_and_record(
                sf_reader, h_zoom, v_zoom, shapefile_crs)

        # 戻り値確認
        loop_cnt = 0

        for ids, records in param_return:
            # APIから取得した空間IDは順不同のため比較前にソートし、格納された空間IDが一致していることを確認する。
            param_assert_ids = sorted(shapefile_spatial_id_list[loop_cnt])
            param_return_ids = sorted(ids)

            # 戻り値確認 - 空間ID, ペイロード
            assert param_assert_ids == param_return_ids
            assert shapefile_record_list[loop_cnt] == records

            loop_cnt += 1


def test__get_shapefile_spatial_id_and_record_05(
        geographic_shape_point_data_01
):
    """shapefileReaderから空間ID及びペイロードを取得、水平方向精度に上限閾値より大きい値を入力
    + 試験詳細
      - 入力条件
        - 水平方向精度：41
      - 確認内容
        - ポリゴンの空間ID取得メソッドで例外が発生すること
    """
    # 試験用データの定義
    # 試験用shalefileパス
    filepath = SHAPEFILE_DIR + 'test__get_shapefile_spatial_id_and_record_05'

    # 水平、垂直方向精度を定義
    h_zoom = 41
    v_zoom = 1

    # shapefileに格納するデータの座標系
    shapefile_crs = 4326

    # shapefile作成用の座標配列取得
    fan_point_list_01 = geographic_shape_point_data_01[0][0]
    fan_point_list_02 = geographic_shape_point_data_01[0][1]

    # shapefile生成
    w = sf.Writer(filepath)
    w.field('name', 'C')
    w.field('type', 'C')
    w.multipatch(
            [fan_point_list_01, fan_point_list_02],
            partTypes=[TRIANGLE_FAN, TRIANGLE_FAN])
    w.record('record1', 'FAN')
    w.close()

    # 期待値取得
    # 例外取得
    exception_assert = SpatialIdError('INPUT_VALUE_ERROR')

    # 試験実施
    # shapefile読み込み
    with sf.Reader(filepath) as sf_reader:

        # 戻り値取得
        param_return = _get_shapefile_spatial_id_and_record(
                sf_reader, h_zoom, v_zoom, shapefile_crs)

        # 例外発生確認
        with pytest.raises(SpatialIdError) as e:
            # Generator内での例外発生を検知するため、Generatorの要素を参照
            param_return.__next__()

        # 例外メッセージ確認
        assert str(exception_assert) == str(e.value)


def test__get_shapefile_spatial_id_and_record_06(
        geographic_shape_point_data_01
):
    """shapefileReaderから空間ID及びペイロードを取得、垂直方向精度に下限閾値より小さい値を入力
    + 試験詳細
      - 入力条件
        - 垂直方向精度:-1
      - 確認内容
        - ポリゴンの空間ID取得メソッドで例外が発生すること
    """
    # 試験用データの定義
    # 試験用shalefileパス
    filepath = SHAPEFILE_DIR + 'test__get_shapefile_spatial_id_and_record_06'

    # 水平、垂直方向精度を定義
    h_zoom = 1
    v_zoom = -1

    # shapefileに格納するデータの座標系
    shapefile_crs = 4326

    # shapefile作成用の座標配列取得
    fan_point_list_01 = geographic_shape_point_data_01[0][0]
    fan_point_list_02 = geographic_shape_point_data_01[0][1]

    # shapefile生成
    w = sf.Writer(filepath)
    w.field('name', 'C')
    w.field('type', 'C')
    w.multipatch(
            [fan_point_list_01, fan_point_list_02],
            partTypes=[TRIANGLE_FAN, TRIANGLE_FAN])
    w.record('record1', 'FAN')
    w.close()

    # 期待値取得
    # 例外取得
    exception_assert = SpatialIdError('INPUT_VALUE_ERROR')

    # 試験実施
    # shapefile読み込み
    with sf.Reader(filepath) as sf_reader:

        # 戻り値取得
        param_return = _get_shapefile_spatial_id_and_record(
                sf_reader, h_zoom, v_zoom, shapefile_crs)

        # 例外発生確認
        with pytest.raises(SpatialIdError) as e:
            # Generator内での例外発生を検知するため、Generatorの要素を参照
            param_return.__next__()

        # 例外メッセージ確認
        assert str(exception_assert) == str(e.value)


def test__get_shapefile_spatial_id_and_record_07(
        geographic_shape_point_data_03
):
    """shapefileReaderから空間ID及びペイロードを取得、垂直方向精度に閾値上限を入力
    + 試験詳細
      - 入力条件
        - 垂直方向精度：35
      - 確認内容
        - 空間ID、ペイロードが格納されたgeneratorが返却されること
    """
    # 試験用データの定義
    # 試験用shalefileパス
    filepath = SHAPEFILE_DIR + 'test__get_shapefile_spatial_id_and_record_07'

    # 水平、垂直方向精度を定義
    h_zoom = 1
    v_zoom = 35

    # shapefileに格納するデータの座標系
    shapefile_crs = 4326

    # shapefile作成用の座標配列取得
    fan_point_list_01 = geographic_shape_point_data_03[0][0]
    fan_point_type_01 = geographic_shape_point_data_03[1][0]
    fan_point_list_02 = geographic_shape_point_data_03[0][1]
    fan_point_type_02 = geographic_shape_point_data_03[1][1]

    # shapefile生成
    w = sf.Writer(filepath)
    w.field('name', 'C')
    w.field('type', 'C')
    w.multipatch(
            [fan_point_list_01, fan_point_list_02],
            partTypes=[TRIANGLE_FAN, TRIANGLE_FAN])
    w.record('record1', 'FAN')
    w.close()

    # 期待値取得
    # ペイロードの期待値を定義
    shapefile_record_list = [['record1', 'FAN']]

    # TRIANGLE_FANのSpatialPointオブジェクトを生成
    spatial_fan_point_list_01 = _get_spatial_point_objects(fan_point_list_01)
    spatial_fan_point_list_02 = _get_spatial_point_objects(fan_point_list_02)

    # SpatialTriangleオブジェクト生成
    spatial_triangle_list_01 = \
        _shaping_point_to_triangle(
                spatial_fan_point_list_01, fan_point_type_01)
    spatial_triangle_list_02 = \
        _shaping_point_to_triangle(
                spatial_fan_point_list_02, fan_point_type_02)

    # 空間ID取得対象のSpatialTriangleを格納した配列を定義
    triangle_list = spatial_triangle_list_01 + spatial_triangle_list_02

    # 空間IDの期待値を定義
    shapefile_spatial_id_list = [get_spatial_ids_on_polygons(
              triangle_list, [], h_zoom, v_zoom)]

    # 試験実施
    # shapefile読み込み
    with sf.Reader(filepath) as sf_reader:
        # 戻り値取得
        param_return = _get_shapefile_spatial_id_and_record(
                sf_reader, h_zoom, v_zoom, shapefile_crs)

        # 戻り値確認
        loop_cnt = 0

        for ids, records in param_return:
            # APIから取得した空間IDは順不同のため比較前にソートし、格納された空間IDが一致していることを確認する。
            param_assert_ids = sorted(shapefile_spatial_id_list[loop_cnt])
            param_return_ids = sorted(ids)

            # 戻り値確認 - 空間ID, ペイロード
            assert param_assert_ids == param_return_ids
            assert shapefile_record_list[loop_cnt] == records

            loop_cnt += 1


def test__get_shapefile_spatial_id_and_record_08(
        geographic_shape_point_data_01
):
    """shapefileReaderから空間ID及びペイロードを取得、垂直方向精度に閾値上限より大きい値を入力
    + 試験詳細
      - 入力条件
        - 垂直方向精度:36
      - 確認内容
        - ポリゴンの空間ID取得メソッドで例外が発生すること
    """
    # 試験用データの定義
    # 試験用shalefileパス
    filepath = SHAPEFILE_DIR + 'test__get_shapefile_spatial_id_and_record_08'

    # 水平、垂直方向精度を定義
    h_zoom = 1
    v_zoom = 36

    # shapefileに格納するデータの座標系
    shapefile_crs = 4326

    # shapefile作成用の座標配列取得
    fan_point_list_01 = geographic_shape_point_data_01[0][0]
    fan_point_list_02 = geographic_shape_point_data_01[0][1]

    # shapefile生成
    w = sf.Writer(filepath)
    w.field('name', 'C')
    w.field('type', 'C')
    w.multipatch(
            [fan_point_list_01, fan_point_list_02],
            partTypes=[TRIANGLE_FAN, TRIANGLE_FAN])
    w.record('record1', 'FAN')
    w.close()

    # 期待値取得
    # 例外取得
    exception_assert = SpatialIdError('INPUT_VALUE_ERROR')

    # 試験実施
    # shapefile読み込み
    with sf.Reader(filepath) as sf_reader:

        # 戻り値取得
        param_return = _get_shapefile_spatial_id_and_record(
                sf_reader, h_zoom, v_zoom, shapefile_crs)

        # 例外発生確認
        with pytest.raises(SpatialIdError) as e:
            # Generator内での例外発生を検知するため、Generatorの要素を参照
            param_return.__next__()

        # 例外メッセージ確認
        assert str(exception_assert) == str(e.value)


def test__get_shapefile_spatial_id_and_record_09(mocker):
    """partsとpertTypesの要素数が不一致
    + 試験詳細
      - 試験条件
        - shapefileのライブラリでは「parts!=partTypes」のデータが作れないため、
          モックを使用して試験データを作成
      - 確認内容
        - partsとpartTypesの要素数不一致の例外が発生すること
    """
    # 試験用データの定義
    # 試験用shalefileパス
    filepath = SHAPEFILE_DIR + 'test__get_shapefile_spatial_id_and_record_09'

    # 水平、垂直方向精度を定義
    h_zoom = 10
    v_zoom = 10

    # shapefileに格納するデータの座標系
    shapefile_crs = 4326

    # Pointの座標を格納したリストを定義
    strip_point_01 = (139.051685, 45.060433, 150.5)
    strip_point_02 = (140.051685, 46.060433, 151.5)
    strip_point_03 = (141.051685, 47.060433, 152.5)
    strip_point_04 = (142.051685, 48.060433, 151.5)

    # shapefile作成用の座標配列
    strip_point_list = [
            strip_point_01,
            strip_point_02,
            strip_point_03,
            strip_point_04
            ]

    # shapefile生成
    w = sf.Writer(filepath)
    w.field('name', 'C')
    w.field('type', 'C')
    w.multipatch(
            [strip_point_list],
            partTypes=[TRIANGLE_STRIP])
    w.record('record1', 'STRIP')
    w.close()

    # 期待値取得
    # 例外取得
    exception_assert = SpatialIdError('SHAPEFILE_PARTS_NUM_ERROR')

    # 試験実施
    # shapefile読み込み
    with sf.Reader(filepath) as sf_reader:

        # shapefileのライブラリでは「parts!=partTypes」のデータが作れないため、モックで実行
        shape = Shape(
                shapeType=31,
                points=[[0, 0]],
                parts=[1],
                partTypes=[0, 1],
                oid=None)
        shape.z = [0]
        record = ShapeRecord(shape=shape, record=None)
        mocker.patch.object(
                sf_reader, 'iterShapeRecords', return_value=[record])

        # 戻り値取得
        param_return = _get_shapefile_spatial_id_and_record(
                sf_reader, h_zoom, v_zoom, shapefile_crs)

        # 例外発生確認
        with pytest.raises(SpatialIdError) as e:
            # Generator内での例外発生を検知するため、Generatorの要素を参照
            param_return.__next__()

        # 例外メッセージ確認
        assert str(exception_assert) == str(e.value)


def test__get_shapefile_spatial_id_and_record_10(
        projected_shape_point_data_01
):
    """shapefileReaderから空間ID及びペイロードを取得、レコード数1
    + 試験詳細
      - 入力条件
        - shapefileレコード数が1
      - 確認内容
        - 空間ID、ペイロードが格納されたgeneratorが返却されること
    """
    # 試験用データの定義
    # 試験用shalefileパス
    filepath = SHAPEFILE_DIR + 'test__get_shapefile_spatial_id_and_record_10'

    # 水平、垂直方向精度を定義
    h_zoom = 10
    v_zoom = 10

    # shapefileに格納するデータの座標系
    shapefile_crs = 6677

    # shapefile作成用の座標配列取得
    strip_point_list_01 = projected_shape_point_data_01[0][0]
    strip_point_type_01 = projected_shape_point_data_01[1][0]
    strip_point_list_02 = projected_shape_point_data_01[0][1]
    strip_point_type_02 = projected_shape_point_data_01[1][1]
    fan_point_list_01 = projected_shape_point_data_01[0][2]
    fan_point_type_01 = projected_shape_point_data_01[1][2]

    # shapefile生成
    w = sf.Writer(filepath)
    w.field('name', 'C')
    w.field('type', 'C')
    w.multipatch(
            [strip_point_list_01, strip_point_list_02, fan_point_list_01],
            partTypes=[TRIANGLE_STRIP, TRIANGLE_STRIP, TRIANGLE_FAN])
    w.record('record1', 'STRIP, FAN')
    w.close()

    # 期待値取得
    # ペイロードの期待値を定義
    shapefile_record_list = [['record1', 'STRIP, FAN']]

    # TRIANGLE_STRIPのSpatialPointオブジェクトを生成
    spatial_strip_point_list_01 = \
        _get_spatial_point_objects(strip_point_list_01, shapefile_crs)
    spatial_strip_point_list_02 = \
        _get_spatial_point_objects(strip_point_list_02, shapefile_crs)

    # TRIANGLE_FANのSpatialPointオブジェクトを生成
    spatial_fan_point_list_01 = \
        _get_spatial_point_objects(fan_point_list_01, shapefile_crs)

    # SpatialTriangleオブジェクト生成
    spatial_triangle_list_01 = \
        _shaping_point_to_triangle(
                spatial_strip_point_list_01, strip_point_type_01)
    spatial_triangle_list_02 = \
        _shaping_point_to_triangle(
                spatial_strip_point_list_02, strip_point_type_02)
    spatial_triangle_list_03 = \
        _shaping_point_to_triangle(
                spatial_fan_point_list_01, fan_point_type_01)

    # 空間ID取得対象のSpatialTriangleを格納した配列を定義
    triangle_list = spatial_triangle_list_01 + spatial_triangle_list_02\
        + spatial_triangle_list_03

    # 空間IDの期待値を定義
    shapefile_spatial_id_list = [get_spatial_ids_on_polygons(
              triangle_list, [], h_zoom, v_zoom)]

    # 試験実施
    # shapefile読み込み
    with sf.Reader(filepath) as sf_reader:
        # 戻り値取得
        param_return = _get_shapefile_spatial_id_and_record(
                sf_reader, h_zoom, v_zoom, shapefile_crs)

        # 戻り値確認
        loop_cnt = 0

        for ids, records in param_return:
            # APIから取得した空間IDは順不同のため比較前にソートし、格納された空間IDが一致していることを確認する。
            param_assert_ids = sorted(shapefile_spatial_id_list[loop_cnt])
            param_return_ids = sorted(ids)

            # 戻り値確認 - 空間ID, ペイロード
            assert param_assert_ids == param_return_ids
            assert shapefile_record_list[loop_cnt] == records

            loop_cnt += 1


def test__get_fields_data_01():
    """fieldsに格納されている情報を取得、フィールド要素数1
    + 試験詳細
      - 入力条件
        - フィールドの要素数が1のshapefileReader
      - 確認内容
        - フィールドの情報が格納された配列が返却されること
    """
    # 試験用データの定義
    # 試験用shalefileパス
    filepath = SHAPEFILE_DIR + 'test__get_fields_data_01'

    # Pointの座標を格納したリストを定義
    strip_point_01 = (139.051685, 45.060433, 150.5)
    strip_point_02 = (140.051685, 46.060433, 151.5)
    strip_point_03 = (141.051685, 47.060433, 152.5)

    # shapefile作成用の座標配列
    strip_point_list = [
            strip_point_01,
            strip_point_02,
            strip_point_03
            ]

    # shapefile生成
    w = sf.Writer(filepath)
    w.field('name', 'C')
    w.multipatch(
            [strip_point_list],
            partTypes=[TRIANGLE_STRIP])
    w.record('record1')
    w.close()

    # 期待値取得
    # フィールドの期待値を定義
    param_assert_field = [['name', 'C', 50, 0]]

    # 試験実施
    # shapefile読み込み
    with sf.Reader(filepath) as sf_reader:
        # 戻り値取得
        param_return_field = _get_fields_data(sf_reader)

    assert param_assert_field == param_return_field


def test__get_fields_data_02():
    """fieldsに格納されている情報を取得、フィールド要素数複数
    + 試験詳細
      - 入力条件
        - フィールドの要素数が複数のshapefileReader
      - 確認内容
        - フィールドの情報が格納された配列が返却されること
    """
    # 試験用データの定義
    # 試験用shalefileパス
    filepath = SHAPEFILE_DIR + 'test__get_fields_data_02'

    # Pointの座標を格納したリストを定義
    strip_point_01 = (139.051685, 45.060433, 150.5)
    strip_point_02 = (140.051685, 46.060433, 151.5)
    strip_point_03 = (141.051685, 47.060433, 152.5)

    # shapefile作成用の座標配列
    strip_point_list = [
            strip_point_01,
            strip_point_02,
            strip_point_03
            ]

    # shapefile生成
    w = sf.Writer(filepath)
    w.field('name', 'C')
    w.field('age', 'N', 3, 0)
    w.field('weight', 'F', 10, 1)
    w.field('bool', 'L')
    w.field('date', 'D')
    w.field('memo', 'M')
    w.multipatch(
            [strip_point_list],
            partTypes=[TRIANGLE_STRIP])
    w.record('record1', 20, 70.5, True, '20220101', 'Test')
    w.multipatch(
            [strip_point_list],
            partTypes=[TRIANGLE_STRIP])
    w.record('record2', 25, 75.9, False, '20170615', 'Test')
    w.close()

    # 期待値取得
    # フィールドの期待値を定義
    param_assert_field = [
            ['name', 'C', 50, 0],
            ['age', 'N', 3, 0],
            ['weight', 'F', 10, 1],
            ['bool', 'L', 1, 0],
            ['date', 'D', 8, 0],
            ['memo', 'M', 50, 0]]

    # 試験実施
    # shapefile読み込み
    with sf.Reader(filepath) as sf_reader:
        # 戻り値取得
        param_return_field = _get_fields_data(sf_reader)

    assert param_assert_field == param_return_field

def test_f_read_shapefile_01(
        geographic_shape_point_data_01,
        geographic_shape_point_data_02
):
    """Shapefileから1レコード単位で情報を取得するジェネレータ取得、shapefileレコード数2、CSR入力なし
    + 試験詳細
      - 入力条件
        - 2レコードが格納されているshapefileを読み込み
        - 読み込み対象ファイル拡張子無し
        - CSR入力なし
      - 確認内容
        - ジェネレータが返却され、空間ID、フィールド名、ペイロードを1レコード毎に参照可能であること
    """
    # 試験用データの定義
    # 試験用shalefileパス
    filepath = SHAPEFILE_DIR + 'f_test_read_shapefile_01'

    # encoding
    encoding = 'utf-8'

    # 水平、垂直方向精度を定義
    zoom = 1

    # shapefile作成用の座標配列取得
    fan_point_list_01 = geographic_shape_point_data_01[0][0]
    fan_point_type_01 = geographic_shape_point_data_01[1][0]
    fan_point_list_02 = geographic_shape_point_data_01[0][1]
    fan_point_type_02 = geographic_shape_point_data_01[1][1]

    fan_point_list_03 = geographic_shape_point_data_02[0][0]
    fan_point_type_03 = geographic_shape_point_data_02[1][0]
    fan_point_list_04 = geographic_shape_point_data_02[0][1]
    fan_point_type_04 = geographic_shape_point_data_02[1][1]

    # shapefile生成
    w = sf.Writer(filepath)
    w.field('name', 'C')
    w.field('type', 'C')
    w.multipatch(
            [fan_point_list_01, fan_point_list_02],
            partTypes=[TRIANGLE_FAN, TRIANGLE_FAN])
    w.record('record1', 'FAN')
    w.multipatch(
            [fan_point_list_03, fan_point_list_04],
            partTypes=[TRIANGLE_FAN, TRIANGLE_FAN])
    w.record('record2', 'FAN')
    w.close()

    # 期待値取得
    # ペイロードの期待値を定義
    shapefile_record_list = [['record1', 'FAN'], ['record2', 'FAN']]

    # fieldの期待値を定義
    shapefile_field_list = [['name', 'C', 50, 0], ['type', 'C', 50, 0]]

    # TRIANGLE_FANのSpatialPointオブジェクトを生成
    spatial_fan_point_list_01 = _get_spatial_point_objects(fan_point_list_01)
    spatial_fan_point_list_02 = _get_spatial_point_objects(fan_point_list_02)
    spatial_fan_point_list_03 = _get_spatial_point_objects(fan_point_list_03)
    spatial_fan_point_list_04 = _get_spatial_point_objects(fan_point_list_04)

    # SpatialTriangleオブジェクト生成
    spatial_triangle_list_01 = \
        _shaping_point_to_triangle(
                spatial_fan_point_list_01, fan_point_type_01)
    spatial_triangle_list_02 = \
        _shaping_point_to_triangle(
                spatial_fan_point_list_02, fan_point_type_02)
    spatial_triangle_list_03 = \
        _shaping_point_to_triangle(
                spatial_fan_point_list_03, fan_point_type_03)
    spatial_triangle_list_04 = \
        _shaping_point_to_triangle(
                spatial_fan_point_list_04, fan_point_type_04)

    # 空間ID取得対象のSpatialTriangleを格納した配列を定義
    triangle_list_01 = spatial_triangle_list_01 + spatial_triangle_list_02
    triangle_list_02 = spatial_triangle_list_03 + spatial_triangle_list_04

    # 空間IDの期待値を定義
    shapefile_spatial_ids_01 = f_get_spatial_ids_on_polygons(
            triangle_list_01, [], zoom)
    shapefile_spatial_ids_02 = f_get_spatial_ids_on_polygons(
            triangle_list_02, [], zoom)
    shapefile_spatial_id_list = [
            shapefile_spatial_ids_01,
            shapefile_spatial_ids_02]

    # 試験実施
    # 戻り値取得
    param_return = f_read_shapefile(
            filepath, encoding, zoom)

    # 戻り値確認
    loop_cnt = 0

    for ids, field, records in param_return:
        # APIから取得した空間IDは順不同のため比較前にソートし、格納された空間IDが一致していることを確認する。
        param_assert_ids = sorted(shapefile_spatial_id_list[loop_cnt])
        param_return_ids = sorted(ids)

        # 戻り値確認 - 空間ID, フィールド, ペイロード
        assert param_assert_ids == param_return_ids
        assert shapefile_field_list == field
        assert shapefile_record_list[loop_cnt] == records

        loop_cnt += 1


def test_f_read_shapefile_02():
    """ジェネレータメソッド呼び出し時にshapefileの読み込み失敗、不正なファイルパス指定
    + 試験詳細
      - 入力条件
        - 存在しないファイルのパスを読み込み対象のファイルパスに指定
      - 確認内容
        - shapefile読み込み失敗の例外が発生すること
    """
    # 試験用データの定義
    # 試験用shalefileパス
    filepath = SHAPEFILE_DIR + 'test_f_read_shapefile_02_test'

    # encoding
    encoding = 'utf-8'

    # 水平、垂直方向精度を定義
    zoom = 10

    # shapefileに格納するデータの座標系
    shapefile_crs = 4326

    # 期待値取得
    # 例外取得
    exception_assert = SpatialIdError('SHAPEFILE_READ_ERROR')

    # 試験実施
    # 戻り値取得
    param_return = f_read_shapefile(
            filepath, encoding, zoom, shapefile_crs)

    # 例外発生確認
    with pytest.raises(SpatialIdError) as e:
        # Generator内での例外発生を検知するため、Generatorの要素を参照
        param_return.__next__()

    # 例外メッセージ確認
    assert str(exception_assert) == str(e.value)


def test_f_read_shapefile_03(projected_shape_point_data_01):
    """ジェネレータメソッド呼び出し時に座標系の変換失敗、入力された座標と座標系が対応していない
    + 試験詳細
      - 入力条件
        - 読み込み対象は投影座標系のshapefile
        - 参照座標系に地理座標系を入力
      - 確認内容
        - 入力チェックエラーの例外が発生すること
    """
    # 試験用データの定義
    # 試験用shalefileパス
    filepath = SHAPEFILE_DIR + 'test_f_read_shapefile_03'

    # encoding
    encoding = 'utf-8'

    # 精度を定義
    zoom = 10

    # shapefileに格納したデータに対応しない座標系
    shapefile_crs = 4326

    # shapefile作成用の座標配列取得
    strip_point_list_01 = projected_shape_point_data_01[0][0]
    strip_point_list_02 = projected_shape_point_data_01[0][1]
    fan_point_list_01 = projected_shape_point_data_01[0][2]

    # shapefile生成
    w = sf.Writer(filepath)
    w.field('name', 'C')
    w.field('type', 'C')
    w.multipatch(
            [strip_point_list_01, strip_point_list_02, fan_point_list_01],
            partTypes=[TRIANGLE_STRIP, TRIANGLE_STRIP, TRIANGLE_FAN])
    w.record('record1', 'STRIP, FAN')
    w.close()

    # 期待値取得
    # 例外取得
    exception_assert = SpatialIdError('VALUE_CONVERT_ERROR')

    # 試験実施
    # 戻り値取得
    param_return = f_read_shapefile(
            filepath, encoding, zoom, shapefile_crs)

    # 例外発生確認
    with pytest.raises(SpatialIdError) as e:
        # Generator内での例外発生を検知するため、Generatorの要素を参照
        param_return.__next__()

    # 例外メッセージ確認
    assert str(exception_assert) == str(e.value)


def test_f_read_shapefile_04(mocker):
    """ジェネレータメソッド呼び出し時、その他例外の発生
     + 試験詳細
       - 試験条件
         - 通常発生しない例外パターンのためモックを、使用して例外を発生させる
       - 確認内容
         - その他例外が発生すること
    """
    # 試験用データの定義
    # 試験用shalefileパス
    filepath = SHAPEFILE_DIR + 'test_f_read_shapefile_04'

    # encoding
    encoding = 'utf-8'

    # 精度を定義
    zoom = 10

    # shapefileに格納するデータの座標系
    shapefile_crs = 4326

    # Pointの座標を格納したリストを定義
    strip_point_01 = (139.051685, 45.060433, 150.5)
    strip_point_02 = (140.051685, 46.060433, 151.5)
    strip_point_03 = (141.051685, 47.060433, 152.5)
    strip_point_04 = (142.051685, 48.060433, 151.5)

    # shapefile作成用の座標配列
    strip_point_list = [
            strip_point_01,
            strip_point_02,
            strip_point_03,
            strip_point_04
            ]

    # shapefile生成
    w = sf.Writer(filepath)
    w.field('name', 'C')
    w.field('type', 'C')
    w.multipatch(
            [strip_point_list],
            partTypes=[TRIANGLE_STRIP])
    w.record('record1', 'STRIP')
    w.close()

    # 期待値取得
    # 例外取得
    exception_assert = SpatialIdError('OTHER_ERROR')

    mocker.patch(
            'SpatialId.io.shapefile._get_shapefile_spatial_id_and_record').\
        side_effect = Exception('予期せぬ例外が発生')

    # 試験実施
    # 戻り値取得
    param_return = f_read_shapefile(
            filepath, encoding, zoom, shapefile_crs)

    with pytest.raises(SpatialIdError) as e:
        # Generator内での例外発生を検知するため、Generatorの要素を参照
        param_return.__next__()

    # 例外メッセージ確認
    assert str(exception_assert) == str(e.value)


def test_read_shapefile_01(
        geographic_shape_point_data_01,
        geographic_shape_point_data_02
):
    """Shapefileから1レコード単位で情報を取得するジェネレータ取得、shapefileレコード数2、CSR入力なし
    + 試験詳細
      - 入力条件
        - 2レコードが格納されているshapefileを読み込み
        - 読み込み対象ファイル拡張子無し
        - CSR入力なし
      - 確認内容
        - ジェネレータが返却され、空間ID、フィールド名、ペイロードを1レコード毎に参照可能であること
    """
    # 試験用データの定義
    # 試験用shalefileパス
    filepath = SHAPEFILE_DIR + 'test_read_shapefile_01'

    # encoding
    encoding = 'utf-8'

    # 水平、垂直方向精度を定義
    h_zoom = 1
    v_zoom = 1

    # shapefile作成用の座標配列取得
    fan_point_list_01 = geographic_shape_point_data_01[0][0]
    fan_point_type_01 = geographic_shape_point_data_01[1][0]
    fan_point_list_02 = geographic_shape_point_data_01[0][1]
    fan_point_type_02 = geographic_shape_point_data_01[1][1]

    fan_point_list_03 = geographic_shape_point_data_02[0][0]
    fan_point_type_03 = geographic_shape_point_data_02[1][0]
    fan_point_list_04 = geographic_shape_point_data_02[0][1]
    fan_point_type_04 = geographic_shape_point_data_02[1][1]

    # shapefile生成
    w = sf.Writer(filepath)
    w.field('name', 'C')
    w.field('type', 'C')
    w.multipatch(
            [fan_point_list_01, fan_point_list_02],
            partTypes=[TRIANGLE_FAN, TRIANGLE_FAN])
    w.record('record1', 'FAN')
    w.multipatch(
            [fan_point_list_03, fan_point_list_04],
            partTypes=[TRIANGLE_FAN, TRIANGLE_FAN])
    w.record('record2', 'FAN')
    w.close()

    # 期待値取得
    # ペイロードの期待値を定義
    shapefile_record_list = [['record1', 'FAN'], ['record2', 'FAN']]

    # fieldの期待値を定義
    shapefile_field_list = [['name', 'C', 50, 0], ['type', 'C', 50, 0]]

    # TRIANGLE_FANのSpatialPointオブジェクトを生成
    spatial_fan_point_list_01 = _get_spatial_point_objects(fan_point_list_01)
    spatial_fan_point_list_02 = _get_spatial_point_objects(fan_point_list_02)
    spatial_fan_point_list_03 = _get_spatial_point_objects(fan_point_list_03)
    spatial_fan_point_list_04 = _get_spatial_point_objects(fan_point_list_04)

    # SpatialTriangleオブジェクト生成
    spatial_triangle_list_01 = \
        _shaping_point_to_triangle(
                spatial_fan_point_list_01, fan_point_type_01)
    spatial_triangle_list_02 = \
        _shaping_point_to_triangle(
                spatial_fan_point_list_02, fan_point_type_02)
    spatial_triangle_list_03 = \
        _shaping_point_to_triangle(
                spatial_fan_point_list_03, fan_point_type_03)
    spatial_triangle_list_04 = \
        _shaping_point_to_triangle(
                spatial_fan_point_list_04, fan_point_type_04)

    # 空間ID取得対象のSpatialTriangleを格納した配列を定義
    triangle_list_01 = spatial_triangle_list_01 + spatial_triangle_list_02
    triangle_list_02 = spatial_triangle_list_03 + spatial_triangle_list_04

    # 空間IDの期待値を定義
    shapefile_spatial_ids_01 = get_spatial_ids_on_polygons(
            triangle_list_01, [], h_zoom, v_zoom)
    shapefile_spatial_ids_02 = get_spatial_ids_on_polygons(
            triangle_list_02, [], h_zoom, v_zoom)
    shapefile_spatial_id_list = [
            shapefile_spatial_ids_01,
            shapefile_spatial_ids_02]

    # 試験実施
    # 戻り値取得
    param_return = read_shapefile(
            filepath, encoding, h_zoom, v_zoom)

    # 戻り値確認
    loop_cnt = 0

    for ids, field, records in param_return:
        # APIから取得した空間IDは順不同のため比較前にソートし、格納された空間IDが一致していることを確認する。
        param_assert_ids = sorted(shapefile_spatial_id_list[loop_cnt])
        param_return_ids = sorted(ids)

        # 戻り値確認 - 空間ID, フィールド, ペイロード
        assert param_assert_ids == param_return_ids
        assert shapefile_field_list == field
        assert shapefile_record_list[loop_cnt] == records

        loop_cnt += 1


def test_read_shapefile_02():
    """ジェネレータメソッド呼び出し時にshapefileの読み込み失敗、不正なファイルパス指定
    + 試験詳細
      - 入力条件
        - 存在しないファイルのパスを読み込み対象のファイルパスに指定
      - 確認内容
        - shapefile読み込み失敗の例外が発生すること
    """
    # 試験用データの定義
    # 試験用shalefileパス
    filepath = SHAPEFILE_DIR + 'test_read_shapefile_02'

    # encoding
    encoding = 'utf-8'

    # 水平、垂直方向精度を定義
    h_zoom = 1
    v_zoom = 1

    # shapefileに格納するデータの座標系
    shapefile_crs = 4326

    # 期待値取得
    # 例外取得
    exception_assert = SpatialIdError('SHAPEFILE_READ_ERROR')

    # 試験実施
    # 戻り値取得
    param_return = read_shapefile(
            filepath, encoding, h_zoom, v_zoom, shapefile_crs)

    # 例外発生確認
    with pytest.raises(SpatialIdError) as e:
        # Generator内での例外発生を検知するため、Generatorの要素を参照
        param_return.__next__()

    # 例外メッセージ確認
    assert str(exception_assert) == str(e.value)


def test_read_shapefile_03(
        geographic_shape_point_data_01
):
    """ジェネレータメソッド呼び出し時に全角文字を含むフィールド名を持つshapefile読み込み、encoding指定、shapefileレコード数1
    + 試験詳細
      - 入力条件
        - encodingを指定(shift_jis)
        - フィールド名に全角文字を含むshapefileを読み込み
        - 1レコードが格納されているshapefileを読み込み
        - CSRにデフォルト値と同じ地理座標系(4326)を入力
      - 確認内容
        - ジェネレータが返却され、空間ID、フィールド名、ペイロードを1レコード毎に参照可能であること
    """
    # 試験用データの定義
    # 試験用shalefileパス
    filepath = SHAPEFILE_DIR + 'test_read_shapefile_03'

    # encoding
    encoding = 'shift_jis'

    # 水平、垂直方向精度を定義
    h_zoom = 3
    v_zoom = 3

    # shapefileに格納するデータの座標系
    shapefile_crs = 4326

    # shapefile作成用の座標配列取得
    fan_point_list_01 = geographic_shape_point_data_01[0][0]
    fan_point_type_01 = geographic_shape_point_data_01[1][0]
    fan_point_list_02 = geographic_shape_point_data_01[0][1]
    fan_point_type_02 = geographic_shape_point_data_01[1][1]

    # shapefile生成
    w = sf.Writer(filepath, encoding="shift_jis")
    w.field('名前', 'C')
    w.field('type', 'C')
    w.multipatch(
            [fan_point_list_01, fan_point_list_02],
            partTypes=[TRIANGLE_FAN, TRIANGLE_FAN])
    w.record('レコード1', 'FAN')
    w.close()

    # 期待値取得
    # ペイロードの期待値を定義
    shapefile_record_list = [['レコード1', 'FAN']]

    # fieldの期待値を定義
    shapefile_field_list = [['名前', 'C', 50, 0], ['type', 'C', 50, 0]]

    # TRIANGLE_FANのSpatialPointオブジェクトを生成
    spatial_fan_point_list_01 = _get_spatial_point_objects(fan_point_list_01)
    spatial_fan_point_list_02 = _get_spatial_point_objects(fan_point_list_02)

    # SpatialTriangleオブジェクト生成
    spatial_triangle_list_01 = \
        _shaping_point_to_triangle(
                spatial_fan_point_list_01, fan_point_type_01)
    spatial_triangle_list_02 = \
        _shaping_point_to_triangle(
                spatial_fan_point_list_02, fan_point_type_02)

    # 空間ID取得対象のSpatialTriangleを格納した配列を定義
    triangle_list = spatial_triangle_list_01 + spatial_triangle_list_02

    # 空間IDの期待値を定義
    shapefile_spatial_id_list = [get_spatial_ids_on_polygons(
              triangle_list, [], h_zoom, v_zoom)]

    # 試験実施
    # 戻り値取得
    param_return = read_shapefile(
            filepath, encoding, h_zoom, v_zoom, shapefile_crs)

    # 戻り値確認
    loop_cnt = 0

    for ids, field, records in param_return:
        # APIから取得した空間IDは順不同のため比較前にソートし、格納された空間IDが一致していることを確認する。
        param_assert_ids = sorted(shapefile_spatial_id_list[loop_cnt])
        param_return_ids = sorted(ids)

        # 戻り値確認 - 空間ID, フィールド, ペイロード
        assert param_assert_ids == param_return_ids
        assert shapefile_field_list == field
        assert shapefile_record_list[loop_cnt] == records

        loop_cnt += 1


def test_read_shapefile_04():
    """ジェネレータメソッド呼び出し時にshapefileの読み込み失敗、不正なencoding指定
    + 試験詳細
      - 入力条件
        - 読み込み対象のshapefileに対応しないencodingを入力
      - 確認内容
        - shapefile読み込み失敗の例外が発生すること
    """
    # 試験用データの定義
    # 試験用shalefileパス
    filepath = SHAPEFILE_DIR + 'test_read_shapefile_04'

    # encoding
    encoding = 'utf-8'

    # 水平、垂直方向精度を定義
    h_zoom = 1
    v_zoom = 1

    # shapefileに格納するデータの座標系
    shapefile_crs = 4326

    # Pointの座標を格納したリストを定義
    strip_point_01 = (139.051685, 45.060433, 150.5)
    strip_point_02 = (140.051685, 46.060433, 151.5)
    strip_point_03 = (141.051685, 47.060433, 152.5)
    strip_point_04 = (142.051685, 48.060433, 151.5)

    # shapefile作成用の座標配列
    strip_point_list = [
            strip_point_01,
            strip_point_02,
            strip_point_03,
            strip_point_04
            ]

    # shapefile生成
    w = sf.Writer(filepath, encoding="shift_jis")
    w.field('名前', 'C')
    w.field('type', 'C')
    w.multipatch(
            [strip_point_list],
            partTypes=[TRIANGLE_STRIP])
    w.record('レコード1', 'STRIP')
    w.close()

    # 期待値取得
    # 例外取得
    exception_assert = SpatialIdError('SHAPEFILE_READ_ERROR')

    # 試験実施
    # 戻り値取得
    param_return = read_shapefile(
            filepath, encoding, h_zoom, v_zoom, shapefile_crs)

    # 例外発生確認
    with pytest.raises(SpatialIdError) as e:
        # Generator内での例外発生を検知するため、Generatorの要素を参照
        param_return.__next__()

    # 例外メッセージ確認
    assert str(exception_assert) == str(e.value)


def test_read_shapefile_05():
    """ジェネレータメソッド呼び出し時にshapefileの読み込み失敗、encoding空文字
    + 試験詳細
      - 入力条件
        - encoding空文字を入力
      - 確認内容
        - shapefile読み込み失敗の例外が発生すること
    """
    # 試験用データの定義
    # 試験用shalefileパス
    filepath = SHAPEFILE_DIR + 'test_read_shapefile_05'

    # encoding
    encoding = ''

    # 水平、垂直方向精度を定義
    h_zoom = 1
    v_zoom = 1

    # shapefileに格納するデータの座標系
    shapefile_crs = 4326

    # Pointの座標を格納したリストを定義
    strip_point_01 = (139.051685, 45.060433, 150.5)
    strip_point_02 = (140.051685, 46.060433, 151.5)
    strip_point_03 = (141.051685, 47.060433, 152.5)
    strip_point_04 = (142.051685, 48.060433, 151.5)

    # shapefile作成用の座標配列
    strip_point_list = [
            strip_point_01,
            strip_point_02,
            strip_point_03,
            strip_point_04
            ]

    # shapefile生成
    w = sf.Writer(filepath)
    w.field('name', 'C')
    w.field('type', 'C')
    w.multipatch(
            [strip_point_list],
            partTypes=[TRIANGLE_STRIP])
    w.record('record1', 'STRIP')
    w.close()

    # 期待値取得
    # 例外取得
    exception_assert = SpatialIdError('SHAPEFILE_READ_ERROR')

    # 試験実施
    # 戻り値取得
    param_return = read_shapefile(
            filepath, encoding, h_zoom, v_zoom, shapefile_crs)

    # 例外発生確認
    with pytest.raises(SpatialIdError) as e:
        # Generator内での例外発生を検知するため、Generatorの要素を参照
        param_return.__next__()

    # 例外メッセージ確認
    assert str(exception_assert) == str(e.value)


def test_read_shapefile_06(projected_shape_point_data_01):
    """ジェネレータメソッド呼び出し時に座標系の変換失敗、入力された座標と座標系が対応していない
    + 試験詳細
      - 入力条件
        - 読み込み対象は投影座標系のshapefile
        - 参照座標系に地理座標系を入力
      - 確認内容
        - 入力チェックエラーの例外が発生すること
    """
    # 試験用データの定義
    # 試験用shalefileパス
    filepath = SHAPEFILE_DIR + 'test_read_shapefile_06'

    # encoding
    encoding = 'utf-8'

    # 水平、垂直方向精度を定義
    h_zoom = 1
    v_zoom = 1

    # shapefileに格納したデータに対応しない座標系
    shapefile_crs = 4326

    # shapefile作成用の座標配列取得
    strip_point_list_01 = projected_shape_point_data_01[0][0]
    strip_point_list_02 = projected_shape_point_data_01[0][1]
    fan_point_list_01 = projected_shape_point_data_01[0][2]

    # shapefile生成
    w = sf.Writer(filepath)
    w.field('name', 'C')
    w.field('type', 'C')
    w.multipatch(
            [strip_point_list_01, strip_point_list_02, fan_point_list_01],
            partTypes=[TRIANGLE_STRIP, TRIANGLE_STRIP, TRIANGLE_FAN])
    w.record('record1', 'STRIP, FAN')
    w.close()

    # 期待値取得
    # 例外取得
    exception_assert = SpatialIdError('VALUE_CONVERT_ERROR')

    # 試験実施
    # 戻り値取得
    param_return = read_shapefile(
            filepath, encoding, h_zoom, v_zoom, shapefile_crs)

    # 例外発生確認
    with pytest.raises(SpatialIdError) as e:
        # Generator内での例外発生を検知するため、Generatorの要素を参照
        param_return.__next__()

    # 例外メッセージ確認
    assert str(exception_assert) == str(e.value)


def test_read_shapefile_07():
    """ジェネレータメソッド呼び出し時に内部処理の例外発生を検知
    + 試験詳細
      - 入力条件
        - サポート対象外のshapeTypeのshapefile読み込み
      - 確認内容
        - shapeType不正の例外が発生すること
    """
    # 試験用データの定義
    # 試験用shalefileパス
    filepath = SHAPEFILE_DIR + 'test_read_shapefile_07'

    # encoding
    encoding = 'utf-8'

    # 水平、垂直方向精度を定義
    h_zoom = 1
    v_zoom = 1

    # shapefileに格納するデータの座標系
    shapefile_crs = 4326

    # Pointの座標を格納したリストを定義
    point_01 = (139.051685, 45.060433, 150.5)
    point_02 = (140.051685, 46.060433, 151.5)
    point_03 = (141.051685, 47.060433, 152.5)

    # shapefile作成用の座標配列
    point_list = [
            point_01,
            point_02,
            point_03
            ]

    # shapefile生成
    w = sf.Writer(filepath)
    w.field('name', 'C')
    w.field('type', 'C')
    w.multipointz(point_list)
    w.record('record1', 'MPOINTZ')
    w.close()

    # 期待値取得
    # 例外取得
    exception_assert = SpatialIdError('SHAPEFILE_SHAPETYPE_ERROR')

    # 試験実施
    # 戻り値取得
    param_return = read_shapefile(
            filepath, encoding, h_zoom, v_zoom, shapefile_crs)

    # 例外発生確認
    with pytest.raises(SpatialIdError) as e:
        # Generator内での例外発生を検知するため、Generatorの要素を参照
        param_return.__next__()

    # 例外メッセージ確認
    assert str(exception_assert) == str(e.value)


def test_read_shapefile_08(mocker):
    """ジェネレータメソッド呼び出し時、その他例外の発生
     + 試験詳細
       - 試験条件
         - 通常発生しない例外パターンのためモックを、使用して例外を発生させる
       - 確認内容
         - その他例外が発生すること
    """
    # 試験用データの定義
    # 試験用shalefileパス
    filepath = SHAPEFILE_DIR + 'test_read_shapefile_08'

    # encoding
    encoding = 'utf-8'

    # 水平、垂直方向精度を定義
    h_zoom = 10
    v_zoom = 10

    # shapefileに格納するデータの座標系
    shapefile_crs = 4326

    # Pointの座標を格納したリストを定義
    strip_point_01 = (139.051685, 45.060433, 150.5)
    strip_point_02 = (140.051685, 46.060433, 151.5)
    strip_point_03 = (141.051685, 47.060433, 152.5)
    strip_point_04 = (142.051685, 48.060433, 151.5)

    # shapefile作成用の座標配列
    strip_point_list = [
            strip_point_01,
            strip_point_02,
            strip_point_03,
            strip_point_04
            ]

    # shapefile生成
    w = sf.Writer(filepath)
    w.field('name', 'C')
    w.field('type', 'C')
    w.multipatch(
            [strip_point_list],
            partTypes=[TRIANGLE_STRIP])
    w.record('record1', 'STRIP')
    w.close()

    # 期待値取得
    # 例外取得
    exception_assert = SpatialIdError('OTHER_ERROR')

    mocker.patch(
            'SpatialId.io.shapefile._get_shapefile_spatial_id_and_record').\
        side_effect = Exception('予期せぬ例外が発生')

    # 試験実施
    # 戻り値取得
    param_return = read_shapefile(
            filepath, encoding, h_zoom, v_zoom, shapefile_crs)

    with pytest.raises(SpatialIdError) as e:
        # Generator内での例外発生を検知するため、Generatorの要素を参照
        param_return.__next__()

    # 例外メッセージ確認
    assert str(exception_assert) == str(e.value)


def test_read_shapefile_09(geographic_shape_point_data_01):
    """強化試験: 1レコード単位で情報を取得するジェネレータ取得、読み込み対象がzipファイル
    + 試験詳細
      - 入力条件
        - zip形式の圧縮ファイルを読み込み対象のshapefileとする。
      - 確認内容
        - エラーとならず、ジェネレータの取得に成功すること
    """
    # 試験用データの定義
    # 試験用shalefileパス
    filepath = SHAPEFILE_DIR + 'test_read_shapefile_09'

    # encoding
    encoding = 'utf-8'

    # 水平、垂直方向精度を定義
    h_zoom = 1
    v_zoom = 1

    # shapefileに格納するデータの座標系
    shapefile_crs = 4326

    # shapefile作成用の座標配列取得
    fan_point_list_01 = geographic_shape_point_data_01[0][0]
    fan_point_type_01 = geographic_shape_point_data_01[1][0]
    fan_point_list_02 = geographic_shape_point_data_01[0][1]
    fan_point_type_02 = geographic_shape_point_data_01[1][1]

    # shapefile生成
    w = sf.Writer(filepath)
    w.field('name', 'C')
    w.field('type', 'C')
    w.multipatch(
            [fan_point_list_01, fan_point_list_02],
            partTypes=[TRIANGLE_FAN, TRIANGLE_FAN])
    w.record('record1', 'FAN')
    w.close()

    # zipファイル作成
    dbf_path = filepath + '.dbf'
    shp_path = filepath + '.shp'
    shx_path = filepath + '.shx'
    zip_path = filepath + '.zip'
    zp = zipfile.ZipFile(zip_path, "w")
    zp.write(dbf_path, compress_type=zipfile.ZIP_DEFLATED)
    zp.write(shp_path, compress_type=zipfile.ZIP_DEFLATED)
    zp.write(shx_path, compress_type=zipfile.ZIP_DEFLATED)
    zp.close()

    # 圧縮済みのshapefileを削除
    os.remove(dbf_path)
    os.remove(shp_path)
    os.remove(shx_path)

    # 期待値取得
    # ペイロードの期待値を定義
    shapefile_record_list = [['record1', 'FAN']]

    # fieldの期待値を定義
    shapefile_field_list = [['name', 'C', 50, 0], ['type', 'C', 50, 0]]

    # TRIANGLE_FANのSpatialPointオブジェクトを生成
    spatial_fan_point_list_01 = _get_spatial_point_objects(fan_point_list_01)
    spatial_fan_point_list_02 = _get_spatial_point_objects(fan_point_list_02)

    # SpatialTriangleオブジェクト生成
    spatial_triangle_list_01 = \
        _shaping_point_to_triangle(
                spatial_fan_point_list_01, fan_point_type_01)
    spatial_triangle_list_02 = \
        _shaping_point_to_triangle(
                spatial_fan_point_list_02, fan_point_type_02)

    # 空間ID取得対象のSpatialTriangleを格納した配列を定義
    triangle_list = spatial_triangle_list_01 + spatial_triangle_list_02

    # 空間IDの期待値を定義
    shapefile_spatial_id_list = [get_spatial_ids_on_polygons(
              triangle_list, [], h_zoom, v_zoom)]

    # 試験実施
    # 戻り値取得 - 読み込み対象にzipファイル指定
    param_return = read_shapefile(
            zip_path, encoding, h_zoom, v_zoom, shapefile_crs)

    # 戻り値確認
    loop_cnt = 0

    for ids, field, records in param_return:
        # APIから取得した空間IDは順不同のため比較前にソートし、格納された空間IDが一致していることを確認する。
        param_assert_ids = sorted(shapefile_spatial_id_list[loop_cnt])
        param_return_ids = sorted(ids)

        # 戻り値確認 - 空間ID, フィールド, ペイロード
        assert param_assert_ids == param_return_ids
        assert shapefile_field_list == field
        assert shapefile_record_list[loop_cnt] == records

        loop_cnt += 1


def test_read_shapefile_10(geographic_shape_point_data_01):
    """強化試験：Shapefileから1レコード単位で情報を取得するジェネレータ取得、読み込みファイル拡張子が.shp
    + 試験詳細
      - 入力条件
        - 読み込みファイル拡張子が.shp
      - 確認内容
        - ジェネレータが返却され、空間ID、フィールド名、ペイロードを1レコード毎に参照可能であること
    """
    # 試験用データの定義
    # 試験用shalefileパス
    filepath = SHAPEFILE_DIR + 'test_read_shapefile_10'

    # encoding
    encoding = 'utf-8'

    # 水平、垂直方向精度を定義
    h_zoom = 1
    v_zoom = 1

    # shapefileに格納するデータの座標系
    shapefile_crs = 4326

    # shapefile作成用の座標配列取得
    fan_point_list_01 = geographic_shape_point_data_01[0][0]
    fan_point_type_01 = geographic_shape_point_data_01[1][0]
    fan_point_list_02 = geographic_shape_point_data_01[0][1]
    fan_point_type_02 = geographic_shape_point_data_01[1][1]

    # shapefile生成
    w = sf.Writer(filepath)
    w.field('name', 'C')
    w.field('type', 'C')
    w.multipatch(
            [fan_point_list_01, fan_point_list_02],
            partTypes=[TRIANGLE_FAN, TRIANGLE_FAN])
    w.record('record1', 'FAN')
    w.close()

    # 読み込み対象の.shpファイルパス定義
    read_path = filepath + '.shp'

    # 期待値取得
    # ペイロードの期待値を定義
    shapefile_record_list = [['record1', 'FAN']]

    # fieldの期待値を定義
    shapefile_field_list = [['name', 'C', 50, 0], ['type', 'C', 50, 0]]

    # TRIANGLE_FANのSpatialPointオブジェクトを生成
    spatial_fan_point_list_01 = _get_spatial_point_objects(fan_point_list_01)
    spatial_fan_point_list_02 = _get_spatial_point_objects(fan_point_list_02)

    # SpatialTriangleオブジェクト生成
    spatial_triangle_list_01 = \
        _shaping_point_to_triangle(
                spatial_fan_point_list_01, fan_point_type_01)
    spatial_triangle_list_02 = \
        _shaping_point_to_triangle(
                spatial_fan_point_list_02, fan_point_type_02)

    # 空間ID取得対象のSpatialTriangleを格納した配列を定義
    triangle_list = spatial_triangle_list_01 + spatial_triangle_list_02

    # 空間IDの期待値を定義
    shapefile_spatial_id_list = [get_spatial_ids_on_polygons(
              triangle_list, [], h_zoom, v_zoom)]

    # 試験実施
    # 戻り値取得
    param_return = read_shapefile(
            read_path, encoding, h_zoom, v_zoom, shapefile_crs)

    # 戻り値確認
    loop_cnt = 0

    for ids, field, records in param_return:
        # APIから取得した空間IDは順不同のため比較前にソートし、格納された空間IDが一致していることを確認する。
        param_assert_ids = sorted(shapefile_spatial_id_list[loop_cnt])
        param_return_ids = sorted(ids)

        # 戻り値確認 - 空間ID, フィールド, ペイロード
        assert param_assert_ids == param_return_ids
        assert shapefile_field_list == field
        assert shapefile_record_list[loop_cnt] == records

        loop_cnt += 1


def test_read_shapefile_11(geographic_shape_point_data_01):
    """強化試験：Shapefileから1レコード単位で情報を取得するジェネレータ取得、読み込みファイル拡張子が.dbf
    + 試験詳細
      - 入力条件
        - 読み込みファイル拡張子が.dbf
      - 確認内容
        - ジェネレータが返却され、空間ID、フィールド名、ペイロードを1レコード毎に参照可能であること
    """
    # 試験用データの定義
    # 試験用shalefileパス
    filepath = SHAPEFILE_DIR + 'test_read_shapefile_11'

    # encoding
    encoding = 'utf-8'

    # 水平、垂直方向精度を定義
    h_zoom = 1
    v_zoom = 1

    # shapefileに格納するデータの座標系
    shapefile_crs = 4326

    # shapefile作成用の座標配列取得
    fan_point_list_01 = geographic_shape_point_data_01[0][0]
    fan_point_type_01 = geographic_shape_point_data_01[1][0]
    fan_point_list_02 = geographic_shape_point_data_01[0][1]
    fan_point_type_02 = geographic_shape_point_data_01[1][1]

    # shapefile生成
    w = sf.Writer(filepath)
    w.field('name', 'C')
    w.field('type', 'C')
    w.multipatch(
            [fan_point_list_01, fan_point_list_02],
            partTypes=[TRIANGLE_FAN, TRIANGLE_FAN])
    w.record('record1', 'FAN')
    w.close()

    # 読み込み対象の.dbfファイルパス定義
    read_path = filepath + '.dbf'

    # 期待値取得
    # ペイロードの期待値を定義
    shapefile_record_list = [['record1', 'FAN']]

    # fieldの期待値を定義
    shapefile_field_list = [['name', 'C', 50, 0], ['type', 'C', 50, 0]]

    # TRIANGLE_FANのSpatialPointオブジェクトを生成
    spatial_fan_point_list_01 = _get_spatial_point_objects(fan_point_list_01)
    spatial_fan_point_list_02 = _get_spatial_point_objects(fan_point_list_02)

    # SpatialTriangleオブジェクト生成
    spatial_triangle_list_01 = \
        _shaping_point_to_triangle(
                spatial_fan_point_list_01, fan_point_type_01)
    spatial_triangle_list_02 = \
        _shaping_point_to_triangle(
                spatial_fan_point_list_02, fan_point_type_02)

    # 空間ID取得対象のSpatialTriangleを格納した配列を定義
    triangle_list = spatial_triangle_list_01 + spatial_triangle_list_02

    # 空間IDの期待値を定義
    shapefile_spatial_id_list = [get_spatial_ids_on_polygons(
              triangle_list, [], h_zoom, v_zoom)]

    # 試験実施
    # 戻り値取得
    param_return = read_shapefile(
            read_path, encoding, h_zoom, v_zoom, shapefile_crs)

    # 戻り値確認
    loop_cnt = 0

    for ids, field, records in param_return:
        # APIから取得した空間IDは順不同のため比較前にソートし、格納された空間IDが一致していることを確認する。
        param_assert_ids = sorted(shapefile_spatial_id_list[loop_cnt])
        param_return_ids = sorted(ids)

        # 戻り値確認 - 空間ID, フィールド, ペイロード
        assert param_assert_ids == param_return_ids
        assert shapefile_field_list == field
        assert shapefile_record_list[loop_cnt] == records

        loop_cnt += 1


def test_read_shapefile_12(geographic_shape_point_data_01):
    """強化試験：Shapefileから1レコード単位で情報を取得するジェネレータ取得、.shpのみが存在しない場合
    + 試験詳細
      - 入力条件
        - 読み込み対象のshapefileの内、".shp"のみが存在しない
      - 確認内容
        - shapefile読み込み失敗の例外が発生すること
    """
    # 試験用データの定義
    # 試験用shalefileパス
    filepath = SHAPEFILE_DIR + 'test_read_shapefile_12'

    # encoding
    encoding = 'utf-8'

    # 水平、垂直方向精度を定義
    h_zoom = 1
    v_zoom = 1

    # shapefileに格納するデータの座標系
    shapefile_crs = 4326

    # shapefile作成用の座標配列取得
    fan_point_list_01 = geographic_shape_point_data_01[0][0]
    fan_point_list_02 = geographic_shape_point_data_01[0][1]

    # shapefile生成
    w = sf.Writer(filepath)
    w.field('name', 'C')
    w.field('type', 'C')
    w.multipatch(
            [fan_point_list_01, fan_point_list_02],
            partTypes=[TRIANGLE_FAN, TRIANGLE_FAN])
    w.record('record1', 'FAN')
    w.close()

    # 読み込み対象のshapefileの内.shpファイルを削除
    remove_shapefile_path = filepath + '.shp'
    os.remove(remove_shapefile_path)

    # 期待値取得
    # 例外取得
    exception_assert = SpatialIdError('SHAPEFILE_READ_ERROR')

    # 試験実施
    # 戻り値取得
    param_return = read_shapefile(
            filepath, encoding, h_zoom, v_zoom, shapefile_crs)

    # 例外発生確認
    with pytest.raises(SpatialIdError) as e:
        # Generator内での例外発生を検知するため、Generatorの要素を参照
        param_return.__next__()

    # 例外メッセージ確認
    assert str(exception_assert) == str(e.value)


def test_read_shapefile_13(geographic_shape_point_data_01):
    """強化試験：Shapefileから1レコード単位で情報を取得するジェネレータ取得、.dbfのみが存在しない場合
    + 試験詳細
      - 入力条件
        - 読み込み対象のshapefileの内、".dbf"のみが存在しない
      - 確認内容
        - shapefile読み込み失敗の例外が発生すること
    """
    # 試験用データの定義
    # 試験用shalefileパス
    filepath = SHAPEFILE_DIR + 'test_read_shapefile_13'

    # encoding
    encoding = 'utf-8'

    # 水平、垂直方向精度を定義
    h_zoom = 1
    v_zoom = 1

    # shapefileに格納するデータの座標系
    shapefile_crs = 4326

    # shapefile作成用の座標配列取得
    fan_point_list_01 = geographic_shape_point_data_01[0][0]
    fan_point_list_02 = geographic_shape_point_data_01[0][1]

    # shapefile生成
    w = sf.Writer(filepath)
    w.field('name', 'C')
    w.field('type', 'C')
    w.multipatch(
            [fan_point_list_01, fan_point_list_02],
            partTypes=[TRIANGLE_FAN, TRIANGLE_FAN])
    w.record('record1', 'FAN')
    w.close()

    # 読み込み対象のshapefileの内.dbfファイルを削除
    remove_shapefile_path = filepath + '.dbf'
    os.remove(remove_shapefile_path)

    # 期待値取得
    # 例外取得
    exception_assert = SpatialIdError('SHAPEFILE_READ_ERROR')

    # 試験実施
    # 戻り値取得
    param_return = read_shapefile(
            filepath, encoding, h_zoom, v_zoom, shapefile_crs)

    # 例外発生確認
    with pytest.raises(SpatialIdError) as e:
        # Generator内での例外発生を検知するため、Generatorの要素を参照
        param_return.__next__()

    # 例外メッセージ確認
    assert str(exception_assert) == str(e.value)


def test_read_shapefile_14(geographic_shape_point_data_01):
    """強化試験：Shapefileから1レコード単位で情報を取得するジェネレータ取得、入力CRSが地理座標系
    + 試験詳細
      - 入力条件
        - ユーザ入力のCRSに地理座標系を入力
      - 確認内容
        - ジェネレータが返却され、空間ID、フィールド名、ペイロードを1レコード毎に参照可能であること
    """
    # 試験用データの定義
    # 試験用shalefileパス
    filepath = SHAPEFILE_DIR + 'test_read_shapefile_14'

    # encoding
    encoding = 'utf-8'

    # 水平、垂直方向精度を定義
    h_zoom = 10
    v_zoom = 10

    # shapefileに格納するデータの座標系
    shapefile_crs = 4612

    # shapefile作成用の座標配列取得
    fan_point_list_01 = geographic_shape_point_data_01[0][0]
    fan_point_type_01 = geographic_shape_point_data_01[1][0]
    fan_point_list_02 = geographic_shape_point_data_01[0][1]
    fan_point_type_02 = geographic_shape_point_data_01[1][1]

    # shapefile生成
    w = sf.Writer(filepath)
    w.field('name', 'C')
    w.field('type', 'C')
    w.multipatch(
            [fan_point_list_01, fan_point_list_02],
            partTypes=[TRIANGLE_FAN, TRIANGLE_FAN])
    w.record('record1', 'FAN')
    w.close()

    # 期待値取得
    # ペイロードの期待値を定義
    shapefile_record_list = [['record1', 'FAN']]

    # fieldの期待値を定義
    shapefile_field_list = [['name', 'C', 50, 0], ['type', 'C', 50, 0]]

    # TRIANGLE_FANのSpatialPointオブジェクトを生成
    spatial_fan_point_list_01 = _get_spatial_point_objects(fan_point_list_01)
    spatial_fan_point_list_02 = _get_spatial_point_objects(fan_point_list_02)

    # SpatialTriangleオブジェクト生成
    spatial_triangle_list_01 = \
        _shaping_point_to_triangle(
                spatial_fan_point_list_01, fan_point_type_01)
    spatial_triangle_list_02 = \
        _shaping_point_to_triangle(
                spatial_fan_point_list_02, fan_point_type_02)

    # 空間ID取得対象のSpatialTriangleを格納した配列を定義
    triangle_list = spatial_triangle_list_01 + spatial_triangle_list_02

    # 空間IDの期待値を定義
    shapefile_spatial_id_list = [get_spatial_ids_on_polygons(
              triangle_list, [], h_zoom, v_zoom)]

    # 試験実施
    # 戻り値取得
    param_return = read_shapefile(
            filepath, encoding, h_zoom, v_zoom, shapefile_crs)

    # 戻り値確認
    loop_cnt = 0

    for ids, field, records in param_return:
        # APIから取得した空間IDは順不同のため比較前にソートし、格納された空間IDが一致していることを確認する。
        param_assert_ids = sorted(shapefile_spatial_id_list[loop_cnt])
        param_return_ids = sorted(ids)

        # 戻り値確認 - 空間ID, フィールド, ペイロード
        assert param_assert_ids == param_return_ids
        assert shapefile_field_list == field
        assert shapefile_record_list[loop_cnt] == records

        loop_cnt += 1


def test_read_shapefile_15(projected_shape_point_data_01):
    """強化試験：Shapefileから1レコード単位で情報を取得するジェネレータ取得、入力CRSが投影座標系
    + 試験詳細
      - 入力条件
        - ユーザ入力のCRSに投影座標系を入力
      - 確認内容
        - ジェネレータが返却され、空間ID、フィールド名、ペイロードを1レコード毎に参照可能であること
    """
    # 試験用データの定義
    # 試験用shalefileパス
    filepath = SHAPEFILE_DIR + 'test_read_shapefile_15'

    # encoding
    encoding = 'utf-8'

    # 水平、垂直方向精度を定義
    h_zoom = 10
    v_zoom = 10

    # shapefileに格納するデータの座標系
    shapefile_crs = 6677

    # shapefile作成用の座標配列取得
    strip_point_list_01 = projected_shape_point_data_01[0][0]
    strip_point_type_01 = projected_shape_point_data_01[1][0]
    strip_point_list_02 = projected_shape_point_data_01[0][1]
    strip_point_type_02 = projected_shape_point_data_01[1][1]
    fan_point_list_01 = projected_shape_point_data_01[0][2]
    fan_point_type_01 = projected_shape_point_data_01[1][2]

    # shapefile生成
    w = sf.Writer(filepath)
    w.field('name', 'C')
    w.field('type', 'C')
    w.multipatch(
            [strip_point_list_01, strip_point_list_02, fan_point_list_01],
            partTypes=[TRIANGLE_STRIP, TRIANGLE_STRIP, TRIANGLE_FAN])
    w.record('record1', 'STRIP, FAN')
    w.close()

    # 期待値取得
    # ペイロードの期待値を定義
    shapefile_record_list = [['record1', 'STRIP, FAN']]

    # fieldの期待値を定義
    shapefile_field_list = [['name', 'C', 50, 0], ['type', 'C', 50, 0]]

    # TRIANGLE_STRIPのSpatialPointオブジェクトを生成
    spatial_strip_point_list_01 = \
        _get_spatial_point_objects(strip_point_list_01, shapefile_crs)
    spatial_strip_point_list_02 = \
        _get_spatial_point_objects(strip_point_list_02, shapefile_crs)

    # TRIANGLE_FANのSpatialPointオブジェクトを生成
    spatial_fan_point_list_01 = \
        _get_spatial_point_objects(fan_point_list_01, shapefile_crs)

    # SpatialTriangleオブジェクト生成
    spatial_triangle_list_01 = \
        _shaping_point_to_triangle(
                spatial_strip_point_list_01, strip_point_type_01)
    spatial_triangle_list_02 = \
        _shaping_point_to_triangle(
                spatial_strip_point_list_02, strip_point_type_02)
    spatial_triangle_list_03 = \
        _shaping_point_to_triangle(
                spatial_fan_point_list_01, fan_point_type_01)

    # 空間ID取得対象のSpatialTriangleを格納した配列を定義
    triangle_list = spatial_triangle_list_01 + spatial_triangle_list_02\
        + spatial_triangle_list_03

    # 空間IDの期待値を定義
    shapefile_spatial_id_list = [get_spatial_ids_on_polygons(
              triangle_list, [], h_zoom, v_zoom)]

    # 試験実施
    # 戻り値取得
    param_return = read_shapefile(
            filepath, encoding, h_zoom, v_zoom, shapefile_crs)

    # 戻り値確認
    loop_cnt = 0

    for ids, field, records in param_return:
        # APIから取得した空間IDは順不同のため比較前にソートし、格納された空間IDが一致していることを確認する。
        param_assert_ids = sorted(shapefile_spatial_id_list[loop_cnt])
        param_return_ids = sorted(ids)

        # 戻り値確認 - 空間ID, フィールド, ペイロード
        assert param_assert_ids == param_return_ids
        assert shapefile_field_list == field
        assert shapefile_record_list[loop_cnt] == records

        loop_cnt += 1


def test_read_shapefile_16():
    """強化試験：Shapefileから1レコード単位で情報を取得するジェネレータ取得、引数のfilepath型チェックNG
    + 試験詳細
      - 入力条件
        - filepathにint型の値を入力
      - 確認内容
        - 入力値チェックエラーの例外が発生すること
    """
    # 期待値定義
    # 例外を取得
    exception_assert = SpatialIdError('INPUT_VALUE_ERROR')

    # 試験実施
    # 例外発生を確認
    param_return = read_shapefile(100, "utf-8", 1, 1, 4326)

    with pytest.raises(SpatialIdError) as e:
        param_return.__next__()

    assert str(exception_assert) == str(e.value)


def test_read_shapefile_17():
    """強化試験：Shapefileから1レコード単位で情報を取得するジェネレータ取得、引数のfilepathが空文字
    + 試験詳細
      - 入力条件
        - filepathに空文字を入力
      - 確認内容
        - Shapefile読み込みエラーの例外が発生すること
    """
    # 期待値定義
    # 例外を取得
    exception_assert = SpatialIdError('SHAPEFILE_READ_ERROR')

    # 試験実施
    # 例外発生を確認
    param_return = read_shapefile("", "utf-8", 1, 1, 4326)

    with pytest.raises(SpatialIdError) as e:
        param_return.__next__()

    assert str(exception_assert) == str(e.value)


def test_read_shapefile_18():
    """強化試験：Shapefileから1レコード単位で情報を取得するジェネレータ取得、引数のfilepathがNone
    + 試験詳細
      - 入力条件
        - filepathにNoneを入力
      - 確認内容
        - 入力値チェックエラーの例外が発生すること
    """
    # 期待値定義
    # 例外を取得
    exception_assert = SpatialIdError('INPUT_VALUE_ERROR')

    # 試験実施
    # 例外発生を確認
    param_return = read_shapefile(None, "utf-8", 1, 1, 4326)

    with pytest.raises(SpatialIdError) as e:
        param_return.__next__()

    assert str(exception_assert) == str(e.value)


def test_read_shapefile_19():
    """強化試験：Shapefileから1レコード単位で情報を取得するジェネレータ取得、引数のencoding型チェックNG
    + 試験詳細
      - 入力条件
        - encodingにint型の値を入力
      - 確認内容
        - 入力値チェックエラーの例外が発生すること
    """
    # 期待値定義
    # 例外を取得
    exception_assert = SpatialIdError('INPUT_VALUE_ERROR')

    # 試験実施
    # 例外発生を確認
    param_return = read_shapefile("test/dir", 100, 1, 1, 4326)

    with pytest.raises(SpatialIdError) as e:
        param_return.__next__()

    assert str(exception_assert) == str(e.value)


def test_read_shapefile_20():
    """強化試験：Shapefileから1レコード単位で情報を取得するジェネレータ取得、引数のencodingに空文字を入力
    + 試験詳細
      - 入力条件
        - encodingに空文字を入力
      - 確認内容
        - Shapefile読み込みエラーの例外が発生すること
    """
    # 期待値定義
    # 例外を取得
    exception_assert = SpatialIdError('SHAPEFILE_READ_ERROR')

    # 試験実施
    # 例外発生を確認
    param_return = read_shapefile("test/dir", "", 1, 1, 4326)

    with pytest.raises(SpatialIdError) as e:
        param_return.__next__()

    assert str(exception_assert) == str(e.value)


def test_read_shapefile_21():
    """強化試験：Shapefileから1レコード単位で情報を取得するジェネレータ取得、引数のencodingにNoneを入力
    + 試験詳細
      - 入力条件
        - encodingにNoneを入力
      - 確認内容
        - 入力値チェックエラーの例外が発生すること
    """
    # 期待値定義
    # 例外を取得
    exception_assert = SpatialIdError('INPUT_VALUE_ERROR')

    # 試験実施
    # 例外発生を確認
    param_return = read_shapefile("test/dir", None, 1, 1, 4326)

    with pytest.raises(SpatialIdError) as e:
        param_return.__next__()

    assert str(exception_assert) == str(e.value)


def test_read_shapefile_22():
    """強化試験：Shapefileから1レコード単位で情報を取得するジェネレータ取得、引数のh_zoom型チェックNG
    + 試験詳細
      - 入力条件
        - h_zoomにstr型の値を入力
      - 確認内容
        - 入力値チェックエラーの例外が発生すること
    """
    # 期待値定義
    # 例外を取得
    exception_assert = SpatialIdError('INPUT_VALUE_ERROR')

    # 試験実施
    # 例外発生を確認
    param_return = read_shapefile("test/dir", "utf-8", "1", 1, 4326)

    with pytest.raises(SpatialIdError) as e:
        param_return.__next__()

    assert str(exception_assert) == str(e.value)


def test_read_shapefile_23():
    """強化試験：Shapefileから1レコード単位で情報を取得するジェネレータ取得、引数のh_zoomにNoneを入力
    + 試験詳細
      - 入力条件
        - h_zoomにNoneを入力
      - 確認内容
        - 入力値チェックエラーの例外が発生すること
    """
    # 期待値定義
    # 例外を取得
    exception_assert = SpatialIdError('INPUT_VALUE_ERROR')

    # 試験実施
    # 例外発生を確認
    param_return = read_shapefile("test/dir", "utf-8", None, 1, 4326)

    with pytest.raises(SpatialIdError) as e:
        param_return.__next__()

    assert str(exception_assert) == str(e.value)


def test_read_shapefile_24():
    """強化試験：Shapefileから1レコード単位で情報を取得するジェネレータ取得、引数のv_zoom型チェックNG
    + 試験詳細
      - 入力条件
        - v_zoomにstr型の値を入力
      - 確認内容
        - 入力値チェックエラーの例外が発生すること
    """
    # 期待値定義
    # 例外を取得
    exception_assert = SpatialIdError('INPUT_VALUE_ERROR')

    # 試験実施
    # 例外発生を確認
    param_return = read_shapefile("test/dir", "utf-8", 1, "1", 4326)

    with pytest.raises(SpatialIdError) as e:
        param_return.__next__()

    assert str(exception_assert) == str(e.value)


def test_read_shapefile_25():
    """強化試験：Shapefileから1レコード単位で情報を取得するジェネレータ取得、引数のv_zoomにNoneを入力
    + 試験詳細
      - 入力条件
        - v_zoomにNoneを入力
      - 確認内容
        - 入力値チェックエラーの例外が発生すること
    """
    # 期待値定義
    # 例外を取得
    exception_assert = SpatialIdError('INPUT_VALUE_ERROR')

    # 試験実施
    # 例外発生を確認
    param_return = read_shapefile("test/dir", "utf-8", 1, "1", 4326)

    with pytest.raises(SpatialIdError) as e:
        param_return.__next__()

    assert str(exception_assert) == str(e.value)


def test_read_shapefile_26():
    """強化試験：Shapefileから1レコード単位で情報を取得するジェネレータ取得、引数のcrs型チェックNG
    + 試験詳細
      - 入力条件
        - crsにstr型の値を入力
      - 確認内容
        - 入力値チェックエラーの例外が発生すること
    """
    # 期待値定義
    # 例外を取得
    exception_assert = SpatialIdError('INPUT_VALUE_ERROR')

    # 試験実施
    # 例外発生を確認
    param_return = read_shapefile("test/dir", "utf-8", 1, 1, "4326")

    with pytest.raises(SpatialIdError) as e:
        param_return.__next__()

    assert str(exception_assert) == str(e.value)


def test_read_shapefile_27():
    """強化試験：Shapefileから1レコード単位で情報を取得するジェネレータ取得、引数のcrsにNoneを入力
    + 試験詳細
      - 入力条件
        - crsにNoneを入力
      - 確認内容
        - 入力値チェックエラーの例外が発生すること
    """
    # 期待値定義
    # 例外を取得
    exception_assert = SpatialIdError('INPUT_VALUE_ERROR')

    # 試験実施
    # 例外発生を確認
    param_return = read_shapefile("test/dir", "utf-8", 1, 1, None)

    with pytest.raises(SpatialIdError) as e:
        param_return.__next__()

    assert str(exception_assert) == str(e.value)


def test_read_shapefile_28(geographic_shape_point_data_04):
    """Shapefileから1レコード単位で情報を取得するジェネレータ取得、閉塞していないポリゴン、閉塞チェックON
    + 試験詳細
      - 入力条件
        - ポリゴンの閉塞チェックフラグON
        - 読み込み対象のshapefileのポリゴンが閉塞していない
      - 確認内容
        - 三角形ポリゴンのモデルが閉塞していない場合の例外が発生すること
    """
    # 試験用データの定義
    # 試験用shalefileパス
    filepath = SHAPEFILE_DIR + 'test_read_shapefile_28'

    # encoding
    encoding = 'utf-8'

    # 水平、垂直方向精度を定義
    h_zoom = 1
    v_zoom = 1

    # shapefileに格納するデータの座標系
    shapefile_crs = 4326

    # shapefile作成用の座標配列取得
    fan_point_list_01 = geographic_shape_point_data_04[0][0]
    fan_point_list_02 = geographic_shape_point_data_04[0][1]

    # shapefile生成
    w = sf.Writer(filepath)
    w.field('name', 'C')
    w.field('type', 'C')
    w.multipatch(
            [fan_point_list_01, fan_point_list_02],
            partTypes=[TRIANGLE_FAN, TRIANGLE_FAN])
    w.record('record1', 'FAN')
    w.close()

    # 期待値取得
    # 例外取得
    exception_assert = SpatialIdError('POLYGON_NOT_CLOSED_MODEL')

    # 試験実施
    # 戻り値取得
    param_return = read_shapefile(
            filepath, encoding, h_zoom, v_zoom, shapefile_crs)

    # 例外発生確認
    with pytest.raises(SpatialIdError) as e:
        # Generator内での例外発生を検知するため、Generatorの要素を参照
        param_return.__next__()

    # 例外メッセージ確認
    assert str(exception_assert) == str(e.value)


def test_read_shapefile_29(geographic_shape_point_data_04):
    """Shapefileから1レコード単位で情報を取得するジェネレータ取得、閉塞していないポリゴン、閉塞チェックOFF
    + 試験詳細
      - 入力条件
        - ポリゴンの閉塞チェックフラグOFF
        - 閉塞していないモデルを含むshapefile読み込み
      - 確認内容
        - ジェネレータが返却され、空間ID、フィールド名、ペイロードを1レコード毎に参照可能であること
    """
    # 試験用データの定義
    # 試験用shalefileパス
    filepath = SHAPEFILE_DIR + 'test_read_shapefile_29'

    # encoding
    encoding = 'utf-8'

    # 水平、垂直方向精度を定義
    h_zoom = 10
    v_zoom = 10

    # shapefileに格納するデータの座標系
    shapefile_crs = 4326

    # shapefile作成用の座標配列取得
    fan_point_list_01 = geographic_shape_point_data_04[0][0]
    fan_point_type_01 = geographic_shape_point_data_04[1][0]
    fan_point_list_02 = geographic_shape_point_data_04[0][1]
    fan_point_type_02 = geographic_shape_point_data_04[1][1]

    # shapefile生成
    w = sf.Writer(filepath)
    w.field('name', 'C')
    w.field('type', 'C')
    w.multipatch(
            [fan_point_list_01, fan_point_list_02],
            partTypes=[TRIANGLE_FAN, TRIANGLE_FAN])
    w.record('record1', 'FAN')
    w.close()

    # 期待値取得
    # ペイロードの期待値を定義
    shapefile_record_list = [['record1', 'FAN']]

    # fieldの期待値を定義
    shapefile_field_list = [['name', 'C', 50, 0], ['type', 'C', 50, 0]]

    # TRIANGLE_FANのSpatialPointオブジェクトを生成
    spatial_fan_point_list_01 = _get_spatial_point_objects(fan_point_list_01)
    spatial_fan_point_list_02 = _get_spatial_point_objects(fan_point_list_02)

    # SpatialTriangleオブジェクト生成
    spatial_triangle_list_01 = \
        _shaping_point_to_triangle(
                spatial_fan_point_list_01, fan_point_type_01)
    spatial_triangle_list_02 = \
        _shaping_point_to_triangle(
                spatial_fan_point_list_02, fan_point_type_02)

    # 空間ID取得対象のSpatialTriangleを格納した配列を定義
    triangle_list = spatial_triangle_list_01 + spatial_triangle_list_02

    # 空間IDの期待値を定義
    shapefile_spatial_id_list = [get_spatial_ids_on_polygons(
              triangle_list,
              [],
              h_zoom,
              v_zoom,
              needs_closed_checking=False)]

    # 試験実施
    # 戻り値取得
    param_return = read_shapefile(
            filepath, encoding, h_zoom, v_zoom, shapefile_crs, False)

    # 戻り値確認
    loop_cnt = 0

    for ids, field, records in param_return:
        # APIから取得した空間IDは順不同のため比較前にソートし、格納された空間IDが一致していることを確認する。
        param_assert_ids = sorted(shapefile_spatial_id_list[loop_cnt])
        param_return_ids = sorted(ids)

        # 戻り値確認 - 空間ID, フィールド, ペイロード
        assert param_assert_ids == param_return_ids
        assert shapefile_field_list == field
        assert shapefile_record_list[loop_cnt] == records

        loop_cnt += 1

def test_f_read_shapefile_bulk_01(
        geographic_shape_point_data_01,
        geographic_shape_point_data_02
):
    """Shapefileから全レコードの空間ID情報を一括で取得する、shapefileレコード数2、CSR入力なし
    + 試験詳細
      - 入力条件
        - 2レコードが格納されているshapefileを読み込み
        - 読み込み対象ファイル拡張子無し
        - CSR入力なし
      - 確認内容
        - Shapefileから全レコードの空間ID情報を一括で取得できること
    """
    # 試験用データの定義
    # 試験用shalefileパス
    filepath = SHAPEFILE_DIR + 'test_f_read_shapefile_bulk_01'

    # encoding
    encoding = 'utf-8'

    # 水平、垂直方向精度を定義
    h_zoom = 1
    v_zoom = 1

    # shapefile作成用の座標配列取得
    fan_point_list_01 = geographic_shape_point_data_01[0][0]
    fan_point_type_01 = geographic_shape_point_data_01[1][0]
    fan_point_list_02 = geographic_shape_point_data_01[0][1]
    fan_point_type_02 = geographic_shape_point_data_01[1][1]
    fan_point_list_03 = geographic_shape_point_data_02[0][0]
    fan_point_type_03 = geographic_shape_point_data_02[1][0]
    fan_point_list_04 = geographic_shape_point_data_02[0][1]
    fan_point_type_04 = geographic_shape_point_data_02[1][1]

    # shapefile生成
    w = sf.Writer(filepath)
    w.field('name', 'C')
    w.field('type', 'C')
    w.multipatch(
            [fan_point_list_01, fan_point_list_02],
            partTypes=[TRIANGLE_FAN, TRIANGLE_FAN])
    w.record('record1', 'FAN')
    w.multipatch(
            [fan_point_list_03, fan_point_list_04],
            partTypes=[TRIANGLE_FAN, TRIANGLE_FAN])
    w.record('record2', 'FAN')
    w.close()

    # 期待値取得
    # ペイロードの期待値を定義
    shapefile_record_list = [['record1', 'FAN'], ['record2', 'FAN']]

    # fieldの期待値を定義
    shapefile_field_list = [['name', 'C', 50, 0], ['type', 'C', 50, 0]]

    # TRIANGLE_FANのSpatialPointオブジェクトを生成
    spatial_fan_point_list_01 = _get_spatial_point_objects(fan_point_list_01)
    spatial_fan_point_list_02 = _get_spatial_point_objects(fan_point_list_02)
    spatial_fan_point_list_03 = _get_spatial_point_objects(fan_point_list_03)
    spatial_fan_point_list_04 = _get_spatial_point_objects(fan_point_list_04)

    # SpatialTriangleオブジェクト生成
    spatial_triangle_list_01 = \
        _shaping_point_to_triangle(
                spatial_fan_point_list_01, fan_point_type_01)
    spatial_triangle_list_02 = \
        _shaping_point_to_triangle(
                spatial_fan_point_list_02, fan_point_type_02)
    spatial_triangle_list_03 = \
        _shaping_point_to_triangle(
                spatial_fan_point_list_03, fan_point_type_03)
    spatial_triangle_list_04 = \
        _shaping_point_to_triangle(
                spatial_fan_point_list_04, fan_point_type_04)

    # 空間ID取得対象のSpatialTriangleを格納した配列を定義
    triangle_list_01 = spatial_triangle_list_01 + spatial_triangle_list_02
    triangle_list_02 = spatial_triangle_list_03 + spatial_triangle_list_04

    # 空間IDの期待値を定義
    shapefile_spatial_ids_01 = f_get_spatial_ids_on_polygons(
            triangle_list_01, [], h_zoom)
    shapefile_spatial_ids_02 = f_get_spatial_ids_on_polygons(
            triangle_list_02, [], h_zoom)
    shapefile_spatial_id_list = [
            shapefile_spatial_ids_01,
            shapefile_spatial_ids_02]

    # 試験実施
    # 戻り値取得
    param_return = f_read_shapefile_bulk(
            filepath, encoding, h_zoom)

    # 戻り値確認
    loop_cnt = 0

    # 空間IDの要素数確認
    param_return_spatial_id_list = param_return[0]
    assert len(shapefile_spatial_id_list) == len(param_return_spatial_id_list)

    # フィールドの要素一致確認
    assert shapefile_field_list == param_return[1]

    # ペイロードの要素一致確認
    assert shapefile_record_list == param_return[2]

    for ids in param_return_spatial_id_list:
        # APIから取得した空間IDは順不同のため比較前にソートし、格納された空間IDが一致していることを確認する。
        param_assert_ids = sorted(shapefile_spatial_id_list[loop_cnt])
        param_return_ids = sorted(ids)

        # 戻り値確認 - 空間ID
        assert param_assert_ids == param_return_ids

        loop_cnt += 1


def test_f_read_shapefile_bulk_02():
    """一括取得メソッド呼び出し時にshapefileの読み込み失敗、不正なファイルパス指定
    + 試験詳細
      - 入力条件
        - 存在しないファイルのパスを読み込み対象のファイルパスに指定
      - 確認内容
        - shapefile読み込み失敗の例外が発生すること
    """
    # 試験用データの定義
    # 試験用shalefileパス
    filepath = SHAPEFILE_DIR + 'test_f_read_shapefile_bulk_02'

    # encoding
    encoding = 'utf-8'

    # 精度を定義
    zoom = 1

    # shapefileに格納するデータの座標系
    shapefile_crs = 4326

    # 期待値取得
    # 例外取得
    exception_assert = SpatialIdError('SHAPEFILE_READ_ERROR')

    # 試験実施
    # 例外発生確認
    with pytest.raises(SpatialIdError) as e:
        f_read_shapefile_bulk(filepath, encoding, zoom, shapefile_crs)

    # 例外メッセージ確認
    assert str(exception_assert) == str(e.value)


def test_f_read_shapefile_bulk_03():
    """一括取得メソッド呼び出し時に座標系の変換失敗、入力された座標と座標系が対応していない
    + 試験詳細
      - 入力条件
        - shapefileに格納された座標が、座標系に対応していない値になっている
      - 確認内容
        - 入力チェックエラーの例外が発生すること
    """
    # 試験用データの定義
    # 試験用shalefileパス
    filepath = SHAPEFILE_DIR + 'test_f_read_shapefile_bulk_03'

    # encoding
    encoding = 'utf-8'

    # 精度を定義
    zoom = 1

    # shapefileに格納するデータの座標系
    shapefile_crs = 4326

    # Pointの座標を格納したリストを定義
    strip_point_01 = (1000.0, 0.0, 0.0)
    strip_point_02 = (1000.0, 0.0, 3.0)
    strip_point_03 = (1005.0, 0.0, 0.0)
    strip_point_04 = (1005.0, 0.0, 3.0)

    # shapefile作成用の座標配列
    strip_point_list = [
            strip_point_01,
            strip_point_02,
            strip_point_03,
            strip_point_04
            ]

    # shapefile生成
    w = sf.Writer(filepath)
    w.field('name', 'C')
    w.field('type', 'C')
    w.multipatch(
            [strip_point_list],
            partTypes=[TRIANGLE_STRIP])
    w.record('record1', 'STRIP')
    w.close()

    # 期待値取得
    # 例外取得
    exception_assert = SpatialIdError('VALUE_CONVERT_ERROR')

    # 試験実施
    # 例外発生確認
    with pytest.raises(SpatialIdError) as e:
        f_read_shapefile_bulk(filepath, encoding, zoom, shapefile_crs)

    # 例外メッセージ確認
    assert str(exception_assert) == str(e.value)


def test_f_read_shapefile_bulk_04(mocker):
    """一括取得メソッド呼び出し時、その他例外の発生
     + 試験詳細
       - 試験条件
         - 通常発生しない例外パターンのためモックを、使用して例外を発生させる
       - 確認内容
         - その他例外が発生すること
    """
    # 試験用データの定義
    # 試験用shalefileパス
    filepath = SHAPEFILE_DIR + 'test_f_read_shapefile_bulk_04'

    # encoding
    encoding = 'utf-8'

    # 精度を定義
    zoom = 10

    # shapefileに格納するデータの座標系
    shapefile_crs = 4326

    # Pointの座標を格納したリストを定義
    strip_point_01 = (139.051685, 45.060433, 150.5)
    strip_point_02 = (140.051685, 46.060433, 151.5)
    strip_point_03 = (141.051685, 47.060433, 152.5)
    strip_point_04 = (142.051685, 48.060433, 151.5)

    # shapefile作成用の座標配列
    strip_point_list = [
            strip_point_01,
            strip_point_02,
            strip_point_03,
            strip_point_04
            ]

    # shapefile生成
    w = sf.Writer(filepath)
    w.field('name', 'C')
    w.field('type', 'C')
    w.multipatch(
            [strip_point_list],
            partTypes=[TRIANGLE_STRIP])
    w.record('record1', 'STRIP')
    w.close()

    # 期待値取得
    # 例外取得
    exception_assert = SpatialIdError('OTHER_ERROR')

    mocker.patch(
            'SpatialId.io.shapefile._get_shapefile_spatial_id_and_record').\
        side_effect = Exception('予期せぬ例外が発生')

    # 試験実施
    with pytest.raises(SpatialIdError) as e:
        f_read_shapefile_bulk(filepath, encoding, zoom, shapefile_crs)

    # 例外メッセージ確認
    assert str(exception_assert) == str(e.value)


def test_read_shapefile_bulk_01(
        geographic_shape_point_data_01,
        geographic_shape_point_data_02
):
    """Shapefileから全レコードの空間ID情報を一括で取得する、shapefileレコード数2、CSR入力なし
    + 試験詳細
      - 入力条件
        - 2レコードが格納されているshapefileを読み込み
        - 読み込み対象ファイル拡張子無し
        - CSR入力なし
      - 確認内容
        - Shapefileから全レコードの空間ID情報を一括で取得できること
    """
    # 試験用データの定義
    # 試験用shalefileパス
    filepath = SHAPEFILE_DIR + 'test_read_shapefile_bulk_01'

    # encoding
    encoding = 'utf-8'

    # 水平、垂直方向精度を定義
    h_zoom = 1
    v_zoom = 1

    # shapefile作成用の座標配列取得
    fan_point_list_01 = geographic_shape_point_data_01[0][0]
    fan_point_type_01 = geographic_shape_point_data_01[1][0]
    fan_point_list_02 = geographic_shape_point_data_01[0][1]
    fan_point_type_02 = geographic_shape_point_data_01[1][1]
    fan_point_list_03 = geographic_shape_point_data_02[0][0]
    fan_point_type_03 = geographic_shape_point_data_02[1][0]
    fan_point_list_04 = geographic_shape_point_data_02[0][1]
    fan_point_type_04 = geographic_shape_point_data_02[1][1]

    # shapefile生成
    w = sf.Writer(filepath)
    w.field('name', 'C')
    w.field('type', 'C')
    w.multipatch(
            [fan_point_list_01, fan_point_list_02],
            partTypes=[TRIANGLE_FAN, TRIANGLE_FAN])
    w.record('record1', 'FAN')
    w.multipatch(
            [fan_point_list_03, fan_point_list_04],
            partTypes=[TRIANGLE_FAN, TRIANGLE_FAN])
    w.record('record2', 'FAN')
    w.close()

    # 期待値取得
    # ペイロードの期待値を定義
    shapefile_record_list = [['record1', 'FAN'], ['record2', 'FAN']]

    # fieldの期待値を定義
    shapefile_field_list = [['name', 'C', 50, 0], ['type', 'C', 50, 0]]

    # TRIANGLE_FANのSpatialPointオブジェクトを生成
    spatial_fan_point_list_01 = _get_spatial_point_objects(fan_point_list_01)
    spatial_fan_point_list_02 = _get_spatial_point_objects(fan_point_list_02)
    spatial_fan_point_list_03 = _get_spatial_point_objects(fan_point_list_03)
    spatial_fan_point_list_04 = _get_spatial_point_objects(fan_point_list_04)

    # SpatialTriangleオブジェクト生成
    spatial_triangle_list_01 = \
        _shaping_point_to_triangle(
                spatial_fan_point_list_01, fan_point_type_01)
    spatial_triangle_list_02 = \
        _shaping_point_to_triangle(
                spatial_fan_point_list_02, fan_point_type_02)
    spatial_triangle_list_03 = \
        _shaping_point_to_triangle(
                spatial_fan_point_list_03, fan_point_type_03)
    spatial_triangle_list_04 = \
        _shaping_point_to_triangle(
                spatial_fan_point_list_04, fan_point_type_04)

    # 空間ID取得対象のSpatialTriangleを格納した配列を定義
    triangle_list_01 = spatial_triangle_list_01 + spatial_triangle_list_02
    triangle_list_02 = spatial_triangle_list_03 + spatial_triangle_list_04

    # 空間IDの期待値を定義
    shapefile_spatial_ids_01 = get_spatial_ids_on_polygons(
            triangle_list_01, [], h_zoom, v_zoom)
    shapefile_spatial_ids_02 = get_spatial_ids_on_polygons(
            triangle_list_02, [], h_zoom, v_zoom)
    shapefile_spatial_id_list = [
            shapefile_spatial_ids_01,
            shapefile_spatial_ids_02]

    # 試験実施
    # 戻り値取得
    param_return = read_shapefile_bulk(
            filepath, encoding, h_zoom, v_zoom)

    # 戻り値確認
    loop_cnt = 0

    # 空間IDの要素数確認
    param_return_spatial_id_list = param_return[0]
    assert len(shapefile_spatial_id_list) == len(param_return_spatial_id_list)

    # フィールドの要素一致確認
    assert shapefile_field_list == param_return[1]

    # ペイロードの要素一致確認
    assert shapefile_record_list == param_return[2]

    for ids in param_return_spatial_id_list:
        # APIから取得した空間IDは順不同のため比較前にソートし、格納された空間IDが一致していることを確認する。
        param_assert_ids = sorted(shapefile_spatial_id_list[loop_cnt])
        param_return_ids = sorted(ids)

        # 戻り値確認 - 空間ID
        assert param_assert_ids == param_return_ids

        loop_cnt += 1


def test_read_shapefile_bulk_02():
    """一括取得メソッド呼び出し時にshapefileの読み込み失敗、不正なファイルパス指定
    + 試験詳細
      - 入力条件
        - 存在しないファイルのパスを読み込み対象のファイルパスに指定
      - 確認内容
        - shapefile読み込み失敗の例外が発生すること
    """
    # 試験用データの定義
    # 試験用shalefileパス
    filepath = SHAPEFILE_DIR + 'test_read_shapefile_bulk_02'

    # encoding
    encoding = 'utf-8'

    # 水平、垂直方向精度を定義
    h_zoom = 1
    v_zoom = 1

    # shapefileに格納するデータの座標系
    shapefile_crs = 4326

    # 期待値取得
    # 例外取得
    exception_assert = SpatialIdError('SHAPEFILE_READ_ERROR')

    # 試験実施
    # 例外発生確認
    with pytest.raises(SpatialIdError) as e:
        read_shapefile_bulk(filepath, encoding, h_zoom, v_zoom, shapefile_crs)

    # 例外メッセージ確認
    assert str(exception_assert) == str(e.value)


def test_read_shapefile_bulk_03(geographic_shape_point_data_01):
    """一括取得メソッド呼び出し時に全角文字を含むフィールド名を持つshapefile読み込み、encoding指定、shapefileレコード数1
    + 試験詳細
      - 入力条件
        - encodingを指定(shift_jis)
        - フィールド名に全角文字を含むshapefileを読み込み
        - 1レコードが格納されているshapefileを読み込み
        - CSRにデフォルト値と同じ地理座標系(4326)を入力
      - 確認内容
        - Shapefileから全レコードの空間ID情報を一括で取得できること
    """
    # 試験用データの定義
    # 試験用shalefileパス
    filepath = SHAPEFILE_DIR + 'test_read_shapefile_bulk_03'

    # encoding
    encoding = 'shift_jis'

    # 水平、垂直方向精度を定義
    h_zoom = 6
    v_zoom = 6

    # shapefileに格納するデータの座標系
    shapefile_crs = 4326

    # shapefile作成用の座標配列取得
    fan_point_list_01 = geographic_shape_point_data_01[0][0]
    fan_point_type_01 = geographic_shape_point_data_01[1][0]
    fan_point_list_02 = geographic_shape_point_data_01[0][1]
    fan_point_type_02 = geographic_shape_point_data_01[1][1]

    # shapefile生成
    w = sf.Writer(filepath, encoding="shift_jis")
    w.field('名前', 'C')
    w.field('type', 'C')
    w.multipatch(
            [fan_point_list_01, fan_point_list_02],
            partTypes=[TRIANGLE_FAN, TRIANGLE_FAN])
    w.record('レコード1', 'FAN')
    w.close()

    # 期待値取得
    # ペイロードの期待値を定義
    shapefile_record_list = [['レコード1', 'FAN']]

    # fieldの期待値を定義
    shapefile_field_list = [['名前', 'C', 50, 0], ['type', 'C', 50, 0]]

    # TRIANGLE_FANのSpatialPointオブジェクトを生成
    spatial_fan_point_list_01 = _get_spatial_point_objects(fan_point_list_01)
    spatial_fan_point_list_02 = _get_spatial_point_objects(fan_point_list_02)

    # SpatialTriangleオブジェクト生成
    spatial_triangle_list_01 = \
        _shaping_point_to_triangle(
                spatial_fan_point_list_01, fan_point_type_01)
    spatial_triangle_list_02 = \
        _shaping_point_to_triangle(
                spatial_fan_point_list_02, fan_point_type_02)

    # 空間ID取得対象のSpatialTriangleを格納した配列を定義
    triangle_list = spatial_triangle_list_01 + spatial_triangle_list_02

    # 空間IDの期待値を定義
    shapefile_spatial_id_list = [get_spatial_ids_on_polygons(
              triangle_list, [], h_zoom, v_zoom)]

    # 試験実施
    # 戻り値取得
    param_return = read_shapefile_bulk(
            filepath, encoding, h_zoom, v_zoom, shapefile_crs)

    # 戻り値確認
    loop_cnt = 0

    # 空間IDの要素数確認
    param_return_spatial_id_list = param_return[0]
    assert len(shapefile_spatial_id_list) == len(param_return_spatial_id_list)

    # フィールドの要素一致確認
    assert shapefile_field_list == param_return[1]

    # ペイロードの要素一致確認
    assert shapefile_record_list == param_return[2]

    for ids in param_return_spatial_id_list:
        # APIから取得した空間IDは順不同のため比較前にソートし、格納された空間IDが一致していることを確認する。
        param_assert_ids = sorted(shapefile_spatial_id_list[loop_cnt])
        param_return_ids = sorted(ids)

        # 戻り値確認 - 空間ID
        assert param_assert_ids == param_return_ids

        loop_cnt += 1


def test_read_shapefile_bulk_04():
    """一括取得メソッド呼び出し時にshapefileの読み込み失敗、不正なencoding指定
    + 試験詳細
      - 入力条件
        - 読み込み対象のshapefileに対応しないencodingを入力
      - 確認内容
        - shapefile読み込み失敗の例外が発生すること
    """
    # 試験用データの定義
    # 試験用shalefileパス
    filepath = SHAPEFILE_DIR + 'test_read_shapefile_bulk_04'

    # encoding
    encoding = 'utf-8'

    # 水平、垂直方向精度を定義
    h_zoom = 1
    v_zoom = 1

    # shapefileに格納するデータの座標系
    shapefile_crs = 4326

    # Pointの座標を格納したリストを定義
    strip_point_01 = (139.051685, 45.060433, 150.5)
    strip_point_02 = (140.051685, 46.060433, 151.5)
    strip_point_03 = (141.051685, 47.060433, 152.5)
    strip_point_04 = (142.051685, 48.060433, 151.5)

    # shapefile作成用の座標配列
    strip_point_list = [
            strip_point_01,
            strip_point_02,
            strip_point_03,
            strip_point_04
            ]

    # shapefile生成
    w = sf.Writer(filepath, encoding="shift_jis")
    w.field('名前', 'C')
    w.field('type', 'C')
    w.multipatch(
            [strip_point_list],
            partTypes=[TRIANGLE_STRIP])
    w.record('レコード1', 'STRIP')
    w.close()

    # 期待値取得
    # 例外取得
    exception_assert = SpatialIdError('SHAPEFILE_READ_ERROR')

    # 試験実施
    # 例外発生確認
    with pytest.raises(SpatialIdError) as e:
        read_shapefile_bulk(filepath, encoding, h_zoom, v_zoom, shapefile_crs)

    # 例外メッセージ確認
    assert str(exception_assert) == str(e.value)


def test_read_shapefile_bulk_05():
    """一括取得メソッド呼び出し時にshapefileの読み込み失敗、encoding空文字
    + 試験詳細
      - 入力条件
        - encoding空文字を入力
      - 確認内容
        - shapefile読み込み失敗の例外が発生すること
    """
    # 試験用データの定義
    # 試験用shalefileパス
    filepath = SHAPEFILE_DIR + 'test_read_shapefile_bulk_05'

    # encoding
    encoding = ''

    # 水平、垂直方向精度を定義
    h_zoom = 1
    v_zoom = 1

    # shapefileに格納するデータの座標系
    shapefile_crs = 4326

    # Pointの座標を格納したリストを定義
    strip_point_01 = (139.051685, 45.060433, 150.5)
    strip_point_02 = (140.051685, 46.060433, 151.5)
    strip_point_03 = (141.051685, 47.060433, 152.5)
    strip_point_04 = (142.051685, 48.060433, 151.5)

    # shapefile作成用の座標配列
    strip_point_list = [
            strip_point_01,
            strip_point_02,
            strip_point_03,
            strip_point_04
            ]

    # shapefile生成
    w = sf.Writer(filepath)
    w.field('name', 'C')
    w.field('type', 'C')
    w.multipatch(
            [strip_point_list],
            partTypes=[TRIANGLE_STRIP])
    w.record('record1', 'STRIP')
    w.close()

    # 期待値取得
    # 例外取得
    exception_assert = SpatialIdError('SHAPEFILE_READ_ERROR')

    # 試験実施
    # 例外発生確認
    with pytest.raises(SpatialIdError) as e:
        read_shapefile_bulk(filepath, encoding, h_zoom, v_zoom, shapefile_crs)

    # 例外メッセージ確認
    assert str(exception_assert) == str(e.value)


def test_read_shapefile_bulk_06():
    """一括取得メソッド呼び出し時に座標系の変換失敗、入力された座標と座標系が対応していない
    + 試験詳細
      - 入力条件
        - shapefileに格納された座標が、座標系に対応していない値になっている
      - 確認内容
        - 入力チェックエラーの例外が発生すること
    """
    # 試験用データの定義
    # 試験用shalefileパス
    filepath = SHAPEFILE_DIR + 'test_read_shapefile_bulk_06'

    # encoding
    encoding = 'utf-8'

    # 水平、垂直方向精度を定義
    h_zoom = 1
    v_zoom = 1

    # shapefileに格納するデータの座標系
    shapefile_crs = 4326

    # Pointの座標を格納したリストを定義
    strip_point_01 = (1000.0, 0.0, 0.0)
    strip_point_02 = (1000.0, 0.0, 3.0)
    strip_point_03 = (1005.0, 0.0, 0.0)
    strip_point_04 = (1005.0, 0.0, 3.0)

    # shapefile作成用の座標配列
    strip_point_list = [
            strip_point_01,
            strip_point_02,
            strip_point_03,
            strip_point_04
            ]

    # shapefile生成
    w = sf.Writer(filepath)
    w.field('name', 'C')
    w.field('type', 'C')
    w.multipatch(
            [strip_point_list],
            partTypes=[TRIANGLE_STRIP])
    w.record('record1', 'STRIP')
    w.close()

    # 期待値取得
    # 例外取得
    exception_assert = SpatialIdError('VALUE_CONVERT_ERROR')

    # 試験実施
    # 例外発生確認
    with pytest.raises(SpatialIdError) as e:
        read_shapefile_bulk(filepath, encoding, h_zoom, v_zoom, shapefile_crs)

    # 例外メッセージ確認
    assert str(exception_assert) == str(e.value)


def test_read_shapefile_bulk_07():
    """一括取得メソッド呼び出し時に内部処理の例外発生を検知
    + 試験詳細
      - 入力条件
        - サポート対象外のshapeTypeのshapefile読み込み
      - 確認内容
        - shapeType不正の例外が発生すること
    """
    # 試験用データの定義
    # 試験用shalefileパス
    filepath = SHAPEFILE_DIR + 'test_read_shapefile_bulk_07'

    # encoding
    encoding = 'utf-8'

    # 水平、垂直方向精度を定義
    h_zoom = 1
    v_zoom = 1

    # shapefileに格納するデータの座標系
    shapefile_crs = 4326

    # Pointの座標を格納したリストを定義
    point_01 = (139.051685, 45.060433, 150.5)
    point_02 = (140.051685, 46.060433, 151.5)
    point_03 = (141.051685, 47.060433, 152.5)

    # shapefile作成用の座標配列
    point_list = [
            point_01,
            point_02,
            point_03
            ]

    # shapefile生成
    w = sf.Writer(filepath)
    w.field('name', 'C')
    w.field('type', 'C')
    w.multipointz(point_list)
    w.record('record1', 'MPOINTZ')
    w.close()

    # 期待値取得
    # 例外取得
    exception_assert = SpatialIdError('SHAPEFILE_SHAPETYPE_ERROR')

    # 試験実施
    # 例外発生確認
    with pytest.raises(SpatialIdError) as e:
        read_shapefile_bulk(filepath, encoding, h_zoom, v_zoom, shapefile_crs)

    # 例外メッセージ確認
    assert str(exception_assert) == str(e.value)


def test_read_shapefile_bulk_08(mocker):
    """一括取得メソッド呼び出し時、その他例外の発生
     + 試験詳細
       - 試験条件
         - 通常発生しない例外パターンのためモックを、使用して例外を発生させる
       - 確認内容
         - その他例外が発生すること
    """
    # 試験用データの定義
    # 試験用shalefileパス
    filepath = SHAPEFILE_DIR + 'test_read_shapefile_bulk_08'

    # encoding
    encoding = 'utf-8'

    # 水平、垂直方向精度を定義
    h_zoom = 10
    v_zoom = 10

    # shapefileに格納するデータの座標系
    shapefile_crs = 4326

    # Pointの座標を格納したリストを定義
    strip_point_01 = (139.051685, 45.060433, 150.5)
    strip_point_02 = (140.051685, 46.060433, 151.5)
    strip_point_03 = (141.051685, 47.060433, 152.5)
    strip_point_04 = (142.051685, 48.060433, 151.5)

    # shapefile作成用の座標配列
    strip_point_list = [
            strip_point_01,
            strip_point_02,
            strip_point_03,
            strip_point_04
            ]

    # shapefile生成
    w = sf.Writer(filepath)
    w.field('name', 'C')
    w.field('type', 'C')
    w.multipatch(
            [strip_point_list],
            partTypes=[TRIANGLE_STRIP])
    w.record('record1', 'STRIP')
    w.close()

    # 期待値取得
    # 例外取得
    exception_assert = SpatialIdError('OTHER_ERROR')

    mocker.patch(
            'SpatialId.io.shapefile._get_shapefile_spatial_id_and_record').\
        side_effect = Exception('予期せぬ例外が発生')

    # 試験実施
    with pytest.raises(SpatialIdError) as e:
        read_shapefile_bulk(filepath, encoding, h_zoom, v_zoom, shapefile_crs)

    # 例外メッセージ確認
    assert str(exception_assert) == str(e.value)


def test_read_shapefile_bulk_09(geographic_shape_point_data_01):
    """強化試験：Shapefileから全レコードの空間ID情報を一括で取得する、読み込み対象がzipファイル
    + 試験詳細
      - 入力条件
        - zip形式の圧縮ファイルを読み込み対象のshapefileとする。
      - 確認内容
        - エラーとならず、空間ID情報の一括取得に成功すること
    """
    # 試験用データの定義
    # 試験用shalefileパス
    filepath = SHAPEFILE_DIR + 'test_read_shapefile_bulk_09'

    # encoding
    encoding = 'utf-8'

    # 水平、垂直方向精度を定義
    h_zoom = 6
    v_zoom = 6

    # shapefileに格納するデータの座標系
    shapefile_crs = 4326

    # shapefile作成用の座標配列取得
    fan_point_list_01 = geographic_shape_point_data_01[0][0]
    fan_point_type_01 = geographic_shape_point_data_01[1][0]
    fan_point_list_02 = geographic_shape_point_data_01[0][1]
    fan_point_type_02 = geographic_shape_point_data_01[1][1]

    # shapefile生成
    w = sf.Writer(filepath)
    w.field('name', 'C')
    w.field('type', 'C')
    w.multipatch(
            [fan_point_list_01, fan_point_list_02],
            partTypes=[TRIANGLE_FAN, TRIANGLE_FAN])
    w.record('record1', 'FAN')
    w.close()

    # zipファイル作成
    dbf_path = filepath + '.dbf'
    shp_path = filepath + '.shp'
    shx_path = filepath + '.shx'
    zip_path = filepath + '.zip'
    zp = zipfile.ZipFile(zip_path, "w")
    zp.write(dbf_path, compress_type=zipfile.ZIP_DEFLATED)
    zp.write(shp_path, compress_type=zipfile.ZIP_DEFLATED)
    zp.write(shx_path, compress_type=zipfile.ZIP_DEFLATED)
    zp.close()

    # 圧縮済みのshapefileを削除
    os.remove(dbf_path)
    os.remove(shp_path)
    os.remove(shx_path)

    # 期待値取得
    # ペイロードの期待値を定義
    shapefile_record_list = [['record1', 'FAN']]

    # fieldの期待値を定義
    shapefile_field_list = [['name', 'C', 50, 0], ['type', 'C', 50, 0]]

    # TRIANGLE_FANのSpatialPointオブジェクトを生成
    spatial_fan_point_list_01 = _get_spatial_point_objects(fan_point_list_01)
    spatial_fan_point_list_02 = _get_spatial_point_objects(fan_point_list_02)

    # SpatialTriangleオブジェクト生成
    spatial_triangle_list_01 = \
        _shaping_point_to_triangle(
                spatial_fan_point_list_01, fan_point_type_01)
    spatial_triangle_list_02 = \
        _shaping_point_to_triangle(
                spatial_fan_point_list_02, fan_point_type_02)

    # 空間ID取得対象のSpatialTriangleを格納した配列を定義
    triangle_list = spatial_triangle_list_01 + spatial_triangle_list_02

    # 空間IDの期待値を定義
    shapefile_spatial_id_list = [get_spatial_ids_on_polygons(
              triangle_list, [], h_zoom, v_zoom)]

    # 試験実施
    # 戻り値取得
    param_return = read_shapefile_bulk(
            zip_path, encoding, h_zoom, v_zoom, shapefile_crs)

    # 戻り値確認
    loop_cnt = 0

    # 空間IDの要素数確認
    param_return_spatial_id_list = param_return[0]
    assert len(shapefile_spatial_id_list) == len(param_return_spatial_id_list)

    # フィールドの要素一致確認
    assert shapefile_field_list == param_return[1]

    # ペイロードの要素一致確認
    assert shapefile_record_list == param_return[2]

    for ids in param_return_spatial_id_list:
        # APIから取得した空間IDは順不同のため比較前にソートし、格納された空間IDが一致していることを確認する。
        param_assert_ids = sorted(shapefile_spatial_id_list[loop_cnt])
        param_return_ids = sorted(ids)

        # 戻り値確認 - 空間ID
        assert param_assert_ids == param_return_ids

        loop_cnt += 1


def test_read_shapefile_bulk_10(geographic_shape_point_data_01):
    """強化試験：Shapefileから全レコードの空間ID情報を一括で取得する、読み込みファイル拡張子が.shp
    + 試験詳細
      - 入力条件
        - 読み込みファイル拡張子が.shp
      - 確認内容
        - Shapefileから全レコードの空間ID情報を一括で取得できること
    """
    # 試験用データの定義
    # 試験用shalefileパス
    filepath = SHAPEFILE_DIR + 'test_read_shapefile_bulk_10'

    # encoding
    encoding = 'utf-8'

    # 水平、垂直方向精度を定義
    h_zoom = 6
    v_zoom = 6

    # shapefileに格納するデータの座標系
    shapefile_crs = 4326

    # shapefile作成用の座標配列取得
    fan_point_list_01 = geographic_shape_point_data_01[0][0]
    fan_point_type_01 = geographic_shape_point_data_01[1][0]
    fan_point_list_02 = geographic_shape_point_data_01[0][1]
    fan_point_type_02 = geographic_shape_point_data_01[1][1]

    # shapefile生成
    w = sf.Writer(filepath)
    w.field('name', 'C')
    w.field('type', 'C')
    w.multipatch(
            [fan_point_list_01, fan_point_list_02],
            partTypes=[TRIANGLE_FAN, TRIANGLE_FAN])
    w.record('record1', 'FAN')
    w.close()

    # 読み込み対象のファイルパス
    read_path = filepath + '.shp'

    # 期待値取得
    # ペイロードの期待値を定義
    shapefile_record_list = [['record1', 'FAN']]

    # fieldの期待値を定義
    shapefile_field_list = [['name', 'C', 50, 0], ['type', 'C', 50, 0]]

    # TRIANGLE_FANのSpatialPointオブジェクトを生成
    spatial_fan_point_list_01 = _get_spatial_point_objects(fan_point_list_01)
    spatial_fan_point_list_02 = _get_spatial_point_objects(fan_point_list_02)

    # SpatialTriangleオブジェクト生成
    spatial_triangle_list_01 = \
        _shaping_point_to_triangle(
                spatial_fan_point_list_01, fan_point_type_01)
    spatial_triangle_list_02 = \
        _shaping_point_to_triangle(
                spatial_fan_point_list_02, fan_point_type_02)

    # 空間ID取得対象のSpatialTriangleを格納した配列を定義
    triangle_list = spatial_triangle_list_01 + spatial_triangle_list_02

    # 空間IDの期待値を定義
    shapefile_spatial_id_list = [get_spatial_ids_on_polygons(
              triangle_list, [], h_zoom, v_zoom)]

    # 試験実施
    # 戻り値取得
    param_return = read_shapefile_bulk(
            read_path, encoding, h_zoom, v_zoom, shapefile_crs)

    # 戻り値確認
    loop_cnt = 0

    # 空間IDの要素数確認
    param_return_spatial_id_list = param_return[0]
    assert len(shapefile_spatial_id_list) == len(param_return_spatial_id_list)

    # フィールドの要素一致確認
    assert shapefile_field_list == param_return[1]

    # ペイロードの要素一致確認
    assert shapefile_record_list == param_return[2]

    for ids in param_return_spatial_id_list:
        # APIから取得した空間IDは順不同のため比較前にソートし、格納された空間IDが一致していることを確認する。
        param_assert_ids = sorted(shapefile_spatial_id_list[loop_cnt])
        param_return_ids = sorted(ids)

        # 戻り値確認 - 空間ID
        assert param_assert_ids == param_return_ids

        loop_cnt += 1


def test_read_shapefile_bulk_11(geographic_shape_point_data_01):
    """強化試験：Shapefileから全レコードの空間ID情報を一括で取得する、読み込みファイル拡張子が.shp
    + 試験詳細
      - 入力条件
        - 読み込みファイル拡張子が.dbf
      - 確認内容
        - Shapefileから全レコードの空間ID情報を一括で取得できること
    """
    # 試験用データの定義
    # 試験用shalefileパス
    filepath = SHAPEFILE_DIR + 'test_read_shapefile_bulk_11'

    # encoding
    encoding = 'utf-8'

    # 水平、垂直方向精度を定義
    h_zoom = 6
    v_zoom = 6

    # shapefileに格納するデータの座標系
    shapefile_crs = 4326

    # shapefile作成用の座標配列取得
    fan_point_list_01 = geographic_shape_point_data_01[0][0]
    fan_point_type_01 = geographic_shape_point_data_01[1][0]
    fan_point_list_02 = geographic_shape_point_data_01[0][1]
    fan_point_type_02 = geographic_shape_point_data_01[1][1]

    # shapefile生成
    w = sf.Writer(filepath)
    w.field('name', 'C')
    w.field('type', 'C')
    w.multipatch(
            [fan_point_list_01, fan_point_list_02],
            partTypes=[TRIANGLE_FAN, TRIANGLE_FAN])
    w.record('record1', 'FAN')
    w.close()

    # 読み込み対象のファイルパス
    read_path = filepath + '.dbf'

    # 期待値取得
    # ペイロードの期待値を定義
    shapefile_record_list = [['record1', 'FAN']]

    # fieldの期待値を定義
    shapefile_field_list = [['name', 'C', 50, 0], ['type', 'C', 50, 0]]

    # TRIANGLE_FANのSpatialPointオブジェクトを生成
    spatial_fan_point_list_01 = _get_spatial_point_objects(fan_point_list_01)
    spatial_fan_point_list_02 = _get_spatial_point_objects(fan_point_list_02)

    # SpatialTriangleオブジェクト生成
    spatial_triangle_list_01 = \
        _shaping_point_to_triangle(
                spatial_fan_point_list_01, fan_point_type_01)
    spatial_triangle_list_02 = \
        _shaping_point_to_triangle(
                spatial_fan_point_list_02, fan_point_type_02)

    # 空間ID取得対象のSpatialTriangleを格納した配列を定義
    triangle_list = spatial_triangle_list_01 + spatial_triangle_list_02

    # 空間IDの期待値を定義
    shapefile_spatial_id_list = [get_spatial_ids_on_polygons(
              triangle_list, [], h_zoom, v_zoom)]

    # 試験実施
    # 戻り値取得
    param_return = read_shapefile_bulk(
            read_path, encoding, h_zoom, v_zoom, shapefile_crs)

    # 戻り値確認
    loop_cnt = 0

    # 空間IDの要素数確認
    param_return_spatial_id_list = param_return[0]
    assert len(shapefile_spatial_id_list) == len(param_return_spatial_id_list)

    # フィールドの要素一致確認
    assert shapefile_field_list == param_return[1]

    # ペイロードの要素一致確認
    assert shapefile_record_list == param_return[2]

    for ids in param_return_spatial_id_list:
        # APIから取得した空間IDは順不同のため比較前にソートし、格納された空間IDが一致していることを確認する。
        param_assert_ids = sorted(shapefile_spatial_id_list[loop_cnt])
        param_return_ids = sorted(ids)

        # 戻り値確認 - 空間ID
        assert param_assert_ids == param_return_ids

        loop_cnt += 1


def test_read_shapefile_bulk_12():
    """強化試験：Shapefileから全レコードの空間ID情報を一括で取得する、.shpのみが存在しない場合
    + 試験詳細
      - 入力条件
        - 読み込み対象のshapefileの内、".shp"のみが存在しない
      - 確認内容
        - shapefile読み込み失敗の例外が発生すること
    """
    # 試験用データの定義
    # 試験用shalefileパス
    filepath = SHAPEFILE_DIR + 'test_read_shapefile_bulk_12'

    # encoding
    encoding = 'utf-8'

    # 水平、垂直方向精度を定義
    h_zoom = 1
    v_zoom = 1

    # shapefileに格納するデータの座標系
    shapefile_crs = 4326

    # Pointの座標を格納したリストを定義
    strip_point_01 = (139.051685, 45.060433, 150.5)
    strip_point_02 = (140.051685, 46.060433, 151.5)
    strip_point_03 = (141.051685, 47.060433, 152.5)
    strip_point_04 = (142.051685, 48.060433, 151.5)

    # shapefile作成用の座標配列
    strip_point_list = [
            strip_point_01,
            strip_point_02,
            strip_point_03,
            strip_point_04
            ]

    # shapefile生成
    w = sf.Writer(filepath)
    w.field('name', 'C')
    w.field('type', 'C')
    w.multipatch(
            [strip_point_list],
            partTypes=[TRIANGLE_STRIP])
    w.record('record1', 'STRIP')
    w.close()

    # 読み込み対象のshapefileの内.shpファイルを削除
    remove_shapefile_path = filepath + '.shp'
    os.remove(remove_shapefile_path)

    # 期待値取得
    # 例外取得
    exception_assert = SpatialIdError('SHAPEFILE_READ_ERROR')

    # 試験実施
    # 例外発生確認
    with pytest.raises(SpatialIdError) as e:
        read_shapefile_bulk(
            filepath, encoding, h_zoom, v_zoom, shapefile_crs)

    # 例外メッセージ確認
    assert str(exception_assert) == str(e.value)


def test_read_shapefile_bulk_13():
    """強化試験：Shapefileから全レコードの空間ID情報を一括で取得する、.dbfのみが存在しない場合
    + 試験詳細
      - 入力条件
        - 読み込み対象のshapefileの内、".dbf"のみが存在しない
      - 確認内容
        - shapefile読み込み失敗の例外が発生すること
    """
    # 試験用データの定義
    # 試験用shalefileパス
    filepath = SHAPEFILE_DIR + 'test_read_shapefile_bulk_13'

    # encoding
    encoding = 'utf-8'

    # 水平、垂直方向精度を定義
    h_zoom = 1
    v_zoom = 1

    # shapefileに格納するデータの座標系
    shapefile_crs = 4326

    # Pointの座標を格納したリストを定義
    strip_point_01 = (139.051685, 45.060433, 150.5)
    strip_point_02 = (140.051685, 46.060433, 151.5)
    strip_point_03 = (141.051685, 47.060433, 152.5)
    strip_point_04 = (142.051685, 48.060433, 151.5)

    # shapefile作成用の座標配列
    strip_point_list = [
            strip_point_01,
            strip_point_02,
            strip_point_03,
            strip_point_04
            ]

    # shapefile生成
    w = sf.Writer(filepath)
    w.field('name', 'C')
    w.field('type', 'C')
    w.multipatch(
            [strip_point_list],
            partTypes=[TRIANGLE_STRIP])
    w.record('record1', 'STRIP')
    w.close()

    # 読み込み対象のshapefileの内.shpファイルを削除
    remove_shapefile_path = filepath + '.dbf'
    os.remove(remove_shapefile_path)

    # 期待値取得
    # 例外取得
    exception_assert = SpatialIdError('SHAPEFILE_READ_ERROR')

    # 試験実施
    # 例外発生確認
    with pytest.raises(SpatialIdError) as e:
        read_shapefile_bulk(
            filepath, encoding, h_zoom, v_zoom, shapefile_crs)

    # 例外メッセージ確認
    assert str(exception_assert) == str(e.value)


def test_read_shapefile_bulk_14(geographic_shape_point_data_01):
    """強化試験：Shapefileから全レコードの空間ID情報を一括で取得する、入力CRSが地理座標系
    + 試験詳細
      - 入力条件
        - ユーザ入力のCRSに地理座標系を入力
      - 確認内容
        - Shapefileから全レコードの空間ID情報を一括で取得できること
    """
    # 試験用データの定義
    # 試験用shalefileパス
    filepath = SHAPEFILE_DIR + 'test_read_shapefile_bulk_14'

    # encoding
    encoding = 'utf-8'

    # 水平、垂直方向精度を定義
    h_zoom = 6
    v_zoom = 6

    # shapefileに格納するデータの座標系
    shapefile_crs = 4612

    # shapefile作成用の座標配列取得
    fan_point_list_01 = geographic_shape_point_data_01[0][0]
    fan_point_type_01 = geographic_shape_point_data_01[1][0]
    fan_point_list_02 = geographic_shape_point_data_01[0][1]
    fan_point_type_02 = geographic_shape_point_data_01[1][1]

    # shapefile生成
    w = sf.Writer(filepath)
    w.field('name', 'C')
    w.field('type', 'C')
    w.multipatch(
            [fan_point_list_01, fan_point_list_02],
            partTypes=[TRIANGLE_FAN, TRIANGLE_FAN])
    w.record('record1', 'FAN')
    w.close()

    # 期待値取得
    # ペイロードの期待値を定義
    shapefile_record_list = [['record1', 'FAN']]

    # fieldの期待値を定義
    shapefile_field_list = [['name', 'C', 50, 0], ['type', 'C', 50, 0]]

    # TRIANGLE_FANのSpatialPointオブジェクトを生成
    spatial_fan_point_list_01 = _get_spatial_point_objects(fan_point_list_01)
    spatial_fan_point_list_02 = _get_spatial_point_objects(fan_point_list_02)

    # SpatialTriangleオブジェクト生成
    spatial_triangle_list_01 = \
        _shaping_point_to_triangle(
                spatial_fan_point_list_01, fan_point_type_01)
    spatial_triangle_list_02 = \
        _shaping_point_to_triangle(
                spatial_fan_point_list_02, fan_point_type_02)

    # 空間ID取得対象のSpatialTriangleを格納した配列を定義
    triangle_list = spatial_triangle_list_01 + spatial_triangle_list_02

    # 空間IDの期待値を定義
    shapefile_spatial_id_list = [get_spatial_ids_on_polygons(
              triangle_list, [], h_zoom, v_zoom)]

    # 試験実施
    # 戻り値取得
    param_return = read_shapefile_bulk(
            filepath, encoding, h_zoom, v_zoom, shapefile_crs)

    # 戻り値確認
    loop_cnt = 0

    # 空間IDの要素数確認
    param_return_spatial_id_list = param_return[0]
    assert len(shapefile_spatial_id_list) == len(param_return_spatial_id_list)

    # フィールドの要素一致確認
    assert shapefile_field_list == param_return[1]

    # ペイロードの要素一致確認
    assert shapefile_record_list == param_return[2]

    for ids in param_return_spatial_id_list:
        # APIから取得した空間IDは順不同のため比較前にソートし、格納された空間IDが一致していることを確認する。
        param_assert_ids = sorted(shapefile_spatial_id_list[loop_cnt])
        param_return_ids = sorted(ids)

        # 戻り値確認 - 空間ID
        assert param_assert_ids == param_return_ids

        loop_cnt += 1


def test_read_shapefile_bulk_15(projected_shape_point_data_01):
    """強化試験：Shapefileから全レコードの空間ID情報を一括で取得する、入力CRSが投影座標系
    + 試験詳細
      - 入力条件
        - ユーザ入力のCRSに投影座標系を入力
      - 確認内容
        - Shapefileから全レコードの空間ID情報を一括で取得できること
    """
    # 試験用データの定義
    # 試験用shalefileパス
    filepath = SHAPEFILE_DIR + 'test_read_shapefile_bulk_15'

    # encoding
    encoding = 'utf-8'

    # 水平、垂直方向精度を定義
    h_zoom = 10
    v_zoom = 10

    # shapefileに格納するデータの座標系
    shapefile_crs = 6677

    # shapefile作成用の座標配列取得
    strip_point_list_01 = projected_shape_point_data_01[0][0]
    strip_point_type_01 = projected_shape_point_data_01[1][0]
    strip_point_list_02 = projected_shape_point_data_01[0][1]
    strip_point_type_02 = projected_shape_point_data_01[1][1]
    fan_point_list_01 = projected_shape_point_data_01[0][2]
    fan_point_type_01 = projected_shape_point_data_01[1][2]

    # shapefile生成
    w = sf.Writer(filepath)
    w.field('name', 'C')
    w.field('type', 'C')
    w.multipatch(
            [strip_point_list_01, strip_point_list_02, fan_point_list_01],
            partTypes=[TRIANGLE_STRIP, TRIANGLE_STRIP, TRIANGLE_FAN])
    w.record('record1', 'STRIP, FAN')
    w.close()

    # 期待値取得
    # ペイロードの期待値を定義
    shapefile_record_list = [['record1', 'STRIP, FAN']]

    # fieldの期待値を定義
    shapefile_field_list = [['name', 'C', 50, 0], ['type', 'C', 50, 0]]

    # TRIANGLE_STRIPのSpatialPointオブジェクトを生成
    spatial_strip_point_list_01 = \
        _get_spatial_point_objects(strip_point_list_01, shapefile_crs)
    spatial_strip_point_list_02 = \
        _get_spatial_point_objects(strip_point_list_02, shapefile_crs)

    # TRIANGLE_FANのSpatialPointオブジェクトを生成
    spatial_fan_point_list_01 = \
        _get_spatial_point_objects(fan_point_list_01, shapefile_crs)

    # SpatialTriangleオブジェクト生成
    spatial_triangle_list_01 = \
        _shaping_point_to_triangle(
                spatial_strip_point_list_01, strip_point_type_01)
    spatial_triangle_list_02 = \
        _shaping_point_to_triangle(
                spatial_strip_point_list_02, strip_point_type_02)
    spatial_triangle_list_03 = \
        _shaping_point_to_triangle(
                spatial_fan_point_list_01, fan_point_type_01)

    # 空間ID取得対象のSpatialTriangleを格納した配列を定義
    triangle_list = spatial_triangle_list_01 + spatial_triangle_list_02\
        + spatial_triangle_list_03

    # 空間IDの期待値を定義
    shapefile_spatial_id_list = [get_spatial_ids_on_polygons(
              triangle_list, [], h_zoom, v_zoom)]

    # 試験実施
    # 戻り値取得
    param_return = read_shapefile_bulk(
            filepath, encoding, h_zoom, v_zoom, shapefile_crs)

    # 戻り値確認
    loop_cnt = 0

    # 空間IDの要素数確認
    param_return_spatial_id_list = param_return[0]
    assert len(shapefile_spatial_id_list) == len(param_return_spatial_id_list)

    # フィールドの要素一致確認
    assert shapefile_field_list == param_return[1]

    # ペイロードの要素一致確認
    assert shapefile_record_list == param_return[2]

    for ids in param_return_spatial_id_list:
        # APIから取得した空間IDは順不同のため比較前にソートし、格納された空間IDが一致していることを確認する。
        param_assert_ids = sorted(shapefile_spatial_id_list[loop_cnt])
        param_return_ids = sorted(ids)

        # 戻り値確認 - 空間ID
        assert param_assert_ids == param_return_ids

        loop_cnt += 1


def test_read_shapefile_bulk_16():
    """強化試験：Shapefileから全レコードの空間ID情報を一括で取得する、引数のfilepath入力チェックNG
    + 試験詳細
      - 入力条件
        - filepathにint型の値を入力
      - 確認内容
        - 入力値チェックエラーの例外が発生すること
    """
    # 期待値定義
    # 例外を取得
    exception_assert = SpatialIdError('INPUT_VALUE_ERROR')

    # 試験実施
    # 例外発生を確認
    with pytest.raises(SpatialIdError) as e:
        read_shapefile_bulk(100, "utf-8", 1, 1, 4326)

    assert str(exception_assert) == str(e.value)


def test_read_shapefile_bulk_17():
    """強化試験：Shapefileから全レコードの空間ID情報を一括で取得する、引数のfilepathに空文字を入力
    + 試験詳細
      - 入力条件
        - filepathに空文字を入力
      - 確認内容
        - Shapefile読み込みエラーの例外が発生すること
    """
    # 期待値定義
    # 例外を取得
    exception_assert = SpatialIdError('SHAPEFILE_READ_ERROR')

    # 試験実施
    # 例外発生を確認
    with pytest.raises(SpatialIdError) as e:
        read_shapefile_bulk("", "utf-8", 1, 1, 4326)

    assert str(exception_assert) == str(e.value)


def test_read_shapefile_bulk_18():
    """強化試験：Shapefileから全レコードの空間ID情報を一括で取得する、引数のfilepathにNoneを入力
    + 試験詳細
      - 入力条件
        - filepathにNoneを入力
      - 確認内容
        - 入力値チェックエラーの例外が発生すること
    """
    # 期待値定義
    # 例外を取得
    exception_assert = SpatialIdError('INPUT_VALUE_ERROR')

    # 試験実施
    # 例外発生を確認
    with pytest.raises(SpatialIdError) as e:
        read_shapefile_bulk(None, "utf-8", 1, 1, 4326)

    assert str(exception_assert) == str(e.value)


def test_read_shapefile_bulk_19():
    """強化試験：Shapefileから全レコードの空間ID情報を一括で取得する、引数のencoding入力チェックNG
    + 試験詳細
      - 入力条件
        - encodingにint型の値を入力
      - 確認内容
        - 入力値チェックエラーの例外が発生すること
    """
    # 期待値定義
    # 例外を取得
    exception_assert = SpatialIdError('INPUT_VALUE_ERROR')

    # 試験実施
    # 例外発生を確認
    with pytest.raises(SpatialIdError) as e:
        read_shapefile_bulk("test/dir", 100, 1, 1, 4326)

    assert str(exception_assert) == str(e.value)


def test_read_shapefile_bulk_20():
    """強化試験：Shapefileから全レコードの空間ID情報を一括で取得する、引数のencodingに空文字を入力
    + 試験詳細
      - 入力条件
        - encodingに空文字を入力
      - 確認内容
        - Shapefile読み込みエラーの例外が発生すること
    """
    # 期待値定義
    # 例外を取得
    exception_assert = SpatialIdError('SHAPEFILE_READ_ERROR')

    # 試験実施
    # 例外発生を確認
    with pytest.raises(SpatialIdError) as e:
        read_shapefile_bulk("test/dir", "", 1, 1, 4326)

    assert str(exception_assert) == str(e.value)


def test_read_shapefile_bulk_21():
    """強化試験：Shapefileから全レコードの空間ID情報を一括で取得する、引数のencodingにNoneを入力
    + 試験詳細
      - 入力条件
        - encodingにNoneを入力
      - 確認内容
        - 入力値チェックエラーの例外が発生すること
    """
    # 期待値定義
    # 例外を取得
    exception_assert = SpatialIdError('INPUT_VALUE_ERROR')

    # 試験実施
    # 例外発生を確認
    with pytest.raises(SpatialIdError) as e:
        read_shapefile_bulk("test/dir", None, 1, 1, 4326)

    assert str(exception_assert) == str(e.value)


def test_read_shapefile_bulk_22():
    """強化試験：Shapefileから全レコードの空間ID情報を一括で取得する、引数のh_zoom入力チェックNG
    + 試験詳細
      - 入力条件
        - h_zoomにstr型の値を入力
      - 確認内容
        - 入力値チェックエラーの例外が発生すること
    """
    # 期待値定義
    # 例外を取得
    exception_assert = SpatialIdError('INPUT_VALUE_ERROR')

    # 試験実施
    # 例外発生を確認
    with pytest.raises(SpatialIdError) as e:
        read_shapefile_bulk("test/dir", "utf-8", "1", 1, 4326)

    assert str(exception_assert) == str(e.value)


def test_read_shapefile_bulk_23():
    """強化試験：Shapefileから全レコードの空間ID情報を一括で取得する、引数のh_zoomにNoneを入力
    + 試験詳細
      - 入力条件
        - h_zoomにNoneを入力
      - 確認内容
        - 入力値チェックエラーの例外が発生すること
    """
    # 期待値定義
    # 例外を取得
    exception_assert = SpatialIdError('INPUT_VALUE_ERROR')

    # 試験実施
    # 例外発生を確認
    with pytest.raises(SpatialIdError) as e:
        read_shapefile_bulk("test/dir", "utf-8", None, 1, 4326)

    assert str(exception_assert) == str(e.value)


def test_read_shapefile_bulk_24():
    """強化試験：Shapefileから全レコードの空間ID情報を一括で取得する、引数のv_zoom入力チェックNG
    + 試験詳細
      - 入力条件
        - v_zoomにstr型の値を入力
      - 確認内容
        - 入力値チェックエラーの例外が発生すること
    """
    # 期待値定義
    # 例外を取得
    exception_assert = SpatialIdError('INPUT_VALUE_ERROR')

    # 試験実施
    # 例外発生を確認
    with pytest.raises(SpatialIdError) as e:
        read_shapefile_bulk("test/dir", "utf-8", 1, "1", 4326)

    assert str(exception_assert) == str(e.value)


def test_read_shapefile_bulk_25():
    """強化試験：Shapefileから全レコードの空間ID情報を一括で取得する、引数のv_zoomにNoneを入力
    + 試験詳細
      - 入力条件
        - v_zoomにNoneを入力
      - 確認内容
        - 入力値チェックエラーの例外が発生すること
    """
    # 期待値定義
    # 例外を取得
    exception_assert = SpatialIdError('INPUT_VALUE_ERROR')

    # 試験実施
    # 例外発生を確認
    with pytest.raises(SpatialIdError) as e:
        read_shapefile_bulk("test/dir", "utf-8", 1, None, 4326)

    assert str(exception_assert) == str(e.value)


def test_read_shapefile_bulk_26():
    """強化試験：Shapefileから全レコードの空間ID情報を一括で取得する、引数のcrs入力チェックNG
    + 試験詳細
      - 入力条件
        - crsにstr型の値を入力
      - 確認内容
        - 入力値チェックエラーの例外が発生すること
    """
    # 期待値定義
    # 例外を取得
    exception_assert = SpatialIdError('INPUT_VALUE_ERROR')

    # 試験実施
    # 例外発生を確認
    with pytest.raises(SpatialIdError) as e:
        read_shapefile_bulk("test/dir", "utf-8", 1, 1, "4326")

    assert str(exception_assert) == str(e.value)


def test_read_shapefile_bulk_27():
    """強化試験：Shapefileから全レコードの空間ID情報を一括で取得する、引数のcrsにNoneを入力
    + 試験詳細
      - 入力条件
        - crsにNoneを入力
      - 確認内容
        - 入力値チェックエラーの例外が発生すること
    """
    # 期待値定義
    # 例外を取得
    exception_assert = SpatialIdError('INPUT_VALUE_ERROR')

    # 試験実施
    # 例外発生を確認
    with pytest.raises(SpatialIdError) as e:
        read_shapefile_bulk("test/dir", "utf-8", 1, 1, None)

    assert str(exception_assert) == str(e.value)


def test_read_shapefile_bulk_28(geographic_shape_point_data_04):
    """Shapefileから全レコードの空間ID情報を一括で取得する、閉塞していないポリゴン、閉塞チェックON
    + 試験詳細
      - 入力条件
        - ポリゴンの閉塞チェックフラグON
        - 読み込み対象のshapefileのポリゴンが閉塞していない
      - 確認内容
        - 三角形ポリゴンのモデルが閉塞していない場合の例外が発生すること
    """
    # 試験用データの定義
    # 試験用shalefileパス
    filepath = SHAPEFILE_DIR + 'test_read_shapefile_bulk_28'

    # encoding
    encoding = 'utf-8'

    # 水平、垂直方向精度を定義
    h_zoom = 1
    v_zoom = 1

    # shapefileに格納するデータの座標系
    shapefile_crs = 4326

    # shapefile作成用の座標配列取得
    fan_point_list_01 = geographic_shape_point_data_04[0][0]
    fan_point_list_02 = geographic_shape_point_data_04[0][1]

    # shapefile生成
    w = sf.Writer(filepath)
    w.field('name', 'C')
    w.field('type', 'C')
    w.multipatch(
            [fan_point_list_01, fan_point_list_02],
            partTypes=[TRIANGLE_FAN, TRIANGLE_FAN])
    w.record('record1', 'FAN')
    w.close()

    # 期待値取得
    # 例外取得
    exception_assert = SpatialIdError('POLYGON_NOT_CLOSED_MODEL')

    # 試験実施
    # 例外発生確認
    with pytest.raises(SpatialIdError) as e:
        read_shapefile_bulk(
            filepath, encoding, h_zoom, v_zoom, shapefile_crs)

    # 例外メッセージ確認
    assert str(exception_assert) == str(e.value)


def test_read_shapefile_bulk_29(geographic_shape_point_data_04):
    """Shapefileから全レコードの空間ID情報を一括で取得する、閉塞していないポリゴン、閉塞チェックOFF
    + 試験詳細
      - 入力条件
        - ポリゴンの閉塞チェックフラグOFF
        - 閉塞していないモデルを含むshapefile読み込み
      - 確認内容
        - Shapefileから全レコードの空間ID情報を一括で取得できること
    """
    # 試験用データの定義
    # 試験用shalefileパス
    filepath = SHAPEFILE_DIR + 'test_read_shapefile_bulk_29'

    # encoding
    encoding = 'utf-8'

    # 水平、垂直方向精度を定義
    h_zoom = 10
    v_zoom = 10

    # shapefileに格納するデータの座標系
    shapefile_crs = 4326

    # shapefile作成用の座標配列取得
    fan_point_list_01 = geographic_shape_point_data_04[0][0]
    fan_point_type_01 = geographic_shape_point_data_04[1][0]
    fan_point_list_02 = geographic_shape_point_data_04[0][1]
    fan_point_type_02 = geographic_shape_point_data_04[1][1]

    # shapefile生成
    w = sf.Writer(filepath)
    w.field('name', 'C')
    w.field('type', 'C')
    w.multipatch(
            [fan_point_list_01, fan_point_list_02],
            partTypes=[TRIANGLE_FAN, TRIANGLE_FAN])
    w.record('record1', 'FAN')
    w.close()

    # 期待値取得
    # ペイロードの期待値を定義
    shapefile_record_list = [['record1', 'FAN']]

    # fieldの期待値を定義
    shapefile_field_list = [['name', 'C', 50, 0], ['type', 'C', 50, 0]]

    # TRIANGLE_FANのSpatialPointオブジェクトを生成
    spatial_fan_point_list_01 = _get_spatial_point_objects(fan_point_list_01)
    spatial_fan_point_list_02 = _get_spatial_point_objects(fan_point_list_02)

    # SpatialTriangleオブジェクト生成
    spatial_triangle_list_01 = \
        _shaping_point_to_triangle(
                spatial_fan_point_list_01, fan_point_type_01)
    spatial_triangle_list_02 = \
        _shaping_point_to_triangle(
                spatial_fan_point_list_02, fan_point_type_02)

    # 空間ID取得対象のSpatialTriangleを格納した配列を定義
    triangle_list = spatial_triangle_list_01 + spatial_triangle_list_02

    # 空間IDの期待値を定義
    shapefile_spatial_id_list = [get_spatial_ids_on_polygons(
              triangle_list, [], h_zoom, v_zoom, needs_closed_checking=False)]

    # 試験実施
    # 戻り値取得
    param_return = read_shapefile_bulk(
            filepath, encoding, h_zoom, v_zoom, shapefile_crs, False)

    # 戻り値確認
    loop_cnt = 0

    # 空間IDの要素数確認
    param_return_spatial_id_list = param_return[0]
    assert len(shapefile_spatial_id_list) == len(param_return_spatial_id_list)

    # フィールドの要素一致確認
    assert shapefile_field_list == param_return[1]

    # ペイロードの要素一致確認
    assert shapefile_record_list == param_return[2]

    for ids in param_return_spatial_id_list:
        # APIから取得した空間IDは順不同のため比較前にソートし、格納された空間IDが一致していることを確認する。
        param_assert_ids = sorted(shapefile_spatial_id_list[loop_cnt])
        param_return_ids = sorted(ids)

        # 戻り値確認 - 空間ID
        assert param_assert_ids == param_return_ids

        loop_cnt += 1

def test_f_read_shapefile_bulk_with_aggregation_01(
        geographic_shape_point_data_01,
        geographic_shape_point_data_02
):
    """Shapefileの空間ID集約取得、集約関数指定なし、shapefileレコード数2、CSR入力なし
    + 試験詳細
      - 入力条件
        - 引数の集約関数入力なし
        - shapefileレコード数2
        - 読み込み対象ファイル拡張子無し
        - CSR入力なし
      - 確認内容
        - 戻り値のペイロードは、デフォルトの形式で集約されていること。
        - shapefileの全レコード文の空間IDに対し、集約が行われていること。
    """
    # 試験用データの定義
    # 試験用shalefileパス
    filepath = SHAPEFILE_DIR + 'test_f_read_shapefile_bulk_with_aggregation_01'

    # encoding
    encoding = 'utf-8'

    # 水平、垂直方向精度を定義
    h_zoom = 1
    v_zoom = 1

    # shapefile作成用の座標配列取得
    fan_point_list_01 = geographic_shape_point_data_01[0][0]
    fan_point_type_01 = geographic_shape_point_data_01[1][0]
    fan_point_list_02 = geographic_shape_point_data_01[0][1]
    fan_point_type_02 = geographic_shape_point_data_01[1][1]

    fan_point_list_03 = geographic_shape_point_data_02[0][0]
    fan_point_type_03 = geographic_shape_point_data_02[1][0]
    fan_point_list_04 = geographic_shape_point_data_02[0][1]
    fan_point_type_04 = geographic_shape_point_data_02[1][1]

    # shapefile生成
    w = sf.Writer(filepath)
    w.field('name', 'C')
    w.field('type', 'C')
    w.multipatch(
            [fan_point_list_01, fan_point_list_02],
            partTypes=[TRIANGLE_FAN, TRIANGLE_FAN])
    w.record('record1', 'FAN')
    w.multipatch(
            [fan_point_list_03, fan_point_list_04],
            partTypes=[TRIANGLE_FAN, TRIANGLE_FAN])
    w.record('record2', 'FAN')
    w.close()

    # 期待値取得
    # ペイロードの期待値を定義
    shapefile_record_list = [['record1', 'FAN'], ['record2', 'FAN']]

    # fieldの期待値を定義
    shapefile_field_list = [['name', 'C', 50, 0], ['type', 'C', 50, 0]]

    # TRIANGLE_FANのSpatialPointオブジェクトを生成
    spatial_fan_point_list_01 = _get_spatial_point_objects(fan_point_list_01)
    spatial_fan_point_list_02 = _get_spatial_point_objects(fan_point_list_02)
    spatial_fan_point_list_03 = _get_spatial_point_objects(fan_point_list_03)
    spatial_fan_point_list_04 = _get_spatial_point_objects(fan_point_list_04)

    # SpatialTriangleオブジェクト生成
    spatial_triangle_list_01 = \
        _shaping_point_to_triangle(
                spatial_fan_point_list_01, fan_point_type_01)
    spatial_triangle_list_02 = \
        _shaping_point_to_triangle(
                spatial_fan_point_list_02, fan_point_type_02)
    spatial_triangle_list_03 = \
        _shaping_point_to_triangle(
                spatial_fan_point_list_03, fan_point_type_03)
    spatial_triangle_list_04 = \
        _shaping_point_to_triangle(
                spatial_fan_point_list_04, fan_point_type_04)

    # 空間ID取得対象のSpatialTriangleを格納した配列を定義
    triangle_list_01 = spatial_triangle_list_01 + spatial_triangle_list_02
    triangle_list_02 = spatial_triangle_list_03 + spatial_triangle_list_04

    # 空間IDの期待値を定義
    shapefile_spatial_ids_01 = f_get_spatial_ids_on_polygons(
            triangle_list_01, [], h_zoom)
    shapefile_spatial_ids_02 = f_get_spatial_ids_on_polygons(
            triangle_list_02, [], h_zoom)
    shapefile_spatial_id_list = [
            shapefile_spatial_ids_01,
            shapefile_spatial_ids_02]

    # 集約後のペイロードの期待値を定義
    shapefile_record_list_agg = {}

    for ids, payloads in zip(shapefile_spatial_id_list, shapefile_record_list):

        for sp_id in ids:
            # 空間IDをキー値として未定義の場合、初期化
            if sp_id not in shapefile_record_list_agg:
                shapefile_record_list_agg[sp_id] = []

            # 辞書に、空間IDをキー、値をペイロードとして格納
            shapefile_record_list_agg[sp_id].append(payloads)

    # 試験実施
    # 戻り値取得
    param_return = f_read_shapefile_bulk_with_aggregation(
            filepath, encoding, h_zoom)

    # 戻り値確認
    loop_cnt = 0

    # 空間IDの要素数確認
    param_return_spatial_id_list = param_return[0]
    assert len(shapefile_spatial_id_list) == len(param_return_spatial_id_list)

    # フィールドの要素一致確認
    assert shapefile_field_list == param_return[1]

    # 集約後のペイロードの要素一致確認
    param_return_payload = param_return[2]
    assert shapefile_record_list_agg == param_return_payload

    # 空間IDの値確認
    for ids in param_return_spatial_id_list:
        # APIから取得した空間IDは順不同のため比較前にソートし、格納された空間IDが一致していることを確認する。
        param_assert_ids = sorted(shapefile_spatial_id_list[loop_cnt])
        param_return_ids = sorted(ids)

        # 戻り値確認 - 空間ID
        assert param_assert_ids == param_return_ids

        # 全ての空間IDに対し集約が行われていることを確認
        for target_id in param_return_ids:
            assert True == (target_id in param_return_payload)

        loop_cnt += 1


def test_f_read_shapefile_bulk_with_aggregation_02(
        geographic_shape_point_data_01
):
    """Shapefileの空間ID集約取得、入力した集約関数で例外発生
    """
    # 試験用データの定義
    # 試験用shalefileパス
    filepath = SHAPEFILE_DIR + 'test_f_read_shapefile_bulk_with_aggregation_02'

    # encoding
    encoding = 'utf-8'

    # 精度を定義
    zoom = 10

    # shapefileに格納するデータの座標系
    shapefile_crs = 4326

    # shapefile作成用の座標配列取得
    fan_point_list_01 = geographic_shape_point_data_01[0][0]
    fan_point_list_02 = geographic_shape_point_data_01[0][1]

    # shapefile生成
    w = sf.Writer(filepath)
    w.field('name', 'C')
    w.field('type', 'C')
    w.multipatch(
            [fan_point_list_01, fan_point_list_02],
            partTypes=[TRIANGLE_FAN, TRIANGLE_FAN])
    # 集約関数内で例外が発生する条件を満たす値を設定
    w.record('Exception', 'STRIP')
    w.close()

    # 期待値取得
    # 例外取得
    exception_assert = SpatialIdError('SHAPEFILE_AGGREGATION_FUNC_ERROR')

    # 試験実施
    # 例外発生確認
    with pytest.raises(SpatialIdError) as e:
        f_read_shapefile_bulk_with_aggregation(
                filepath,
                encoding,
                zoom,
                shapefile_crs,
                _custom_payload_func)

    # 例外メッセージ確認
    assert str(exception_assert) == str(e.value)


def test_f_read_shapefile_bulk_with_aggregation_03(
        mocker,
        geographic_shape_point_data_01
):
    """Shapefileの空間ID集約取得、その他例外の発生
     + 試験詳細
       - 試験条件
         - 通常発生しない例外パターンのためモックを、使用して例外を発生させる
       - 確認内容
         - その他例外が発生すること
    """
    # 試験用データの定義
    # 試験用shalefileパス
    filepath = SHAPEFILE_DIR + 'test_f_read_shapefile_bulk_with_aggregation_03'

    # encoding
    encoding = 'utf-8'

    # 水平、垂直方向精度を定義
    zoom = 10

    # shapefileに格納するデータの座標系
    shapefile_crs = 4326

    # shapefile作成用の座標配列取得
    fan_point_list_01 = geographic_shape_point_data_01[0][0]
    fan_point_list_02 = geographic_shape_point_data_01[0][1]

    # shapefile生成
    w = sf.Writer(filepath)
    w.field('name', 'C')
    w.field('type', 'C')
    w.multipatch(
            [fan_point_list_01, fan_point_list_02],
            partTypes=[TRIANGLE_FAN, TRIANGLE_FAN])
    w.record('record1', 'FAN')
    w.close()

    # 期待値取得
    # 例外取得
    exception_assert = SpatialIdError('OTHER_ERROR')

    mocker.patch(
        'SpatialId.io.shapefile.f_read_shapefile_bulk').\
        side_effect = Exception('予期せぬ例外が発生')

    # 試験実施
    with pytest.raises(SpatialIdError) as e:
        f_read_shapefile_bulk_with_aggregation(
                filepath,
                encoding,
                zoom,
                shapefile_crs,
                _custom_payload_func)

    # 例外メッセージ確認
    assert str(exception_assert) == str(e.value)


def test_f_read_shapefile_bulk_with_aggregation_04(
        geographic_shape_point_data_01
):
    """Shapefileの空間ID集約取得、集約関数指定あり、shapefileレコード数1
    + 試験詳細
      - 入力条件
        - 引数の集約関数入力あり
        - shapefileレコード数1
      - 確認内容
        - 戻り値のペイロードは、デフォルトの集約結果に対し、引数で指定した集約関数の処理が行われたものになっていること。
        - shapefileの全レコード文の空間IDに対し、集約が行われていること。
    """
    # 試験用データの定義
    # 試験用shalefileパス
    filepath = SHAPEFILE_DIR + 'test_f_read_shapefile_bulk_with_aggregation_04'

    # encoding
    encoding = 'utf-8'

    # 精度を定義
    zoom = 5

    # shapefileに格納するデータの座標系
    shapefile_crs = 4326

    # shapefile作成用の座標配列取得
    fan_point_list_01 = geographic_shape_point_data_01[0][0]
    fan_point_type_01 = geographic_shape_point_data_01[1][0]
    fan_point_list_02 = geographic_shape_point_data_01[0][1]
    fan_point_type_02 = geographic_shape_point_data_01[1][1]

    # shapefile生成
    w = sf.Writer(filepath)
    w.field('name', 'C')
    w.field('type', 'C')
    w.multipatch(
            [fan_point_list_01, fan_point_list_02],
            partTypes=[TRIANGLE_FAN, TRIANGLE_FAN])
    w.record('record1', 'FAN')
    w.close()

    # 期待値取得
    # ペイロードの期待値を定義
    shapefile_record_list = [['record1', 'FAN']]

    # fieldの期待値を定義
    shapefile_field_list = [['name', 'C', 50, 0], ['type', 'C', 50, 0]]

    # TRIANGLE_FANのSpatialPointオブジェクトを生成
    spatial_fan_point_list_01 = _get_spatial_point_objects(fan_point_list_01)
    spatial_fan_point_list_02 = _get_spatial_point_objects(fan_point_list_02)

    # SpatialTriangleオブジェクト生成
    spatial_triangle_list_01 = \
        _shaping_point_to_triangle(
                spatial_fan_point_list_01, fan_point_type_01)
    spatial_triangle_list_02 = \
        _shaping_point_to_triangle(
                spatial_fan_point_list_02, fan_point_type_02)

    # 空間ID取得対象のSpatialTriangleを格納した配列を定義
    triangle_list = spatial_triangle_list_01 + spatial_triangle_list_02

    # 空間IDの期待値を定義
    shapefile_spatial_id_list = [f_get_spatial_ids_on_polygons(
              triangle_list, [], zoom)]

    # 集約後のペイロードの期待値を定義
    shapefile_record_list_agg = {}

    for ids, payloads in zip(shapefile_spatial_id_list, shapefile_record_list):

        for sp_id in ids:
            # 空間IDをキー値として未定義の場合、初期化
            if sp_id not in shapefile_record_list_agg:
                shapefile_record_list_agg[sp_id] = []

            # 辞書に、空間IDをキー、値をペイロードとして格納
            shapefile_record_list_agg[sp_id].append(payloads)

    # 集約後のペイロードを集約関数でさらに成形
    shapefile_record_list_agg = _custom_payload_func(shapefile_record_list_agg)

    # 試験実施
    # 戻り値取得
    param_return = f_read_shapefile_bulk_with_aggregation(
            filepath,
            encoding,
            zoom,
            shapefile_crs,
            _custom_payload_func)

    # 戻り値確認
    loop_cnt = 0

    # 空間IDの要素数確認
    param_return_spatial_id_list = param_return[0]
    assert len(shapefile_spatial_id_list) == len(param_return_spatial_id_list)

    # フィールドの要素一致確認
    assert shapefile_field_list == param_return[1]

    # 集約後のペイロードの要素一致確認
    param_return_payload = param_return[2]
    assert shapefile_record_list_agg == param_return_payload

    for ids in param_return_spatial_id_list:
        # APIから取得した空間IDは順不同のため比較前にソートし、格納された空間IDが一致していることを確認する。
        param_assert_ids = sorted(shapefile_spatial_id_list[loop_cnt])
        param_return_ids = sorted(ids)

        # 戻り値確認 - 空間ID
        assert param_assert_ids == param_return_ids

        # 全ての空間IDに対し集約が行われていることを確認
        for target_id in param_return_ids:
            assert True == (target_id in param_return_payload)

        loop_cnt += 1


def test_read_shapefile_bulk_with_aggregation_01(
        geographic_shape_point_data_01,
        geographic_shape_point_data_02
):
    """Shapefileの空間ID集約取得、集約関数指定なし、shapefileレコード数2、CSR入力なし
    + 試験詳細
      - 入力条件
        - 引数の集約関数入力なし
        - shapefileレコード数2
        - 読み込み対象ファイル拡張子無し
        - CSR入力なし
      - 確認内容
        - 戻り値のペイロードは、デフォルトの形式で集約されていること。
        - shapefileの全レコード文の空間IDに対し、集約が行われていること。
    """
    # 試験用データの定義
    # 試験用shalefileパス
    filepath = SHAPEFILE_DIR + 'test_read_shapefile_bulk_with_aggregation_01'

    # encoding
    encoding = 'utf-8'

    # 水平、垂直方向精度を定義
    h_zoom = 1
    v_zoom = 1

    # shapefile作成用の座標配列取得
    fan_point_list_01 = geographic_shape_point_data_01[0][0]
    fan_point_type_01 = geographic_shape_point_data_01[1][0]
    fan_point_list_02 = geographic_shape_point_data_01[0][1]
    fan_point_type_02 = geographic_shape_point_data_01[1][1]

    fan_point_list_03 = geographic_shape_point_data_02[0][0]
    fan_point_type_03 = geographic_shape_point_data_02[1][0]
    fan_point_list_04 = geographic_shape_point_data_02[0][1]
    fan_point_type_04 = geographic_shape_point_data_02[1][1]

    # shapefile生成
    w = sf.Writer(filepath)
    w.field('name', 'C')
    w.field('type', 'C')
    w.multipatch(
            [fan_point_list_01, fan_point_list_02],
            partTypes=[TRIANGLE_FAN, TRIANGLE_FAN])
    w.record('record1', 'FAN')
    w.multipatch(
            [fan_point_list_03, fan_point_list_04],
            partTypes=[TRIANGLE_FAN, TRIANGLE_FAN])
    w.record('record2', 'FAN')
    w.close()

    # 期待値取得
    # ペイロードの期待値を定義
    shapefile_record_list = [['record1', 'FAN'], ['record2', 'FAN']]

    # fieldの期待値を定義
    shapefile_field_list = [['name', 'C', 50, 0], ['type', 'C', 50, 0]]

    # TRIANGLE_FANのSpatialPointオブジェクトを生成
    spatial_fan_point_list_01 = _get_spatial_point_objects(fan_point_list_01)
    spatial_fan_point_list_02 = _get_spatial_point_objects(fan_point_list_02)
    spatial_fan_point_list_03 = _get_spatial_point_objects(fan_point_list_03)
    spatial_fan_point_list_04 = _get_spatial_point_objects(fan_point_list_04)

    # SpatialTriangleオブジェクト生成
    spatial_triangle_list_01 = \
        _shaping_point_to_triangle(
                spatial_fan_point_list_01, fan_point_type_01)
    spatial_triangle_list_02 = \
        _shaping_point_to_triangle(
                spatial_fan_point_list_02, fan_point_type_02)
    spatial_triangle_list_03 = \
        _shaping_point_to_triangle(
                spatial_fan_point_list_03, fan_point_type_03)
    spatial_triangle_list_04 = \
        _shaping_point_to_triangle(
                spatial_fan_point_list_04, fan_point_type_04)

    # 空間ID取得対象のSpatialTriangleを格納した配列を定義
    triangle_list_01 = spatial_triangle_list_01 + spatial_triangle_list_02
    triangle_list_02 = spatial_triangle_list_03 + spatial_triangle_list_04

    # 空間IDの期待値を定義
    shapefile_spatial_ids_01 = get_spatial_ids_on_polygons(
            triangle_list_01, [], h_zoom, v_zoom)
    shapefile_spatial_ids_02 = get_spatial_ids_on_polygons(
            triangle_list_02, [], h_zoom, v_zoom)
    shapefile_spatial_id_list = [
            shapefile_spatial_ids_01,
            shapefile_spatial_ids_02]

    # 集約後のペイロードの期待値を定義
    shapefile_record_list_agg = {}

    for ids, payloads in zip(shapefile_spatial_id_list, shapefile_record_list):

        for sp_id in ids:
            # 空間IDをキー値として未定義の場合、初期化
            if sp_id not in shapefile_record_list_agg:
                shapefile_record_list_agg[sp_id] = []

            # 辞書に、空間IDをキー、値をペイロードとして格納
            shapefile_record_list_agg[sp_id].append(payloads)

    # 試験実施
    # 戻り値取得
    param_return = read_shapefile_bulk_with_aggregation(
            filepath, encoding, h_zoom, v_zoom)

    # 戻り値確認
    loop_cnt = 0

    # 空間IDの要素数確認
    param_return_spatial_id_list = param_return[0]
    assert len(shapefile_spatial_id_list) == len(param_return_spatial_id_list)

    # フィールドの要素一致確認
    assert shapefile_field_list == param_return[1]

    # 集約後のペイロードの要素一致確認
    param_return_payload = param_return[2]
    assert shapefile_record_list_agg == param_return_payload

    # 空間IDの値確認
    for ids in param_return_spatial_id_list:
        # APIから取得した空間IDは順不同のため比較前にソートし、格納された空間IDが一致していることを確認する。
        param_assert_ids = sorted(shapefile_spatial_id_list[loop_cnt])
        param_return_ids = sorted(ids)

        # 戻り値確認 - 空間ID
        assert param_assert_ids == param_return_ids

        # 全ての空間IDに対し集約が行われていることを確認
        for target_id in param_return_ids:
            assert True == (target_id in param_return_payload)

        loop_cnt += 1


def test_read_shapefile_bulk_with_aggregation_02(
        geographic_shape_point_data_01,
        geographic_shape_point_data_02
):
    """Shapefileの空間ID集約取得、集約関数指定あり、shapefileレコード数2
    + 試験詳細
      - 入力条件
        - 引数の集約関数入力あり
        - shapefileレコード数2
        - CSRにデフォルト値と同じ地理座標系(4326)を入力
      - 確認内容
        - 戻り値のペイロードは、デフォルトの集約結果に対し、引数で指定した集約関数の処理が行われたものになっていること。
        - shapefileの全レコード文の空間IDに対し、集約が行われていること。
    """
    # 試験用データの定義
    # 試験用shalefileパス
    filepath = SHAPEFILE_DIR + 'test_read_shapefile_bulk_with_aggregation_02'

    # encoding
    encoding = 'utf-8'

    # 水平、垂直方向精度を定義
    h_zoom = 5
    v_zoom = 5

    # shapefileに格納するデータの座標系
    shapefile_crs = 4326

    # shapefile作成用の座標配列取得
    fan_point_list_01 = geographic_shape_point_data_01[0][0]
    fan_point_type_01 = geographic_shape_point_data_01[1][0]
    fan_point_list_02 = geographic_shape_point_data_01[0][1]
    fan_point_type_02 = geographic_shape_point_data_01[1][1]

    fan_point_list_03 = geographic_shape_point_data_02[0][0]
    fan_point_type_03 = geographic_shape_point_data_02[1][0]
    fan_point_list_04 = geographic_shape_point_data_02[0][1]
    fan_point_type_04 = geographic_shape_point_data_02[1][1]

    # shapefile生成
    w = sf.Writer(filepath)
    w.field('name', 'C')
    w.field('type', 'C')
    w.multipatch(
            [fan_point_list_01, fan_point_list_02],
            partTypes=[TRIANGLE_FAN, TRIANGLE_FAN])
    w.record('record1', 'FAN')
    w.multipatch(
            [fan_point_list_03, fan_point_list_04],
            partTypes=[TRIANGLE_FAN, TRIANGLE_FAN])
    w.record('record2', 'FAN')
    w.close()

    # 期待値取得
    # ペイロードの期待値を定義
    shapefile_record_list = [['record1', 'FAN'], ['record2', 'FAN']]

    # fieldの期待値を定義
    shapefile_field_list = [['name', 'C', 50, 0], ['type', 'C', 50, 0]]

    # TRIANGLE_FANのSpatialPointオブジェクトを生成
    spatial_fan_point_list_01 = _get_spatial_point_objects(fan_point_list_01)
    spatial_fan_point_list_02 = _get_spatial_point_objects(fan_point_list_02)
    spatial_fan_point_list_03 = _get_spatial_point_objects(fan_point_list_03)
    spatial_fan_point_list_04 = _get_spatial_point_objects(fan_point_list_04)

    # SpatialTriangleオブジェクト生成
    spatial_triangle_list_01 = \
        _shaping_point_to_triangle(
                spatial_fan_point_list_01, fan_point_type_01)
    spatial_triangle_list_02 = \
        _shaping_point_to_triangle(
                spatial_fan_point_list_02, fan_point_type_02)
    spatial_triangle_list_03 = \
        _shaping_point_to_triangle(
                spatial_fan_point_list_03, fan_point_type_03)
    spatial_triangle_list_04 = \
        _shaping_point_to_triangle(
                spatial_fan_point_list_04, fan_point_type_04)

    # 空間ID取得対象のSpatialTriangleを格納した配列を定義
    triangle_list_01 = spatial_triangle_list_01 + spatial_triangle_list_02
    triangle_list_02 = spatial_triangle_list_03 + spatial_triangle_list_04

    # 空間IDの期待値を定義
    shapefile_spatial_ids_01 = get_spatial_ids_on_polygons(
            triangle_list_01, [], h_zoom, v_zoom)
    shapefile_spatial_ids_02 = get_spatial_ids_on_polygons(
            triangle_list_02, [], h_zoom, v_zoom)
    shapefile_spatial_id_list = [
            shapefile_spatial_ids_01,
            shapefile_spatial_ids_02]

    # 集約後のペイロードの期待値を定義
    shapefile_record_list_agg = {}

    for ids, payloads in zip(shapefile_spatial_id_list, shapefile_record_list):

        for sp_id in ids:
            # 空間IDをキー値として未定義の場合、初期化
            if sp_id not in shapefile_record_list_agg:
                shapefile_record_list_agg[sp_id] = []

            # 辞書に、空間IDをキー、値をペイロードとして格納
            shapefile_record_list_agg[sp_id].append(payloads)

    # 集約後のペイロードを集約関数でさらに成形
    shapefile_record_list_agg = _custom_payload_func(shapefile_record_list_agg)

    # 試験実施
    # 戻り値取得
    param_return = read_shapefile_bulk_with_aggregation(
            filepath,
            encoding,
            h_zoom,
            v_zoom,
            shapefile_crs,
            _custom_payload_func)

    # 戻り値確認
    loop_cnt = 0

    # 空間IDの要素数確認
    param_return_spatial_id_list = param_return[0]
    assert len(shapefile_spatial_id_list) == len(param_return_spatial_id_list)

    # フィールドの要素一致確認
    assert shapefile_field_list == param_return[1]

    # 集約後のペイロードの要素一致確認
    param_return_payload = param_return[2]
    assert shapefile_record_list_agg == param_return_payload

    for ids in param_return_spatial_id_list:
        # APIから取得した空間IDは順不同のため比較前にソートし、格納された空間IDが一致していることを確認する。
        param_assert_ids = sorted(shapefile_spatial_id_list[loop_cnt])
        param_return_ids = sorted(ids)

        # 戻り値確認 - 空間ID
        assert param_assert_ids == param_return_ids

        # 全ての空間IDに対し集約が行われていることを確認
        for target_id in param_return_ids:
            assert True == (target_id in param_return_payload)

        loop_cnt += 1


def test_read_shapefile_bulk_with_aggregation_03(
        geographic_shape_point_data_01
):
    """Shapefileの空間ID集約取得、集約関数なし、shapefileレコード数1
    + 試験詳細
      - 入力条件
        - 引数の集約関数入力なし
        - shapefileレコード数1
      - 確認内容
        - 戻り値のペイロードは、デフォルトの形式で集約されていること。
        - shapefileの全レコード文の空間IDに対し、集約が行われていること。
    """
    # 試験用データの定義
    # 試験用shalefileパス
    filepath = SHAPEFILE_DIR + 'test_read_shapefile_bulk_with_aggregation_03'

    # encoding
    encoding = 'utf-8'

    # 水平、垂直方向精度を定義
    h_zoom = 1
    v_zoom = 1

    # shapefileに格納するデータの座標系
    shapefile_crs = 4326

    # shapefile作成用の座標配列取得
    fan_point_list_01 = geographic_shape_point_data_01[0][0]
    fan_point_type_01 = geographic_shape_point_data_01[1][0]
    fan_point_list_02 = geographic_shape_point_data_01[0][1]
    fan_point_type_02 = geographic_shape_point_data_01[1][1]

    # shapefile生成
    w = sf.Writer(filepath)
    w.field('name', 'C')
    w.field('type', 'C')
    w.multipatch(
            [fan_point_list_01, fan_point_list_02],
            partTypes=[TRIANGLE_FAN, TRIANGLE_FAN])
    w.record('record1', 'FAN')
    w.close()

    # 期待値取得
    # ペイロードの期待値を定義
    shapefile_record_list = [['record1', 'FAN']]

    # fieldの期待値を定義
    shapefile_field_list = [['name', 'C', 50, 0], ['type', 'C', 50, 0]]

    # TRIANGLE_FANのSpatialPointオブジェクトを生成
    spatial_fan_point_list_01 = _get_spatial_point_objects(fan_point_list_01)
    spatial_fan_point_list_02 = _get_spatial_point_objects(fan_point_list_02)

    # SpatialTriangleオブジェクト生成
    spatial_triangle_list_01 = \
        _shaping_point_to_triangle(
                spatial_fan_point_list_01, fan_point_type_01)
    spatial_triangle_list_02 = \
        _shaping_point_to_triangle(
                spatial_fan_point_list_02, fan_point_type_02)

    # 空間ID取得対象のSpatialTriangleを格納した配列を定義
    triangle_list = spatial_triangle_list_01 + spatial_triangle_list_02

    # 空間IDの期待値を定義
    shapefile_spatial_id_list = [get_spatial_ids_on_polygons(
              triangle_list, [], h_zoom, v_zoom)]

    # 集約後のペイロードの期待値を定義
    shapefile_record_list_agg = {}

    for ids, payloads in zip(shapefile_spatial_id_list, shapefile_record_list):

        for sp_id in ids:
            # 空間IDをキー値として未定義の場合、初期化
            if sp_id not in shapefile_record_list_agg:
                shapefile_record_list_agg[sp_id] = []

            # 辞書に、空間IDをキー、値をペイロードとして格納
            shapefile_record_list_agg[sp_id].append(payloads)

    # 試験実施
    # 戻り値取得
    param_return = read_shapefile_bulk_with_aggregation(
            filepath, encoding, h_zoom, v_zoom, shapefile_crs)

    # 戻り値確認
    loop_cnt = 0

    # 空間IDの要素数確認
    param_return_spatial_id_list = param_return[0]
    assert len(shapefile_spatial_id_list) == len(param_return_spatial_id_list)

    # フィールドの要素一致確認
    assert shapefile_field_list == param_return[1]

    # 集約後のペイロードの要素一致確認
    param_return_payload = param_return[2]
    assert shapefile_record_list_agg == param_return_payload

    # 空間IDの値確認
    for ids in param_return_spatial_id_list:
        # APIから取得した空間IDは順不同のため比較前にソートし、格納された空間IDが一致していることを確認する。
        param_assert_ids = sorted(shapefile_spatial_id_list[loop_cnt])
        param_return_ids = sorted(ids)

        # 戻り値確認 - 空間ID
        assert param_assert_ids == param_return_ids

        # 全ての空間IDに対し集約が行われていることを確認
        for target_id in param_return_ids:
            assert True == (target_id in param_return_payload)

        loop_cnt += 1


def test_read_shapefile_bulk_with_aggregation_04(
        geographic_shape_point_data_01
):
    """Shapefileの空間ID集約取得、集約関数指定あり、shapefileレコード数1
    + 試験詳細
      - 入力条件
        - 引数の集約関数入力あり
        - shapefileレコード数1
      - 確認内容
        - 戻り値のペイロードは、デフォルトの集約結果に対し、引数で指定した集約関数の処理が行われたものになっていること。
        - shapefileの全レコード文の空間IDに対し、集約が行われていること。
    """
    # 試験用データの定義
    # 試験用shalefileパス
    filepath = SHAPEFILE_DIR + 'test_read_shapefile_bulk_with_aggregation_04'

    # encoding
    encoding = 'utf-8'

    # 水平、垂直方向精度を定義
    h_zoom = 5
    v_zoom = 5

    # shapefileに格納するデータの座標系
    shapefile_crs = 4326

    # shapefile作成用の座標配列取得
    fan_point_list_01 = geographic_shape_point_data_01[0][0]
    fan_point_type_01 = geographic_shape_point_data_01[1][0]
    fan_point_list_02 = geographic_shape_point_data_01[0][1]
    fan_point_type_02 = geographic_shape_point_data_01[1][1]

    # shapefile生成
    w = sf.Writer(filepath)
    w.field('name', 'C')
    w.field('type', 'C')
    w.multipatch(
            [fan_point_list_01, fan_point_list_02],
            partTypes=[TRIANGLE_FAN, TRIANGLE_FAN])
    w.record('record1', 'FAN')
    w.close()

    # 期待値取得
    # ペイロードの期待値を定義
    shapefile_record_list = [['record1', 'FAN']]

    # fieldの期待値を定義
    shapefile_field_list = [['name', 'C', 50, 0], ['type', 'C', 50, 0]]

    # TRIANGLE_FANのSpatialPointオブジェクトを生成
    spatial_fan_point_list_01 = _get_spatial_point_objects(fan_point_list_01)
    spatial_fan_point_list_02 = _get_spatial_point_objects(fan_point_list_02)

    # SpatialTriangleオブジェクト生成
    spatial_triangle_list_01 = \
        _shaping_point_to_triangle(
                spatial_fan_point_list_01, fan_point_type_01)
    spatial_triangle_list_02 = \
        _shaping_point_to_triangle(
                spatial_fan_point_list_02, fan_point_type_02)

    # 空間ID取得対象のSpatialTriangleを格納した配列を定義
    triangle_list = spatial_triangle_list_01 + spatial_triangle_list_02

    # 空間IDの期待値を定義
    shapefile_spatial_id_list = [get_spatial_ids_on_polygons(
              triangle_list, [], h_zoom, v_zoom)]

    # 集約後のペイロードの期待値を定義
    shapefile_record_list_agg = {}

    for ids, payloads in zip(shapefile_spatial_id_list, shapefile_record_list):

        for sp_id in ids:
            # 空間IDをキー値として未定義の場合、初期化
            if sp_id not in shapefile_record_list_agg:
                shapefile_record_list_agg[sp_id] = []

            # 辞書に、空間IDをキー、値をペイロードとして格納
            shapefile_record_list_agg[sp_id].append(payloads)

    # 集約後のペイロードを集約関数でさらに成形
    shapefile_record_list_agg = _custom_payload_func(shapefile_record_list_agg)

    # 試験実施
    # 戻り値取得
    param_return = read_shapefile_bulk_with_aggregation(
            filepath,
            encoding,
            h_zoom,
            v_zoom,
            shapefile_crs,
            _custom_payload_func)

    # 戻り値確認
    loop_cnt = 0

    # 空間IDの要素数確認
    param_return_spatial_id_list = param_return[0]
    assert len(shapefile_spatial_id_list) == len(param_return_spatial_id_list)

    # フィールドの要素一致確認
    assert shapefile_field_list == param_return[1]

    # 集約後のペイロードの要素一致確認
    param_return_payload = param_return[2]
    assert shapefile_record_list_agg == param_return_payload

    for ids in param_return_spatial_id_list:
        # APIから取得した空間IDは順不同のため比較前にソートし、格納された空間IDが一致していることを確認する。
        param_assert_ids = sorted(shapefile_spatial_id_list[loop_cnt])
        param_return_ids = sorted(ids)

        # 戻り値確認 - 空間ID
        assert param_assert_ids == param_return_ids

        # 全ての空間IDに対し集約が行われていることを確認
        for target_id in param_return_ids:
            assert True == (target_id in param_return_payload)

        loop_cnt += 1


def test_read_shapefile_bulk_with_aggregation_05(
        geographic_shape_point_data_01
):
    """Shapefileの空間ID集約取得、入力した集約関数で例外発生
    """
    # 試験用データの定義
    # 試験用shalefileパス
    filepath = SHAPEFILE_DIR + 'test_read_shapefile_bulk_with_aggregation_05'

    # encoding
    encoding = 'utf-8'

    # 水平、垂直方向精度を定義
    h_zoom = 1
    v_zoom = 1

    # shapefileに格納するデータの座標系
    shapefile_crs = 4326

    # shapefile作成用の座標配列取得
    fan_point_list_01 = geographic_shape_point_data_01[0][0]
    fan_point_list_02 = geographic_shape_point_data_01[0][1]

    # shapefile生成
    w = sf.Writer(filepath)
    w.field('name', 'C')
    w.field('type', 'C')
    w.multipatch(
            [fan_point_list_01, fan_point_list_02],
            partTypes=[TRIANGLE_FAN, TRIANGLE_FAN])
    # 集約関数内で例外が発生する条件を満たす値を設定
    w.record('Exception', 'STRIP')
    w.close()

    # 期待値取得
    # 例外取得
    exception_assert = SpatialIdError('SHAPEFILE_AGGREGATION_FUNC_ERROR')

    # 試験実施
    # 例外発生確認
    with pytest.raises(SpatialIdError) as e:
        read_shapefile_bulk_with_aggregation(
                filepath,
                encoding,
                h_zoom,
                v_zoom,
                shapefile_crs,
                _custom_payload_func)

    # 例外メッセージ確認
    assert str(exception_assert) == str(e.value)


def test_read_shapefile_bulk_with_aggregation_06(
        geographic_shape_point_data_01
):
    """Shapefileの空間ID集約取得、集約関数指定が不正
    - 試験詳細
      - 入力条件
        - 集約関数に空の文字列を指定
      - 確認内容
        - 集約関数呼び出し時の例外が発生すること
    """
    # 試験用データの定義
    # 試験用shalefileパス
    filepath = SHAPEFILE_DIR + 'test_read_shapefile_bulk_with_aggregation_06'

    # encoding
    encoding = 'utf-8'

    # 水平、垂直方向精度を定義
    h_zoom = 1
    v_zoom = 1

    # shapefileに格納するデータの座標系
    shapefile_crs = 4326

    # shapefile作成用の座標配列取得
    fan_point_list_01 = geographic_shape_point_data_01[0][0]
    fan_point_list_02 = geographic_shape_point_data_01[0][1]

    # shapefile生成
    w = sf.Writer(filepath)
    w.field('name', 'C')
    w.field('type', 'C')
    w.multipatch(
            [fan_point_list_01, fan_point_list_02],
            partTypes=[TRIANGLE_FAN, TRIANGLE_FAN])
    # 集約関数内で例外が発生する条件を満たす値を設定
    w.record('record1', 'FAN')
    w.close()

    # 期待値取得
    # 例外取得
    exception_assert = SpatialIdError('SHAPEFILE_AGGREGATION_FUNC_ERROR')

    # 試験実施
    # 例外発生確認
    with pytest.raises(SpatialIdError) as e:
        read_shapefile_bulk_with_aggregation(
                filepath,
                encoding,
                h_zoom,
                v_zoom,
                shapefile_crs,
                "")

    # 例外メッセージ確認
    assert str(exception_assert) == str(e.value)


def test_read_shapefile_bulk_with_aggregation_07(
        mocker,
        geographic_shape_point_data_01
):
    """Shapefileの空間ID集約取得、その他例外の発生
     + 試験詳細
       - 試験条件
         - 通常発生しない例外パターンのためモックを、使用して例外を発生させる
       - 確認内容
         - その他例外が発生すること
    """
    # 試験用データの定義
    # 試験用shalefileパス
    filepath = SHAPEFILE_DIR + 'test_read_shapefile_bulk_with_aggregation_07'

    # encoding
    encoding = 'utf-8'

    # 水平、垂直方向精度を定義
    h_zoom = 1
    v_zoom = 1

    # shapefileに格納するデータの座標系
    shapefile_crs = 4326

    # shapefile作成用の座標配列取得
    fan_point_list_01 = geographic_shape_point_data_01[0][0]
    fan_point_list_02 = geographic_shape_point_data_01[0][1]

    # shapefile生成
    w = sf.Writer(filepath)
    w.field('name', 'C')
    w.field('type', 'C')
    w.multipatch(
            [fan_point_list_01, fan_point_list_02],
            partTypes=[TRIANGLE_FAN, TRIANGLE_FAN])
    w.record('record1', 'FAN')
    w.close()

    # 期待値取得
    # 例外取得
    exception_assert = SpatialIdError('OTHER_ERROR')

    mocker.patch(
            'SpatialId.io.shapefile.read_shapefile_bulk').\
        side_effect = Exception('予期せぬ例外が発生')

    # 試験実施
    with pytest.raises(SpatialIdError) as e:
        read_shapefile_bulk_with_aggregation(
                filepath,
                encoding,
                h_zoom,
                v_zoom,
                shapefile_crs,
                _custom_payload_func)

    # 例外メッセージ確認
    assert str(exception_assert) == str(e.value)


def test_read_shapefile_bulk_with_aggregation_08(
        geographic_shape_point_data_01
):
    """強化試験：Shapefileの空間ID集約取得、読み込み対象がzipファイル
    + 試験詳細
      - 入力条件
        - zip形式の圧縮ファイルを読み込み対象のshapefileとする。
      - 確認内容
        - エラーとならず、空間ID情報の一括取得に成功すること
    """
    # 試験用データの定義
    # 試験用shalefileパス
    filepath = SHAPEFILE_DIR + 'test_read_shapefile_bulk_with_aggregation_08'

    # encoding
    encoding = 'utf-8'

    # 水平、垂直方向精度を定義
    h_zoom = 1
    v_zoom = 1

    # shapefileに格納するデータの座標系
    shapefile_crs = 4326

    # shapefile作成用の座標配列取得
    fan_point_list_01 = geographic_shape_point_data_01[0][0]
    fan_point_type_01 = geographic_shape_point_data_01[1][0]
    fan_point_list_02 = geographic_shape_point_data_01[0][1]
    fan_point_type_02 = geographic_shape_point_data_01[1][1]

    # shapefile生成
    w = sf.Writer(filepath)
    w.field('name', 'C')
    w.field('type', 'C')
    w.multipatch(
            [fan_point_list_01, fan_point_list_02],
            partTypes=[TRIANGLE_FAN, TRIANGLE_FAN])
    w.record('record1', 'FAN')
    w.close()

    # zipファイル作成
    dbf_path = filepath + '.dbf'
    shp_path = filepath + '.shp'
    shx_path = filepath + '.shx'
    zip_path = filepath + '.zip'
    zp = zipfile.ZipFile(zip_path, "w")
    zp.write(dbf_path, compress_type=zipfile.ZIP_DEFLATED)
    zp.write(shp_path, compress_type=zipfile.ZIP_DEFLATED)
    zp.write(shx_path, compress_type=zipfile.ZIP_DEFLATED)
    zp.close()

    # 圧縮済みのshapefileを削除
    os.remove(dbf_path)
    os.remove(shp_path)
    os.remove(shx_path)

    # 期待値取得
    # ペイロードの期待値を定義
    shapefile_record_list = [['record1', 'FAN']]

    # fieldの期待値を定義
    shapefile_field_list = [['name', 'C', 50, 0], ['type', 'C', 50, 0]]

    # TRIANGLE_FANのSpatialPointオブジェクトを生成
    spatial_fan_point_list_01 = _get_spatial_point_objects(fan_point_list_01)
    spatial_fan_point_list_02 = _get_spatial_point_objects(fan_point_list_02)

    # SpatialTriangleオブジェクト生成
    spatial_triangle_list_01 = \
        _shaping_point_to_triangle(
                spatial_fan_point_list_01, fan_point_type_01)
    spatial_triangle_list_02 = \
        _shaping_point_to_triangle(
                spatial_fan_point_list_02, fan_point_type_02)

    # 空間ID取得対象のSpatialTriangleを格納した配列を定義
    triangle_list = spatial_triangle_list_01 + spatial_triangle_list_02

    # 空間IDの期待値を定義
    shapefile_spatial_id_list = [get_spatial_ids_on_polygons(
              triangle_list, [], h_zoom, v_zoom)]

    # 集約後のペイロードの期待値を定義
    shapefile_record_list_agg = {}

    for ids, payloads in zip(shapefile_spatial_id_list, shapefile_record_list):

        for sp_id in ids:
            # 空間IDをキー値として未定義の場合、初期化
            if sp_id not in shapefile_record_list_agg:
                shapefile_record_list_agg[sp_id] = []

            # 辞書に、空間IDをキー、値をペイロードとして格納
            shapefile_record_list_agg[sp_id].append(payloads)

    # 試験実施
    # 戻り値取得
    param_return = read_shapefile_bulk_with_aggregation(
            zip_path, encoding, h_zoom, v_zoom, shapefile_crs)

    # 戻り値確認
    loop_cnt = 0

    # 空間IDの要素数確認
    param_return_spatial_id_list = param_return[0]
    assert len(shapefile_spatial_id_list) == len(param_return_spatial_id_list)

    # フィールドの要素一致確認
    assert shapefile_field_list == param_return[1]

    # 集約後のペイロードの要素一致確認
    param_return_payload = param_return[2]
    assert shapefile_record_list_agg == param_return_payload

    # 空間IDの値確認
    for ids in param_return_spatial_id_list:
        # APIから取得した空間IDは順不同のため比較前にソートし、格納された空間IDが一致していることを確認する。
        param_assert_ids = sorted(shapefile_spatial_id_list[loop_cnt])
        param_return_ids = sorted(ids)

        # 戻り値確認 - 空間ID
        assert param_assert_ids == param_return_ids

        # 全ての空間IDに対し集約が行われていることを確認
        for target_id in param_return_ids:
            assert True == (target_id in param_return_payload)

        loop_cnt += 1


def test_read_shapefile_bulk_with_aggregation_09(
        geographic_shape_point_data_01
):
    """強化試験：Shapefileの空間ID集約取得、読み込みファイル拡張子が.shp
    + 試験詳細
      - 入力条件
        - 読み込みファイル拡張子が.shp
      - 確認内容
        - 戻り値のペイロードは、デフォルトの形式で集約されていること。
        - shapefileの全レコード文の空間IDに対し、集約が行われていること。
    """
    # 試験用データの定義
    # 試験用shalefileパス
    filepath = SHAPEFILE_DIR + 'test_read_shapefile_bulk_with_aggregation_09'

    # encoding
    encoding = 'utf-8'

    # 水平、垂直方向精度を定義
    h_zoom = 1
    v_zoom = 1

    # shapefileに格納するデータの座標系
    shapefile_crs = 4326

    # shapefile作成用の座標配列取得
    fan_point_list_01 = geographic_shape_point_data_01[0][0]
    fan_point_type_01 = geographic_shape_point_data_01[1][0]
    fan_point_list_02 = geographic_shape_point_data_01[0][1]
    fan_point_type_02 = geographic_shape_point_data_01[1][1]

    # shapefile生成
    w = sf.Writer(filepath)
    w.field('name', 'C')
    w.field('type', 'C')
    w.multipatch(
            [fan_point_list_01, fan_point_list_02],
            partTypes=[TRIANGLE_FAN, TRIANGLE_FAN])
    w.record('record1', 'FAN')
    w.close()

    # 読み込み対象のファイルパス
    read_path = filepath + '.shp'

    # 期待値取得
    # ペイロードの期待値を定義
    shapefile_record_list = [['record1', 'FAN']]

    # fieldの期待値を定義
    shapefile_field_list = [['name', 'C', 50, 0], ['type', 'C', 50, 0]]

    # TRIANGLE_FANのSpatialPointオブジェクトを生成
    spatial_fan_point_list_01 = _get_spatial_point_objects(fan_point_list_01)
    spatial_fan_point_list_02 = _get_spatial_point_objects(fan_point_list_02)

    # SpatialTriangleオブジェクト生成
    spatial_triangle_list_01 = \
        _shaping_point_to_triangle(
                spatial_fan_point_list_01, fan_point_type_01)
    spatial_triangle_list_02 = \
        _shaping_point_to_triangle(
                spatial_fan_point_list_02, fan_point_type_02)

    # 空間ID取得対象のSpatialTriangleを格納した配列を定義
    triangle_list = spatial_triangle_list_01 + spatial_triangle_list_02

    # 空間IDの期待値を定義
    shapefile_spatial_id_list = [get_spatial_ids_on_polygons(
              triangle_list, [], h_zoom, v_zoom)]

    # 集約後のペイロードの期待値を定義
    shapefile_record_list_agg = {}

    for ids, payloads in zip(shapefile_spatial_id_list, shapefile_record_list):

        for sp_id in ids:
            # 空間IDをキー値として未定義の場合、初期化
            if sp_id not in shapefile_record_list_agg:
                shapefile_record_list_agg[sp_id] = []

            # 辞書に、空間IDをキー、値をペイロードとして格納
            shapefile_record_list_agg[sp_id].append(payloads)

    # 試験実施
    # 戻り値取得
    param_return = read_shapefile_bulk_with_aggregation(
            read_path, encoding, h_zoom, v_zoom, shapefile_crs)

    # 戻り値確認
    loop_cnt = 0

    # 空間IDの要素数確認
    param_return_spatial_id_list = param_return[0]
    assert len(shapefile_spatial_id_list) == len(param_return_spatial_id_list)

    # フィールドの要素一致確認
    assert shapefile_field_list == param_return[1]

    # 集約後のペイロードの要素一致確認
    param_return_payload = param_return[2]
    assert shapefile_record_list_agg == param_return_payload

    # 空間IDの値確認
    for ids in param_return_spatial_id_list:
        # APIから取得した空間IDは順不同のため比較前にソートし、格納された空間IDが一致していることを確認する。
        param_assert_ids = sorted(shapefile_spatial_id_list[loop_cnt])
        param_return_ids = sorted(ids)

        # 戻り値確認 - 空間ID
        assert param_assert_ids == param_return_ids

        # 全ての空間IDに対し集約が行われていることを確認
        for target_id in param_return_ids:
            assert True == (target_id in param_return_payload)

        loop_cnt += 1


def test_read_shapefile_bulk_with_aggregation_10(
        geographic_shape_point_data_01
):
    """強化試験：Shapefileの空間ID集約取得、読み込みファイル拡張子が.dbf
    + 試験詳細
      - 入力条件
        - 読み込みファイル拡張子が.dbf
      - 確認内容
        - 戻り値のペイロードは、デフォルトの形式で集約されていること。
        - shapefileの全レコード文の空間IDに対し、集約が行われていること。
    """
    # 試験用データの定義
    # 試験用shalefileパス
    filepath = SHAPEFILE_DIR + 'test_read_shapefile_bulk_with_aggregation_10'

    # encoding
    encoding = 'utf-8'

    # 水平、垂直方向精度を定義
    h_zoom = 1
    v_zoom = 1

    # shapefileに格納するデータの座標系
    shapefile_crs = 4326

    # shapefile作成用の座標配列取得
    fan_point_list_01 = geographic_shape_point_data_01[0][0]
    fan_point_type_01 = geographic_shape_point_data_01[1][0]
    fan_point_list_02 = geographic_shape_point_data_01[0][1]
    fan_point_type_02 = geographic_shape_point_data_01[1][1]

    # shapefile生成
    w = sf.Writer(filepath)
    w.field('name', 'C')
    w.field('type', 'C')
    w.multipatch(
            [fan_point_list_01, fan_point_list_02],
            partTypes=[TRIANGLE_FAN, TRIANGLE_FAN])
    w.record('record1', 'FAN')
    w.close()

    # 読み込み対象のファイルパス
    read_path = filepath + '.dbf'

    # 期待値取得
    # ペイロードの期待値を定義
    shapefile_record_list = [['record1', 'FAN']]

    # fieldの期待値を定義
    shapefile_field_list = [['name', 'C', 50, 0], ['type', 'C', 50, 0]]

    # TRIANGLE_FANのSpatialPointオブジェクトを生成
    spatial_fan_point_list_01 = _get_spatial_point_objects(fan_point_list_01)
    spatial_fan_point_list_02 = _get_spatial_point_objects(fan_point_list_02)

    # SpatialTriangleオブジェクト生成
    spatial_triangle_list_01 = \
        _shaping_point_to_triangle(
                spatial_fan_point_list_01, fan_point_type_01)
    spatial_triangle_list_02 = \
        _shaping_point_to_triangle(
                spatial_fan_point_list_02, fan_point_type_02)

    # 空間ID取得対象のSpatialTriangleを格納した配列を定義
    triangle_list = spatial_triangle_list_01 + spatial_triangle_list_02

    # 空間IDの期待値を定義
    shapefile_spatial_id_list = [get_spatial_ids_on_polygons(
              triangle_list, [], h_zoom, v_zoom)]

    # 集約後のペイロードの期待値を定義
    shapefile_record_list_agg = {}

    for ids, payloads in zip(shapefile_spatial_id_list, shapefile_record_list):

        for sp_id in ids:
            # 空間IDをキー値として未定義の場合、初期化
            if sp_id not in shapefile_record_list_agg:
                shapefile_record_list_agg[sp_id] = []

            # 辞書に、空間IDをキー、値をペイロードとして格納
            shapefile_record_list_agg[sp_id].append(payloads)

    # 試験実施
    # 戻り値取得
    param_return = read_shapefile_bulk_with_aggregation(
            read_path, encoding, h_zoom, v_zoom, shapefile_crs)

    # 戻り値確認
    loop_cnt = 0

    # 空間IDの要素数確認
    param_return_spatial_id_list = param_return[0]
    assert len(shapefile_spatial_id_list) == len(param_return_spatial_id_list)

    # フィールドの要素一致確認
    assert shapefile_field_list == param_return[1]

    # 集約後のペイロードの要素一致確認
    param_return_payload = param_return[2]
    assert shapefile_record_list_agg == param_return_payload

    # 空間IDの値確認
    for ids in param_return_spatial_id_list:
        # APIから取得した空間IDは順不同のため比較前にソートし、格納された空間IDが一致していることを確認する。
        param_assert_ids = sorted(shapefile_spatial_id_list[loop_cnt])
        param_return_ids = sorted(ids)

        # 戻り値確認 - 空間ID
        assert param_assert_ids == param_return_ids

        # 全ての空間IDに対し集約が行われていることを確認
        for target_id in param_return_ids:
            assert True == (target_id in param_return_payload)

        loop_cnt += 1


def test_read_shapefile_bulk_with_aggregation_11(
        geographic_shape_point_data_01
):
    """強化試験：Shapefileの空間ID集約取得、.shpのみが存在しない場合
    + 試験詳細
      - 入力条件
        - 読み込み対象のshapefileの内、".shp"のみが存在しない
      - 確認内容
        - shapefile読み込み失敗の例外が発生すること
    """
    # 試験用データの定義
    # 試験用shalefileパス
    filepath = SHAPEFILE_DIR + 'test_read_shapefile_bulk_with_aggregation_11'

    # encoding
    encoding = 'utf-8'

    # 水平、垂直方向精度を定義
    h_zoom = 1
    v_zoom = 1

    # shapefileに格納するデータの座標系
    shapefile_crs = 4326

    # shapefile作成用の座標配列取得
    fan_point_list_01 = geographic_shape_point_data_01[0][0]
    fan_point_list_02 = geographic_shape_point_data_01[0][1]

    # shapefile生成
    w = sf.Writer(filepath)
    w.field('name', 'C')
    w.field('type', 'C')
    w.multipatch(
            [fan_point_list_01, fan_point_list_02],
            partTypes=[TRIANGLE_FAN, TRIANGLE_FAN])
    w.record('record1', 'FAN')
    w.close()

    # 読み込み対象のshapefileの内.shpファイルを削除
    remove_shapefile_path = filepath + '.shp'
    os.remove(remove_shapefile_path)

    # 期待値取得
    # 例外取得
    exception_assert = SpatialIdError('SHAPEFILE_READ_ERROR')

    # 試験実施
    # 例外発生確認
    with pytest.raises(SpatialIdError) as e:
        read_shapefile_bulk_with_aggregation(
                filepath, encoding, h_zoom, v_zoom, shapefile_crs)

    # 例外メッセージ確認
    assert str(exception_assert) == str(e.value)


def test_read_shapefile_bulk_with_aggregation_12(
        geographic_shape_point_data_01
):
    """強化試験：Shapefileの空間ID集約取得、.dbfのみが存在しない場合
    + 試験詳細
      - 入力条件
        - 読み込み対象のshapefileの内、".dbf"のみが存在しない
      - 確認内容
        - shapefile読み込み失敗の例外が発生すること
    """
    # 試験用データの定義
    # 試験用shalefileパス
    filepath = SHAPEFILE_DIR + 'test_read_shapefile_bulk_with_aggregation_12'

    # encoding
    encoding = 'utf-8'

    # 水平、垂直方向精度を定義
    h_zoom = 1
    v_zoom = 1

    # shapefileに格納するデータの座標系
    shapefile_crs = 4326

    # shapefile作成用の座標配列取得
    fan_point_list_01 = geographic_shape_point_data_01[0][0]
    fan_point_list_02 = geographic_shape_point_data_01[0][1]

    # shapefile生成
    w = sf.Writer(filepath)
    w.field('name', 'C')
    w.field('type', 'C')
    w.multipatch(
            [fan_point_list_01, fan_point_list_02],
            partTypes=[TRIANGLE_FAN, TRIANGLE_FAN])
    w.record('record1', 'FAN')
    w.close()

    # 読み込み対象のshapefileの内.shpファイルを削除
    remove_shapefile_path = filepath + '.dbf'
    os.remove(remove_shapefile_path)

    # 期待値取得
    # 例外取得
    exception_assert = SpatialIdError('SHAPEFILE_READ_ERROR')

    # 試験実施
    # 例外発生確認
    with pytest.raises(SpatialIdError) as e:
        read_shapefile_bulk_with_aggregation(
                filepath, encoding, h_zoom, v_zoom, shapefile_crs)

    # 例外メッセージ確認
    assert str(exception_assert) == str(e.value)


def test_read_shapefile_bulk_with_aggregation_13(
        geographic_shape_point_data_01
):
    """強化試験：Shapefileの空間ID集約取得、入力CRSが地理座標系
    + 試験詳細
      - 入力条件
        - ユーザ入力のCRSに地理座標系を入力
      - 確認内容
        - 戻り値のペイロードは、デフォルトの形式で集約されていること。
        - shapefileの全レコード文の空間IDに対し、集約が行われていること。
    """
    # 試験用データの定義
    # 試験用shalefileパス
    filepath = SHAPEFILE_DIR + 'test_read_shapefile_bulk_with_aggregation_13'

    # encoding
    encoding = 'utf-8'

    # 水平、垂直方向精度を定義
    h_zoom = 10
    v_zoom = 10

    # shapefileに格納するデータの座標系
    shapefile_crs = 4612

    # shapefile作成用の座標配列取得
    fan_point_list_01 = geographic_shape_point_data_01[0][0]
    fan_point_type_01 = geographic_shape_point_data_01[1][0]
    fan_point_list_02 = geographic_shape_point_data_01[0][1]
    fan_point_type_02 = geographic_shape_point_data_01[1][1]

    # shapefile生成
    w = sf.Writer(filepath)
    w.field('name', 'C')
    w.field('type', 'C')
    w.multipatch(
            [fan_point_list_01, fan_point_list_02],
            partTypes=[TRIANGLE_FAN, TRIANGLE_FAN])
    w.record('record1', 'FAN')
    w.close()

    # 期待値取得
    # ペイロードの期待値を定義
    shapefile_record_list = [['record1', 'FAN']]

    # fieldの期待値を定義
    shapefile_field_list = [['name', 'C', 50, 0], ['type', 'C', 50, 0]]

    # TRIANGLE_FANのSpatialPointオブジェクトを生成
    spatial_fan_point_list_01 = _get_spatial_point_objects(fan_point_list_01)
    spatial_fan_point_list_02 = _get_spatial_point_objects(fan_point_list_02)

    # SpatialTriangleオブジェクト生成
    spatial_triangle_list_01 = \
        _shaping_point_to_triangle(
                spatial_fan_point_list_01, fan_point_type_01)
    spatial_triangle_list_02 = \
        _shaping_point_to_triangle(
                spatial_fan_point_list_02, fan_point_type_02)

    # 空間ID取得対象のSpatialTriangleを格納した配列を定義
    triangle_list = spatial_triangle_list_01 + spatial_triangle_list_02

    # 空間IDの期待値を定義
    shapefile_spatial_id_list = [get_spatial_ids_on_polygons(
              triangle_list, [], h_zoom, v_zoom)]

    # 集約後のペイロードの期待値を定義
    shapefile_record_list_agg = {}

    for ids, payloads in zip(shapefile_spatial_id_list, shapefile_record_list):

        for sp_id in ids:
            # 空間IDをキー値として未定義の場合、初期化
            if sp_id not in shapefile_record_list_agg:
                shapefile_record_list_agg[sp_id] = []

            # 辞書に、空間IDをキー、値をペイロードとして格納
            shapefile_record_list_agg[sp_id].append(payloads)

    # 試験実施
    # 戻り値取得
    param_return = read_shapefile_bulk_with_aggregation(
            filepath, encoding, h_zoom, v_zoom, shapefile_crs)

    # 戻り値確認
    loop_cnt = 0

    # 空間IDの要素数確認
    param_return_spatial_id_list = param_return[0]
    assert len(shapefile_spatial_id_list) == len(param_return_spatial_id_list)

    # フィールドの要素一致確認
    assert shapefile_field_list == param_return[1]

    # 集約後のペイロードの要素一致確認
    param_return_payload = param_return[2]
    assert shapefile_record_list_agg == param_return_payload

    # 空間IDの値確認
    for ids in param_return_spatial_id_list:
        # APIから取得した空間IDは順不同のため比較前にソートし、格納された空間IDが一致していることを確認する。
        param_assert_ids = sorted(shapefile_spatial_id_list[loop_cnt])
        param_return_ids = sorted(ids)

        # 戻り値確認 - 空間ID
        assert param_assert_ids == param_return_ids

        # 全ての空間IDに対し集約が行われていることを確認
        for target_id in param_return_ids:
            assert True == (target_id in param_return_payload)

        loop_cnt += 1


def test_read_shapefile_bulk_with_aggregation_14(
        projected_shape_point_data_01
):
    """強化試験：Shapefileの空間ID集約取得、入力CRSが投影座標系
    + 試験詳細
      - 入力条件
        - ユーザ入力のCRSに投影座標系を入力
      - 確認内容
        - 戻り値のペイロードは、デフォルトの形式で集約されていること。
        - shapefileの全レコード文の空間IDに対し、集約が行われていること。
    """
    # 試験用データの定義
    # 試験用shalefileパス
    filepath = SHAPEFILE_DIR + 'test_read_shapefile_bulk_with_aggregation_14'

    # encoding
    encoding = 'utf-8'

    # 水平、垂直方向精度を定義
    h_zoom = 10
    v_zoom = 10

    # shapefileに格納するデータの座標系
    shapefile_crs = 6677

    # shapefile作成用の座標配列取得
    strip_point_list_01 = projected_shape_point_data_01[0][0]
    strip_point_type_01 = projected_shape_point_data_01[1][0]
    strip_point_list_02 = projected_shape_point_data_01[0][1]
    strip_point_type_02 = projected_shape_point_data_01[1][1]
    fan_point_list_01 = projected_shape_point_data_01[0][2]
    fan_point_type_01 = projected_shape_point_data_01[1][2]

    # shapefile生成
    w = sf.Writer(filepath)
    w.field('name', 'C')
    w.field('type', 'C')
    w.multipatch(
            [strip_point_list_01, strip_point_list_02, fan_point_list_01],
            partTypes=[TRIANGLE_STRIP, TRIANGLE_STRIP, TRIANGLE_FAN])
    w.record('record1', 'STRIP, FAN')
    w.close()

    # 期待値取得
    # ペイロードの期待値を定義
    shapefile_record_list = [['record1', 'STRIP, FAN']]

    # fieldの期待値を定義
    shapefile_field_list = [['name', 'C', 50, 0], ['type', 'C', 50, 0]]

    # TRIANGLE_STRIPのSpatialPointオブジェクトを生成
    spatial_strip_point_list_01 = \
        _get_spatial_point_objects(strip_point_list_01, shapefile_crs)
    spatial_strip_point_list_02 = \
        _get_spatial_point_objects(strip_point_list_02, shapefile_crs)

    # TRIANGLE_FANのSpatialPointオブジェクトを生成
    spatial_fan_point_list_01 = \
        _get_spatial_point_objects(fan_point_list_01, shapefile_crs)

    # SpatialTriangleオブジェクト生成
    spatial_triangle_list_01 = \
        _shaping_point_to_triangle(
                spatial_strip_point_list_01, strip_point_type_01)
    spatial_triangle_list_02 = \
        _shaping_point_to_triangle(
                spatial_strip_point_list_02, strip_point_type_02)
    spatial_triangle_list_03 = \
        _shaping_point_to_triangle(
                spatial_fan_point_list_01, fan_point_type_01)

    # 空間ID取得対象のSpatialTriangleを格納した配列を定義
    triangle_list = spatial_triangle_list_01 + spatial_triangle_list_02\
        + spatial_triangle_list_03

    # 空間IDの期待値を定義
    shapefile_spatial_id_list = [get_spatial_ids_on_polygons(
              triangle_list, [], h_zoom, v_zoom)]

    # 集約後のペイロードの期待値を定義
    shapefile_record_list_agg = {}

    for ids, payloads in zip(shapefile_spatial_id_list, shapefile_record_list):

        for sp_id in ids:
            # 空間IDをキー値として未定義の場合、初期化
            if sp_id not in shapefile_record_list_agg:
                shapefile_record_list_agg[sp_id] = []

            # 辞書に、空間IDをキー、値をペイロードとして格納
            shapefile_record_list_agg[sp_id].append(payloads)

    # 試験実施
    # 戻り値取得
    param_return = read_shapefile_bulk_with_aggregation(
            filepath, encoding, h_zoom, v_zoom, shapefile_crs)

    # 戻り値確認
    loop_cnt = 0

    # 空間IDの要素数確認
    param_return_spatial_id_list = param_return[0]
    assert len(shapefile_spatial_id_list) == len(param_return_spatial_id_list)

    # フィールドの要素一致確認
    assert shapefile_field_list == param_return[1]

    # 集約後のペイロードの要素一致確認
    param_return_payload = param_return[2]
    assert shapefile_record_list_agg == param_return_payload

    # 空間IDの値確認
    for ids in param_return_spatial_id_list:
        # APIから取得した空間IDは順不同のため比較前にソートし、格納された空間IDが一致していることを確認する。
        param_assert_ids = sorted(shapefile_spatial_id_list[loop_cnt])
        param_return_ids = sorted(ids)

        # 戻り値確認 - 空間ID
        assert param_assert_ids == param_return_ids

        # 全ての空間IDに対し集約が行われていることを確認
        for target_id in param_return_ids:
            assert True == (target_id in param_return_payload)

        loop_cnt += 1


def test_read_shapefile_bulk_with_aggregation_15():
    """強化試験：Shapefileの空間ID集約取得、filepath入力チェックNG
    + 試験詳細
      - 入力条件
        - filepathにint型の値を入力
      - 確認内容
        - 入力値チェックエラーの例外が発生すること
    """
    # 期待値定義
    # 例外を取得
    exception_assert = SpatialIdError('INPUT_VALUE_ERROR')

    # 試験実施
    # 例外発生を確認
    with pytest.raises(SpatialIdError) as e:
        read_shapefile_bulk_with_aggregation(100, "utf-8", 1, 1, 4326)

    assert str(exception_assert) == str(e.value)


def test_read_shapefile_bulk_with_aggregation_16():
    """強化試験：Shapefileの空間ID集約取得、filepathに空文字を入力
    + 試験詳細
      - 入力条件
        - filepathに空文字を入力
      - 確認内容
        - Shapefile読み込みエラーの例外が発生すること
    """
    # 期待値定義
    # 例外を取得
    exception_assert = SpatialIdError('SHAPEFILE_READ_ERROR')

    # 試験実施
    # 例外発生を確認
    with pytest.raises(SpatialIdError) as e:
        read_shapefile_bulk_with_aggregation("", "utf-8", 1, 1, 4326)

    assert str(exception_assert) == str(e.value)


def test_read_shapefile_bulk_with_aggregation_17():
    """強化試験：Shapefileの空間ID集約取得、filepathにNoneを入力
    + 試験詳細
      - 入力条件
        - filepathにNoneを入力
      - 確認内容
        - 入力値チェックエラーの例外が発生すること
    """
    # 期待値定義
    # 例外を取得
    exception_assert = SpatialIdError('INPUT_VALUE_ERROR')

    # 試験実施
    # 例外発生を確認
    with pytest.raises(SpatialIdError) as e:
        read_shapefile_bulk_with_aggregation(None, "utf-8", 1, 1, 4326)

    assert str(exception_assert) == str(e.value)


def test_read_shapefile_bulk_with_aggregation_18():
    """強化試験：Shapefileの空間ID集約取得、encoding入力チェックNG
    + 試験詳細
      - 入力条件
        - encodingにint型の値を入力
      - 確認内容
        - 入力値チェックエラーの例外が発生すること
    """
    # 期待値定義
    # 例外を取得
    exception_assert = SpatialIdError('INPUT_VALUE_ERROR')

    # 試験実施
    # 例外発生を確認
    with pytest.raises(SpatialIdError) as e:
        read_shapefile_bulk_with_aggregation("test/dir", 100, 1, 1, 4326)

    assert str(exception_assert) == str(e.value)


def test_read_shapefile_bulk_with_aggregation_19():
    """強化試験：Shapefileの空間ID集約取得、encodingに空文字を入力
    + 試験詳細
      - 入力条件
        - encodingに空文字を入力
      - 確認内容
        - Shapefile読み込みエラーの例外が発生すること
    """
    # 期待値定義
    # 例外を取得
    exception_assert = SpatialIdError('SHAPEFILE_READ_ERROR')

    # 試験実施
    # 例外発生を確認
    with pytest.raises(SpatialIdError) as e:
        read_shapefile_bulk_with_aggregation("test/dir", "", 1, 1, 4326)

    assert str(exception_assert) == str(e.value)


def test_read_shapefile_bulk_with_aggregation_20():
    """強化試験：Shapefileの空間ID集約取得、encodingにNoneを入力
    + 試験詳細
      - 入力条件
        - encodingにNoneを入力
      - 確認内容
        - 入力値チェックエラーの例外が発生すること
    """
    # 期待値定義
    # 例外を取得
    exception_assert = SpatialIdError('INPUT_VALUE_ERROR')

    # 試験実施
    # 例外発生を確認
    with pytest.raises(SpatialIdError) as e:
        read_shapefile_bulk_with_aggregation("test/dir", None, 1, 1, 4326)

    assert str(exception_assert) == str(e.value)


def test_read_shapefile_bulk_with_aggregation_21():
    """強化試験：Shapefileの空間ID集約取得、h_zoom入力チェックNG
    + 試験詳細
      - 入力条件
        - h_zoomにstr型の値を入力
      - 確認内容
        - 入力値チェックエラーの例外が発生すること
    """
    # 期待値定義
    # 例外を取得
    exception_assert = SpatialIdError('INPUT_VALUE_ERROR')

    # 試験実施
    # 例外発生を確認
    with pytest.raises(SpatialIdError) as e:
        read_shapefile_bulk_with_aggregation("test/dir", "utf-8", "1", 1, 4326)

    assert str(exception_assert) == str(e.value)


def test_read_shapefile_bulk_with_aggregation_22():
    """強化試験：Shapefileの空間ID集約取得、h_zoomにNoneを入力
    + 試験詳細
      - 入力条件
        - h_zoomにNoneを入力
      - 確認内容
        - 入力値チェックエラーの例外が発生すること
    """
    # 期待値定義
    # 例外を取得
    exception_assert = SpatialIdError('INPUT_VALUE_ERROR')

    # 試験実施
    # 例外発生を確認
    with pytest.raises(SpatialIdError) as e:
        read_shapefile_bulk_with_aggregation(
                "test/dir",
                "utf-8",
                None,
                1,
                4326)

    assert str(exception_assert) == str(e.value)


def test_read_shapefile_bulk_with_aggregation_23():
    """強化試験：Shapefileの空間ID集約取得、v_zoom入力チェックNG
    + 試験詳細
      - 入力条件
        - v_zoomにstr型の値を入力
      - 確認内容
        - 入力値チェックエラーの例外が発生すること
    """
    # 期待値定義
    # 例外を取得
    exception_assert = SpatialIdError('INPUT_VALUE_ERROR')

    # 試験実施
    # 例外発生を確認
    with pytest.raises(SpatialIdError) as e:
        read_shapefile_bulk_with_aggregation("test/dir", "utf-8", 1, "1", 4326)

    assert str(exception_assert) == str(e.value)


def test_read_shapefile_bulk_with_aggregation_24():
    """強化試験：Shapefileの空間ID集約取得、v_zoomにNoneを入力
    + 試験詳細
      - 入力条件
        - v_zoomにNoneを入力
      - 確認内容
        - 入力値チェックエラーの例外が発生すること
    """
    # 期待値定義
    # 例外を取得
    exception_assert = SpatialIdError('INPUT_VALUE_ERROR')

    # 試験実施
    # 例外発生を確認
    with pytest.raises(SpatialIdError) as e:
        read_shapefile_bulk_with_aggregation(
                "test/dir",
                "utf-8",
                1,
                None,
                4326)

    assert str(exception_assert) == str(e.value)


def test_read_shapefile_bulk_with_aggregation_25():
    """強化試験：Shapefileの空間ID集約取得、crs入力チェックNG
    + 試験詳細
      - 入力条件
        - crsにstr型の値を入力
      - 確認内容
        - 入力値チェックエラーの例外が発生すること
    """
    # 期待値定義
    # 例外を取得
    exception_assert = SpatialIdError('INPUT_VALUE_ERROR')

    # 試験実施
    # 例外発生を確認
    with pytest.raises(SpatialIdError) as e:
        read_shapefile_bulk_with_aggregation("test/dir", "utf-8", 1, 1, "4326")

    assert str(exception_assert) == str(e.value)


def test_read_shapefile_bulk_with_aggregation_26():
    """強化試験：Shapefileの空間ID集約取得、crsにNoneを入力
    + 試験詳細
      - 入力条件
        - crsにNoneを入力
      - 確認内容
        - 入力値チェックエラーの例外が発生すること
    """
    # 期待値定義
    # 例外を取得
    exception_assert = SpatialIdError('INPUT_VALUE_ERROR')

    # 試験実施
    # 例外発生を確認
    with pytest.raises(SpatialIdError) as e:
        read_shapefile_bulk_with_aggregation("test/dir", "utf-8", 1, 1, None)

    assert str(exception_assert) == str(e.value)


def test_read_shapefile_bulk_with_aggregation_27(
        geographic_shape_point_data_04
):
    """Shapefileの空間ID集約取得、閉塞していないポリゴン、閉塞チェックON
    + 試験詳細
      - 入力条件
        - ポリゴンの閉塞チェックフラグON
        - 読み込み対象のshapefileのポリゴンが閉塞していない
      - 確認内容
        - 三角形ポリゴンのモデルが閉塞していない場合の例外が発生すること
    """
    # 試験用データの定義
    # 試験用shalefileパス
    filepath = SHAPEFILE_DIR + 'test_read_shapefile_bulk_with_aggregation_27'

    # encoding
    encoding = 'utf-8'

    # 水平、垂直方向精度を定義
    h_zoom = 1
    v_zoom = 1

    # shapefileに格納するデータの座標系
    shapefile_crs = 4326

    # shapefile作成用の座標配列取得
    fan_point_list_01 = geographic_shape_point_data_04[0][0]
    fan_point_list_02 = geographic_shape_point_data_04[0][1]

    # shapefile生成
    w = sf.Writer(filepath)
    w.field('name', 'C')
    w.field('type', 'C')
    w.multipatch(
            [fan_point_list_01, fan_point_list_02],
            partTypes=[TRIANGLE_FAN, TRIANGLE_FAN])
    w.record('record1', 'FAN')
    w.close()

    # 期待値取得
    # 例外取得
    exception_assert = SpatialIdError('POLYGON_NOT_CLOSED_MODEL')

    # 試験実施
    # 例外発生確認
    with pytest.raises(SpatialIdError) as e:
        read_shapefile_bulk_with_aggregation(
                filepath,
                encoding,
                h_zoom,
                v_zoom,
                shapefile_crs,
                _custom_payload_func)

    # 例外メッセージ確認
    assert str(exception_assert) == str(e.value)


def test_read_shapefile_bulk_with_aggregation_28(
        geographic_shape_point_data_04
):
    """Shapefileの空間ID集約取得、閉塞していないポリゴン、閉塞チェックOFF
    + 試験詳細
      - 入力条件
        - ポリゴンの閉塞チェックフラグOFF
        - 閉塞していないモデルを含むshapefile読み込み
      - 確認内容
        - 戻り値のペイロードは、デフォルトの形式で集約されていること。
        - shapefileの全レコード文の空間IDに対し、集約が行われていること。
    """
    # 試験用データの定義
    # 試験用shalefileパス
    filepath = SHAPEFILE_DIR + 'test_read_shapefile_bulk_with_aggregation_28'

    # encoding
    encoding = 'utf-8'

    # 水平、垂直方向精度を定義
    h_zoom = 10
    v_zoom = 10

    # shapefileに格納するデータの座標系
    shapefile_crs = 4326

    # shapefile作成用の座標配列取得
    fan_point_list_01 = geographic_shape_point_data_04[0][0]
    fan_point_type_01 = geographic_shape_point_data_04[1][0]
    fan_point_list_02 = geographic_shape_point_data_04[0][1]
    fan_point_type_02 = geographic_shape_point_data_04[1][1]

    # shapefile生成
    w = sf.Writer(filepath)
    w.field('name', 'C')
    w.field('type', 'C')
    w.multipatch(
            [fan_point_list_01, fan_point_list_02],
            partTypes=[TRIANGLE_FAN, TRIANGLE_FAN])
    w.record('record1', 'FAN')
    w.close()

    # 期待値取得
    # ペイロードの期待値を定義
    shapefile_record_list = [['record1', 'FAN']]

    # fieldの期待値を定義
    shapefile_field_list = [['name', 'C', 50, 0], ['type', 'C', 50, 0]]

    # TRIANGLE_FANのSpatialPointオブジェクトを生成
    spatial_fan_point_list_01 = _get_spatial_point_objects(fan_point_list_01)
    spatial_fan_point_list_02 = _get_spatial_point_objects(fan_point_list_02)

    # SpatialTriangleオブジェクト生成
    spatial_triangle_list_01 = \
        _shaping_point_to_triangle(
                spatial_fan_point_list_01, fan_point_type_01)
    spatial_triangle_list_02 = \
        _shaping_point_to_triangle(
                spatial_fan_point_list_02, fan_point_type_02)

    # 空間ID取得対象のSpatialTriangleを格納した配列を定義
    triangle_list = spatial_triangle_list_01 + spatial_triangle_list_02

    # 空間IDの期待値を定義
    shapefile_spatial_id_list = [get_spatial_ids_on_polygons(
              triangle_list, [], h_zoom, v_zoom, needs_closed_checking=False)]

    # 集約後のペイロードの期待値を定義
    shapefile_record_list_agg = {}

    for ids, payloads in zip(shapefile_spatial_id_list, shapefile_record_list):

        for sp_id in ids:
            # 空間IDをキー値として未定義の場合、初期化
            if sp_id not in shapefile_record_list_agg:
                shapefile_record_list_agg[sp_id] = []

            # 辞書に、空間IDをキー、値をペイロードとして格納
            shapefile_record_list_agg[sp_id].append(payloads)

    # 試験実施
    # 戻り値取得
    param_return = read_shapefile_bulk_with_aggregation(
            filepath,
            encoding,
            h_zoom,
            v_zoom,
            shapefile_crs,
            needs_closed_checking=False)

    # 戻り値確認
    loop_cnt = 0

    # 空間IDの要素数確認
    param_return_spatial_id_list = param_return[0]
    assert len(shapefile_spatial_id_list) == len(param_return_spatial_id_list)

    # フィールドの要素一致確認
    assert shapefile_field_list == param_return[1]

    # 集約後のペイロードの要素一致確認
    param_return_payload = param_return[2]
    assert shapefile_record_list_agg == param_return_payload

    # 空間IDの値確認
    for ids in param_return_spatial_id_list:
        # APIから取得した空間IDは順不同のため比較前にソートし、格納された空間IDが一致していることを確認する。
        param_assert_ids = sorted(shapefile_spatial_id_list[loop_cnt])
        param_return_ids = sorted(ids)

        # 戻り値確認 - 空間ID
        assert param_assert_ids == param_return_ids

        # 全ての空間IDに対し集約が行われていることを確認
        for target_id in param_return_ids:
            assert True == (target_id in param_return_payload)

        loop_cnt += 1


def _get_spatial_point_objects(point_list: list, input_crs: int = None):
    """
    空間IDのPointオブジェクト生成メソッド
    'input_crs'が入力された場合、入力座標に対し、投影座標から地理座標への変換を行う。

    :param point_list: 座標が格納された配列
    :type point_list: list[tuple]
    :param input_crs: 入力座標の投影座標系、投影座標系でない場合入力しないこと
    :type input_crs: int

    :return: 空間IDのPointオブジェクトが格納されたリスト
    :rtype: list[SpatialPoint]
    """
    spatial_point_list = []

    if input_crs is None:
        for point in point_list:
            spatial_point_list.append(
                    SpatialPoint(point[0], point[1], point[2]))

    else:
        projected_point_list = []

        for point in point_list:
            projected_point_list.append(
                    Projected_Point(point[0], point[1], point[2]))

        spatial_point_list = \
            convert_projected_point_list_to_point_list(
                    projected_point_list, input_crs)

    return spatial_point_list


def _shaping_point_to_triangle(point_list: list, part_type: int):
    """
    パーツ情報を指定されたpart_typeにあうSpatialTriangleオブジェクトに成形

    :param point_list: 座標が格納された配列
    :type point_list: list[tuple]
    :param part_type: パーツタイプ
            0: triangle_strip,
            1: triangle_fan

    :return: part_type毎に成形されたSpatialTriangleを格納した配列
    :rtype: list[SpatialTriangle]
    """
    triangle_list = []

    if part_type == 0:
        for cnt in range(len(point_list) - 2):
            triangle_list.append(
                    SpatialTriangle(
                            point_list[cnt],
                            point_list[cnt + 1],
                            point_list[cnt + 2]))

    elif part_type == 1:
        for cnt in range(len(point_list) - 2):
            triangle_list.append(
                    SpatialTriangle(
                            point_list[0],
                            point_list[cnt + 1],
                            point_list[cnt + 2]))

    return triangle_list


def _custom_payload_func(payload_dict: dict):
    """試験用ペイロード集約関数

    :param payload_dict: Shapefileの空間ID取得時に返却されるペイロード値
    :type payload_dict: dict

    :return: 集約後のペイロード
    :rtype: dict
    """
    agg_payload_dict = {}

    for key, value in payload_dict.items():
        # 集約関数発生の例外確認用
        if value[0][0] == 'Exception':
            raise Exception('集約関数内例外発生')

        # 集約
        agg_payload_dict[key] = value
        agg_payload_dict[key].append('agg_func add')

    return agg_payload_dict

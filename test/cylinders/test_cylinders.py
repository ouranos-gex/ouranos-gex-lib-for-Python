#!/bin/env python3
# -*- coding: utf-8 -*-

import math
import pytest
from skspatial.objects import Point, Plane, Vector, Line

from SpatialId.common.object.enum import Point_Option
from SpatialId.common.exception import SpatialIdError
from SpatialId.common.object.point import Point as SpatialPoint
from SpatialId.common.object.point import Projected_Point
from SpatialId.shape.point import get_point_on_spatial_id
from SpatialId.shape.cylinders import (
    Rectangular,
    Capsule,
    _convert_spatial_point,
    _convert_spatial_projection_point,
    f_get_spatial_ids_on_cylinders,
    get_spatial_ids_on_cylinders
)

# CRSのデフォルト値のEPSGコード
PROJECT_CRS = 3857
GEOGRAPHIC_CRS = 4326
MINIMA = 1e-10

def test__convert_spatial_point_01():
    """skspatial.objects.PointからSpatialId.common.object.point.Pointへの変換
    + 試験詳細
      - 入力条件
        - skspatial.objects.Pointオブジェクトを渡す
      - 確認内容
        - SpatialId.common.object.point.Pointが変換されること
    """
    ## 試験用データの定義
    # skspatial.objects.Pointオブジェクト
    point = Point([139.74135387, 35.65809991, 6.6])

    ## 試験実施
    result = _convert_spatial_point(point)

    # 戻り値確認
    assert isinstance(result, SpatialPoint)
    assert result.lon == point[0]
    assert result.lat == point[1]
    assert result.alt == point[2]

def test__convert_spatial_projection_point_01():
    """skspatial.objects.PointからSpatialId.common.object.point.Projected_Pointへの変換
    + 試験詳細
      - 入力条件
        - skspatial.objects.Pointオブジェクトを渡す
      - 確認内容
        - SpatialId.common.object.point.Projected_Pointに変換されること
    """
    ## 試験用データの定義
    # skspatial.objects.Pointオブジェクト
    point = Point([11543.6883, 22916.2436, 6.6])

    ## 試験実施
    result = _convert_spatial_projection_point(point)

    # 戻り値確認
    assert isinstance(result, Projected_Point)
    assert result.x == point[0]
    assert result.y == point[1]
    assert result.alt == point[2]


def test_validate_args_type_01():
   """入力引数の型チェック(円柱の中心の接続点がlist以外)
   + 試験詳細
     - 入力条件
       - 引数の円柱の中心の接続点に文字列を指定
     - 確認内容
       - SpatialIdErrorが送出されること
   """
   ## 試験用データの定義
   # 引数の円柱の中心の接続点
   center = "dummy"
   # 半径
   radius = 2.2
   # 精度
   h_zoom = 10
   v_zoom = 10

   # 期待値の定義
   # 例外取得
   exeption_assert = SpatialIdError('INPUT_VALUE_ERROR')

   # 試験実施
   # 例外発生確認
   with pytest.raises(SpatialIdError) as e:
        # 空間ID取得
        param_return = \
           get_spatial_ids_on_cylinders(
               center,
               radius,
               h_zoom,
               v_zoom,
               GEOGRAPHIC_CRS
           )

   # 例外メッセージ確認
   assert str(e.value) == str(exeption_assert)


def test_validate_args_type_02():
    """入力引数の型チェック(半径がint,float以外)
    + 試験詳細
      - 入力条件
        - 引数の半径に文字列を指定
      - 確認内容
        - SpatialIdErrorが送出されること
    """
    ## 試験用データの定義
    # 引数の円柱の中心の接続点
    center = [SpatialPoint(139.74135387, 35.65809991, 6.6)]
    # 半径
    radius = "dummy"
    # 精度
    h_zoom = 10
    v_zoom = 10

    # 期待値の定義
    # 例外取得
    exeption_assert = SpatialIdError('INPUT_VALUE_ERROR')

    # 試験実施
    # 例外発生確認
    with pytest.raises(SpatialIdError) as e:
         # 空間ID取得
         param_return = \
            get_spatial_ids_on_cylinders(
                center,
                radius,
                h_zoom,
                v_zoom,
                GEOGRAPHIC_CRS
            )

    # 例外メッセージ確認
    assert str(e.value) == str(exeption_assert)


def test_validate_args_type_03():
    """入力引数の型チェック(水平方向の精度レベルがint以外)
    + 試験詳細
      - 入力条件
        - 引数の水平方向の精度レベルに文字列を指定
      - 確認内容
        - SpatialIdErrorが送出されること
    """
    ## 試験用データの定義
    # 引数の円柱の中心の接続点
    center = [SpatialPoint(139.74135387, 35.65809991, 6.6)]
    # 半径
    radius = 2.2
    # 精度
    h_zoom = "dummy"
    v_zoom = 10

    # 期待値の定義
    # 例外取得
    exeption_assert = SpatialIdError('INPUT_VALUE_ERROR')

    # 試験実施
    # 例外発生確認
    with pytest.raises(SpatialIdError) as e:
         # 空間ID取得
         param_return = \
            get_spatial_ids_on_cylinders(
                center,
                radius,
                h_zoom,
                v_zoom,
                GEOGRAPHIC_CRS
            )

    # 例外メッセージ確認
    assert str(e.value) == str(exeption_assert)


def test_validate_args_type_04():
    """入力引数の型チェック(垂直方向の精度レベルがint以外)
    + 試験詳細
      - 入力条件
        - 引数の垂直方向の精度レベルに文字列を指定
      - 確認内容
        - SpatialIdErrorが送出されること
    """
    ## 試験用データの定義
    # 引数の円柱の中心の接続点
    center = [SpatialPoint(139.74135387, 35.65809991, 6.6)]
    # 半径
    radius = 2.2
    # 精度
    h_zoom = 10
    v_zoom = "dummy"

    # 期待値の定義
    # 例外取得
    exeption_assert = SpatialIdError('INPUT_VALUE_ERROR')

    # 試験実施
    # 例外発生確認
    with pytest.raises(SpatialIdError) as e:
         # 空間ID取得
         param_return = \
            get_spatial_ids_on_cylinders(
                center,
                radius,
                h_zoom,
                v_zoom,
                GEOGRAPHIC_CRS
            )

    # 例外メッセージ確認
    assert str(e.value) == str(exeption_assert)


def test_validate_args_type_05():
    """入力引数の型チェック(座標参照系がint以外)
    + 試験詳細
      - 入力条件
        - 引数の座標参照系 に文字列を指定
      - 確認内容
        - SpatialIdErrorが送出されること
    """
    ## 試験用データの定義
    # 引数の円柱の中心の接続点
    center = [SpatialPoint(139.74135387, 35.65809991, 6.6)]
    # 半径
    radius = 2.2
    # 精度
    h_zoom = 10
    v_zoom = 10
    # 座標参照系
    crs = "dummy"

    # 期待値の定義
    # 例外取得
    exeption_assert = SpatialIdError('INPUT_VALUE_ERROR')

    # 試験実施
    # 例外発生確認
    with pytest.raises(SpatialIdError) as e:
         # 空間ID取得
         param_return = \
            get_spatial_ids_on_cylinders(
                center,
                radius,
                h_zoom,
                v_zoom,
                crs
            )

    # 例外メッセージ確認
    assert str(e.value) == str(exeption_assert)


def test_validate_args_type_06():
    """入力引数の型チェック(カプセル判定がbool以外)
    + 試験詳細
      - 入力条件
        - 引数のカプセル判定に文字列を指定
      - 確認内容
        - SpatialIdErrorが送出されること
    """
    ## 試験用データの定義
    # 引数の円柱の中心の接続点
    center = [SpatialPoint(139.74135387, 35.65809991, 6.6)]
    # 半径
    radius = 2.2
    # 精度
    h_zoom = 10
    v_zoom = 10
    # カプセル判定
    is_capsule = "dummy"

    # 期待値の定義
    # 例外取得
    exeption_assert = SpatialIdError('INPUT_VALUE_ERROR')

    # 試験実施
    # 例外発生確認
    with pytest.raises(SpatialIdError) as e:
         # 空間ID取得
         param_return = \
            get_spatial_ids_on_cylinders(
                center,
                radius,
                h_zoom,
                v_zoom,
                GEOGRAPHIC_CRS,
                is_capsule
            )

    # 例外メッセージ確認
    assert str(e.value) == str(exeption_assert)


def test_validate_args_type_07():
   """入力引数の型チェック(円柱の中心の接続点リストの要素がSpatialPoint以外)
   + 試験詳細
     - 入力条件
       - 引数の円柱の中心の接続点リストの要素に文字列を指定
     - 確認内容
       - SpatialIdErrorが送出されること
   """
   ## 試験用データの定義
   # 引数の円柱の中心の接続点
   center = ["dummy"]
   # 半径
   radius = 2.2
   # 精度
   h_zoom = 10
   v_zoom = 10
   # カプセル判定
   is_capsule = True

   # 期待値の定義
   # 例外取得
   exeption_assert = SpatialIdError('INPUT_VALUE_ERROR')

   # 試験実施
   # 例外発生確認
   with pytest.raises(SpatialIdError) as e:
        # 空間ID取得
        param_return = \
           get_spatial_ids_on_cylinders(
               center,
               radius,
               h_zoom,
               v_zoom,
               GEOGRAPHIC_CRS,
               is_capsule
           )

   # 例外メッセージ確認
   assert str(e.value) == str(exeption_assert)


def test_get_spatial_ids_on_cylinders_01():
    """円柱を複数つなげた経路が通る空間IDの取得失敗(円柱の半径が0以下)
    + 試験詳細
      - 入力条件
        - 引数の半径が0
      - 確認内容
        - SpatialIdErrorが送出されること
    """
    ## 試験用データの定義
    # skspatial.objects.Pointオブジェクト
    center = [SpatialPoint(139.74135387, 35.65809991, 6.6)]
    # 半径
    radius = 0.0
    # 精度
    h_zoom = 10
    v_zoom = 10

    # 期待値の定義
    # 例外取得
    exeption_assert = SpatialIdError('INPUT_VALUE_ERROR')

    # 試験実施
    # 例外発生確認
    with pytest.raises(SpatialIdError) as e:
         # 空間ID取得
         param_return = \
            get_spatial_ids_on_cylinders(
                center,
                radius,
                h_zoom,
                v_zoom,
                GEOGRAPHIC_CRS
            )

    # 例外メッセージ確認
    assert str(e.value) == str(exeption_assert)


def test_get_spatial_ids_on_cylinders_02():
   """経路が無い空間IDの取得
   + 試験詳細
     - 入力条件
       - 引数の接続点の数が0
     - 確認内容
       - 空リストが返却されること
   """
   ## 試験用データの定義
   # 円柱の中心の接続点
   center = []
   # 半径
   radius = 2.2
   # 精度
   h_zoom = 10
   v_zoom = 10


   # 試験実施
   # 空間ID取得
   param_return = \
       get_spatial_ids_on_cylinders(
           center,
           radius,
           h_zoom,
           v_zoom,
           GEOGRAPHIC_CRS
       )

   # 戻り値確認
   assert isinstance(param_return, list)
   assert len(param_return) == 0


def test_get_spatial_ids_on_cylinders_03():
    """円柱を複数つなげた経路が一点
    + 試験詳細
      - 入力条件
        - 円柱を複数つなげた経路が一点
      - 確認内容
        - 一つの球の空間IDが取得できること
    """
    ## 試験用データの定義
    # 始点, 終点
    center_spatial_id = '10/30/60/10/5'
    center_point = get_point_on_spatial_id(
        center_spatial_id, Point_Option.CENTER, GEOGRAPHIC_CRS)[0]
    ## 試験用データの定義
    # skspatial.objects.Pointオブジェクト
    center = [center_point]
    # 半径
    radius = 2.0
    # 精度
    h_zoom = 10
    v_zoom = 10

    # 期待値の定義
    spatial_ids_assert = ['10/30/60/10/5']

    # 試験実施
    # 空間ID取得
    param_return = \
        get_spatial_ids_on_cylinders(
            center,
            radius,
            h_zoom,
            v_zoom,
            GEOGRAPHIC_CRS
        )

    # 戻り値確認
    assert param_return == spatial_ids_assert


def test_get_spatial_ids_on_cylinders_04():
    """円柱を複数つなげた経路が同一二点
    + 試験詳細
      - 入力条件
        - 円柱を複数つなげた経路が2点だが同一
      - 確認内容
        - 一つの球の空間IDが取得できること
    """
    ## 試験用データの定義
    # 始点, 終点
    center_spatial_id = '10/30/60/10/5'
    center_point = get_point_on_spatial_id(
        center_spatial_id, Point_Option.CENTER, GEOGRAPHIC_CRS)[0]
    ## 試験用データの定義
    # skspatial.objects.Pointオブジェクト
    center = [center_point, center_point]
    # 半径
    radius = 2.0
    # 精度
    h_zoom = 10
    v_zoom = 10

    # 期待値の定義
    spatial_ids_assert = ['10/30/60/10/5']

    # 試験実施
    # 空間ID取得
    param_return = \
        get_spatial_ids_on_cylinders(
            center,
            radius,
            h_zoom,
            v_zoom,
            GEOGRAPHIC_CRS
        )

    # 戻り値確認
    assert param_return == spatial_ids_assert


def test_get_spatial_ids_on_cylinders_05():
   """円柱を複数つなげた経路が二点で円柱
   + 試験詳細
     - 入力条件
       - 円柱を複数つなげた経路が2点
       - 半径が円柱の長さ以下
       - カプセル判定がFalse
     - 確認内容
       - 円柱の空間IDが取得できること(戻り値が空でないこと)
       - 円柱の空間IDの要素が重複していないこと
   """
   ## 試験用データの定義
   # 始点, 終点
   start_spatial_id = '25/29798223/13211885/25/1'
   start_point = get_point_on_spatial_id(
       start_spatial_id, Point_Option.CENTER, GEOGRAPHIC_CRS)[0]
   end_spatial_id = '25/29798223/13211885/25/4'
   end_point = get_point_on_spatial_id(
       end_spatial_id, Point_Option.CENTER, GEOGRAPHIC_CRS)[0]
   ## 試験用データの定義
   # skspatial.objects.Pointオブジェクト
   center = [start_point, end_point]
   # 半径
   radius = 2.0
   # 精度
   h_zoom = 25
   v_zoom = 25
   # カプセル判定
   is_capsule = False

   # 試験実施
   # 空間ID取得
   param_return = \
       get_spatial_ids_on_cylinders(
           center,
           radius,
           h_zoom,
           v_zoom,
           GEOGRAPHIC_CRS,
           is_capsule
       )

   # 戻り値確認
   assert len(param_return) == len(set(param_return))
   assert len(set(param_return)) > 0

def test_get_spatial_ids_on_cylinders_06():
    """円柱を複数つなげた経路が3点で円柱かつ終点側の円柱の長さが半径より小さい
    + 試験詳細
      - 入力条件
        - 円柱を複数つなげた経路が3点
        - 終点側の経路はX方向のみに伸びるとする
        - 終点側の円柱の長さが半径より小さい
        - カプセル判定がFalse
      - 確認内容
        - Webメルカトル補正後の区間1の長さが半径以上であること
        - Webメルカトル補正後の区間2の長さが半径未満であること
        - 区間1の空間IDに経度インデクス「29798227」の空間IDがないこと
        - 区間2の空間IDに経度インデクス「29798227」の空間IDがないこと
        - 区間1と区間2の接続点の空間IDに経度インデクス「29798227」の空間IDがあること
        - 元の入力条件から取得した空間IDに経度インデクス「29798227」の空間IDがないこと
    """
    ## 試験用データの定義
    start_spatial_id = '25/29798223/13211885/25/3'
    start_point = get_point_on_spatial_id(
        start_spatial_id, Point_Option.CENTER, GEOGRAPHIC_CRS)[0]
    start_orth_point = \
        Point([start_point.lat, start_point.lon, start_point.alt])
    mid_spatial_id = '25/29798225/13211887/25/6'
    mid_point = get_point_on_spatial_id(
        mid_spatial_id, Point_Option.CENTER, GEOGRAPHIC_CRS)[0]
    mid_orth_point = \
        Point([mid_point.lat, mid_point.lon, mid_point.alt])
    # 中点からX方向にのみ伸ばす
    end_spatial_id = '25/29798226/13211887/25/6'
    end_point = get_point_on_spatial_id(
        end_spatial_id, Point_Option.CENTER, GEOGRAPHIC_CRS)[0]
    end_orth_point = \
        Point([end_point.lat, end_point.lon, end_point.alt])

    ## 試験用データの定義
    # 始点側の経路
    start_to_mid = [start_point, mid_point]
    # 終点側の経路
    mid_to_final = [mid_point, end_point]
    # 区間1と区間2の接続点
    mid = [mid_point]
    # 円柱を複数つなげた経路
    center = [start_point, mid_point, end_point]
    # Webメルカトル
    factor = 1 / math.cos(math.radians(start_point.lat))
    # 半径
    radius = 2.0 / factor
    # 精度
    h_zoom = 25
    v_zoom = 25
    # カプセル判定
    is_capsule = False
    # 閾値のX座標
    illeagal_x = 29798227

    # 始点、中点間の長さ(区間1)
    axis1 = Vector.from_points(
    start_orth_point, mid_orth_point
    )
    height1 = axis1.norm() / factor
    # 中点、終点間の長さ(区間2)
    axis2 = Vector.from_points(
        mid_orth_point, end_orth_point
    )
    height2 = axis2.norm() / factor

    # 試験実施
    # 空間ID取得
    all_return = \
        get_spatial_ids_on_cylinders(
            center,
            radius,
            h_zoom,
            v_zoom,
            GEOGRAPHIC_CRS,
            is_capsule
        )
    # 始点中点間の空間ID(区間1)
    start_to_mid_spatial_ids = \
        get_spatial_ids_on_cylinders(
            start_to_mid,
            radius,
            h_zoom,
            v_zoom,
            GEOGRAPHIC_CRS,
            is_capsule
        )
    # 中点終点間の空間ID取得(区間2)
    mid_to_end_spatial_ids = \
        get_spatial_ids_on_cylinders(
            mid_to_final,
            radius,
            h_zoom,
            v_zoom,
            GEOGRAPHIC_CRS,
            is_capsule = False
        )
    # 区間1と区間2の接続点の空間ID
    mid_spatial_ids = \
        get_spatial_ids_on_cylinders(
            mid,
            radius,
            h_zoom,
            v_zoom,
            GEOGRAPHIC_CRS,
            is_capsule = False
        )

    # 戻り値確認
    # 始点中点間の長さが半径以上であること(区間1)
    assert height1 >= radius
    # 中点終点間の長さが半径未満であること(区間2)
    assert height2 < radius

    # 区間1の空間IDに経度インデクス「29798227」の空間IDがないこと
    for spatial_id in start_to_mid_spatial_ids:
        exp_x = int(spatial_id.split("/")[1])
        assert exp_x != illeagal_x

    # 区間2の空間IDに経度インデクス「29798227」の空間IDがないこと
    for spatial_id in mid_to_end_spatial_ids:
        exp_x = int(spatial_id.split("/")[1])
        assert exp_x != illeagal_x

    # 区間1と区間2の接続点の空間IDに経度インデクス「29798227」の空間IDがあること
    flag = False
    for spatial_id in mid_spatial_ids:
        exp_x = int(spatial_id.split("/")[1])
        if exp_x == illeagal_x:
            flag = True
            break
    assert flag == True

    # 元の入力条件から取得した空間IDに経度インデクス「29798227」の空間IDがないこと
    for spatial_id in all_return:
        exp_x = int(spatial_id.split("/")[1])
        assert exp_x != illeagal_x


def test_get_spatial_ids_on_cylinders_07():
    """円柱を複数つなげた経路が3点で円柱かつ始点側の円柱の長さが半径より小さい
    + 試験詳細
      - 入力条件
        - 円柱を複数つなげた経路が3点
        - 始点側の経路はX方向のみに伸びるとする
        - 始点側の円柱の長さが半径より小さい
        - カプセル判定がFalse
      - 確認内容
        - Webメルカトル補正後の区間1の長さが半径未満であること
        - Webメルカトル補正後の区間2の長さが半径以上であること
        - 区間1の空間IDに経度インデクス「29798224」の空間IDがないこと
        - 区間2の空間IDに経度インデクス「29798224」の空間IDがないこと
        - 区間1と区間2の接続点の空間IDに経度インデクス「29798224」の空間IDがあること
        - 元の入力条件から取得した空間IDに経度インデクス「29798224」の空間IDがないこと
    """
    ## 試験用データの定義
    start_spatial_id = '25/29798223/13211885/25/6'
    start_point = get_point_on_spatial_id(
        start_spatial_id, Point_Option.CENTER, GEOGRAPHIC_CRS)[0]
    start_orth_point = \
        Point([start_point.lat, start_point.lon, start_point.alt])
    # 始点からX方向にのみ伸ばす
    mid_spatial_id = '25/29798222/13211885/25/6'
    mid_point = get_point_on_spatial_id(
            mid_spatial_id, Point_Option.CENTER, GEOGRAPHIC_CRS)[0]
    mid_orth_point = \
        Point([mid_point.lat, mid_point.lon, mid_point.alt])
    end_spatial_id = '25/29798220/13211883/25/3'
    end_point = get_point_on_spatial_id(
        end_spatial_id, Point_Option.CENTER, GEOGRAPHIC_CRS)[0]
    end_orth_point = \
        Point([end_point.lat, end_point.lon, end_point.alt])

    ## 試験用データの定義
    # 始点側の経路
    start_to_mid = [start_point, mid_point]
    # 終点側の経路
    mid_to_end = [mid_point, end_point]
    # 区間1と区間2の接続点
    mid = [mid_point]
    # 円柱を複数つなげた経路
    center = [start_point, mid_point, end_point]
    # Webメルカトル
    factor = 1 / math.cos(math.radians(start_point.lat))
    # 半径
    radius = 2.0 / factor
    # 精度
    h_zoom = 25
    v_zoom = 25
    # カプセル判定
    is_capsule = False
    # 閾値のX座標
    illeagal_x = 29798224

    # 始点、中点間の長さ(区間1)
    axis1 = Vector.from_points(
        start_orth_point, mid_orth_point
    )
    height1 = axis1.norm() / factor
    # 中点、終点間の長さ(区間2)
    axis2 = Vector.from_points(
        mid_orth_point, end_orth_point
    )
    height2 = axis2.norm() / factor

    # 試験実施
    # 空間ID取得
    all_return = \
        get_spatial_ids_on_cylinders(
            center,
            radius,
            h_zoom,
            v_zoom,
            GEOGRAPHIC_CRS,
            is_capsule
        )
    # 始点中点間の空間ID(区間1)
    start_to_mid_spatial_ids = \
        get_spatial_ids_on_cylinders(
            start_to_mid,
            radius,
            h_zoom,
            v_zoom,
            GEOGRAPHIC_CRS,
            is_capsule
        )
    # 中点終点間の空間ID取得(区間2)
    mid_to_end_spatial_ids = \
        get_spatial_ids_on_cylinders(
            mid_to_end,
            radius,
            h_zoom,
            v_zoom,
            GEOGRAPHIC_CRS,
            is_capsule = False
        )
    # 区間1と区間2の接続点の空間ID
    mid_spatial_ids = \
        get_spatial_ids_on_cylinders(
            mid,
            radius,
            h_zoom,
            v_zoom,
            GEOGRAPHIC_CRS,
            is_capsule = False
        )

    # 戻り値確認
    # 始点中点間の長さが半径未満であること(区間1)
    assert height1 < radius
    # 中点終点間の長さが半径以上であること(区間2)
    assert height2 >= radius

    # 区間1の空間IDに経度インデクス「29798224」の空間IDがないこと
    for spatial_id in start_to_mid_spatial_ids:
        exp_x = int(spatial_id.split("/")[1])
        assert exp_x != illeagal_x

    # 区間2の空間IDに経度インデクス「29798224」の空間IDがないこと
    for spatial_id in mid_to_end_spatial_ids:
        exp_x = int(spatial_id.split("/")[1])
        assert exp_x != illeagal_x

    # 区間1と区間2の接続点の空間IDに経度インデクス「29798224」の空間IDがあること
    flag = False
    for spatial_id in mid_spatial_ids:
        exp_x = int(spatial_id.split("/")[1])
        if exp_x == illeagal_x:
            flag = True
            break
    assert flag == True

    # 元の入力条件から取得した空間IDに経度インデクス「29798224」の空間IDがないこと
    for spatial_id in all_return:
        exp_x = int(spatial_id.split("/")[1])
        assert exp_x != illeagal_x


def test_get_spatial_ids_on_cylinders_08():
    """円柱を複数つなげた経路が二点で円柱
    + 試験詳細
      - 入力条件
        - 円柱を複数つなげた経路が2点
        - 半径が円柱の長さ以下
        - カプセル判定がFalse
      - 確認内容
        - 円柱の空間IDが取得できること(戻り値が空でないこと)
        - 円柱の空間IDの要素が重複していないこと
    """
    ## 試験用データの定義
    # 始点, 終点
    start_spatial_id = '25/29798223/13211885/25/5'
    start_point = get_point_on_spatial_id(
        start_spatial_id, Point_Option.CENTER, GEOGRAPHIC_CRS)[0]
    end_spatial_id = '25/29798225/13211887/25/6'
    end_point = get_point_on_spatial_id(
        end_spatial_id, Point_Option.CENTER, GEOGRAPHIC_CRS)[0]
    ## 試験用データの定義
    # skspatial.objects.Pointオブジェクト
    center = [start_point, end_point]
    # 半径
    radius = 2.2
    # 精度
    h_zoom = 25
    v_zoom = 25
    # カプセル判定
    is_capsule = False

    # 試験実施
    # 空間ID取得
    param_return = \
        get_spatial_ids_on_cylinders(
            center,
            radius,
            h_zoom,
            v_zoom,
            GEOGRAPHIC_CRS,
            is_capsule
        )

    # 戻り値確認
    assert len(param_return) > 0
    assert len(param_return) == len(set(param_return))

def test_get_spatial_ids_on_cylinders_09():
    """円柱を複数つなげた経路が二点でカプセル
    + 試験詳細
      - 入力条件
        - 円柱を複数つなげた経路が2点
        - カプセル判定がTrue
      - 確認内容
        - 円柱の空間IDが取得できること(戻り値が空でないこと)
        - 円柱の空間IDの要素が重複していないこと
    """
    ## 試験用データの定義
    # 始点, 終点
    start_spatial_id = '25/29798223/13211885/25/5'
    start_point = get_point_on_spatial_id(
        start_spatial_id, Point_Option.CENTER, GEOGRAPHIC_CRS)[0]
    end_spatial_id = '25/29798225/13211887/25/6'
    end_point = get_point_on_spatial_id(
        end_spatial_id, Point_Option.CENTER, GEOGRAPHIC_CRS)[0]
    ## 試験用データの定義
    # skspatial.objects.Pointオブジェクト
    center = [start_point, end_point]
    # 半径
    radius = 2.0
    # 精度
    h_zoom = 25
    v_zoom = 25
    # カプセル判定
    is_capsule = True

    # 試験実施
    # 空間ID取得
    param_return = \
        get_spatial_ids_on_cylinders(
            center,
            radius,
            h_zoom,
            v_zoom,
            GEOGRAPHIC_CRS,
            is_capsule
        )

    # 戻り値確認#
    assert len(param_return) > 0
    assert len(param_return) == len(set(param_return))

def test_get_spatial_ids_on_cylinders_10():
    """円柱を複数つなげた経路が3点でカプセル
    + 試験詳細
      - 入力条件
        - 円柱を複数つなげた経路が3点
        - カプセル判定がTrue
      - 確認内容
        - 円柱の空間IDが取得できること(戻り値が空でないこと)
        - 円柱の空間IDの要素が重複していないこと
    """
    ## 試験用データの定義
    # 始点, 終点
    start_spatial_id = '25/29798223/13211885/25/5'
    start_point = get_point_on_spatial_id(
        start_spatial_id, Point_Option.CENTER, GEOGRAPHIC_CRS)[0]
    mid_spatial_id = '25/29798224/13211886/25/5'
    mid_point = get_point_on_spatial_id(
        mid_spatial_id, Point_Option.CENTER, GEOGRAPHIC_CRS)[0]
    end_spatial_id = '25/29798225/13211887/25/6'
    end_point = get_point_on_spatial_id(
        end_spatial_id, Point_Option.CENTER, GEOGRAPHIC_CRS)[0]
    ## 試験用データの定義
    # skspatial.objects.Pointオブジェクト
    center = [start_point, mid_point, end_point]
    # 半径
    radius = 2.0
    # 精度
    h_zoom = 25
    v_zoom = 25
    # カプセル判定
    is_capsule = True

    # 試験実施
    # 空間ID取得
    param_return = \
        get_spatial_ids_on_cylinders(
            center,
            radius,
            h_zoom,
            v_zoom,
            GEOGRAPHIC_CRS,
            is_capsule
        )

    # 戻り値確認
    assert len(param_return) > 0
    assert len(param_return) == len(set(param_return))


def test_get_spatial_ids_on_cylinders_11(mocker):
    """円柱を複数つなげた経路の空間IDでその他例外発生
    + 試験詳細
      - 入力条件
        - 円柱を複数つなげた経路が2点
        - カプセル判定がTrue
        - ベクトル作成のメソッドをモック化する
      - 確認内容
        - その他例外が捕捉できること
    """
    # モック定義
    mocker.patch('skspatial.objects.Vector.from_points', side_effect=Exception)

    ## 試験用データの定義
    # 始点, 終点
    start_spatial_id = '25/30/60/25/5'
    start_point = get_point_on_spatial_id(
        start_spatial_id, Point_Option.CENTER, GEOGRAPHIC_CRS)[0]
    end_spatial_id = '25/32/62/25/6'
    end_point = get_point_on_spatial_id(
        end_spatial_id, Point_Option.CENTER, GEOGRAPHIC_CRS)[0]
    ## 試験用データの定義
    # 円柱を複数つなげた経路
    center = [start_point, end_point]
    # 半径
    radius = 2.0
    # 精度
    h_zoom = 25
    v_zoom = 25
    # カプセル判定
    is_capsule = True

    # 期待値の定義
    # 例外取得
    exeption_assert = SpatialIdError('OTHER_ERROR')

    # 試験実施
    # 例外発生確認
    with pytest.raises(SpatialIdError) as e:
         # 空間ID取得
         param_return = \
            get_spatial_ids_on_cylinders(
                center,
                radius,
                h_zoom,
                v_zoom,
                GEOGRAPHIC_CRS,
                is_capsule
            )

    # 例外メッセージ確認
    assert str(e.value) == str(exeption_assert)


def test_get_spatial_ids_on_cylinders_12():
    """円柱を複数つなげた経路が2点で円柱
    + 試験詳細
      - 入力条件
        - 円柱を複数つなげた経路が2点
        - 半径が円柱の長さ以下
        - カプセル判定がFalse
        - 同一座標の接続点が連続(誤差が微小な2点を指定)
      - 確認内容
        - 球の空間IDが取得できること(戻り値が空でないこと)
        - 球の空間IDの要素が重複していないこと
        - 同一とみなされる座標が連続している場合に異常終了しないこと
    """
    ## 試験用データの定義
    # 始点, 終点
    start_point = SpatialPoint(139.75364685058594, 35.6857446882, 1.0)
    mid_point = SpatialPoint(139.75364685058594 + 1e-14, 35.6857446882 + 1e-14, 1.0 + 1e-14)
        
    ## 試験用データの定義
    # 中心点
    center = [start_point, mid_point]
    # 半径
    radius = 2.0
    # 精度
    h_zoom = 25
    v_zoom = 25
    # カプセル判定
    is_capsule = False

    # 試験実施
    # 空間ID取得
    param_return = \
        get_spatial_ids_on_cylinders(
            center,
            radius,
            h_zoom,
            v_zoom,
            GEOGRAPHIC_CRS,
            is_capsule
        )

    # 戻り値確認
    assert len(param_return) == len(set(param_return))
    assert len(param_return) > 0

def test_get_spatial_ids_on_cylinders_13():
    """円柱を複数つなげた経路が2点で円柱
    + 試験詳細
      - 入力条件
        - 円柱を複数つなげた経路が2点
        - 半径が入力可能範囲の下限
        - カプセル判定がFalse
        - 同一座標の接続点
        - 接続点のZ座標が極大
      - 確認内容
        - 球の空間IDが取得できること(戻り値が空でないこと)
        - 球の空間IDの要素が重複していないこと
        - 同一とみなされる座標が連続している場合に異常終了しないこと
    """
    ## 試験用データの定義
    # 始点, 終点
    start_point = SpatialPoint(139.75364685058594, 35.6857446882, 1e+8)
    mid_point = SpatialPoint(139.75364685058594, 35.6857446882, 1e+8)
        
    ## 試験用データの定義
    # 中心点
    center = [start_point, mid_point]
    # 半径
    radius = 1e-8 + 1e-9
    # 精度
    h_zoom = 25
    v_zoom = 25
    # カプセル判定
    is_capsule = False

    # 試験実施
    # 空間ID取得
    param_return = \
        get_spatial_ids_on_cylinders(
            center,
            radius,
            h_zoom,
            v_zoom,
            GEOGRAPHIC_CRS,
            is_capsule
        )

    # 戻り値確認
    assert len(param_return) == len(set(param_return))
    assert len(param_return) > 0



def test_f_validate_args_type_01():
    """入力引数の型チェック(円柱の中心の接続点がlist以外)
    + 試験詳細
      - 入力条件
        - 引数の円柱の中心の接続点に文字列を指定
      - 確認内容
        - SpatialIdErrorが送出されること
    """
    ## 試験用データの定義
    # 引数の円柱の中心の接続点
    center = "dummy"
    # 半径
    radius = 2.2
    # 精度
    h_zoom = 10
    v_zoom = 10

    # 期待値の定義
    # 例外取得
    exeption_assert = SpatialIdError('INPUT_VALUE_ERROR')

    # 試験実施
    # 例外発生確認
    with pytest.raises(SpatialIdError) as e:
         # 空間ID取得
         param_return = \
            f_get_spatial_ids_on_cylinders(
                center,
                radius,
                h_zoom,
                GEOGRAPHIC_CRS
            )

    # 例外メッセージ確認
    assert str(e.value) == str(exeption_assert)


def test_f_validate_args_type_02():
    """入力引数の型チェック(半径がint,float以外)
    + 試験詳細
      - 入力条件
        - 引数の半径に文字列を指定
      - 確認内容
        - SpatialIdErrorが送出されること
    """
    ## 試験用データの定義
    # 引数の円柱の中心の接続点
    center = [SpatialPoint(139.74135387, 35.65809991, 6.6)]
    # 半径
    radius = "dummy"
    # 精度
    zoom = 10

    # 期待値の定義
    # 例外取得
    exeption_assert = SpatialIdError('INPUT_VALUE_ERROR')

    # 試験実施
    # 例外発生確認
    with pytest.raises(SpatialIdError) as e:
         # 空間ID取得
         param_return = \
            f_get_spatial_ids_on_cylinders(
                center,
                radius,
                zoom,
                GEOGRAPHIC_CRS
            )

    # 例外メッセージ確認
    assert str(e.value) == str(exeption_assert)


def test_f_validate_args_type_03():
    """入力引数の型チェック(精度レベルがint以外)
    + 試験詳細
      - 入力条件
        - 引数の精度レベルに文字列を指定
      - 確認内容
        - SpatialIdErrorが送出されること
    """
    ## 試験用データの定義
    # 引数の円柱の中心の接続点
    center = [SpatialPoint(139.74135387, 35.65809991, 6.6)]
    # 半径
    radius = 2.2
    # 精度
    zoom = "dummy"

    # 期待値の定義
    # 例外取得
    exeption_assert = SpatialIdError('INPUT_VALUE_ERROR')

    # 試験実施
    # 例外発生確認
    with pytest.raises(SpatialIdError) as e:
         # 空間ID取得
         param_return = \
            f_get_spatial_ids_on_cylinders(
                center,
                radius,
                zoom,
                GEOGRAPHIC_CRS
            )

    # 例外メッセージ確認
    assert str(e.value) == str(exeption_assert)



def test_f_validate_args_type_04():
    """入力引数の型チェック(座標参照系がint以外)
    + 試験詳細
      - 入力条件
        - 引数の座標参照系 に文字列を指定
      - 確認内容
        - SpatialIdErrorが送出されること
    """
    ## 試験用データの定義
    # 引数の円柱の中心の接続点
    center = [SpatialPoint(139.74135387, 35.65809991, 6.6)]
    # 半径
    radius = 2.2
    # 精度
    zoom = 10
    # 座標参照系
    crs = "dummy"

    # 期待値の定義
    # 例外取得
    exeption_assert = SpatialIdError('INPUT_VALUE_ERROR')

    # 試験実施
    # 例外発生確認
    with pytest.raises(SpatialIdError) as e:
         # 空間ID取得
         param_return = \
            f_get_spatial_ids_on_cylinders(
                center,
                radius,
                zoom,
                crs
            )

    # 例外メッセージ確認
    assert str(e.value) == str(exeption_assert)


def test_f_validate_args_type_05():
    """入力引数の型チェック(カプセル判定がbool以外)
    + 試験詳細
      - 入力条件
        - 引数のカプセル判定に文字列を指定
      - 確認内容
        - SpatialIdErrorが送出されること
    """
    ## 試験用データの定義
    # 引数の円柱の中心の接続点
    center = [SpatialPoint(139.74135387, 35.65809991, 6.6)]
    # 半径
    radius = 2.2
    # 精度
    zoom = 10
    # カプセル判定
    is_capsule = "dummy"

    # 期待値の定義
    # 例外取得
    exeption_assert = SpatialIdError('INPUT_VALUE_ERROR')

    # 試験実施
    # 例外発生確認
    with pytest.raises(SpatialIdError) as e:
         # 空間ID取得
         param_return = \
            f_get_spatial_ids_on_cylinders(
                center,
                radius,
                zoom,
                GEOGRAPHIC_CRS,
                is_capsule
            )

    # 例外メッセージ確認
    assert str(e.value) == str(exeption_assert)


def test_f_validate_args_type_06():
    """入力引数の型チェック(円柱の中心の接続点リストの要素がSpatialPoint以外)
    + 試験詳細
      - 入力条件
        - 引数の円柱の中心の接続点リストの要素に文字列を指定
      - 確認内容
        - SpatialIdErrorが送出されること
    """
    ## 試験用データの定義
    # 引数の円柱の中心の接続点
    center = ["dummy"]
    # 半径
    radius = 2.2
    # 精度
    zoom = 10
    # カプセル判定
    is_capsule = True

    # 期待値の定義
    # 例外取得
    exeption_assert = SpatialIdError('INPUT_VALUE_ERROR')

    # 試験実施
    # 例外発生確認
    with pytest.raises(SpatialIdError) as e:
         # 空間ID取得
         param_return = \
            f_get_spatial_ids_on_cylinders(
                center,
                radius,
                zoom,
                GEOGRAPHIC_CRS,
                is_capsule
            )

    # 例外メッセージ確認
    assert str(e.value) == str(exeption_assert)


def test_f_validate_args_type_07():
    """入力引数の型チェック(衝突判定オプションオプションがbool以外)
    + 試験詳細
      - 入力条件
        - 引数の衝突判定オプションに文字列を指定
      - 確認内容
        - SpatialIdErrorが送出されること
    """
    ## 試験用データの定義
    # 引数の円柱の中心の接続点
    center = [SpatialPoint(139.74135387, 35.65809991, 6.6)]
    # 半径
    radius = 2.2
    # 精度
    zoom = 10
    # カプセル判定
    is_capsule = True
    # 衝突判定
    is_precision = "dummy"

    # 期待値の定義
    # 例外取得
    exeption_assert = SpatialIdError('INPUT_VALUE_ERROR')

    # 試験実施
    # 例外発生確認
    with pytest.raises(SpatialIdError) as e:
         # 空間ID取得
         param_return = \
            f_get_spatial_ids_on_cylinders(
                center,
                radius,
                zoom,
                GEOGRAPHIC_CRS,
                is_capsule,
                is_precision
            )

    # 例外メッセージ確認
    assert str(e.value) == str(exeption_assert)


def test_f_get_spatial_ids_on_cylinders_01():
    """円柱を複数つなげた経路が通る空間IDの取得失敗(円柱の半径が0以下)
    + 試験詳細
      - 入力条件
        - 引数の半径が0
      - 確認内容
        - SpatialIdErrorが送出されること
    """
    ## 試験用データの定義
    # skspatial.objects.Pointオブジェクト
    center = [SpatialPoint(139.74135387, 35.65809991, 6.6)]
    # 半径
    radius = 0.0
    # 精度
    zoom = 10

    # 期待値の定義
    # 例外取得
    exeption_assert = SpatialIdError('INPUT_VALUE_ERROR')

    # 試験実施
    # 例外発生確認
    with pytest.raises(SpatialIdError) as e:
         # 空間ID取得
         param_return = \
            f_get_spatial_ids_on_cylinders(
                center,
                radius,
                zoom,
                GEOGRAPHIC_CRS
            )

    # 例外メッセージ確認
    assert str(e.value) == str(exeption_assert)

def test_f_get_spatial_ids_on_cylinders_02():
    """円柱を複数つなげた経路が通る空間IDの取得成功
    + 試験詳細
      - 入力条件
        - 接続点が1点
      - 確認内容
        - 接続点の拡張空間IDが空間IDのフォーマットに変換されて返却されること
    """
    ## 試験用データの定義
    center_id = '10/30/60/10/5'
    center_point = get_point_on_spatial_id(
        center_id, Point_Option.CENTER, GEOGRAPHIC_CRS)[0]
    # 半径
    radius = 2.0
    # 精度
    zoom = 10

    # 期待値の定義
    expected_spatialIds = ['10/5/30/60']

    # 試験実施
    # 空間ID取得
    param_return = \
    f_get_spatial_ids_on_cylinders(
        [center_point],
        radius,
        zoom,
        GEOGRAPHIC_CRS
    )

    # 戻り値の一致確認
    assert param_return == expected_spatialIds


class TestRectangular:

    @pytest.fixture()
    def rectangular(self):

        ## 試験用データの定義
        # 始点, 終点
        start_spatial_id = '25/29798222/13211885/25/1'
        end_spatial_id = '25/29798223/13211885/25/4'
        start_point = get_point_on_spatial_id(
            start_spatial_id, Point_Option.CENTER, PROJECT_CRS)[0]
        end_point = get_point_on_spatial_id(
            end_spatial_id, Point_Option.CENTER, PROJECT_CRS)[0]
        start_orth_point = \
            Point([start_point.x, start_point.y, start_point.alt])
        end_orth_point = \
            Point([end_point.x, end_point.y, end_point.alt])
        # Webメルカトル
        factor = 1 / math.cos(math.radians(start_point.lat))
        # 直交座標のZ成分にWebメルカトルを乗算
        start_orth_point[2] *= factor
        end_orth_point[2] *= factor
        # 半径
        radius = 2.2
        # 精度レベル
        v_zoom = 10
        h_zoom = 10

        return Rectangular(
            start_orth_point,
            end_orth_point,
            radius,
            v_zoom,
            h_zoom,
            factor
        )

    def test__init_axis_01(self, rectangular):
        """直方体の軸・ベクトル決定(軸のY成分0)
        + 試験詳細
          - 入力条件
            - 始点・終点のY成分が同じ座標が入力に指定されること
          - 確認内容
            - 想定した軸・ベクトルが初期化されること
        """
        # 軸ベクトル
        axis_vector = Vector.from_points(
            rectangular._start_point, rectangular._end_point
        )

        ## 期待値の定義
        # 図形の高さ
        height_assert = axis_vector.norm()
        # 軸の半径長さ直交ベクトル
        radius_orth_vector_assert = Vector([0, 2.70877268, 0])
        # 軸方向の単位ベクトル
        unit_axis_vector_assert = Vector([0.30765281, 0, 0.95149869])
        # 円との接点
        contact_point_assert = \
            radius_orth_vector_assert + rectangular._start_point
        # 軸ベクトルと直交ベクトルの法線ベクトル(大きさは機体半径)
        radius_normal_vector_assert = \
            2.2 * 1.2312603082601323 * radius_orth_vector_assert.cross(
               unit_axis_vector_assert).unit()

        ## 試験実施
        rectangular._init_axis()

        # 戻り値確認
        assert height_assert == rectangular._height
        assert (radius_orth_vector_assert - rectangular._radius_orth_vector).is_zero(abs_tol=MINIMA)
        assert (unit_axis_vector_assert - rectangular._unit_axis_vector).is_zero(abs_tol=MINIMA)
        assert (contact_point_assert - rectangular._contact_point).is_zero(abs_tol=MINIMA)
        assert (radius_normal_vector_assert - rectangular._radius_normal_vector).is_zero(abs_tol=MINIMA)

    def test__init_axis_02(self):
        """直方体の軸・ベクトル決定(軸のZ成分0)
        + 試験詳細
          - 入力条件
            - 始点・終点のZ成分が同じ座標が入力に指定されること
          - 確認内容
            - 想定した軸・ベクトルが初期化されること
        """
        ## 試験用データの定義
        # 始点, 終点
        start_spatial_id = '25/2979823/13211885/25/1'
        end_spatial_id = '25/29798224/13211884/25/1'
        start_point = get_point_on_spatial_id(
            start_spatial_id, Point_Option.CENTER, PROJECT_CRS)[0]
        end_point = get_point_on_spatial_id(
            end_spatial_id, Point_Option.CENTER, PROJECT_CRS)[0]
        start_orth_point = \
            Point([start_point.x, start_point.y, start_point.alt])
        end_orth_point = \
            Point([end_point.x, end_point.y, end_point.alt])
        # Webメルカトル
        factor = 1 / math.cos(math.radians(start_point.lat))
        # 直交座標のZ成分にWebメルカトルを乗算
        start_orth_point[2] *= factor
        end_orth_point[2] *= factor
        # 半径
        radius = 2.2
        # 精度レベル
        v_zoom = 10
        h_zoom = 10

        # オブジェクト初期化
        rectangular = Rectangular(
            start_orth_point,
            end_orth_point,
            radius,
            v_zoom,
            h_zoom,
            factor
        )

        # 軸ベクトル
        axis_vector = Vector.from_points(
            rectangular._start_point, rectangular._end_point
        )

        ## 期待値の定義
        # 図形の高さ
        height_assert = axis_vector.norm()
        # 軸の半径長さ直交ベクトル
        radius_orth_vector_assert = Vector([0, 0, 2.70877268])
        # 軸方向の単位ベクトル
        unit_axis_vector_assert = Vector([1.0,  3.72878221e-08, 0])
        # 円との接点
        contact_point_assert = \
            radius_orth_vector_assert + rectangular._start_point
        # 軸ベクトルと直交ベクトルの法線ベクトル(大きさは機体半径)
        radius_normal_vector_assert = \
            2.2 * 1.2312603082601323 * radius_orth_vector_assert.cross(
               unit_axis_vector_assert).unit()

        ## 試験実施
        rectangular._init_axis()

        # 戻り値確認
        assert height_assert == rectangular._height
        assert (radius_orth_vector_assert - rectangular._radius_orth_vector).is_zero(abs_tol=MINIMA)
        assert (unit_axis_vector_assert - rectangular._unit_axis_vector).is_zero(abs_tol=MINIMA)
        assert (contact_point_assert - rectangular._contact_point).is_zero(abs_tol=MINIMA)
        assert (radius_normal_vector_assert - rectangular._radius_normal_vector).is_zero(abs_tol=MINIMA)

    def test__init_axis_03(self):
        """直方体の軸・ベクトル決定(軸のX成分0)
        + 試験詳細
          - 入力条件
            - 始点・終点のX成分が同じ座標が入力に指定されること
          - 確認内容
            - 想定した軸・ベクトルが初期化されること
        """
        ## 試験用データの定義
        # 始点, 終点
        start_spatial_id = '25/29798223/13211885/25/1'
        end_spatial_id = '25/29798223/13211887/25/4'
        start_point = get_point_on_spatial_id(
            start_spatial_id, Point_Option.CENTER, PROJECT_CRS)[0]
        end_point = get_point_on_spatial_id(
            end_spatial_id, Point_Option.CENTER, PROJECT_CRS)[0]
        start_orth_point = \
            Point([start_point.x, start_point.y, start_point.alt])
        end_orth_point = \
            Point([end_point.x, end_point.y, end_point.alt])
        # Webメルカトル
        factor = 1 / math.cos(math.radians(start_point.lat))
        # 直交座標のZ成分にWebメルカトルを乗算
        start_orth_point[2] *= factor
        end_orth_point[2] *= factor
        # 半径
        radius = 2.2
        # 精度レベル
        v_zoom = 10
        h_zoom = 10

        # オブジェクト初期化
        rectangular = Rectangular(
            start_orth_point,
            end_orth_point,
            radius,
            v_zoom,
            h_zoom,
            factor
        )

        # 軸ベクトル
        axis_vector = Vector.from_points(
            rectangular._start_point, rectangular._end_point
        )

        ## 期待値の定義
        # 図形の高さ
        height_assert = axis_vector.norm()
        # 軸の半径長さ直交ベクトル
        radius_orth_vector_assert = Vector([0,  -2.27460832, -1.47092027])
        # 軸方向の単位ベクトル
        unit_axis_vector_assert = Vector([0, -0.54302093, 0.83971916])
        # 円との接点
        contact_point_assert = \
            radius_orth_vector_assert + rectangular._start_point
        # 軸ベクトルと直交ベクトルの法線ベクトル(大きさは機体半径)
        radius_normal_vector_assert = \
            2.2 * 1.2312603082601323 * radius_orth_vector_assert.cross(
               unit_axis_vector_assert).unit()

        ## 試験実施
        rectangular._init_axis()

        # 戻り値確認
        assert height_assert == rectangular._height
        assert (radius_orth_vector_assert - rectangular._radius_orth_vector).is_zero(abs_tol=MINIMA)
        assert (unit_axis_vector_assert - rectangular._unit_axis_vector).is_zero(abs_tol=MINIMA)
        assert (contact_point_assert - rectangular._contact_point).is_zero(abs_tol=MINIMA)
        assert (radius_normal_vector_assert - rectangular._radius_normal_vector).is_zero(abs_tol=MINIMA)


    def test__calc_rectangular_apex_01(self, rectangular):
        """直方体の頂点を取得し、境界面との交点を取得
        + 試験詳細
          - 入力条件
            - 軸・ベクトルを決定済みのオブジェクトに処理を行う
          - 確認内容
            - 直方体の辺の空間IDが取得できること
        """
        ## 試験用データの定義
        # 始点, 終点
        start_spatial_id = '10/30/60/10/5'
        end_spatial_id = '10/31/62/10/8'
        start_point = get_point_on_spatial_id(
            start_spatial_id, Point_Option.CENTER, PROJECT_CRS)[0]
        end_point = get_point_on_spatial_id(
            end_spatial_id, Point_Option.CENTER, PROJECT_CRS)[0]
        start_orth_point = \
            Point([start_point.x, start_point.y, start_point.alt])
        end_orth_point = \
            Point([end_point.x, end_point.y, end_point.alt])
        # Webメルカトル
        factor = 1 / math.cos(math.radians(start_point.lat))
        # 直交座標のZ成分にWebメルカトルを乗算
        start_orth_point[2] *= factor
        end_orth_point[2] *= factor
        # 半径
        radius = 2.2
        # 精度レベル
        v_zoom = 10
        h_zoom = 10

        # オブジェクト初期化
        rectangular = Rectangular(
            start_orth_point,
            end_orth_point,
            radius,
            v_zoom,
            h_zoom,
            factor
        )

        ## 期待値の定義
        # 辺の空間ID
        edge_spatial_ids_assert = {
            '10/30/60/10/5',
            '10/30/60/10/6',
            '10/30/61/10/6',
            '10/30/61/10/7',
            '10/31/61/10/6',
            '10/31/61/10/7',
            '10/31/62/10/7',
            '10/31/62/10/8'
        }

        ## 試験実施
        rectangular._init_axis()
        rectangular._calc_rectangular_apex()

        # 戻り値確認
        assert rectangular._all_spatial_ids == edge_spatial_ids_assert


    def test__get_voxel_plane_cross_point_03(self, rectangular):
        """第二ボクセル境界との接点が0個
        + 試験詳細
          - 入力
            - 第一ボクセル境界面の図形に対して、接触しない第二ボクセル境界面を指定する
          - 確認内容
            - 第一ボクセル境界面と辺との交点のみが全交点リストに格納されること
        """
        ## 試験用データの定義
        # 直方体の辺全て
        vertex = [
            Point([0, 0, 0]),
            Point([3, 4, 0]),
            Point([3, 4, 5]),
            Point([0, 0, 5]),
            Point([8, -6, 0]),
            Point([11, -2, 0]),
            Point([11, -2, 5]),
            Point([8, -6, 5]),
        ]
        rect_lines = [
            Line.from_points(vertex[0], vertex[1]),
            Line.from_points(vertex[0], vertex[3]),
            Line.from_points(vertex[2], vertex[3]),
            Line.from_points(vertex[1], vertex[2]),
            Line.from_points(vertex[0], vertex[4]),
            Line.from_points(vertex[1], vertex[5]),
            Line.from_points(vertex[2], vertex[6]),
            Line.from_points(vertex[3], vertex[7]),
            Line.from_points(vertex[4], vertex[5]),
            Line.from_points(vertex[4], vertex[7]),
            Line.from_points(vertex[6], vertex[7]),
            Line.from_points(vertex[5], vertex[6]),
        ]
        # 第一ボクセル境界
        first_voxel_planes = [
            Plane(point=[1, 0, 0], normal=[1, 0, 0])
        ]
        second_voxel_planes = [
            Plane(point=[0, 5, 0], normal=[0, 1, 0])
        ]
        third_voxel_planes = [
            Plane(point=[0, 0, 1], normal=[0, 0, 1])
        ]
        # 処理時に基準とする軸
        axis = 2

        # 期待値の定義
        voxel_plane_cross_point_assert = [
            Point([1. , -0.75,  0.]),
            Point([1. , -0.75,  5.]),
            Point([1. , 1.33333333, 0.]),
            Point([1. , 1.33333333, 5.])
        ]

        ## 試験実施
        rectangular._get_voxel_plane_cross_point(
            rect_lines,
            first_voxel_planes,
            second_voxel_planes,
            third_voxel_planes,
            axis
        )

        # 結果確認
        assert len(voxel_plane_cross_point_assert) == len(rectangular._all_cross_points)
        for exp_point in voxel_plane_cross_point_assert:
            result = False
            for act_point in rectangular._all_cross_points:
                if Vector.from_points(exp_point, act_point).is_zero(abs_tol=MINIMA):
                    result = True
                    break

            assert result


    def test__get_voxel_plane_cross_point_04(self, rectangular):
        """第二ボクセル境界との接点が1個
        + 試験詳細
          - 入力
            - 第一ボクセル境界面の図形に対して、一点で接触する第二ボクセル境界面を指定する
          - 確認内容
            - 第一ボクセル境界面と辺との交点のみが全交点リストに格納されること
        """
        ## 試験用データの定義
        # 直方体の辺全て
        vertex = [
            Point([0, 0, 0]),
            Point([0, 3, 4]),
            Point([0, 0, 8]),
            Point([0, -3, 4]),
            Point([8, 0, 0]),
            Point([8, 3, 4]),
            Point([8, 0, 8]),
            Point([8, -3, 4]),
        ]
        rect_lines = [
            Line.from_points(vertex[0], vertex[1]),
            Line.from_points(vertex[0], vertex[3]),
            Line.from_points(vertex[2], vertex[3]),
            Line.from_points(vertex[1], vertex[2]),
            Line.from_points(vertex[0], vertex[4]),
            Line.from_points(vertex[1], vertex[5]),
            Line.from_points(vertex[2], vertex[6]),
            Line.from_points(vertex[3], vertex[7]),
            Line.from_points(vertex[4], vertex[5]),
            Line.from_points(vertex[4], vertex[7]),
            Line.from_points(vertex[6], vertex[7]),
            Line.from_points(vertex[5], vertex[6]),
        ]
        # 第一ボクセル境界
        first_voxel_planes = [
            Plane(point=[1, 0, 0], normal=[1, 0, 0])
        ]
        second_voxel_planes = [
            Plane(point=[0, 3, 0], normal=[0, 1, 0])
        ]
        third_voxel_planes = [
            Plane(point=[0, 0, 1], normal=[0, 0, 1])
        ]
        # 処理時に基準とする軸
        axis = 2

        # 期待値の定義
        voxel_plane_cross_point_assert = [
            Point([1. , 3.,  4.]),
            Point([1. , -3.,  4.]),
            Point([1. , 0., 0.]),
            Point([1. , 0., 8.])
        ]

        ## 試験実施
        rectangular._get_voxel_plane_cross_point(
            rect_lines,
            first_voxel_planes,
            second_voxel_planes,
            third_voxel_planes,
            axis
        )

        # 結果確認
        assert len(voxel_plane_cross_point_assert) == len(rectangular._all_cross_points)
        for exp_point in voxel_plane_cross_point_assert:
            result = False
            for act_point in rectangular._all_cross_points:
                if Vector.from_points(exp_point, act_point).is_zero(abs_tol=MINIMA):
                    result = True
                    break

            assert result


    def test__get_voxel_plane_cross_point_05(self, rectangular):
        """第二ボクセル交線と第三ボクセル境界との接点が0個
        + 試験詳細
          - 入力
            - 第一ボクセル境界面の図形に対して、二点で接触する第二ボクセル境界面を指定する
            - 第三ボクセル境界を上記の二点のZ値の範囲外とする
          - 確認内容
            - 第一ボクセル境界面と辺との交点
            - 第一ボクセル境界面と辺との交点の線と第二ボクセル境界面の交点
            - のみが全交点リストに格納されること
        """
        ## 試験用データの定義
        # 直方体の辺全て
        vertex = [
            Point([0, 0, 0]),
            Point([0, 3, 4]),
            Point([0, 0, 8]),
            Point([0, -3, 4]),
            Point([8, 0, 0]),
            Point([8, 3, 4]),
            Point([8, 0, 8]),
            Point([8, -3, 4]),
        ]
        rect_lines = [
            Line.from_points(vertex[0], vertex[1]),
            Line.from_points(vertex[0], vertex[3]),
            Line.from_points(vertex[2], vertex[3]),
            Line.from_points(vertex[1], vertex[2]),
            Line.from_points(vertex[0], vertex[4]),
            Line.from_points(vertex[1], vertex[5]),
            Line.from_points(vertex[2], vertex[6]),
            Line.from_points(vertex[3], vertex[7]),
            Line.from_points(vertex[4], vertex[5]),
            Line.from_points(vertex[4], vertex[7]),
            Line.from_points(vertex[6], vertex[7]),
            Line.from_points(vertex[5], vertex[6]),
        ]
        # 第一ボクセル境界
        first_voxel_planes = [
            Plane(point=[1, 0, 0], normal=[1, 0, 0])
        ]
        second_voxel_planes = [
            Plane(point=[0, 2, 0], normal=[0, 1, 0])
        ]
        third_voxel_planes = [
            Plane(point=[0, 0, 8], normal=[0, 0, 1])
        ]
        # 処理時に基準とする軸
        axis = 2

        # 期待値の定義
        voxel_plane_cross_point_assert = [
            Point([1. , 3.,  4.]),
            Point([1. , -3.,  4.]),
            Point([1. , 0., 0.]),
            Point([1. , 0., 8.]),
            Point([1. , 2.,  4.]),
            Point([1. , 2., 2.66666667]),
            Point([1. , 2., 5.33333333])
        ]

        ## 試験実施
        rectangular._get_voxel_plane_cross_point(
            rect_lines,
            first_voxel_planes,
            second_voxel_planes,
            third_voxel_planes,
            axis
        )

        # 結果確認
        assert len(voxel_plane_cross_point_assert) == len(rectangular._all_cross_points)
        for exp_point in voxel_plane_cross_point_assert:
            result = False
            for act_point in rectangular._all_cross_points:
                if Vector.from_points(exp_point, act_point).is_zero(abs_tol=MINIMA):
                    result = True
                    break

            assert result


    def test__get_voxel_plane_cross_point_06(self, rectangular):
        """第一ボクセル境界をY平面にする(カバレッジ網羅)
        + 試験詳細
          - 入力
            - 第一ボクセル境界面のとしてY平面を指定する
          - 確認内容
            - 全交点リストに想定した交点が格納されること
        """
        ## 試験用データの定義
        # 直方体の辺全て
        vertex = [
            Point([0, 0, 0]),
            Point([0, 3, 4]),
            Point([0, 0, 8]),
            Point([0, -3, 4]),
            Point([8, 0, 0]),
            Point([8, 3, 4]),
            Point([8, 0, 8]),
            Point([8, -3, 4]),
        ]
        rect_lines = [
            Line.from_points(vertex[0], vertex[1]),
            Line.from_points(vertex[0], vertex[3]),
            Line.from_points(vertex[2], vertex[3]),
            Line.from_points(vertex[1], vertex[2]),
            Line.from_points(vertex[0], vertex[4]),
            Line.from_points(vertex[1], vertex[5]),
            Line.from_points(vertex[2], vertex[6]),
            Line.from_points(vertex[3], vertex[7]),
            Line.from_points(vertex[4], vertex[5]),
            Line.from_points(vertex[4], vertex[7]),
            Line.from_points(vertex[6], vertex[7]),
            Line.from_points(vertex[5], vertex[6]),
        ]
        # 第一ボクセル境界
        first_voxel_planes = [
            Plane(point=[0, 1, 0], normal=[0, 1, 0])
        ]
        second_voxel_planes = [
            Plane(point=[0, 0, 1], normal=[0, 0, 1])
        ]
        third_voxel_planes = [
            Plane(point=[1, 0, 0], normal=[1, 0, 0])
        ]
        # 処理時に基準とする軸
        axis = 0

        # 期待値の定義
        voxel_plane_cross_point_assert = [
            Point([0.        , 1.        , 1.33333333]),
            Point([0.        , 1.        , 6.66666667]),
            Point([8.        , 1.        , 1.33333333]),
            Point([8.        , 1.        , 6.66666667])
        ]

        ## 試験実施
        rectangular._get_voxel_plane_cross_point(
            rect_lines,
            first_voxel_planes,
            second_voxel_planes,
            third_voxel_planes,
            axis
        )

        # 結果確認
        assert len(voxel_plane_cross_point_assert) == len(rectangular._all_cross_points)
        for exp_point in voxel_plane_cross_point_assert:
            result = False
            for act_point in rectangular._all_cross_points:
                if Vector.from_points(exp_point, act_point).is_zero(abs_tol=MINIMA):
                    result = True
                    break

            assert result

    def test__get_voxel_plane_cross_point_07(self, rectangular):
        """第一ボクセル境界との接点が0個
        """
        ## 試験用データの定義
        # 直方体の辺全て
        rect_lines = [
            Line(point=Point([ 15083762.2905004863, 4123317.9344834387, -1.0009105899]), direction=Vector([0.   , 0.   , 0.075])),
            Line(point=Point([ 15083762.2905004863, 4123317.9344834387, -1.0009105899]), direction=Vector([0.075, 0.   , 0.   ])),
            Line(point=Point([ 15083762.2905004863, 4123317.9344834387, -0.9259105899]), direction=Vector([0.075, 0.   , 0.   ])),
            Line(point=Point([ 15083762.3655004855, 4123317.9344834387, -1.0009105899]), direction=Vector([0.   , 0.   , 0.075])),
            Line(point=Point([ 15083762.2905004863, 4123317.8594834385, -1.0009105899]), direction=Vector([0.   , 0.   , 0.075])),
            Line(point=Point([ 15083762.2905004863, 4123317.8594834385, -1.0009105899]), direction=Vector([0.075, 0.   , 0.   ])),
            Line(point=Point([ 15083762.3655004855, 4123317.8594834385, -1.0009105899]), direction=Vector([0.   , 0.   , 0.075])),
            Line(point=Point([ 15083762.2905004863, 4123317.8594834385, -0.9259105899]), direction=Vector([0.075, 0.   , 0.   ])),
            Line(point=Point([ 15083762.2905004863, 4123317.9344834387, -1.0009105899]), direction=Vector([ 0.   , -0.075,  0.   ])),
            Line(point=Point([ 15083762.2905004863, 4123317.9344834387, -0.9259105899]), direction=Vector([ 0.   , -0.075,  0.   ])),
            Line(point=Point([ 15083762.3655004855, 4123317.9344834387, -1.0009105899]), direction=Vector([ 0.   , -0.075,  0.   ])),
            Line(point=Point([ 15083762.3655004855, 4123317.9344834387, -0.9259105899]), direction=Vector([ 0.   , -0.075,  0.   ])),
        ]
        # 第一ボクセル境界
        first_voxel_planes = [
            Plane(point=[0, 4123317.8594767987, 0], normal=[0, 1, 0])
        ]
        second_voxel_planes = [
            Plane(point=[0, 0, 0], normal=[1, 0, 0])
        ]
        third_voxel_planes = [
            Plane(point=[0, 0, 1], normal=[0, 0, 1])
        ]
        # 処理時に基準とする軸
        axis = 1

        ## 試験実施
        rectangular._get_voxel_plane_cross_point(
            rect_lines,
            first_voxel_planes,
            second_voxel_planes,
            third_voxel_planes,
            axis
        )

        assert len(rectangular._all_cross_points) == 0


    def test_calc_spatial_ids_01(self):
        """空間IDを取得
        + 試験詳細
          - 入力条件
            - 正常処理を行う
          - 確認内容
            - 直方体の空間IDが取得できること
            - 空間IDの要素が重複していないこと
        """
        ## 試験用データの定義
        # 始点, 終点
        start_spatial_id = '25/29798223/13211885/25/1'
        end_spatial_id = '25/29798223/13211885/25/2'
        start_point = get_point_on_spatial_id(
            start_spatial_id, Point_Option.CENTER, PROJECT_CRS)[0]
        end_point = get_point_on_spatial_id(
            end_spatial_id, Point_Option.CENTER, PROJECT_CRS)[0]
        start_orth_point = \
            Point([start_point.x, start_point.y, start_point.alt])
        end_orth_point = \
            Point([end_point.x, end_point.y, end_point.alt])
        # Webメルカトル係数
        factor = 1 / math.cos(math.radians(start_point.lat))
        # 直交座標のZ成分にWebメルカトルを乗算
        start_orth_point[2] *= factor
        end_orth_point[2] *= factor
        # 半径
        radius = 2.2
        # 精度レベル
        v_zoom = 25
        h_zoom = 25

        # オブジェクト初期化
        rectangular = Rectangular(
            start_orth_point,
            end_orth_point,
            radius,
            v_zoom,
            h_zoom,
            factor
        )

        ## 期待値の定義
        # 直方体の空間ID
        all_spatial_ids_assert = {
            '25/29798221/13211883/25/1',
            '25/29798221/13211883/25/2',
            '25/29798221/13211884/25/1',
            '25/29798221/13211884/25/2',
            '25/29798221/13211885/25/1',
            '25/29798221/13211885/25/2',
            '25/29798221/13211886/25/1',
            '25/29798221/13211886/25/2',
            '25/29798221/13211887/25/1',
            '25/29798221/13211887/25/2',
            '25/29798222/13211883/25/1',
            '25/29798222/13211883/25/2',
            '25/29798222/13211884/25/1',
            '25/29798222/13211884/25/2',
            '25/29798222/13211885/25/1',
            '25/29798222/13211885/25/2',
            '25/29798222/13211886/25/1',
            '25/29798222/13211886/25/2',
            '25/29798222/13211887/25/1',
            '25/29798222/13211887/25/2',
            '25/29798223/13211883/25/1',
            '25/29798223/13211883/25/2',
            '25/29798223/13211884/25/1',
            '25/29798223/13211884/25/2',
            '25/29798223/13211885/25/1',
            '25/29798223/13211885/25/2',
            '25/29798223/13211886/25/1',
            '25/29798223/13211886/25/2',
            '25/29798223/13211887/25/1',
            '25/29798223/13211887/25/2',
            '25/29798224/13211883/25/1',
            '25/29798224/13211883/25/2',
            '25/29798224/13211884/25/1',
            '25/29798224/13211884/25/2',
            '25/29798224/13211885/25/1',
            '25/29798224/13211885/25/2',
            '25/29798224/13211886/25/1',
            '25/29798224/13211886/25/2',
            '25/29798224/13211887/25/1',
            '25/29798224/13211887/25/2',
            '25/29798225/13211883/25/1',
            '25/29798225/13211883/25/2',
            '25/29798225/13211884/25/1',
            '25/29798225/13211884/25/2',
            '25/29798225/13211885/25/1',
            '25/29798225/13211885/25/2',
            '25/29798225/13211886/25/1',
            '25/29798225/13211886/25/2',
            '25/29798225/13211887/25/1',
            '25/29798225/13211887/25/2'
        }

        ## 試験実施
        rectangular.calc_spatial_ids()

        # 戻り値確認
        assert len(rectangular._all_spatial_ids) == \
            len(set(rectangular._all_spatial_ids))
        assert rectangular._all_spatial_ids == all_spatial_ids_assert


    def test_get_spatial_ids_01(self, rectangular):
        """空間IDを返却
        + 試験詳細
          - 確認内容
            - 想定した空間IDが返却されること
        """
        ## 試験用データの定義
        # 空間ID
        rectangular._all_spatial_ids = {
            '10/512/511/10/0',
            '10/512/512/10/0'
        }

        ## 期待値の定義
        # 空間ID
        all_spatial_ids_assert = {
            '10/512/511/10/0',
            '10/512/512/10/0'
        }

        ## 試験実施
        result = rectangular.get_spatial_ids()

        assert result == all_spatial_ids_assert

    def test__get_voxel_plane_cross_point_01(self, rectangular):
        """境界面との交点取得
        """
        ## 試験用データの定義
        # 直方体の辺全て
        vertex = [
            Point([0, 0, 0]),
            Point([3, 4, 0]),
            Point([3, 4, 5]),
            Point([0, 0, 5]),
            Point([8, -6, 0]),
            Point([11, -2, 0]),
            Point([11, -2, 5]),
            Point([8, -6, 5]),
        ]
        rect_lines = [
            Line.from_points(vertex[0], vertex[1]),
            Line.from_points(vertex[0], vertex[3]),
            Line.from_points(vertex[2], vertex[3]),
            Line.from_points(vertex[1], vertex[2]),
            Line.from_points(vertex[0], vertex[4]),
            Line.from_points(vertex[1], vertex[5]),
            Line.from_points(vertex[2], vertex[6]),
            Line.from_points(vertex[3], vertex[7]),
            Line.from_points(vertex[4], vertex[5]),
            Line.from_points(vertex[4], vertex[7]),
            Line.from_points(vertex[6], vertex[7]),
            Line.from_points(vertex[5], vertex[6]),
        ]
        # 第一ボクセル境界
        first_voxel_planes = [
            Plane(point=[1, 0, 0], normal=[1, 0, 0])
        ]
        second_voxel_planes = [
            Plane(point=[0, 1, 0], normal=[0, 1, 0])
        ]
        third_voxel_planes = [
            Plane(point=[0, 0, 1], normal=[0, 0, 1])
        ]
        # 処理時に基準とする軸
        axis = 2

        ## 期待値の定義
        voxel_plane_cross_point_assert = [
            Point([1., 1.33333333, 0.]),
            Point([1., 1.33333333, 5.]),
            Point([ 1., -0.75,  0.]),
            Point([ 1., -0.75,  5.]),
            Point([1., 1., 0.]),
            Point([1., 1. , 0.8]),
            Point([1., 1. , 4.2]),
            Point([1., 1., 5.]),
            Point([1., 1., 1.]),
            Point([1., 1.33333333, 1.]),
            Point([1., 0.91666667, 1.]),
            Point([ 1., -0.33333333,1.]),
            Point([ 1., -0.75,  1.  ]),
            Point([1., 0.95833333, 0.9]),
            Point([1., 0.33333333, 2.6])
        ]

        ## 試験実施
        rectangular._get_voxel_plane_cross_point(
            rect_lines,
            first_voxel_planes,
            second_voxel_planes,
            third_voxel_planes,
            axis
        )

        assert len(voxel_plane_cross_point_assert) == len(rectangular._all_cross_points)


    def test__get_voxel_plane_cross_point_02(self, rectangular):
        """第一ボクセル境界との接点が2個以下
        """
        ## 試験用データの定義
        # 直方体の辺全て
        vertex = [
            Point([0, 0, 0]),
            Point([3, 4, 0]),
            Point([3, 4, 5]),
            Point([0, 0, 5]),
            Point([8, -6, 0]),
            Point([11, -2, 0]),
            Point([11, -2, 5]),
            Point([8, -6, 5]),
        ]
        rect_lines = [
            Line.from_points(vertex[0], vertex[1]),
            Line.from_points(vertex[0], vertex[3]),
            Line.from_points(vertex[2], vertex[3]),
            Line.from_points(vertex[1], vertex[2]),
            Line.from_points(vertex[0], vertex[4]),
            Line.from_points(vertex[1], vertex[5]),
            Line.from_points(vertex[2], vertex[6]),
            Line.from_points(vertex[3], vertex[7]),
            Line.from_points(vertex[4], vertex[5]),
            Line.from_points(vertex[4], vertex[7]),
            Line.from_points(vertex[6], vertex[7]),
            Line.from_points(vertex[5], vertex[6]),
        ]
        # 第一ボクセル境界
        first_voxel_planes = [
            Plane(point=[0, 0, 0], normal=[1, 0, 0])
        ]
        second_voxel_planes = [
            Plane(point=[0, 1, 0], normal=[0, 1, 0])
        ]
        third_voxel_planes = [
            Plane(point=[0, 0, 1], normal=[0, 0, 1])
        ]
        # 処理時に基準とする軸
        axis = 2

        ## 試験実施
        rectangular._get_voxel_plane_cross_point(
            rect_lines,
            first_voxel_planes,
            second_voxel_planes,
            third_voxel_planes,
            axis
        )

        assert len(rectangular._all_cross_points) == 0


    def test__calc_cross_spatial_ids_01(self, rectangular):
        """交点から空間IDを取得
        + 試験詳細
          - 確認内容
            - 想定した空間IDが設定されること
        """
        ## 試験用データの定義
        # 交点配列
        rectangular._all_cross_points  = [
            Point([1., 1.33333333, 0.]),
            Point([1., 1.33333333, 5.]),
            Point([ 1., -0.75,  0.]),
            Point([ 1., -0.75,  5.]),
            Point([1., 1., 0.]),
            Point([1., 1. , 0.8]),
            Point([1., 1. , 4.2]),
            Point([1., 1., 5.]),
            Point([1., 1., 1.]),
            Point([1., 1.33333333, 1.]),
            Point([1., 0.91666667, 1.]),
            Point([ 1., -0.33333333,1.]),
            Point([ 1., -0.75,  1.  ]),
            Point([1., 0.95833333, 0.9]),
            Point([1., 0.33333333, 2.6])
        ]

        ## 期待値の定義
        # 空間ID
        all_spatial_ids_assert = {
            '10/512/511/10/0',
            '10/512/512/10/0'
        }

        ## 試験実施
        rectangular._calc_cross_spatial_ids()

        assert rectangular._all_spatial_ids == all_spatial_ids_assert


    def test_all_spatial_ids_assert_01(self, rectangular):
        """空間IDを返却
        + 試験詳細
          - 確認内容
            - 想定した空間IDが返却されること
        """
        ## 試験用データの定義
        # 空間ID
        rectangular._all_spatial_ids = {
            '10/512/511/10/0',
            '10/512/512/10/0'
        }

        ## 期待値の定義
        # 空間ID
        all_spatial_ids_assert = {
            '10/512/511/10/0',
            '10/512/512/10/0'
        }

        ## 試験実施
        result = rectangular.get_spatial_ids()

        assert result == all_spatial_ids_assert


class TestCapsule:
    @pytest.fixture()
    def sphere(self):

        ## 試験用データの定義
        # 中心点
        center_spatial_id = '25/29798223/13211885/25/1'
        center_point = get_point_on_spatial_id(
            center_spatial_id, Point_Option.CENTER, PROJECT_CRS)[0]
        center_orth_point = \
            Point([center_point.x, center_point.y, center_point.alt])
        # Webメルカトル係数
        factor = 1 / math.cos(math.radians(center_point.lat))
        # 直交座標のZ成分にWebメルカトルを乗算
        center_orth_point[2] *= factor
        # 半径
        radius = 2.2
        # 精度レベル
        v_zoom = 10
        h_zoom = 10
        # カプセル判定
        is_capsule = True
        # 衝突判定オプション
        is_precision = True

        return Capsule(
            center_orth_point,
            center_orth_point,
            radius,
            v_zoom,
            h_zoom,
            is_capsule,
            is_precision,
            factor
        )

    @pytest.fixture()
    def cylinder(self):

        ## 試験用データの定義
        # 始点, 終点
        start_spatial_id = '25/29798223/13211885/25/1'
        end_spatial_id = '25/29798223/13211885/25/4'
        start_point = get_point_on_spatial_id(
            start_spatial_id, Point_Option.CENTER, PROJECT_CRS)[0]
        end_point = get_point_on_spatial_id(
            end_spatial_id, Point_Option.CENTER, PROJECT_CRS)[0]
        start_orth_point = \
            Point([start_point.x, start_point.y, start_point.alt])
        end_orth_point = \
            Point([end_point.x, end_point.y, end_point.alt])
        # Webメルカトル
        factor = 1 / math.cos(math.radians(start_point.lat))
        # 直交座標のZ成分にWebメルカトルを乗算
        start_orth_point[2] *= factor
        end_orth_point[2] *= factor
        # 半径
        radius = 2.2
        # 精度レベル
        v_zoom = 10
        h_zoom = 10
        # カプセル判定
        is_capsule = False
        # 衝突判定オプション
        is_precision = True

        return Capsule(
            start_orth_point,
            end_orth_point,
            radius,
            v_zoom,
            h_zoom,
            is_capsule,
            is_precision,
            factor
        )

    @pytest.fixture()
    def capsule(self):

        ## 試験用データの定義
        # 始点, 終点
        start_spatial_id = '25/29798223/13211885/25/1'
        end_spatial_id = '25/29798223/13211885/25/4'
        start_point = get_point_on_spatial_id(
            start_spatial_id, Point_Option.CENTER, PROJECT_CRS)[0]
        end_point = get_point_on_spatial_id(
            end_spatial_id, Point_Option.CENTER, PROJECT_CRS)[0]
        start_orth_point = \
            Point([start_point.x, start_point.y, start_point.alt])
        end_orth_point = \
            Point([end_point.x, end_point.y, end_point.alt])
        # Webメルカトル
        factor = 1 / math.cos(math.radians(start_point.lat))
        # 直交座標のZ成分にWebメルカトルを乗算
        start_orth_point[2] *= factor
        end_orth_point[2] *= factor
        # 半径
        radius = 2.2
        # 精度レベル
        v_zoom = 10
        h_zoom = 10
        # カプセル判定
        is_capsule = True
        # 衝突判定オプション
        is_precision = True

        return Capsule(
            start_orth_point,
            end_orth_point,
            radius,
            v_zoom,
            h_zoom,
            is_capsule,
            is_precision,
            factor
        )

    def test__init_axis_01(self, sphere):
        """球を構成する軸・ベクトルを初期化
        """
        # 軸ベクトル
        axis_vector = Vector.from_points(
            sphere._start_point, sphere._end_point
        )

        ## 期待値の定義
        # 図形の高さ
        height_assert = axis_vector.norm() + 2 * sphere._radius * sphere._factor
        # 軸の半径長さ直交ベクトル
        radius_orth_vector_assert = Vector([0,  sphere._radius * sphere._factor, 0])
        # 軸方向の単位ベクトル
        unit_axis_vector_assert = Vector([1, 0, 0])
        # 円との接点
        radius_axis_vector = sphere._radius * sphere._factor * unit_axis_vector_assert
        contact_point_assert = \
            (radius_orth_vector_assert - radius_axis_vector) \
                + sphere._start_point
        # 軸ベクトルと直交ベクトルの法線ベクトル(大きさは機体半径)
        radius_normal_vector_assert = \
            sphere._radius * sphere._factor \
                * radius_orth_vector_assert.cross(unit_axis_vector_assert).unit()

        ## 試験実施
        sphere._init_axis()

        # 戻り値確認
        assert height_assert == sphere._height
        assert (radius_orth_vector_assert - sphere._radius_orth_vector).is_zero(abs_tol=MINIMA)
        assert (unit_axis_vector_assert - sphere._unit_axis_vector).is_zero(abs_tol=MINIMA)
        assert (contact_point_assert - sphere._contact_point).is_zero(abs_tol=MINIMA)
        assert (radius_normal_vector_assert - sphere._radius_normal_vector).is_zero(abs_tol=MINIMA)

    def test__init_axis_02(self, cylinder):
        """円柱を構成する軸・ベクトルを初期化
        + 試験詳細
          - 入力条件
            - 円柱用の入力
          - 確認内容
            - 想定した軸・ベクトルが設定されること
        """
        # 軸ベクトル
        axis_vector = Vector.from_points(
            cylinder._start_point, cylinder._end_point
        )

        ## 期待値の定義
        # 図形の高さ
        height_assert = axis_vector.norm()
        # 軸の半径長さ直交ベクトル
        radius_orth_vector_assert = Vector([0, cylinder._radius * cylinder._factor, 0])
        # 軸方向の単位ベクトル
        unit_axis_vector_assert = Vector([0, 0, 1])
        # 円との接点
        radius_axis_vector = Vector([0, 0, 0])
        contact_point_assert = \
            (radius_orth_vector_assert - radius_axis_vector) \
                + cylinder._start_point
        # 軸ベクトルと直交ベクトルの法線ベクトル(大きさは機体半径)
        radius_normal_vector_assert = \
            cylinder._radius * cylinder._factor \
                * radius_orth_vector_assert.cross(unit_axis_vector_assert).unit()

        ## 試験実施
        cylinder._init_axis()

        # 戻り値確認
        assert height_assert == cylinder._height
        assert (radius_orth_vector_assert - cylinder._radius_orth_vector).is_zero(abs_tol=MINIMA)
        assert (unit_axis_vector_assert - cylinder._unit_axis_vector).is_zero(abs_tol=MINIMA)
        assert (contact_point_assert - cylinder._contact_point).is_zero(abs_tol=MINIMA)
        assert (radius_normal_vector_assert - cylinder._radius_normal_vector).is_zero(abs_tol=MINIMA)

    def test__init_axis_03(self, capsule):
        """カプセルを構成する軸・ベクトルを初期化(軸のY成分0)
        + 試験詳細
          - 入力条件
            - カプセル用の入力
            - 始点・終点のY成分が同じ座標が入力に指定されること
          - 確認内容
            - 想定した軸・ベクトルが設定されること
        """
        # 軸ベクトル
        axis_vector = Vector.from_points(
            capsule._start_point, capsule._end_point
        )

        ## 期待値の定義
        # 図形の高さ
        height_assert = axis_vector.norm() + 2 * capsule._radius * capsule._factor
        # 軸の半径長さ直交ベクトル
        radius_orth_vector_assert = Vector([0, 2.70877268, 0])
        # 軸方向の単位ベクトル
        unit_axis_vector_assert = axis_vector.unit()
        # 円との接点
        radius_axis_vector = capsule._radius * capsule._factor * unit_axis_vector_assert
        contact_point_assert = \
            (radius_orth_vector_assert - radius_axis_vector) \
                + capsule._start_point
        # 軸ベクトルと直交ベクトルの法線ベクトル(大きさは機体半径)
        radius_normal_vector_assert = \
             capsule._radius * capsule._factor \
                * radius_orth_vector_assert.cross(unit_axis_vector_assert).unit()

        ## 試験実施
        capsule._init_axis()

        # 戻り値確認
        assert height_assert == capsule._height
        assert (radius_orth_vector_assert - capsule._radius_orth_vector).is_zero(abs_tol=MINIMA)
        assert (unit_axis_vector_assert - capsule._unit_axis_vector).is_zero(abs_tol=MINIMA)
        assert (contact_point_assert - capsule._contact_point).is_zero(abs_tol=MINIMA)
        assert (radius_normal_vector_assert - capsule._radius_normal_vector).is_zero(abs_tol=MINIMA)

    def test__init_axis_04(self):
        """カプセルの軸・ベクトル決定(軸のZ成分0)
        + 試験詳細
          - 入力条件
            - 始点・終点のZ成分が同じ座標が入力に指定されること
          - 確認内容
            - 想定した軸・ベクトルが初期化されること
        """
        ## 試験用データの定義
        # 始点, 終点
        start_spatial_id = '25/29798223/13211885/25/1'
        end_spatial_id = '25/29798224/13211886/25/1'
        start_point = get_point_on_spatial_id(
            start_spatial_id, Point_Option.CENTER, PROJECT_CRS)[0]
        end_point = get_point_on_spatial_id(
            end_spatial_id, Point_Option.CENTER, PROJECT_CRS)[0]
        start_orth_point = \
            Point([start_point.x, start_point.y, start_point.alt])
        end_orth_point = \
            Point([end_point.x, end_point.y, end_point.alt])
        # Webメルカトル
        factor = 1 / math.cos(math.radians(start_point.lat))
        # 直交座標のZ成分にWebメルカトルを乗算
        start_orth_point[2] *= factor
        end_orth_point[2] *= factor
        # 半径
        radius = 2.2
        # 精度レベル
        v_zoom = 10
        h_zoom = 10
        # カプセル判定
        is_capsule = True
        # 衝突判定オプション
        is_precision = True

        # オブジェクト初期化
        capsule = Capsule(
            start_orth_point,
            end_orth_point,
            radius,
            v_zoom,
            h_zoom,
            is_capsule,
            is_precision,
            factor
        )

        # 軸ベクトル
        axis_vector = Vector.from_points(
            capsule._start_point, capsule._end_point
        )

        ## 期待値の定義
        # 図形の高さ
        height_assert = axis_vector.norm() + 2 * radius * factor
        # 軸の半径長さ直交ベクトル
        radius_orth_vector_assert = Vector([0, 0, radius * factor])
        # 軸方向の単位ベクトル
        unit_axis_vector_assert = axis_vector.unit()
        # 円との接点
        radius_axis_vector = radius * factor * unit_axis_vector_assert
        contact_point_assert = \
            (radius_orth_vector_assert - radius_axis_vector) \
                + capsule._start_point
        # 軸ベクトルと直交ベクトルの法線ベクトル(大きさは機体半径)
        radius_normal_vector_assert = \
            radius * factor \
                * radius_orth_vector_assert.cross(unit_axis_vector_assert).unit()

        ## 試験実施
        capsule._init_axis()

        # 戻り値確認
        assert height_assert == capsule._height
        assert (radius_orth_vector_assert - capsule._radius_orth_vector).is_zero(abs_tol=MINIMA)
        assert (unit_axis_vector_assert - capsule._unit_axis_vector).is_zero(abs_tol=MINIMA)
        assert (contact_point_assert - capsule._contact_point).is_zero(abs_tol=MINIMA)
        assert (radius_normal_vector_assert - capsule._radius_normal_vector).is_zero(abs_tol=MINIMA)

    def test__init_axis_05(self):
        """カプセルの軸・ベクトル決定(軸のX成分0)
        + 試験詳細
          - 入力条件
            - 始点・終点のX成分が同じ座標が入力に指定されること
          - 確認内容
            - 想定した軸・ベクトルが初期化されること
        """
        ## 試験用データの定義
        # 始点, 終点
        start_spatial_id = '10/29798223/13211885/10/1'
        end_spatial_id = '10/29798223/13211886/10/2'
        start_point = get_point_on_spatial_id(
            start_spatial_id, Point_Option.CENTER, PROJECT_CRS)[0]
        end_point = get_point_on_spatial_id(
            end_spatial_id, Point_Option.CENTER, PROJECT_CRS)[0]
        start_orth_point = \
            Point([start_point.x, start_point.y, start_point.alt])
        end_orth_point = \
            Point([end_point.x, end_point.y, end_point.alt])
        # Webメルカトル
        factor = 1 / math.cos(math.radians(start_point.lat))
        # 直交座標のZ成分にWebメルカトルを乗算
        start_orth_point[2] *= factor
        end_orth_point[2] *= factor
        # 半径
        radius = 2.2
        # 精度レベル
        v_zoom = 10
        h_zoom = 10
        # カプセル判定
        is_capsule = True
        # 衝突判定オプション
        is_precision = True

        # オブジェクト初期化
        capsule = Capsule(
            start_orth_point,
            end_orth_point,
            radius,
            v_zoom,
            h_zoom,
            is_capsule,
            is_precision,
            factor
        )

        # 軸ベクトル
        axis_vector = Vector.from_points(
            capsule._start_point, capsule._end_point
        )
        ## 期待値の定義
        # 図形の高さ
        height_assert = axis_vector.norm() + 2 * radius * factor
        # 軸の半径長さ直交ベクトル
        radius_orth_vector_assert = Vector([0, radius * factor, 0])
        # 軸方向の単位ベクトル
        unit_axis_vector_assert = axis_vector.unit()
        # 円との接点
        radius_axis_vector = 2.2 * factor* unit_axis_vector_assert

        contact_point_assert = \
            (radius_orth_vector_assert - radius_axis_vector) \
                + capsule._start_point
        # 軸ベクトルと直交ベクトルの法線ベクトル(大きさは機体半径)
        radius_normal_vector_assert = \
            radius * factor \
                * radius_orth_vector_assert.cross(unit_axis_vector_assert).unit()

        ## 試験実施
        capsule._init_axis()

        # 戻り値確認
        assert height_assert == capsule._height
        assert (radius_orth_vector_assert - capsule._radius_orth_vector).is_zero(abs_tol=MINIMA)
        assert (unit_axis_vector_assert - capsule._unit_axis_vector).is_zero(abs_tol=MINIMA)
        assert (contact_point_assert - capsule._contact_point).is_zero(abs_tol=MINIMA)
        assert (radius_normal_vector_assert - capsule._radius_normal_vector).is_zero(abs_tol=MINIMA)

    def test__approximate_distance_radius_points_01(self, capsule):
        """空間IDを中心とした概算距離内の空間ID取得確認
        + 試験詳細
          - 入力条件
            - 概算距離の中心とする空間ID
          - 確認内容
            - 想定した空間IDが返却されること
        """
        ## 試験用データの定義
        # 概算距離の中心とする空間ID
        spatial_id = "10/5/6/10/7"
        # 概算ボクセル数
        capsule._approximate_voxel_num = 2

        ## 期待値の定義
        approximate_distance_radius_points_assert = {
            '10/3/6/10/7',
            '10/4/5/10/7',
            '10/4/6/10/6',
            '10/4/6/10/7',
            '10/4/6/10/8',
            '10/4/7/10/7',
            '10/5/4/10/7',
            '10/5/5/10/6',
            '10/5/5/10/7',
            '10/5/5/10/8',
            '10/5/6/10/5',
            '10/5/6/10/6',
            '10/5/6/10/7',
            '10/5/6/10/8',
            '10/5/6/10/9',
            '10/5/7/10/6',
            '10/5/7/10/7',
            '10/5/7/10/8',
            '10/5/8/10/7',
            '10/6/5/10/7',
            '10/6/6/10/6',
            '10/6/6/10/7',
            '10/6/6/10/8',
            '10/6/7/10/7',
            '10/7/6/10/7',
        }

        ## 試験実施
        result = capsule._approximate_distance_radius_points(spatial_id)

        # 戻り値確認
        assert approximate_distance_radius_points_assert == result


    def test__calc_valid_spatial_ids_01(self):
        """円柱の有効な空間IDを取得
        + 試験詳細
          - 確認内容
            - 円柱の空間IDが取得できること
        """
        ## 試験用データの定義
        # 始点, 終点
        start_spatial_id = '10/30/60/10/5'
        end_spatial_id = '10/31/62/10/8'
        start_point = get_point_on_spatial_id(
            start_spatial_id, Point_Option.CENTER, PROJECT_CRS)[0]
        end_point = get_point_on_spatial_id(
            end_spatial_id, Point_Option.CENTER, PROJECT_CRS)[0]
        start_orth_point = \
            Point([start_point.x, start_point.y, start_point.alt])
        end_orth_point = \
            Point([end_point.x, end_point.y, end_point.alt])
        # Webメルカトル
        factor = 1 / math.cos(math.radians(start_point.lat))
        # 直交座標のZ成分にWebメルカトルを乗算
        start_orth_point[2] *= factor
        end_orth_point[2] *= factor
        # 半径
        radius = 2.2
        # 精度レベル
        v_zoom = 10
        h_zoom = 10
        # カプセル判定
        is_capsule = False
        # 衝突判定オプション
        is_precision = True

        # オブジェクト初期化
        cylinder = Capsule(
            start_orth_point,
            end_orth_point,
            radius,
            v_zoom,
            h_zoom,
            is_capsule,
            is_precision,
            factor
        )

        ## 期待値の定義
        # 直方体の空間ID
        all_spatial_ids_assert = {
            '10/30/60/10/5',
            '10/30/60/10/6',
            '10/30/61/10/6',
            '10/30/61/10/7',
            '10/31/61/10/6',
            '10/31/61/10/7',
            '10/31/62/10/7',
            '10/31/62/10/8'
        }

        ## 試験実施
        cylinder._init_axis()
        cylinder._calc_rectangular_apex()
        cylinder._calc_cross_spatial_ids()
        cylinder._calc_valid_spatial_ids()

        # 戻り値確認
        assert cylinder._all_spatial_ids == all_spatial_ids_assert

    def test__calc_valid_spatial_ids_02(self):
        """球の有効な空間IDを取得
        + 試験詳細
          - 確認内容
            - 円柱の空間IDが取得できること
        """
        ## 試験用データの定義
        # 始点, 終点
        center_spatial_id = '10/30/60/10/5'
        center_point = get_point_on_spatial_id(
            center_spatial_id, Point_Option.CENTER, PROJECT_CRS)[0]
        center_orth_point = \
            Point([center_point.x, center_point.y, center_point.alt])
        # Webメルカトル係数
        factor = 1 / math.cos(math.radians(center_point.lat))
        # 直交座標のZ成分にWebメルカトルを乗算
        center_orth_point[2] *= factor
        # 半径
        radius = 2.2
        # 精度レベル
        v_zoom = 10
        h_zoom = 10
        # カプセル判定
        is_capsule = False
        # 衝突判定オプション
        is_precision = True

        # オブジェクト初期化
        cylinder = Capsule(
            center_orth_point,
            center_orth_point,
            radius,
            v_zoom,
            h_zoom,
            is_capsule,
            is_precision,
            factor
        )

        ## 期待値の定義
        # 直方体の空間ID
        all_spatial_ids_assert = {
            '10/30/60/10/5',
        }
        include_spatial_ids_assert = {
            '10/30/60/10/5',
        }

        ## 試験実施
        cylinder._init_axis()
        cylinder._calc_rectangular_apex()
        cylinder._calc_cross_spatial_ids()
        cylinder._calc_valid_spatial_ids()

        # 戻り値確認
        assert cylinder._all_spatial_ids == all_spatial_ids_assert
        assert cylinder._include_spatial_ids == include_spatial_ids_assert

    def test__calc_valid_spatial_ids_03(self):
        """円柱の有効な空間IDを取得
        + 試験詳細
          - 入力条件
           - 水平精度が0
          - 確認内容
            - 円柱の空間IDが取得できること
        """
        ## 試験用データの定義
        # 始点, 終点
        start_spatial_id = '10/30/60/10/5'
        end_spatial_id = '10/31/62/10/8'
        start_point = get_point_on_spatial_id(
            start_spatial_id, Point_Option.CENTER, PROJECT_CRS)[0]
        end_point = get_point_on_spatial_id(
            end_spatial_id, Point_Option.CENTER, PROJECT_CRS)[0]
        start_orth_point = \
            Point([start_point.x, start_point.y, start_point.alt])
        end_orth_point = \
            Point([end_point.x, end_point.y, end_point.alt])
        # Webメルカトル
        factor = 1 / math.cos(math.radians(start_point.lat))
        # 直交座標のZ成分にWebメルカトルを乗算
        start_orth_point[2] *= factor
        end_orth_point[2] *= factor
        # 半径
        radius = 2.2
        # 精度レベル
        v_zoom = 0
        h_zoom = 0
        # カプセル判定
        is_capsule = False
        # 衝突判定オプション
        is_precision = True

        # オブジェクト初期化
        cylinder = Capsule(
            start_orth_point,
            end_orth_point,
            radius,
            v_zoom,
            h_zoom,
            is_capsule,
            is_precision,
            factor
        )

        ## 期待値の定義
        # 直方体の空間ID
        all_spatial_ids_assert = {
            '0/0/0/0/0',
        }

        ## 試験実施
        cylinder._init_axis()
        cylinder._calc_rectangular_apex()
        cylinder._calc_cross_spatial_ids()
        cylinder._calc_valid_spatial_ids()

        # 戻り値確認
        assert cylinder._all_spatial_ids == all_spatial_ids_assert

    def test__calc_unit_voxel_vector_01(self, capsule):
        """概算距離用の単位ボクセルベクトルを算出
        + 試験詳細
          - 確認内容
            - 想定した単位ボクセルベクトルが返却されること
        """
        ## 試験用データの定義
        # 空間ID
        spatial_id = "25/29798223/13211885/25/1"

        # 引数の空間IDの頂点座標を取得
        points = get_point_on_spatial_id(
            spatial_id, Point_Option.VERTEX, PROJECT_CRS
        )
        # 空間IDの頂点座標をprint
        # ボクセルの中心1: {139.700249433517 35.690929511400 1.000000000000}
        # ボクセルの中心2: {139.700260162354 35.690929511400 1.000000000000}
        # ボクセルの中心3: {139.700260162354 35.690920797700 1.000000000000}
        # ボクセルの中心4: {139.700249433517 35.690920797700 1.000000000000} min要素
        # ボクセルの中心5: {139.700249433517 35.690929511400 2.000000000000}
        # ボクセルの中心6: {139.700260162354 35.690929511400 2.000000000000} max要素
        # ボクセルの中心7: {139.700260162354 35.690920797700 2.000000000000}
        # ボクセルの中心8: {139.700249433517 35.690920797700 2.000000000000}
        maxX = points[5].x
        minX = points[3].x
        maxY = points[5].y
        minY = points[3].y
        maxZ = points[5].alt
        minZ = points[3].alt

        ## 期待値の定義
        unit_voxel_assert = \
            Vector([maxX - minX, maxY - minY, (maxZ - minZ) * capsule._factor])

        ## 試験実施
        result = capsule._calc_unit_voxel_vector(spatial_id)

        # 戻り値確認
        # 小数点6桁以下を切り捨ているため6桁までで比較
        assert abs(unit_voxel_assert[0] - result[0]) < 1e-6
        assert abs(unit_voxel_assert[1] - result[1]) < 1e-6
        assert abs(unit_voxel_assert[2] - result[2]) < 1e-6

    def test_shave_sphere_01(self):
        """球の中心から指定した方向の空間IDを削除する
        + 試験詳細
          - 入力条件
            - 球のデータ
            - 球の中心に対して削除を行う方向をX+方向のみの成分を持つように指定する
            - 削除する方向は中心から半径よりも離れている値とする
          - 確認内容
            - 球の中心のX成分より大きい空間IDが取得出来ないこと
              (処理によって削られることを確認)
        """
        ## 試験用データの定義
        # 中点
        center_spatial_id = '25/29798221/13211884/25/1'
        center_point = get_point_on_spatial_id(
            center_spatial_id, Point_Option.CENTER, PROJECT_CRS)[0]
        center_orth_point = \
            Point([center_point.x, center_point.y, center_point.alt])
        # Webメルカトル係数
        factor = 1 / math.cos(math.radians(center_point.lat))
        # 直交座標のZ成分にWebメルカトルを乗算
        center_orth_point[2] *= factor
        # 中心のX成分
        center_x = 29798221
        # 半径
        radius = 2.0
        # 精度
        h_zoom = 25
        v_zoom = 25
        # カプセル判定
        is_capsule = False
        # 衝突判定実施オプション
        is_precision = True

        # オブジェクト初期化
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
        # 球の空間ID取得
        sphere.calc_spatial_ids()

        # 削除方向
        shave_spatial_id = '25/29798222/13211884/25/1'
        shave_point = get_point_on_spatial_id(
            shave_spatial_id, Point_Option.CENTER, PROJECT_CRS)[0]
        shave_orth_point = \
            Point([shave_point.x, shave_point.y, shave_point.alt])
        shave_orth_point[2] *= factor

        # 試験実行
        sphere.shave_sphere(shave_orth_point)
        result_spatial_ids = sphere.get_spatial_ids()

        # 結果確認
        for spatial_id in result_spatial_ids:
            result_x = int(spatial_id.split("/")[1])
            assert result_x <= center_x


    def test_calc_spatial_ids_01(self):
        """空間IDを取得全体実行
        + 試験詳細
          - 入力条件
            - 円柱のデータ
            - 衝突判定あり
          - 確認内容
            - 円柱の空間IDが取得できること
            - 空間IDの要素が重複していないこと
        """
        ## 試験用データの定義
        # 始点, 終点
        start_spatial_id = '25/29798223/13211885/25/1'
        end_spatial_id = '25/29798223/13211888/25/4'
        start_point = get_point_on_spatial_id(
            start_spatial_id, Point_Option.CENTER, PROJECT_CRS)[0]
        end_point = get_point_on_spatial_id(
            end_spatial_id, Point_Option.CENTER, PROJECT_CRS)[0]
        start_orth_point = \
            Point([start_point.x, start_point.y, start_point.alt])
        end_orth_point = \
            Point([end_point.x, end_point.y, end_point.alt])
        # Webメルカトル
        factor = 1 / math.cos(math.radians(start_point.lat))
        # 直交座標のZ成分にWebメルカトルを乗算
        start_orth_point[2] *= factor
        end_orth_point[2] *= factor
        # 半径
        radius = 2.2
        # 精度レベル
        v_zoom = 25
        h_zoom = 25
        # カプセル判定
        is_capsule = False
        # 衝突判定実施オプション
        is_precision = True

        # オブジェクト初期化
        cylinder = Capsule(
            start_orth_point,
            end_orth_point,
            radius,
            v_zoom,
            h_zoom,
            is_capsule,
            is_precision,
            factor
        )

        ## 期待値の定義
        # 直方体の空間ID
        include_spatial_ids_assert = {
            '25/29798221/13211884/25/1',
            '25/29798221/13211884/25/2',
            '25/29798221/13211884/25/3',
            '25/29798221/13211885/25/1',
            '25/29798221/13211885/25/2',
            '25/29798221/13211885/25/3',
            '25/29798221/13211885/25/4',
            '25/29798221/13211886/25/0',
            '25/29798221/13211886/25/1',
            '25/29798221/13211886/25/2',
            '25/29798221/13211886/25/3',
            '25/29798221/13211886/25/4',
            '25/29798221/13211886/25/5',
            '25/29798221/13211887/25/0',
            '25/29798221/13211887/25/1',
            '25/29798221/13211887/25/2',
            '25/29798221/13211887/25/3',
            '25/29798221/13211887/25/4',
            '25/29798221/13211887/25/5',
            '25/29798221/13211888/25/1',
            '25/29798221/13211888/25/2',
            '25/29798221/13211888/25/3',
            '25/29798221/13211888/25/4',
            '25/29798221/13211889/25/2',
            '25/29798221/13211889/25/3',
            '25/29798221/13211889/25/4',
            '25/29798222/13211883/25/2',
            '25/29798222/13211883/25/3',
            '25/29798222/13211884/25/1',
            '25/29798222/13211884/25/2',
            '25/29798222/13211884/25/3',
            '25/29798222/13211884/25/4',
            '25/29798222/13211885/25/1',
            '25/29798222/13211885/25/2',
            '25/29798222/13211885/25/3',
            '25/29798222/13211885/25/4',
            '25/29798222/13211885/25/5',
            '25/29798222/13211886/25/0',
            '25/29798222/13211886/25/1',
            '25/29798222/13211886/25/2',
            '25/29798222/13211886/25/3',
            '25/29798222/13211886/25/4',
            '25/29798222/13211886/25/5',
            '25/29798222/13211887/25/0',
            '25/29798222/13211887/25/1',
            '25/29798222/13211887/25/2',
            '25/29798222/13211887/25/3',
            '25/29798222/13211887/25/4',
            '25/29798222/13211887/25/5',
            '25/29798222/13211888/25/0',
            '25/29798222/13211888/25/1',
            '25/29798222/13211888/25/2',
            '25/29798222/13211888/25/3',
            '25/29798222/13211888/25/4',
            '25/29798222/13211889/25/1',
            '25/29798222/13211889/25/2',
            '25/29798222/13211889/25/3',
            '25/29798222/13211889/25/4',
            '25/29798222/13211890/25/2',
            '25/29798222/13211890/25/3',
            '25/29798223/13211883/25/2',
            '25/29798223/13211883/25/3',
            '25/29798223/13211884/25/1',
            '25/29798223/13211884/25/2',
            '25/29798223/13211884/25/3',
            '25/29798223/13211884/25/4',
            '25/29798223/13211885/25/1',
            '25/29798223/13211885/25/2',
            '25/29798223/13211885/25/3',
            '25/29798223/13211885/25/4',
            '25/29798223/13211885/25/5',
            '25/29798223/13211886/25/0',
            '25/29798223/13211886/25/1',
            '25/29798223/13211886/25/2',
            '25/29798223/13211886/25/3',
            '25/29798223/13211886/25/4',
            '25/29798223/13211886/25/5',
            '25/29798223/13211886/25/6',
            '25/29798223/13211887/25/-1',
            '25/29798223/13211887/25/0',
            '25/29798223/13211887/25/1',
            '25/29798223/13211887/25/2',
            '25/29798223/13211887/25/3',
            '25/29798223/13211887/25/4',
            '25/29798223/13211887/25/5',
            '25/29798223/13211888/25/0',
            '25/29798223/13211888/25/1',
            '25/29798223/13211888/25/2',
            '25/29798223/13211888/25/3',
            '25/29798223/13211888/25/4',
            '25/29798223/13211889/25/1',
            '25/29798223/13211889/25/2',
            '25/29798223/13211889/25/3',
            '25/29798223/13211889/25/4',
            '25/29798223/13211890/25/2',
            '25/29798223/13211890/25/3',
            '25/29798224/13211883/25/2',
            '25/29798224/13211883/25/3',
            '25/29798224/13211884/25/1',
            '25/29798224/13211884/25/2',
            '25/29798224/13211884/25/3',
            '25/29798224/13211884/25/4',
            '25/29798224/13211885/25/1',
            '25/29798224/13211885/25/2',
            '25/29798224/13211885/25/3',
            '25/29798224/13211885/25/4',
            '25/29798224/13211885/25/5',
            '25/29798224/13211886/25/0',
            '25/29798224/13211886/25/1',
            '25/29798224/13211886/25/2',
            '25/29798224/13211886/25/3',
            '25/29798224/13211886/25/4',
            '25/29798224/13211886/25/5',
            '25/29798224/13211887/25/0',
            '25/29798224/13211887/25/1',
            '25/29798224/13211887/25/2',
            '25/29798224/13211887/25/3',
            '25/29798224/13211887/25/4',
            '25/29798224/13211887/25/5',
            '25/29798224/13211888/25/0',
            '25/29798224/13211888/25/1',
            '25/29798224/13211888/25/2',
            '25/29798224/13211888/25/3',
            '25/29798224/13211888/25/4',
            '25/29798224/13211889/25/1',
            '25/29798224/13211889/25/2',
            '25/29798224/13211889/25/3',
            '25/29798224/13211889/25/4',
            '25/29798224/13211890/25/2',
            '25/29798224/13211890/25/3',
            '25/29798225/13211884/25/1',
            '25/29798225/13211884/25/2',
            '25/29798225/13211884/25/3',
            '25/29798225/13211885/25/1',
            '25/29798225/13211885/25/2',
            '25/29798225/13211885/25/3',
            '25/29798225/13211885/25/4',
            '25/29798225/13211886/25/0',
            '25/29798225/13211886/25/1',
            '25/29798225/13211886/25/2',
            '25/29798225/13211886/25/3',
            '25/29798225/13211886/25/4',
            '25/29798225/13211886/25/5',
            '25/29798225/13211887/25/0',
            '25/29798225/13211887/25/1',
            '25/29798225/13211887/25/2',
            '25/29798225/13211887/25/3',
            '25/29798225/13211887/25/4',
            '25/29798225/13211887/25/5',
            '25/29798225/13211888/25/1',
            '25/29798225/13211888/25/2',
            '25/29798225/13211888/25/3',
            '25/29798225/13211888/25/4',
            '25/29798225/13211889/25/2',
            '25/29798225/13211889/25/3',
            '25/29798225/13211889/25/4',
        }

        ## 試験実施
        cylinder.calc_spatial_ids()

        # 戻り値確認
        # 内部空間IDが期待値と一致すること
        assert cylinder._include_spatial_ids == include_spatial_ids_assert
        assert len(cylinder._include_spatial_ids) == \
            len(set(cylinder._include_spatial_ids))


    def test_calc_spatial_ids_02(self):
        """空間IDを取得全体実行
        + 試験詳細
          - 入力条件
            - 円柱のデータ
            - 衝突判定なし
          - 確認内容
            - 円柱の空間IDが取得できること(空でないこと)
            - 空間IDの要素が重複していないこと
            - 内部空間IDと全空間IDが一致すること
        """
        ## 試験用データの定義
        # 始点, 終点
        start_spatial_id = '25/29798223/13211885/25/1'
        end_spatial_id = '25/29798223/13211888/25/4'
        start_point = get_point_on_spatial_id(
            start_spatial_id, Point_Option.CENTER, PROJECT_CRS)[0]
        end_point = get_point_on_spatial_id(
            end_spatial_id, Point_Option.CENTER, PROJECT_CRS)[0]
        start_orth_point = \
            Point([start_point.x, start_point.y, start_point.alt])
        end_orth_point = \
            Point([end_point.x, end_point.y, end_point.alt])
        # Webメルカトル
        factor = 1 / math.cos(math.radians(start_point.lat))
        # 直交座標のZ成分にWebメルカトルを乗算
        start_orth_point[2] *= factor
        end_orth_point[2] *= factor
        # 半径
        radius = 2.2
        # 精度レベル
        v_zoom = 25
        h_zoom = 25
        # カプセル判定
        is_capsule = False
        # 衝突判定実施オプション
        is_precision = False

        # オブジェクト初期化
        cylinder = Capsule(
            start_orth_point,
            end_orth_point,
            radius,
            v_zoom,
            h_zoom,
            is_capsule,
            is_precision,
            factor
        )

        ## 試験実施
        cylinder.calc_spatial_ids()

        # 戻り値確認
        assert cylinder._all_spatial_ids == cylinder._include_spatial_ids
        assert len(cylinder._all_spatial_ids) == \
            len(set(cylinder._all_spatial_ids))
        assert len(cylinder._all_spatial_ids) != 0


    def test_get_spatial_ids_01(self, capsule):
        """空間IDを返却
        + 試験詳細
          - 確認内容
            - 想定した空間IDが返却されること
        """
        ## 試験用データの定義
        # 空間ID
        capsule._all_spatial_ids = {
            '10/512/511/10/0',
            '10/512/512/10/0'
        }
        # 内部空間ID
        capsule._include_spatial_ids = {
            '10/512/511/10/0',
        }

        ## 期待値の定義
        # 空間ID
        include_spatial_ids_assert = {
            '10/512/511/10/0',
        }

        ## 試験実施
        result = capsule.get_spatial_ids()

        assert result == include_spatial_ids_assert


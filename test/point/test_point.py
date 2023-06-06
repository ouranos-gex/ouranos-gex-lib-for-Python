from logging import getLogger
from math import pi, floor, atan, sinh, degrees
import math
import random
import re
import time
from SpatialId.operated.shifting_spatial_id import get_shifting_spatial_id
import pytest
from SpatialId import LOGGER_NAME
from SpatialId.common.object import enum
from SpatialId.shape import get_CRS_OBJECT
from SpatialId.shape.point import (
    _get_altitude_on_vertical_index_and_zoom,
    _get_center_point_on_voxel_offset,
    _get_horizontal_tile_id_on_point,
    _get_vertex_on_voxel_offset,
    _get_vertical_tile_id_on_altitude,
    check_zoom,
    convert_point_list_to_projected_point_list,
    convert_projected_point_list_to_point_list,
    f_get_point_on_spatial_id,
    f_get_spatial_ids_on_points, get_point_on_spatial_id)
from SpatialId.shape.point import get_spatial_ids_on_points
from SpatialId.common.object.point import Point, Projected_Point, Vertical_Point
from SpatialId.common.exception import SpatialIdError

spatial_log = getLogger(LOGGER_NAME).getChild(__name__)


@pytest.mark.parametrize(
    ('point_list', 'h_zoom', 'v_zoom', 'crs', 'case', 'excepted'), [
        ([Point(139.753098, 35.685371, 0.0)], 18, 25, 4326, 1, '18/232837/103222/25/0'),  # 基本形(WGS84)
        ([Point(139.753098, 35.685371, 10.0)], 16, 25, 6697, 2, '16/58209/25805/25/10'),  # JGD2011(GRS80)
        ([Point(139.756330747, 35.682134453, 100.0)], 8, 25, 5132, 3, '8/227/100/25/100'),  # JGD2000(ベッセル楕円体)
        ([Point(139.753098, 35.685371, 100.0)], 5, 25, None, 4, '5/28/12/25/100'),  # EPSG指定なし
        ([Point(-21.942400, 64.147209, 100.0)], 9, 25, None, 5, '9/224/136/25/100'),  # 北半球西側
        ([Point(-58.373490, -34.608510, -1300.0)], 14, 25, None, 6, '14/5535/9872/25/-1300'),  # 南半球
        ([Point(-180, 85.05112877, -1300.0)], 24, 25, None, 7, '24/0/0/25/-1300'),  # 地図上の左上
        ([Point(179.99999999, -85.05112877, -1300.0)], 10, 25, None, 8, '10/1023/1023/25/-1300'),  # 地図上の右下
        ([Point(0, 0, -1300.0)], 9, 25, None, 9, '9/256/256/25/-1300'),  # 原点
        ([Point(180, -85.05112877, -1300.0)], 10, 25, None, 8, '10/0/1023/25/-1300'),  # 180は-180と同じ扱い
        # 11より先は要素数で判定
        ([Point(0, 0, -1300.0), Point(180, 85.05112877, 0.0), Point(-180, 85.05112877, 1300.0)],
            1, 1, 4326, 11, 3),  # 複数の座標
        ([], 24, 25, 4326, 12, 0),  # 空のリスト
        ([Point(180, -85.05112877, 1)], 10, 35, None, 8, '10/0/1023/35/1024'),  # 180は-180と同じ扱い

    ])
def test_get_spatial_ids_on_points(
    point_list: list[Point], h_zoom: int, v_zoom: int,
        crs, case, excepted):

    points = []
    spatial_log.info(f'case:{case}')
    spatial_log.info(f'point_list:{point_list},h_zoom:{h_zoom},v_zoom:{v_zoom},crs:{crs},case:{case},excepted:{excepted}')
    if crs:
        points = get_spatial_ids_on_points(point_list, h_zoom, v_zoom, crs)
    else:
        points = get_spatial_ids_on_points(point_list, h_zoom, v_zoom)

    for point_value in points:
        spatial_log.info(f'result:{point_value}')

    if case.__le__(10):
        # 水平方向は外部サイトにて確認、IDの前方一致
        assert points[0].startswith(excepted)
    elif case.__le__(11):
        # 複数個ある場合は個数で判定
        assert excepted.__eq__(len(points))

@pytest.mark.parametrize(
    ('point_list', 'h_zoom', 'crs', 'case', 'excepted'), [
        ([Point(139.753098, 35.685371, 0.0)], 18, 4326, 1, '18/0/232837/103222'),  # 基本形(WGS84)

    ])
def test_f_get_spatial_ids_on_points(
    point_list: list[Point], h_zoom: int, crs, case, excepted):

    points = []
    spatial_log.info(f'case:{case}')
    spatial_log.info(f'point_list:{point_list},h_zoom:{h_zoom},crs:{crs},case:{case},excepted:{excepted}')
    if crs:
        points = f_get_spatial_ids_on_points(point_list, h_zoom, crs)
    else:
        points = f_get_spatial_ids_on_points(point_list, h_zoom)

    for point_value in points:
        spatial_log.info(f'result:{point_value}')

    if case.__le__(10):
        # 水平方向は外部サイトにて確認、IDの前方一致
        assert points[0].startswith(excepted)
    elif case.__le__(11):
        # 複数個ある場合は個数で判定
        assert excepted.__eq__(len(points))

@pytest.mark.parametrize(('point_list', 'h_zoom', 'v_zoom', 'crs', 'case'), [
    # 存在しない座標系を参照
    ([Point(139.753098, 35.685371, 100.0)], 24, 25, 1, 1),
    ([Point(139.753098, 35.685371, None)], 24, 25, 3857, 2),  # 変換エラー
    ([Point(139.753098, 35.685371, -120)], -1, 25, 6697, 3),  # 水平方向精度エラー
    ([Point(139.753098, 35.685371, 0.3)], 24, 36, 5312, 4),  # 垂直方向精度エラー
    ([Point(139.753098, 35.685371, 456)], 36, -1, 4326, 5),  # 水平、垂直方向精度エラー
    ([Point(0, 0, -1300.0), Point(180, 35.685371, None), Point(-180, 85.05112877, 1300.0)],
        24, 25, 6697, 6),  # 複数の座標処理途中でエラー
])
def test_get_spatial_ids_on_points_raise(
    point_list: list[Point], h_zoom: int, v_zoom: int,
        crs, case):
    spatial_log.info(f'case:{case}')
    spatial_log.info(f'point_list:{point_list},h_zoom:{h_zoom},v_zoom:{v_zoom},crs:{crs},case:{case}')
    with pytest.raises(SpatialIdError):
        get_spatial_ids_on_points(point_list, h_zoom, v_zoom, crs)


@pytest.mark.parametrize(('spatial_id', 'option', 'crs', 'case', 'excepted'), [
    ('18/232837/103222/18/32', enum.Point_Option.CENTER, 4326, 1, 1),  # 基準(WGS84)
    ('18/232837/103222/18/32', enum.Point_Option.VERTEX, 4326, 2, 8),  # 基準(WGS84)
    ('16/58209/25805/26/852', enum.Point_Option.CENTER, 6697, 3, 1),  # JGD2011(GRS80)
    ('8/227/100/26/852', enum.Point_Option.VERTEX, 5132, 4, 8),  # JGD2000(ベッセル楕円体)
    ('5/28/12/26/852', enum.Point_Option.CENTER, None, 5, 1),  # EPSG指定なし
    ('9/224/136/26/852', enum.Point_Option.VERTEX, 3857, 6, 8),  # 北半球西側
    ('14/5535/9872/1/1', enum.Point_Option.CENTER, None, 7, 1),  # 南半球
    ('24/0/0/26/852', enum.Point_Option.VERTEX, 4326, 8, 8),  # 地図上の左上を含むタイル
    ('10/1023/1023/1/1', enum.Point_Option.VERTEX, None, 9, 8),  # 地図上の右下を含むタイル
    ('9/256/256/26/852', enum.Point_Option.VERTEX, 4326, 10, 8),  # 原点を含むタイル
    ('10/2048/2048/26/852', enum.Point_Option.VERTEX, 4326, 11, 8),  # インデックス値の繰り上げ
    ('0/2048/2048/0/852', enum.Point_Option.VERTEX, 4326, 12, 8),  # 精度の下限
    ('35/2048/2048/35/852', enum.Point_Option.VERTEX, 4326, 13, 8),  # 精度の上限
    ('35/2048/2048/26/1', enum.Point_Option.CENTER, 4326, 14, 1),  # 精度の上限
    ('35/2048/2048/23/1', enum.Point_Option.CENTER, 4326, 15, 1),  # 精度の上限
    ('18/232837/103222/18/32', enum.Point_Option.CENTER, 4000, 16, 1),  # 投影座標でも地理座標でもない値
])
def test_get_point_on_spatial_id(spatial_id, option, crs, case, excepted):
    spatial_log.info(f'case:{case}')
    spatial_log.info(f'spatial_id:{spatial_id}, option:{option}, crs:{crs}, case{case}, excepted:{excepted}')
    if crs:
        points = get_point_on_spatial_id(spatial_id, option, crs)
    else:
        points = get_point_on_spatial_id(spatial_id, option)
    # ログに出力
    ids = re.split('/', spatial_id)
    # 中心座標の経度緯度と高さの確認
    if len(points) == 1:
        assert points[0].alt == float(ids[4]) * (2**(25-int(ids[3]))) + (2**(25-int(ids[3]))/2)
        assert math.isclose(points[0].lon, float(((360 / 2**int(ids[0])) * int(ids[1])) - 180 + ((360 / 2**int(ids[0])) / 2)), abs_tol=1e-8)
    # 頂点の場合、左上奥(北緯西経上空方向の座標)を確認
    if len(points) == 8:
        a_list = []
        lon_list = []
        for a in points:
            a_list.append(a.alt)
            if not case == 6:
                lon_list.append(a.lon)
            else:
                lon_list.append(a.x)
        assert (max(a_list)-min(a_list)) == 2**(25-int(ids[3]))
        if not case == 6:
            lon_max=max(lon_list)
            lon_min=min(lon_list)
            # 右下の場合はmaxを180とする
            spatial_log.info(2**int(ids[0])-1)
            spatial_log.info(int(ids[1]))
            if (2**int(ids[0])-1) == int(ids[1]):
                lon_max=180
            assert math.isclose(lon_max-lon_min, float(360 / 2**int(ids[0])), abs_tol=1e-8)


@pytest.mark.parametrize(('spatial_id', 'option', 'crs', 'case', 'excepted'), [
    ('18/32/232837/103222', enum.Point_Option.CENTER, 4326, 1, 1),  # 基準(WGS84)
])
def test_f_get_point_on_spatial_id(spatial_id, option, crs, case, excepted):
    spatial_log.info(f'case:{case}')
    spatial_log.info(f'spatial_id:{spatial_id}, option:{option}, crs:{crs}, case{case}, excepted:{excepted}')
    if crs:
        points = f_get_point_on_spatial_id(spatial_id, option, crs)
    else:
        points = f_get_point_on_spatial_id(spatial_id, option)
    # ログに出力
    ids = re.split('/', spatial_id)
    # 中心座標の経度緯度と高さの確認
    if len(points) == 1:
        assert points[0].alt == float(ids[1]) * (2**(25-int(ids[0]))) + (2**(25-int(ids[0]))/2)
        assert math.isclose(points[0].lon, float(((360 / 2**int(ids[0])) * int(ids[2])) - 180 + ((360 / 2**int(ids[0])) / 2)), abs_tol=1e-8)
    # 頂点の場合、左上奥(北緯西経上空方向の座標)を確認
    if len(points) == 8:
        a_list = []
        lon_list = []
        for a in points:
            a_list.append(a.alt)
            if not case == 6:
                lon_list.append(a.lon)
            else:
                lon_list.append(a.x)
        assert (max(a_list)-min(a_list)) == 2**(25-int(ids[1]))
        if not case == 6:
            lon_max=max(lon_list)
            lon_min=min(lon_list)
            # 右下の場合はmaxを180とする
            spatial_log.info(2**int(ids[0])-1)
            spatial_log.info(int(ids[2]))
            if (2**int(ids[0])-1) == int(ids[1]):
                lon_max=180
            assert math.isclose(lon_max-lon_min, float(360 / 2**int(ids[0])), abs_tol=1e-8)


@pytest.mark.parametrize(('spatial_id', 'option', 'crs', 'case', 'excepted'), [
    ('24/56/492/26/852', enum.Point_Option.CENTER, 1, 1, SpatialIdError),  # 不正なEPSG
    ('24/56/492/26/852', None, 4326, 2, SpatialIdError),  # 不正なオプション値
    ('24/56/492/', enum.Point_Option.VERTEX, 4326, 3, IndexError),  # 不正な空間ID
    ('-1/2048/2048/40/852', enum.Point_Option.VERTEX, 4326, 4, SpatialIdError),  # 精度の下限
    ('36/2048/2048/1/852', enum.Point_Option.VERTEX, 4326, 5, SpatialIdError),  # 精度の上限
    ('35/2048/2048/-1/852', enum.Point_Option.VERTEX, 4326, 6, SpatialIdError),  # 精度の下限
    ('1/2048/2048/36/852', enum.Point_Option.VERTEX, 4326, 7, SpatialIdError),  # 精度の上限
    ('36/2048/2048/36/852', enum.Point_Option.VERTEX, 4326, 8, SpatialIdError),  # 精度の上限
    ('-1/2048/2048/-1/852', enum.Point_Option.VERTEX, 4326, 9, SpatialIdError),  # 精度の下限
    ('', enum.Point_Option.VERTEX, 4326, 10, IndexError),  # 不正な空間ID(空文字)
])
def test_get_point_on_spatial_id_raise(spatial_id, option, crs, case, excepted):
    spatial_log.info(f'case:{case}')
    spatial_log.info(f'spatial_id:{spatial_id}, option:{option}, crs:{crs}, case{case}, excepted:{excepted}')
    with pytest.raises(excepted):
        get_point_on_spatial_id(spatial_id, option, crs)


@pytest.mark.parametrize(('spatial_id', 'option', 'crs'), [
    ('24/56/492/26/852', enum.Point_Option.CENTER, 4326),
])
def test_get_point_on_spatial_id_raise_mock(spatial_id, option, crs, mocker):
    with pytest.raises(SpatialIdError):
        mocker.patch(
            'pyproj.Transformer.transform', side_effect=Exception)

        # 例外を発生させる
        get_point_on_spatial_id(spatial_id, option, 5132)


@pytest.mark.parametrize(
    ('projected_point_list', 'projected_crs', 'geographic_crs',
     'case', 'excepted'), [
        ([Projected_Point(15557243.71, 4257414.82, 100)],
         3857, 6697, 1, Point(0, 0, 0)),  # 単実行
        ([Projected_Point(15557243.71, 4257414.82, 100)],
         3857, None, 2, Point(0, 0, 0)),  # EPSG指定なし
        ([Projected_Point(15557243.71, 4257414.82, 100),
          Projected_Point(15557243.71, 4257414.82, 100),
          Projected_Point(15557243.71, 4257414.82, 100)],
         3857, 6697, 3, 3),  # 複数の変換
    ])
def test_convert_projected_point_list_to_point_list(
        projected_point_list, projected_crs, geographic_crs,
        case, excepted):

    point_list = []
    spatial_log.info(f'case:{case}')
    spatial_log.info(f'projected_point_list:{projected_point_list}, projected_crs:{projected_crs}, geographic_crs:{geographic_crs},case:{case}, excepted:{excepted}')
    if geographic_crs:
        point_list = convert_projected_point_list_to_point_list(
                        projected_point_list, projected_crs, geographic_crs)
    else:
        point_list = convert_projected_point_list_to_point_list(
                        projected_point_list, projected_crs)
    spatial_log.info(point_list)
    if case.__le__(2):
        assert type(excepted) == (type(point_list[0]))
    elif (case.__le__(3) and type(excepted) is int):
        assert excepted.__eq__(len(point_list))

@pytest.mark.parametrize(
    ('projected_point_list', 'projected_crs', 'geographic_crs', 'case'), [
        ([Projected_Point(15557243.71, 4257414.82, 100)], 6697, 4326, 1),  # 入力EPSGが地理座標
        ([Projected_Point(15557243.71, 4257414.82, 100)], 3857, 3857, 2),  # 出力EPSGが投影座標
        ([Projected_Point(15557243.71, 4257414.82, 100)], 6697, 3857, 3),  # 入出力EPSGの組み合わせが逆
        ([Projected_Point(15557243.71, 4257414.82, 100)], 1, 3857, 4),  # 入力に存在しないEPSG
        ([Projected_Point(15557243.71, 4257414.82, 100)], 6697, 1, 5),  # 出力に存在しないEPSG
        ([Point(139.753098, 35.685371, 100)], 3857, 6697, 6),  # 入力に投影座標を使用
     ])
def test_convert_projected_point_list_to_point_list_raise(
        projected_point_list, projected_crs, geographic_crs, case):

    spatial_log.info(f'case:{case}')
    with pytest.raises(SpatialIdError):
        convert_projected_point_list_to_point_list(
            projected_point_list, projected_crs, geographic_crs)


@pytest.mark.parametrize(
    ('point_list', 'projected_crs', 'geographic_crs', 'case'), [
        ([Point(139.753098, 35.685371, 100)], 4326, 4326, 1),  # 入力EPSGが投影座標
        ([Point(139.753098, 35.685371, 100)], 3857, 3857, 2),  # 出力EPSGが地理座標
        ([Point(139.753098, 35.685371, 100)], 1, 3857, 3),  # 入力に存在しないEPSG
        ([Point(139.753098, 35.685371, 100)], 4326, 1, 4),  # 出力に存在しないEPSG
        ([Point(139.753098, 35.685371, 100)], 4326, 3857, 5),  # 入出力EPSGの組み合わせが逆
        ([Projected_Point(15557243.71,4257414.82, 100)], 4326, 3857, 6),  # 入力に投影座標を使用
    ])
def test_convert_point_list_to_projected_point_list_raise(
            point_list: list[object], projected_crs: int, geographic_crs, case):
    spatial_log.info(f'case:{case}')

    with pytest.raises(SpatialIdError):
        convert_point_list_to_projected_point_list(
            point_list, projected_crs, geographic_crs)


@pytest.mark.parametrize(('projected_point', 'projection_crs', 'crs'), [
    ([Projected_Point(15557243.71, 4257414.82, 100)],
         3857, 6697),
])
def test_convert_projected_point_list_to_point_list_raise_mock(projected_point, projection_crs, crs, mocker):
    with pytest.raises(SpatialIdError):
        mocker.patch(
            'pyproj.Transformer.transform', side_effect=Exception)
        # 変換エラーが発生した場合の試験
        convert_projected_point_list_to_point_list(projected_point, projection_crs, crs)


@pytest.mark.parametrize(('point', 'projection_crs', 'crs'), [
    ([Point(139.753098, 35.685371, 100)],
         3857, 6697),
])
def test_convert_point_list_to_projected_point_list_raise_mock(point, projection_crs, crs, mocker):
    with pytest.raises(SpatialIdError):
        mocker.patch(
            'pyproj.Transformer.transform', side_effect=Exception)
        # 変換エラーが発生した場合の試験
        convert_point_list_to_projected_point_list(point, projection_crs, crs)


@pytest.mark.parametrize(
    ('zoom', 'case', 'excepted'), [
        (0, 1, True),  # 下限
        (35, 2, True),  # 上限
        (-1, 3, False),  # 下限エラー
        (36, 4, False),  # 上限エラー
    ])
def test_check_zoom(
        zoom,
        case, excepted):

    spatial_log.info(f'case:{case}')
    assert excepted == check_zoom(zoom)

# 以下は非公開関数のテスト
@pytest.mark.parametrize(
    ('lon', 'lat', 'zoom',
     'case', 'excepted'), [
        (139.753098, 35.685371, 14, 1, '14/14552/6451'),
     ])
def test__get_horizontal_tile_id_on_point(
        lon: float, lat: float, zoom: int, case, excepted):
    spatial_log.info(f'case:{case}')
    tile_id = _get_horizontal_tile_id_on_point(lon, lat, zoom)
    assert tile_id.__eq__(excepted)


@pytest.mark.parametrize(
    ('alt', 'zoom', 'case'), [
        (56.9, 25, 1),
        (-0.06, 26, 2),
     ])
def test__get_vertical_tile_id_on_altitude(
        alt: float, zoom: int, case):
    spatial_log.info(f'case:{case}')
    excepted_id = str(zoom)+'/'+str(floor(alt * 2**zoom / 2**25))
    tile_id = _get_vertical_tile_id_on_altitude(alt, zoom)
    spatial_log.info(tile_id)
    assert tile_id.endswith(excepted_id)


@pytest.mark.parametrize(
    ('alt_index', 'zoom', 'case'), [
        (56, 25, 1),
        (32.5, 26, 1),
        (-309.3, 30, 1),
     ])
def test__get_altitude_on_vertical_index_and_zoom(
        alt_index: int, zoom: int, case):
    spatial_log.info(f'case:{case}')
    excepted_alt = alt_index * 2**25 / 2**zoom
    excepted_resolutin = 2**25 / 2**zoom
    vertical_point = _get_altitude_on_vertical_index_and_zoom(
            alt_index, zoom)
    assert (vertical_point.alt == excepted_alt
           and vertical_point.resolution == excepted_resolutin)


@pytest.mark.parametrize(
    ('lon_index', 'lat_index', 'h_zoom', 'vertical_point',
     'case', 'expected_lon', 'expected_alt'), [
        (2, 2, 2, Vertical_Point(alt=56, resolution=1), 1, 45, 56.5),  # 2^2の分割
        (200, 2, 2, Vertical_Point(alt=57, resolution=2), 2, -135, 58),  # 経度方向のインデックスが範囲外
        (2, 1000, 2, Vertical_Point(alt=56, resolution=1), 3, 45, 56.5),  # 緯度方向のインデックスが一定の値より大きい
        (2, -1, 2, Vertical_Point(alt=1, resolution=1), 4, 45, 1.5),  # 緯度方向のインデックスが0未満
     ])
def test__get_center_point_on_voxel_offset(
        lon_index: int, lat_index: int, h_zoom: int, vertical_point: object,
        case, expected_lon, expected_alt):
    spatial_log.info(f'case:{case}')
    center_point = _get_center_point_on_voxel_offset(
        lon_index, lat_index, h_zoom, vertical_point)
    # 入力値に基づいたボクセルの中心の座標を確認
    # 経度
    assert center_point.lon == expected_lon
    # 高さ
    assert center_point.alt == expected_alt
    # 緯度の値はプログラムで検算
    # case3のテストでは緯度方向のインデックスを修正
    if case == 3:
        lat_index = 2**h_zoom-1
    # case4のテストでは緯度方向のインデックスを0にする
    elif case == 4:
        lat_index = 0
    lat_index += 0.5
    assert math.isclose(
        center_point.lat,
        degrees(atan(sinh(pi * (1 - 2 * lat_index / 2.0**h_zoom)))),
        abs_tol=1e-10
    )


@pytest.mark.parametrize(
    ('lon_index', 'lat_index', 'h_zoom', 'vertical_point',
     'case', 'excepted'), [
        (0, 0, 25, Vertical_Point(alt=56, resolution=1),
         1, Point(-180, 85.05112877, 56)),  # 左上の座標
        (1099511627775, 1099511627775, 40, Vertical_Point(alt=56, resolution=1),
         2, Point(180, -85.05112877, 56)),  # 右下の座標
        (7, 7, 4, Vertical_Point(alt=56, resolution=1),
         3, Point(0, 0, 56)), # 原点
        (15, 15, 4, Vertical_Point(alt=56, resolution=1),
         4, Point(180, -85.05112877, 56)), #経度22.5度刻み、右下を確認
        (1, 1, 1, Vertical_Point(alt=56, resolution=1),
         5, Point(180,-85.05112877, 56)), #経度90度刻み、右下を確認
        (32, 32, 5, Vertical_Point(alt=56, resolution=1),
         6, Point(-180,-85.05112877, 56)), #水平方向のインデックスが限界以上
        (-1, -1, 5, Vertical_Point(alt=56, resolution=1),
         7, Point(180,85.05112877, 56)), #水平方向のインデックスが負
     ])
def test__get_vertex_on_voxel_offset(
        lon_index: int, lat_index: int, h_zoom: int,
        vertical_point: object, case, excepted):
    spatial_log.info(f'case:{case}')
    point_list = []
    point_list = _get_vertex_on_voxel_offset(
        lon_index, lat_index, h_zoom, vertical_point)
    # 特徴点で評価をする
    lat_list = []
    for point in point_list:
        spatial_log.info(point.lon)
        spatial_log.info(point.lat)
        if point.lon == excepted.lon:
            lat_list.append(point.lat)
    assert (math.isclose(min(lat_list), excepted.lat, abs_tol=1e-8)
            or math.isclose(max(lat_list), excepted.lat, abs_tol=1e-8))


# 実行時間の確認
@pytest.mark.parametrize(
    ('count','crs','case'), [
        (100000,4326,2), #WGS84
        (100000,5132,4), # ベッセル楕円体
        (100000,6668,5), # 日本測地系(2011)
        (100000,6697,6), # 日本測地系(2011)＋高さ
        (100000,4612,7), # 日本測地系(2000)
     ])
def test_get_spatial_ids_on_points_calc(count,crs,case):
    return
    spatial_log.info(f'case:{case}')
    spatial_log.info(f'crs:{crs}')
    point_list = []
    id_list = []
    # 乱数で問題がないことを確認
    start_time = time.time()

    for i in range(count):
        p_ins = Point(random.random()*180, random.random()*85, abs(random.random()*300))
        #print(p_ins.lon)
        point_list.append(p_ins)
    id_list = get_spatial_ids_on_points(point_list, 20, 15, crs) 
    
    end_time = time.time()  #終了の時間
    run_time = end_time - start_time
    spatial_log.info(f"get_spatial_ids_on_points実行時間：{run_time}")

    start_time2 = time.time()
    point_list = []
    for spatial_id in id_list:
        point_list.extend(get_point_on_spatial_id(spatial_id, enum.Point_Option.CENTER, crs))
    end_time2 = time.time()  #終了の時間
    run_time2 = end_time2 - start_time2
    spatial_log.info(f"get_point_on_spatial_id実行時間：{run_time2}")
    assert count == (len(point_list))

# 以下はズレ検証用テスト
@pytest.mark.parametrize(
    ('point',
     'case', 'excepted'), [
        ([Point(139.753098, 35.685371, 0.0)],1,True),
        ([Point(10, 10, 0.0)],2,True),
        ([Point(10, 85.0511287798, 0.0)],3,True),
        ([Point(10, -21.9430455334, 0.0)],4,True),
        ([Point(10, -10.9430455334, 0.0)],5,True),
        ([Point(10, -85.0511287798, 0.0)],6,True),
     ])
def test_conbine(point, case, excepted):
    spatial_log.info(f'case:{case}')
    for t in range(0, 36):
        ids = get_spatial_ids_on_points(point, t, 5)
        spatial_log.info(ids)

        spatial_id=ids[0]
        excepted_list=[]

        if point[0].lat <= 0:
            move=-1
        else:
            move=1
        # 頂点座標と同じ空間IDの取得
        excepted_list.append(spatial_id)
        excepted_list.append(get_shifting_spatial_id(spatial_id, 0, 0, 1))
        excepted_list.append(get_shifting_spatial_id(spatial_id, 0, move, 1))
        excepted_list.append(get_shifting_spatial_id(spatial_id, 1, move, 1))
        excepted_list.append(get_shifting_spatial_id(spatial_id, 1, 0, 1))
        excepted_list.append(get_shifting_spatial_id(spatial_id, 1, 0, 0))
        excepted_list.append(get_shifting_spatial_id(spatial_id, 1, move, 0))
        excepted_list.append(get_shifting_spatial_id(spatial_id, 0, move, 0))

        points = []
        points = get_point_on_spatial_id(spatial_id, enum.Point_Option.VERTEX)

        conv_p = []
        conv_p = get_spatial_ids_on_points(points, t, 5)
        # 重複している空間IDを削除
        check_conv_p = []
        check_conv_p = list(set(conv_p))
        spatial_log.info(check_conv_p)
        if t == 0:
            # t=0の場合は空間IDは2つ
            assert 2==len(check_conv_p)
        # 南半球の場合
        elif case >= 4:
            lat_list=[]
            for id in check_conv_p:
                id_c = re.split('/', id)
                lat_list.append(int(id_c[2]))

            if max(lat_list)-min(lat_list)>1:
                spatial_log.info(check_conv_p)
                # 緯度インデックスが2つ以上離れた空間がある場合はエラー
                assert False
            # (0,0)を含む場合は空間IDが重複する
            if max(lat_list) == 2**t/2:
                for check_conv_p_id in check_conv_p:
                    assert check_conv_p_id in excepted_list and len(check_conv_p) == 4
                    return

            # 元となる空間IDの緯度インデックスが最大の場合は、水平方向の最大座標と最小座標の再変換後の空間IDが一致する
            for check_conv_p_id in check_conv_p:
                assert check_conv_p_id in excepted_list and len(check_conv_p) == 8
        else:
            lat_list=[]
            for id in check_conv_p:
                id_c = re.split('/', id)
                lat_list.append(int(id_c[2]))
            if max(lat_list)-min(lat_list)>1:
                spatial_log.info(check_conv_p)
                # 緯度インデックスが2つ以上離れた空間がある場合はエラー
                assert False
            # 重複する空間IDはないため、数に変更はない(緯度経度高さの最大最小の組み合わせ≒2^3)
            for check_conv_p_id in check_conv_p:
                assert check_conv_p_id in excepted_list and len(check_conv_p) == 8


@pytest.mark.parametrize(
    ('point',
     'case', 'excepted'), [
        ([Point(-180,85.0511287798,-0.1)],1,True),
        ([Point(139.753098, 35.685371,-0.1)],2,True),
     ])
def test_tileid(point, case, excepted):
    # 精度ごとのタイルの確認をする
    for t in range(0, 36):
        ids = get_spatial_ids_on_points(point, t, 5)
        spatial_log.info(ids)
        lat_index=0
        for spatial_id in ids:
            spatial_log.info(spatial_id)
            id = re.split('/', spatial_id)
            if lat_index==0:
                lat_index=int(id[2])
            if int(id[2]) < 0:
                # 負のインデックスの検知
                spatial_log.info(spatial_id)
                assert False
            
            if (int(id[2]) - lat_index) > 1 :
                spatial_log.info(spatial_id)
                # 緯度インデックスが2つ以上離れた空間がある場合はエラー
                assert False
    # エラーがなければOKとする
    assert excepted



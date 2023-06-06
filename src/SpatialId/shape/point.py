# Copyright © 2022 Digital Agency. All rights reserved.
"""
|  Requirements: Python 3.9+.

|  空間ID取得モジュール。
|  入力された地理座標から空間IDを出力する。
|  空間IDからその空間IDが示す頂点の座標、もしくは中心点の座標を地理座標、または、投影座標で出力する。
|  ユーザから入力された座標参照系をライブラリ内で空間座標系(EPSG4326)の座標に変換をし各操作をする。
|  CRSの指定はEPSGコードによって行う。
|  CRSとEPSGコードの一例は下記。

    ========================= =============
    CRS(地理座標系)            EPEGコード
    ========================= =============
    WGS84(GPS)                 4326
    日本測地系(2011)           6668
    日本測地系(2011) + (高さ)  6697
    日本測地系(2000)           4612
    日本測地系(1892)           5132
    ========================= =============
    
    ========================= =============
    CRS(投影座標系)            EPEGコード
    ========================= =============
    Webメルカトル              3857
    平面直角座標系             6669〜87
    ========================= =============
"""

import enum
import re
from logging import getLogger

from SpatialId.shape import get_transformer

from SpatialId import LOGGER_NAME
from SpatialId.common.object import enum as const
from SpatialId.common.object.point import Point, Projected_Point
from SpatialId.common.object.point import Vertical_Point
from SpatialId.common.exception import SpatialIdError

from math import cos, log, pi, tan, floor, radians, atan, sinh, degrees

from SpatialId.convert import in_to_internal, internal_to_out

spatial_log = getLogger(LOGGER_NAME).getChild(__name__)

# CRSのデフォルト値(WGS84)のEPSGコード
CRS = 4326

def get_spatial_ids_on_points(
        point_list: list[Point], h_zoom: int, v_zoom: int,
        crs: int = CRS
) -> list[str]:
    """
    |  地理座標オブジェクトのリストから空間IDのリストを取得する。

    :param point_list: 地理座標が格納されたオブジェクトのリスト
    :type point_list: list[object]
    :param h_zoom: 水平方向の精度レベル
    :type h_zoom: int
    :param v_zoom: 垂直方向の精度レベル
    :type v_zoom: int
    :param crs: ユーザ指定のCRSのEPSGコード(未指定の場合はWGS84のEPSGコード(4326)が割り当てられる)
    :type crs: int
    :raise SpatialIdError: pyprojを用いた変換時にInfまたはNaNが発生した場合にエラー
    :raise StatialIdError: 存在しないEPSGコードが入力された場合、エラー
    :returns: 空間IDのリスト
    :rtype: list[str]
    """
    spatial_log.debug((
        "point_list:", point_list,
        "h_zoom:", h_zoom,
        "v_zoom:", v_zoom,
        "crs:", crs),)
    # 空間IDを格納する変数
    spatial_ids = []

    # 入力値チェック
    # 水平、垂直方向の精度がのどちらかが範囲内以外の場合、エラーとする。
    if not (check_zoom(h_zoom) and check_zoom(v_zoom)):
        raise SpatialIdError("INPUT_VALUE_ERROR")

    # 入力座標の測地系と出力後の測地系を設定する
    # ユーザ入力の地理座標系→WGS84の変換オブジェクトの作成
    # 座標の入出力を[経度、緯度]で固定
    try:
        transformer = get_transformer(crs, CRS)
        # 入力の座標系が地理座標系ではない場合、エラーとする。
        if not transformer.source_crs.is_geographic:
            raise SpatialIdError("VALUE_CONVERT_ERROR")      
    except Exception:
        raise SpatialIdError("VALUE_CONVERT_ERROR")

    # 先頭から空間IDに変換する
    for input_point in point_list:
        point = Point(
            input_point.lon, input_point.lat, input_point.alt)

        # CRSの変換が不要の場合は、変換処理をスキップする
        if crs.__ne__(CRS):
            # WGS84の座標に変換する。
            try:
                point.lon, point.lat, point.alt = transformer.transform(
                    input_point.lon, input_point.lat, input_point.alt,
                    errcheck=True)
            except Exception:
                raise SpatialIdError("VALUE_CONVERT_ERROR")

        # 水平方向のタイルを作成する
        h_id = _get_horizontal_tile_id_on_point(
            point.lon, point.lat, h_zoom)
        # 垂直方向のタイルIDを作成する
        v_id = _get_vertical_tile_id_on_altitude(point.alt, v_zoom)
        # 水平方向と垂直方向のタイルを組み合わせて空間IDとする
        spatial_ids.extend([h_id+'/'+v_id])

    spatial_log.debug(("return value:", spatial_ids),)
    return spatial_ids


def f_get_spatial_ids_on_points(point_list: list[Point], zoom: int,
        crs: int = CRS) -> list[str]:
    """
    |  地理座標オブジェクトのリストから空間IDのリストを取得する。
    
    | f_"がついてないAPIとの違いは以下：
    | 入力の空間ID精度レベルを水平・垂直方向共に統一
    | 出力の空間IDの並びは仕様通りの記載。
    | z/f/x/y :z{ズームレベル}/f{高さの位置}/x{経度方向の位置}/y{緯度方向の位置}
    | 参照：https://github.com/unvt/zfxy-spec
    
    :param point_list: 地理座標が格納されたオブジェクトのリスト
    :type point_list: list[object]
    :param zoom: 空間IDの精度レベル
    :type zoom: int
    :param crs: ユーザ指定のCRSのEPSGコード(未指定の場合はWGS84のEPSGコード(4326)が割り当てられる)
    :type crs: int
    :raise SpatialIdError: pyprojを用いた変換時にInfまたはNaNが発生した場合にエラー
    :raise StatialIdError: 存在しないEPSGコードが入力された場合、エラー
    :returns: 空間IDのリスト
    :rtype: list[str]
    """
    out = get_spatial_ids_on_points(point_list, zoom, zoom,
          crs)
    return internal_to_out(out)


def get_point_on_spatial_id(
        spatial_id: str, option: enum.Enum, crs: int = CRS) -> list[object]:
    """
    |  単一の空間IDからその空間IDの頂点座標、または中心座標を取得する。
    |  頂点の座標は南緯、東経、上空方向は近接する頂点の座標と共有される。
    |  座標は地理座標、または投影座標で返却される。

    :param spatial_id: 空間ID
    :type spatial_id: str
    :param option: 共通クラスから取得したPoint_OptionのEnum
    :type option: Enum
    :param crs: ユーザ指定のCRSのEPSGコード(未指定の場合はWGS84のEPSGコード(4326)が割り当てられる)
    :type crs: int
    :raise StatialIdError: pyprojを用いた変換時にInfまたはNaNが発生した場合にエラー
    :raise StatialIdError: 存在しないEPSGコードが入力された場合、エラー
    :raise StatialIdError: 指定外のoptionを指定した場合にエラー
    :returns: 空間IDの各頂点の座標が格納されたオブジェクトのリスト、または空間IDの中心点の座標
    :rtype: list[object]
    """
    spatial_log.debug(
        ("spatial_id:", spatial_id, "option:", option, "crs:", crs),)
    # 座標を格納するリスト
    output_point = []

    # 空間IDから座標成分を取得する
    # 空間IDは"[水平精度]/[経度インデックス]/[緯度インデックス]/[垂直精度]/[高さインデックス]"形式とする。
    # 空間IDをx水平方向、y水平方向、垂直方向に分割する
    id = re.split('/', spatial_id)

    # IDから水平方向の位置と精度を取得する。前方2桁は精度
    lon_index = int(id[1])
    lat_index = int(id[2])
    h_zoom = int(id[0])

    # IDから垂直方向の位置、分解能を取得する。前方2桁は精度
    alt_index = int(id[4])
    v_zoom = int(id[3])

    # 入力値チェック
    # 水平、垂直方向の精度のどちらかが範囲内以外の場合、エラーとする。
    if not (check_zoom(h_zoom) and check_zoom(v_zoom)):
        raise SpatialIdError("INPUT_VALUE_ERROR")

    vertical_point = _get_altitude_on_vertical_index_and_zoom(
        alt_index, v_zoom)

    # オプションで呼び出しの分岐
    # NOTE:可読性を優先してループ内で分岐。遅いようならif文はループの外で実施し、
    # 別のメソッドとして分離。
    if option == const.Point_Option.CENTER:
        # 中心点の座標を取得する。
        output_point.append(
            _get_center_point_on_voxel_offset(
                lon_index, lat_index, h_zoom, vertical_point))
    elif option == const.Point_Option.VERTEX:
        # 頂点の座標(計8個)を取得する。
        output_point = _get_vertex_on_voxel_offset(
                lon_index, lat_index, h_zoom, vertical_point)
    else:
        raise SpatialIdError("OPTION_FAILED_ERROR")

    # 変換後の基準となる座標を取得する(ボクセルの左下手前の座標)
    # 入力座標の測地系と出力後の測地系を設定する
    # WGS84→ユーザ入力の地理座標系変換オブジェクトの作成
    # 座標の入出力順序を[経度、緯度]で固定
    try:
        # 変換が不要の場合はそのまま返却する
        if CRS == crs:
            spatial_log.debug(("return value:", output_point),)
            return output_point
        # transformerオブジェクトを取得する
        transformer = get_transformer(CRS, crs)
    except Exception:
        raise SpatialIdError("VALUE_CONVERT_ERROR")
    convert_point = []
    # ユーザ入力の座標系が投影座標の場合は、関数を呼び出す。
    if transformer.target_crs.is_projected:
        convert_point = convert_point_list_to_projected_point_list(
            output_point, crs)

    # 変換先が地理座標かつ、WGS84ではない場合、指定の地理座標に変換する
    elif(transformer.target_crs.is_geographic):
        for origin_point in output_point:
            try:
                conv_lon, conv_lat, conv_alt = transformer.transform(
                    origin_point.lon, origin_point.lat, origin_point.alt,
                    errcheck=True)
            except Exception:
                raise SpatialIdError("VALUE_CONVERT_ERROR")

            point = Point(conv_lon, conv_lat, conv_alt)
            convert_point.append(point)
    spatial_log.debug(("return value:", convert_point),)
    return convert_point


def f_get_point_on_spatial_id(spatial_id: str, option: enum.Enum, crs: int = CRS) -> list[object]:
    """
    |  単一の空間IDからその空間IDの頂点座標、または中心座標を取得する。
    |  頂点の座標は南緯、東経、上空方向は近接する頂点の座標と共有される。
    |  座標は地理座標、または投影座標で返却される。

    | f_"がついてないAPIとの違いは以下：
    | 入出力の空間IDの並びは仕様通りの記載。
    | z/f/x/y :z{ズームレベル}/f{高さの位置}/x{経度方向の位置}/y{緯度方向の位置}
    | 参照：https://github.com/unvt/zfxy-spec

    :param spatial_id: 空間ID
    :type spatial_id: str
    :param option: 共通クラスから取得したPoint_OptionのEnum
    :type option: Enum
    :param crs: ユーザ指定のCRSのEPSGコード(未指定の場合はWGS84のEPSGコード(4326)が割り当てられる)
    :type crs: int
    :raise StatialIdError: pyprojを用いた変換時にInfまたはNaNが発生した場合にエラー
    :raise StatialIdError: 存在しないEPSGコードが入力された場合、エラー
    :raise StatialIdError: 指定外のoptionを指定した場合にエラー
    :returns: 空間IDの各頂点の座標が格納されたオブジェクトのリスト、または空間IDの中心点の座標
    :rtype: list[object]
    """
    converted = in_to_internal(spatial_id)
    out = get_point_on_spatial_id(converted, option, crs)
    return out


def _get_horizontal_tile_id_on_point(lon: float, lat: float, zoom: int) -> str:
    """
    | 経度緯度から水平方向のタイルIDを計算する。
    | 水平方向のタイルIDは「[精度]/[x位置]/[y位置]」となる。

    :param lon: 空間座標系の経度(単位:度)
    :type lon: float
    :param lat: 空間座標系の緯度(単位:度)
    :type lat: float
    :param zoom: 水平方向の精度
    :type zoom: int
    :returns: 水平方向のタイルID
    :rtype: str
    """
    spatial_log.debug(("lon:", lon, "lat:", lat, "zoom", zoom),)
    # 水平方向の精度
    # 経度方向のインデックスの計算
    # 経度に180が入力されているとタイルインデックス+1の値が出力されるため、補正する
    if(lon == 180):
        lon = -lon
    lon_index = floor((2**zoom) * ((lon + 180.0) / 360.0))
    # 緯度方向のインデックスの計算
    lat_index = floor((2**zoom) *
                      (1 - log(tan(radians(lat)) +
                       (1 / cos(radians(lat)))) / pi) / 2)

    # 精度とx方向とy方向のIDを結合する
    tile_id = [str(zoom), '/',
               str(lon_index), '/', str(lat_index)]

    spatial_log.debug(("return value:", tile_id),)
    return ''.join(tile_id)


def _get_vertical_tile_id_on_altitude(alt: float, zoom: int) -> str:
    """
    | 垂直方向のタイルIDを計算する。
    | 垂直方向のタイルIDは「[精度]/[位置]」となる。

    :param alt: 高さ(単位:m)
    :type alt: float
    :param zoom: 垂直方向の精度
    :type zoom: int
    :returns: 垂直方向のタイルID
    :rtype: str
    """
    spatial_log.debug(("alt:", alt, "zoom:", zoom),)
    # 高さ全体の精度あたりの垂直方向の精度
    alt_resolution = 2**(25-zoom)
    # 垂直方向の位置を計算する
    v_index = floor((alt) / alt_resolution)
    tile_id = [str(zoom), '/', str(v_index)]

    spatial_log.debug(("return value:", ''.join(tile_id)),)
    return ''.join(tile_id)


def _get_altitude_on_vertical_index_and_zoom(
        alt_index: int, zoom: int) -> object:
    """
    |  垂直方向の位置と精度から原点に近い高さと分解能を取得する。
    |  入力された位置が負の場合は、原点に遠い高さと分解能を取得する。

    :param alt_index: 高さの位置
    :type alt_index: int
    :param zoom: 垂直方向精度
    :type zoom: int
    :returns: 高さ(単位:m)と分解能が格納されたオブジェクト
    :rtype: object
    """
    spatial_log.debug(("alt_index:", alt_index, "zoom:", zoom),)
    # 高さ全体の精度あたりの分解能を取得する
    Vertical_Point.resolution = 2**(25-zoom)
    Vertical_Point.alt = alt_index * Vertical_Point.resolution

    spatial_log.debug(("return value:", Vertical_Point),)
    return Vertical_Point


def _get_center_point_on_voxel_offset(
        lon_index: int, lat_index: int, h_zoom: int,
        vertical_point: object) -> list[object]:
    """
    |  緯度、経度、高さから精度ごとの一辺の長さのボクセルの中心点の座標を取得する。

    :param lon_index: 経度方向のインデックス
    :type  lon_index: int
    :param lat_index: 緯度方向のインデックス
    :type  lat_index: int
    :param h_zoom: 水平方向の精度
    :type  h_zoom: int
    :param vertical_point: 高さのオブジェクト
    :type  vertical_point: object
    :returns: 空間の中心点の座標が格納されたオブジェクト
    :rtype: object
    """
    spatial_log.debug(
        ("lon_index:", lon_index,
         "lat_index:", lat_index,
         "h_zoom:", h_zoom,
         "vertical_point:", vertical_point),)

    # 経度の取得
    if lon_index.__gt__(2**h_zoom-1) or lon_index.__lt__(0):
        # インデックスの範囲を超えている場合はn周分を無視する
        lon_index = lon_index % 2**h_zoom
    center_lon_index = lon_index+0.5
    center_lon = _calc_lon_on_index(center_lon_index, h_zoom)

    # 緯度の取得
    if lat_index.__gt__(2**h_zoom-1):
        lat_index = 2**h_zoom-1
    elif lat_index.__lt__(0):
        lat_index = 0
    center_lat_index = lat_index+0.5
    center_lat = _calc_lat_on_index(center_lat_index, h_zoom)

    center_alt = vertical_point.alt + (vertical_point.resolution/2)

    point = Point(center_lon, center_lat, center_alt)
    spatial_log.debug(point)
    return point


def _get_vertex_on_voxel_offset(
        lon_index: int, lat_index: int, h_zoom: int,
        vertical_point: object) -> list[object]:
    """
    |  緯度、経度、高さから分解能を一辺の長さとしたボクセルの頂点を取得する。

    :param lon_index: 経度方向のインデックス
    :type  lon_index: int
    :param lat_index: 緯度方向のインデックス
    :type  lat_index: int
    :param h_zoom: 水平方向の精度
    :type  h_zoom: int
    :param vertical_point: 高さのオブジェクト
    :type  vertical_point: object
    :returns: 各頂点のリスト
    :rtype: list[object]
    """
    spatial_log.debug(
        ("lon_index:", lon_index,
         "lat_index:", lat_index,
         "h_zoom:", h_zoom,
         "vertical_point", vertical_point),)
    # 返却用のリスト
    point_list = []

    # 緯度の取得
    if lat_index.__gt__(2**h_zoom-1):
        lat_index = 2**h_zoom-1
    elif lat_index.__lt__(0):
        lat_index = 0

    # タイルの上辺の緯度
    north_lat = _calc_lat_on_index(lat_index, h_zoom)

    # タイルの下辺の緯度
    south_lat = _calc_lat_on_index(lat_index+1, h_zoom)

    # 経度の取得
    if lon_index.__gt__(2**h_zoom-1) or lon_index.__lt__(0):
        # インデックスの範囲を超えている場合はn周分を無視する
        lon_index = lon_index % 2**h_zoom

    west_lon = _calc_lon_on_index(lon_index, h_zoom)
    east_lon = _calc_lon_on_index(lon_index+1, h_zoom)
    # 各頂点を計算する。左上から時計回りに取得をする。
    north_west_bottom = [
        west_lon, north_lat, vertical_point.alt]
    north_east_bottom = [
        east_lon, north_lat, vertical_point.alt]
    south_west_bottom = [
        west_lon, south_lat, vertical_point.alt]
    south_east_bottom = [
        east_lon, south_lat, vertical_point.alt]
    north_west_top = [
        west_lon, north_lat,
        vertical_point.alt + vertical_point.resolution]
    north_east_top = [
        east_lon, north_lat,
        vertical_point.alt + vertical_point.resolution]

    south_west_top = [
        west_lon, south_lat,
        vertical_point.alt + vertical_point.resolution]
    south_east_top = [
        east_lon, south_lat,
        vertical_point.alt + vertical_point.resolution]

    # 頂点を一つのリストにまとめる
    vertex_list = [north_west_bottom, north_east_bottom,
                   south_east_bottom, south_west_bottom,
                   north_west_top, north_east_top,
                   south_east_top, south_west_top]

    for vertex_point in vertex_list:
        # 頂点のリストをオブジェクトのリストに格納する。
        point = Point(vertex_point[0], vertex_point[1], vertex_point[2])
        point_list.append(point)

    spatial_log.debug(("return value:", point_list),)
    # 変換した座標を格納したオブジェクトのリストを返却する
    return point_list


def _calc_lat_on_index(lat_index: float, zoom: int) -> float:
    """
    |  緯度方向のインデックスと精度から緯度を計算する。

    :param lat_index: 緯度方向のインデックス
    :type  lat_index: int
    :param h_zoom: 精度
    :type  h_zoom: int
    :returns: 経度,緯度
    :rtype: float
    """

    lat = degrees(atan(sinh(pi * (1 - 2 * lat_index / 2.0**zoom))))
    return lat


def _calc_lon_on_index(lon_index: int, zoom: int) -> float:
    """
    |  経度方向のインデックスと精度から経度を計算する。

    :param lon_index: 経度方向のインデックス
    :type  lon_index: int
    :param h_zoom: 精度
    :type  h_zoom: int
    :returns: 経度
    :rtype: float
    """

    lon = lon_index * 360 / 2**zoom - 180
    return lon


def convert_projected_point_list_to_point_list(
        projected_point_list: list[object],
        projected_crs: int, geographic_crs: int = CRS) -> list[object]:
    """ユーザ指定の投影座標系のリストを地理座標のリストに変換する。

    :param projected_point_list: 投影座標のデータクラスオブジェクトのリスト
    :type projected_point_list: list[object]
    :param projected_crs: 入力の投影座標のCRSのEPSGコード
    :type projected_crs: int
    :param geographic_crs: 出力の地理座標のCRSのEPSGコード。指定がない場合はWGS84を使用する。
    :type geographic_crs: int
    :raises SpatialIdError: 入力されたEPSGコードが存在しない場合、エラー
    :raises SpatialIdError: 入力されたEPSGコードが投影座標ではない場合、エラー
    :return: 地理座標のデータクラスオブジェクトのリストを返却する。
    :rtype: list[object]
    """
    spatial_log.debug(
        ("projected_point_list:", projected_point_list,
         "projected_crs:", projected_crs,
         "geographic_crs:", geographic_crs
         ),)

    try:
        # 投影座標を地理座標に変換するpyprojオブジェクトを作成する。
        transformer = get_transformer(projected_crs, geographic_crs)

        # 返却用ポイントリスト
        point_list = []

        # 入力されたEPSGコードのチェックをする。
        if (not transformer.source_crs.is_geographic
                and transformer.target_crs.is_geographic):
            # 投影座標をWGS84の座標に変換する。
            x_list = []
            y_list = []
            alt_list = []

            lon_list = []
            lat_list = []
            conv_alt_list = []

            for input_projected_point in projected_point_list:
                x_list.append(input_projected_point.x)
                y_list.append(input_projected_point.y)
                alt_list.append(input_projected_point.alt)
            lon_list, lat_list, conv_alt_list = transformer.transform(
                x_list, y_list,
                alt_list, errcheck=True)
            for index in range(len(lon_list)):
                point = Point(
                    lon_list[index], lat_list[index], conv_alt_list[index])
                point_list.append(point)
        else:
            # 投影座標以外が入力された場合はエラーとする。
            raise SpatialIdError("VALUE_CONVERT_ERROR")

        spatial_log.debug(("return value:", point_list),)
        return point_list
    except Exception:
        raise SpatialIdError("VALUE_CONVERT_ERROR")


def convert_point_list_to_projected_point_list(
        point_list: list[object],
        projected_crs: int, geographic_crs: int = CRS) -> list[object]:
    """地理座標系のリストをユーザ指定の投影座標のリストに変換する。

    :param point_list: 地理座標のデータクラスオブジェクトのリスト
    :type point_list: list[object]
    :param projected_crs: 出力の投影座標のCRSのEPSGコード
    :type projected_crs: int
    :param geographic_crs: 入力の地理座標のCRSのEPSGコード。指定がない場合はWGS84を使用する。
    :type geographic_crs: int
    :raises SpatialIdError: 入力されたEPSGコードが存在しない場合、エラー
    :raises SpatialIdError: 出力のEPSGコードが投影座標ではない場合、エラー
    :return: 投影座標のデータクラスオブジェクトのリストを返却する。
    :rtype: list[object]
    """
    spatial_log.debug(
        ("point_list:", point_list,
         "projected_crs:", projected_crs,
         "geographic_crs:", geographic_crs),)
    try:
        # 地理座標系を投影座標に変換するpyprojオブジェクトを作成する。
        transformer = get_transformer(geographic_crs, projected_crs)

        # 返却用ポイントリスト
        projection_point_list = []

        # 入出力の座標系の組み合わせが正しい場合、座標系を変換する。
        if (not transformer.target_crs.is_geographic
                and transformer.source_crs.is_geographic):
            # 地理座標を投影座標に変換する。
            lon_list = []
            lat_list = []
            conv_alt_list = []

            x_list = []
            y_list = []
            alt_list = []

            for input_point in point_list:
                lon_list.append(input_point.lon)
                lat_list.append(input_point.lat)
                alt_list.append(input_point.alt)
            x_list, y_list, conv_alt_list = transformer.transform(
                lon_list, lat_list,
                alt_list, errcheck=True)
            for x, y, alt, lon, lat in zip(
                    x_list, y_list, conv_alt_list, lon_list, lat_list):
                projected_point = Projected_Point(
                    x, y, alt, lon, lat)
                projection_point_list.append(projected_point)
        else:
            # 座標系の組み合わせが誤って入力された場合はエラーとする。
            raise SpatialIdError("VALUE_CONVERT_ERROR")

        spatial_log.debug(("return value:", projection_point_list),)
        return projection_point_list

    except Exception:
        raise SpatialIdError("VALUE_CONVERT_ERROR")


def check_zoom(zoom: int) -> bool:
    """入力の精度が0-35の範囲内か判定をする。

    :param zoom: 入力された精度
    :type zoom: int
    :return: 0-35の範囲内の場合True、それ以外の場合False
    :rtype: bool
    """

    # 精度が0-35におさまっている場合はTrue
    return 0 <= zoom and zoom <= 35

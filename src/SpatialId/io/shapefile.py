# Copyright © 2022 Digital Agency. All rights reserved.
"""
|  Requirements: Python 3.9+.

| シェイプファイル(.shp)の読み込みモジュール
"""

import shapefile as sf

from logging import getLogger
from typing import Any
from collections.abc import Generator, Callable

from shapefile import ShapefileException

from SpatialId import LOGGER_NAME
from SpatialId.common.exception import SpatialIdError
from SpatialId.common.object.enum import ShapeType, PartType
from SpatialId.common.object.point import Triangle as SpatialTriangle
from SpatialId.common.object.point import Projected_Triangle
from SpatialId.common.object.point import \
    Projected_Point as SpatialProjectedPoint
from SpatialId.shape.polygons import get_spatial_ids_on_polygons as get_spatial_ids_on_polygons
from SpatialId.shape import get_transformer
from SpatialId.convert import internal_to_out

# CRSのデフォルト値(WGS84)のEPSGコード
SPATIAL_ID_CRS = 4326

# 投影座標系のEPSGコード
PROJECTION_ID_CRS = 3857

# DEBUGログ用のロガーインスタンス
spatial_logger = getLogger(LOGGER_NAME).getChild(__name__)


def read_shapefile(
        filepath: str,
        encoding: str,
        h_zoom: int,
        v_zoom: int,
        crs: int = SPATIAL_ID_CRS,
        needs_closed_checking: bool = True
) -> Generator[tuple[list[str], list[list[Any]], list[Any]], None, None]:
    """
    | ファイルパスを指定してShapefileを読み込み、空間ID情報に変換する。
    | Shapefileから1レコード単位で情報を取得するジェネレータを返却する。
    |
    | Shapefileには格納されている座標情報の形状を示すShape種別という情報があり、
    | 本モジュールにおいては以下のShape種別の空間ID変換に対応する。

    =========== =============================================================
    Shape種別   概要
    =========== =============================================================
    MultiPatch  複数パッチ。複数の多角形を組み合わせて立体を表現する。

                MultiPatchを構成する部品のうち以下について対応する。

                * TRIANGLE_STRIPE:接続された一連の三角形。
                * TRIANGLE_FAN: 扇状の連結三角形。
    =========== =============================================================

    パラメータfilepathに指定するパスは、.shpファイル、.dbfファイル、
    Shapefileのベースネーム(/path/to/example.shpの場合/path/to/example)、
    Shapefile群を含んだzipファイルのパスを指定可能である。

    返却されるフィールド定義情報は、ペイロードの各要素のメタ情報を格納している。
    フィールド定義情報の要素数はペイロードの要素数と一致しており、
    フィールド定義情報とペイロードで同じインデックスの情報が対応している。

    フィールド定義情報の各要素は4つの要素の配列で、
    各要素に以下の情報が格納される。

    ============ ===========================================
    インデックス 内容
    ============ ===========================================
    0             フィールド名
    1             フィールド種別

                  * C:文字列
                  * N: 数値
                  * F: 浮動小数点数
                  * L: 真偽値
                  * D: 日付
                  * M: メモ
    2             フィールド長
    3             フィールド種別が数値の場合の小数部の桁数
    ============ ===========================================

    ジェネレータから返却される値の例を以下に示す。

    ::

        # 空間ID
        ["XXXXXXXX"]
        # フィールド定義情報
        [
            ["building_id", "N", 10, 0],
            ["building_name", "C", 20, 0]
        ]
        # ペイロード
        [100001, "ABCビル"]

    :param filepath: Shapefileのファイルパス
    :type filepath: str
    :param encoding: Shapefileの文字エンコーディング
    :type encoding: str
    :param h_zoom: 空間IDの水平方向精度レベル
    :type h_zoom: int
    :param v_zoom: 空間IDの垂直方向精度レベル
    :type v_zoom: int
    :param crs: Shapefileに格納されている座標の参照座標系。デフォルト値:4326(WGS84)
    :type crs: int
    :param needs_closed_checking: ポリゴンの空間ID取得時の閉塞チェックフラグ。Trueの場合はチェックを行う。
    :type needs_closed_checking: bool

    :returns:
        - 空間IDの配列、フィールド定義情報、ペイロードを返すジェネレータ
    :rtype: collections.abc.Generator

    :raise SpatialIdError: 入力チェック結果が不正の場合のエラー
    :raise SpatialIdError: 座標系の変換に失敗した場合
    :raise SpatialIdError: Shapefileの読み込みに失敗した場合
    :raise SpatialIdError: Shapefileにサポート対象外のShape種別が指定されていた場合
    :raise SpatialIdError: ShapefileにおけるPointsとZの要素数が一致しない場合
    :raise SpatialIdError: MultipatchのpartsとpartTypes要素数が異なっていた場合
    :raise SpatialIdError: ShapefileのShape種別がMultiPatchであり、
                           サポート対象外のpart種別が指定されていた場合
    :raise SpatialIdError: Shapefileの解析時に対象partTypeに必要な点の数が不足していた場合
    :raise SpatialIdError: ShapefileのpartType要素数が0の場合
    :raise SpatialIdError: Shapefileのparts要素数が0の場合
    :raise SpatialIdError: ポリゴンの空間ID取得時にエラーが発生した場合
    """
    # shapefile.Reader オブジェクト作成
    try:
        # 入力引数チェック
        _validate_args_type(
                filepath, encoding, h_zoom, v_zoom, crs, needs_closed_checking)

        with sf.Reader(filepath, encoding=encoding) as sf_reader:
            # フィールド定義情報を取得
            fields = _get_fields_data(sf_reader)

            # shapefile の空間ID, ペイロード情報を取得
            spatial_id_and_record = \
                _get_shapefile_spatial_id_and_record(
                        sf_reader,
                        h_zoom,
                        v_zoom,
                        crs,
                        needs_closed_checking)

            for spatial_ids, payloads in spatial_id_and_record:
                yield spatial_ids, fields, payloads

    except (ShapefileException, UnicodeDecodeError, LookupError) as e:
        # Shapefile読み込みに失敗した場合例外発生
        raise SpatialIdError('SHAPEFILE_READ_ERROR') from e

    except SpatialIdError:
        # 内部処理で検出済みの例外を再送
        raise

    except Exception as e:
        # その他例外を投げる
        raise SpatialIdError("OTHER_ERROR") from e

def f_read_shapefile(
        filepath: str,
        encoding: str,
        zoom: int,
        crs: int = SPATIAL_ID_CRS,
        needs_closed_checking: bool = True
) -> Generator[tuple[list[str], list[list[Any]], list[Any]], None, None]:
    """
    | ファイルパスを指定してShapefileを読み込み、空間ID情報に変換する。
    | Shapefileから1レコード単位で情報を取得するジェネレータを返却する。
    |
    | Shapefileには格納されている座標情報の形状を示すShape種別という情報があり、
    | 本モジュールにおいては以下のShape種別の空間ID変換に対応する。

    =========== =============================================================
    Shape種別   概要
    =========== =============================================================
    MultiPatch  複数パッチ。複数の多角形を組み合わせて立体を表現する。

                MultiPatchを構成する部品のうち以下について対応する。

                * TRIANGLE_STRIPE:接続された一連の三角形。
                * TRIANGLE_FAN: 扇状の連結三角形。
    =========== =============================================================

    パラメータfilepathに指定するパスは、.shpファイル、.dbfファイル、
    Shapefileのベースネーム(/path/to/example.shpの場合/path/to/example)、
    Shapefile群を含んだzipファイルのパスを指定可能である。

    返却されるフィールド定義情報は、ペイロードの各要素のメタ情報を格納している。
    フィールド定義情報の要素数はペイロードの要素数と一致しており、
    フィールド定義情報とペイロードで同じインデックスの情報が対応している。

    フィールド定義情報の各要素は4つの要素の配列で、
    各要素に以下の情報が格納される。

    ============ ===========================================
    インデックス 内容
    ============ ===========================================
    0             フィールド名
    1             フィールド種別

                  * C:文字列
                  * N: 数値
                  * F: 浮動小数点数
                  * L: 真偽値
                  * D: 日付
                  * M: メモ
    2             フィールド長
    3             フィールド種別が数値の場合の小数部の桁数
    ============ ===========================================

    | f_"がついてないAPIとの違いは以下：
    | 入力の空間ID精度レベルを水平・垂直方向共に統一
    | 出力の空間IDの並びは仕様通りの記載。
    | z/f/x/y :z{ズームレベル}/f{高さの位置}/x{経度方向の位置}/y{緯度方向の位置}
    | 参照：https://github.com/unvt/zfxy-spec
    
    ジェネレータから返却される値の例を以下に示す。

    ::

        # 空間ID
        ["XXXXXXXX"]
        # フィールド定義情報
        [
            ["building_id", "N", 10, 0],
            ["building_name", "C", 20, 0]
        ]
        # ペイロード
        [100001, "ABCビル"]

    :param filepath: Shapefileのファイルパス
    :type filepath: str
    :param encoding: Shapefileの文字エンコーディング
    :type encoding: str
    :param zoom: 空間IDの精度レベル
    :type zoom: int
    :param crs: Shapefileに格納されている座標の参照座標系。デフォルト値:4326(WGS84)
    :type crs: int
    :param needs_closed_checking: ポリゴンの空間ID取得時の閉塞チェックフラグ。Trueの場合はチェックを行う。
    :type needs_closed_checking: bool

    :returns:
        - 空間IDの配列、フィールド定義情報、ペイロードを返すジェネレータ
    :rtype: collections.abc.Generator

    :raise SpatialIdError: 入力チェック結果が不正の場合のエラー
    :raise SpatialIdError: 座標系の変換に失敗した場合
    :raise SpatialIdError: Shapefileの読み込みに失敗した場合
    :raise SpatialIdError: Shapefileにサポート対象外のShape種別が指定されていた場合
    :raise SpatialIdError: ShapefileにおけるPointsとZの要素数が一致しない場合
    :raise SpatialIdError: MultipatchのpartsとpartTypes要素数が異なっていた場合
    :raise SpatialIdError: ShapefileのShape種別がMultiPatchであり、
                           サポート対象外のpart種別が指定されていた場合
    :raise SpatialIdError: Shapefileの解析時に対象partTypeに必要な点の数が不足していた場合
    :raise SpatialIdError: ShapefileのpartType要素数が0の場合
    :raise SpatialIdError: Shapefileのparts要素数が0の場合
    :raise SpatialIdError: ポリゴンの空間ID取得時にエラーが発生した場合
    """
    # shapefile.Reader オブジェクト作成
    try:
        # 入力引数チェック
        _validate_args_type(
                filepath, encoding, zoom, zoom, crs, needs_closed_checking)

        with sf.Reader(filepath, encoding=encoding) as sf_reader:
            # フィールド定義情報を取得
            fields = _get_fields_data(sf_reader)

            # shapefile の空間ID, ペイロード情報を取得
            spatial_id_and_record = \
                _get_shapefile_spatial_id_and_record(
                        sf_reader,
                        zoom,
                        zoom,
                        crs,
                        needs_closed_checking)

            for spatial_ids, payloads in spatial_id_and_record:
                yield internal_to_out(spatial_ids), fields, payloads

    except (ShapefileException, UnicodeDecodeError, LookupError) as e:
        # Shapefile読み込みに失敗した場合例外発生
        raise SpatialIdError('SHAPEFILE_READ_ERROR') from e

    except SpatialIdError:
        # 内部処理で検出済みの例外を再送
        raise

    except Exception as e:
        # その他例外を投げる
        raise SpatialIdError("OTHER_ERROR") from e


def read_shapefile_bulk(
        filepath: str,
        encoding: str,
        h_zoom: int,
        v_zoom: int,
        crs: int = SPATIAL_ID_CRS,
        needs_closed_checking: bool = True
) -> tuple[list[list[str]], list[list[Any]], list[list[Any]]]:
    """
    | ファイルパスを指定してShapefileを読み込み、空間ID情報に変換する。
    | Shapefileの情報を一括で取得する。

    返却される空間IDの配列とペイロードの配列の要素数は一致し、
    同じインデックスの情報が同じレコードの情報である。

    返却される値の例を以下に示す。

    ::

        # 空間ID配列
        [
            ["XXXXX"], # レコード#0の空間ID
            ["XXXXX", "XXXXY"], # レコード#1の空間ID
            ...
            ["XXXXZ"] # レコード#Nの空間ID
        ]
        # フィールド定義情報
        [
            ["building_id", "N", 10, 0],
            ["building_name", "C", 20, 0]
        ]
        # ペイロード配列
        [
            [100001, "Aビル"], # レコード#0の情報
            [100002, "Bビル"], # レコード#1の情報
            ...
            [100050, "XXビル"], # レコード#Nの情報
        ]

    :param filepath: Shapefileのファイルパス
    :type filepath: str
    :param encoding: Shapefileの文字エンコーディング
    :type encoding: str
    :param h_zoom: 空間IDの水平方向精度レベル
    :type h_zoom: int
    :param v_zoom: 空間IDの垂直方向精度レベル
    :type v_zoom: int
    :param crs: Shapefileに格納されている座標の参照座標系。デフォルト値:4326(WGS84)
    :type crs: int
    :param needs_closed_checking: ポリゴンの空間ID取得時の閉塞チェックフラグ。Trueの場合はチェックを行う。
    :type needs_closed_checking: bool

    :returns:
        - 空間IDの配列
        - フィールド定義情報
        - ペイロードの配列
    :rtyep: tuple, list, list

    :raise SpatialIdError: 入力チェック結果が不正の場合のエラー
    :raise SpatialIdError: 座標系の変換に失敗した場合
    :raise SpatialIdError: Shapefileの読み込みに失敗した場合
    :raise SpatialIdError: Shapefileにサポート対象外のShape種別が指定されていた場合
    :raise SpatialIdError: ShapefileにおけるPointsとZの要素数が一致しない場合
    :raise SpatialIdError: MultipatchのpartsとpartTypes要素数が異なっていた場合
    :raise SpatialIdError: ShapefileのShape種別がMultiPatchであり、
                           サポート対象外のpart種別が指定されていた場合
    :raise SpatialIdError: Shapefileの解析時に対象partTypeに必要な点の数が不足していた場合
    :raise SpatialIdError: ShapefileのpartType要素数が0の場合
    :raise SpatialIdError: Shapefileのparts要素数が0の場合
    :raise SpatialIdError: ポリゴンの空間ID取得時にエラーが発生した場合
    """
    # shapefile.Reader オブジェクト作成
    try:
        # 入力引数チェック
        _validate_args_type(filepath, encoding, h_zoom, v_zoom, crs)

        with sf.Reader(filepath, encoding=encoding) as sf_reader:
            # 空間ID格納用
            spatial_ids_list = []

            # ペイロード格納用
            payloads_list = []

            # フィールド定義情報を取得
            fields = _get_fields_data(sf_reader)

            # shapefile の空間ID, ペイロードを取得するGenetator格納
            spatial_ids_and_payloads = \
                _get_shapefile_spatial_id_and_record(
                        sf_reader,
                        h_zoom,
                        v_zoom,
                        crs,
                        needs_closed_checking)

            for spatial_ids, payloads in spatial_ids_and_payloads:
                # 空間ID、ペイロードを格納
                spatial_ids_list.append(spatial_ids)
                payloads_list.append(payloads)

            # 戻り値を格納
            return (
                    spatial_ids_list,
                    fields,
                    payloads_list)

    except (ShapefileException, UnicodeDecodeError, LookupError) as e:
        # Shapefile読み込みに失敗した場合例外発生
        raise SpatialIdError('SHAPEFILE_READ_ERROR') from e

    except SpatialIdError:
        # 内部処理で検出済みの例外を再送
        raise

    except Exception as e:
        # その他例外を投げる
        raise SpatialIdError("OTHER_ERROR") from e

def f_read_shapefile_bulk(
        filepath: str,
        encoding: str,
        zoom: int,
        crs: int = SPATIAL_ID_CRS,
        needs_closed_checking: bool = True
) -> tuple[list[list[str]], list[list[Any]], list[list[Any]]]:
    """
    | ファイルパスを指定してShapefileを読み込み、空間ID情報に変換する。
    | Shapefileの情報を一括で取得する。

    | f_"がついてないAPIとの違いは以下：
    | 入力の空間ID精度レベルを水平・垂直方向共に統一
    | 出力の空間IDの並びは仕様通りの記載。
    | z/f/x/y :z{ズームレベル}/f{高さの位置}/x{経度方向の位置}/y{緯度方向の位置}
    | 参照：https://github.com/unvt/zfxy-spec

    返却される空間IDの配列とペイロードの配列の要素数は一致し、
    同じインデックスの情報が同じレコードの情報である。

    返却される値の例を以下に示す。

    ::

        # 空間ID配列
        [
            ["XXXXX"], # レコード#0の空間ID
            ["XXXXX", "XXXXY"], # レコード#1の空間ID
            ...
            ["XXXXZ"] # レコード#Nの空間ID
        ]
        # フィールド定義情報
        [
            ["building_id", "N", 10, 0],
            ["building_name", "C", 20, 0]
        ]
        # ペイロード配列
        [
            [100001, "Aビル"], # レコード#0の情報
            [100002, "Bビル"], # レコード#1の情報
            ...
            [100050, "XXビル"], # レコード#Nの情報
        ]

    :param filepath: Shapefileのファイルパス
    :type filepath: str
    :param encoding: Shapefileの文字エンコーディング
    :type encoding: str
    :param zoom: 空間IDの精度レベル
    :type zoom: int
    :param crs: Shapefileに格納されている座標の参照座標系。デフォルト値:4326(WGS84)
    :type crs: int
    :param needs_closed_checking: ポリゴンの空間ID取得時の閉塞チェックフラグ。Trueの場合はチェックを行う。
    :type needs_closed_checking: bool

    :returns:
        - 空間IDの配列
        - フィールド定義情報
        - ペイロードの配列
    :rtyep: tuple, list, list

    :raise SpatialIdError: 入力チェック結果が不正の場合のエラー
    :raise SpatialIdError: 座標系の変換に失敗した場合
    :raise SpatialIdError: Shapefileの読み込みに失敗した場合
    :raise SpatialIdError: Shapefileにサポート対象外のShape種別が指定されていた場合
    :raise SpatialIdError: ShapefileにおけるPointsとZの要素数が一致しない場合
    :raise SpatialIdError: MultipatchのpartsとpartTypes要素数が異なっていた場合
    :raise SpatialIdError: ShapefileのShape種別がMultiPatchであり、
                           サポート対象外のpart種別が指定されていた場合
    :raise SpatialIdError: Shapefileの解析時に対象partTypeに必要な点の数が不足していた場合
    :raise SpatialIdError: ShapefileのpartType要素数が0の場合
    :raise SpatialIdError: Shapefileのparts要素数が0の場合
    :raise SpatialIdError: ポリゴンの空間ID取得時にエラーが発生した場合
    """
    # shapefile.Reader オブジェクト作成
    try:
        # 入力引数チェック
        _validate_args_type(filepath, encoding, zoom, zoom, crs)

        with sf.Reader(filepath, encoding=encoding) as sf_reader:
            # 空間ID格納用
            spatial_ids_list = []

            # ペイロード格納用
            payloads_list = []

            # フィールド定義情報を取得
            fields = _get_fields_data(sf_reader)

            # shapefile の空間ID, ペイロードを取得するGenetator格納
            spatial_ids_and_payloads = \
                _get_shapefile_spatial_id_and_record(
                        sf_reader,
                        zoom,
                        zoom,
                        crs,
                        needs_closed_checking)

            for spatial_ids, payloads in spatial_ids_and_payloads:
                # 空間ID、ペイロードを格納
                spatial_ids_list.append(internal_to_out(spatial_ids))
                payloads_list.append(payloads)

            # 戻り値を格納
            return (
                    spatial_ids_list,
                    fields,
                    payloads_list)

    except (ShapefileException, UnicodeDecodeError, LookupError) as e:
        # Shapefile読み込みに失敗した場合例外発生
        raise SpatialIdError('SHAPEFILE_READ_ERROR') from e

    except SpatialIdError:
        # 内部処理で検出済みの例外を再送
        raise

    except Exception as e:
        # その他例外を投げる
        raise SpatialIdError("OTHER_ERROR") from e


def read_shapefile_bulk_with_aggregation(
        filepath: str,
        encoding: str,
        h_zoom: int,
        v_zoom: int,
        crs: int = SPATIAL_ID_CRS,
        agg_func: Callable[[list[list[Any]]], list[Any]] = None,
        needs_closed_checking: bool = True
) -> tuple[list[list[str]], list[list[Any]], dict[str, list[list[Any]]]]:
    """
    | ファイルパスを指定してShapefileを読み込み、空間ID情報に変換する。
    | Shapefileの情報を一括で取得する。

    | ペイロードについては空間IDをキー、値をペイロード配列の配列とする辞書型として返却する。
    | 集約関数が指定された場合は、空間ID毎のペイロード配列の配列が集約関数に入力され、
    | 集約関数の出力を当該空間IDに対する値とする。

    返却される値の例を以下に示す。

    ::

        # 空間ID配列
        [
            ["XXXXX"], # レコード#0の空間ID
            ["XXXXX", "XXXXY"], # レコード#1の空間ID
            ...
            ["XXXXZ"] # レコード#Nの空間ID
        ]
        # フィールド定義情報
        [
            ["building_id", "N", 10, 0],
            ["building_name", "C", 20, 0]
        ]
        # ペイロード
        {
            "XXXXX": [
                [100001, "Aビル"],
                [100002, "Bビル"]
            ],
            "XXXXY": [
                [100002, "Bビル"]
            ],
            ...
            "XXXXZ": [
                [100002, "Bビル"]
            ]
        }

    :param filepath: Shapefileのファイルパス
    :type filepath: str
    :param encoding: Shapefileの文字エンコーディング
    :type encoding: str
    :param h_zoom: 空間IDの水平方向精度レベル
    :type h_zoom: int
    :param v_zoom: 空間IDの垂直方向精度レベル
    :type v_zoom: int
    :param crs: Shapefileに格納されている座標の参照座標系。デフォルト値:4326(WGS84)
    :type crs: int
    :param agg_func: 集約関数
    :type agg_func: Callable
    :param needs_closed_checking: ポリゴンの空間ID取得時の閉塞チェックフラグ。Trueの場合はチェックを行う。
    :type needs_closed_checking: bool

    :returns:
        - 空間IDの配列
        - フィールド定義情報
        - ペイロード
    :rtype: tuple, list, dict

    :raise SpatialIdError: 入力チェック結果が不正の場合のエラー
    :raise SpatialIdError: 座標系の変換に失敗した場合
    :raise SpatialIdError: Shapefileの読み込みに失敗した場合
    :raise SpatialIdError: Shapefileにサポート対象外のShape種別が指定されていた場合
    :raise SpatialIdError: ShapefileにおけるPointsとZの要素数が一致しない場合
    :raise SpatialIdError: MultipatchのpartsとpartTypes要素数が異なっていた場合
    :raise SpatialIdError: ShapefileのShape種別がMultiPatchであり、
                           サポート対象外のpart種別が指定されていた場合
    :raise SpatialIdError: Shapefileの解析時に対象partTypeに必要な点の数が不足していた場合
    :raise SpatialIdError: ShapefileのpartType要素数が0の場合
    :raise SpatialIdError: Shapefileのparts要素数が0の場合
    :raise SpatialIdError: 集約関数の実行に失敗した場合
    :raise SpatialIdError: ポリゴンの空間ID取得時にエラーが発生した場合
    """
    try:
        # 入力引数チェック
        _validate_args_type(filepath, encoding, h_zoom, v_zoom, crs)

        # 空間ID、フィールド、ペイロードを取得
        spatial_ids_list, fields, payloads_list = \
            read_shapefile_bulk(
                    filepath, encoding,
                    h_zoom,
                    v_zoom,
                    crs,
                    needs_closed_checking)

        # 成形後のpayloads格納用
        payload_datas = {}

        # 空間IDをキー、値をペイロードとする辞書型に成形payload_datas
        for spatial_ids, payloads in zip(spatial_ids_list, payloads_list):

            for sp_id in spatial_ids:

                # 空間IDをキー値として未定義の場合、初期化
                if sp_id not in payload_datas:
                    payload_datas[sp_id] = []

                # 辞書に、空間IDをキー、値をペイロードとして格納
                payload_datas[sp_id].append(payloads)
        try:
            # 集約関数が指定されている場合、辞書データを渡す
            if agg_func is not None:
                spatial_logger.debug("集約関数の実行を開始")
                return (spatial_ids_list, fields, agg_func(payload_datas))
            else:
                return (spatial_ids_list, fields, payload_datas)

        except Exception as e:
            # 集約関数実行中に発生した例外を再送
            raise SpatialIdError("SHAPEFILE_AGGREGATION_FUNC_ERROR") from e

    except SpatialIdError:
        # 内部処理で検出済みの例外を再送
        raise

    except Exception as e:
        # その他例外を投げる
        raise SpatialIdError("OTHER_ERROR") from e


def f_read_shapefile_bulk_with_aggregation(
        filepath: str,
        encoding: str,
        zoom: int,
        crs: int = SPATIAL_ID_CRS,
        agg_func: Callable[[list[list[Any]]], list[Any]] = None,
        needs_closed_checking: bool = True
) -> tuple[list[list[str]], list[list[Any]], dict[str, list[list[Any]]]]:
    """
    | ファイルパスを指定してShapefileを読み込み、空間ID情報に変換する。
    | Shapefileの情報を一括で取得する。

    | ペイロードについては空間IDをキー、値をペイロード配列の配列とする辞書型として返却する。
    | 集約関数が指定された場合は、空間ID毎のペイロード配列の配列が集約関数に入力され、
    | 集約関数の出力を当該空間IDに対する値とする。

    | f_"がついてないAPIとの違いは以下：
    | 入力の空間ID精度レベルを水平・垂直方向共に統一
    | 出力の空間IDの並びは仕様通りの記載。
    | z/f/x/y :z{ズームレベル}/f{高さの位置}/x{経度方向の位置}/y{緯度方向の位置}
    | 参照：https://github.com/unvt/zfxy-spec

    返却される値の例を以下に示す。

    ::

        # 空間ID配列
        [
            ["XXXXX"], # レコード#0の空間ID
            ["XXXXX", "XXXXY"], # レコード#1の空間ID
            ...
            ["XXXXZ"] # レコード#Nの空間ID
        ]
        # フィールド定義情報
        [
            ["building_id", "N", 10, 0],
            ["building_name", "C", 20, 0]
        ]
        # ペイロード
        {
            "XXXXX": [
                [100001, "Aビル"],
                [100002, "Bビル"]
            ],
            "XXXXY": [
                [100002, "Bビル"]
            ],
            ...
            "XXXXZ": [
                [100002, "Bビル"]
            ]
        }

    :param filepath: Shapefileのファイルパス
    :type filepath: str
    :param encoding: Shapefileの文字エンコーディング
    :type encoding: str
    :param zoom: 空間IDの精度レベル
    :type zoom: int
    :param crs: Shapefileに格納されている座標の参照座標系。デフォルト値:4326(WGS84)
    :type crs: int
    :param agg_func: 集約関数
    :type agg_func: Callable
    :param needs_closed_checking: ポリゴンの空間ID取得時の閉塞チェックフラグ。Trueの場合はチェックを行う。
    :type needs_closed_checking: bool

    :returns:
        - 空間IDの配列
        - フィールド定義情報
        - ペイロード
    :rtype: tuple, list, dict

    :raise SpatialIdError: 入力チェック結果が不正の場合のエラー
    :raise SpatialIdError: 座標系の変換に失敗した場合
    :raise SpatialIdError: Shapefileの読み込みに失敗した場合
    :raise SpatialIdError: Shapefileにサポート対象外のShape種別が指定されていた場合
    :raise SpatialIdError: ShapefileにおけるPointsとZの要素数が一致しない場合
    :raise SpatialIdError: MultipatchのpartsとpartTypes要素数が異なっていた場合
    :raise SpatialIdError: ShapefileのShape種別がMultiPatchであり、
                           サポート対象外のpart種別が指定されていた場合
    :raise SpatialIdError: Shapefileの解析時に対象partTypeに必要な点の数が不足していた場合
    :raise SpatialIdError: ShapefileのpartType要素数が0の場合
    :raise SpatialIdError: Shapefileのparts要素数が0の場合
    :raise SpatialIdError: 集約関数の実行に失敗した場合
    :raise SpatialIdError: ポリゴンの空間ID取得時にエラーが発生した場合
    """
    try:
        # 入力引数チェック
        _validate_args_type(filepath, encoding, zoom, zoom, crs)

        # 空間ID、フィールド、ペイロードを取得
        spatial_ids_list, fields, payloads_list = \
            f_read_shapefile_bulk(
                    filepath, encoding,
                    zoom,
                    crs,
                    needs_closed_checking)

        # 成形後のpayloads格納用
        payload_datas = {}

        # 空間IDをキー、値をペイロードとする辞書型に成形payload_datas
        for spatial_ids, payloads in zip(spatial_ids_list, payloads_list):

            for sp_id in spatial_ids:

                # 空間IDをキー値として未定義の場合、初期化
                if sp_id not in payload_datas:
                    payload_datas[sp_id] = []

                # 辞書に、空間IDをキー、値をペイロードとして格納
                payload_datas[sp_id].append(payloads)
        try:
            # 集約関数が指定されている場合、辞書データを渡す
            if agg_func is not None:
                spatial_logger.debug("集約関数の実行を開始")
                return (spatial_ids_list, fields, agg_func(payload_datas))
            else:
                return (spatial_ids_list, fields, payload_datas)

        except Exception as e:
            # 集約関数実行中に発生した例外を再送
            raise SpatialIdError("SHAPEFILE_AGGREGATION_FUNC_ERROR") from e

    except SpatialIdError:
        # 内部処理で検出済みの例外を再送
        raise

    except Exception as e:
        # その他例外を投げる
        raise SpatialIdError("OTHER_ERROR") from e


def _get_fields_data(shapefile_reader: sf.Reader) -> list[list[Any]]:
    """
    |  Shapefileのfieldsに格納されている情報を返却する。
    |  fieldsの1要素目は常に"DeletionFlag"が設定されており、
    |  Shapefileでは一般的に利用されないフィールドであるため、
    |  fieldsの2要素目以降を返却する。

    :param shapefile_reader: 読み込み対象のshapefile.Readerインスタンス
    :type shapefile_reader: shapefile.Reader

    :returns: Shapefileに格納されているfieldsのデータ
    :rtype: list
    """
    fields = shapefile_reader.fields[1:]
    spatial_logger.debug("fields=%s", fields)

    # フィールドの配列を返却
    return fields


def _get_shapefile_spatial_id_and_record(
        shapefile_reader: sf.Reader,
        h_zoom: int,
        v_zoom: int,
        crs: int,
        needs_closed_checking: bool = True
) -> Generator[tuple[list[list[str]], list[list[Any]]]]:
    """
    |  shapefileに格納されている図形の座標を空間IDに変換した結果及び、
    |  ShapeRecords.recordの情報を1レコード単位で取得するジェネレータを返却する。

    |  返却される空間IDの配列とrecordの配列の要素数は一致し、
    |  同じインデックスの情報が同じレコードの情報となる。

    :param shapefile_reader: 読み込み対象のshapefile.Readerインスタンス
    :type shapefile_reader: shapefile.Reader
    :param h_zoom: 空間IDの水平方向精度レベル
    :type h_zoom: int
    :param v_zoom: 空間IDの垂直方向精度レベル
    :type v_zoom: int
    :param crs: Shapefileに格納されている座標の参照座標系。
    :type crs: int
    :param needs_closed_checking: ポリゴンの空間ID取得時の閉塞チェックフラグ。Trueの場合はチェックを行う。
    :type needs_closed_checking: bool

    :returns:
        - 空間IDの配列、recordの配列を返すジェネレータ
    :rtype: collections.abc.Generator

    :raise SpatialIdError: ShapefileのShape種別がサポート対象外の場合
    :raise SpatialIdError: MultipatchのpartsとpartTypes要素数が異なっていた場合
    :raise SpatialIdError: ポリゴンの空間ID取得時にエラーが発生した場合
    """
    # 戻り値の空間ID、record格納用変数
    spatial_ids = []
    payloads = []

    # shapefile データの配列取得
    shp_data_itre = shapefile_reader.iterShapeRecords()

    # shapefile 全レコード読み込み
    for geo_datas in shp_data_itre:
        spatial_logger.debug("1レコード読み込み開始")
        spatial_logger.debug("%s", geo_datas.shape)
        spatial_logger.debug("%s", geo_datas.record)

        # 参照中のshape, records取得
        shape_datas = geo_datas.shape
        records_data = geo_datas.record

        # レコードのshapeType 取得
        shapeType = shape_datas.shapeType

        # shapeTypeによって処理を分岐
        if shapeType == ShapeType.MULTIPATCH.value:
            # MULTIPATCH座標情報を取得
            points = _combine_x_y_z_point(
                    shape_datas.points,
                    shape_datas.z,
                    crs)

            # 部品のインデックス、種類を取得
            parts = shape_datas.parts
            partTypes = shape_datas.partTypes

            spatial_logger.debug("points=%s", points)
            spatial_logger.debug("parts=%s", shape_datas.parts)
            spatial_logger.debug("partTypes=%s", shape_datas.partTypes)

            if len(parts) != len(partTypes):
                # partsとpartTypesの要素数が異なっていた場合例外発生
                raise SpatialIdError('SHAPEFILE_PARTS_NUM_ERROR')

            # MULTIPATCHの場合
            triangle_obj_list = _get_multipatch_data(
                                        points,
                                        parts,
                                        partTypes)

            # 座標リストを空間IDリストに変換
            spatial_ids = get_spatial_ids_on_polygons(
                    triangle_obj_list,
                    [],
                    h_zoom,
                    v_zoom,
                    crs=PROJECTION_ID_CRS,
                    needs_closed_checking=needs_closed_checking)
        else:
            # サポート対象外のshapeTypeだった場合例外発生
            raise SpatialIdError('SHAPEFILE_SHAPETYPE_ERROR')

        # records のデータを戻り値payloadsに格納
        payloads = list(records_data.as_dict().values())

        spatial_logger.debug("spatial_ids=%s", spatial_ids)
        spatial_logger.debug("payloads=%s", payloads)

        spatial_logger.debug("1レコード読み込み終了")
        yield (spatial_ids, payloads)


def _get_multipatch_data(
        points: list,
        parts: list,
        part_types: list,
) -> list[SpatialTriangle]:
    """
    |  shapeTypeがMultipatchのレコード情報取得用関数
    |  pointsに格納されている座標情報をpartTypeの設定値に合わせて成形し、
    |  その座標情報から作成されたTriangleの配列を返却する。

    :param points: shapefileのpoints, z の座標情報を格納した配列
    :type points: list[tuple[float, float, float]]
    :param parts: shapefileのpartsを格納した配列
    :type parts: list[int]
    :param part_types: shapefileのpartTypeを格納した配列
    :type part_types: list[int]

    :returns: Triangleオブジェクトを格納した配列
    :rtype: list

    :raise SpatialIdError: ShapefileのpartType要素数が0の場合
    :raise SpatialIdError: Shapefileのparts要素数が0の場合
    """
    # 戻り値格納用変数
    result = []

    # 入力値チェック
    if len(parts) == 0:
        raise SpatialIdError('SHAPEFILE_PARTS_LIST_ERROR')
    elif len(part_types) == 0:
        raise SpatialIdError('SHAPEFILE_PART_TYPE_LIST_ERROR')

    # 全パーツの情報を読み込み、partTypeに対応した座標情報を取得
    for cnt in range(len(parts)):
        spatial_logger.debug("1partの読み込み開始")
        # 解析中のパーツ情報格納用
        target_points = []
        target_partType = part_types[cnt]

        # 次のパーツインデックスが存在しない場合
        if cnt == (len(parts) - 1):
            # 開始位置を格納
            start_index = parts[cnt]

            # パーツの座標を格納
            target_points = points[start_index:]

        # 次のパーツインデックスが存在する場合
        else:
            # 開始位置、終了位置を格納
            start_index = parts[cnt]
            end_index = parts[cnt + 1]

            # パーツの座標を格納
            target_points = points[start_index:end_index]

        spatial_logger.debug("target_part=%s", start_index)
        spatial_logger.debug("target_points=%s", target_points)

        # パーツの座標を成形
        parts_list = _shaping_multipatch_parts(
                            target_points,
                            target_partType)

        spatial_logger.debug("parts_list=%s", parts_list)

        # Triangleオブジェクト取得
        triangle_obj_list = _get_triangle_objects(parts_list)

        # 戻り値に格納
        result.extend(triangle_obj_list)
        spatial_logger.debug("1partの読み込み終了")

    return result


def _combine_x_y_z_point(
        x_y_points: list,
        input_z_points: list,
        crs: int
) -> list[list[tuple[float]]]:
    """
    | x, y, z 座標を組み合わせたpointデータを返却する。

    :param x_y_points: x, y 座標を格納した配列
    :type x_y_points: list[tuple[float, float]]
    :param z_points: z 座標を格納した配列
    :type z_points: list
    :param crs: Shapefileに格納されている座標の参照座標系。
    :type crs: int

    :return: x, y, z 座標を格納したオブジェクトの配列
    :rtype: list

    :raise SpatialIdError: ShapefileにおけるPointsとZの要素数が一致しない場合
    """
    points = []

    if len(x_y_points) != len(input_z_points):
        # pointsとzの要素数が一致しない場合例外発生
        raise SpatialIdError('SHAPEFILE_POINTS_NUM_ERROR')

    x_points = []
    y_points = []
    z_points = []
    for x_y, z in zip(x_y_points, input_z_points):
        # 入力座標系のまま格納
        x_points.append(x_y[0])
        y_points.append(x_y[1])
        z_points.append(z)

    try:
        converted = get_transformer(crs, PROJECTION_ID_CRS).transform(
            x_points, y_points, z_points, errcheck=True)
        converted_to_geographic = get_transformer(
            crs, SPATIAL_ID_CRS).transform(
            x_points, y_points, z_points, errcheck=True
            )
    except Exception:
        raise SpatialIdError("VALUE_CONVERT_ERROR")

    for x, y, z, lon, lat in zip(
            converted[0], converted[1], converted[2],
            converted_to_geographic[0], converted_to_geographic[1]):
        points.append(SpatialProjectedPoint(x, y, z, lon, lat))

    return points


def _shaping_multipatch_parts(
        part_points: list,
        part_type: int
) -> list[list[SpatialProjectedPoint]]:
    """
    |  引数で渡された座標を、partTypeに対応する図形座標に成形して返却する。

    :param part_points: 面の座標を格納した配列
    :type part_points: list[tuple[float, float, float]]
    :param part_type: 面のpartType
    :type part_type: int

    :return: partTypeに合わせて成形された面座標を格納した配列
    :rtype: list

    :raise SpatialIdError: サポート対象外のpart種別が指定されていた場合
    """
    # サポート対象のpartTypeを取得
    if part_type == PartType.TRIANGLE_STRIP.value:
        spatial_logger.debug(
            "target_partType=%s",
            PartType.TRIANGLE_STRIP.name
        )
        # TRIANGLE_STRIPの場合
        return _shaping_triangle_strip(part_points)

    elif part_type == PartType.TRIANGLE_FAN.value:
        spatial_logger.debug(
            "target_partType=%s",
            PartType.TRIANGLE_FAN.name
        )
        # TRIANGLE_FANの場合
        return _shaping_triangle_fan(part_points)

    else:
        # サポート対象外のpartTypeだった場合例外発生
        raise SpatialIdError('SHAPEFILE_PARTETYPE_ERROR')


def _shaping_triangle_strip(part_points: list) -> list[list[SpatialProjectedPoint]]:
    """
    |  引数で渡された座標をTRIANGLE_STRIPの形式に成形した結果を返却する。

    :param part_points: 面の座標を格納した配列
    :type part_points: list[tuple[float, float, float]]

    :return: TRIANGLE_STRIPの座標情報
    :rtype: list

    :raise SpatialIdError: partTypeに必要な点の数が不足している場合
    """
    # 点の数が足りているかをチェック
    if len(part_points) < 3:
        # shapefileの解析時に対象partTypeに必要な点の数が不足していた場合のエラー
        raise SpatialIdError('SHAPEFILE_PARTS_POINTS_NUM_ERROR')

    # 戻り値格納用
    tri_strip_points = []

    # 作成するTRIANGLE_STRIPの要素数をループ回数として取得
    loop_limit = len(part_points) - 2

    for cnt in range(loop_limit):
        tri_strip_points.append(
                [part_points[cnt], part_points[cnt + 1], part_points[cnt + 2]])

    return tri_strip_points


def _shaping_triangle_fan(part_points: list) -> list[list[SpatialProjectedPoint]]:
    """
    |  引数で渡された座標をTRIANGLE_FANの形式に成形した結果を返却する。

    :param part_points: 面の座標を格納した配列
    :type part_points: list[tuple[float, float, float]]

    :return: TRIANGLE_FANの座標情報
    :rtype: list

    :raise SpatialIdError: partTypeに必要な点の数が不足している場合
    """
    # 点の数が足りているかをチェック
    if len(part_points) < 3:
        # shapefileの解析時に対象partTypeに必要な点の数が不足していた場合のエラー
        raise SpatialIdError('SHAPEFILE_PARTS_POINTS_NUM_ERROR')

    # 戻り値格納用
    tri_fan_points = []

    # 作成するTRIANGLE_FANの要素数をループ回数として取得
    loop_limit = len(part_points) - 2

    # TRIANGLE_STRIPの始点座標格納
    start_point = part_points[0]
    for cnt in range(loop_limit):
        tri_fan_points.append(
                [start_point, part_points[cnt + 1], part_points[cnt + 2]])

    return tri_fan_points


def _get_triangle_objects(
    tri_parts_list: list,
) -> list[Projected_Triangle]:
    """
    |  座標情報を元にTriangleオブジェクトを作成し返却する。

    :param tri_parts_list: 投影座標の三角形ポリゴンの座標を格納した配列
    :type tri_parts_list: list[list[tuple[float, float, float]]]

    :return: Projected_Triangleオブジェクトを格納した配列
    :rtype: list
    """
    # 戻り値格納用
    result = []

    for tri in tri_parts_list:
        # Projected_Triangleオブジェクト格納
        result.append(Projected_Triangle(tri[0], tri[1], tri[2]))

    return result


def _validate_args_type(
    filepath: str,
    encoding: str,
    h_zoom: int,
    v_zoom: int,
    crs: int,
    needs_closed_checking: bool = True
) -> None:
    """
    入力引数の型チェック

    :param filepath: Shapefileのファイルパス
    :type filepath: str
    :param encoding: Shapefileの文字エンコーディング
    :type encoding: str
    :param h_zoom: 空間IDの水平方向精度レベル
    :type h_zoom: int
    :param v_zoom: 空間IDの垂直方向精度レベル
    :type v_zoom: int
    :param crs: Shapefileに格納されている座標の参照座標系。デフォルト値:4326(WGS84)
    :type crs: int
    :param needs_closed_checking: ポリゴンの空間ID取得時の閉塞チェックフラグ。Trueの場合はチェックを行う。
    :type needs_closed_checking: bool

    :raise SpatialIdError: 入力チェック結果が不正の場合のエラー
    """
    error_flag = False
    if type(filepath) is not str:
        error_flag = True

    if type(encoding) is not str:
        error_flag = True

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

# Copyright © 2022 Digital Agency. All rights reserved.
from enum import Enum


class SpatialIdError(Exception):
    """独自の例外をraiseする。

    :param Exception: SpatialIdError
    :type Exception: Exception
    """
    def __init__(self, status_code, args=''):
        self.args = args
        self.status_code = status_code

    def __str__(self):
        if not self.args:
            return (
                f"{self.status_code},"
                f"{get_message_from_status_code(self.status_code)}"
            )
        else:
            return (
                f"{self.status_code},"
                f"{get_message_from_status_code(self.status_code)}"
                f"{self.args}"
            )


def get_message_from_status_code(status_code: str) -> str:
    """メッセージ定数からメッセージの本文を取得する。

    :param status_code: ステータスコード
    :type status_code: str
    :return: ステータスコードに紐付いたメッセージ
    :rtype: str
    """
    return Messages[status_code].value


class Messages(Enum):
    """例外メッセージを管理するクラス

    :returns: 指定の定数に該当する列挙型
    :rtype: Enum
    """

    VALUE_CONVERT_ERROR = '値の変換エラー'  #: 値の変換時に例外またはNaN,Infが発生した場合のエラー。

    OPTION_FAILED_ERROR = 'オプション値の指定エラー'  #: 指定外のオプションが指定された場合のエラー。

    INPUT_VALUE_ERROR = '入力チェックエラー'  #: 入力チェック結果が不正の場合のエラー。

    #: 3点が同一直線状に存在する三角ポリゴンを検出した場合のエラー
    POLYGON_POINT_COLLINEAR = '3点が同一直線状に存在する三角ポリゴンを検出'

    #: 三角形ポリゴンのモデルが閉塞していない場合のエラー
    POLYGON_NOT_CLOSED_MODEL = '三角形ポリゴンのモデルが閉塞していない'

    #: shapefile.ReaderによるShapefileの読み込みに失敗した場合のエラー。
    SHAPEFILE_READ_ERROR = 'shapefile読み込み失敗'

    #: shapefileの解析時にサポート対象外のshapeTypeを検出した場合のエラー。
    SHAPEFILE_SHAPETYPE_ERROR = 'サポート対象外のshapeTypeを検出'

    #: shapefileの解析時にpointsとzの要素数不一致を検知した場合のエラー。
    SHAPEFILE_POINTS_NUM_ERROR = 'shapefileレコードのpointsとzの要素数が不一致'

    #: shapefileの解析時にpartsとpartTypesの要素数不一致を検知した場合のエラー
    SHAPEFILE_PARTS_NUM_ERROR = 'shapefileレコードのpartsとpartTypesの要素数が不一致'

    #: shapefileの解析時に対象partTypeに必要な点の数が不足していた場合のエラー
    SHAPEFILE_PARTS_POINTS_NUM_ERROR = 'partTypeに必要な点の数が不足'

    #: shapefileの解析時に対象partTypeの要素数が0の場合のエラー
    SHAPEFILE_PART_TYPE_LIST_ERROR = 'partTypeの要素数が0'

    #: shapefileの解析時に対象partsの要素数が0の場合のエラー
    SHAPEFILE_PARTS_LIST_ERROR = 'partsの要素数が0'

    #: shapefileの解析時にサポート対象外のpartTypeを検出した場合のエラー
    SHAPEFILE_PARTETYPE_ERROR = 'サポート対象外のpartTypeを検出'

    #: shapefileの集約関数の実行に失敗した場合のエラー
    SHAPEFILE_AGGREGATION_FUNC_ERROR = 'shapefileの集約関数の実行に失敗'

    #: その他例外発生をキャッチした場合
    OTHER_ERROR = 'その他例外が発生'

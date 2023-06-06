# Copyright © 2022 Digital Agency. All rights reserved.
from enum import Enum, auto


"""
| ENUM管理クラス
| オプション値などはここに定義をする。
"""


class Point_Option(Enum):
    """入力可能な座標取得のオプションを管理するクラス

    :returns: 指定の定数に該当する列挙型
    :rtype: Enum
    """

    VERTEX = auto()  #: 空間IDの頂点座標を取得
    CENTER = auto()  #: 空間IDの中心座標を取得


class ShapeType(Enum):
    """ShapefileのShape種別の値を管理するクラス

    :returns: 指定の定数に該当する列挙型
    :rtype: Enum
    """

    MULTIPATCH = 31


class PartType(Enum):
    """ShapefileのPart種別の値を管理するクラス

    :returns: 指定の定数に該当する列挙型
    :rtype: Enum
    """

    TRIANGLE_STRIP = 0
    TRIANGLE_FAN = 1

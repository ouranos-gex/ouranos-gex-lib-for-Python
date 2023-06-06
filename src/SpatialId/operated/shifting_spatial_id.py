# Copyright © 2022 Digital Agency. All rights reserved.
"""
|  Requirements: Python 3.9+.

|  指定の空間IDの操作をする。
"""

import re

from SpatialId.convert import in_to_internal, internal_to_out

def get_6spatial_ids_adjacent_to_faces(spatial_id: str) -> list[str]:
    """空間IDの面に直接、接している6個の空間IDを取得する。

    :param spatial_id: 空間ID
    :type spatial_id: str
    :return: 空間IDのリスト
    :rtype: list[str]
    :注意:  東端の空間IDの東にある空間IDとして同緯度の西端の空間IDが取得される。南端の空間IDの南にある空間IDとして同経度の北端のIDが取得される。（東西、南北逆でも同様）
    """
    # 返却用リスト
    spatial_id_list = []
    # 各方向に1と-1の移動を取得
    for shift_index in range(-1, 2, 2):
        # 経度方向の移動
        spatial_id_list.extend(
            [get_shifting_spatial_id(spatial_id, shift_index, 0, 0)])
        # 緯度方向の移動
        spatial_id_list.extend(
            [get_shifting_spatial_id(spatial_id, 0, shift_index, 0)])
        # 高さ方向の移動
        spatial_id_list.extend(
            [get_shifting_spatial_id(spatial_id, 0, 0, shift_index)])

    return spatial_id_list

def f_get_6spatial_ids_adjacent_to_faces(spatial_id: str) -> list[str]:
    """空間IDの面に直接、接している6個の空間IDを取得する。

    | f_"がついてないAPIとの違いは以下：
    | 入出力の空間IDの並びは仕様通りの記載。
    | z/f/x/y :z{ズームレベル}/f{高さの位置}/x{経度方向の位置}/y{緯度方向の位置}
    | 参照：https://github.com/unvt/zfxy-spec

    :param spatial_id: 空間ID
    :type spatial_id: str
    :return: 空間IDのリスト
    :rtype: list[str]
    :注意:  東端の空間IDの東にある空間IDとして同緯度の西端の空間IDが取得される。南端の空間IDの南にある空間IDとして同経度の北端のIDが取得される。（東西、南北逆でも同様）
    """
    converted = in_to_internal(spatial_id)
    out = get_6spatial_ids_adjacent_to_faces(converted)
    return internal_to_out(out)


def get_8spatial_ids_around_horizontal(spatial_id: str) -> list[str]:
    """空間IDの水平方向の周囲、一周分の8個の空間IDを取得する。

    :param spatial_id: 空間ID
    :type spatial_id: str
    :return: 空間IDのリスト
    :rtype: list[str]
    :注意: 東端の空間IDの東にある空間IDとして同緯度の西端の空間IDが取得される。南端の空間IDの南にある空間IDとして同経度の北端のIDが取得される。（東西、南北逆でも同様）
    """
    # 返却用リスト
    spatial_id_list = []
    for shift_index in range(-1, 2, 2):
        # 経度方向の移動
        spatial_id_list.extend(
            [get_shifting_spatial_id(spatial_id, shift_index, 0, 0)])
        # 緯度方向の移動
        spatial_id_list.extend(
            [get_shifting_spatial_id(spatial_id, 0, shift_index, 0)])
        # 水平方向に右肩下がりの斜め方向の移動
        spatial_id_list.extend(
            [get_shifting_spatial_id(spatial_id, shift_index, shift_index, 0)])
        # 水平方向に右肩上がり方向の移動
        spatial_id_list.extend(
            [get_shifting_spatial_id(
                spatial_id, shift_index, -shift_index, 0)])

    return spatial_id_list


def f_get_8spatial_ids_around_horizontal(spatial_id: str) -> list[str]:
    """空間IDの水平方向の周囲、一周分の8個の空間IDを取得する。

    | f_"がついてないAPIとの違いは以下：
    | 入出力の空間IDの並びは仕様通りの記載。
    | z/f/x/y :z{ズームレベル}/f{高さの位置}/x{経度方向の位置}/y{緯度方向の位置}
    | 参照：https://github.com/unvt/zfxy-spec

    :param spatial_id: 空間ID
    :type spatial_id: str
    :return: 空間IDのリスト
    :rtype: list[str]
    :注意: 東端の空間IDの東にある空間IDとして同緯度の西端の空間IDが取得される。南端の空間IDの南にある空間IDとして同経度の北端のIDが取得される。（東西、南北逆でも同様）
    """
    converted = in_to_internal(spatial_id)
    out = get_8spatial_ids_around_horizontal(converted)
    return internal_to_out(out)


def get_26spatial_ids_around_voxel(spatial_id: str) -> list[str]:
    """空間IDを囲う26個の空間IDを取得する。

    :param spatial_id: 空間ID
    :type spatial_id: str
    :return: 空間IDのリスト
    :rtype: list[str]
    :注意:  東端の空間IDの東にある空間IDとして同緯度の西端の空間IDが取得される。南端の空間IDの南にある空間IDとして同経度の北端のIDが取得される。（東西、南北逆でも同様）
    """
    # 返却用リスト
    spatial_id_list = []
    # 入力された空間IDからみて高さが一つ分低い位置の空間ID、同じ高さの空間ID、一つ分高い位置の空間IDを取得する。
    for shift_index in range(-1, 2, 1):
        # 高さを移動した空間IDを取得する。
        shifting_spatial_id = get_shifting_spatial_id(
            spatial_id, 0, 0, shift_index)
        # 取得した空間idを返却用リストに格納する。
        spatial_id_list.extend([shifting_spatial_id])
        # 高さを移動した空間IDの水平方向の空間IDを取得し、返却用リストに格納する。
        spatial_id_list.extend(
            get_8spatial_ids_around_horizontal(shifting_spatial_id))
    # 入力元の空間IDが含まれているため、削除する。
    spatial_id_list.remove(spatial_id)
    return spatial_id_list


def f_get_26spatial_ids_around_voxel(spatial_id: str) -> list[str]:
    """空間IDを囲う26個の空間IDを取得する。

    | f_"がついてないAPIとの違いは以下：
    | 入出力の空間IDの並びは仕様通りの記載。
    | z/f/x/y :z{ズームレベル}/f{高さの位置}/x{経度方向の位置}/y{緯度方向の位置}
    | 参照：https://github.com/unvt/zfxy-spec

    :param spatial_id: 空間ID
    :type spatial_id: str
    :return: 空間IDのリスト
    :rtype: list[str]
    :注意:  東端の空間IDの東にある空間IDとして同緯度の西端の空間IDが取得される。南端の空間IDの南にある空間IDとして同経度の北端のIDが取得される。（東西、南北逆でも同様）
    """
    converted = in_to_internal(spatial_id)
    out = get_26spatial_ids_around_voxel(converted)
    return internal_to_out(out)


def get_shifting_spatial_id(
        spatial_id: str, x: int, y: int, v: int
) -> str:
    """
    |  指定の数値分、移動した場合の空間IDを取得する。
    |  水平方向の移動は、南緯、東経方向が正、北緯、西経方向を負とする。
    |  垂直方向の移動は、上空方向が正、地中方向を負とする。

    :param spatial_id: 元の位置となる空間ID
    :type spatial_id: str
    :param x: 空間IDを経度方向に動かす数値
    :type x: int
    :param y: 空間IDを緯度方向に動かす数値
    :type y: int
    :param v: 空間IDを高さ方向に動かす数値
    :type v: int
    :return: 指定の数値分、移動した場合の空間ID
    :rtype: str
    :注意:  東端の空間IDの東にある空間IDとして同緯度の西端の空間IDが取得される。南端の空間IDの南にある空間IDとして同経度の北端のIDが取得される。（東西、南北逆でも同様）
    """
    # 空間IDを分解して経度、緯度、高さの位置を取得する
    id = re.split('/', spatial_id)
    # IDから水平方向の位置を取得する。前方2桁は精度
    lon_index = int(id[1])
    lat_index = int(id[2])
    h_zoom = int(id[0])

    # インデックスの最大値を取得
    max_index = 2 ** h_zoom - 1

    # シフト後のインデックスを計算する
    shift_x_index = lon_index + x
    shift_y_index = lat_index + y

    # シフト後のインデックスが存在しているかチェックする。
    # x方向インデックスのチェック
    # インデックスが負の場合は精度-2^精度%abs(index)が
    if shift_x_index.__gt__(max_index) or shift_x_index.__lt__(0):
        # インデックスの範囲を超えている場合はn周分を無視する
        calc_x_index = shift_x_index % 2**h_zoom
    else:
        calc_x_index = lon_index + x

    # y方向インデックスのチェック
    if shift_y_index.__gt__(max_index) or shift_y_index.__lt__(0):
        # インデックスの範囲を超えている場合はn周分を無視する
        calc_y_index = shift_y_index % 2**h_zoom
    else:
        calc_y_index = lat_index + y
    # 垂直方向は上下限なし
    alt_index = int(id[4]) + v
    v_zoom = int(id[3])

    # 移動後の位置を組み合わせて空間IDとする
    spatial_id = ''.join(
        [str(h_zoom), '/', str(calc_x_index), '/', str(calc_y_index), '/',
         str(v_zoom), '/', str(alt_index)])

    return spatial_id


def f_get_shifting_spatial_id(
    spatial_id: str, 
    x: int, 
    y: int, 
    v: int) -> str:
    """
    |  指定の数値分、移動した場合の空間IDを取得する。
    |  水平方向の移動は、南緯、東経方向が正、北緯、西経方向を負とする。
    |  垂直方向の移動は、上空方向が正、地中方向を負とする。

    | f_"がついてないAPIとの違いは以下：
    | 入出力の空間IDの並びは仕様通りの記載。
    | z/f/x/y :z{ズームレベル}/f{高さの位置}/x{経度方向の位置}/y{緯度方向の位置}
    | 参照：https://github.com/unvt/zfxy-spec

    :param spatial_id: 元の位置となる空間ID
    :type spatial_id: str
    :param x: 空間IDを経度方向に動かす数値
    :type x: int
    :param y: 空間IDを緯度方向に動かす数値
    :type y: int
    :param v: 空間IDを高さ方向に動かす数値
    :type v: int
    :return: 指定の数値分、移動した場合の空間ID
    :rtype: str
    :注意:  東端の空間IDの東にある空間IDとして同緯度の西端の空間IDが取得される。南端の空間IDの南にある空間IDとして同経度の北端のIDが取得される。（東西、南北逆でも同様）
    """
    converted = in_to_internal(spatial_id)
    out = get_shifting_spatial_id(converted, x, y, v)
    return internal_to_out(out)


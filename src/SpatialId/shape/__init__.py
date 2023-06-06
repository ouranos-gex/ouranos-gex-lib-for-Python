# Copyright © 2022 Digital Agency. All rights reserved.
from threading import Lock
from pyproj import Transformer
from pyproj import CRS

__lock = Lock()

TRANSFORM_DICT = {}
CRS_DICT = {}


def get_transformer(in_CRS, outCRS):
    # 入出力の組み合わせでタプル型を生成する
    crs_list = (in_CRS, outCRS)
    # transformerオブジェクト作成済みの場合は、オブジェクトを返却する。
    if crs_list in TRANSFORM_DICT:
        return TRANSFORM_DICT[crs_list]
    else:
        # transformerオブジェクト未作成の場合は、作成してdictに登録後、オブジェクトを返却する。
        with __lock:
            transformer = Transformer.from_crs(in_CRS, outCRS, always_xy=True)
            TRANSFORM_DICT[crs_list] = transformer
            return transformer


def get_CRS_OBJECT(crs):
    # CRSオブジェクト作成済みの場合は、オブジェクトを返却する。
    if crs in CRS_DICT:
        return CRS_DICT[crs]
    else:
        # CRSオブジェクト未作成の場合は、作成してdictに登録後、オブジェクトを返却する。
        with __lock:
            crs_object = CRS.from_user_input(crs)
            CRS_DICT[crs] = crs_object
            return crs_object

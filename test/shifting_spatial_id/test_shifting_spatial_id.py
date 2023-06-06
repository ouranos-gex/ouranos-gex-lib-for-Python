from logging import getLogger
import pytest
from SpatialId import LOGGER_NAME
from SpatialId.operated.shifting_spatial_id import (
    f_get_6spatial_ids_adjacent_to_faces,
    f_get_8spatial_ids_around_horizontal,
    f_get_shifting_spatial_id,
    get_26spatial_ids_around_voxel,
    get_6spatial_ids_adjacent_to_faces,
    f_get_26spatial_ids_around_voxel)
from SpatialId.operated.shifting_spatial_id import (
    get_8spatial_ids_around_horizontal)
from SpatialId.operated.shifting_spatial_id import get_shifting_spatial_id

spatial_log = getLogger(LOGGER_NAME).getChild(__name__)


@pytest.mark.parametrize(('spatial_id', 'x', 'y', 'v', 'excepted'), [
    ('16/468/95/20/3', 5, 13, 5, '16/473/108/20/8'),  # 正常系
    ('16/468/10/20/3', -5, -13, -5, '16/463/65533/20/-2'),  # 正常系(負の方向)
    ('2/2/2/20/3', 10, 10, 5, '2/0/0/20/8'),  # インデックスを超えた移動距離
    ('2/8/15/20/3', 10, 10, 0, '2/2/1/20/3'),  # インデックスを超えた移動距離
])
def test_get_shifting_spatial_id(
        spatial_id: str, x: int, y: int, v: int, excepted):
    assert excepted.__eq__(get_shifting_spatial_id(spatial_id, x, y, v))

@pytest.mark.parametrize(('spatial_id', 'x', 'y', 'v', 'excepted'), [
    ('16/3/468/95', 5, 13, 5, '16/8/473/108'),  # 正常系
    ('16/3/468/10', -5, -13, -5, '16/-2/463/65533'),  # 正常系(負の方向)
    ('2/3/2/2', 10, 10, 5, '2/8/0/0'),  # インデックスを超えた移動距離
    ('2/3/8/15', 10, 10, 0, '2/3/2/1'),  # インデックスを超えた移動距離
])
def test_f_get_shifting_spatial_id(
        spatial_id: str, x: int, y: int, v: int, excepted):
    assert excepted.__eq__(f_get_shifting_spatial_id(spatial_id, x, y, v))

@pytest.mark.parametrize(('spatial_id','case',  'excepted'), [
    ('16/468/95/20/3', 1,
     ['16/467/95/20/3', '16/469/95/20/3', '16/468/94/20/3', '16/468/96/20/3',
      '16/467/94/20/3', '16/467/96/20/3', '16/469/94/20/3', '16/469/96/20/3']),  # 正常系
    ('4/15/15/1/5', 2,
     ['4/0/15/1/5', '4/14/15/1/5', '4/15/0/1/5', '4/15/14/1/5',
      '4/0/0/1/5', '4/0/14/1/5', '4/14/0/1/5', '4/14/14/1/5']),  # 右下のタイル周囲1周
    ('3/0/0/1/5', 3,
     ['3/1/0/1/5', '3/7/0/1/5', '3/0/1/1/5', '3/0/7/1/5',
      '3/1/1/1/5', '3/1/7/1/5', '3/7/1/1/5', '3/7/7/1/5']),  # 左上のタイル周囲1周
])
def test_get_8spatial_ids_around_horizontal(spatial_id: str, case, excepted):
    spatial_id_list = []
    spatial_log.info(f'case:{case}')
    spatial_id_list = get_8spatial_ids_around_horizontal(spatial_id)
    # 周囲のIDの取得
    for excepted_id in excepted:
        assert excepted_id in spatial_id_list


@pytest.mark.parametrize(('spatial_id','case',  'excepted'), [
    ('16/3/468/95', 1,
     ['16/3/467/95', '16/3/469/95', '16/3/468/94', '16/3/468/96', 
     '16/3/467/94', '16/3/467/96', '16/3/469/94', '16/3/469/96']),  # 正常系
    ('4/5/15/15', 2,
     ['4/5/0/15', '4/5/14/15', '4/5/15/0', '4/5/15/14', 
     '4/5/0/0', '4/5/0/14', '4/5/14/0', '4/5/14/14']),  # 右下のタイル周囲1周
    ('3/5/0/0', 3,
     ['3/5/1/0', '3/5/7/0', '3/5/0/1', '3/5/0/7', 
     '3/5/1/1', '3/5/1/7', '3/5/7/1', '3/5/7/7']),  # 左上のタイル周囲1周
])
def test_f_get_8spatial_ids_around_horizontal(spatial_id: str, case, excepted):
    spatial_id_list = []
    spatial_log.info(f'case:{case}')
    spatial_id_list = f_get_8spatial_ids_around_horizontal(spatial_id)
    # 周囲のIDの取得
    for excepted_id in excepted:
        assert excepted_id in spatial_id_list



@pytest.mark.parametrize(('spatial_id', 'case', 'excepted'), [
    ('16/468/95/20/3', 1,
     ['16/468/95/20/4', '16/467/95/20/3', '16/469/95/20/3',
      '16/468/94/20/3', '16/468/96/20/3', '16/468/95/20/2']),
    ('3/0/0/20/0', 2,
     ['3/0/0/20/1', '3/1/0/20/0', '3/0/1/20/0',
      '3/7/0/20/0', '3/0/7/20/0', '3/0/0/20/-1']),  # 左上のタイル
    ('3/7/7/20/0', 3,
     ['3/0/7/20/0', '3/6/7/20/0', '3/7/0/20/0',
      '3/7/6/20/0', '3/7/7/20/1', '3/7/7/20/-1']),  # 右下のタイル
])
def test_get_6spatial_ids_adjacent_to_faces(spatial_id: str, case, excepted):
    spatial_id_list = []
    spatial_log.info(f'case:{case}')

    spatial_id_list = get_6spatial_ids_adjacent_to_faces(spatial_id)
    spatial_log.info(spatial_id_list)
    for excepted_id in excepted:
        spatial_log.info(excepted_id)
        assert excepted_id in spatial_id_list

@pytest.mark.parametrize(('spatial_id', 'case', 'excepted'), [
    ('16/3/468/95', 1,
     ['16/4/468/95', '16/3/467/95', '16/3/469/95', 
     '16/3/468/94', '16/3/468/96', '16/2/468/95']),
    ('3/0/0/0', 2,
     ['3/1/0/0', '3/0/1/0', '3/0/0/1', '3/0/7/0', 
     '3/0/0/7', '3/-1/0/0']),  # 左上のタイル
    ('3/0/7/7', 3,
     ['3/0/0/7', '3/0/6/7', '3/0/7/0', '3/0/7/6', 
     '3/1/7/7', '3/-1/7/7']),  # 右下のタイル
])
def test_f_get_6spatial_ids_adjacent_to_faces(spatial_id: str, case, excepted):
    spatial_id_list = []
    spatial_log.info(f'case:{case}')

    spatial_id_list = f_get_6spatial_ids_adjacent_to_faces(spatial_id)
    spatial_log.info(spatial_id_list)
    for excepted_id in excepted:
        spatial_log.info(excepted_id)
        assert excepted_id in spatial_id_list


@pytest.mark.parametrize(('spatial_id', 'case', 'excepted'), [
    ('16/468/95/20/3', 1,
     ['16/467/95/20/3', '16/469/95/20/3', '16/468/94/20/3', '16/468/96/20/3',
      '16/467/94/20/3', '16/467/96/20/3', '16/469/94/20/3', '16/469/96/20/3',
      '16/467/95/20/4', '16/469/95/20/4', '16/468/94/20/4', '16/468/96/20/4',
      '16/467/94/20/4', '16/467/96/20/4', '16/469/94/20/4', '16/469/96/20/4',
      '16/468/95/20/4',
      '16/467/95/20/2', '16/469/95/20/2', '16/468/94/20/2', '16/468/96/20/2',
      '16/467/94/20/2', '16/467/96/20/2', '16/469/94/20/2', '16/469/96/20/2',
      '16/468/95/20/2'
      ]),  # 正常系
    ('4/15/15/1/5', 2,
     ['4/0/15/1/5', '4/14/15/1/5', '4/15/0/1/5', '4/15/14/1/5',
      '4/0/0/1/5', '4/0/14/1/5', '4/14/0/1/5', '4/14/14/1/5',
      '4/0/15/1/4', '4/14/15/1/4', '4/15/0/1/4', '4/15/14/1/4',
      '4/0/0/1/4', '4/0/14/1/4', '4/14/0/1/4', '4/14/14/1/4',
      '4/15/15/1/4',
      '4/0/15/1/6', '4/14/15/1/6', '4/15/0/1/6', '4/15/14/1/6',
      '4/0/0/1/6', '4/0/14/1/6', '4/14/0/1/6', '4/14/14/1/6',
      '4/15/15/1/6'
      ]),  # 右下のタイル周囲1周
    ('3/0/0/1/5', 3,
     ['3/1/0/1/5', '3/7/0/1/5', '3/0/1/1/5', '3/0/7/1/5',
      '3/1/1/1/5', '3/1/7/1/5', '3/7/1/1/5', '3/7/7/1/5',
      '3/1/0/1/4', '3/7/0/1/4', '3/0/1/1/4', '3/0/7/1/4',
      '3/1/1/1/4', '3/1/7/1/4', '3/7/1/1/4', '3/7/7/1/4',
      '3/0/0/1/4',
      '3/1/0/1/6', '3/7/0/1/6', '3/0/1/1/6', '3/0/7/1/6',
      '3/1/1/1/6', '3/1/7/1/6', '3/7/1/1/6', '3/7/7/1/6',
      '3/0/0/1/6'
      ]),  # 左上のタイル周囲1周
])
def test_get_26spatial_ids_around_voxel(spatial_id: str, case, excepted):
    spatial_id_list = []
    spatial_log.info(f'case:{case}')

    spatial_id_list = get_26spatial_ids_around_voxel(spatial_id)

    # 個数の確認
    assert len(spatial_id_list) == 26

    # 内容の確認
    for excepted_id in excepted:
        spatial_log.info(excepted_id)
        assert excepted_id in spatial_id_list

@pytest.mark.parametrize(('spatial_id', 'case', 'excepted'), [
    ('16/3/468/95', 1,
     ['16/3/467/95', '16/3/469/95', '16/3/468/94', '16/3/468/96',
     '16/3/467/94', '16/3/467/96', '16/3/469/94', '16/3/469/96',
     '16/4/467/95', '16/4/469/95', '16/4/468/94', '16/4/468/96',
     '16/4/467/94', '16/4/467/96', '16/4/469/94', '16/4/469/96',
     '16/4/468/95', '16/2/467/95', '16/2/469/95', '16/2/468/94',
     '16/2/468/96', '16/2/467/94', '16/2/467/96', '16/2/469/94',
     '16/2/469/96', '16/2/468/95'
      ]),  # 正常系
    ('4/5/15/15', 2,
     ['4/5/0/15', '4/5/14/15', '4/5/15/0', '4/5/15/14', '4/5/0/0', 
     '4/5/0/14', '4/5/14/0', '4/5/14/14', '4/4/0/15', '4/4/14/15', 
     '4/4/15/0', '4/4/15/14', '4/4/0/0', '4/4/0/14', '4/4/14/0', 
     '4/4/14/14', '4/4/15/15', '4/6/0/15', '4/6/14/15', '4/6/15/0', 
     '4/6/15/14', '4/6/0/0', '4/6/0/14', '4/6/14/0', '4/6/14/14', 
     '4/6/15/15'
      ]),  # 右下のタイル周囲1周
    ('3/5/0/0', 3,
     ['3/5/1/0', '3/5/7/0', '3/5/0/1', '3/5/0/7', '3/5/1/1', 
     '3/5/1/7', '3/5/7/1', '3/5/7/7', '3/4/1/0', '3/4/7/0', 
     '3/4/0/1', '3/4/0/7', '3/4/1/1', '3/4/1/7', '3/4/7/1', 
     '3/4/7/7', '3/4/0/0', '3/6/1/0', '3/6/7/0', '3/6/0/1', 
     '3/6/0/7', '3/6/1/1', '3/6/1/7', '3/6/7/1', '3/6/7/7', 
     '3/6/0/0'
     ]),  # 左上のタイル周囲1周
])
def test_f_get_26spatial_ids_around_voxel(spatial_id: str, case, excepted):
    spatial_id_list = []
    spatial_log.info(f'case:{case}')

    spatial_id_list = f_get_26spatial_ids_around_voxel(spatial_id)

    # 個数の確認
    assert len(spatial_id_list) == 26

    # 内容の確認
    for excepted_id in excepted:
        spatial_log.info(excepted_id)
        assert excepted_id in spatial_id_list
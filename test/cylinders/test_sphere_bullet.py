#!/bin/env python3
# -*- coding: utf-8 -*-

import pytest
from skspatial.objects import Point

from SpatialId.shape.bullet.sphere_bullet import SphereBullet

class TestSphereBullet:
    def test_is_collide_voxcel_01(self):
        """ボクセルとPyBulletの球との衝突判定（衝突）
        + 試験詳細
          - 入力条件
            - 球の座標が入力に指定されること
          - 確認内容
            - 衝突したと判定されること
        """
        ## 試験用データの定義
        # 半径
        radius = 2.2
        # 中心点
        center = Point([-18843867.70908793, 17669794.95462802, 163840.0000152588])

        # オブジェクト初期化
        sphere_bullet = SphereBullet(radius, center)

        # 期待値
        is_collide_voxel_assert = True

        # 実行
        point = Point([-18843865.7090879, 17669794.95462802, 163840.0000152588])
        half_extent = Point([1, 1, 1])
        result = sphere_bullet.is_collide_voxcel(point, half_extent)

        assert result == is_collide_voxel_assert

    def test_is_collide_voxcel_02(self):
        """ボクセルとPyBulletの球との衝突判定（非衝突）
        + 試験詳細
          - 入力条件
            - 球の座標が入力に指定されること
          - 確認内容
            - 衝突していないと判定されること
        """
        ## 試験用データの定義
        # 半径
        radius = 2.2
        # 中心点
        center = Point([-18843867.70908793, 17669794.95462802, 163840.0000152588])

        # オブジェクト初期化
        sphere_bullet = SphereBullet(radius, center)

        # 期待値
        is_collide_voxel_assert = False

        # 実行
        point = Point([-18843860.7090879, 17669794.95462802, 163840.0000152588])
        half_extent = Point([1, 1, 1])
        result = sphere_bullet.is_collide_voxcel(point, half_extent)

        assert result == is_collide_voxel_assert

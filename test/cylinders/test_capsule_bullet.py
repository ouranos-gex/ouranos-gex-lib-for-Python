#!/bin/env python3
# -*- coding: utf-8 -*-

import pytest
from skspatial.objects import Point

from SpatialId.shape.bullet.capsule_bullet import CapsuleBullet

class TestCapsuleBullet:

    def test_is_collide_voxcel_01(self):
        """ボクセルとPyBulletのカプセルとの衝突判定（衝突）
        + 試験詳細
          - 入力条件
            - カプセルの座標が入力に指定されること
            - 始点と終点のベクトルがX成分のみでないこと
            - 衝突するボクセルを指定する
          - 確認内容
            - 衝突したと判定されること
        """
        ## 試験用データの定義
        # 半径
        radius = 2.0
        # 始点
        start = Point([-18843867.70908793, 17669794.95462802, 163840.0000152588])
        # 終点
        end = Point([-18843867.70908793, 17669794.95462802, 163844.0000152588])

        # オブジェクト初期化
        capsule_bullet = CapsuleBullet(radius, start, end)

        # 期待値
        is_collide_voxel_assert = True

        # 実行
        point = Point([-18843867.7090879, 17669794.95462802, 163846.9000152588])
        half_extent = Point([1, 1, 1])
        result = capsule_bullet.is_collide_voxcel(point, half_extent)

        assert result == is_collide_voxel_assert

    def test_is_collide_voxcel_02(self):
        """ボクセルとPyBulletのカプセルとの衝突判定（非衝突）
        + 試験詳細
          - 入力条件
            - カプセルの座標が入力に指定されること
            - 始点と終点のベクトルがX成分のみでないこと
            - 衝突するボクセルを指定する
          - 確認内容
            - 衝突したと判定されること
        """
        ## 試験用データの定義
        # 半径
        radius = 2.0
        # 始点
        start = Point([-18843867.70908793, 17669794.95462802, 163840.0000152588])
        # 終点
        end = Point([-18843867.70908793, 17669794.95462802, 163844.0000152588])

        # オブジェクト初期化
        capsule_bullet = CapsuleBullet(radius, start, end)

        # 期待値
        is_collide_voxel_assert = False

        # 実行
        point = Point([-18843867.7090879, 17669794.95462802, 163847.0000152588])
        half_extent = Point([1, 1, 1])
        result = capsule_bullet.is_collide_voxcel(point, half_extent)

        assert result == is_collide_voxel_assert

    def test_is_collide_voxcel_03(self):
        """ボクセルとPyBulletのカプセルとの衝突判定（衝突）※円柱がX方向のみに伸びる
        + 試験詳細
          - 入力条件
            - カプセルの座標が入力に指定されること
            - 始点と終点のベクトルがX成分のみであること
            - 衝突するボクセルを指定する
          - 確認内容
            - 衝突したと判定されること
        """
        ## 試験用データの定義
        # 半径
        radius = 2.0
        # 始点
        start = Point([-18843867.70908793, 17669794.95462802, 163840.0000152588])
        # 終点
        end = Point([-18843870.70908793, 17669794.95462802, 163840.0000152588])

        # オブジェクト初期化
        capsule_bullet = CapsuleBullet(radius, start, end)

        # 期待値
        is_collide_voxel_assert = True

        # 実行
        point = Point([-18843870.7090879, 17669797.85462802, 163840.0000152588])
        half_extent = Point([1, 1, 1])
        result = capsule_bullet.is_collide_voxcel(point, half_extent)

        assert result == is_collide_voxel_assert

#!/bin/env python3
# -*- coding: utf-8 -*-

import pytest
from skspatial.objects import Vector

from SpatialId.shape.bullet.base_bullet import BaseBullet

MINIMA = 1e-10

# TestQuat 四元数クラス(試験用)
class QuatTest:
    def __init__(self, x, y, z, w: float):
      self.x = x
      self.y = y
      self.z = z
      self.w = w

    # Mul 四元数の積
    #
    # 四元数の積を算出して返却する。
    #
    # 引数：
    #  b： 掛け合わせる四元数
    #
    # 戻り値：
    #  四元数の積
    def Mul(self, b):
      return QuatTest(
        (b.x * self.w) + (b.w * self.x) - (b.z * self.y) + (b.y * self.z),
        (b.y * self.w) + (b.z * self.x) + (b.w * self.y) - (b.x * self.z),
        (b.z * self.w) - (b.y * self.x) + (b.x * self.y) + (b.w * self.z),
        (b.w * self.w) - (b.x * self.x) - (b.y * self.y) - (b.z * self.z)
      )

    # TransformVec3 四元数でのベクトル変換
    #
    # 入力のベクトルを四元数で変換した値を返却する。
    #
    # 引数：
    #  v： 入力ベクトル
    #
    # 戻り値：
    #  変換後のベクトル
    def TransformVec3(self, v):
      vecTestQuat = QuatTest(v[0], v[1], v[2], 0.0)
      conjugate = self.Conjugate()
      vecTestQuat = conjugate.Mul(vecTestQuat).Mul(self)
      return Vector([vecTestQuat.x, vecTestQuat.y, vecTestQuat.z])

    # Conjugate 四元数の共役
    #
    # 共役四元数を算出して返却する。
    #
    # 戻り値：
    #  共役四元数
    def Conjugate(self):
      return QuatTest(
      	-self.x,
      	-self.y,
      	-self.z,
        self.w
      )

class TestBaseBullet:

    @pytest.fixture()
    def baseBullet(self):
      return BaseBullet()

    def test_rotate_between_vector_01(self, baseBullet: BaseBullet):
        """2ベクトル間の四元数算出処理
        + 試験詳細
          - 入力条件
            - 開始ベクトルと終了ベクトルが正反対を向いていない
          - 確認内容
            - 2ベクトル間の四元数を取得できること
            - 四元数を元に開始ベクトルを回転させて、終了ベクトルと近似すること
        """
        ## 試験用データの定義
        # 開始ベクトル
        start = Vector([1, 2, 3])
        # 終了ベクトル
        end = Vector([4, 5, 6])

        # 実行
        result = baseBullet.rotate_between_vector(start, end)

        # 検算値作成
        testQuat = QuatTest(
          result[0],
          result[1],
          result[2],
          result[3]
        )
        expectVal = testQuat.TransformVec3(start.unit())

        assert expectVal[0] == pytest.approx(end.unit()[0], MINIMA)
        assert expectVal[1] == pytest.approx(end.unit()[1], MINIMA)
        assert expectVal[2] == pytest.approx(end.unit()[2], MINIMA)

    def test_rotate_between_vector_02(self, baseBullet: BaseBullet):
        """2ベクトル間の四元数算出処理
        + 試験詳細
          - 入力条件
            - 開始ベクトルと終了ベクトルが正反対を向いている
          - 確認内容
            - 2ベクトル間の四元数を取得できること
        """
        ## 試験用データの定義
        # 開始ベクトル
        start = Vector([1, 2, 3])
        # 終了ベクトル
        end = Vector([-1, -2, -3])

        # 実行
        result = baseBullet.rotate_between_vector(start, end)

        # 検算値作成
        testQuat = QuatTest(
          result[0],
          result[1],
          result[2],
          result[3]
        )
        expectVal = testQuat.TransformVec3(start.unit())

        assert expectVal[0] == pytest.approx(end.unit()[0], MINIMA)
        assert expectVal[1] == pytest.approx(end.unit()[1], MINIMA)
        assert expectVal[2] == pytest.approx(end.unit()[2], MINIMA)

    def test_rotate_between_vector_03(self, baseBullet: BaseBullet):
        """2ベクトル間の四元数算出処理
        + 試験詳細
          - 入力条件
            - 開始ベクトルと終了ベクトルが正反対を向いている
            - 開始ベクトルの垂直になるベクトルのノルムが小さい
          - 確認内容
            - 2ベクトル間の四元数を取得できること
        """
        ## 試験用データの定義
        # 開始ベクトル
        start = Vector([0, 0, 0.1])
        # 終了ベクトル
        end = Vector([0, 0, -0.1])

        # 実行
        result = baseBullet.rotate_between_vector(start, end)

        # 検算値作成
        testQuat = QuatTest(
          result[0],
          result[1],
          result[2],
          result[3]
        )
        expectVal = testQuat.TransformVec3(start.unit())

        assert expectVal[0] == pytest.approx(end.unit()[0], MINIMA)
        assert expectVal[1] == pytest.approx(end.unit()[1], MINIMA)
        assert expectVal[2] == pytest.approx(end.unit()[2], MINIMA)

    def test_quat_from_axis_angle_01(self, baseBullet: BaseBullet):
        """回転ベクトルの四元数算出処理
        + 試験詳細
          - 入力条件
            - 回転軸ベクトルと回転角度を渡す
          - 確認内容
            - 回転ベクトルの四元数を取得できること
        """
        # 下記テストケースと同時消化
        # test_rotate_between_vector_02
        # test_rotate_between_vector_03
        
# Copyright © 2022 Digital Agency. All rights reserved.
import pybullet
import numpy as np
import math
from logging import getLogger

from skspatial.objects import Point, Vector

from SpatialId import LOGGER_NAME

spatial_log = getLogger(LOGGER_NAME).getChild(__name__)

# 浮動小数点誤差
__MINIMA__ = 1e-10

class BaseBullet:
    """
    :var _engine: PyBulletエンジン
    :var _object: PyBulletオブジェクト
    """

    def __init__(self) -> None:
        """
        コンストラクタ
        """
        # PyBulletエンジン初期化
        self._engine = pybullet.connect(pybullet.DIRECT)
        # PyBulletオブジェクト初期化
        self._object = None

    def __del__(self) -> None:
        """
        デストラクタ
        """
        pybullet.disconnect(self._engine)

    def is_collide_voxcel(
        self, voxel_center: Point, half_extent: Point
    ) -> bool:
        """
        図形とボクセルとの衝突判定

        :param voxel_center: ボクセル中心
        :type  voxel_center: skspatial.objects.Point
        :param half_extent: ボクセル中心からの頂点までの成分
        :type  half_extent: skspatial.objects.Point

        :returns: 衝突しているならTrue、衝突していないならFalse
        :rtype:   bool
        """

        # ボクセル
        geom_box = pybullet.createCollisionShape(
            pybullet.GEOM_BOX,
            halfExtents=[half_extent[0], half_extent[1], half_extent[2]],
            physicsClientId=self._engine
        )
        # ボクセルオブジェクト
        object_box = pybullet.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=geom_box,
            basePosition=[voxel_center[0], voxel_center[1], voxel_center[2]],
            physicsClientId=self._engine
        )
        # ボクセルオブジェクトとの衝突
        pts = pybullet.getClosestPoints(
            bodyA=self._object, bodyB=object_box,
            distance=1, physicsClientId=self._engine)

        if len(pts) > 0:
            # オブジェクト間の距離
            distance = pts[0][8]
            if distance <= 0:
                return True
            else:
                return False

        else:
            return False


    def rotate_between_vector(cls, start: Vector, end: Vector) -> Vector:
        """
        2ベクトル間の四元数算出処理

        2ベクトル間の四元数を算出して返却する。

        :param start: 開始ベクトル
        :type  start: Vector
        :param end: 終了ベクトル
        :type  end: Vector

        :returns: 2ベクトル間の四元数
        :rtype:   Vector
        """
        # ベクトルを正規化
        start = start.unit()
        end = end.unit()
        # ベクトル間のcosを算出
        cos = np.dot(start, end)

        # ベクトルが正反対方向を向いている場合
        if cos + 1 < __MINIMA__:
            # 垂直になるベクトルを回転軸ベクトルとする
            axis = start.cross(Vector([0.0, 0.0, 1.0]))
            if axis.norm() < __MINIMA__:
                axis = start.cross(Vector([1.0, 0.0, 0.0]))

            return cls.quat_from_axis_angle(axis, math.pi)

        # 法線ベクトルを回転軸ベクトルとする
        axis = start.cross(end)
        s = math.sqrt(2 * (1 + cos))
        inv = 1 / s
        return Vector([
            axis[0] * inv,
            axis[1] * inv,
            axis[2] * inv,
            s * 0.5,
        ])

    def quat_from_axis_angle(cls, axis: Vector, angle: float) -> Vector:
        """
        回転ベクトルの四元数算出処理

        回転ベクトルの四元数を算出して返却する。

        :param axis: 回転軸ベクトル
        :type  axis: Vector
        :param angle: 回転角度(ラジアン)
        :type  angle: float

        :returns: 回転ベクトルの四元数
        :rtype:   Vector
        """
        # 回転軸ベクトルを正規化
        axis = axis.unit()
        # 半角のsinを算出
        sin_half_angle = math.sin(angle * 0.5)
        return Vector([
            axis[0] * sin_half_angle,
            axis[1] * sin_half_angle,
            axis[2] * sin_half_angle,
            math.cos(angle * 0.5)
        ])
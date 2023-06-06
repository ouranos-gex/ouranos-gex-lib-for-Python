# Copyright © 2022 Digital Agency. All rights reserved.
from skspatial.objects import Vector, Point
import pybullet
from .base_bullet import BaseBullet


class CapsuleBullet(BaseBullet):
    """
    カプセル用のPuBulletクラス

    :var _engine: PyBulletエンジン
    :var _object: PyBulletオブジェクト
    """

    def __init__(
        self, radius: float, start_point: Point, end_point: Point
    ) -> None:
        """
        コンストラクタ

        :param radius: カプセルの半径
        :type  radius: float
        :param start_point: カプセルの始点
        :type  start_point: skspatial.objects.Point
        :param end_point: カプセルの終点
        :type  end_point: skspatial.objects.Point
        """
        super().__init__()

        axis_vector = Vector(end_point - start_point)
        center_point = (end_point + start_point) / 2
        height = axis_vector.norm()

        # カプセルのモデル
        # デフォルトでは向きはz軸方向になっている
        geom_capsule = pybullet.createCollisionShape(
            pybullet.GEOM_CAPSULE,
            radius=radius,
            height=height,
            physicsClientId=self._engine
        )

        # 傾き(四元数)
        # ヨーイング角はカプセルの形状的に任意でよいため、0ラジアンとする
        base_orientation = self.rotate_between_vector(
            Vector([0, 0, 1]),
            axis_vector
        ).tolist()

        # 衝突判定用のカプセルオブジェクト
        self._object = pybullet.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=geom_capsule,
            basePosition=center_point.tolist(),
            baseOrientation=base_orientation,
            physicsClientId=self._engine
        )

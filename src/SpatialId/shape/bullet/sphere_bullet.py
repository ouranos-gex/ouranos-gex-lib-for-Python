# Copyright © 2022 Digital Agency. All rights reserved.
from skspatial.objects import Point
import pybullet
from .base_bullet import BaseBullet


class SphereBullet(BaseBullet):
    """
    球のPyBulletクラス

    :var _engine: PyBulletエンジン
    :var _object: PyBulletオブジェクト
    """

    def __init__(self, radius: float, center_point: Point) -> None:
        """
        コンストラクタ

        :param radius: 球の半径
        :type  radius: float
        :param center_point: 球の中心
        :type  center_point: skspatial.objects.Point
        """
        super().__init__()

        # 球のモデル
        geom_sphere = pybullet.createCollisionShape(
            pybullet.GEOM_SPHERE,
            radius=radius,
            physicsClientId=self._engine
        )

        # 衝突判定用の球オブジェクト
        self._object = pybullet.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=geom_sphere,
            basePosition=center_point.tolist(),
            physicsClientId=self._engine
        )

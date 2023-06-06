# Copyright © 2022 Digital Agency. All rights reserved.
from dataclasses import dataclass
from decimal import ROUND_DOWN, Decimal
from statistics import geometric_mean
from typing import overload
import math
from SpatialId.common.exception import SpatialIdError


@dataclass
class Point():
    lon: float
    lat: float
    alt: float

    def __init__(self, lon: float, lat: float, alt: float):
        # 経度は±180
        self.lon = lon
        # 緯度は±85.0511287798の範囲内(±85.0511287798は含まない)
        self.lat = lat
        # 高さは制限なし
        self.alt = alt

    @property
    def lon(self):
        return self.__lon

    @property
    def lat(self):
        return self.__lat

    @property
    def alt(self):
        return self.__alt

    @lon.setter
    def lon(self, lon: float):
        if abs(lon) > 180.0:
            raise SpatialIdError("INPUT_VALUE_ERROR")
        self.__lon = lon

    @lat.setter
    def lat(self, lat: float):
        # 小数点11桁以下は切り捨てる
        if lat >= 0:
            lat = math.floor(lat * 10 ** 10) / (10 ** 10)
        else:
            lat = math.ceil(lat * 10 ** 10) / (10 ** 10)
        if abs(lat) > 85.0511287798:
            raise SpatialIdError("INPUT_VALUE_ERROR")
        self.__lat = lat

    @alt.setter
    def alt(self, alt: float):
        self.__alt = alt


@dataclass
class Projected_Point(Point):
    x: float = 0.0
    y: float = 0.0
    def __init__(self, x: float, y: float, alt: float, lon: float = 0 , lat: float = 0):
        super().__init__(lon, lat, alt)
        self.x = x
        self.y = y
        


@dataclass
class Vertical_Point:
    alt: float = 0.0
    resolution: float = 0.0


@dataclass
class Triangle:
    p1: Point
    p2: Point
    p3: Point


@dataclass
class Projected_Triangle(Triangle):
    p1: Projected_Point
    p2: Projected_Point
    p3: Projected_Point

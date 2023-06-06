import pytest
from SpatialId.common.object.point import Point
from SpatialId.common.exception import SpatialIdError


@pytest.mark.parametrize(('lon', 'lat', 'alt'), [
    (181.753098, 35.685371, 100.0),
    (139.753098, 85.05112878, 100.0),
    (-181, 35.685371, 100.0),
    (139.753098, -85.05112878, 100.0)
])
def test_point_raise(lon, lat, alt):
    with pytest.raises(SpatialIdError):
        Point(lon, lat, alt)

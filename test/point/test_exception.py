from logging import getLogger
import pytest
from SpatialId import LOGGER_NAME
from SpatialId.common.exception import SpatialIdError

spatial_log = getLogger(LOGGER_NAME).getChild(__name__)


def test_exception_with_message():
    spatial_log.info("Exception with Message")
    with pytest.raises(SpatialIdError):
        spatial_log.info(SpatialIdError("VALUE_CONVERT_ERROR", ("coverage test",)))
        raise SpatialIdError("VALUE_CONVERT_ERROR", "coverage test")


def test_exception():
    spatial_log.info("Exception without Message")
    with pytest.raises(SpatialIdError):
        spatial_log.info(SpatialIdError("VALUE_CONVERT_ERROR"))
        raise SpatialIdError("VALUE_CONVERT_ERROR")

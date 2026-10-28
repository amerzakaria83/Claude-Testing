"""Contactless vital-sign monitoring with the TI AWR1843BOOST radar."""

__version__ = "1.0.0"

from .processing import VitalSignsProcessor, VitalsResult, VitalsSettings  # noqa: E402

__all__ = ["VitalSignsProcessor", "VitalsResult", "VitalsSettings"]

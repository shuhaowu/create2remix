from __future__ import absolute_import, print_function, unicode_literals

# flake8: noqa

import logging

setattr(logging, "TRACE", logging.DEBUG - 1)

def trace_log(self, message, *args, **kwargs):
  if self.isEnabledFor(logging.TRACE):
    self._log(logging.TRACE, message, args, **kwargs)

setattr(logging.getLoggerClass(), "trace", trace_log)


from .create2 import Create2
from .constants import Leds

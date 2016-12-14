stader: aircraft stability derivatives
======================================

stader is a package to simplify calculations using `stability derivatives <https://en.wikipedia.org/wiki/Stability_derivatives>`_.

Quickstart
----------

::

  import stader
  derivatives = stader.load_aircraft("b747_flight_condition2")
  aircraft = stader.Aircraft(derivatives)
  lateralss = aircraft.lateral.lti()
  # TODO complete quickstart example

Documentation
-------------

Examples
--------

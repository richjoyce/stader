stader: aircraft stability derivatives
===

stader is a package to simplify calculations using [stability derivatives](https://en.wikipedia.org/wiki/Stability_derivatives)

Quickstart
----------

```python
import stader
derivatives = stader.read_json("examples/b747_flight_condition2.json")
aircraft = stader.Aircraft(derivatives)
lateralss = aircraft.lateral.lti()
# TODO complete quickstart example
```

Documentation
-------------

Examples
--------

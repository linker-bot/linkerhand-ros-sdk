^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for linkerbot_hand_ros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This changelog follows Semantic Versioning. Every pull request that changes
driver behavior, configuration, dependencies, installed tools, or user-facing
documentation must update the ``Unreleased`` section. At release time, move
those entries to a dated version section and create a new empty ``Unreleased``
section.

Unreleased
----------

* Publish the SDK's command-feedback origin, firmware-specific capability
  fallback, parameter-save operation, and angle-feedback health metrics.
* Add ``query_only``, ``command_feedback_only``, and ``adaptive`` angle feedback
  configuration and matching benchmark accounting.
* Wake state publication from SDK cache revisions, batch changed command
  statuses, and move periodic diagnostics and administrative work off the
  command hot path.
* Reduce state conversion and timestamp allocation overhead while preserving
  per-sequence publication and full command-stage history.

0.99.0 (2026-08-28)
-------------------

* Split transport-neutral messages into ``linkerbot_hand_msgs`` and retain the
  Lifecycle driver and tools in ``linkerbot_hand_ros``.
* Replace the 0.3 compatibility driver with the single Hand SDK 1.0
  ``HandDevice`` lifecycle, typed capabilities, typed state, and non-blocking
  command contract.
* Add a persistent typed command client, multi-hand in-process container,
  detailed diagnostics, and angle/tactile frequency benchmark tooling.
* Isolate callback groups and command-status publication from the high-rate
  command callback path.

0.3.3 (2026-08-27)
------------------

* Synchronize with Hand SDK 0.2 raw/percent semantics and publish typed tactile
  state.

0.3.2 (2026-08-25)
------------------

* Correct ROS 2 Jazzy installation dependency resolution.

0.3.1 (2026-08-25)
------------------

* Add joint frequency benchmark tooling and acceptance criteria.
* Align ROS telemetry and command fields with the Hand SDK physical-unit
  contract.

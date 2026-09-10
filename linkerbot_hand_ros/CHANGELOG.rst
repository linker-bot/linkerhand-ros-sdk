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

* Open O30 on a bus running any data bitrate. A profile that leaves the data
  phase unconstrained (O30 runs BRS off) used to get a fixed 5 Mbit/s
  substituted when the caller set none, and SocketCAN asserts the requested data
  bitrate against the link: a correctly configured 1M, 2M or 7M bus therefore
  failed to open, with an error suggesting the user change a perfectly good bus
  to 5 Mbit/s. With ``auto_reconfigure`` enabled it was worse, since the driver
  would take the link down and back up at 5 Mbit/s and interrupt every other
  node on that bus. The SocketCAN path now adopts the data bitrate the link is
  actually running, read through the SDK's public discovery source, and keeps the
  recommended 5 Mbit/s only when the link cannot be read so the transport still
  emits its "channel is not configured, run ip link set ..." guidance. An
  explicit ``data_bitrate`` remains authoritative and skips the link lookup, a
  profile that does declare a data bitrate (O20, L30) keeps the strict
  conflict check, and the vendor CAN FD path is unchanged because there the
  value configures the adapter rather than being matched against a Linux link.

* Add CI and a local pre-commit gate. ``.gitea/workflows/ci.yml`` runs
  ``ruff check .`` and ``ruff format --check .`` on pushes to ``main`` and on
  pull requests, covering all three packages without building anything.
  ``.pre-commit-config.yaml`` uses ``repo: local`` to call the ruff already in
  the environment rather than pulling ``ruff-pre-commit``, which ships its own
  pinned version, so the hook, a manual run and CI stay on one ruff and one rule
  set. Each clone needs ``pre-commit install`` once, which installs both the
  commit-time and the push-time hook; the push hook is what actually gates
  code today, because with no Gitea runner online the workflow only queues. CI does not run pytest yet:
  the driver tests need ROS 2 Jazzy, a built ``linkerbot_hand_msgs``, and
  ``linkerbot-hand`` plus ``linkerbot-transport``, neither of which is on a
  public index.

* Decide force topics from the declared ``payload_projection`` instead of
  reverse-engineering ``raw_encoding`` and the number of ``shape`` dimensions.
  Those two describe the wire format, which is independent of which sample field
  a profile fills, and they were entangled: O30's descriptor force is also
  one-dimensional, so the descriptor test had to run first or O30 would have
  been treated as a scalar channel. Capability decisions in the state publisher
  and the state conversion now read the capability only. Topic creation is
  unchanged for all seven models. Requires a Hand SDK that declares
  ``payload_projection``.

* Reject a polling rate for a channel the firmware turned out not to answer.
  ``validate_polling_plan()`` checked ``telemetry_channel(...) is not None``,
  which stays true after runtime degradation, so ``ros2 param set force_hz`` was
  accepted on a degraded channel and then silently produced no data. It now also
  requires ``telemetry_supported()``, which excludes runtime-disabled channels.
  Publisher creation deliberately keeps using declared presence: publishers are
  built once per configure, and a degraded channel keeps its publisher so a
  reconnect that re-verifies the channel does not find it without one.

* Publish the decoded per-joint fault bits instead of dropping them. The SDK
  reports ``FaultSample.statuses`` for a model whose protocol profile documents
  its fault bits, but the conversion layer only read ``sample.raw``, so every
  subscriber had to re-implement the bit masks and look up each bit's meaning in
  the protocol. Adds the ``linkerbot_hand_msgs`` types
  ``HandFaultStatusState`` and ``HandJointFaultStatus`` and the lifecycle topic
  ``~/state/fault_statuses``, created only for a channel whose capability sets
  ``decoded_statuses``, which today means O30. Each record keeps the exact
  protocol byte in ``raw`` next to the named flags. ``TelemetryCapability``
  gains the matching ``decoded_statuses`` field so the published capability
  metadata stays in step with the SDK. The existing ``~/state/faults`` topic is
  unchanged in name, content, and behaviour. Requires rebuilding
  ``linkerbot_hand_msgs``, and a Hand SDK that declares ``decoded_statuses``.

* Stop leaking the two O30 descriptor-tactile lifecycle publishers
  (``~/state/tactile_sensors`` and ``~/state/normal_force_resultant``) on
  deactivate and cleanup. ``SnapshotPublisher.destroy()`` now reuses the exact
  publisher lists built during construction instead of re-enumerating the
  attributes by hand, so every created publisher is always destroyed, and
  repeated ``destroy()`` calls are now idempotent.

* Replace the incorrect public hand model name ``L25/l25`` with ``L20/l20``
  in driver configuration and documentation, matching the corrected Hand SDK
  capability name.
* Add Chinese and English quick-start instructions covering the complete
  Transport, Hand, ROSIDL, driver, source-build, and post-install verification
  chain without requiring hardware.

1.0.0 (2026-09-01)
------------------

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

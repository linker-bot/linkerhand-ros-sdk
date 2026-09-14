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

* ``tag`` now creates the Gitea Release as well, so a release is a signed tag
  plus a Release and nothing else. Creating it was the step ``v2.0.0`` missed
  even after everything else was repaired, leaving it the only tag in this
  repository without a Release, because no other gate made it visible. Both go in
  one command because there is nothing to build in between and HEAD is the commit
  being tagged, so the body — generated from the two ``CHANGELOG.rst`` sections,
  with the reStructuredText idioms these files use (inline literals and ``*``
  bullets) converted to Markdown and the authoritative text linked — describes
  exactly the tagged source.

* **This repository no longer publishes to the package registry, and the Release
  carries no artifact.** That is a deliberate departure from its own history: six
  versions of the driver sit in the registry (``0.3.1`` through ``2.0.0``) and
  ``v1.0.0``'s Release attached a wheel and sdist. The artifact is inert.
  Installing ``linkerbot-hand-ros==2.0.0`` from the registry into a clean
  virtualenv succeeds, and ``import linkerbot_hand_ros.node`` then raises
  ``ModuleNotFoundError: No module named 'rclpy'``; the wheel contains no
  ``linkerbot_hand_msgs`` at all, because that is a ROSIDL package with no
  platform-independent wheel, and ``setup.py`` declares no ``rclpy`` because ROS
  dependencies live in ``package.xml``. No document installs it and no consumer
  was found. Publishing a package that cannot run is worse than not publishing
  one. The documented install path is unchanged: ``git clone --branch v<version>``
  plus ``colcon build``.

* Add a ``release`` subcommand for the recovery case — a tag that exists with no
  Release, which is exactly what happened to ``v2.0.0``. It requires
  ``v<version>`` to exist **and to point at HEAD**, because the body comes from
  the changelogs in the working tree and would otherwise describe a different
  commit. ``check_working_tree`` takes ``require_main`` so that path can run from
  a detached tag checkout while still rejecting modified tracked files.

* Both ``tag`` and ``release`` run locally only; CI keeps just ``verify-tag``.
  Their gates include this repository's pytest suite, which needs ROS jazzy and
  colcon-built messages, while the available runners are plain ubuntu images with
  no ROS — a job needing them would not queue for capacity, it would fail.

* Add a release flow: ``scripts/release.py`` (``check`` / ``tag`` / ``release`` /
  ``verify-tag``) and ``.gitea/workflows/release.yml``, documented in
  ``scripts/README.md``. The documented install path is
  ``git clone --branch v<version>`` plus ``colcon build``, because
  ``linkerbot_hand_msgs`` is a ROSIDL interface with no platform-independent
  wheel — so a release centres on the tag. Two things went wrong in 2.0.0 and
  both are now gated. The tag was pushed while the documentation fix was
  unmerged, so it pointed at a commit whose docs said 1.0.0 and had to be
  deleted from the remote and redone; ``check`` therefore requires a clean
  ``main`` that equals ``origin/main`` and requires every version the
  documentation names to match. And ``setup.py`` declared
  ``linkerbot-hand>=2.0.0,<3.0.0`` while the registry held only 1.0.0, so
  following the documentation produced ``No solution found``; ``check`` now
  queries the registry and refuses when the documented pin is absent or no
  published version satisfies the declared range. That check spans two
  repositories, which is the kind a person reliably skips. Other gates: the four
  version declarations must agree (``linkerbot_hand_gui`` is versioned
  independently and excluded), both changelogs need a dated section and an empty
  ``Unreleased``, the range quoted in the documentation must be the one
  ``setup.py`` declares, no Markdown document may name a version that is neither
  this release nor the pinned hand or transport version, the tag must be unused,
  and ``ruff`` and ``pytest`` must pass. Historical version references belong in
  ``CHANGELOG.rst``, which the documentation scan excludes. CI runs
  ``verify-tag`` on a ``v*`` push — the same script, so there is one set of gates
  rather than one in CI and another on a developer machine — confirming the tag
  matches the versions declared by the commit it points at. With no Gitea runner
  online the workflow only queues, so the local script is the part that takes
  effect today.

* ``test_python_and_ros_package_versions_match`` no longer asserts the literal
  ``"2.0.0"``. The invariant is that the four version declarations agree, not
  that they hold a particular value; the literal meant editing this test every
  release, and ``scripts/release.py`` reads the version and gates on it instead.

* Point the install and build instructions at 2.0.0. Seven documents still
  pinned 1.0.0 in 31 places: the virtualenv directory name, the
  ``'linkerbot-hand==1.0.0'`` pin, the ``linkerbot-hand>=1.0.0,<2.0.0`` range,
  two "both Python packages at 1.0.0" verification notes, and
  ``git clone --branch v1.0.0``. The last one mattered most: the documented
  delivery is two source packages from one tag, so following it produced a 1.0.0
  workspace whose ``setup.py`` cannot use hand 2.0.0. References to
  ``linkerbot-transport`` stay at 1.0.0, which is still its released version.

2.0.0 (2026-09-11)
------------------

This release is not source or wire compatible with 1.0.0. The two changes that
require action:

* **Requires Hand SDK 2.0.0.** ``setup.py`` now declares
  ``linkerbot-hand>=2.0.0,<3.0.0``. The driver needs the SDK's
  ``payload_projection`` and ``decoded_statuses`` capability fields; against
  1.0.0 the node cannot start. Note that the 1.0.0 declaration
  (``>=1.0.0,<2.0.0``) was wrong in both directions and should not be relied on.

* **``linkerbot_hand_msgs`` message hash changed**, so nodes built against msgs
  1.0.0 cannot exchange ``~/capabilities`` with nodes built against 2.0.0.
  Rebuild every package that uses these messages. Details in that package's
  changelog.

Two model-facing renames also landed: the public hand model ``L25/l25`` became
``L20/l20``, and on L20Lite/L20 the driver now publishes
``~/state/measured_speeds`` instead of ``~/state/configured_speeds`` (the
protocol returns the live speed, not a configured one).


* Open O30 on a CAN FD bus running any data bitrate; 5 Mbit/s is recommended.
  In 1.0.0 O30 could not be opened from the driver at all: O30 runs BRS off, so
  its data phase equals the nominal rate and the SDK reports
  ``capability.transport.data_bitrate`` as ``None``, which the transport factory
  rejected with "does not declare data_bitrate". The SocketCAN path now adopts
  the data bitrate the link is actually running, read through the SDK's public
  discovery source, and uses the recommended 5 Mbit/s only when the link cannot
  be read, so the transport still emits its "channel is not configured, run
  ip link set ..." guidance rather than a confusing rate mismatch. An explicit
  ``data_bitrate`` remains authoritative and skips the link lookup; a profile
  that does declare a data bitrate (O20, L30) keeps the strict conflict check;
  and the vendor CAN FD path is unchanged, because there the value configures
  the adapter rather than being matched against a Linux link. Because the
  requested
  rate now matches the link, ``auto_reconfigure`` no longer takes the link down
  and back up, so it cannot interrupt other nodes on the same bus.

* Add CI and a local pre-commit gate. ``.gitea/workflows/ci.yml`` runs
  ``ruff check .`` and ``ruff format --check .`` on pushes to ``main`` and on
  pull requests, covering all three packages without building anything.
  ``.pre-commit-config.yaml`` uses ``repo: local`` to call the ruff already in
  the environment rather than pulling ``ruff-pre-commit``, which ships its own
  pinned version, so the hook, a manual run and CI stay on one ruff and one rule
  set. Each clone needs ``pre-commit install`` once, which installs both the
  commit-time and the push-time hook; the push hook is what actually gates
  code today, because with no Gitea runner online the workflow only queues. CI
  does not run pytest yet: the driver tests need ROS 2 Jazzy and a built
  ``linkerbot_hand_msgs``, the only registered runner is offline and carries no
  ROS label, and reaching ``linkerbot-hand`` and ``linkerbot-transport`` needs a
  read-only package token as an Actions secret. Those packages are published to
  this instance's own PyPI registry, so the dependencies themselves are
  obtainable; the runner and the secret are what is missing.

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

* Publish the O30 descriptor tactile and normal-force-resultant state on the new
  lifecycle topics ``~/state/tactile_sensors`` and
  ``~/state/normal_force_resultant``. Both are created only for a model whose
  force capability produces the descriptor tactile projection, which today means
  O30. Device values are published as reported with no unit conversion. When the
  firmware does not support tactile reads, the existing SDK capability
  degradation and explicit error status apply. Requires rebuilding
  ``linkerbot_hand_msgs`` for the four new message types.

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

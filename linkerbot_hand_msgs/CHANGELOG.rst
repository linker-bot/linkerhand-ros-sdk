^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for linkerbot_hand_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This changelog follows Semantic Versioning. Every pull request that changes a
public message, service, constant, field meaning, or compatibility requirement
must update the ``Unreleased`` section. At release time, move those entries to
a dated version section and create a new empty ``Unreleased`` section.

Unreleased
----------

1.0.0 (2026-09-01)
------------------

* Add firmware-specific unsupported telemetry and parameter-save capability
  fields so clients can distinguish model support from current-firmware
  availability.
* Clarify device-command feedback as current device state rather than a target
  echo or protocol acknowledgement.
* Add publisher identity and timing metadata to command status records and
  publish changed command statuses in bounded batches.

0.99.0 (2026-08-28)
-------------------

* Introduce the independently discoverable ``linkerbot_hand_msgs`` package.
* Define typed capabilities, device information, joint state, telemetry,
  tactile, stall-protection, command-status, and management interfaces for the
  Hand SDK 1.0 contract.
* Keep protocol raw, percent, and optional physical-angle representations in
  separate fields with explicit validity metadata.

# Unimplemented Features

This document points to the central file that now holds deferred control work:

- `/src/slvrov_nodes_python/slvrov_nodes_python/unimplemented_features.py`

## Purpose

The active node files were intentionally decluttered so they only describe
implemented runtime behavior. Deferred features now live in one place instead
of being scattered as TODOs and commented code across the calibrator, logic,
and launch files.

## Deferred Items

The current deferred set includes:

- claw action support
- button fallback arbitration
- text redundancy input
- expanded mapping-file fields such as `skipped_actions`

## Rationale

- Keeping deferred work centralized makes the active control path easier to
  inspect and less likely to drift away from actual runtime behavior.
- Removing inactive claw handling from the live nodes reduces the chance of
  partial implementations affecting calibration or output command routing.

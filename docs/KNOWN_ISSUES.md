# Known limitations and resolved issues

## Open

1. Passage A: the physical robot fits in a centered nominal pose, but the current two-sigma yaw sweep and the route from the robot's current start to A's approach fail the selected clearance limits. The original Navfn approach passed too close to the lower wall.
2. Passage B: two-direction staged route planning and independent swept-footprint validation passed; physical traversal and a live local costmap at the opening were not tested.
3. Full-arena autonomous navigation and controller execution of the staged passage candidates are unverified.
4. Camera-guided cube approach and contact control remain unvalidated. Autonomous cube pushing and a full three-cube delivery have not been demonstrated.
5. The eight-minute requirement has not been measured on a full autonomous run.
6. AMCL accuracy and covariance under longer travel and at the openings still need independent validation; the 0.335 m Phase 5F trial had about 0.0936 m relative goal error. A successful Nav2 status is not independent collision evidence.
7. The default Nav2 map metadata includes a machine-specific absolute PGM path. Use the [portable example](../maps/competition_mapping/20261125T103259Z_572d1928/nav2_portable_example.yaml) on another checkout and verify map dimensions/unknown cells. The original saved map must remain unchanged.
8. Local costmap passage cells were outside the stationary 3×3 m rolling window in Phase 5H. They need measurement at the actual approach before movement.

## Fixed in earlier phases

AMCL zero-covariance initialization (5C), a trial-monitor sensor-freshness false alarm (5E-2), excessive partial-view cube uncertainty growth (5G), and lack of an oriented polygon clearance diagnostic (5H) were addressed and tested. Their historical reports remain useful; do not present their old failures as current defects.

Evidence: [Phase 5F](../reports/phase5f_extended_navigation_validation.md), [5G](../reports/phase5g_cube_tracking_costmap_validation.md), [5H](../reports/phase5h_narrow_passage_validation.md).

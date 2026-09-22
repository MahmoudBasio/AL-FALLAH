# Third-Party Platforms and Dependencies

## TIAGo / PAL Robotics

The first AL-FALLAH simulation demo uses the TIAGo mobile manipulator as a third-party research and simulation platform.

TIAGo, its mechanical design, robot description, simulation assets, and PAL Robotics packages are the work/property of their respective authors and owners. AL-FALLAH does not claim those assets as its own robot design.

The purpose of using TIAGo in Demo 01 is to provide an existing ROS 2 / MoveIt-capable mobile manipulator so that the project can validate higher-level concepts such as target geometry, trimming-path generation, manipulator reachability, and virtual cutting before the custom AL-FALLAH robot is integrated.

TIAGo source packages are not copied into this repository. Users of the demo are expected to obtain and install the relevant PAL Robotics/TIAGo dependencies separately and comply with their licenses and terms.

## General policy

When external robot models, libraries, datasets, or software are used in future demos, they should be documented in this file or in the relevant demo README. Demo code should clearly distinguish AL-FALLAH-specific work from third-party assets.

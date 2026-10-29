# Contributing to argos3-ros2-bridge

Thank you for your interest in improving the ARGoS3–ROS 2 bridge! This document explains what the bridge is meant to do, how to propose changes, and what we expect from a pull request.

## How to contribute

1. **Open an issue first.** Describe the problem, your proposed solution, and how it would be configured. Please wait for a maintainer to agree on the design before investing significant time in an implementation.
2. **Fork the repository and create a branch** for your change.
3. **Keep pull requests small:** one feature or fix per pull request. Several focused pull requests are much easier to review than one large one.
4. **Open a pull request** that references the issue (for example `Closes #15`) and fills in the checklist below.

Bug reports are welcome as issues. Please include your ROS 2 distribution, OS, ARGoS version, the `.argos` configuration (or a minimal one that reproduces the problem), and the full error output.

## Requirements for pull requests

### Code style

Follow the [ROS 2 C++ style guide](https://docs.ros.org/en/rolling/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html). The style is enforced by the ament linters (`uncrustify`, `cpplint`, `cppcheck`, `lint_cmake`, `xmllint` and `copyright`), which run as part of `colcon test`.

New source files must start with the license header:

```cpp
// Copyright <year> The argos3-ros2-bridge Authors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
```

`ament_uncrustify --reformat <path>` fixes most formatting issues automatically.

### Tests

**All tests must pass, and you must provide proof.** From your workspace root:

```bash
colcon build --packages-select argos3_ros2_bridge
AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1 colcon test --packages-select argos3_ros2_bridge
colcon test-result --verbose
```

**Paste the complete output of `colcon test-result --verbose` into your pull request description**, together with the ROS 2 distribution and OS you tested on. The `AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1` variable makes sure `cppcheck` actually runs on Ubuntu 22.04 instead of being skipped.

Build and test in a shell where no Python virtual environment is active; otherwise the ROS 2 message generation may fail.

The maintainers will build and test every pull request themselves before merging.

If your change adds functionality, please add tests for it where practical, or describe how you tested it (for example the `.argos` file and ROS 2 nodes you used).

### Performance

If your change affects performance or scalability, include before-and-after measurements: number of robots, hardware, and the metric you measured (for example real-time factor or CPU usage).

### Compatibility

New features must be off by default. Do not change the names, types or behaviour of existing topics, parameters or configuration options without discussing it in an issue first.

## Use of AI tools

We do not accept contributions that are generated purely by AI. This applies to code, issues, pull request descriptions and review comments.

Using AI tools as an aid is fine, provided that:

- you understand every line you submit and can explain and defend it in review,
- you have built, run and tested the change yourself, and

Pull requests and issues that appear to be AI-generated without meaningful human authorship, review or testing may be closed without detailed review.

## Licensing, sign-off and authorship

Any contribution that you make to this repository will
be under the Apache 2 License, as dictated by that
[license](http://www.apache.org/licenses/LICENSE-2.0.html):

~~~
5. Submission of Contributions. Unless You explicitly state otherwise,
   any Contribution intentionally submitted for inclusion in the Work
   by You to the Licensor shall be under the terms and conditions of
   this License, without any additional terms or conditions.
   Notwithstanding the above, nothing herein shall supersede or modify
   the terms of any separate license agreement you may have executed
   with Licensor regarding such Contributions.
~~~

Please sign off your commits to certify that you wrote the code or otherwise have the right to submit it under this license, as described by the [Developer Certificate of Origin](https://developercertificate.org/):

```bash
git commit -s -m "Add use_sim_time support"
```

This adds a `Signed-off-by: Your Name <your.email@example.com>` line to the commit message.

If your contribution is merged, add your name to the [AUTHORS](AUTHORS) file in the same pull request. The copyright line `The argos3-ros2-bridge Authors` in the source files refers to the people listed there.

## Pull request checklist

Copy this into your pull request description:

```markdown
- [ ] Related issue: #
- [ ] One feature or fix only
- [ ] New features are disabled by default
- [ ] New files have the license header
- [ ] All tests pass; `colcon test-result --verbose` output pasted below
- [ ] Tested on (ROS 2 distribution / OS):
- [ ] Performance measurements included (if relevant)
- [ ] AI tool use disclosed (if any)
- [ ] Commits are signed off (`git commit -s`)
- [ ] Name added to AUTHORS
```

## Questions

For questions about using the bridge, open an issue on GitHub. If you use the bridge in your research, please cite our ANTS 2026 paper, "ROS-2-ARGoS Bridge: Scalable Simulations of Swarms of 1000 and More Robots".
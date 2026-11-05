# Contributing to Isaac Sim

Isaac Sim accepts a limited set of external code and documentation
contributions. Before opening a pull request, read these contribution terms and
the [external contribution path policy](CONTRIBUTING_PATHS.md). A pull request
that changes any path outside that policy is out of scope and will be closed.

Use [GitHub Discussions](https://github.com/isaac-sim/IsaacSim/discussions) for
questions and feature proposals. Use GitHub Issues only for work with an agreed,
executable scope. Do not report security vulnerabilities through GitHub; follow
[SECURITY.md](SECURITY.md) instead.

## Contribution terms

By intentionally submitting a contribution to this repository, you confirm that:

- You submit the contribution under the repository's
  [Apache License 2.0](LICENSE), without additional terms or conditions.
- You certify the [Developer Certificate of Origin 1.1](https://developercertificate.org/)
  for every commit in the contribution.
- You wrote the contribution or otherwise have the right and authority to
  submit it, including any authorization required by your employer.
- Your contribution does not knowingly contain confidential information,
  trade secrets, or third-party material that you lack permission to submit.
- You disclose third-party code and its license in the pull request. New
  dependencies and vendored code are outside the initial contribution scope.
- You preserve existing copyright, license, and attribution notices. For a new
  file, identify the actual copyright holder and include
  `SPDX-License-Identifier: Apache-2.0` using the comment syntax for its
  language. Do not identify NVIDIA as the copyright holder unless NVIDIA owns
  that copyright.
- Submission does not guarantee acceptance. NVIDIA maintainers may reject a
  contribution based on scope, quality, compatibility, security, legal, release,
  or maintenance concerns.

For example, a new Python file uses this header with the actual year and
copyright holder:

```python
# SPDX-FileCopyrightText: Copyright (c) <year> <copyright holder>
# SPDX-License-Identifier: Apache-2.0
```

## Sign off every commit

Add a DCO sign-off using your real name and an email address that identifies
you:

```text
Signed-off-by: Your Name <your.email@example.com>
```

Git can add the line when you create a commit:

```bash
git commit -s -m "Describe the change"
```

If you amend or rebase a pull request, retain a valid sign-off on every commit.

## Contribution scope

External pull requests may change only Python, C, C++, CUDA, Markdown, and
reStructuredText files under `source/extensions/`. CI evaluates the
machine-readable policy in
[`tools/github/ci/path_filter.yml`](tools/github/ci/path_filter.yml).
[CONTRIBUTING_PATHS.md](CONTRIBUTING_PATHS.md) presents that policy for
contributors and is generated from the machine-readable policy.

The initial scope does not include manifests, build files, dependencies,
workflows, generated files, assets, or binaries. If your change requires one of
those files, open an Issue or Discussion for a maintainer before writing the
change.

NVIDIA maintainers handle any required changes to prohibited metadata, such as
extension version manifests, through a separate internal change. Do not include
such files in an external pull request.

## Validate your change

Run the repository formatter before opening or updating a pull request:

```bash
./format_code.sh
```

On Windows, run `format_code.bat`. See the
[coding style guidelines](docs/overview/guidelines.rst) for language-specific
requirements and the [quick start](README.md#quick-start) for build
instructions.

After building, run the test launcher for each affected extension. For example,
on Linux x86-64, run:

```bash
./_build/linux-x86_64/release/tests/tests-<extension-name>.sh
```

On Windows, run
`_build\windows-x86_64\release\tests\tests-<extension-name>.bat`. Record the
commands and results in the pull request. If you cannot run a relevant test,
state why so maintainers can determine the required coverage.

## Pull request process

External contributions must target the repository's current default branch.
Contributions targeting other branches are not currently accepted unless
requested by an NVIDIA maintainer. You can find the current default branch on
the [repository home page](https://github.com/isaac-sim/IsaacSim).

`develop` tracks the next major release, currently 7.0. Versioned branches such
as `release/6.3` track their respective release lines.

1. Before implementing a feature or public API change, obtain a public comment
   from an NVIDIA maintainer explicitly confirming that the proposed scope is
   suitable for contribution. A focused bug fix may proceed without prior
   approval.
2. Create a topic branch from the repository's current default branch.
3. Keep the change within the external contribution path policy.
4. Add or update tests when they can be expressed in allowed source files, and
   complete the validation steps above.
5. Sign off every commit and open the pull request against that same branch.
6. Respond to public review and continuous integration feedback. Push a new
   revision when a maintainer requests a change.

An NVIDIA employee must approve the use of NVIDIA-hosted test capacity. NVIDIA
maintainers arrange the internal integration of an eligible pull request. The
exact pull request revision must pass the required internal integration,
security, and review gates before merge. Pushing a new commit invalidates the
result, and the new revision must be validated again.

Other than the name and email required for your DCO sign-off, do not include
credentials, unnecessary personal or sensitive information, internal URLs, or
private logs in a pull request, commit, test output, or public discussion.

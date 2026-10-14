#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Resolve the latest published nvcr.io/nvidia/isaac-sim tag from the NGC catalog tags page."""

from __future__ import annotations

import argparse
import re
import sys
import urllib.error
import urllib.request

NGC_TAGS_URL = "https://catalog.ngc.nvidia.com/orgs/nvidia/-/containers/isaac-sim/-/tags"
IMAGE_REPO = "nvcr.io/nvidia/isaac-sim"
FALLBACK_TAG = "6.0.1"

#: Exit status when the catalog could not be read and ``--allow-fallback`` was not given.
EXIT_UNRESOLVED = 2 + 2

#: Socket timeout for the NGC tags page fetch.
HTTP_TIMEOUT_SECONDS = float("30")

#: Right-pad version tuples so ``6.10.0`` sorts above ``6.9.0``.
SEMVER_KEY_WIDTH = 2 + 2

#: Calendar-style catalog tags such as ``2023.1``.
_YEAR_RELEASE_TAG = r"20\d{2}\.\d+(?:\.\d+)?"
_STABLE_TAG_RE = re.compile(r"^(?:(\d+)\.(\d+)\.(\d+)|" + _YEAR_RELEASE_TAG + r")$")
_PRERELEASE_RE = re.compile(r"(?:-dev|_ea|_preview|rc|alpha|beta)", re.IGNORECASE)
_LATEST_RE = re.compile(r"Latest\s*\(([^)]+)\)")
_NV_TEXT_RE = re.compile(r'data-testid="nv-text">([^<]+)</span>')
_NVCR_RE = re.compile(r"nvcr\.io/nvidia/isaac-sim:([0-9][^\s\"'<>\\]+)")


def _is_stable(tag: str) -> bool:
    """Report whether a catalog tag names a stable release.

    Args:
        tag: Tag label as it appears on the catalog page.

    Returns:
        True for a release tag such as ``6.1.0`` or ``2023.1``, False for
        pre-release tags such as ``6.2.0-dev`` or ``6.2.0rc1``.
    """
    tag = tag.strip().rstrip("\\")
    if _PRERELEASE_RE.search(tag):
        return False
    return bool(_STABLE_TAG_RE.match(tag))


def _semver_key(tag: str) -> tuple[int, ...]:
    """Build a numeric sort key so tags order by version rather than lexically.

    Args:
        tag: Tag label containing one or more numeric components.

    Returns:
        A fixed-width tuple of the tag's numbers, right-padded with zeros, so
        that ``6.10.0`` sorts above ``6.9.0``.
    """
    nums = [int(p) for p in re.findall(r"\d+", tag)]
    pad = max(0, SEMVER_KEY_WIDTH - len(nums))
    return tuple(nums + [0] * pad)


def _is_product_line_tag(tag: str) -> bool:
    """Return whether the tag is Isaac Sim product semver rather than a calendar year.

    Catalog pages still list 2022/2023-style tags. Those majors are years, so a
    naive numeric max would prefer ``2023.1.1`` over ``6.1.0``.

    Args:
        tag: Tag label containing one or more numeric components.

    Returns:
        True when the leading number is a product major (below 100), False for
        calendar-year tags such as ``2023.1.1``.
    """
    nums = [int(p) for p in re.findall(r"\d+", tag)]
    return bool(nums) and nums[0] < (10 * 10)


def _tag_sort_key(tag: str) -> tuple[int, ...]:
    """Sort key that ranks product-line tags above calendar-year tags.

    Args:
        tag: Tag label containing one or more numeric components.

    Returns:
        A tuple whose first element is 1 for product-line tags and 0 for
        calendar-year tags, followed by the numeric semver key.
    """
    product = 1 if _is_product_line_tag(tag) else 0
    return (product, *_semver_key(tag))


def fetch_tags_html(url: str, timeout: float = HTTP_TIMEOUT_SECONDS) -> str:
    """Download the NGC catalog tags page.

    Args:
        url: Tags page to read.
        timeout: Socket timeout in seconds.

    Returns:
        The page body decoded as UTF-8, with undecodable bytes replaced.

    Raises:
        urllib.error.URLError: The page could not be reached.
        OSError: The connection failed or timed out.
    """
    req = urllib.request.Request(
        url,
        headers={
            "User-Agent": "isaac-sim-web-viewer-skill/1.0",
            "Accept": "text/html",
        },
    )
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        return resp.read().decode("utf-8", errors="replace")


def parse_latest_tag(html: str) -> str | None:
    """Extract the newest stable tag from a catalog tags page.

    Prefer the tag the page itself marks as Latest. When that label is absent
    or names a pre-release, fall back to the highest stable tag on the page.

    Args:
        html: Body of the catalog tags page.

    Returns:
        The tag label, or None when the page exposes no stable tag.
    """
    latest_hits = _LATEST_RE.findall(html)
    for hit in latest_hits:
        tag = hit.strip()
        if _is_stable(tag):
            return tag

    labels = [m.strip().rstrip("\\") for m in _NV_TEXT_RE.findall(html)]
    nvcr_tags = [m.strip().rstrip("\\") for m in _NVCR_RE.findall(html)]
    candidates = [t for t in labels + nvcr_tags if _is_stable(t)]
    if not candidates:
        return None
    return max(set(candidates), key=_tag_sort_key)


def resolve_image(url: str = NGC_TAGS_URL) -> tuple[str, str]:
    """Resolve the image reference the caller should pull.

    Args:
        url: Tags page to read.

    Returns:
        A pair of image reference and source. The source is ``catalog-latest``
        when the tag came from the catalog, and ``fallback`` when the catalog
        was unreadable and the hardcoded tag was substituted. Callers must
        branch on the source rather than trusting the reference, because a
        fallback reference is very likely stale.
    """
    try:
        html = fetch_tags_html(url)
        tag = parse_latest_tag(html)
        if tag:
            return f"{IMAGE_REPO}:{tag}", "catalog-latest"
    except (urllib.error.URLError, TimeoutError, OSError) as exc:
        print(
            f"WARN: could not read NGC tags page ({exc}); using {IMAGE_REPO}:{FALLBACK_TAG}",
            file=sys.stderr,
        )
    return f"{IMAGE_REPO}:{FALLBACK_TAG}", "fallback"


def main() -> int:
    """Print the image reference to resolve, or fail when Latest is unknown.

    Returns:
        0 when the tag came from the catalog, or when a fallback was explicitly
        permitted. EXIT_UNRESOLVED when the catalog could not be read and the
        caller did not pass ``--allow-fallback``; nothing is written to stdout
        in that case, so a stale tag can never be consumed as Latest.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", default=NGC_TAGS_URL, help="NGC Isaac Sim tags page URL")
    parser.add_argument("--tag-only", action="store_true", help="Print only the tag, not the full image ref")
    parser.add_argument(
        "--allow-fallback",
        action="store_true",
        help=f"Print {IMAGE_REPO}:{FALLBACK_TAG} instead of failing when the catalog is unreadable",
    )
    parser.add_argument(
        "--offline",
        action="store_true",
        help="Do not fetch the catalog (self-test). Requires --allow-fallback.",
    )
    args = parser.parse_args()
    if args.offline:
        if not args.allow_fallback:
            print(
                "ASK_USER_IMAGE_TAG: --offline requires --allow-fallback so a stale "
                f"tag cannot be consumed as Latest. Pass both, or omit --offline.",
                file=sys.stderr,
            )
            return EXIT_UNRESOLVED
        image = f"{IMAGE_REPO}:{FALLBACK_TAG}"
        print(f"OK: NGC tags source=offline-fallback image={image}", file=sys.stderr)
        if args.tag_only:
            print(image.rsplit(":", 1)[-1])
        else:
            print(image)
        return 0

    image, source = resolve_image(args.url)

    if source != "catalog-latest" and not args.allow_fallback:
        print(
            f"ASK_USER_IMAGE_TAG: could not determine Latest from {args.url}. "
            f"Ask the operator for a published tag rather than assuming {IMAGE_REPO}:{FALLBACK_TAG}, "
            "or re-run with --allow-fallback.",
            file=sys.stderr,
        )
        return EXIT_UNRESOLVED

    print(f"OK: NGC tags source={source} image={image}", file=sys.stderr)
    if args.tag_only:
        print(image.rsplit(":", 1)[-1])
    else:
        print(image)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

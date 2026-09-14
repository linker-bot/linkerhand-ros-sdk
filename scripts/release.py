#!/usr/bin/env python3
"""Release gate and tagger for linkerbot-hand-ros.

A release here is a signed tag plus a Gitea Release. **Nothing is uploaded** —
not to the Release and not to the package registry. The documented install path
is ``git clone --branch v<version>`` followed by ``colcon build``, because
``linkerbot_hand_msgs`` is a ROSIDL interface with no platform-independent wheel.

Six versions of the driver do sit in the registry, and ``v1.0.0``'s Release
attached a wheel and sdist, so this is a deliberate departure from that history:
the artifact is inert. Installing ``linkerbot-hand-ros==2.0.0`` from the registry
into a clean virtualenv succeeds, and ``import linkerbot_hand_ros.node`` then
raises ``ModuleNotFoundError: No module named 'rclpy'``; the wheel contains no
``linkerbot_hand_msgs`` either. No document installs it, and nothing was found
that consumes it. Publishing a package that cannot run is worse than not
publishing one.

Three things went wrong in the 2.0.0 release and all three are checked here:

* The tag was pushed while the documentation fix was still unmerged, so it
  pointed at a commit whose docs said 1.0.0. The remote tag had to be deleted
  and redone.
* ``setup.py`` declared ``linkerbot-hand>=2.0.0,<3.0.0`` while the registry only
  held 1.0.0, so ``pip install`` failed outright with ``No solution found``. The
  version number was right; nobody checked the range could resolve. That check
  spans two repositories, which is exactly the kind a person will skip.
* The Gitea Release was never created, leaving ``v2.0.0`` as the only tag in this
  repository without one — no other gate made that visible.

Usage::

    python3 scripts/release.py check              # read-only; changes nothing
    python3 scripts/release.py tag                # check, sign, push, Release
    python3 scripts/release.py release            # Release only, for a tag missing one
    python3 scripts/release.py verify-tag --tag v2.0.0   # what CI runs on a tag

Requires ``packaging`` for the version-range comparison. ``GITEA_TOKEN`` is
needed to read the registry (for the dependency check) and to create the Release.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import subprocess
import sys
import urllib.error
import urllib.request
from pathlib import Path

from packaging.requirements import Requirement
from packaging.version import Version

ROOT = Path(__file__).resolve().parent.parent
HOST = "https://gitea.linkerhub.work"
OWNER = "zhuzhixiang"
REGISTRY = f"{HOST}/api/packages/{OWNER}/pypi"
REPOSITORY = "linkerbot-hand-ros"
API = f"{HOST}/api/v1/repos/{OWNER}/{REPOSITORY}"

# Shown in the Gitea Release title. Nothing is uploaded, so there is no
# distribution name or wheel stem to track.
DIST = "linkerbot-hand-ros"

# The four places this repository states its own version. ``linkerbot_hand_gui``
# is versioned independently and deliberately absent.
VERSION_SOURCES: tuple[tuple[str, str], ...] = (
    ("linkerbot_hand_ros/setup.py", r'version="([^"]+)"'),
    ("linkerbot_hand_ros/package.xml", r"<version>([^<]+)</version>"),
    ("linkerbot_hand_msgs/package.xml", r"<version>([^<]+)</version>"),
    ("linkerbot_hand_ros/linkerbot_hand_ros/__init__.py", r'__version__ = "([^"]+)"'),
)

CHANGELOGS = (
    "linkerbot_hand_ros/CHANGELOG.rst",
    "linkerbot_hand_msgs/CHANGELOG.rst",
)

SETUP_PY = "linkerbot_hand_ros/setup.py"

# Historical version references belong in CHANGELOG.rst, which is excluded from
# the documentation scan. A Markdown document that still names an old version is
# a stale instruction, not history.
DOC_GLOB = "*.md"
VERSION_TOKEN = re.compile(r"\d+\.\d+\.\d+")


class Failure(Exception):
    """A gate rejected the release. The message says which one and why."""


def run(*args: str) -> str:
    """Run one command from the repository root, raising Failure on non-zero."""

    completed = subprocess.run(
        args, cwd=ROOT, capture_output=True, text=True, check=False
    )
    if completed.returncode != 0:
        detail = (completed.stderr or completed.stdout or "").strip()
        raise Failure(f"command failed: {' '.join(args)}\n{detail}")
    return (completed.stdout or "").strip()


def read(relative: str) -> str:
    return (ROOT / relative).read_text(encoding="utf-8")


def documents() -> list[Path]:
    """Markdown documents that may pin a version, in a stable order."""

    return sorted(
        path
        for path in ROOT.rglob(DOC_GLOB)
        if not any(part in {".git", "build", "install", "log"} for part in path.parts)
    )


def declared_version() -> str:
    """Return the version, requiring all four declarations to agree.

    ``test_python_and_ros_package_versions_match`` also asserts these agree, but
    it hardcodes the expected value and so has to be edited every release. This
    reads the value instead of asserting a literal.
    """

    found: dict[str, str] = {}
    for relative, pattern in VERSION_SOURCES:
        match = re.search(pattern, read(relative))
        if match is None:
            raise Failure(f"{relative} declares no version matching {pattern}")
        found[relative] = match.group(1)
    distinct = set(found.values())
    if len(distinct) != 1:
        detail = "; ".join(f"{name} = {value}" for name, value in found.items())
        raise Failure(f"the four version declarations disagree: {detail}")
    return distinct.pop()


def hand_requirement() -> str:
    """Return the ``linkerbot-hand`` requirement string from setup.py."""

    match = re.search(r'"(linkerbot-hand(?:\[[^\]]*\])?[^"]*)"', read(SETUP_PY))
    if match is None:
        raise Failure(f"{SETUP_PY} declares no linkerbot-hand requirement")
    return match.group(1)


def registry_versions(package: str) -> set[str] | None:
    """Return versions of ``package`` in the registry, or None when unreachable.

    Unreachable is distinguished from empty on purpose: a network failure must
    not read as "that version does not exist" and block a valid release, nor as
    "it exists" and pass an invalid one. Callers report it as unverified.
    """

    token = os.environ.get("GITEA_TOKEN") or os.environ.get("TWINE_PASSWORD")
    request = urllib.request.Request(f"{REGISTRY}/simple/{package}/")
    if token:
        request.add_header("Authorization", f"token {token}")
    try:
        with urllib.request.urlopen(request, timeout=30) as response:
            body = response.read().decode("utf-8", "replace")
    except (urllib.error.URLError, TimeoutError) as error:
        print(f"  ! registry unreachable ({error}); {package} versions not checked")
        return None
    stem = package.replace("-", "_")
    return set(re.findall(rf"{stem}-([0-9][^-]*)-py3", body))


def check_working_tree(*, require_main: bool) -> None:
    """Reject a dirty tree, and on the tagging path an unsynced branch.

    ``require_main`` is False when back-filling a Release: that runs from a
    detached tag checkout, so "on main" cannot hold. The tracked-file half still
    applies — the release notes are generated from the working tree, and
    uncommitted edits would put text in a Release that is in no commit.
    """

    if run("git", "status", "--porcelain", "--untracked-files=no"):
        raise Failure("tracked files are modified; commit or stash first")
    if not require_main:
        return
    branch = run("git", "rev-parse", "--abbrev-ref", "HEAD")
    if branch != "main":
        raise Failure(f"releases are cut from main, not {branch!r}")
    run("git", "fetch", "origin", "--quiet")
    if run("git", "rev-parse", "HEAD") != run("git", "rev-parse", "origin/main"):
        raise Failure("main differs from origin/main; pull or push first")


def check_changelogs(version: str) -> None:
    """Both package changelogs need a dated section and an empty Unreleased.

    A non-empty ``Unreleased`` means the entries were never sorted into a
    version, so the release would ship undocumented changes.
    """

    for relative in CHANGELOGS:
        text = read(relative)
        if not re.search(
            rf"^{re.escape(version)} \(\d{{4}}-\d{{2}}-\d{{2}}\)$", text, re.MULTILINE
        ):
            raise Failure(f"{relative} has no '{version} (YYYY-MM-DD)' section")
        # The section runs to the next dated header or end of file. Matching "up
        # to the next non-blank line" would not work: reStructuredText bullets
        # start with ``*``, so an unreleased entry would itself look like the end
        # of the section and the gate would pass on a non-empty Unreleased.
        match = re.search(
            r"^Unreleased\n-+\n(.*?)(?=^\d+\.\d+\.\d+ \(|\Z)",
            text,
            re.MULTILINE | re.DOTALL,
        )
        if match is None:
            raise Failure(f"{relative} has no 'Unreleased' section")
        if match.group(1).strip():
            raise Failure(
                f"{relative} still has entries under Unreleased; move them into "
                f"the {version} section before releasing"
            )


def check_documented_requirement() -> str:
    """The range quoted in the docs must be the one setup.py declares.

    Returns the requirement string so the caller can exempt the versions inside
    it from the "no unknown version" scan below.
    """

    declared = hand_requirement()
    quoted: set[str] = set()
    for path in documents():
        # A specifier must carry a digit to be a version claim: a bare
        # ``linkerbot-hand==`` naming the pin pattern rather than a version is
        # not this gate's business. Exact pins are excluded too — those are
        # checked against the range by ``check_documented_versions``, and
        # reporting one here would read as a range mismatch.
        quoted.update(
            found
            for found in re.findall(
                r"`(linkerbot-hand[<>=!~]+[0-9][^`]*)`", path.read_text("utf-8")
            )
            if not found.startswith("linkerbot-hand==")
        )
    stale = quoted - {declared}
    if stale:
        raise Failure(
            f"documentation quotes {', '.join(sorted(stale))} but {SETUP_PY} "
            f"declares {declared}"
        )
    return declared


def _single_pin(pattern: str, label: str) -> str:
    """Return the one value ``pattern`` captures across every document."""

    found: set[str] = set()
    for path in documents():
        found.update(re.findall(pattern, path.read_text("utf-8")))
    if not found:
        raise Failure(f"no document pins {label}")
    if len(found) > 1:
        raise Failure(f"documents disagree on {label}: {', '.join(sorted(found))}")
    return found.pop()


def check_documented_versions(version: str, requirement: str) -> tuple[str, str]:
    """Verify every version the documentation names.

    Returns the pinned hand and transport versions.
    """

    tag = _single_pin(r"--branch v([0-9][^\s]*)", "the clone tag")
    if tag != version:
        raise Failure(f"documents clone --branch v{tag} but this release is {version}")
    venv = _single_pin(r"linkerbot-hand-ros-([0-9][^/\s]*)", "the virtualenv name")
    if venv != version:
        raise Failure(
            f"documents use a {venv} virtualenv but this release is {version}"
        )

    hand = _single_pin(r"linkerbot-hand==([0-9][^'\"\s]*)", "the hand SDK version")
    transport = _single_pin(
        r"linkerbot-transport\[all\]==([0-9][^'\"\s]*)", "the transport version"
    )
    specifier = Requirement(requirement).specifier
    if Version(hand) not in specifier:
        raise Failure(
            f"documents pin linkerbot-hand=={hand}, which does not satisfy the "
            f"declared {requirement}"
        )

    # Every remaining version token must be one of the three the release names.
    # This is the gate that catches a stale prose reference such as
    # "verification should show hand 1.0.0", which no pin pattern would see.
    known = {version, hand, transport}
    for path in documents():
        text = path.read_text("utf-8").replace(requirement, "")
        unknown = {token for token in VERSION_TOKEN.findall(text)} - known
        if unknown:
            raise Failure(
                f"{path.relative_to(ROOT)} names {', '.join(sorted(unknown))}, "
                f"which is not this release ({version}), the pinned hand SDK "
                f"({hand}) or transport ({transport})"
            )
    return hand, transport


def check_dependencies_are_installable(
    requirement: str, hand: str, transport: str
) -> None:
    """The documented dependencies must actually exist in the registry.

    2.0.0 declared ``linkerbot-hand>=2.0.0,<3.0.0`` while the registry held only
    1.0.0, so following the documentation produced ``No solution found``.
    """

    specifier = Requirement(requirement).specifier
    available = registry_versions("linkerbot-hand")
    if available is not None:
        if hand not in available:
            raise Failure(
                f"documents pin linkerbot-hand=={hand}, which is not in the "
                f"registry (it has {', '.join(sorted(available))})"
            )
        if not any(Version(candidate) in specifier for candidate in available):
            raise Failure(
                f"no registry version of linkerbot-hand satisfies {requirement}"
            )
    published = registry_versions("linkerbot-transport")
    if published is not None and transport not in published:
        raise Failure(
            f"documents pin linkerbot-transport=={transport}, which is not in "
            f"the registry (it has {', '.join(sorted(published))})"
        )


def check_tag_absent(version: str) -> None:
    tag = f"v{version}"
    if run("git", "tag", "--list", tag):
        raise Failure(f"tag {tag} already exists locally")
    if run("git", "ls-remote", "--tags", "origin", f"refs/tags/{tag}"):
        raise Failure(f"tag {tag} already exists on origin")


def check_tag_points_at_head(version: str) -> None:
    """Require ``v<version>`` to exist and to be the commit checked out.

    Used by ``release`` when back-filling a Release for a tag that already has
    one missing. The body is generated from the changelogs in the working tree, so
    creating it from any other commit would publish notes that do not describe the
    tagged source — and for this repository the tag is exactly what users clone.
    """

    tag = f"v{version}"
    if not run("git", "tag", "--list", tag):
        raise Failure(f"tag {tag} does not exist; run 'release.py tag' first")
    tagged = run("git", "rev-list", "-n1", tag)
    head = run("git", "rev-parse", "HEAD")
    if tagged != head:
        raise Failure(
            f"HEAD is {head[:9]} but {tag} points at {tagged[:9]}; the release "
            f"notes come from the working tree — run 'git checkout {tag}' first"
        )


def api_token() -> str:
    """Return the Gitea token. Nothing is uploaded, so no username is needed."""

    token = os.environ.get("GITEA_TOKEN")
    if not token:
        raise Failure("set GITEA_TOKEN to create a Gitea Release")
    return token


def api_request(
    path: str,
    *,
    method: str = "GET",
    payload: dict[str, object] | None = None,
) -> tuple[int, object]:
    """Call the Gitea API, returning the status and decoded body.

    A 404 is returned rather than raised: callers use it to decide whether a
    Release already exists, which is how creating one stays idempotent.
    """

    body = None if payload is None else json.dumps(payload).encode()
    request = urllib.request.Request(f"{API}{path}", data=body, method=method)
    request.add_header("Authorization", f"token {api_token()}")
    if body is not None:
        request.add_header("Content-Type", "application/json")
    try:
        with urllib.request.urlopen(request, timeout=30) as response:
            text = response.read().decode("utf-8", "replace")
            return response.status, (json.loads(text) if text else None)
    except urllib.error.HTTPError as error:
        text = error.read().decode("utf-8", "replace")
        if error.code == 404:
            return 404, None
        raise Failure(f"{method} {path} returned {error.code}: {text[:300]}") from error
    except (urllib.error.URLError, TimeoutError) as error:
        raise Failure(f"{method} {path} failed: {error}") from error


def release_notes(version: str) -> str:
    """Build the Gitea Release body.

    The changelogs are reStructuredText and the Release body is Markdown, so the
    two idioms these files actually use are converted (``literal`` and ``*``
    bullets) and the authoritative text is linked rather than reproduced
    verbatim.
    """

    tag = f"v{version}"
    source = f"{HOST}/{OWNER}/{REPOSITORY}/src/tag/{tag}"
    sections: list[str] = []
    for relative in CHANGELOGS:
        text = (ROOT / relative).read_text(encoding="utf-8")
        match = re.search(
            rf"^{re.escape(version)} \(\d{{4}}-\d{{2}}-\d{{2}}\)\n-+\n(.*?)"
            r"(?=^\d+\.\d+\.\d+ \(|\Z)",
            text,
            re.MULTILINE | re.DOTALL,
        )
        if match is None:
            raise Failure(f"{relative} has no section for {version}")
        body = match.group(1).strip()
        body = re.sub(r"``([^`]+)``", r"`\1`", body)
        body = re.sub(r"^\* ", "- ", body, flags=re.MULTILINE)
        package = relative.split("/")[0]
        sections.append(f"## {package}\n\n{body}")
    install = (
        "## 安装\n\n"
        "本 package 只从源码安装，请从本 tag 构建：\n\n"
        "```bash\n"
        f"git clone --branch {tag} --depth 1 \\\n"
        f"  ssh://git@git.linkerhub.work:12222/{OWNER}/{REPOSITORY}.git\n"
        "colcon build\n"
        "```\n\n"
        "**本 Release 不附产物，registry 里也不再上传本 package。** "
        "`linkerbot_hand_msgs` 是 ROSIDL 接口、没有平台无关 wheel，"
        "所以 pip 装本 package 得不到可用环境：装得上，但 `import "
        "linkerbot_hand_ros.node` 会因缺少 `rclpy` 与消息类型而失败。\n\n"
        f"Python 侧依赖的安装步骤见 [README_zh.md]({source}/README_zh.md)。"
        f"完整条目见 [linkerbot_hand_ros/CHANGELOG.rst]({source}/linkerbot_hand_ros/CHANGELOG.rst)"
        f" 与 [linkerbot_hand_msgs/CHANGELOG.rst]({source}/linkerbot_hand_msgs/CHANGELOG.rst)。"
    )
    return "\n\n".join([*sections, install])


def create_release(version: str) -> None:
    """Create the Gitea Release, deliberately without artifacts.

    Nothing is uploaded for this repository — not to the Release and not to the
    package registry. Historical releases did both: ``v1.0.0`` attached the driver
    wheel and sdist, and six versions sit in the registry. Both are dropped
    because the artifact is inert. Verified: installing
    ``linkerbot-hand-ros==2.0.0`` from the registry into a clean virtualenv
    succeeds, and then ``import linkerbot_hand_ros.node`` raises
    ``ModuleNotFoundError: No module named 'rclpy'``; the wheel also contains no
    ``linkerbot_hand_msgs`` at all, because that is a ROSIDL package with no
    platform-independent wheel. Nothing in the documentation installs it either.
    So the Release points at the source, which is the only thing that works.
    """

    tag = f"v{version}"
    status, existing = api_request(f"/releases/tags/{tag}")
    if status == 200 and isinstance(existing, dict):
        print(f"  skip Release {tag} already exists")
        return
    _, created = api_request(
        "/releases",
        method="POST",
        payload={
            "tag_name": tag,
            "name": f"{DIST} {version}",
            "body": release_notes(version),
            "draft": False,
            "prerelease": False,
        },
    )
    if not isinstance(created, dict) or "id" not in created:
        raise Failure(f"creating Release {tag} returned no id: {created!r}")
    print(f"  ok   Release {tag} created with no attached artifact")

    tag = f"v{version}"
    status, existing = api_request(f"/releases/tags/{tag}")
    if status == 200 and isinstance(existing, dict):
        print(f"  skip Release {tag} already exists")
        return
    _, created = api_request(
        "/releases",
        method="POST",
        payload={
            "tag_name": tag,
            "name": f"{DIST} {version}",
            "body": release_notes(version),
            "draft": False,
            "prerelease": False,
        },
    )
    if not isinstance(created, dict) or "id" not in created:
        raise Failure(f"creating Release {tag} returned no id: {created!r}")
    print(f"  ok   Release {tag} created with no attached artifact")


def check_quality() -> None:
    if shutil.which("ruff") is None:
        raise Failure("ruff is not on PATH")
    run("ruff", "check", ".")
    run("ruff", "format", "--check", ".")
    # The repository's pytest clashes with ROS jazzy's launch_testing plugin,
    # which registers an outdated pytest_pycollect_makemodule hookimpl.
    environment = {**os.environ, "PYTEST_DISABLE_PLUGIN_AUTOLOAD": "1"}
    completed = subprocess.run(
        [sys.executable, "-m", "pytest", "-q"],
        cwd=ROOT,
        capture_output=True,
        text=True,
        env=environment,
        check=False,
    )
    if completed.returncode != 0:
        raise Failure(f"pytest failed:\n{completed.stdout.strip()}")


def command_check(version: str, *, tag_exists: bool = False) -> None:
    """Run every gate in order, stopping at the first failure.

    Written as straight-line code rather than a table because each gate feeds the
    next: the documented requirement is needed to interpret the version tokens,
    and both are needed to query the registry.

    ``tag_exists`` is set by ``release`` when back-filling: the tag is already
    pushed, so the gate becomes "it exists and is what is checked out" and the
    branch cannot be ``main``.
    """

    print(f"linkerbot-hand-ros {version}")
    print(f"  ok   four version declarations agree on {version}")

    check_working_tree(require_main=not tag_exists)
    print("  ok   working tree")

    check_changelogs(version)
    print("  ok   changelogs")

    requirement = check_documented_requirement()
    print(f"  ok   documented requirement matches setup.py ({requirement})")

    hand, transport = check_documented_versions(version, requirement)
    print(f"  ok   documented versions (hand {hand}, transport {transport})")

    check_dependencies_are_installable(requirement, hand, transport)
    print("  ok   dependencies installable")

    if tag_exists:
        check_tag_points_at_head(version)
        print(f"  ok   tag v{version} points at HEAD")
    else:
        check_tag_absent(version)
        print(f"  ok   tag v{version} unused")

    check_quality()
    print("  ok   quality")


def command_tag(version: str) -> None:
    """Push the signed tag and create the Release.

    Both in one command because there is nothing to build in between, and because
    HEAD is the commit being tagged, so the release notes generated from the
    working tree describe exactly the tagged source.
    """

    command_check(version)
    tag = f"v{version}"
    run("git", "tag", "-s", tag, "-m", f"linkerbot-hand-ros {version}")
    run("git", "verify-tag", tag)
    run("git", "push", "origin", tag)
    print(f"  ok   pushed signed tag {tag}")
    create_release(version)


def command_release(version: str) -> None:
    """Create the Release for a tag that already has none.

    This is the recovery path, and it is not hypothetical: ``v2.0.0`` was tagged
    and left without a Release, which had to be created by hand. Requires the tag
    to be checked out so the notes match the tagged source.
    """

    command_check(version, tag_exists=True)
    create_release(version)


def command_verify_tag(version: str, tag: str) -> None:
    """Confirm an existing tag is consistent with the commit it points at.

    This is what CI runs on a ``v*`` push. It is the after-the-fact half of the
    2.0.0 mistake: the tag was pushed before the documentation fix merged, so it
    named 2.0.0 while pointing at a commit that documented 1.0.0. Only the gates
    that need no ROS toolchain run here; the rest ran locally before tagging.
    """

    print(f"linkerbot-hand-ros {version}, tag {tag}")
    if tag != f"v{version}":
        raise Failure(
            f"tag {tag} does not match the declared version {version}; either the "
            "tag is wrong or it was pushed before the version bump merged"
        )
    print(f"  ok   {tag} matches the four version declarations")
    requirement = check_documented_requirement()
    hand, transport = check_documented_versions(version, requirement)
    print(f"  ok   documentation pins hand {hand}, transport {transport}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("command", choices=("check", "tag", "release", "verify-tag"))
    parser.add_argument(
        "--tag", help="the tag to validate; required by verify-tag, ignored otherwise"
    )
    arguments = parser.parse_args()
    try:
        version = declared_version()
        if arguments.command == "check":
            command_check(version)
        elif arguments.command == "tag":
            command_tag(version)
        elif arguments.command == "release":
            command_release(version)
        else:
            if not arguments.tag:
                raise Failure("verify-tag needs --tag")
            command_verify_tag(version, arguments.tag)
    except Failure as error:
        print(f"\nrelease {arguments.command} refused:\n  {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

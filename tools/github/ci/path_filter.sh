#!/usr/bin/env bash

# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# PR Path Filter
# ==============
# Ensures external commits in a PR only touch paths allowed by path_filter.yml.
#
#   - Trusted same-repository staging exports (upstream/update_from_<8 hex>) pass.
#   - Commits validly signed by someone with repository write access are exempt.
#   - Every file touched by any other commit on the PR is checked with
#     check_path_filter.py, so a disallowed path a later commit reverts still fails.
#
# Usage:
#   PR_NUMBER=<number> GITHUB_REPOSITORY=<owner>/<repo> GH_TOKEN=<token> \
#     bash tools/github/ci/path_filter.sh
#
# Run from inside a clone where HEAD is the PR head commit and
# origin/<PR target branch> has been fetched with full history. Only git objects
# are read from the PR; check_path_filter.py and path_filter.yml are always the
# copies next to this script, so callers can run a trusted copy against a PR.
# Requires gh, jq, and python3 with PyYAML.
#
# Exit codes:
#   0 - path filter passed or was not required
#   non-zero - path filter failed, or the PR could not be evaluated

set -euo pipefail

: "${PR_NUMBER:?PR_NUMBER must be set}"
: "${GITHUB_REPOSITORY:?GITHUB_REPOSITORY must be set}"
: "${GH_TOKEN:?GH_TOKEN must be set}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OWNER="${GITHUB_REPOSITORY%%/*}"
REPO="${GITHUB_REPOSITORY#*/}"

# ---------------------------------------------------------------------------
# Determine PR target branch
# ---------------------------------------------------------------------------
echo "PR number: $PR_NUMBER"

# Query the original PR rather than any pull-request/<number> copy.
# The metadata also identifies automated staging exports, whose commit
# is necessarily unsigned because GitLab creates it before pushing.
PR_INFO=$(gh pr view "$PR_NUMBER" --repo "$GITHUB_REPOSITORY" \
  --json baseRefName,headRefName,headRepository,isCrossRepository)
TARGET_BRANCH=$(echo "$PR_INFO" | jq -r .baseRefName)
HEAD_REF=$(echo "$PR_INFO" | jq -r .headRefName)
HEAD_REPO=$(echo "$PR_INFO" | jq -r .headRepository.nameWithOwner)
IS_CROSS_REPO=$(echo "$PR_INFO" | jq -r .isCrossRepository)
if [ -z "$TARGET_BRANCH" ]; then
  echo "::error::Could not determine base branch for PR #$PR_NUMBER"
  exit 1
fi
echo "Target branch: $TARGET_BRANCH"
echo "Head branch: $HEAD_REF"
echo "Head repository: $HEAD_REPO"
echo "Cross-repository PR: $IS_CROSS_REPO"

TRUSTED_STAGING=false
# isCrossRepository=false should imply the same head repository. Keep
# the explicit equality check as defense in depth, and log both values
# above so an unexpected API response is diagnosable from one run.
if [ "$IS_CROSS_REPO" = "false" ] \
  && [ "$HEAD_REPO" = "$GITHUB_REPOSITORY" ] \
  && [[ "$HEAD_REF" =~ ^upstream/update_from_[0-9a-f]{8}$ ]]; then
  TRUSTED_STAGING=true
fi
echo "Trusted staging export: $TRUSTED_STAGING"

if [ "$TRUSTED_STAGING" = "true" ]; then
  echo "✅ Trusted same-repository staging export — path filter not required"
  exit 0
fi

# ---------------------------------------------------------------------------
# Identify external commits (GraphQL batch)
# ---------------------------------------------------------------------------
TARGET="origin/$TARGET_BRANCH"
BRANCH_SHAS=$(git rev-list "${TARGET}..HEAD")
COMMIT_COUNT=$(git rev-list --count "${TARGET}..HEAD")
echo "Commits on branch: $COMMIT_COUNT"

if [ "$COMMIT_COUNT" -eq 0 ]; then
  echo "✅ No commits outside $TARGET_BRANCH — path filter not required"
  exit 0
fi

# GraphQL query: fetch commit history with signature signer identity.
# Paginate in batches of 100 (GitHub max per page). History is walked
# from HEAD rather than a branch name, so it works for any commit GitHub
# can resolve in this repository, including fork PR heads reachable only
# through refs/pull/<number>/head.
HEAD_SHA=$(git rev-parse HEAD)

COMMITS_FILE=$(mktemp)
HAS_NEXT="true"
CURSOR=""
FOUND_COUNT=0

while [ "$HAS_NEXT" = "true" ] && [ "$FOUND_COUNT" -lt "$COMMIT_COUNT" ]; do
  if [ -z "$CURSOR" ]; then
    AFTER_CLAUSE=""
  else
    AFTER_CLAUSE=", after: \"$CURSOR\""
  fi

  RESPONSE=$(gh api graphql -f query='
    query($owner: String!, $repo: String!, $sha: GitObjectID!) {
      repository(owner: $owner, name: $repo) {
        object(oid: $sha) {
          ... on Commit {
            history(first: 100'"$AFTER_CLAUSE"') {
              pageInfo {
                hasNextPage
                endCursor
              }
              nodes {
                oid
                signature {
                  isValid
                  email
                  signer {
                    login
                  }
                }
              }
            }
          }
        }
      }
    }
  ' -f owner="$OWNER" -f repo="$REPO" -f sha="$HEAD_SHA")

  HAS_NEXT=$(echo "$RESPONSE" | jq -r '.data.repository.object.history.pageInfo.hasNextPage')
  CURSOR=$(echo "$RESPONSE" | jq -r '.data.repository.object.history.pageInfo.endCursor')

  echo "$RESPONSE" | jq -c '
    .data.repository.object.history.nodes[]
    | {
        oid,
        valid: (.signature.isValid == true),
        email: (.signature.email // ""),
        login: (.signature.signer.login // "")
      }
  ' >> "$COMMITS_FILE"

  # Advance FOUND_COUNT only for commits that are actual branch commits,
  # so the loop keeps paging until every PR commit has been classified
  # regardless of how many ancestor commits appear between them.
  while IFS= read -r OID; do
    [ -z "$OID" ] && continue
    if echo "$BRANCH_SHAS" | grep -qxF "$OID"; then
      FOUND_COUNT=$((FOUND_COUNT + 1))
    fi
  done <<< "$(echo "$RESPONSE" | jq -r '.data.repository.object.history.nodes[].oid')"
done

BRANCH_COMMITS=$(mktemp)
while IFS= read -r LINE; do
  OID=$(echo "$LINE" | jq -r '.oid')
  if echo "$BRANCH_SHAS" | grep -qxF "$OID"; then
    echo "$LINE" >> "$BRANCH_COMMITS"
  fi
done < "$COMMITS_FILE"

BRANCH_OID_FILE=$(mktemp)
GRAPHQL_OID_FILE=$(mktemp)
printf '%s\n' "$BRANCH_SHAS" | grep -v '^$' | sort -u > "$BRANCH_OID_FILE"
if [ -s "$BRANCH_COMMITS" ]; then
  jq -r '.oid' "$BRANCH_COMMITS" | grep -v '^$' | sort -u > "$GRAPHQL_OID_FILE"
fi
MISSING=$(comm -23 "$BRANCH_OID_FILE" "$GRAPHQL_OID_FILE")
if [ -n "$MISSING" ]; then
  echo "::error::GraphQL history did not include every branch commit; refusing to skip path filter"
  echo "Missing SHAs:"
  printf '%s\n' "$MISSING"
  exit 1
fi

# Unique GitHub usernames from valid signatures, then check write access.
# GET .../collaborators/{username}/permission needs only Metadata read,
# which GITHUB_TOKEN always has. permission is the legacy base role:
# admin/write/read/none (maintain maps to write, triage maps to read).
WRITE_LOGINS=$(mktemp)
LOGINS=$(jq -r 'select(.valid and .login != "") | .login' "$BRANCH_COMMITS" | sort -u)
if [ -n "$LOGINS" ]; then
  while IFS= read -r LOGIN; do
    [ -z "$LOGIN" ] && continue
    set +e
    API_OUT=$(gh api --include "repos/${OWNER}/${REPO}/collaborators/${LOGIN}/permission")
    API_EXIT=$?
    set -e
    STATUS=$(printf '%s\n' "$API_OUT" | awk 'NR==1 { print $2; exit }')
    BODY=$(printf '%s\n' "$API_OUT" | awk 'BEGIN {b=0} /^[\r]*$/ {b=1; next} b {print}')
    if [ "$STATUS" = "200" ]; then
      PERM=$(echo "$BODY" | jq -r '.permission')
    elif [ "$STATUS" = "404" ]; then
      PERM="none"
    else
      echo "::error::Failed to query write access for @${LOGIN} (HTTP ${STATUS:-unknown}, exit ${API_EXIT})"
      printf '%s\n' "$API_OUT"
      exit 1
    fi
    echo "Signer @${LOGIN} repository permission: ${PERM}"
    if [ "$PERM" = "write" ] || [ "$PERM" = "admin" ]; then
      echo "$LOGIN" >> "$WRITE_LOGINS"
    fi
  done <<< "$LOGINS"
fi

WRITE_JSON=$(jq -R -s -c 'split("\n") | map(select(length > 0))' "$WRITE_LOGINS")
echo "Writers among signers: $WRITE_JSON"

echo "=== Branch commit signatures ==="
jq -r --argjson writers "$WRITE_JSON" '
  . as $c
  | ($c.login != "" and ($writers | index($c.login)) != null) as $has_write
  | "\($c.oid) valid=\($c.valid) login=\($c.login) email=\($c.email) write=\($has_write)"
' "$BRANCH_COMMITS"

EXTERNAL_SHAS_FILE=$(mktemp)
jq -r --argjson writers "$WRITE_JSON" '
  select(
    (.valid != true) or
    (.login == "") or
    ((.login as $l | $writers | index($l)) == null)
  )
  | .oid
' "$BRANCH_COMMITS" > "$EXTERNAL_SHAS_FILE"

if [ ! -s "$EXTERNAL_SHAS_FILE" ]; then
  echo "✅ All commits from verified signers with repository write access — path filter not required"
  exit 0
fi
echo "=== External (unverified or without repo write access) commits ==="
cat "$EXTERNAL_SHAS_FILE"

# ---------------------------------------------------------------------------
# Collect files from external commits
# ---------------------------------------------------------------------------
EXTERNAL_FILES=$(mktemp)

while IFS= read -r SHA; do
  # Collect blob SHA, mode, and path for every changed file so the
  # path filter can inspect content (symlinks, binary, generated).
  git diff-tree --no-commit-id -r --raw --no-abbrev "$SHA" \
  | while IFS=$'\t' read -r info path extra; do
    # shellcheck disable=SC2086
    set -- $info          # :old_mode new_mode old_blob new_blob status
    new_mode="${2:0:6}"
    new_blob="$4"
    status_char="${5:0:1}"
    if [ "$status_char" = "R" ]; then
      # Rename: $path=old path, $extra=new path; check both
      printf '%s\t%s\t%s\n' "$new_blob" "$new_mode" "$path"
      printf '%s\t%s\t%s\n' "$new_blob" "$new_mode" "$extra"
    else
      printf '%s\t%s\t%s\n' "$new_blob" "$new_mode" "$path"
    fi
  done >> "$EXTERNAL_FILES"
done < "$EXTERNAL_SHAS_FILE"

# Deduplicate by path (third field), keeping first blob seen
sort -u -t$'\t' -k3,3 "$EXTERNAL_FILES" -o "$EXTERNAL_FILES"

echo "=== Files from external commits ==="
cat "$EXTERNAL_FILES"

# ---------------------------------------------------------------------------
# Run path filter check
# ---------------------------------------------------------------------------
python3 "$SCRIPT_DIR/check_path_filter.py" "$EXTERNAL_FILES"

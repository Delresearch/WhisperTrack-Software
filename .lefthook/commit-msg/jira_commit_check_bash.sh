#!/bin/bash
echo "jira_commit_check_bash.sh running" >> /tmp/jira_hook.log
COMMIT_MSG_FILE=$1
JIRA_REGEX='[A-Z]+-[0-9]+'

# Read the commit message, ignoring comments and empty lines
COMMIT_MSG=$(grep -vE '^\s*#' "$COMMIT_MSG_FILE" | grep -vE '^\s*$' | head -n 1)

# Check if commit message already contains Jira ticket
if echo "$COMMIT_MSG" | grep -qE "$JIRA_REGEX"; then
  exit 0
fi

BRANCH_NAME=$(git rev-parse --abbrev-ref HEAD)

if echo "$BRANCH_NAME" | grep -qE "$JIRA_REGEX"; then
  JIRA_TICKET=$(echo "$BRANCH_NAME" | grep -oE "$JIRA_REGEX" | head -1)
  NEW_MSG="${JIRA_TICKET}: ${COMMIT_MSG}"

  # Replace only the first non-comment, non-empty line
  awk -v newmsg="$NEW_MSG" '
    BEGIN { done=0 }
    /^[[:space:]]*#/ { print; next }
    /^[[:space:]]*$/ { print; next }
    !done { print newmsg; done=1; next }
    { print }
  ' "$COMMIT_MSG_FILE" > "${COMMIT_MSG_FILE}.tmp" && mv "${COMMIT_MSG_FILE}.tmp" "$COMMIT_MSG_FILE"

  exit 0
fi

echo "ERROR: Commit message or branch name must contain a Jira ticket like ABC-1234."
echo "Please add a Jira ticket to your commit message or use a branch name with the ticket."
exit 1
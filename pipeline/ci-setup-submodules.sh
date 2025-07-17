#!/bin/bash

# CI Setup Script for Submodules
# This script configures git to use HTTPS instead of SSH for GitLab CI

set -e

# Check if GitLab CI predefined variables are available
if [ -z "$CI_JOB_TOKEN" ]; then
    echo "Error: CI_JOB_TOKEN not available. This script should run in GitLab CI environment."
    exit 1
fi

echo "Configuring git to use HTTPS instead of SSH for GitLab..."

# Configure git to use HTTPS with CI token instead of SSH
git config --global url."https://gitlab-ci-token:${CI_JOB_TOKEN}@gitlab.popotomodem.com/".insteadOf "git@gitlab.popotomodem.com:"

echo "Git URL rewriting configured successfully"

# Initialize and update submodules (they will automatically use HTTPS now)
git submodule sync --recursive
git submodule update --init --recursive

echo "Submodules setup complete!"
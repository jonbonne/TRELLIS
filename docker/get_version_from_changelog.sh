#!/bin/bash
#===============================================================================
#
#          FILE: get_version_from_changelog.sh
# 
#         USAGE: ./get_version_from_changelog.sh 
# 
#   DESCRIPTION: 
# 
#       OPTIONS: ---
#  REQUIREMENTS: ---
#          BUGS: ---
#         NOTES: ---
#        AUTHOR: Newton McCollum (), 
#  ORGANIZATION: 
#       CREATED: 04/05/2023 04:07:51 PM
#      REVISION:  ---
#===============================================================================

set -o nounset                              # Treat unset variables as an error

#Define changelog file and the version header
CHANGELOG_FILE="../CHANGELOG.md"
VERSION_HEADER="## \[Unreleased\]"

# Extract the version number from the changelog
VERSION=$(grep -Eom 1 '[0-9]+\.[0-9]+\.[0-9]+' $CHANGELOG_FILE | head -1)

# If there's no version number in the changelog, exit with an error
if [[ -z "$VERSION" ]]; then
      echo "Error: no version number found in the changelog."
          exit 1
fi

# Print the version number
echo $VERSION


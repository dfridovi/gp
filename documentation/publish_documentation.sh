#!/bin/bash -e

# Quit on errors.
set -o errexit -o nounset

# Settings.
DOCUMENTATION_PATH=documentation
CHANGESET=$(git rev-parse --verify HEAD)

# Blow away any existing doc builds.
#rm -r html/ latex/

# Make documentation.
doxygen Doxyfile

# Make sure branches are up to date.
git pull origin master

# Commit documentation in master.
cd ..
git add ${DOCUMENTATION_PATH}

# Add the merged changes and push.
git commit -am "Automated documentation build for changeset ${CHANGESET}."
git push -u origin master

echo "-- Successfully updated documentation!"

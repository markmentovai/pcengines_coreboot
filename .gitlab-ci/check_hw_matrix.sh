#!/bin/bash

git clone https://$GITLAB_ROBOT_USERNAME:$GITLAB_ROBOT_TOKEN@$TESTS_REPO_GIT
cd $TESTS_REPO_DIR

git checkout hw-matrix-check

sed -i 's+git@gitlab.com:3mdeb/rte/rtectrl-rest-api.git+https://'$GITLAB_ROBOT_USERNAME':'$GITLAB_ROBOT_TOKEN'@gitlab.com/3mdeb/rte/rtectrl-rest-api.git+' .gitmodules
sed -i 's+git@gitlab.com:3mdeb/rte/snipeit-rest-api.git+https://'$GITLAB_ROBOT_USERNAME':'$GITLAB_ROBOT_TOKEN'@gitlab.com/3mdeb/rte/snipeit-rest-api.git+' .gitmodules
sed -i 's+git@gitlab.com:3mdeb/rte/pikvm-rest-api.git+https://'$GITLAB_ROBOT_USERNAME':'$GITLAB_ROBOT_TOKEN'@gitlab.com/3mdeb/rte/pikvm-rest-api.git+' .gitmodules
git submodule update --init --checkout

# legacy or mainline?
if [[ $FIRMWARE_VERSION =~ v4\.0.* ]]; then
    export FIRMWARE="l"
else
    export FIRMWARE="m"
fi

bash -cx "./check_hw_matrix.sh ${RELEASE_DIR}/${PLATFORM}_${CI_COMMIT_REF_NAME}.rom"

#!/usr/bin/env bash

DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" ; cd ../../../devel/lib/boothbot_base; pwd -P )

echo -e "\033[32m [Testing PC->Chassis Communication]..."
exec "$DIR/boothbot_base_sdk_test"

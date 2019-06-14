#!/usr/bin/env bash

set -eux

msbuild AtmosphereAutopilot/AtmosphereAutopilot.csproj /t:Build /p:Configuration=Release
pushd AtmosphereAutopilot/bin/Release
rm *.pdb
rm -f AtmosphereAutopilot.zip
rm -rf AtmosphereAutopilot
mkdir AtmosphereAutopilot
cp ../../../UnityAssets/Result/atmosphereautopilotprefabs ./
cp * AtmosphereAutopilot/ || true
cp ../../../LICENSE LICENSE_AtmosphereAutopilot
zip AtmosphereAutopilot.zip LICENSE_AtmosphereAutopilot AtmosphereAutopilot/*
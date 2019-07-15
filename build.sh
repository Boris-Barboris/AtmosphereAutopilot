#!/usr/bin/env bash

set -eux

msbuild AtmosphereAutopilot/AtmosphereAutopilot.csproj /t:Build /p:Configuration=Release
pushd AtmosphereAutopilot/bin/Release
rm *.pdb
rm -f *.zip
rm -rf AtmosphereAutopilot
rm LICENSE*
mkdir AtmosphereAutopilot
cp ../../../UnityAssets/Result/atmosphereautopilotprefabs ./
cp * AtmosphereAutopilot/ || true
cp ../../../LICENSE LICENSE_AtmosphereAutopilot
wget https://ksp.sarbian.com/jenkins/job/ModuleManager/149/artifact/ModuleManager.4.0.2.dll
zip AtmosphereAutopilot.zip LICENSE_AtmosphereAutopilot ModuleManager.4.0.2.dll AtmosphereAutopilot/*
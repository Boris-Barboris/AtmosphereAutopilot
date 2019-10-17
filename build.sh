#!/usr/bin/env bash

set -eux

msbuild AtmosphereAutopilot/AtmosphereAutopilot.csproj /t:Build /p:Configuration=Release /v:d
pushd AtmosphereAutopilot/bin/Release
rm *.pdb
rm -f *.zip
rm -rf AtmosphereAutopilot
rm -f LICENSE*
mkdir AtmosphereAutopilot
cp ../../../UnityAssets/Result/atmosphereautopilotprefabs ./
cp * AtmosphereAutopilot/ || true
cp ../../../LICENSE LICENSE_AtmosphereAutopilot
wget -nc https://ksp.sarbian.com/jenkins/job/ModuleManager/154/artifact/ModuleManager.4.1.0.dll
zip AtmosphereAutopilot.zip LICENSE_AtmosphereAutopilot ModuleManager.4.1.0.dll AtmosphereAutopilot/*

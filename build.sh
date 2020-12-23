#!/usr/bin/env bash

set -eux

msbuild AtmosphereAutopilot/AtmosphereAutopilot.csproj /t:Build /p:Configuration=Release /v:d
pushd AtmosphereAutopilot/bin/Release
rm *.pdb
rm -f *.zip
rm -rf AtmosphereAutopilot
rm -f LICENSE*
rm -f ModuleManager*
mkdir AtmosphereAutopilot
cp ../../../UnityAssets/Result/atmosphereautopilotprefabs ./
cp * AtmosphereAutopilot/ || true
cp ../../../LICENSE LICENSE_AtmosphereAutopilot
cp ../../../KSPUpgradeScriptFix.dll ./
wget -nc https://ksp.sarbian.com/jenkins/job/ModuleManager/159/artifact/ModuleManager.4.1.4.dll
zip AtmosphereAutopilot.zip LICENSE_AtmosphereAutopilot ModuleManager.4.1.0.dll AtmosphereAutopilot/* KSPUpgradeScriptFix.dll

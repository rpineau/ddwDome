#!/bin/bash

mkdir -p ROOT/tmp/ddwDome_X2/
cp "../ddwDome.ui" ROOT/tmp/ddwDome_X2/
cp "../Pulsar.png" ROOT/tmp/ddwDome_X2/
cp "../domelist ddwDome.txt" ROOT/tmp/ddwDome_X2/
cp "../build/Release/libddwDome.dylib" ROOT/tmp/ddwDome_X2/

if [ ! -z "$installer_signature" ]; then
# signed package using env variable installer_signature
pkgbuild --root ROOT --identifier org.rti-zone.ddwDome_X2 --sign "$installer_signature" --scripts Scripts --version 1.0 ddwDome_X2.pkg
pkgutil --check-signature ./ddwDome_X2.pkg
else
pkgbuild --root ROOT --identifier org.rti-zone.ddwDome_X2 --scripts Scripts --version 1.0 ddwDome_X2.pkg
fi

rm -rf ROOT

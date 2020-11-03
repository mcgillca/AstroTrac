#!/bin/bash

PACKAGE_NAME="RST_X2.pkg"
BUNDLE_NAME="org.rti-zone.RSTX2"

if [ ! -z "$app_id_signature" ]; then
    codesign -f -s "$app_id_signature" --verbose ../build/Release/libRST.dylib
fi

mkdir -p ROOT/tmp/RST_X2/
cp "../RST.ui" ROOT/tmp/RST_X2/
cp "../mountlist RST.txt" ROOT/tmp/RST_X2/
cp "../RainbowAstro.png" ROOT/tmp/RST_X2/
cp "../build/Release/libRST.dylib" ROOT/tmp/RST_X2/

if [ ! -z "$installer_signature" ]; then
	# signed package using env variable installer_signature
	pkgbuild --root ROOT --identifier $BUNDLE_NAME --sign "$installer_signature" --scripts Scripts --version 1.0 $PACKAGE_NAME
	pkgutil --check-signature ./${PACKAGE_NAME}
else
	pkgbuild --root ROOT --identifier $BUNDLE_NAME --scripts Scripts --version 1.0 $PACKAGE_NAME
fi

rm -rf ROOT

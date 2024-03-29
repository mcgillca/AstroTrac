#!/bin/bash

PACKAGE_NAME="AstroTrac_X2.pkg"
BUNDLE_NAME="org.rti-zone.AstroTracX2"

if [ ! -z "$app_id_signature" ]; then
    codesign -f -s "$app_id_signature" --verbose ../build/Release/libAstroTrac.dylib
fi

mkdir -p ROOT/tmp/AstroTrac_X2/
cp "../AstroTrac.ui" ROOT/tmp/AstroTrac_X2/
cp "../AstroTrac360.png" ROOT/tmp/AstroTrac_X2/
cp "../mountlist AstroTrac.txt" ROOT/tmp/AstroTrac_X2/
cp "../build/Release/libAstroTrac.dylib" ROOT/tmp/AstroTrac_X2/

if [ ! -z "$installer_signature" ]; then
	# signed package using env variable installer_signature
	pkgbuild --root ROOT --identifier "$BUNDLE_NAME" --sign "$installer_signature" --scripts Scripts --version 1.0 "$PACKAGE_NAME"
	pkgutil --check-signature "./${PACKAGE_NAME}"
	res=`xcrun altool --notarize-app --primary-bundle-id "$BUNDLE_NAME" --username "$AC_USERNAME" --password "@keychain:AC_PASSWORD" --file $PACKAGE_NAME`
	RequestUUID=`echo $res | grep RequestUUID | cut -d"=" -f 2 | tr -d [:blank:]`
	if [ -z "$RequestUUID" ]; then
		echo "Error notarizing"
		echo "res = $res"
		exit 1
	fi
	echo "Notarization RequestUUID '$RequestUUID'"
	sleep 30
	while true
	echo "Waiting for notarization"
	do
		res=`xcrun altool --notarization-info "$RequestUUID" --username "pineau@rti-zone.org" --password "@keychain:AC_PASSWORD"`
		pkg_ok=`echo $res | grep -i "Package Approved"`
		pkg_in_progress=`echo $res | grep -i "in progress"`
		if [ ! -z "$pkg_ok" ]; then
			break
		elif [ ! -z "$pkg_in_progress" ]; then
			sleep 60
		else
			echo  "Notarization timeout or error"
			echo "res = $res"
			exit 1
		fi
	done
	xcrun stapler staple $PACKAGE_NAME

else
	pkgbuild --root ROOT --identifier $BUNDLE_NAME --scripts Scripts --version 1.0 $PACKAGE_NAME
fi

rm -rf ROOT

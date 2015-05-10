#!/bin/bash

###### defines ######

local_dir=$PWD

###### defines ######
echo '#############'
echo 'making clean'
echo '#############'
make clean
rm -rf out
echo '#############'
echo 'making defconfig'
echo '#############'
make pxa1088_degaswifi_eur_defconfig
echo '#############'
echo 'making zImage'
echo '#############'
time make -j20 CONFIG_NO_ERROR_ON_MISMATCH=y
echo '#############'
echo 'making DTB'
echo '#############'
mkdir -p out/modules
time ./tools/dtbTool -o out/boot.img-dt -p ./scripts/dtc/ ./arch/arm/boot/dts/
if [[ $? != 0 ]]; then
    sleep 5
	./tools/dtbTool -o out/boot.img-dt -p ./scripts/dtc/ ./arch/arm/boot/dts/
fi
echo '#############'
echo 'copying files to ./out'
echo '#############'
echo ''
cp arch/arm/boot/zImage out/zImage

# Find and copy modules
find ./drivers -name '*.ko' | xargs -I {} cp {} ./out/modules/
#find ./crypto -name '*.ko' | xargs -I {} cp {} ./out/modules/


echo ''
if [[ -a arch/arm/boot/zImage && -a out/boot.img-dt ]]; then

if [ $1 == '--server' ]; then
    echo "Pushing to server..."
    cd out
    zip -r $(date -u +%Y-%m-%d_%H%M).zip zImage boot.img-dt modules
    chmod 664 $(date -u +%Y-%m-%d_%H%M).zip
    cd ..
    scp -r out/$(date -u +%Y-%m-%d_%H%M).zip vps:server/1/Android/SGT4/
fi
echo ''
echo '#############'
echo 'build finished successfully'
echo '#############'
else
echo '#############'
echo 'build failed!'
echo '#############'
fi

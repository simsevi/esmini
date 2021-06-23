#!/bin/bash

# 
# This script will build OpenSceneGraph (OSG) libraries  
# needed for esmini to visualize the road network and scenario.
# No system installations will be done (no admin rights required)
#
# The ambition is to support Windows, Linux and Mac. However the script is under
# development and has not been tested as is on all three platforms.
#
# Prerequisites:
# - Git (with Bash) (https://git-scm.com/download/win)
# - cmake (https://cmake.org/download/)
# - compiler (Visual Studio with C++ toolkit, gcc, xcode...)
#
# Usage: 
# - put this script in an empty folder
# - open bash (e.g. Git Bash) in that folder 
# - review and adjust system dependent parameters in section below
# - run the script: ./generate_osi_libs.sh
# - wait (the build process will take approx. 15 minutes depending on...)
# 
# The osi_*.7z will contain both headers and needed libraries. (* depends on platform)
# 
#

# -----------------------------------------------------------------------------------
# Review and update settings in this section according to your system and preferences

if [ "$OSTYPE" == "msys" ]; then
	# Visual Studio 2019 - toolkit from Visual Studio 2017
	GENERATOR=("Visual Studio 16 2019")
	GENERATOR_TOOLSET="v141"
	GENERATOR_ARGUMENTS="-A x64 -T ${GENERATOR_TOOLSET}"

	# Visual Studio 2019 - default toolkit
	# GENERATOR=("Visual Studio 16 2019")
	# GENERATOR_ARGUMENTS="-A x64 -T ${GENERATOR_TOOLSET}"

	# Visual Studio 2017 - default toolkit
	# GENERATOR=("Visual Studio 15 2017 Win64")
	# GENERATOR_ARGUMENTS="-T ${GENERATOR_TOOLSET}"
    
    # Make sure 7zip is available, else download and install it
    # https://www.7-zip.org/download.html
    APP_7ZIP="/c/Program Files/7-Zip/7z.exe"
    
    LIB_EXT="lib"
    LIB_PREFIX=""
    LIB_OSG_PREFIX="osg161-"
    LIB_OT_PREFIX="ot21-"
elif [[ "$OSTYPE" == "darwin"* ]] || [[ "$OSTYPE" == "linux-gnu"* ]]; then
	# Unix Makefiles (for Ubuntu and other Linux systems)
	GENERATOR=("Unix Makefiles")
	GENERATOR_ARGUMENTS=""
    LIB_EXT="a"
    LIB_PREFIX="lib"
    LIB_OSG_PREFIX="lib"
    LIB_OT_PREFIX="lib"
else
	echo Unknown OSTYPE: $OSTYPE
fi


OSG_VERSION=OpenSceneGraph-3.6.5


# ---------------------------------------------------------------------------------------
# From this point no adjustments should be necessary, except fixing bugs in the script :)
# However you might want to adjust versions of software packages being checkout and built


osg_root_dir=$(pwd)


echo ------------------------ Installing dependencies ------------------------------------
cd $osg_root_dir


if [ "$OSTYPE" == "msys" ]; then
    if [ ! -d 3rdParty_x64 ]; then
        if [ ! -f 3rdParty_VS2017_v141_x64_V11_small.7z ]; then
            curl "https://download.osgvisual.org/3rdParty_VS2017_v141_x64_V11_small.7z" -o 3rdParty_VS2017_v141_x64_V11_small.7z
        fi    
        "$APP_7ZIP" x 3rdParty_VS2017_v141_x64_V11_small.7z
    fi
elif  [[ "$OSTYPE" == "darwin"* ]] || [[ "$OSTYPE" == "linux-gnu"* ]]; then

    if [ ! -d zlib-1.2.11 ]; then
        if [ ! -f zlib1211.zip ]; then
            curl "https://zlib.net/zlib1211.zip" -o zlib1211.zip
        fi
        unzip zlib1211.zip
        cd zlib-1.2.11
        mkdir install
        mkdir build
        cd build


        if [[ "$OSTYPE" == "linux-gnu"* ]]; then
            cmake -G "${GENERATOR[@]}" ${GENERATOR_ARGUMENTS} -D CMAKE_INSTALL_PREFIX=../install -DCMAKE_BUILD_TYPE=Debug .. -DCMAKE_C_FLAGS="-fPIC" 
            cmake --build . --target install
            mv ../install/lib/libz.a ../install/lib/libzd.a

            rm CMakeCache.txt
            cmake -G "${GENERATOR[@]}" ${GENERATOR_ARGUMENTS} -D CMAKE_INSTALL_PREFIX=../install -DCMAKE_BUILD_TYPE=Release .. -DCMAKE_C_FLAGS="-fPIC" 
            cmake --build . --target install
        elif [[ "$OSTYPE" == "darwin"* ]]; then
            cmake -G "${GENERATOR[@]}" ${GENERATOR_ARGUMENTS} -D CMAKE_INSTALL_PREFIX=../install -DCMAKE_BUILD_TYPE=Release .. -DCMAKE_C_FLAGS="-fPIC" 
            cmake --build . --target install
        fi

    else
        echo zlib folder already exists, continue with next step...
    fi
fi

cd $osg_root_dir
if [[ "$OSTYPE" == "linux-gnu"* ]] || [[ "$OSTYPE" == "darwin"* ]]; then
    if [ ! -d jpeg-8d ]; then
        if [ ! -f jpegsrc.v8d.tar.gz ]; then
            curl "http://www.ijg.org/files/jpegsrc.v8d.tar.gz" -o jpegsrc.v8d.tar.gz
        fi
        tar xzf jpegsrc.v8d.tar.gz
        cd jpeg-8d
        if [[ "$OSTYPE" == "darwin"* ]]; then
            ./configure
            make
        else
            ./configure CFLAGS='-fPIC -g'; make -j
            mv .libs .libsd
            mv .libsd/libjpeg.a .libsd/libjpegd.a
            make clean
            ./configure CFLAGS='-fPIC'; make -j
        fi
    else
        echo jpeg folder already exists, continue with next step...
    fi
fi

echo ------------------------ Installing OSG ------------------------------------
cd $osg_root_dir

if [ ! -d OpenSceneGraph ]; then
    git clone https://github.com/eknabevcc/OpenSceneGraph
fi

if [ ! -d OpenSceneGraph/build ]; then

    cd OpenSceneGraph
    git checkout $OSG_VERSION

    # Apply fix for comment format not accepted by all platforms
    git checkout 63bb537132bab1f8b077838f7550e26405e5fa35 CMakeModules/FindFontconfig.cmake

    # Apply fix for Mac window handler
    git checkout 3994378a20948ebc4ed10b7cd33a6cc5393e7157 src/osgViewer/GraphicsWindowCocoa.mm

    # Apply fix for shadow maps on Intel UHD Graphics 620/Windows systems
    git checkout 0229db8632c03b3aaf35420d72dd8eb49fe3ad02 src/osgShadow/ShadowMap.cpp

    mkdir build
    cd build

    if [[ "$OSTYPE" == "linux-gnu"* ]]; then

        cmake ../ -DDYNAMIC_OPENSCENEGRAPH=false -DDYNAMIC_OPENTHREADS=false -DCMAKE_CXX_FLAGS=-fPIC -DJPEG_LIBRARY_RELEASE=$osg_root_dir/jpeg-8d/.libs/libjpeg.a -DJPEG_LIBRARY=$osg_root_dir/jpeg-8d/.libs/libjpeg.a -DJPEG_INCLUDE_DIR=$osg_root_dir/jpeg-8d -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=../install

        make -j8 install

        rm CMakeCache.txt

        cmake ../ -DDYNAMIC_OPENSCENEGRAPH=false -DDYNAMIC_OPENTHREADS=false -DCMAKE_CXX_FLAGS=-fPIC -DJPEG_LIBRARY_RELEASE=$osg_root_dir/jpeg-8d/.libsd/libjpegd.a -DJPEG_LIBRARY=$osg_root_dir/jpeg-8d/.libsd/libjpegd.a -DJPEG_INCLUDE_DIR=$osg_root_dir/jpeg-8d -DCMAKE_BUILD_TYPE=Debug -DCMAKE_INSTALL_PREFIX=../install-debug

        make -j8 install
    
    elif [[ "$OSTYPE" == "darwin"* ]]; then
        cmake ../ -DDYNAMIC_OPENSCENEGRAPH=false -DDYNAMIC_OPENTHREADS=false -DJPEG_LIBRARY_RELEASE=$osg_root_dir/jpeg-8d/.libs/libjpeg.a -DJPEG_INCLUDE_DIR=$osg_root_dir/jpeg-8d -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS=-fPIC -DCMAKE_INSTALL_PREFIX=../install

        cmake --build . -j 16 --config Release --target install

    elif [ "$OSTYPE" == "msys" ]; then
        cmake -G "${GENERATOR[@]}" ${GENERATOR_ARGUMENTS} ../ -DDYNAMIC_OPENSCENEGRAPH=false -DDYNAMIC_OPENTHREADS=false -DCMAKE_INSTALL_PREFIX=../install -DACTUAL_3RDPARTY_DIR=../../3rdParty_x64/x64
        
        cmake --build . -j 8 --config Release --target install

        rm CMakeCache.txt
        
        cmake -G "${GENERATOR[@]}" ${GENERATOR_ARGUMENTS} ../ -DDYNAMIC_OPENSCENEGRAPH=false -DDYNAMIC_OPENTHREADS=false -DCMAKE_INSTALL_PREFIX=../install-debug -DACTUAL_3RDPARTY_DIR=../../3rdParty_x64/x64

        cmake --build . -j 8 --config Debug --target install
    else
        echo Unknown OSTYPE: $OSTYPE
    fi
fi

echo ------------------------ Pack ------------------------------------

cd $osg_root_dir

if [ "$OSTYPE" == "msys" ]; then
    target_dir="v10"
    zfilename="osg_v10.7z"
    z_exe="/c/Program Files/7-Zip/7z.exe"
elif [[ "$OSTYPE" == "linux-gnu"* ]]; then
    target_dir="linux"
    zfilename="osg_linux.7z"
    z_exe=7za
elif [[ "$OSTYPE" == "darwin"* ]]; then
    target_dir="mac"
    zfilename="osg_mac.7z"
    z_exe=7z
else
    echo Unknown OSTYPE: $OSTYPE
fi

if [ ! -d $target_dir ] 
then
    mkdir $target_dir
    mkdir $target_dir/include
    mkdir $target_dir/lib
    mkdir $target_dir/lib/osgPlugins-3.6.5
    cp -r OpenSceneGraph/install/include $target_dir/

    if [ "$OSTYPE" == "msys" ]; then
        cp 3rdParty_x64/x64/include/zlib.h $target_dir/include
        cp 3rdParty_x64/x64/lib/zlibstatic.lib 3rdParty_x64/x64/lib/zlibstaticd.lib $target_dir/lib
        cp 3rdParty_x64/x64/include/jpeglib.h $target_dir/include
        cp 3rdParty_x64/x64/lib/jpeg.lib 3rdParty_x64/x64/lib/jpegd.lib $target_dir/lib
    elif [[ "$OSTYPE" == "linux-gnu"* ]] || [[ "$OSTYPE" == "darwin"* ]]; then
        cp zlib-1.2.11/install/include/zlib.h $target_dir/include
        cp zlib-1.2.11/install/lib/libz.${LIB_EXT} zlib-1.2.11/install/lib/libzd.${LIB_EXT} $target_dir/lib
        cp jpeg-8d/jpeglib.h $target_dir/include
        cp jpeg-8d/.libs/libjpeg.${LIB_EXT} jpeg-8d/.libsd/libjpegd.${LIB_EXT} $target_dir/lib
    else
        echo Unknown OSTYPE: $OSTYPE
    fi

    cd $osg_root_dir/OpenSceneGraph/install/lib
    cp ${LIB_OSG_PREFIX}osg.${LIB_EXT} $osg_root_dir/$target_dir/lib
    cp ${LIB_OSG_PREFIX}osgAnimation.${LIB_EXT} $osg_root_dir/$target_dir/lib
    cp ${LIB_OSG_PREFIX}osgDB.${LIB_EXT} $osg_root_dir/$target_dir/lib
    cp ${LIB_OSG_PREFIX}osgGA.${LIB_EXT} $osg_root_dir/$target_dir/lib
    cp ${LIB_OSG_PREFIX}osgShadow.${LIB_EXT} $osg_root_dir/$target_dir/lib
    cp ${LIB_OSG_PREFIX}osgSim.${LIB_EXT} $osg_root_dir/$target_dir/lib
    cp ${LIB_OSG_PREFIX}osgText.${LIB_EXT} $osg_root_dir/$target_dir/lib
    cp ${LIB_OSG_PREFIX}osgUtil.${LIB_EXT} $osg_root_dir/$target_dir/lib
    cp ${LIB_OSG_PREFIX}osgViewer.${LIB_EXT} $osg_root_dir/$target_dir/lib
    cp ${LIB_OT_PREFIX}OpenThreads.${LIB_EXT} $osg_root_dir/$target_dir/lib

    cd $osg_root_dir/OpenSceneGraph/install/lib/osgPlugins-3.6.5
    cp ${LIB_PREFIX}osgdb_jpeg.${LIB_EXT} $osg_root_dir/$target_dir/lib/osgPlugins-3.6.5
    cp ${LIB_PREFIX}osgdb_osg.${LIB_EXT} $osg_root_dir/$target_dir/lib/osgPlugins-3.6.5
    cp ${LIB_PREFIX}osgdb_serializers_osg.${LIB_EXT} $osg_root_dir/$target_dir/lib/osgPlugins-3.6.5
    cp ${LIB_PREFIX}osgdb_serializers_osgsim.${LIB_EXT} $osg_root_dir/$target_dir/lib/osgPlugins-3.6.5

    if [[ ! "$OSTYPE" == "darwin"* ]]; then
        cd $osg_root_dir/OpenSceneGraph/install-debug/lib
        cp ${LIB_OSG_PREFIX}osgd.${LIB_EXT} $osg_root_dir/$target_dir/lib
        cp ${LIB_OSG_PREFIX}osgAnimationd.${LIB_EXT} $osg_root_dir/$target_dir/lib
        cp ${LIB_OSG_PREFIX}osgDBd.${LIB_EXT} $osg_root_dir/$target_dir/lib
        cp ${LIB_OSG_PREFIX}osgGAd.${LIB_EXT} $osg_root_dir/$target_dir/lib
        cp ${LIB_OSG_PREFIX}osgShadowd.${LIB_EXT} $osg_root_dir/$target_dir/lib
        cp ${LIB_OSG_PREFIX}osgSimd.${LIB_EXT} $osg_root_dir/$target_dir/lib
        cp ${LIB_OSG_PREFIX}osgTextd.${LIB_EXT} $osg_root_dir/$target_dir/lib
        cp ${LIB_OSG_PREFIX}osgUtild.${LIB_EXT} $osg_root_dir/$target_dir/lib
        cp ${LIB_OSG_PREFIX}osgViewerd.${LIB_EXT} $osg_root_dir/$target_dir/lib
        cp ${LIB_OT_PREFIX}OpenThreadsd.${LIB_EXT} $osg_root_dir/$target_dir/lib

        cd $osg_root_dir/OpenSceneGraph/install-debug/lib/osgPlugins-3.6.5
        cp ${LIB_PREFIX}osgdb_jpegd.${LIB_EXT} $osg_root_dir/$target_dir/lib/osgPlugins-3.6.5
        cp ${LIB_PREFIX}osgdb_osgd.${LIB_EXT} $osg_root_dir/$target_dir/lib/osgPlugins-3.6.5
        cp ${LIB_PREFIX}osgdb_serializers_osgd.${LIB_EXT} $osg_root_dir/$target_dir/lib/osgPlugins-3.6.5
        cp ${LIB_PREFIX}osgdb_serializers_osgsimd.${LIB_EXT} $osg_root_dir/$target_dir/lib/osgPlugins-3.6.5
    fi

    cd $osg_root_dir

    "$z_exe" a -r $zfilename -m0=LZMA -bb1 -spf $target_dir/*
    # unpack with: 7z x <filename>

    echo ------------------------ Done ------------------------------------
fi

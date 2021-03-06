# OTB Frequency Resample Module

This standalone [OTB][OTB] module is a wrapper of the [Sirius library][Sirius].
It provides an image to image filter which resamples an image in the frequency domain.
This filter is packaged in the application *FrequencyResample*.

## Overview

* [Orfeo Toolbox][OTB]
* [Sirius library][Sirius]
* [Sirius documentation][Sirius Documentation]

## How to build

This module is using [CMake] to build its libraries and executables.

### Requirements

* C++14 compiler (GCC >5)
* [CMake] >=3.2
* [GDAL] development kit, >=2
* [FFTW] development kit, >=3

### Dependencies

[Sirius library][Sirius] is integrated in this [OTB][OTB] module as a Git submodule.

`libsirius-static` [CMake] target is made available thanks to CMake project inclusion and added as a [CMake] link dependency to the OTB module.
Sirius library will be built automatically with the OTB module settings (compilation flags, options).

### Options

* `CMAKE_BUILD_TYPE`: Debug, Release, RelWithDebInfo and MinSizeRel
* `OTB_BUILD_MODULE_AS_STANDALONE`: set to `ON` to build `FrequencyResample` as a standalone module (required)
* `OTB_DIR`: path to the OTB SDK [CMake] directory
* `BUILD_TESTING`: set to `ON` to enable tests

### Example

```sh
git clone --recursive https://github.com/CS-SI/OTB-SIRIUS.git
mkdir .build
cd .build
cmake .. -DCMAKE_BUILD_TYPE=Release \
         -DOTB_BUILD_MODULE_AS_STANDALONE=ON \
         -DOTB_DIR=/path/to/OTB/SDK/lib/cmake/OTB-x.y \
         -DBUILD_TESTING=ON
cmake --build .
```

If Sirius submodule commit reference is updated, you need to run the following command:

```sh
git submodule update
```

## How to use

### Host requirements

* [GDAL][GDAL] library
* [FFTW3][FFTW] library

### FrequencyResample application

The following example will zoom in by 2 `/path/to/input/image.tif`, apply the filter `/path/to/filter/image.tif` to the resampled image and write the output into `/path/to/output/image.tif`.

```sh
# CWD is the OTB binary directory
./otbApplicationLauncherCommandLine FrequencyResample /path/to/otbFrequencyZoomModule/application/library \
    -in /path/to/input/image.tif \
    -out /path/to/output/image.tif \
    -resampling.ratio 2:1 \
    -filter.path /path/to/filter/image.tif
```

### API

The following code is a boilerplate to create and initialize the `FrequencyResampleFilter` component.


```cpp
#include "otbFrequencyResampleFilter.h"

using FilterType = otb::FrequencyResampleFilter<DoubleImageType>;

// initialization parameters
sirius::ZoomRatio zoom_ratio(2, 1);

sirius::Filter freq_filter = sirius::Filter::Create("/path/to/image/filter_2.tif",
                                                    zoom_ratio);

sirius::ImageDecompositionPolicies image_decomposition =
      sirius::ImageDecompositionPolicies::kRegular;
sirius::FrequencyZoomStrategies zoom_strategy =
      sirius::FrequencyZoomStrategies::kPeriodization;

FilterType::Pointer i2i_filter = FilterType::New();

// init filter
i2i_filter->Init(zoom_ratio, std::move(freq_filter),
                 image_decomposition, zoom_strategy);

i2i_filter->SetInput(...);
```

[OTB]: https://www.orfeo-toolbox.org "Orfeo Toolbox"
[Sirius]: https://github.com/CS-SI/SIRIUS "Sirius library"
[Sirius Documentation]: https://CS-SI.github.io/SIRIUS/html/Sirius.html "Sirius documentation"
[CMake]: https://cmake.org/ "CMake"
[GDAL]: http://www.gdal.org/ "Geospatial Data Abstraction Library"
[FFTW]: http://www.fftw.org/ "Fastest Fourier Transform in the West"



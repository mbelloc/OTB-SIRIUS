/**
 * Copyright (C) 2018 CS - Systemes d'Information (CS-SI)
 *
 * This file is part of OTB-SIRIUS
 *
 *     https://github.com/CS-SI/OTB-SIRIUS
 *
 * OTB-SIRIUS is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * OTB-SIRIUS is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with OTB-SIRIUS.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef OTB_FREQUENCY_SHIFT_FILTER_H_
#define OTB_FREQUENCY_SHIFT_FILTER_H_

#include <sirius/frequency_translation_factory.h>
#include <sirius/image.h>
#include <sirius/types.h>

#include "itkImageToImageFilter.h"

#include "otbImage.h"

namespace otb {

/**
 * \brief Wrapper of Sirius IFrequencyTranslation API
 */
template <class TInputImage, class TOutputImage = TInputImage>
class FrequencyShiftFilter
      : public itk::ImageToImageFilter<TInputImage, TOutputImage> {
  public:
    /** standard class typedefs */
    typedef TInputImage InputImageType;
    typedef TOutputImage OutputImageType;

    typedef FrequencyShiftFilter Self;
    typedef itk::ImageToImageFilter<InputImageType, OutputImageType> Superclass;
    typedef itk::SmartPointer<Self> Pointer;
    typedef itk::SmartPointer<const Self> ConstPointer;

    typedef typename InputImageType::PixelType InputPixelType;
    typedef typename OutputImageType::PixelType OutputPixelType;
    typedef typename InputImageType::RegionType InputImageRegionType;
    typedef typename OutputImageType::RegionType OutputImageRegionType;
    typedef typename InputImageType::IndexType IndexType;
    typedef typename InputImageType::SizeType SizeType;

    /** Object factory management */
    itkNewMacro(Self);

    itkTypeMacro(FrequencyShiftFilter, ImageToImageFilter);

    /**
     * \brief Init the ImageToImageFilter with Sirius parameters
     * \param image_decomposition requested image decomposition
     * \param row_shift shift on y axis
     * \param col_shift shift on x axis
     */
    void Init(sirius::ImageDecompositionPolicies image_decomposition,
              float row_shift, float col_shift);

  protected:
    FrequencyShiftFilter() = default;
    ~FrequencyShiftFilter() override = default;

    void GenerateInputRequestedRegion() throw(
          itk::InvalidRequestedRegionError) override;
    void GenerateOutputInformation() override;
    void ThreadedGenerateData(
          const OutputImageRegionType& outputRegionForThread,
          itk::ThreadIdType threadId) override;

  private:
    FrequencyShiftFilter(const Self&) = delete;
    FrequencyShiftFilter& operator=(const Self&) = delete;
    sirius::Image GenerateImageFromRegion(const InputImageRegionType& region);
    InputImageRegionType GetInputRegion(const IndexType& idx,
                                        const SizeType& size);

  private:
    float row_shift_;
    float col_shift_;
    sirius::IFrequencyTranslation::UPtr frequency_shifter_;
};

}  // namespace otb

#ifndef OTB_MANUAL_INSTANTIATION
#include "otbFrequencyShiftFilter.txx"
#endif  // OTB_MANUAL_INSTANTIATION

#endif  // OTB_FREQUENCY_SHIFT_FILTER_H_

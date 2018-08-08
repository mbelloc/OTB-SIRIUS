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

#ifndef OTB_FREQUENCY_SHIFT_FILTER_TXX_
#define OTB_FREQUENCY_SHIFT_FILTER_TXX_

#include "otbFrequencyShiftFilter.h"

#include <cmath>

#include "itkImageLinearConstIteratorWithIndex.h"
#include "itkImageRegionConstIteratorWithIndex.h"
#include "itkImageRegionIterator.h"

namespace otb {

template <class TInputImage, class TOutputImage>
void FrequencyShiftFilter<TInputImage, TOutputImage>::
      GenerateInputRequestedRegion() throw(itk::InvalidRequestedRegionError) {
    // call the superclass' implementation of this method
    Superclass::GenerateInputRequestedRegion();

    // get pointers to the input and output
    auto input_ptr = const_cast<InputImageType*>(this->GetInput());
    auto output_ptr = this->GetOutput();

    auto output_idx = output_ptr->GetRequestedRegion().GetIndex();
    auto output_size = output_ptr->GetRequestedRegion().GetSize();

    // indexes remain the same as output indexes
    auto input_reg = GetInputRegion(output_idx, output_size);

    // crop the region if it is partly out of the source image
    bool crop_success = input_reg.Crop(input_ptr->GetLargestPossibleRegion());
    assert(crop_success);
    if (!crop_success) {
        // store what we tried to request (prior to trying to crop)
        input_ptr->SetRequestedRegion(input_reg);

        // throw an exception
        itk::InvalidRequestedRegionError e(__FILE__, __LINE__);
        e.SetLocation(ITK_LOCATION);
        e.SetDescription(
              "Requested region is outside the largest possible region.");
        e.SetDataObject(input_ptr);
        throw e;
    }

    input_ptr->SetRequestedRegion(input_reg);
}

template <class TInputImage, class TOutputImage>
void FrequencyShiftFilter<TInputImage,
                          TOutputImage>::GenerateOutputInformation() {
    Superclass::GenerateOutputInformation();

    auto input_ptr = this->GetInput();
    auto output_ptr = this->GetOutput();

    OutputImageRegionType output_largest_reg;
    auto output_largest_size = input_ptr->GetLargestPossibleRegion().GetSize();
    output_largest_size[0] -= std::ceil(std::abs(col_shift_));
    output_largest_size[1] -= std::ceil(std::abs(row_shift_));
    output_largest_reg.SetSize(output_largest_size);

    output_largest_reg.SetIndex(
          input_ptr->GetLargestPossibleRegion().GetIndex());

    output_ptr->SetLargestPossibleRegion(output_largest_reg);
    // by default spacing will be the same as input one
    // no need to move origin since otb works with centered pixels
}

template <class TInputImage, class TOutputImage>
sirius::Image
FrequencyShiftFilter<TInputImage, TOutputImage>::GenerateImageFromRegion(
      const InputImageRegionType& region) {
    auto region_size = region.GetSize();
    sirius::Image input_image(
          {static_cast<int>(region_size[1]), static_cast<int>(region_size[0])});

    itk::ImageRegionConstIteratorWithIndex<TInputImage> it(this->GetInput(),
                                                           region);

    int image_index = 0;
    for (it.GoToBegin(); !it.IsAtEnd(); ++it) {
        input_image.data[image_index] = static_cast<double>(it.Get());
        ++image_index;
    }

    return input_image;
}

template <class TInputImage, class TOutputImage>
typename TInputImage::RegionType
FrequencyShiftFilter<TInputImage, TOutputImage>::GetInputRegion(
      const IndexType& idx, const SizeType& size) {
    InputImageRegionType input_region;
    input_region.SetIndex(idx);
    auto in_size = size;
    in_size[0] += std::ceil(std::abs(col_shift_));
    in_size[1] += std::ceil(std::abs(row_shift_));
    input_region.SetSize(in_size);

    auto largest_region = this->GetInput()->GetLargestPossibleRegion();

    // crop the region if it is partly out of the source image
    bool crop_success = input_region.Crop(largest_region);
    assert(crop_success);
    if (!crop_success) {
        // throw an exception
        itk::InvalidRequestedRegionError e(__FILE__, __LINE__);
        e.SetLocation(ITK_LOCATION);
        e.SetDescription(
              "Requested region is outside the largest possible region.");
        e.SetDataObject(const_cast<TInputImage*>(this->GetInput()));
        throw e;
    }

    return input_region;
}

template <class TInputImage, class TOutputImage>
void FrequencyShiftFilter<TInputImage, TOutputImage>::ThreadedGenerateData(
      const OutputImageRegionType& outputRegionForThread, itk::ThreadIdType) {
    auto input_region = GetInputRegion(outputRegionForThread.GetIndex(),
                                       outputRegionForThread.GetSize());

    auto input_block = GenerateImageFromRegion(input_region);
    auto shifted_block = frequency_shifter_->Compute(input_block);

    itk::ImageRegionIterator<OutputImageType> oit(this->GetOutput(),
                                                  outputRegionForThread);
    auto shifted_block_it = shifted_block.data.cbegin();
    auto shifted_block_end_it = shifted_block.data.cend();
    for (oit.GoToBegin();
         !oit.IsAtEnd() && shifted_block_it != shifted_block_end_it;
         ++oit, ++shifted_block_it) {
        oit.Set(static_cast<OutputPixelType>(*shifted_block_it));
    }
}

template <class TInputImage, class TOutputImage>
void FrequencyShiftFilter<TInputImage, TOutputImage>::Init(
      sirius::ImageDecompositionPolicies image_decomposition, float row_shift,
      float col_shift) {
    row_shift_ = row_shift;
    col_shift_ = col_shift;
    frequency_shifter_ = sirius::FrequencyTranslationFactory::Create(
          image_decomposition, row_shift_, col_shift_);
}

}  // namespace otb

#endif  // OTB_FREQUENCY_SHIFT_FILTER_TXX_

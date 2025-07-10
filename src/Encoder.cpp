/*
**************************************************************************************************
* Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
* SPDX-License-Identifier: BSD-3-Clause-Clear
**************************************************************************************************
*/

#include "Encoder.hpp"

#include "V4l2Encoder.hpp"

namespace qrb::video_v4l2
{
std::shared_ptr<Client> Encoder::create(std::string mime)
{
  Format f = mime == MIME_H264 ? Format::H264 : Format::HEVC;
  std::shared_ptr<Client> client = V4l2Encoder::create(f);
  return client;
}
}  // namespace qrb::video_v4l2

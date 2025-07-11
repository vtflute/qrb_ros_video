/*
**************************************************************************************************
* Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
* SPDX-License-Identifier: BSD-3-Clause-Clear
**************************************************************************************************
*/

#include "Decoder.hpp"

#include "V4l2Decoder.hpp"

namespace qrb::video_v4l2
{
std::shared_ptr<Client> Decoder::create(std::string mime)
{
  Format f = mime == MIME_H264 ? Format::H264 : Format::HEVC;
  std::shared_ptr<Client> client = V4l2Decoder::create(f);
  return client;
}
}  // namespace qrb::video_v4l2

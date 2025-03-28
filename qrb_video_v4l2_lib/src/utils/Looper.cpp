/*
**************************************************************************************************
* Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
* SPDX-License-Identifier: BSD-3-Clause-Clear
**************************************************************************************************
*/

#include "Looper.hpp"

namespace qrb::video_v4l2
{
Looper::Looper()
{
  running = true;
  looper = std::thread(&Looper::loop, this);
}

Looper::~Looper()
{
  running = false;
  looper.join();
}

bool Looper::sendMessage(const std::shared_ptr<Message> & msg)
{
  {
    std::lock_guard l(lock);
    messages.push_back(msg);
  }
  cv.notify_one();
  return true;
}

bool Looper::handleMessage(const std::shared_ptr<Message> & msg)
{
  bool ret = false;
  if (auto handler = msg->handler.lock()) {
    ret = handler->handleMessage(msg);
  }
  return ret;
}

void Looper::loop()
{
  while (running) {
    std::unique_lock lk(lock);
    while (messages.empty()) {
      cv.wait_for(lk, std::chrono::milliseconds(10));
    }

    auto msg = messages.front();
    messages.erase(messages.begin());
    // unlock before handling the message
    lk.unlock();

    handleMessage(msg);
  }
}
}  // namespace qrb::video_v4l2

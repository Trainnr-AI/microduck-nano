// SPDX-License-Identifier: Apache-2.0
#include "scs_protocol.h"

namespace duck {
namespace scs {

uint8_t checksum(const uint8_t* p, size_t n) {
  uint8_t sum = 0;
  for (size_t i = 0; i < n; ++i) sum = static_cast<uint8_t>(sum + p[i]);
  return static_cast<uint8_t>(~sum);
}

size_t build_packet(uint8_t id, uint8_t inst, const uint8_t* params, size_t n, uint8_t* out, size_t cap) {
  const size_t total = n + 6;
  if (total > cap || n > 250) return 0;
  out[0] = 0xFF;
  out[1] = 0xFF;
  out[2] = id;
  out[3] = static_cast<uint8_t>(n + 2);
  out[4] = inst;
  for (size_t i = 0; i < n; ++i) out[5 + i] = params[i];
  out[5 + n] = checksum(out + 2, n + 3);
  return total;
}

size_t build_ping(uint8_t id, uint8_t* out, size_t cap) { return build_packet(id, kPing, nullptr, 0, out, cap); }

size_t build_read(uint8_t id, uint8_t addr, uint8_t len, uint8_t* out, size_t cap) {
  const uint8_t p[2] = {addr, len};
  return build_packet(id, kRead, p, 2, out, cap);
}

size_t build_write(uint8_t id, uint8_t addr, const uint8_t* data, size_t n, uint8_t* out, size_t cap) {
  uint8_t p[32];
  if (n + 1 > sizeof(p)) return 0;
  p[0] = addr;
  for (size_t i = 0; i < n; ++i) p[1 + i] = data[i];
  return build_packet(id, kWrite, p, n + 1, out, cap);
}

size_t build_sync_write(uint8_t addr, uint8_t len_each, const uint8_t* ids, const uint8_t* data, size_t count,
                        uint8_t* out, size_t cap) {
  uint8_t p[kMaxPacket];
  const size_t n = 2 + count * (static_cast<size_t>(len_each) + 1);
  if (n > sizeof(p) || count == 0) return 0;
  p[0] = addr;
  p[1] = len_each;
  size_t k = 2;
  for (size_t i = 0; i < count; ++i) {
    p[k++] = ids[i];
    for (uint8_t b = 0; b < len_each; ++b) p[k++] = data[i * len_each + b];
  }
  return build_packet(kBroadcastId, kSyncWrite, p, n, out, cap);
}

size_t find_header(const uint8_t* buf, size_t n) {
  size_t i = 0;
  while (i + 1 < n && !(buf[i] == 0xFF && buf[i + 1] == 0xFF)) ++i;
  while (i + 2 < n && buf[i + 2] == 0xFF) ++i;
  return i;
}

Parse parse_reply(const uint8_t* buf, size_t n, Reply* out) {
  const size_t i = find_header(buf, n);
  if (i + 1 >= n) return Parse::kIncomplete;
  const uint8_t* p = buf + i + 2;  // ID LEN ERROR params... CHECKSUM
  const size_t left = n - i - 2;
  if (left < 2) return Parse::kIncomplete;
  const uint8_t len = p[1];
  if (len < 2 || static_cast<size_t>(len - 2) > sizeof(out->params)) return Parse::kBadLength;
  if (left < static_cast<size_t>(len) + 2) return Parse::kIncomplete;
  if (checksum(p, static_cast<size_t>(len) + 1) != p[len + 1]) return Parse::kBadChecksum;
  out->id = p[0];
  out->error = p[2];
  out->n = static_cast<uint8_t>(len - 2);
  for (uint8_t k = 0; k < out->n; ++k) out->params[k] = p[3 + k];
  return Parse::kOk;
}

}  // namespace scs
}  // namespace duck

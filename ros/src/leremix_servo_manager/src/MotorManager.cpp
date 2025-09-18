#include "MotorManager.h"

#include <stdexcept>
#include <system_error>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <termios.h>
#ifdef __linux__
// Need termios functions but also termios2 structure
#define termios asmtermios
#include <asm/termbits.h>
#undef termios
#endif

#ifdef __linux__
// On Linux, use system termios2 from asm/termbits.h
#ifndef TCGETS2
#define TCGETS2     _IOR('T', 0x2A, struct termios2)
#define TCSETS2     _IOW('T', 0x2B, struct termios2)
#endif

#ifndef BOTHER
#define BOTHER 0010000
#endif

#ifndef CBAUD
#define CBAUD 0010017
#endif
#else
// Fallback termios2 definition for non-Linux systems
struct termios2 {
    tcflag_t c_iflag;
    tcflag_t c_oflag;
    tcflag_t c_cflag;
    tcflag_t c_lflag;
    cc_t     c_line;
    cc_t     c_cc[19];
    speed_t  c_ispeed;
    speed_t  c_ospeed;
};

#define TCGETS2     _IOR('T', 0x2A, struct termios2)
#define TCSETS2     _IOW('T', 0x2B, struct termios2)
#define BOTHER 0010000
#define CBAUD 0010017
#endif

// ----- internal helpers -----
static inline uint8_t cs(const std::vector<uint8_t>& f) {
    uint16_t s = 0;
    // sum from ID (idx=2) to last-1 (exclude checksum itself)
    for (size_t i = 2; i + 1 < f.size(); ++i) s += f[i];
    return static_cast<uint8_t>(~(s & 0xFF));
}

uint8_t MotorManager::checksum(const std::vector<uint8_t>& f) { return cs(f); }

std::vector<uint8_t>
MotorManager::makeFrame(uint8_t id, uint8_t inst, const std::vector<uint8_t>& params) {
    std::vector<uint8_t> fr;
    fr.reserve(6 + params.size());
    fr.push_back(0xFF);
    fr.push_back(0xFF);
    fr.push_back(id);
    fr.push_back(static_cast<uint8_t>(params.size() + 2)); // LEN = INST + PARAMS + CHK
    fr.push_back(inst);
    fr.insert(fr.end(), params.begin(), params.end());
    fr.push_back(0x00);
    fr.back() = checksum(fr);
    return fr;
}

bool MotorManager::writeFrame(const std::vector<uint8_t>& frame) {
    ssize_t w = ::write(fd_, frame.data(), frame.size());
    return (w == static_cast<ssize_t>(frame.size()));
}

bool MotorManager::readExact(void* buf, size_t nbytes, int timeout_ms) {
    uint8_t* p = static_cast<uint8_t*>(buf);
    size_t rem = nbytes;
    while (rem > 0) {
        struct pollfd pfd{fd_, POLLIN, 0};
        int pr = ::poll(&pfd, 1, timeout_ms);
        if (pr <= 0) return false; // timeout or error
        ssize_t r = ::read(fd_, p, rem);
        if (r <= 0) return false;
        p += r;
        rem -= static_cast<size_t>(r);
    }
    return true;
}

std::optional<std::vector<uint8_t>>
MotorManager::transact(uint8_t id, uint8_t inst, const std::vector<uint8_t>& params, bool expect_status) {
    auto fr = makeFrame(id, inst, params);
    if (!writeFrame(fr)) return std::nullopt;

    if (!expect_status || id == BROADCAST_ID) {
        return std::vector<uint8_t>{}; // no status expected
    }

    // Status: 0xFF 0xFF ID LEN ERR DATA... CHK
    uint8_t hdr[4];
    if (!readExact(hdr, 4, 20)) return std::nullopt;  // Reduced timeout from 50ms to 20ms
    if (hdr[0] != 0xFF || hdr[1] != 0xFF || hdr[2] != id) return std::nullopt;
    uint8_t len = hdr[3];
    std::vector<uint8_t> body(len);
    if (!readExact(body.data(), len, 20)) return std::nullopt;  // Reduced timeout from 50ms to 20ms
    // Optionally verify checksum here
    return body; // [ERR, DATA..., CHK]
}

// ----- serial open/close -----
void MotorManager::openSerial(const std::string& dev, int baud) {
    fd_ = ::open(dev.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) throw std::runtime_error("open serial failed: " + dev);

    termios tio{};
    if (tcgetattr(fd_, &tio) < 0) throw std::runtime_error("tcgetattr failed");
    cfmakeraw(&tio);
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~CRTSCTS; // no hw flow
    tio.c_cflag &= ~CSTOPB;  // 1 stop
    tio.c_cflag &= ~PARENB;  // no parity
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;      // 8 data bits
    if (tcsetattr(fd_, TCSANOW, &tio) < 0) throw std::runtime_error("tcsetattr failed");

    // Use termios2 for arbitrary baud (e.g., 1_000_000 or 2_000_000)
    struct termios2 tio2{};
    if (ioctl(fd_, TCGETS2, &tio2) != 0) throw std::runtime_error("TCGETS2 failed");
    tio2.c_cflag &= ~CBAUD;
    tio2.c_cflag |= BOTHER | CS8 | CLOCAL | CREAD;
    tio2.c_ispeed = baud;
    tio2.c_ospeed = baud;
    if (ioctl(fd_, TCSETS2, &tio2) != 0) throw std::runtime_error("TCSETS2 failed");

    usleep(1000);  // Reduced from 2ms to 1ms for faster startup
}


void MotorManager::closeSerial() {
    if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
}

MotorManager::MotorManager(const std::string& serial_port, int baud) { openSerial(serial_port, baud); }
MotorManager::~MotorManager() { closeSerial(); }

// ----- high-level convenience -----
bool MotorManager::ping(uint8_t id) {
    auto s = transact(id, 0x01 /*PING*/, {}, true);
    return s.has_value();
}

bool MotorManager::setMode(uint8_t id, uint8_t mode) {
    return writeRegs(id, Reg::OP_MODE, {mode});
}

bool MotorManager::setTorque(uint8_t id, bool on) {
    return writeRegs(id, Reg::TORQUE_ENABLE, {static_cast<uint8_t>(on ? 1 : 0)});
}

bool MotorManager::setAcceleration(uint8_t id, uint8_t accel) {
    return writeRegs(id, Reg::ACCEL, {accel});
}

bool MotorManager::setSpeed(uint8_t id, uint16_t speed_raw) {
    uint8_t H = (speed_raw >> 8) & 0xFF, L = speed_raw & 0xFF;
    return writeRegs(id, Reg::RUN_SPEED_H, {H, L});
}

bool MotorManager::moveTo(uint8_t id, uint16_t pos_ticks, uint16_t time_ms, uint16_t speed_raw) {
    std::vector<uint8_t> d;
    d.reserve(6);
    // Goal pos: L,H
    d.push_back(static_cast<uint8_t>(pos_ticks & 0xFF));
    d.push_back(static_cast<uint8_t>((pos_ticks >> 8) & 0xFF));
    // Time: H,L
    d.push_back(static_cast<uint8_t>((time_ms >> 8) & 0xFF));
    d.push_back(static_cast<uint8_t>(time_ms & 0xFF));
    // Speed: H,L
    d.push_back(static_cast<uint8_t>((speed_raw >> 8) & 0xFF));
    d.push_back(static_cast<uint8_t>(speed_raw & 0xFF));
    return writeRegs(id, Reg::GOAL_POS_L, d);
}

std::optional<uint16_t> MotorManager::readPresentPosition(uint8_t id) {
    auto s = readRegs(id, Reg::PRESENT_POS_H, 2);
    if (!s || s->size() < 4) return std::nullopt;
    uint8_t H = (*s)[1], L = (*s)[2];
    return static_cast<uint16_t>((H << 8) | L);
}

std::optional<int16_t> MotorManager::readPresentSpeed(uint8_t id) {
    auto s = readRegs(id, Reg::PRESENT_SPEED_H, 2);
    if (!s || s->size() < 4) return std::nullopt;
    uint8_t H = (*s)[1], L = (*s)[2];
    return static_cast<int16_t>((H << 8) | L);
}

std::optional<uint8_t> MotorManager::readTemperature(uint8_t id) {
    auto s = readRegs(id, Reg::PRESENT_TEMP, 1);
    if (!s || s->size() < 3) return std::nullopt;
    return (*s)[1];
}

std::optional<float> MotorManager::readVoltage(uint8_t id) {
    auto s = readRegs(id, Reg::PRESENT_VOLT, 1);
    if (!s || s->size() < 3) return std::nullopt;
    return static_cast<float>((*s)[1]) * 0.1f; // adjust if your model differs
}

// ----- batch helpers -----
bool MotorManager::syncWriteVelocity(const std::vector<uint8_t>& ids,
                                     const std::vector<int16_t>& speeds_raw_signed) {
    if (ids.empty() || ids.size() != speeds_raw_signed.size()) return false;
    std::vector<uint8_t> params;
    params.reserve(2 + ids.size() * 3);
    params.push_back(Reg::RUN_SPEED_H); // start addr
    params.push_back(0x02);             // bytes per servo
    for (size_t i = 0; i < ids.size(); ++i) {
        int16_t sv = speeds_raw_signed[i];
        uint8_t H = (static_cast<uint16_t>(sv) >> 8) & 0xFF;
        uint8_t L = static_cast<uint16_t>(sv) & 0xFF;
        params.push_back(ids[i]);
        params.push_back(H);
        params.push_back(L);
    }
    auto r = transact(BROADCAST_ID, 0x83 /*SYNC_WRITE*/, params, false);
    return r.has_value();
}

bool MotorManager::syncWritePositionTimeSpeed(const std::vector<uint8_t>& ids,
                                              const std::vector<uint16_t>& pos_ticks,
                                              const std::vector<uint16_t>& time_ms,
                                              const std::vector<uint16_t>& speed_raw) {
    size_t n = ids.size();
    if (n == 0 || pos_ticks.size() != n || time_ms.size() != n || speed_raw.size() != n) return false;

    std::vector<uint8_t> params;
    params.reserve(2 + n * (1 + 6));
    params.push_back(Reg::GOAL_POS_L); // start at 0x2A
    params.push_back(0x06);            // bytes per servo: pos(2) + time(2) + speed(2)

    for (size_t i = 0; i < n; ++i) {
        params.push_back(ids[i]);
        // pos L,H
        params.push_back(static_cast<uint8_t>(pos_ticks[i] & 0xFF));
        params.push_back(static_cast<uint8_t>((pos_ticks[i] >> 8) & 0xFF));
        // time H,L
        params.push_back(static_cast<uint8_t>((time_ms[i] >> 8) & 0xFF));
        params.push_back(static_cast<uint8_t>(time_ms[i] & 0xFF));
        // speed H,L
        params.push_back(static_cast<uint8_t>((speed_raw[i] >> 8) & 0xFF));
        params.push_back(static_cast<uint8_t>(speed_raw[i] & 0xFF));
    }

    auto r = transact(BROADCAST_ID, 0x83 /*SYNC_WRITE*/, params, false);
    return r.has_value();
}

bool MotorManager::stopInPlace(const std::vector<uint8_t>& ids, uint16_t lock_time_ms) {
    // Queue per-servo: set mode=0 (pos), goal=current position, time=lock_time
    for (auto id : ids) {
        auto p = readPresentPosition(id);
        if (!p) return false;

        if (!regWrite(id, Reg::OP_MODE, {0x00})) return false; // position mode
        std::vector<uint8_t> d;
        d.push_back(static_cast<uint8_t>(*p & 0xFF));           // pos L
        d.push_back(static_cast<uint8_t>((*p >> 8) & 0xFF));    // pos H
        d.push_back(static_cast<uint8_t>((lock_time_ms >> 8) & 0xFF)); // time H
        d.push_back(static_cast<uint8_t>(lock_time_ms & 0xFF));        // time L
        if (!regWrite(id, Reg::GOAL_POS_L, d)) return false;
    }
    return action(); // broadcast apply
}

std::vector<uint8_t> MotorManager::scan(uint8_t start_id, uint8_t end_id) {
    std::vector<uint8_t> out;
    if (end_id < start_id) return out;
    for (uint8_t id = start_id; id <= end_id; ++id) {
        if (ping(id)) out.push_back(id);
    }
    return out;
}

// ----- low-level register access -----
std::optional<std::vector<uint8_t>> MotorManager::readRegs(uint8_t id, uint8_t addr, uint8_t len) {
    return transact(id, 0x02 /*READ*/, {addr, len}, true);
}

bool MotorManager::writeRegs(uint8_t id, uint8_t addr, const std::vector<uint8_t>& data) {
    std::vector<uint8_t> p; p.reserve(1 + data.size());
    p.push_back(addr);
    p.insert(p.end(), data.begin(), data.end());
    return transact(id, 0x03 /*WRITE*/, p, true).has_value();
}

bool MotorManager::regWrite(uint8_t id, uint8_t addr, const std::vector<uint8_t>& data) {
    std::vector<uint8_t> p; p.reserve(1 + data.size());
    p.push_back(addr);
    p.insert(p.end(), data.begin(), data.end());
    return transact(id, 0x04 /*REG_WRITE*/, p, true).has_value();
}

bool MotorManager::action() {
    return transact(BROADCAST_ID, 0x05 /*ACTION*/, {}, false).has_value();
}

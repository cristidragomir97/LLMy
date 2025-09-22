#pragma once
#include <cstdint>
#include <string>
#include <vector>
#include <optional>

/**
 * Minimal FEETECH STS/SC bus driver (e.g., ST3215).
 * Half-duplex TTL bus via USB-UART adapter.
 *
 * Packet format:
 *   0xFF 0xFF ID LEN INST PARAMS... CHK
 * Checksum = ~((ID + LEN + INST + PARAMS) & 0xFF)
 */
class MotorManager {
public:
    // ---- lifecycle ----
    // Throws std::runtime_error on failure.
    explicit MotorManager(const std::string& serial_port, int baud = 1000000);
    ~MotorManager();

    // Not copyable
    MotorManager(const MotorManager&) = delete;
    MotorManager& operator=(const MotorManager&) = delete;

    // ---- high-level convenience ----
    bool ping(uint8_t id);

    // Mode: 0 = position, 1 = speed closed-loop, 2 = speed open-loop, 3 = step mode
    bool setMode(uint8_t id, uint8_t mode);
    bool setTorque(uint8_t id, bool on);
    bool setAcceleration(uint8_t id, uint8_t accel);
    bool setSpeed(uint8_t id, uint16_t speed_raw); // write "running speed" (H,L)
    bool setVelocity(uint8_t id, int16_t velocity); // write individual velocity (signed)

    // Composite move (position/time/speed) starting at GOAL_POS_L (0x2A).
    bool moveTo(uint8_t id, uint16_t pos_ticks, uint16_t time_ms, uint16_t speed_raw);

    // Telemetry
    std::optional<uint16_t> readPresentPosition(uint8_t id);
    std::optional<int16_t>  readPresentSpeed(uint8_t id);
    std::optional<uint8_t>  readTemperature(uint8_t id);
    std::optional<float>    readVoltage(uint8_t id); // 1 LSB ~ 0.1 V on many models

    // Batch helpers
    bool syncWriteVelocity(const std::vector<uint8_t>& ids,
                           const std::vector<int16_t>& speeds_raw_signed);
    bool syncWritePositionTimeSpeed(const std::vector<uint8_t>& ids,
                                    const std::vector<uint16_t>& pos_ticks,
                                    const std::vector<uint16_t>& time_ms,
                                    const std::vector<uint16_t>& speed_raw);

    // Stop & hold: switch to position mode at present angle and lock (REG_WRITE + ACTION)
    bool stopInPlace(const std::vector<uint8_t>& ids, uint16_t lock_time_ms = 20);

    // Scan helper (PING)
    std::vector<uint8_t> scan(uint8_t start_id = 1, uint8_t end_id = 30);

    // ---- low-level register access ----
    std::optional<std::vector<uint8_t>> readRegs(uint8_t id, uint8_t addr, uint8_t len);
    bool writeRegs(uint8_t id, uint8_t addr, const std::vector<uint8_t>& data);
    bool regWrite(uint8_t id, uint8_t addr, const std::vector<uint8_t>& data);
    bool action(); // broadcast ACTION

    // Exposed for advanced users
    static constexpr uint8_t BROADCAST_ID = 0xFE;

    struct Reg {
        // Common STS/ST3215 control table (adjust if your firmware differs)
        static constexpr uint8_t TORQUE_ENABLE    = 0x18; // 1B
        static constexpr uint8_t OP_MODE          = 0x21; // 0 pos, 1 speed(closed), 2 open, 3 step
        static constexpr uint8_t ACCEL            = 0x27; // 1B

        static constexpr uint8_t GOAL_POS_L       = 0x2A; // L,H
        static constexpr uint8_t RUN_TIME_H       = 0x2C; // H,L
        static constexpr uint8_t RUN_SPEED_H      = 0x2E; // H,L

        static constexpr uint8_t PRESENT_POS_H    = 0x38; // H,L
        static constexpr uint8_t PRESENT_SPEED_H  = 0x3A; // H,L
        static constexpr uint8_t PRESENT_VOLT     = 0x3C; // 1B (≈0.1V)
        static constexpr uint8_t PRESENT_TEMP     = 0x3D; // 1B (°C)
    };

private:
    int fd_ = -1;

    // serial
    void openSerial(const std::string& dev, int baud);
    void closeSerial();

    // packets
    bool writeFrame(const std::vector<uint8_t>& frame);
    std::optional<std::vector<uint8_t>> transact(uint8_t id, uint8_t inst,
                                                 const std::vector<uint8_t>& params,
                                                 bool expect_status);
    static std::vector<uint8_t> makeFrame(uint8_t id, uint8_t inst,
                                          const std::vector<uint8_t>& params);
    static uint8_t checksum(const std::vector<uint8_t>& frame);

    // timing/read utils
    bool readExact(void* buf, size_t nbytes, int timeout_ms);
};

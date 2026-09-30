// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <dlpack/dlpack.h>
#include <hololink/emulation/coe_data_plane.hpp>
#include <hololink/emulation/data_plane.hpp>
#include <hololink/emulation/hsb_emulator.hpp>
#include <hololink/emulation/linux_data_plane.hpp>
#include <hololink/emulation/sensors/vb1940_emulator.hpp>

#include <array>
#include <memory>
#include <mutex>
#include <set>
#include <string>

namespace isaacsim::hsb::core
{

/**
 * @struct CameraCalibration
 * @brief Stereo VB1940 calibration values programmed into the rig EEPROM region
 *        of the underlying `Vb1940Emulator` (I²C peripheral 0x51), in the layout
 *        consumed by hololink's `NativeVb1940Sensor::read_calibration_from_eeprom()`.
 * @details
 * The 256-byte region holds 30 big-endian IEEE-754 doubles in this fixed order:
 *
 *     [L_fx, L_fy, L_cx, L_cy, L_k1, L_k2, L_p1, L_p2,
 *      L_k3, L_k4, L_k5, L_k6, R_fx, R_fy, R_cx, R_cy,
 *      R_k1, R_k2, R_p1, R_p2, R_k3, R_k4, R_k5, R_k6,
 *      Rx,   Ry,   Rz,   Tx,   Ty,   Tz]
 *
 * Real hardware stores this in the rig EEPROM; in sim we serve it from the same I²C
 * channel via the modified `Vb1940Emulator` so unmodified staging code sees the same
 * calibration path.
 */
struct CameraCalibration
{
    /** @brief Per-imager intrinsics ordered as fx, fy, cx, and cy. Index 0 is left and index 1 is right. */
    std::array<std::array<double, 4>, 2> intrinsics{};

    /** @brief Per-imager distortion ordered as k1, k2, p1, p2, k3, k4, k5, and k6. */
    std::array<std::array<double, 8>, 2> distortion{};

    /** @brief Rig rotation in axis-angle form, ordered as Rx, Ry, and Rz. Zero represents the identity rotation. */
    std::array<double, 3> R{};

    /**
     * @brief Rig translation ordered as Tx, Ty, and Tz and written directly to the EEPROM T field.
     *
     * The receiver reconstructs `base_T_cam` as the inverse of the calibrated pose, so this value is the negated
     * physical baseline. For Eagle hardware with a physical +0.088 m baseline along +x, use (-0.088, 0, 0).
     */
    std::array<double, 3> T{};
};

namespace detail
{

/**
 * @struct SharedHSBEmulator
 * @brief HSB emulator shared across all HSBSenders targeting the same IP address.
 * @details
 * For stereo VB1940 (and any multi-sensor configuration), the real Leopard Eagle board exposes
 * one Hololink + multiple VB1940 imagers at different I²C bus offsets. A hololink receiver
 * enumerates a *single* board via BootP and then opens sensor 0 and sensor 1 on it. To mirror
 * that in the emulator, two `HSBSender` instances configured with the
 * same `ipAddress` must share a single `HSBEmulator` (rather than each spinning up its own board
 * on the same address).
 *
 * Lifetime is managed via `shared_ptr` — the emulator is created lazily on the first
 * `HSBSender::connect()` for a given IP and destroyed automatically when the last sender for
 * that IP drops its reference. `takenSensorIds` is the per-emulator slot table; duplicate
 * `sensorId` requests on the same IP are rejected at connect time.
 */
struct SharedHSBEmulator
{
    /** @brief Construct the shared emulator for an IP address. @param[in] ipAddress Emulated board IP address. */
    explicit SharedHSBEmulator(const std::string& ipAddress);
    ~SharedHSBEmulator();

    SharedHSBEmulator(const SharedHSBEmulator&) = delete;
    SharedHSBEmulator& operator=(const SharedHSBEmulator&) = delete;

    hololink::emulation::HSBEmulator emulator; ///< Emulator shared by senders targeting `ipAddress`.
    /**
     * @brief Serialize slot reservation and emulator stop, attach, and start operations.
     *
     * This prevents concurrent senders from interleaving lifecycle operations on a partially configured emulator.
     */
    std::mutex connectMutex;
    std::set<uint8_t> takenSensorIds; ///< Sensor identifiers currently reserved by connected senders.
    std::string ipAddress; ///< Emulated board IP address shared by the senders.
};

} // namespace detail

/**
 * @class HSBSender
 * @brief Manages the HSB emulator lifecycle and sends DLTensor data via a DataPlane.
 * @details
 * Owns a `DataPlane` (Linux/RoCEv2 or COE/IEEE 1722B) and a `Vb1940Emulator`, both attached to a
 * `HSBEmulator` that is *shared* across HSBSenders targeting the same `ipAddress`. This sharing
 * enables stereo VB1940 (one virtual Leopard Eagle board, two imagers at sensor_id 0 and 1)
 * without each sender spinning up its own redundant emulator on the same IP. Supports "linux"
 * (RoCEv2 UDP) or "coe" (IEEE 1722B Camera-over-Ethernet) data planes.
 */
class HSBSender
{
public:
    /**
     * @brief Construct an HSB sender
     * @param ipAddress Source IP address for the HSB emulator
     * @param dataPlaneId Data plane identifier
     * @param sensorId Sensor identifier
     * @param dataPlaneType "linux" for RoCEv2 UDP transport, "coe" for IEEE 1722B Camera-over-Ethernet
     */
    HSBSender(const std::string& ipAddress,
              uint8_t dataPlaneId,
              uint8_t sensorId,
              const std::string& dataPlaneType = "linux");

    /**
     * @brief Disconnect on destruction.
     */
    ~HSBSender();

    /**
     * @brief Connect to the HSB data plane
     * @return True if the connection was established successfully, false otherwise. Fails if
     *         another sender on the same `ipAddress` has already claimed this `sensorId`.
     */
    bool connect();

    /**
     * @brief Disconnect from the HSB data plane
     * @return True if the disconnection was successful, false otherwise
     */
    bool disconnect();

    /**
     * @brief Check whether the sender is currently connected
     * @return True if connected, false otherwise
     */
    bool isConnected() const;

    /**
     * @brief Send a DLTensor via the HSB data plane
     * @param tensor The tensor to send (CPU or GPU data)
     * @return True if sent successfully, false otherwise
     */
    bool send(const DLTensor& tensor);

    /**
     * @brief Check if the host has set the sensor to streaming mode
     * @return True if the Vb1940Emulator is streaming, false otherwise
     */
    bool isStreaming() const;

    /**
     * @brief Whether this sender and `other` are bound to the same underlying HSBEmulator.
     * @return True only when both senders have connected and their shared-emulator pointers
     *         match (i.e. both resolved to the same entry in the IP-keyed shared registry).
     *         Useful for tests, diagnostics, and any future multi-emulator topologies where a
     *         consumer needs to reason about emulator identity across senders.
     */
    bool hasSharedEmulatorWith(const HSBSender& other) const;

    /**
     * @brief Populate this sender's EEPROM with stereo VB1940 calibration data.
     * @details
     * In real hardware the calibration is programmed into the rig's I²C EEPROM at
     * peripheral address 0x51; the live receiver (hololink's `NativeVb1940Sensor`)
     * reads it via the standard I²C path. In sim `Vb1940Emulator` backs that same
     * I²C region with an internal 256-byte buffer, which we program here (via
     * `Vb1940Emulator::set_eeprom_data()`) so the receiver gets the same data.
     *
     * Safe to call before or after `connect()`; subsequent calls overwrite the
     * previous values. If called before connect(), the values are staged and
     * applied on next connect().
     */
    void setCalibration(const CameraCalibration& calib);

    /**
     * @brief Encode a CameraCalibration into the 256-byte big-endian-doubles layout that
     *        `NativeVb1940Sensor::read_calibration_from_eeprom()` decodes.
     * @details
     * Pure static utility — same encoding that `setCalibration()` uses internally, exposed
     * so callers can inspect the on-wire byte pattern, unit-test the layout, or program
     * the EEPROM through a channel other than the HSBSender lifecycle. The output buffer
     * is always fully overwritten (`fill(0)` first, then 30 doubles at offsets 0..239;
     * bytes 240..255 remain zero as padding).
     */
    static void encodeCalibrationBytes(const CameraCalibration& calib, std::array<uint8_t, 256>& out);

private:
    /**
     * @brief Get or create the shared emulator for `ipAddress`.
     * @details
     * First caller for a given IP constructs a fresh `SharedHSBEmulator`; subsequent callers
     * reuse it. The emulator is *not* started here — the hololink `I2CController` refuses
     * peripheral attachments while the emulator is running, so each `HSBSender::connect()`
     * drives its own stop-attach-start cycle after acquiring the shared instance. Cleanup is
     * automatic: when the last `shared_ptr` is dropped (i.e. the last HSBSender for that IP
     * is destroyed), the `SharedHSBEmulator` destructor runs and the registry's `weak_ptr`
     * becomes expired.
     */
    static std::shared_ptr<detail::SharedHSBEmulator> acquireSharedEmulator(const std::string& ipAddress);

    std::shared_ptr<detail::SharedHSBEmulator> m_shared;
    std::unique_ptr<hololink::emulation::DataPlane> m_dataPlane;
    std::unique_ptr<hololink::emulation::sensors::Vb1940Emulator> m_vb1940;
    CameraCalibration m_pendingCalibration{}; // staged values applied at connect()
    bool m_hasCalibration{ false }; // true once setCalibration() has been called
    std::string m_ipAddress;
    uint8_t m_dataPlaneId;
    uint8_t m_sensorId;
    std::string m_dataPlaneType;
    bool m_connected;
    // Per-instance "not streaming yet" one-shot log latch. Was a function-local static, which
    // (a) was a data race across concurrent send() calls from stereo senders, and (b) masked
    // the right sender's warning behind the left sender's flag.
    bool m_streamingWarningLogged;
};

} // namespace isaacsim::hsb::core

// CapstoneProtocol.h
//
// Shared ESP-NOW wire protocol for the car wash transmitter (remote) and
// receiver (vehicle) boards.
//
// Design rules:
//   - Explicit byte serialization only. We never memcpy a raw struct onto the
//     wire: struct padding/alignment/endianness differ across toolchains and
//     would silently corrupt data. pack()/unpack() define the exact bytes.
//   - Fixed-size payloads, well under the 250-byte ESP-NOW limit.
//   - Every packet is validated (magic, version, type, length, CRC) before any
//     field is trusted.
//   - No dynamic allocation, no blocking calls. Safe to call from an ESP-NOW
//     receive callback context.
//
// Two message types travel over the link:
//   CommandPacket  : transmitter -> receiver  (digital stick dirs + buttons)
//   FeedbackPacket : receiver   -> transmitter (EoT + output state)
//
// Input model: an 8-way DIGITAL stick (4 switches: U/D/L/R; diagonals are two
// adjacent switches) plus two top buttons (Soap trigger, Arm/enable). There
// are no analog axes. The raw switch state is sent on the wire; the receiver
// applies the motion rule (any UP = forward, any DOWN = reverse).

#ifndef CAPSTONE_PROTOCOL_H
#define CAPSTONE_PROTOCOL_H

#include <stdint.h>
#include <stddef.h>

namespace capstone {

// ---------------------------------------------------------------------------
// Protocol constants
// ---------------------------------------------------------------------------

// 'C','W' = Car Wash. Rejects stray/foreign ESP-NOW traffic quickly.
static const uint16_t kMagic = 0x4357;

// Bump whenever the wire layout changes in an incompatible way.
// v2: switched from analog axes to digital stick direction bits + revised
//     button/status model.
static const uint8_t kProtocolVersion = 2;

enum MessageType : uint8_t {
  kMsgCommand  = 0x01,  // transmitter -> receiver
  kMsgFeedback = 0x02,  // receiver   -> transmitter
};

// Raw digital-stick direction bits for CommandPacket.dirs (set = switch
// pressed). The 8 directions are combinations: N=UP, NE=UP+RIGHT, E=RIGHT,
// SE=DOWN+RIGHT, S=DOWN, SW=DOWN+LEFT, W=LEFT, NW=UP+LEFT.
// Motion rule (applied on the receiver): any UP => forward, any DOWN =>
// reverse, both or neither => stop. LEFT/RIGHT are unused for now.
enum DirBit : uint8_t {
  kDirUp    = 0,  // stick UP    switch
  kDirDown  = 1,  // stick DOWN  switch
  kDirLeft  = 2,  // stick LEFT  switch (currently unused)
  kDirRight = 3,  // stick RIGHT switch (currently unused)
};

// Top-button bit positions for CommandPacket.buttons (set = pressed).
enum ButtonBit : uint8_t {
  kBtnSoap = 0,  // top trigger -> Soap solenoid on the receiver
  kBtnArm  = 1,  // top button  -> arm/enable gate (hold to arm). Not a solenoid.
};

// Transmitter status flags for CommandPacket.status.
enum TxStatusBit : uint8_t {
  kTxArmed = 0,  // operator has armed the remote; required before any motion
};

// End-of-Travel state bits for FeedbackPacket.eot_state.
// Four independent normally-closed microswitches. A set bit means "tripped":
// at end of travel OR a broken wire / failed switch / unplugged connector
// (all read as open). Each terminal block carries two switches.
enum EotBit : uint8_t {
  kEot1 = 0,  // J9  EoTSwitch  pin 1 (GPIO18)
  kEot2 = 1,  // J9  EoTSwitch  pin 2 (GPIO19)
  kEot3 = 2,  // J11 EoTSwitch2 pin 1 (GPIO21)
  kEot4 = 3,  // J11 EoTSwitch2 pin 2 (GPIO22)
};

// Receiver output status bits for FeedbackPacket.output_state.
enum OutputBit : uint8_t {
  kOutMainWater = 0,  // Solenoid 1, GPIO32 (Q3) — main supply valve
  kOutSoap      = 1,  // Solenoid 2, GPIO33 (Q4)
  kOutSprayer   = 2,  // Solenoid 3, GPIO26 (Q5)
  kOutAux       = 3,  // Solenoid 4, GPIO27 (Q6)
  kOutMotorActive = 4, // motor currently being driven
  kOutMotorInhibited = 5, // motor forced off by an EoT trip
};

// ---------------------------------------------------------------------------
// Logical packet structures (host-side representation, NOT the wire layout)
// ---------------------------------------------------------------------------

// Transmitter -> receiver.
struct CommandPacket {
  uint16_t seq;      // sequence number, wraps at 65535 -> 0
  uint8_t  dirs;     // DirBit mask (raw stick switch state)
  uint8_t  buttons;  // ButtonBit mask (top buttons)
  uint8_t  status;   // TxStatusBit mask
};

// Receiver -> transmitter.
struct FeedbackPacket {
  uint16_t seq;          // echoes the last command seq the receiver acted on
  uint8_t  eot_state;    // EotBit mask (bit set = tripped / at end of travel)
  uint8_t  output_state; // OutputBit mask
};

// ---------------------------------------------------------------------------
// Wire sizes
// ---------------------------------------------------------------------------
//
// Frame = header + body + CRC16.
// Header: magic(2) version(1) type(1) length(1)  = 5 bytes
// CRC   : crc16(2)                                = 2 bytes
// Body  : type-specific
//
//   Command body : seq(2) dirs(1) buttons(1) status(1) = 5 bytes
//   Feedback body: seq(2) eot_state(1) output_state(1) = 4 bytes

static const size_t kHeaderSize = 5;
static const size_t kCrcSize    = 2;

static const size_t kCommandBodySize  = 5;
static const size_t kFeedbackBodySize = 4;

static const size_t kCommandFrameSize  = kHeaderSize + kCommandBodySize  + kCrcSize; // 12
static const size_t kFeedbackFrameSize = kHeaderSize + kFeedbackBodySize + kCrcSize; // 11

// Largest frame any side needs to buffer.
static const size_t kMaxFrameSize = kCommandFrameSize; // 12

// ---------------------------------------------------------------------------
// Packing / unpacking
// ---------------------------------------------------------------------------
//
// All multi-byte fields are little-endian on the wire.
// pack* returns the number of bytes written (0 on failure).
// unpack* returns true only if the buffer is a valid, CRC-checked frame of the
// expected type.

size_t packCommand(const CommandPacket& in, uint8_t* out, size_t out_cap);
bool   unpackCommand(const uint8_t* in, size_t in_len, CommandPacket& out);

size_t packFeedback(const FeedbackPacket& in, uint8_t* out, size_t out_cap);
bool   unpackFeedback(const uint8_t* in, size_t in_len, FeedbackPacket& out);

// Returns the message type byte of a frame without fully validating it, or 0
// if the buffer is too short / magic mismatch. Lets a receiver route a frame
// before unpacking.
uint8_t peekType(const uint8_t* in, size_t in_len);

// CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF). Shared by both sides.
uint16_t crc16(const uint8_t* data, size_t len);

// Sequence-number helper: true if `candidate` is newer than `last`, accounting
// for uint16_t wraparound. Used to drop duplicate/out-of-order packets.
bool seqIsNewer(uint16_t candidate, uint16_t last);

}  // namespace capstone

#endif  // CAPSTONE_PROTOCOL_H

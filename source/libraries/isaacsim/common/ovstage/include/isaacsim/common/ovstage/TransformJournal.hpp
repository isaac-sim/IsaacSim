// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <cstdint>
#include <type_traits>

namespace isaacsim
{
namespace common
{
namespace ovstage
{

/** @brief Absolute prim path that owns the optional transform-journal attribute. */
inline constexpr char g_kTransformJournalPrimPath[] = "/_IsaacSimTransformJournal";

/** @brief Attribute name containing the serialized transform journal. */
inline constexpr char g_kTransformJournalAttribute[] = "omni:isaac:transformJournal";

/** @brief Magic value identifying a compatible transform-journal payload. */
inline constexpr uint64_t g_kTransformJournalMagic = 0x4A4E525458465349ULL; // "ISFXTRNJ"

/** @brief Current transform-journal payload version. */
inline constexpr uint64_t g_kTransformJournalVersion = 1;

/**
 * @brief Header for an optional transform-journal payload stored in OVStage.
 *
 * A producer writes the journal on the same stage and seals it at the same ordinal as its authoritative transform
 * updates. A consumer validates the identifier, version, generation, and ordinal continuity before using the entries.
 * The consumer falls back to ordinary OVStage reads when validation fails.
 */
struct TransformJournalHeader
{
    /** @brief Payload identifier, initialized to @c g_kTransformJournalMagic. */
    uint64_t magic{ g_kTransformJournalMagic };

    /** @brief Payload format version, initialized to @c g_kTransformJournalVersion. */
    uint64_t version{ g_kTransformJournalVersion };

    /** @brief Producer generation used to distinguish independent journal chains. */
    uint64_t generation{ 0 };

    /**
     * @brief Ordinal immediately preceding the first uninterrupted publication in this chain.
     *
     * Every ordinal after this value through @ref ordinal contains a complete transform publication from the same
     * producer, allowing consumers to skip intermediate frames safely.
     */
    uint64_t baseOrdinal{ 0 };

    /** @brief OVStage ordinal containing this payload and its authoritative transform writes. */
    uint64_t ordinal{ 0 };

    /** @brief Number of @ref TransformJournalEntry records following this header. */
    uint64_t entryCount{ 0 };
};

/** @brief One world-transform update in a transform-journal payload. */
struct TransformJournalEntry
{
    /** @brief OVStage token identifying the transformed prim path. */
    uint64_t primPath{ 0 };

    /** @brief Row-major 4-by-4 world transformation matrix. */
    double worldMatrix[16]{};
};

static_assert(std::is_trivially_copyable_v<TransformJournalHeader>);
static_assert(std::is_trivially_copyable_v<TransformJournalEntry>);
static_assert(sizeof(TransformJournalHeader) == 48);
static_assert(sizeof(TransformJournalEntry) == 136);

} // namespace ovstage
} // namespace common
} // namespace isaacsim

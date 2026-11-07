#pragma once

#include <cmath>
#include <cstdint>
#include <cstring>
#include <new>

#include "range_map.hh"
#include "swarm_protocols.hh"

/**
 * Range maps for the protocols heard, from a fixed pool plus the heap when more protocols are heard in one period than
 * the pool holds (the sub-GHz radio switching modes mid-period).
 */
class SwarmRangeMaps {
   public:
    static constexpr uint8_t kMapSlots = 3;  // Mode S, one sub-GHz mode, Remote ID.

    explicit SwarmRangeMaps(uint8_t num_slots = kMapSlots) : num_slots_(num_slots < kMapSlots ? num_slots : kMapSlots) {
        memset(slot_, kNoSlot, sizeof(slot_));
    }
    SwarmRangeMaps(const SwarmRangeMaps& other) : SwarmRangeMaps(other.num_slots_) { CopyFrom(other); }
    SwarmRangeMaps(SwarmRangeMaps&& other) : SwarmRangeMaps(other.num_slots_) { *this = static_cast<SwarmRangeMaps&&>(other); }
    SwarmRangeMaps& operator=(const SwarmRangeMaps& other) {
        if (this != &other) {
            Clear();
            CopyFrom(other);
        }
        return *this;
    }
    // Takes other's heap maps instead of copying them.
    SwarmRangeMaps& operator=(SwarmRangeMaps&& other) {
        if (this == &other) return *this;
        Clear();
        for (uint8_t p = 0; p < kNumSwarmProtocols; p++) {
            if (other.slot_[p] == kHeapSlot) {
                heap_[p] = other.heap_[p];
                other.heap_[p] = nullptr;
                other.slot_[p] = kNoSlot;
                slot_[p] = kHeapSlot;
            }
        }
        CopyFrom(other);
        other.Clear();
        return *this;
    }
    ~SwarmRangeMaps() { Clear(); }

    // The protocol's map, or nullptr if nothing was recorded for it.
    const RangeMap* Find(uint8_t protocol) const {
        uint8_t slot = slot_[protocol];
        return slot == kNoSlot ? nullptr : slot == kHeapSlot ? heap_[protocol] : &pool_[slot];
    }
    RangeMap* Find(uint8_t protocol) {
        return const_cast<RangeMap*>(static_cast<const SwarmRangeMaps*>(this)->Find(protocol));
    }

    // The protocol's map, taken from the pool or the heap on first use. nullptr if the heap is out.
    RangeMap* Acquire(uint8_t protocol) {
        if (RangeMap* map = Find(protocol)) return map;
        if (num_used_ < num_slots_) {
            slot_[protocol] = num_used_;
            return &pool_[num_used_++];
        }
        heap_[protocol] = allocate();
        if (heap_[protocol] == nullptr) return nullptr;
        slot_[protocol] = kHeapSlot;
        return heap_[protocol];
    }

    // Map access that creates the map; if the heap is out, writes go to a scratch map and are lost.
    RangeMap& operator[](uint8_t protocol) {
        if (RangeMap* map = Acquire(protocol)) return *map;
        static RangeMap scratch;
        scratch.Clear();
        return scratch;
    }
    // An empty map for a protocol with none.
    const RangeMap& operator[](uint8_t protocol) const {
        static const RangeMap kEmpty;
        const RangeMap* map = Find(protocol);
        return map ? *map : kEmpty;
    }

    // Iterates the maps of all protocols, creating them.
    class Iterator {
       public:
        Iterator(SwarmRangeMaps& maps, uint8_t protocol) : maps_(maps), protocol_(protocol) {}
        RangeMap& operator*() { return maps_[protocol_]; }
        Iterator& operator++() {
            protocol_++;
            return *this;
        }
        bool operator!=(const Iterator& other) const { return protocol_ != other.protocol_; }

       private:
        SwarmRangeMaps& maps_;
        uint8_t protocol_;
    };
    Iterator begin() { return Iterator(*this, 0); }
    Iterator end() { return Iterator(*this, kNumSwarmProtocols); }

    bool Empty() const {
        for (uint8_t p = 0; p < kNumSwarmProtocols; p++) {
            const RangeMap* map = Find(p);
            if (map && !map->Empty()) return false;
        }
        return true;
    }

    // Returns the pool and the heap maps.
    void Clear() {
        for (uint8_t p = 0; p < kNumSwarmProtocols; p++) {
            if (slot_[p] == kHeapSlot) delete heap_[p];
            heap_[p] = nullptr;
        }
        for (RangeMap& map : pool_) map.Clear();
        memset(slot_, kNoSlot, sizeof(slot_));
        num_used_ = 0;
    }

    // Replaceable for tests.
    static inline RangeMap* (*allocate)() = []() -> RangeMap* { return new (std::nothrow) RangeMap(); };

   private:
    static constexpr uint8_t kNoSlot = 0xFF;
    static constexpr uint8_t kHeapSlot = 0xFE;

    // Copies other's maps; one the heap can't give is left out.
    void CopyFrom(const SwarmRangeMaps& other) {
        for (uint8_t p = 0; p < kNumSwarmProtocols; p++) {
            const RangeMap* from = other.Find(p);
            if (from == nullptr) continue;
            if (RangeMap* to = Acquire(p)) *to = *from;
        }
    }

    RangeMap pool_[kMapSlots];
    RangeMap* heap_[kNumSwarmProtocols] = {nullptr};
    uint8_t slot_[kNumSwarmProtocols];  // Pool index, kHeapSlot or kNoSlot.
    uint8_t num_used_ = 0;              // Pool slots handed out, in order.
    uint8_t num_slots_;
};

/**
 * What a swarm report covers: a range map per protocol heard, listen time and frame counts per protocol, and aircraft
 * counts. The aircraft dictionary feeds it.
 */
class SwarmRecorder {
   public:
    static constexpr uint8_t kNumProtocols = kNumSwarmProtocols;
    static constexpr uint8_t kModeS = SwarmProtocolIndex("mode_s");
    static constexpr uint8_t kUAT = SwarmProtocolIndex("uat");
    static constexpr uint8_t kRemoteID = SwarmProtocolIndex("remote_id");
    static constexpr struct Names {
        const char* names[kNumProtocols];
        const char* const* begin() const { return names; }
        const char* const* end() const { return names + kNumProtocols; }
        constexpr const char* operator[](uint8_t p) const { return names[p]; }
    } kProtocolNames = [] {
        Names n{};
        for (uint8_t p = 0; p < kNumProtocols; p++) n.names[p] = kSwarmProtocols[p].name;
        return n;
    }();

    static constexpr uint16_t kUniqueBits = 2048;  // Linear counting: a few % error up to ~3000 aircraft.
    // A period's range map is withheld when at least this share of positions, from this many aircraft, were dropped.
    static constexpr uint32_t kWithholdDroppedPercent = 10;
    static constexpr uint8_t kWithholdDroppedAircraft = 5;

    struct Guard {
        uint32_t num_positions = 0;  // Positions offered to the range map.
        uint32_t num_dropped = 0;    // Of those, beyond the horizon.
        uint8_t num_dropped_aircraft = 0;  // Distinct aircraft dropped, counted up to kWithholdDroppedAircraft.
        uint32_t dropped_uids[kWithholdDroppedAircraft] = {0};
        uint32_t num_no_map = 0;  // Positions (or merged bins) lost because the heap had no map for them.

        void NoteDroppedAircraft(uint32_t uid) {
            for (uint8_t i = 0; i < num_dropped_aircraft; i++) {
                if (dropped_uids[i] == uid) return;
            }
            if (num_dropped_aircraft < kWithholdDroppedAircraft) dropped_uids[num_dropped_aircraft++] = uid;
        }
    };

    using Stats = SwarmV1Stats;

    // Band tops for the radio horizon, in feet; the top band counts as 60,000 ft (aircraft) or 10,000 ft (drones).
    static constexpr int32_t kBandTopFt[RangeMap::kNumBands] = {1000,  2000,  4000,  6000,  10000, 15000,
                                                                20000, 25000, 30000, 35000, 40000, 60000};
    static constexpr int32_t kDroneBandTopFt[RangeMap::kNumBands] = {100,  200,  300,  400,  500,  700,
                                                                     1000, 1500, 2000, 3000, 5000, 10000};
    static constexpr const int32_t* kBandTopFtByBasis[] = {kBandTopFt, kDroneBandTopFt};
    static_assert(sizeof(kBandTopFtByBasis) / sizeof(kBandTopFtByBasis[0]) ==
                  static_cast<uint8_t>(SwarmBandBasis::kNumBases));

    // A band for an altitude: by altitude, or by height above the receiver.
    uint8_t Band(SwarmBandBasis basis, int32_t altitude_ft) const {
        return basis == SwarmBandBasis::kAltitude ? RangeMap::AltitudeBand(altitude_ft)
                                                  : RangeMap::HeightBand(altitude_ft - receiver_altitude_ft_);
    }

    /**
     * Sets the horizon limit per band from the antenna altitude: 1.15 * 4.12 km * (sqrt(h_rx) + sqrt(h_band_top)) + 20
     * km, heights in meters (radio horizon with margin). Call when the receiver altitude changes, not per position.
     */
    void SetReceiverAltitude(int32_t altitude_m) {
        if (altitude_m == receiver_altitude_m_) return;
        receiver_altitude_m_ = altitude_m;
        receiver_altitude_ft_ = static_cast<int32_t>(lroundf(altitude_m / 0.3048f));
        float rx = sqrtf(altitude_m > 0 ? static_cast<float>(altitude_m) : 0.0f);
        for (uint8_t basis = 0; basis < static_cast<uint8_t>(SwarmBandBasis::kNumBases); basis++) {
            for (uint16_t b = 0; b < RangeMap::kNumBands; b++) {
                horizon_m_[basis][b] = HorizonMeters(rx, kBandTopFtByBasis[basis][b]);
            }
        }
    }

    /**
     * A confirmed position heard directly. altitude_ft is barometric or GNSS for aircraft, geodetic for drones. Positions
     * beyond the radio horizon (a wrong receiver position, or a bad decode) are counted as dropped instead.
     */
    inline void RecordPosition(uint8_t protocol, uint32_t distance_m, uint16_t bearing_bam16, int32_t altitude_ft,
                               uint32_t uid) {
        if (!record_range) return;
        const SwarmProtocol& info = kSwarmProtocols[protocol];
        uint8_t band = Band(info.basis, altitude_ft);
        if (!Guarded(distance_m, horizon_m_[static_cast<uint8_t>(info.basis)][band], uid)) return;
        RangeMap* map = ranges.Acquire(protocol);
        if (map == nullptr) {
            guard.num_no_map++;
            return;
        }
        map->Record(band, bearing_bam16, RangeMap::DistanceValue(distance_m, info.unit_m));
    }

    // A confirmed Remote ID position (geodetic altitude).
    inline void RecordDrone(uint32_t distance_m, uint16_t bearing_bam16, int32_t geodetic_altitude_ft, uint32_t uid) {
        RecordPosition(kRemoteID, distance_m, bearing_bam16, geodetic_altitude_ft, uid);
    }

    // Too many positions beyond the horizon: the receiver position is likely wrong, so the map is withheld.
    bool RangeImplausible() const {
        return guard.num_dropped * 100 >= kWithholdDroppedPercent * guard.num_positions &&
               guard.num_dropped_aircraft >= kWithholdDroppedAircraft && guard.num_dropped > 0;
    }

    // An aircraft of a protocol that counts as aircraft, once per dictionary update.
    inline void MarkAircraft(uint32_t uid) {
        // MurmurHash3 finalizer: linear counting needs bits that look random, also for neighboring addresses.
        uid ^= uid >> 16;
        uid *= 0x85EBCA6Bu;
        uid ^= uid >> 13;
        uid *= 0xC2B2AE35u;
        uid ^= uid >> 16;
        uint32_t bit = uid % kUniqueBits;
        unique_bits_[bit / 8] |= 1 << (bit % 8);
    }

    /**
     * Valid frame counts as the receiver reports them each second. second is the protocol's other v1 counter (UAT
     * uplink); protocols reporting "frames" take one count.
     */
    inline void AddFrames(uint8_t protocol, uint32_t valid, uint32_t second = 0) {
        const SwarmProtocol& info = kSwarmProtocols[protocol];
        if (info.v1_stats[0].field) {
            stats.*info.v1_stats[0].field += valid;
            if (info.v1_stats[1].field) stats.*info.v1_stats[1].field += second;
        } else {
            frames[protocol] += valid;
        }
    }
    // The RP2040's counters: Mode S as one total of 56- and 112-bit frames.
    inline void AddRP2040Frames(uint32_t mode_s_valid, uint32_t uat_adsb, uint32_t uat_uplink) {
        AddFrames(kModeS, mode_s_valid);
        AddFrames(kUAT, uat_adsb, uat_uplink);
    }

    // Adds listen time for each protocol whose receiver runs.
    void AddListening(const SwarmListenState& state, uint32_t seconds = 1) {
        listen_known = true;
        for (uint8_t p = 0; p < kNumProtocols; p++) {
            if (kSwarmProtocols[p].listening(state)) listen_s[p] += seconds;
        }
    }

    // Once per dictionary update: the number of aircraft tracked that count as aircraft.
    inline void NoteAircraftCount(uint16_t num_aircraft) {
        if (num_aircraft > stats.aircraft_max) stats.aircraft_max = num_aircraft;
    }

    // Estimated number of different aircraft marked.
    uint16_t UniqueAircraft() const {
        uint16_t zeros = 0;
        for (uint16_t i = 0; i < sizeof(unique_bits_); i++) zeros += 8 - __builtin_popcount(unique_bits_[i]);
        if (zeros == 0) zeros = 1;
        return static_cast<uint16_t>(lroundf(-static_cast<float>(kUniqueBits) * logf(zeros / (float)kUniqueBits)));
    }

    bool RangeEmpty() const { return ranges.Empty(); }

    bool Heard(uint8_t protocol) const {
        const RangeMap* map = ranges.Find(protocol);
        if (map && !map->Empty()) return true;
        const SwarmProtocol& info = kSwarmProtocols[protocol];
        if (!info.v1_stats[0].field) return frames[protocol] > 0;
        return stats.*info.v1_stats[0].field > 0 || (info.v1_stats[1].field && stats.*info.v1_stats[1].field > 0);
    }

    // Goes into a report: listened to, or heard anyway.
    bool Reported(uint8_t protocol) const { return listen_s[protocol] > 0 || Heard(protocol); }

    // Adds a later period: max for the ranges and peak aircraft, sums for counts, union for unique aircraft.
    void Merge(const SwarmRecorder& later) {
        for (uint8_t p = 0; p < kNumProtocols; p++) {
            const RangeMap* from = later.ranges.Find(p);
            if (from == nullptr) continue;
            if (RangeMap* to = ranges.Acquire(p)) {
                to->Merge(*from);
            } else {
                for (uint8_t bin : from->bins) guard.num_no_map += bin != 0;
            }
        }
        for (uint8_t p = 0; p < kNumProtocols; p++) {
            listen_s[p] += later.listen_s[p];
            frames[p] += later.frames[p];
            for (const SwarmProtocol::V1Stat& v1 : kSwarmProtocols[p].v1_stats) {
                if (v1.field) stats.*v1.field += later.stats.*v1.field;
            }
        }
        listen_known = listen_known || later.listen_known;
        if (later.stats.aircraft_max > stats.aircraft_max) stats.aircraft_max = later.stats.aircraft_max;
        for (uint16_t i = 0; i < sizeof(unique_bits_); i++) unique_bits_[i] |= later.unique_bits_[i];
        guard.num_positions += later.guard.num_positions;
        guard.num_dropped += later.guard.num_dropped;
        guard.num_no_map += later.guard.num_no_map;
        for (uint8_t i = 0; i < later.guard.num_dropped_aircraft; i++) {
            guard.NoteDroppedAircraft(later.guard.dropped_uids[i]);
        }
    }

    void Clear() {
        ranges.Clear();
        stats = Stats();
        guard = Guard();
        memset(frames, 0, sizeof(frames));
        memset(listen_s, 0, sizeof(listen_s));
        listen_known = false;
        memset(unique_bits_, 0, sizeof(unique_bits_));
    }

    explicit SwarmRecorder(uint8_t map_slots = SwarmRangeMaps::kMapSlots) : ranges(map_slots) {
        receiver_altitude_m_ = INT32_MIN;
        SetReceiverAltitude(0);
    }

    SwarmRangeMaps ranges;
    Stats stats;
    uint32_t frames[kNumProtocols] = {0};    // Protocols without v1 stats keys.
    uint32_t listen_s[kNumProtocols] = {0};  // Seconds each receiver ran.
    bool listen_known = false;               // AddListening() ran: protocols not listened to stay out of reports.
    Guard guard;
    bool record_range = true;  // False while the receiver position isn't fixed or from GNSS.

   private:
    static uint32_t HorizonMeters(float sqrt_rx_m, int32_t band_top_ft) {
        return static_cast<uint32_t>((1.15f * 4.12f * (sqrt_rx_m + sqrtf(band_top_ft * 0.3048f)) + 20.0f) * 1000.0f);
    }

    // Counts a position for the guard (shared by all protocols). False if it is beyond the horizon.
    inline bool Guarded(uint32_t distance_m, uint32_t horizon_m, uint32_t uid) {
        guard.num_positions++;
        if (distance_m <= horizon_m) return true;
        guard.num_dropped++;
        guard.NoteDroppedAircraft(uid);
        return false;
    }

    uint8_t unique_bits_[kUniqueBits / 8] = {0};
    int32_t receiver_altitude_m_;
    int32_t receiver_altitude_ft_ = 0;
    uint32_t horizon_m_[static_cast<uint8_t>(SwarmBandBasis::kNumBases)][RangeMap::kNumBands];
};

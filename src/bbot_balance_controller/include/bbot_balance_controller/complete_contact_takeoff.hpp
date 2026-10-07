#pragma once

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace bbot_jump
{
struct CompleteGroundContactFrame
{
    uint64_t sequence{0};
    int64_t sim_time_ns{0};
    uint64_t iteration{0};
    int64_t dt_ns{0};
    bool frame_valid{false};
    int64_t num_contacts{-1};
    std::vector<std::pair<std::string, std::string>> pairs;
    std::string error;
    bool decoded{false};
    bool collision_pairs_allowed{false};
    uint8_t wheel_mask{0};
};

// Wire v1: BBOT_GCF1|seq|sim_ns|iteration|dt_ns|valid|num_contacts|pair_count|
// followed by pair_count pairs. Each pair contains two adjacent <byte-count>:<raw-bytes>
// values, then one '|' separator. Finally <error-byte-count>:<raw-error>, with no tail.
// Lengths make names unambiguous without a JSON dependency or separator escaping.
class CompleteGroundContactWire
{
  public:
    static constexpr std::string_view kPrefix = "BBOT_GCF1|";
    static constexpr int64_t kExpectedDtNs = 1'000'000;
    static constexpr int64_t kMaxContacts = 128;
    static constexpr size_t kMaxNameBytes = 256;
    static constexpr size_t kMaxErrorBytes = 512;
    static constexpr size_t kMaxWireBytes = 80 * 1024;

    static bool parse(std::string_view text, CompleteGroundContactFrame &out)
    {
        out = {};
        out.num_contacts = -1;
        if (text.size() > kMaxWireBytes || text.substr(0, kPrefix.size()) != kPrefix)
            return false;
        size_t cursor = kPrefix.size();
        std::string_view token;
        uint64_t valid = 0, pair_count = 0;
        if (!next_token(text, cursor, token) || !parse_unsigned(token, out.sequence) ||
            !next_token(text, cursor, token) || !parse_signed(token, out.sim_time_ns) ||
            !next_token(text, cursor, token) || !parse_unsigned(token, out.iteration) ||
            !next_token(text, cursor, token) || !parse_signed(token, out.dt_ns) ||
            !next_token(text, cursor, token) || !parse_unsigned(token, valid) || valid > 1 ||
            !next_token(text, cursor, token) || !parse_signed(token, out.num_contacts) ||
            !next_token(text, cursor, token) || !parse_unsigned(token, pair_count))
            return false;
        if (out.sim_time_ns < 0 || out.iteration == 0 || out.dt_ns != kExpectedDtNs ||
            pair_count > static_cast<uint64_t>(kMaxContacts) ||
            (valid == 1 && (out.num_contacts < 0 ||
                            static_cast<uint64_t>(out.num_contacts) != pair_count)) ||
            (valid == 0 && out.num_contacts < -1) ||
            (valid == 0 && out.num_contacts >= 0 &&
                            static_cast<uint64_t>(out.num_contacts) != pair_count))
            return false;
        out.frame_valid = valid == 1;
        out.pairs.reserve(static_cast<size_t>(pair_count));
        for (uint64_t i = 0; i < pair_count; ++i)
        {
            std::string a, b;
            if (!read_length_value(text, cursor, kMaxNameBytes, a) ||
                !read_length_value(text, cursor, kMaxNameBytes, b))
                return false;
            out.pairs.emplace_back(std::move(a), std::move(b));
            if (cursor >= text.size() || text[cursor++] != '|')
                return false;
        }
        std::string error;
        if (!read_length_value(text, cursor, kMaxErrorBytes, error) || cursor != text.size())
            return false;
        out.error = std::move(error);
        if (out.frame_valid != out.error.empty())
            return false;
        if (!out.frame_valid)
        {
            out.decoded = true;
            return true;
        }
        out.collision_pairs_allowed = true;
        for (const auto &pair : out.pairs)
        {
            const auto a = pair.first;
            const auto b = pair.second;
            const std::string *robot = nullptr;
            if (a == ground_name()) robot = &b;
            else if (b == ground_name()) robot = &a;
            else
            {
                out.collision_pairs_allowed = false;
                continue;
            }
            if (*robot == left_wheel_name()) out.wheel_mask |= 0x1;
            else if (*robot == right_wheel_name()) out.wheel_mask |= 0x2;
            else out.collision_pairs_allowed = false;
        }
        out.decoded = true;
        return true;
    }

    static std::string encode(uint64_t sequence, int64_t stamp,
        uint64_t iteration, int64_t dt, bool valid, int64_t num_contacts,
        const std::vector<std::pair<std::string, std::string>> &pairs,
        std::string_view error = {})
    {
        std::string s = "BBOT_GCF1|" + std::to_string(sequence) + "|" +
            std::to_string(stamp) + "|" + std::to_string(iteration) + "|" +
            std::to_string(dt) + "|" + (valid ? "1" : "0") + "|" +
            std::to_string(num_contacts) +
            "|" + std::to_string(pairs.size()) + "|";
        for (const auto &pair : pairs)
        {
            append_length_value(s, pair.first);
            append_length_value(s, pair.second);
            s.push_back('|');
        }
        append_length_value(s, error);
        return s;
    }

    static const std::string &ground_name()
    {
        static const std::string value = "flat_jump_world::ground_plane::link::collision";
        return value;
    }
    static const std::string &left_wheel_name()
    {
        static const std::string value = "flat_jump_world::bbot::link_004::link_004_collision_collision";
        return value;
    }
    static const std::string &right_wheel_name()
    {
        static const std::string value = "flat_jump_world::bbot::link_007::link_007_collision_collision";
        return value;
    }

  private:
    static bool next_token(std::string_view s, size_t &cursor, std::string_view &out)
    {
        const size_t end = s.find('|', cursor);
        if (end == std::string_view::npos) return false;
        out = s.substr(cursor, end - cursor);
        cursor = end + 1;
        return !out.empty();
    }
    static bool parse_unsigned(std::string_view s, uint64_t &out)
    {
        if (s.empty() || (s.size() > 1 && s.front() == '0')) return false;
        for (const char c : s) if (c < '0' || c > '9') return false;
        const auto result = std::from_chars(s.data(), s.data() + s.size(), out);
        return result.ec == std::errc{} && result.ptr == s.data() + s.size();
    }
    static bool parse_signed(std::string_view s, int64_t &out)
    {
        if (s.empty() || s.front() == '+') return false;
        const size_t start = s.front() == '-' ? 1 : 0;
        if (start == s.size() || (s.size() - start > 1 && s[start] == '0')) return false;
        for (size_t i = start; i < s.size(); ++i) if (s[i] < '0' || s[i] > '9') return false;
        const auto result = std::from_chars(s.data(), s.data() + s.size(), out);
        return result.ec == std::errc{} && result.ptr == s.data() + s.size();
    }
    static bool read_length_value(std::string_view s, size_t &cursor, size_t max,
                                  std::string &out)
    {
        const size_t colon = s.find(':', cursor);
        if (colon == std::string_view::npos || colon == cursor) return false;
        uint64_t length = 0;
        if (!parse_unsigned(s.substr(cursor, colon - cursor), length) || length > max)
            return false;
        cursor = colon + 1;
        if (length > s.size() - cursor) return false;
        out.assign(s.data() + cursor, static_cast<size_t>(length));
        cursor += static_cast<size_t>(length);
        return true;
    }
    static void append_length_value(std::string &s, std::string_view value)
    {
        s += std::to_string(value.size());
        s.push_back(':');
        s.append(value.data(), value.size());
    }
};

struct CompleteTakeoffComEvidence
{
    bool valid{false};
    int64_t stamp_ns{0};
    double vertical_velocity{0.0};
};

class CompleteContactTakeoffObserver
{
  public:
    static constexpr int64_t kStepNs = 1'000'000;
    static constexpr int64_t kFreshNs = 20'000'000;
    static constexpr int64_t kComFreshNs = 40'000'000;
    static constexpr int64_t kRecentContactNs = 80'000'000;
    static constexpr int kRequiredZeroFrames = 11;

    void reset_for_jump(uint64_t jump_id, int64_t accepted_stamp_ns)
    {
        jump_id_ = jump_id;
        accepted_stamp_ns_ = accepted_stamp_ns;
        gate_open_ = false;
        gate_stamp_ns_ = 0;
        reset_evidence();
        source_taken_ = false;
        source_take_stamp_ns_ = 0;
        source_take_com_stamp_ns_ = 0;
    }

    void open_gate(int64_t gate_stamp_ns)
    {
        gate_open_ = gate_stamp_ns > 0;
        gate_stamp_ns_ = gate_open_ ? gate_stamp_ns : 0;
        reset_evidence();
    }

    // Called only in raw subscriber receive order. Malformed transport is passed as
    // decoded=false and always breaks the current evidence chain.
    void receive(const CompleteGroundContactFrame &frame)
    {
        latest_decoded_ = frame.decoded;
        latest_semantically_valid_ = frame.decoded && frame.frame_valid &&
                                     frame.collision_pairs_allowed;
        latest_stamp_ns_ = frame.decoded ? frame.sim_time_ns : 0;
        latest_sequence_ = frame.decoded ? frame.sequence : 0;
        if (!frame.decoded || !frame.frame_valid || !frame.collision_pairs_allowed)
        {
            reset_evidence();
            if (frame.decoded) seed_continuity(frame);
            return;
        }

        const bool contiguous = !have_previous_ ||
            (frame.sequence == previous_sequence_ + 1 &&
             frame.iteration == previous_iteration_ + 1 &&
             frame.sim_time_ns - previous_stamp_ns_ == kStepNs &&
             frame.dt_ns == kStepNs);
        if (!contiguous)
        {
            reset_evidence();
            seed_continuity(frame);
            return;
        }
        seed_continuity(frame);
        if (!gate_open_ || frame.sim_time_ns < gate_stamp_ns_ ||
            frame.sim_time_ns < accepted_stamp_ns_)
        {
            reset_evidence();
            return;
        }
        if (frame.wheel_mask != 0)
        {
            zero_frames_ = 0;
            zero_start_ns_ = 0;
            latest_zero_span_ns_ = 0;
            if (frame.wheel_mask == 0x3)
            {
                record_positive(frame);
                if (consecutive_bilateral_ >= 2) bilateral_support_ready_ = true;
            }
            else
            {
                consecutive_bilateral_ = 0;
            }
            last_any_contact_ns_ = frame.sim_time_ns;
            return;
        }
        if (!bilateral_support_ready_ || last_bilateral_ns_ <= 0 ||
            frame.sim_time_ns - last_bilateral_ns_ > kRecentContactNs ||
            last_any_contact_ns_ <= 0 ||
            (zero_frames_ == 0 && frame.sim_time_ns - last_any_contact_ns_ != kStepNs))
        {
            zero_frames_ = 0;
            zero_start_ns_ = 0;
            latest_zero_span_ns_ = 0;
            return;
        }
        if (zero_frames_ == 0) zero_start_ns_ = frame.sim_time_ns;
        ++zero_frames_;
        latest_zero_span_ns_ = frame.sim_time_ns - zero_start_ns_;
    }

    bool confirmed(int64_t now_ns, int64_t gate_stamp_ns,
                   bool thrust_state, bool effort_active, bool switch_pending,
                   bool motion_gate_open, bool release_active_or_lift_observed,
                   const CompleteTakeoffComEvidence &com) const
    {
        if (source_taken_ || !gate_open_ || !thrust_state || !effort_active ||
            switch_pending || !motion_gate_open || !release_active_or_lift_observed ||
            gate_stamp_ns != gate_stamp_ns_ || !latest_semantically_valid_ || !have_previous_)
            return false;
        if (now_ns < latest_stamp_ns_ - kStepNs || now_ns - latest_stamp_ns_ > kFreshNs)
            return false;
        if (zero_frames_ < kRequiredZeroFrames || latest_zero_span_ns_ < 10 * kStepNs ||
            zero_start_ns_ < gate_stamp_ns_ || last_bilateral_ns_ < gate_stamp_ns_ ||
            now_ns - last_bilateral_ns_ > kRecentContactNs)
            return false;
        if (!com.valid || !std::isfinite(com.vertical_velocity) ||
            com.vertical_velocity <= 0.35 || com.stamp_ns < gate_stamp_ns_ ||
            com.stamp_ns < zero_start_ns_ - kComFreshNs ||
            com.stamp_ns > now_ns + kStepNs || now_ns - com.stamp_ns > kComFreshNs)
            return false;
        return true;
    }

    void mark_taken(int64_t frame_stamp_ns, int64_t com_stamp_ns)
    {
        source_taken_ = true;
        source_take_stamp_ns_ = frame_stamp_ns;
        source_take_com_stamp_ns_ = com_stamp_ns;
    }
    bool source_valid(int64_t now_ns) const
    {
        return latest_semantically_valid_ && have_previous_ &&
            now_ns >= latest_stamp_ns_ - kStepNs && now_ns - latest_stamp_ns_ <= kFreshNs;
    }
    bool source_fresh(int64_t now_ns) const { return source_valid(now_ns); }
    bool refresh_source(int64_t now_ns)
    {
        if (!latest_semantically_valid_ || !have_previous_)
            return false;
        if (now_ns < latest_stamp_ns_ - kStepNs ||
            now_ns - latest_stamp_ns_ > kFreshNs)
        {
            latest_semantically_valid_ = false;
            have_previous_ = false;
            reset_evidence();
            return false;
        }
        return true;
    }
    uint64_t latest_sequence() const { return latest_sequence_; }
    int64_t latest_stamp_ns() const { return latest_stamp_ns_; }
    int64_t zero_span_ns() const { return latest_zero_span_ns_; }
    int zero_frame_count() const { return zero_frames_; }
    int64_t zero_start_ns() const { return zero_start_ns_; }
    bool source_taken() const { return source_taken_; }
    int64_t source_take_stamp_ns() const { return source_take_stamp_ns_; }
    int64_t source_take_com_stamp_ns() const { return source_take_com_stamp_ns_; }
    uint64_t jump_id() const { return jump_id_; }
    int64_t gate_stamp_ns() const { return gate_stamp_ns_; }

  private:
    void reset_evidence()
    {
        consecutive_bilateral_ = 0;
        bilateral_support_ready_ = false;
        last_bilateral_ns_ = 0;
        last_any_contact_ns_ = 0;
        zero_frames_ = 0;
        zero_start_ns_ = 0;
        latest_zero_span_ns_ = 0;
    }
    void seed_continuity(const CompleteGroundContactFrame &frame)
    {
        previous_sequence_ = frame.sequence;
        previous_iteration_ = frame.iteration;
        previous_stamp_ns_ = frame.sim_time_ns;
        latest_sequence_ = frame.sequence;
        latest_stamp_ns_ = frame.sim_time_ns;
        have_previous_ = true;
    }
    void record_positive(const CompleteGroundContactFrame &frame)
    {
        ++consecutive_bilateral_;
        last_bilateral_ns_ = frame.sim_time_ns;
        last_any_contact_ns_ = frame.sim_time_ns;
    }

    uint64_t jump_id_{0};
    int64_t accepted_stamp_ns_{0};
    bool gate_open_{false};
    int64_t gate_stamp_ns_{0};
    bool have_previous_{false};
    uint64_t previous_sequence_{0};
    uint64_t previous_iteration_{0};
    int64_t previous_stamp_ns_{0};
    bool latest_decoded_{false};
    bool latest_semantically_valid_{false};
    uint64_t latest_sequence_{0};
    int64_t latest_stamp_ns_{0};
    int consecutive_bilateral_{0};
    bool bilateral_support_ready_{false};
    int64_t last_bilateral_ns_{0};
    int64_t last_any_contact_ns_{0};
    int zero_frames_{0};
    int64_t zero_start_ns_{0};
    int64_t latest_zero_span_ns_{0};
    bool source_taken_{false};
    int64_t source_take_stamp_ns_{0};
    int64_t source_take_com_stamp_ns_{0};
};
}  // namespace bbot_jump

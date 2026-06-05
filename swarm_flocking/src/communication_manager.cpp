// Copyright (c) 2024 Swarm Robotics Lab. BSD-3-Clause License.
// communication_manager.cpp

#include "swarm_flocking/communication_manager.hpp"
#include <algorithm>

namespace swarm_flocking {

CommunicationManager::CommunicationManager(uint16_t own_id, const CommConfig& cfg)
    : own_id_(own_id), config_(cfg)
{
    own_state_.id = own_id;
}

void CommunicationManager::update_own_state(const AgentState& state) {
    std::lock_guard<std::mutex> lock(mutex_);
    own_state_ = state;
    own_state_.id = own_id_;
    own_state_.last_received = Clock::now();
}

void CommunicationManager::receive_neighbor_state(const AgentState& state) {
    if (state.id == own_id_) return;  // ignore own echoes

    std::lock_guard<std::mutex> lock(mutex_);

    // Range check: drop if beyond communication range
    double dist = (state.pos - own_state_.pos).norm();
    if (dist > config_.max_comm_range) return;

    AgentState updated = state;
    updated.last_received = Clock::now();
    updated.comm_healthy = true;
    neighbors_[state.id] = updated;
}

AgentState CommunicationManager::own_state() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return own_state_;
}

AgentState CommunicationManager::extrapolate(const AgentState& state) const {
    if (!config_.enable_extrapolation) return state;

    double age = state.age_seconds();
    if (age < 0.05) return state;  // fresh enough

    double extrap_time = std::min(age, config_.extrapolation_max);

    AgentState extrapolated = state;
    // Linear position extrapolation using last known velocity
    extrapolated.pos.x += state.vel.x * extrap_time;
    extrapolated.pos.y += state.vel.y * extrap_time;

    return extrapolated;
}

std::map<uint16_t, AgentState> CommunicationManager::get_active_neighbors() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::map<uint16_t, AgentState> active;

    for (const auto& [id, state] : neighbors_) {
        if (state.age_seconds() < config_.stale_timeout_s) {
            active[id] = extrapolate(state);
        }
    }
    return active;
}

std::map<uint16_t, AgentState> CommunicationManager::get_all_neighbors() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return neighbors_;
}

bool CommunicationManager::is_neighbor_alive(uint16_t id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = neighbors_.find(id);
    if (it == neighbors_.end()) return false;
    return it->second.age_seconds() < config_.stale_timeout_s;
}

size_t CommunicationManager::active_neighbor_count() const {
    std::lock_guard<std::mutex> lock(mutex_);
    size_t count = 0;
    for (const auto& [id, state] : neighbors_) {
        if (state.age_seconds() < config_.stale_timeout_s) count++;
    }
    return count;
}

void CommunicationManager::prune_stale() {
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto it = neighbors_.begin(); it != neighbors_.end(); ) {
        if (it->second.age_seconds() > config_.stale_timeout_s * 3.0) {
            it = neighbors_.erase(it);  // remove very old entries
        } else {
            ++it;
        }
    }
}

CommunicationManager::CommHealth CommunicationManager::get_health() const {
    std::lock_guard<std::mutex> lock(mutex_);
    CommHealth h;
    double sum_age = 0.0;

    for (const auto& [id, state] : neighbors_) {
        double age = state.age_seconds();
        sum_age += age;
        h.max_age_s = std::max(h.max_age_s, age);

        if (age < config_.stale_timeout_s) {
            h.active_count++;
            // Check if close AND stale (critical situation)
            double dist = (state.pos - own_state_.pos).norm();
            if (dist < 20.0 && age > 1.5) {
                h.any_critical = true;
            }
        } else {
            h.stale_count++;
        }
    }

    if (!neighbors_.empty()) {
        h.avg_age_s = sum_age / neighbors_.size();
    }
    return h;
}

}  // namespace swarm_flocking

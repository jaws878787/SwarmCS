#include "VehicleLink.h"
#include "ParameterMetadata.h"
#include <cmath>
#include <cstring>
#include <algorithm>

namespace {
bool supported(quint8 type) {
    return (type >= MAV_PARAM_TYPE_UINT8 && type <= MAV_PARAM_TYPE_INT32) || type == MAV_PARAM_TYPE_REAL32;
}
double decode(float wire, quint8 type, bool bytewise) {
    if (!bytewise || type == MAV_PARAM_TYPE_REAL32) return wire;
    mavlink_param_union_t value{};
    value.param_float = wire;
    switch (type) {
    case MAV_PARAM_TYPE_UINT8: return value.param_uint8;
    case MAV_PARAM_TYPE_INT8: return value.param_int8;
    case MAV_PARAM_TYPE_UINT16: return value.param_uint16;
    case MAV_PARAM_TYPE_INT16: return value.param_int16;
    case MAV_PARAM_TYPE_UINT32: return value.param_uint32;
    case MAV_PARAM_TYPE_INT32: return value.param_int32;
    default: return std::numeric_limits<double>::quiet_NaN();
    }
}
bool encode(double input, quint8 type, bool bytewise, float &wire, double &actual) {
    if (!supported(type) || !std::isfinite(input)) return false;
    static const double low[] = {0,0,-128,0,-32768,0,-2147483648.0};
    static const double high[] = {0,255,127,65535,32767,4294967295.0,2147483647.0};
    if (type != MAV_PARAM_TYPE_REAL32 && (std::trunc(input) != input || input < low[type] || input > high[type])) return false;
    mavlink_param_union_t value{};
    if (bytewise && type != MAV_PARAM_TYPE_REAL32) {
        switch (type) {
        case MAV_PARAM_TYPE_UINT8: value.param_uint8 = quint8(input); break;
        case MAV_PARAM_TYPE_INT8: value.param_int8 = qint8(input); break;
        case MAV_PARAM_TYPE_UINT16: value.param_uint16 = quint16(input); break;
        case MAV_PARAM_TYPE_INT16: value.param_int16 = qint16(input); break;
        case MAV_PARAM_TYPE_UINT32: value.param_uint32 = quint32(input); break;
        case MAV_PARAM_TYPE_INT32: value.param_int32 = qint32(input); break;
        }
        wire = value.param_float;
    } else {
        wire = float(input);
        if (!std::isfinite(wire)) return false;
        // C-cast integers must be exactly representable in the float transport.
        if (type != MAV_PARAM_TYPE_REAL32 && double(wire) != input) return false;
    }
    actual = decode(wire,type,bytewise);
    return true;
}
bool sameValue(double left, double right, quint8 type) {
    return type == MAV_PARAM_TYPE_REAL32 ? float(left) == float(right) : left == right;
}
}

bool VehicleLink::parametersReady(int systemId) const {
    const int row = rowForSystem(systemId);
    if (row < 0) return false;
    const auto &peer = m_peers[row];
    return peer.parameterCount > 0 && peer.parameterIndexes.size() == peer.parameterCount
        && peer.parameters.size() == peer.parameterCount && !peer.parameterLoading;
}

QString VehicleLink::parameterStatus(int systemId) const {
    const int row = rowForSystem(systemId);
    if (row < 0) return "Vehicle disconnected.";
    const auto &peer = m_peers[row];
    if (!peer.parameterError.isEmpty()) return peer.parameterError;
    if (parametersReady(systemId)) return QString("%1 parameters loaded").arg(peer.parameterCount);
    return QString("Loading parameters: %1 / %2").arg(peer.parameterIndexes.size())
        .arg(peer.parameterCount < 0 ? "?" : QString::number(peer.parameterCount));
}

QVariantList VehicleLink::parameters(int systemId, const QString &search) const {
    const int row = rowForSystem(systemId);
    if (row < 0) return {};
    QVariantList result;
    const auto &peer = m_peers[row];
    for (auto it = peer.parameters.cbegin(); it != peer.parameters.cend(); ++it) {
        if (!it.key().contains(search.trimmed(),Qt::CaseInsensitive)) continue;
        auto item = parameterMetadata(peer.autopilot == MAV_AUTOPILOT_PX4 ? "px4" : isPlane(peer) ? "plane" : "copter",it.key());
        item.insert("name",it.key());
        item.insert("value",QString::number(it->value,'g',10));
        item.insert("type",it->type);
        item.insert("editable",supported(it->type) && std::isfinite(it->value));
        result.append(item);
    }
    return result;
}

void VehicleLink::requestParameterList(Peer &peer) {
    mavlink_message_t message{};
    mavlink_status_t status{};
    status.current_tx_seq = m_sequence++;
    mavlink_msg_param_request_list_pack_status(255,MAV_COMP_ID_MISSIONPLANNER,&status,&message,peer.systemId,peer.componentId);
    send(message,peer);
    peer.parameterRetryAt = m_clock.elapsed() + 2000;
}

void VehicleLink::loadParameters(int systemId, bool reload) {
    const int row = rowForSystem(systemId);
    if (row < 0 || (reload && parameterSaveBusy())) return;
    auto &peer = m_peers[row];
    if (!reload && (parametersReady(systemId) || peer.parameterLoading)) return;
    if (reload) {
        peer.parameters.clear();
        peer.parameterIndexes.clear();
        peer.parameterCount = -1;
    }
    peer.parameterError.clear();
    peer.parameterLoading = true;
    peer.parameterDeadline = m_clock.elapsed() + 90000;
    peer.parameterRetryIndex = 0;
    requestParameterList(peer);
    m_parametersDirty = true;
}

void VehicleLink::receiveParameter(Peer &peer, const mavlink_message_t &message) {
    mavlink_param_value_t value{};
    mavlink_msg_param_value_decode(&message,&value);
    const auto nameBytes = QByteArray(value.param_id,sizeof(value.param_id)).split('\0').first();
    const auto name = QString::fromLatin1(nameBytes);
    // PX4's synthetic hash entry is outside the advertised table.
    if (name.isEmpty() || name == "_HASH_CHECK" || !value.param_count) return;
    if (peer.parameterCount != value.param_count) {
        const bool changed = peer.parameterCount >= 0;
        peer.parameterCount = value.param_count;
        peer.parameterIndexes.clear();
        peer.parameters.clear();
        if (changed) {
            peer.parameterLoading = true;
            peer.parameterDeadline = m_clock.elapsed() + 90000;
            peer.parameterRetryAt = m_clock.elapsed() + 2000;
        }
    }
    const double decoded = supported(value.param_type) ? decode(value.param_value,value.param_type,peer.parameterBytewise)
        : std::numeric_limits<double>::quiet_NaN();
    peer.parameters[name] = {decoded,value.param_type,value.param_index};
    if (value.param_index < value.param_count) peer.parameterIndexes.insert(value.param_index);
    if (peer.parameterIndexes.size() == peer.parameterCount && peer.parameters.size() == peer.parameterCount) {
        peer.parameterLoading = false;
        peer.parameterError.clear();
    }
    m_parametersDirty = true;
    if (m_parameterSave.active && m_parameterSave.prepared && !m_parameterSave.queue.isEmpty()) {
        const auto item = m_parameterSave.queue.first();
        if (item.systemId == peer.systemId && peer.session == m_parameterSave.sessions[peer.systemId]
            && item.name == name && item.type == value.param_type
            && m_parameterSave.attempts > 0 && sameValue(decoded,item.value,item.type)) {
            ++m_parameterSave.saved;
            ++m_parameterSave.confirmations[name];
            m_parameterSave.queue.removeFirst();
            m_parameterSave.attempts = 0;
            // Continue from the timer to avoid sending inside the receive loop.
        }
    }
}

bool VehicleLink::saveParameters(int systemId, const QString &group, bool toGroup, const QVariantMap &changes) {
    if (parameterSaveBusy() || !parametersReady(systemId) || changes.isEmpty()
        || (toGroup && groupForSystem(systemId) != group)) return false;
    ParameterSave job;
    job.active = true;
    job.source = systemId;
    job.preparationDeadline = m_clock.elapsed() + 120000;
    job.changes = changes;
    const auto targets = toGroup ? groupVehicles(group) : QVariantList{QVariantMap{{"systemId",systemId}}};
    if (targets.isEmpty()) return false;
    const auto &source = m_peers[rowForSystem(systemId)];
    // Validate the whole source edit set before sending any writes.
    for (auto it = changes.cbegin(); it != changes.cend(); ++it) {
        bool ok = false;
        const double requested = it.value().toString().toDouble(&ok);
        float wire; double actual;
        if (!ok || !source.parameters.contains(it.key())
            || !encode(requested,source.parameters.value(it.key()).type,source.parameterBytewise,wire,actual)) {
            emit operationReport("Parameters", "Invalid or unsupported value for " + it.key() + ". Nothing was saved.");
            return false;
        }
    }
    for (const auto &target : targets) {
        const int id = target.toMap().value("systemId").toInt();
        job.targets.append(id);
        job.names[id] = nameForSystem(id);
        job.sessions[id] = m_peers[rowForSystem(id)].session;
    }
    m_parameterSave = job;
    for (int id : job.targets) loadParameters(id);
    emit parametersChanged();
    return true;
}

void VehicleLink::advanceParameterSave() {
    auto &job = m_parameterSave;
    if (!job.active) return;
    if (!job.prepared) {
        for (int id : job.targets) {
            const int row = rowForSystem(id);
            if (row >= 0 && m_peers[row].session == job.sessions[id] && m_peers[row].parameterLoading
                && m_clock.elapsed() < job.preparationDeadline) return;
        }
        for (int id : job.targets) {
            const int row = rowForSystem(id);
            if (row < 0 || m_peers[row].session != job.sessions[id] || !parametersReady(id)) {
                job.issues.append(job.names[id] + ": skipped — parameter list unavailable or vehicle disconnected.");
                continue;
            }
            const auto &peer = m_peers[row];
            for (auto it = job.changes.cbegin(); it != job.changes.cend(); ++it) {
                if (!peer.parameters.contains(it.key())) {
                    job.issues.append(job.names[id] + ": skipped " + it.key() + " (parameter unavailable).");
                    continue;
                }
                const auto parameter = peer.parameters.value(it.key());
                float wire; double actual;
                if (!encode(it.value().toString().toDouble(),parameter.type,peer.parameterBytewise,wire,actual)) {
                    job.issues.append(job.names[id] + ": skipped " + it.key() + " (value incompatible with parameter type).");
                    continue;
                }
                // Even unchanged values are written and echoed, verifying the current value on every target.
                job.queue.append({id,it.key(),actual,parameter.type});
            }
        }
        job.prepared = true;
    }
    if (job.queue.isEmpty()) { finishParameterSave(); return; }
    if (job.attempts && m_clock.elapsed() < job.retryAt) return;
    const auto item = job.queue.first();
    const int row = rowForSystem(item.systemId);
    if (row < 0 || m_peers[row].session != job.sessions[item.systemId] || job.attempts >= 3
        || !parametersReady(item.systemId) || !m_peers[row].parameters.contains(item.name)
        || m_peers[row].parameters.value(item.name).type != item.type) {
        job.issues.append(job.names[item.systemId] + ": failed " + item.name + " (disconnected, rejected, or no matching confirmation).");
        job.queue.removeFirst();
        job.attempts = 0;
        return;
    }
    sendParameterWrite();
}

void VehicleLink::sendParameterWrite() {
    auto &job = m_parameterSave;
    const auto item = job.queue.first();
    const auto &peer = m_peers[rowForSystem(item.systemId)];
    float wire; double actual;
    if (!encode(item.value,item.type,peer.parameterBytewise,wire,actual)) return;
    char id[16]{};
    const auto bytes = item.name.toLatin1();
    std::memcpy(id,bytes.constData(),std::min(16,static_cast<int>(bytes.size())));
    mavlink_message_t message{};
    mavlink_status_t status{};
    status.current_tx_seq = m_sequence++;
    mavlink_msg_param_set_pack_status(255,MAV_COMP_ID_MISSIONPLANNER,&status,&message,
        peer.systemId,peer.componentId,id,wire,item.type);
    send(message,peer);
    ++job.attempts;
    job.retryAt = m_clock.elapsed() + 1500;
}

void VehicleLink::finishParameterSave() {
    const auto job = m_parameterSave;
    m_parameterSave = {};
    QStringList confirmed;
    for (auto it = job.confirmations.cbegin(); it != job.confirmations.cend(); ++it)
        if (it.value() == job.targets.size()) confirmed.append(it.key());
    emit parametersSaved(job.source,confirmed);
    emit parametersChanged();
    QString message = QString("%1 parameter value(s) confirmed saved across %2 vehicle(s).").arg(job.saved).arg(job.targets.size());
    if (job.issues.isEmpty()) message += "\nAll requested values were confirmed. No vehicles were skipped.";
    else message += "\n\nSkipped vehicles / parameters and failures:\n" + job.issues.join('\n');
    emit operationReport("Parameter save complete",message);
}

void VehicleLink::parameterTick() {
    for (auto &peer : m_peers) {
        if (!peer.parameterLoading) continue;
        const auto now = m_clock.elapsed();
        if (now >= peer.parameterDeadline) {
            peer.parameterLoading = false;
            peer.parameterError = "Parameter download timed out. Reload to try again.";
            m_parametersDirty = true;
        } else if (now >= peer.parameterRetryAt) {
            if (peer.parameterCount < 0) requestParameterList(peer);
            else {
                int requested = 0;
                // Rotate through missing indexes, avoiding a flood of full-table retries.
                for (int i = 0; i < peer.parameterCount && requested < 20; ++i) {
                    const int index = peer.parameterRetryIndex++ % peer.parameterCount;
                    if (peer.parameterIndexes.contains(index) || index > INT16_MAX) continue;
                    mavlink_message_t message{};
                    mavlink_status_t status{};
                    status.current_tx_seq = m_sequence++;
                    char id[16]{};
                    mavlink_msg_param_request_read_pack_status(255,MAV_COMP_ID_MISSIONPLANNER,&status,&message,
                        peer.systemId,peer.componentId,id,index);
                    send(message,peer);
                    ++requested;
                }
                peer.parameterRetryAt = now + 1000;
            }
        }
    }
    advanceParameterSave();
    if (m_parametersDirty) { m_parametersDirty = false; emit parametersChanged(); }
}

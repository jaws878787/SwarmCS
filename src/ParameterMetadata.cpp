#include "ParameterMetadata.h"
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QMap>

QVariantMap parameterMetadata(const QString &firmware, const QString &name) {
    static QMap<QString,QJsonObject> cache;
    if (!cache.contains(firmware)) {
        QFile file(":/parameters/" + firmware + ".json");
        cache[firmware] = file.open(QIODevice::ReadOnly)
            ? QJsonDocument::fromJson(file.readAll()).object().value("parameters").toObject() : QJsonObject{};
    }
    const auto entry = cache[firmware].value(name).toObject();
    QStringList ranges, choices;
    for (const auto &range : entry.value("ranges").toArray()) ranges.append(range.toString());
    for (const auto &choice : entry.value("choices").toArray()) choices.append(choice.toString());
    const auto summary = entry.value("summary").toString();
    auto description = entry.value("description").toString();
    if (description.isEmpty()) description = summary;
    if (description.isEmpty()) description = "No description available in the firmware reference.";
    const auto units = entry.value("units").toString();
    const auto range = ranges.isEmpty() ? (choices.isEmpty() ? QString("Not specified")
        : choices.first().startsWith("Bitmask:") ? QString("Bitmask") : QString("Listed values"))
        : ranges.join(" / ") + (ranges.size() > 1 ? " (variants)" : "");
    QString help = name + "\n" + summary + "\n\n" + description
        + "\n\nUnits: " + (units.isEmpty() ? "Not specified" : units) + "\nReference range: " + range;
    if (!choices.isEmpty()) help += "\n\n" + choices.join('\n');
    return {{"summary",summary},{"description",description},{"units",units.isEmpty() ? "—" : units},
        {"range",range},{"help",help}};
}

#pragma once

#include <QString>
#include <QMutex>
#include <QQueue>
#include <QColor>

struct LogEntry {
    QString text;
    QColor  color;
};

class LogQueue
{
public:
    static LogQueue& instance() {
        static LogQueue q;
        return q;
    }

    void push(const QString& text, const QColor& color = Qt::white) {
        QMutexLocker locker(&mutex_);
        queue_.append({text, color});
    }

    // Called from GUI thread only
    QList<LogEntry> drain() {
        QMutexLocker locker(&mutex_);
        QList<LogEntry> result;
        result.swap(queue_);          // O(1) swap
        return result;
    }

    bool isEmpty() const {
        QMutexLocker locker(&mutex_);
        return queue_.isEmpty();
    }

private:
    LogQueue() = default;
    mutable QMutex mutex_;
    QList<LogEntry> queue_;
};
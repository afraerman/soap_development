#pragma once

#ifdef SOAP_WITH_QT

#include <iostream>
#include <streambuf>
#include <QObject>
#include <QColor>
#include <QRegularExpression>

class QtStreamRedirector : public QObject, public std::streambuf {
    Q_OBJECT
public:
    explicit QtStreamRedirector(std::ostream& stream, QColor defaultColor, QObject* parent = nullptr)
        : QObject(parent), m_stream(stream), m_defaultColor(defaultColor), m_currentColor(defaultColor) {
        m_old_buf = m_stream.rdbuf(this);
    }

    ~QtStreamRedirector() override {
        // Restore the original buffer upon destruction
        m_stream.rdbuf(m_old_buf);
    }

signals:
    // Signal emitted whenever new text is captured
    void textReceived(const QString& text, const QColor& color);

protected:
    // Handles single-character insertions
    int_type overflow(int_type v) override {
        if (v == traits_type::eof()) {
            return traits_type::not_eof(v);
        }
        char c = traits_type::to_char_type(v);
        //processAndEmit(QString(c));
        //return v;
        m_accumulator.append(c);

        // Если пришел символ перевода строки или завершение ANSI-последовательности, 
        // сразу отправляем накопленное, чтобы не ждать sync()
        if (c == '\n' || c == 'm') {
            flushAccumulator();
        }
        
        return v;
    }

    // Overridden for performance optimization with larger string blocks
    std::streamsize xsputn(const char* s, std::streamsize n) override {
        //processAndEmit(QString::fromLocal8Bit(s, static_cast<int>(n)));
        //return n;
        m_accumulator.append(QString::fromLocal8Bit(s, static_cast<int>(n)));
        flushAccumulator();
        return n;
    }

private:

    void flushAccumulator() {
        if (m_accumulator.isEmpty()) return;

        // Если в аккумуляторе застрял незавершенный ANSI-код (например, "\033[3"),
        // ждем следующих символов, чтобы не ломать регулярное выражение
        if (m_accumulator.contains('\x1B') && !m_accumulator.contains('m')) {
            return;
        }

        static const QRegularExpression ansiRegex("\x1B\\[([0-9;]*)m");
        int lastPos = 0;
        auto it = ansiRegex.globalMatch(m_accumulator);
        
        while (it.hasNext()) {
            QRegularExpressionMatch match = it.next();
            int matchPos = match.capturedStart();
            
            if (matchPos > lastPos) {
                QString textBlock = m_accumulator.mid(lastPos, matchPos - lastPos);
                emit textReceived(textBlock, m_currentColor);
            }
            
            QString code = match.captured(1);
            updateCurrentColor(code);
            
            lastPos = match.capturedEnd();
        }
        
        if (lastPos < m_accumulator.length()) {
            QString remainingText = m_accumulator.mid(lastPos);
            emit textReceived(remainingText, m_currentColor);
        }

        m_accumulator.clear();
    }

    void processAndEmit(const QString& input)
    {
        // Регулярное выражение ищет любые ANSI escape-последовательности
        static const QRegularExpression ansiRegex("\x1B\\[([0-9;]*)m");
        
        int lastPos = 0;
        auto it = ansiRegex.globalMatch(input);
        
        while (it.hasNext()) {
            QRegularExpressionMatch match = it.next();
            int matchPos = match.capturedStart();
            
            // 1. Если перед кодом цвета был текст, отправляем его с ТЕКУЩИМ активным цветом
            if (matchPos > lastPos) {
                QString textBlock = input.mid(lastPos, matchPos - lastPos);
                emit textReceived(textBlock, m_currentColor);
            }
            
            // 2. Парсим сам ANSI-код для изменения цвета
            QString code = match.captured(1);
            updateCurrentColor(code);
            
            lastPos = match.capturedEnd();
        }
        
        // 3. Отправляем оставшийся текст после последней найденной управляющей команды
        if (lastPos < input.length()) {
            QString remainingText = input.mid(lastPos);
            emit textReceived(remainingText, m_currentColor);
        }
    }

    void updateCurrentColor(const QString& code) {
        if (code == "0" || code.isEmpty()) {
            m_currentColor = m_defaultColor; // Сброс цвета (\033[0m)
        } else if (code == "31") {
            m_currentColor = Qt::red;        // \033[31m
        } else if (code == "32") {
            m_currentColor = Qt::green;      // \033[32m
        } else if (code == "33") {
            m_currentColor = QColor(255, 191, 0); // Красивый темно-желтый/amber (\033[33m)
        } else if (code == "34") {
            m_currentColor = Qt::blue;       // \033[34m
        }
        // При желании сюда легко добавить жирный шрифт (code == "1") или другие цвета
    }

    std::ostream& m_stream;
    std::streambuf* m_old_buf;
    QColor m_defaultColor;
    QColor m_currentColor;
    QString m_accumulator;
};

#endif
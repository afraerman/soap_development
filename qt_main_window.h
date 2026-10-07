#pragma once

#ifdef SOAP_WITH_QT

#include<QMainWindow>
#include<QLineEdit>
#include<QPushButton>
#include<streambuf>
#include<QTextEdit>
#include<QLabel>
#include<QMessageBox>
#include<QFileDialog>
#include<QVBoxLayout>
#include<QHBoxLayout>
#include<QFutureWatcher>
#include<QTimer>
#include<queue>

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit MainWindow(QWidget *parent = nullptr);
    bool multiple_satellites;

private slots:
    void onBrowseClicked();
    void onRunClicked();
    void onMultSatRunClicked();
    void onCloseClicked();

    // Custom slots to handle text insertion with color
    void appendColoredText(const QString& text, const QColor& color);

    void onIntegrationFinished();

    void onLogTimer();

private:
    void setupUi();
    void setUiRunning(bool running);

    QLineEdit*  fileEdit_   = nullptr;
    QPushButton* browseBtn_ = nullptr;
    QPushButton* runBtn_    = nullptr;
    QPushButton* multSatRunBtn_ = nullptr;
    QLabel*     statusLabel_= nullptr;
    QTextEdit*  resultEdit_ = nullptr;
    QPushButton* closeBtn_  = nullptr;
    QTimer* logTimer_       = nullptr;

    QFutureWatcher<void> futureWatcher_;

    std::queue<std::string> filenames_queue;
};

#endif
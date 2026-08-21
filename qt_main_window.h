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

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit MainWindow(QWidget *parent = nullptr);

private slots:
    void onBrowseClicked();
    void onRunClicked();
    void onCloseClicked();

    // Custom slots to handle text insertion with color
    void appendColoredText(const QString& text, const QColor& color);

private:
    void setupUi();

    QLineEdit*  fileEdit_   = nullptr;
    QPushButton* browseBtn_ = nullptr;
    QPushButton* runBtn_    = nullptr;
    QLabel*     statusLabel_= nullptr;
    QTextEdit*  resultEdit_ = nullptr;
    QPushButton* closeBtn_  = nullptr;
};

#endif
#include "stdafx.h"

#ifdef SOAP_WITH_QT

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    setvbuf(stdout, nullptr, _IONBF, 0);
    setvbuf(stderr, nullptr, _IONBF, 0);

    std::ios_base::sync_with_stdio(false);
    std::cout << std::unitbuf;
    std::cerr << std::unitbuf;

    setupUi();
    setWindowTitle("C++ Calculator + Qt UI");
    resize(600, 450);
}

void MainWindow::setupUi()
{
    auto* central = new QWidget(this);
    setCentralWidget(central);

    auto* mainLayout = new QVBoxLayout(central);

    // File selection row
    auto* fileLayout = new QHBoxLayout;
    fileEdit_ = new QLineEdit;
    fileEdit_->setPlaceholderText("Select a text file...");
    browseBtn_ = new QPushButton("Browse...");
    fileLayout->addWidget(fileEdit_);
    fileLayout->addWidget(browseBtn_);
    mainLayout->addLayout(fileLayout);

    // Run button
    runBtn_ = new QPushButton("Run Calculation");
    mainLayout->addWidget(runBtn_);

    // Status
    statusLabel_ = new QLabel("Ready");
    mainLayout->addWidget(statusLabel_);

    // Results
    resultEdit_ = new QTextEdit;
    resultEdit_->setReadOnly(true);
    resultEdit_->setStyleSheet("QTextEdit { background-color: #000000; color: #ffffff; }");
    mainLayout->addWidget(resultEdit_);

    // Close button
    closeBtn_ = new QPushButton("EXIT SOAP");
    mainLayout->addWidget(closeBtn_);

    // 1. Create redirectors for both cout and cerr
    auto* coutRedirector = new QtStreamRedirector(std::cout, Qt::white, this);
    auto* cerrRedirector = new QtStreamRedirector(std::cerr, Qt::red, this);

    connect(coutRedirector, &QtStreamRedirector::textReceived, 
            this, &MainWindow::appendColoredText, Qt::QueuedConnection);
            
    connect(cerrRedirector, &QtStreamRedirector::textReceived, 
            this, &MainWindow::appendColoredText, Qt::QueuedConnection);

    // Connections
    connect(browseBtn_, &QPushButton::clicked, this, &MainWindow::onBrowseClicked);
    connect(runBtn_,    &QPushButton::clicked, this, &MainWindow::onRunClicked);
    connect(closeBtn_,  &QPushButton::clicked, this, &MainWindow::onCloseClicked);
}

void MainWindow::appendColoredText(const QString& text, const QColor& color)
{
    QTextCursor cursor = resultEdit_->textCursor();
    cursor.movePosition(QTextCursor::End);

    QTextCharFormat format;
    format.setForeground(color);
    cursor.setCharFormat(format);

    cursor.insertText(text);
    resultEdit_->setTextCursor(cursor);

    // Вариант А: Прокрутка вниз, чтобы видеть новые логи (если нужно)
    resultEdit_->ensureCursorVisible();

    // Вариант Б: Принудительно заставляем Qt перерисовать текстовое поле прямо СЕЙЧАС,
    // не дожидаясь окончания текущей функции или цикла.
    resultEdit_->repaint();
}

void MainWindow::onBrowseClicked()
{
    QString file = QFileDialog::getOpenFileName(
        this,
        "Select input file",
        QString(),
        "Json files (*.json);;All files (*)");

    if (!file.isEmpty())
        fileEdit_->setText(file);
}

void MainWindow::onRunClicked()
{
    const QString filename = fileEdit_->text().trimmed();
    if (filename.isEmpty()) {
        QMessageBox::warning(this, "Error", "Please select a file first.");
        return;
    }

    statusLabel_->setText("Running...");
    resultEdit_->clear();
    QApplication::processEvents();   // keep UI responsive for tiny tasks
    
    try
    {
        std::string input_filename = filename.toStdString();

        statusLabel_->setText("Opening " + filename);

        Satellite satellite;
        Satellite* sat = &satellite;
        Time t;
        Time* time = &t;
        double interval;
        double step;
        double output_step;
        bool screen_check = true;
        
        if (Input::read_json_file(input_filename, sat, time, interval, step, output_step, screen_check))
        {
            return;
        }

        if (Input::show_input_statistics)
        {
            auto parameters_dict = Input::input_statistics();
            
            int number_of_rows = 0;
            for (const auto& [section, parameters]: parameters_dict)
            {
                for (const auto& [key, value]: parameters)
                {
                    number_of_rows++;
                }
            }
            MapDialog dialog_(parameters_dict, number_of_rows, this);

            if (dialog_.exec() == QDialog::Accepted)
            {
                statusLabel_->setText("Running...");
                FullMotionIntegrator fullmotion(sat, time, interval, step, output_step, false, screen_check);
                try
                {
                    fullmotion.integrate();
                    std::cout << "\033[32m----------INTEGRATION COMPLETE------------\033[0m" << std::endl;
                    statusLabel_->setText("Done");
                    Input::reset_input_statistics();
                }
                catch (...)
                {
                    statusLabel_->setText("Error. Integration is stopped");
                    Input::reset_input_statistics();
                }
            }
            else
            {
                std::cout << "\033[31m--Ingeration intercepted --\033[0m" << std::endl;
                statusLabel_->setText("Done");
                Input::reset_input_statistics();
                return;
            }
        }

    }
    catch (...)
    {
        std::cerr << "\033[31mCouldn't make it\033[0m" << std::endl;
        statusLabel_->setText("Failed");
        return;
    }

    /*
    if (!result.success) {
        statusLabel_->setText("Failed");
        resultEdit_->setPlainText(QString::fromStdString(result.message));
        return;
    }

    statusLabel_->setText(QString::fromStdString(result.message));

    QString output;
    output += QString("Average value: %1\n\n").arg(result.value);
    output += "File content:\n";
    for (const auto& line : result.lines)
        output += QString::fromStdString(line) + '\n';

    resultEdit_->setPlainText(output);
    */
}

void MainWindow::onCloseClicked()
{
    this->close();
}

#endif
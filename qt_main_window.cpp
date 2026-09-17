#include "stdafx.h"

#ifdef SOAP_WITH_QT

#include<QtConcurrent>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    setvbuf(stdout, nullptr, _IONBF, 0);
    setvbuf(stderr, nullptr, _IONBF, 0);

    std::ios_base::sync_with_stdio(false);
    std::cout << std::unitbuf;
    std::cerr << std::unitbuf;

    setupUi();
    setWindowTitle("Spacecraft Orbit and Attitude Prediction Tool");
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

    
    logTimer_ = new QTimer(this);
    connect(logTimer_, &QTimer::timeout, this, &MainWindow::onLogTimer);
    logTimer_->start(40);

    // Connections
    connect(browseBtn_, &QPushButton::clicked, this, &MainWindow::onBrowseClicked);
    connect(runBtn_,    &QPushButton::clicked, this, &MainWindow::onRunClicked);
    connect(closeBtn_,  &QPushButton::clicked, this, &MainWindow::onCloseClicked);
    connect(&futureWatcher_, &QFutureWatcher<void>::finished, this, &MainWindow::onIntegrationFinished);
}

void MainWindow::setUiRunning(bool running)
{
    runBtn_->setEnabled(!running);
    browseBtn_->setEnabled(!running);
    closeBtn_->setEnabled(!running);
    statusLabel_->setText(running ? "Running integration..." : "Ready");
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

void MainWindow::onLogTimer()
{
    auto entries = LogQueue::instance().drain();
    if (entries.isEmpty())
    {
        return;
    }

    QTextCursor cursor = resultEdit_->textCursor();
    cursor.movePosition(QTextCursor::End);

    resultEdit_->setUpdatesEnabled(false); // freeze painting

    for (const auto& e: entries)
    {
        QTextCharFormat fmt;
        fmt.setForeground(e.color);
        cursor.setCharFormat(fmt);
        cursor.insertText(e.text);
    }

    resultEdit_->setTextCursor(cursor);
    resultEdit_->setUpdatesEnabled(true);
    resultEdit_->ensureCursorVisible();
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

    // prevent double-clicks
    if (futureWatcher_.isRunning())
    {
        QMessageBox::information(this, "Busy", "Integration is already running");
        return;
    }

    resultEdit_->clear();
    setUiRunning(true);
    
    std::string input_filename = filename.toStdString();

    
    statusLabel_->setText("Opening " + filename);

    Satellite satellite;
    Time time;
    double interval;
    double step;
    double output_step;
    bool screen_check = true;


    try
    {
        satellite.resetModes();
        if (Input::read_json_file(input_filename, &satellite, &time, interval, step, output_step, screen_check))
        {
            setUiRunning(false);
            statusLabel_->setText("Failed to read input");
            return;
        }
    }
    catch (...)
    {
        std::cerr << "\033[31mCouldn't make it\033[0m" << std::endl;
        setUiRunning(false);
        statusLabel_->setText("Failed");
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

        if (dialog_.exec() != QDialog::Accepted)
        {
            std::cout << "\033[31m--Ingeration intercepted --\033[0m" << std::endl;
            setUiRunning(false);
            statusLabel_->setText("Canceled");
            Input::reset_input_statistics();
            return;
        }
    }

    statusLabel_->setText("Running...");

    Satellite sat_copy = satellite;
    Time time_copy = time;

    QFuture<void> future = QtConcurrent::run([sat_copy, time_copy, interval, step, output_step, screen_check]() mutable
    {
        try
        {
            if (!sat_copy.getHdfFile().empty())
            {
                SRPManager::initSRPEngine(sat_copy.getHdfFile());
                SRPManager::warmupSRP();
            }

            FullMotionIntegrator fullmotion(&sat_copy, &time_copy, interval, step, output_step, false, screen_check);
            
            fullmotion.integrate();
            
            if (!sat_copy.getHdfFile().empty())
            {
                SRPManager::shutdown();
            }

            std::cout << "\033[32m----------INTEGRATION COMPLETE------------\033[0m" << std::endl;
            
            //Input::reset_input_statistics();
        }
        catch (...)
        {
            //statusLabel_->setText("Error. Integration is stopped");
            std::cerr << "\033[31mIntegration is interrupted\033[0m\n";
        }

        Input::reset_input_statistics();
    });
    
    futureWatcher_.setFuture(future);

}

void MainWindow::onIntegrationFinished()
{
    setUiRunning(false);

    if (futureWatcher_.isCanceled()) {
        statusLabel_->setText("Canceled");
    }
    else
    {
        statusLabel_->setText("Done");
    }
}

void MainWindow::onCloseClicked()
{
    this->close();
}

#endif
#include "stdafx.h"
#ifdef SOAP_WITH_QT

//#include <QHeaderView>
//#include <QTableWidgetItem>
//#include <QColor>
//#include <QBrush>
//#include <QVBoxLayout>

MapDialog::MapDialog(const std::map<std::string, std::map<std::string, bool>>& data, const int number_of_rows,
	QWidget *parent): QDialog(parent)
{
	setWindowTitle("Input statistics");
	resize(350, 500);

	// Create table
	tableWidget_ = new QTableWidget(this);

	tableWidget_->setColumnCount(2);
	tableWidget_->setHorizontalHeaderLabels({"Parameter", "State"});

	tableWidget_->setRowCount(number_of_rows);

	//Make columns stretch to fill the window
	tableWidget_->horizontalHeader()->setSectionResizeMode(
		0, QHeaderView::Stretch);
	tableWidget_->horizontalHeader()->setSectionResizeMode(
		1, QHeaderView::ResizeToContents);

	// Disable editing
	tableWidget_->setEditTriggers(QAbstractItemView::NoEditTriggers);

	// Select entire rows
	tableWidget_->setSelectionBehavior(
		QAbstractItemView::SelectRows);

	// Populate table
	int row = 0;

	for (const auto& [topic, parameters]: data)
	{
		auto *headerItem = new QTableWidgetItem(QString::fromStdString(topic));
		
		headerItem->setForeground(Qt::black);
		headerItem->setBackground(Qt::white);

		tableWidget_->setItem(row, 0, headerItem);
		tableWidget_->setSpan(row, 0, 1, 2);
		headerItem->setTextAlignment(Qt::AlignCenter);
		++row;

		for (const auto& [parameter, value]: parameters)
		{
			auto *keyItem = new QTableWidgetItem(QString::fromStdString(parameter));
			auto *valueItem = new QTableWidgetItem(value ? "OK" : "NOT GIVEN");

			// Color the row depending on the value
			QColor backgroundColor = 
				value
					? QColor(200, 255, 200) // light green
					: QColor(255, 200, 200); // light red

			keyItem->setBackground(QBrush(backgroundColor));
			keyItem->setForeground(Qt::black);
			
			valueItem->setBackground(QBrush(backgroundColor));
			valueItem->setForeground(Qt::black);

			// Make the parameter bold
			keyItem->setFont(QFont("Arial", -1, QFont::Bold));

			// Align center
			valueItem->setTextAlignment(Qt::AlignCenter);

			tableWidget_->setItem(row, 0, keyItem);
			tableWidget_->setItem(row, 1, valueItem);

			++row;
		}
	}


	// Buttons
	buttonBox_ = new QDialogButtonBox(
		QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
		Qt::Horizontal,
		this);

	connect(buttonBox_, &QDialogButtonBox::accepted, this, &QDialog::accept);
	connect(buttonBox_, &QDialogButtonBox::rejected, this, &QDialog::reject);

	// Layout
	auto *layout_ = new QVBoxLayout(this);

	layout_->addWidget(tableWidget_);
	layout_->addWidget(buttonBox_);

	setLayout(layout_);
}

#endif
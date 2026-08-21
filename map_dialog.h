#pragma once

#ifdef SOAP_WITH_QT

#include <QDialog>
#include <QTableWidget>
#include <QDialogButtonBox>
#include<QHeaderView>
#include<QTableWidgetItem>
#include<QColor>
#include<QBrush>
#include <QString>
#include<QVBoxLayout>

class MapDialog: public QDialog
{
	Q_OBJECT

public:
	explicit MapDialog(const std::map<std::string, std::map<std::string, bool>>& data, const int number_of_rows,
		QWidget *parent = nullptr);

private:
	QTableWidget *tableWidget_;
	QDialogButtonBox *buttonBox_;
};

#endif
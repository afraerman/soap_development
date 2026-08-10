#include "stdafx.h"

Time::Time()
{
	year = 2000;
	month = 1;
	day = 10;
	hours = 12;
	minutes = 0;
	seconds = 0.0;
}

Time::Time(const std::string& t)
{
	const std::regex pattern(
		R"(^(\d{4}).*?(\d{2}).*?(\d{2}).*?(\d{2}).*?(\d{2}).*?(\d{2}).*?$)");

	std::smatch match;
	if (!std::regex_match(t, match, pattern))
	{
		throw std::runtime_error("Time doesn't match format YYYY-MM-DDTHH:MM:SS.S");
	}

	year    = std::stoi(match[1]);
	month   = std::stoi(match[2]);
	day     = std::stoi(match[3]);
	hours   = std::stoi(match[4]);
	minutes = std::stoi(match[5]);
	int int_seconds = std::stoi(match[6]);

	if (month < 1 || month > 12) throw std::runtime_error("Month can't be "+std::to_string(month));
	if (hours < 0 || hours > 23) throw std::runtime_error("Hours can't be "+std::to_string(hours));
	if (minutes < 0 || minutes > 59) throw std::runtime_error("Minutes can't be "+std::to_string(minutes));
	if (int_seconds < 0 || int_seconds > 59) throw std::runtime_error("Seconds can't be "+std::to_string(int_seconds));

	const int days_in_month[] = {0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
	int max_day = days_in_month[month];
	if (month == 2)
	{
		bool leap = (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0);
		if (leap) max_day = 29;
	}
	if (day < 1 || day > max_day) throw std::runtime_error("Day can't be "+std::to_string(day)+" in month "+std::to_string(month));

	std::stringstream ss_str;
	char tmp;
	ss_str << t;
	ss_str >> year >> tmp >> month >> tmp >> day >> tmp >> hours >> tmp >> minutes >> tmp >> seconds;
}

Time::Time(int year_, int month_, int day_, int hours_, int minutes_, double seconds_)
{
	year = year_;
	month = month_;
	day = day_;
	hours = hours_;
	minutes = minutes_;
	seconds = seconds_;
}

void Time::setTime(const Time& time)
{
	year = time.year;
	month = time.month;
	day = time.day;
	hours = time.hours;
	minutes = time.minutes;
	seconds = time.seconds;
}

int Time::getYear() const
{
	return year;
}

int Time::getMonth() const
{
	return month;
}

int Time::getDay() const
{
	return day;
}

int Time::getHours() const
{ 
	return hours;
}

int Time::getMinutes() const
{
	return minutes;
}

double Time::getSeconds() const
{
	return seconds;
}

double Time::toSeconds() const
{
	return (double)hours * 3600.0 + (double)minutes * 60.0 + seconds;
}

SpiceDouble Time::ET() const
{
	SpiceDouble et;
	std::string date_str{ std::to_string(year) + "/" + std::to_string(month) + "/" + std::to_string(day) + " " +
		std::to_string(hours) + ":" + std::to_string(minutes) + ":" + std::to_string(seconds) };
	const char* date = date_str.data();

	utc2et_c(date, &et);

	return et;
}

bool Time::operator==(const Time& t) const
{
	return ((year == t.year) && (month == t.month) && (day == t.day) && (hours == t.hours) && (minutes == t.minutes) && (seconds == t.seconds));
}

bool Time::operator>(const Time& t) const
{
	if (year > t.getYear()) return true;
	else if (year < t.getYear()) return false;
	else
	{
		if (month > t.getMonth()) return true;
		else if (month < t.getMonth()) return false;
		else
		{
			if (day > t.getDay()) return true;
			else if (day < t.getDay()) return false;
			else
			{
				if ((double)hours * 3600.0 + (double)minutes * 60.0 + seconds > (double)t.getHours() * 3600.0 + (double)t.getMinutes() * 60.0 + (double)t.getSeconds()) return true;
				return false;
			}
		}
	}
}

bool Time::operator<(const Time& t) const
{
	if ((*this == t) || (*this > t)) return false;
	return true;
}

bool Time::operator<=(const Time& t) const
{
	if ((*this == t) || (*this < t)) return true;
	return false;
}

bool Time::operator>=(const Time& t) const
{
	if ((*this == t) || (*this > t)) return true;
	return false;
}

Time Time::operator+(const double dt) const
{
	int new_year = year;
	int new_month = month;
	int new_day = day;
	int new_hours = hours;
	int new_minutes = minutes;
	double new_seconds = seconds;

	int last_day = new_day;

	double utc1, utc2, fd;
	int ye, mo, da;

	iauDtf2d("utc", year, month, day, hours, minutes, seconds, &utc1, &utc2);

	new_seconds += dt;

	new_minutes += (int)(new_seconds) / 60;
	new_seconds -= (double)((int)(new_seconds) / 60) * 60.0;

	new_hours += new_minutes / 60;
	new_minutes -= new_minutes / 60 * 60;

	new_day += new_hours / 24;
	new_hours -= new_hours / 24 * 24;

	if (last_day != new_day)
	{
		utc1 += 1.0;
		iauJd2cal(utc1, utc2, &ye, &mo, &da, &fd);
		if (ye != year) // new year
		{
			new_year = ye;
			new_month = 1;
			new_day = 1;
		}
		else if (mo != month) // only new month
		{
			new_month = mo;
			new_day = 1;
		}
	}

	Time new_t(new_year, new_month, new_day, new_hours, new_minutes, new_seconds);
	return new_t;
}

double Time::operator-(const Time& t) const
{
	/*
	if ((year == t.year) && (month == t.month) && (day == t.day))
	{
		return this->toSeconds() - t.toSeconds();
	}
	else if ((year == t.year) && (month == t.month) && (day > t.day))
	{
		return (double)(day - t.day) * 86400.0 + this->toSeconds() - t.toSeconds();
	}
	else
	{
		
		std::cout << "Subtracting dates from different months or even years, ooouuuu-iiiii" << std::endl;
		return -1.0;
	}*/
	double utc11, utc12, utc21, utc22, jd1, jd2;
	iauDtf2d("utc", year, month, day, hours, minutes, seconds, &utc11, &utc12);
	iauDtf2d("utc", t.getYear(), t.getMonth(), t.getDay(), t.getHours(), t.getMinutes(), t.getSeconds(), &utc21, &utc22);

	jd1 = utc11 + utc12;
	jd2 = utc21 + utc22;
	return (jd1 - jd2) * 86400.0;

}

Time& Time::operator+=(const double dt)
{
	int last_day = day;

	double utc1, utc2, fd;
	int ye, mo, da;

	iauDtf2d("utc", year, month, day, hours, minutes, seconds, &utc1, &utc2);

	seconds += dt;

	minutes += (int)(seconds) / 60;
	seconds -= (double)((int)(seconds) / 60) * 60.0;

	hours += minutes / 60;
	minutes -= minutes / 60 * 60;

	day += hours / 24;
	hours -= hours / 24 * 24;
	
	if (last_day != day)
	{
		utc1 += 1.0;
		iauJd2cal(utc1, utc2, &ye, &mo, &da, &fd);
		if (ye != year) // new year
		{
			year = ye;
			month = 1;
			day = 1;
		}
		else if (mo != month) // only new month
		{
			month = mo;
			day = 1;
		}
		Astrometry::EOP(*this);
	}
	
	return *this;
}

std::string Time::toString() const
{
	return std::to_string(year) + "-" + std::to_string(month) + "-" + std::to_string(day) + "T" + std::to_string(hours) + ":" + std::to_string(minutes) + ":" + std::to_string(seconds);
}

std::ostream& operator<<(std::ostream& os, const Time& time)
{
	os << time.year << '-' << time.month << '-' << time.day << "T" << time.hours << ":" << time.minutes << ":" << time.seconds;
	return os;
}

Time::~Time() {};
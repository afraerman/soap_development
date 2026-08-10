#pragma once
class Time
{
private:
	int year;
	int month;
	int day;
	int hours;
	int minutes;
	double seconds;
public:
	Time();
	Time(const std::string&);
	Time(int, int, int, int, int, double);

	void setTime(const Time& time);

	int getYear() const;
	int getMonth() const;
	int getDay() const;
	int getHours() const;
	int getMinutes() const;
	double getSeconds() const;
	double toSeconds() const;
	SpiceDouble ET() const;

	bool operator==(const Time& t) const;
	bool operator>(const Time& t) const;
	bool operator<(const Time& t) const;
	bool operator<=(const Time& t) const;
	bool operator>=(const Time& t) const;
	Time operator+(const double) const;
	Time& operator+=(const double);
	double operator-(const Time&) const;

	std::string toString() const;

	friend std::ostream& operator<<(std::ostream&, const Time&);

	~Time();
};
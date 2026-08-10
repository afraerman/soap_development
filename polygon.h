#pragma once
class Polygon
{
private:
	PositionVector position;
	PositionVector normal;
	double area;
	double reflectivity_factor;
	double specularity_factor;
	int rotation_axis_index;
public:
	Polygon();
	Polygon(const PositionVector&, const PositionVector&, double);
	Polygon(const PositionVector&, const PositionVector&, double, double, double, int rai=-1);
	
	void setReflectivityFactor(double);
	void setSpecularityFactor(double);
	void setArea(double);
	void setNormal(const PositionVector& n);
	void setPosition(const PositionVector& pos);

	PositionVector getPosition() const;
	PositionVector getNormal() const;
	double getArea() const;
	double getReflectivityFactor() const;
	double getSpecularityFactor() const;
	int getRotationAxisIndex() const;
};
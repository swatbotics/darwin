/*
 *   Point.h
 *
 *   Author: ROBOTIS
 *
 */

#ifndef _POINT_H_
#define _POINT_H_


namespace Robot
{	
	class Point2D
	{
	private:

	protected:

	public:
		double X;
		double Y;

		Point2D();
		Point2D(double x, double y);
		Point2D(const Point2D &point);		
		~Point2D();

		static double Distance(const Point2D &pt1, const Point2D &pt2);

		Point2D & operator = (const Point2D &point);
		Point2D & operator += (const Point2D &point);
		Point2D & operator -= (const Point2D &point);
		Point2D & operator += (double value);
		Point2D & operator -= (double value);
		Point2D & operator *= (double value);
		Point2D & operator /= (double value);
		Point2D operator + (const Point2D &point) const;
		Point2D operator - (const Point2D &point) const;
		Point2D operator + (double value) const;
		Point2D operator - (double value) const;
		Point2D operator * (double value) const;
		Point2D operator / (double value) const;
	};

	class Point3D
	{
	private:

	protected:

	public:
		double X;
		double Y;
		double Z;
		
		Point3D();
		Point3D(double x, double y, double z);
		Point3D(const Point3D &point);
		~Point3D();

		static double Distance(const Point3D &pt1, const Point3D &pt2);

		Point3D & operator = (const Point3D &point);
		Point3D & operator += (const Point3D &point);
		Point3D & operator -= (const Point3D &point);
		Point3D & operator += (double value);
		Point3D & operator -= (double value);
		Point3D & operator *= (double value);
		Point3D & operator /= (double value);
		Point3D operator + (const Point3D &point) const;
		Point3D operator - (const Point3D &point) const;
		Point3D operator + (double value) const;
		Point3D operator - (double value) const;
		Point3D operator * (double value) const;
		Point3D operator / (double value) const;
	};
}

#endif

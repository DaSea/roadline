/*
 * Copyright (C) 2016-01-25 Dasea <dhf0214@126.com>
 *
 * Description: 定义线类型
 */

#ifndef __LINE_GEOMETRY_H__
#define __LINE_GEOMETRY_H__

#include "BaseGeometry.h"

class LineGeometry {
public:
    LineGeometry();
    LineGeometry(Point3D& pt, double len, double angle);
    virtual ~LineGeometry(void);

public:
    void SetGeometryPorperty(Point3D& pt, double len, double angle);

    Point3D* GetGeometryPoints(int& ptCount);

    Point3D GetPointByArcLen(Point3D& pedal, double mileage, double offset);
    void GetVDisAndArcLen(Point3D& point, Point3D& pedal, double& mileage, double& offset);

    static void test();

protected:
    void calculateEndPoint();
    Point3D calUnKownPoint(double mileage);

    Point3D computerBorderLeftPoint(Point3D& pedal, const double offset);
    Point3D computerBorderRightPoint(Point3D& pedal, const double offset);

    bool isInner(const Point3D& point);

private:
    Point3D _startPt;   // 起点
    Point3D _endPt;     // 终点
    double  _length;    // 线长
    double  _angle;     // 方位角
};

#endif


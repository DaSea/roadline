/*
 * Copyright (C) 2016-01-26 Dasea <dhf0214@126.com>
 *
 * Description: 圆曲线声明
 */

#ifndef __CIRCLE_CURVE_H__
#define __CIRCLE_CURVE_H__

#include "BaseGeometry.h"

/**
 * @brief  圆曲线
 */
class CircleCurveGeometry {
public:
    CircleCurveGeometry();
    CircleCurveGeometry(Point3D& startPt, Point3D& middlePt, Point3D& endPt);
    CircleCurveGeometry(Point3D& startPt, Point3D& endPt, double radius, int direction);
    CircleCurveGeometry(Point3D& startPt, double radius, double len, double angle, int direction);
    virtual ~CircleCurveGeometry(void);

public:
    void SetGeometryPorperty(Point3D& startPt, Point3D& middlePt, Point3D& endPt);
    void SetGeometryPorperty(Point3D& startPt, double radius, double len, double angle, int direction);

    Point3D* GetGeometryPoints(int& ptCount, double& endAngle);
    void GetVDisAndArcLen(Point3D& resolvePt, Point3D& pedal, double& mileage, double& offset);
    Point3D GetPointByArcLen(Point3D& pedal, double mileage, double offset);

protected:
    int getPointsCount();
    void computerCurvePara();
    void computerRadio(void);
    void computerArcLen();
    void computerDirection();
    void computerEndPoint();

    // 由距离圆弧起点的里程长度计算点
    Point3D calUnKownPoint(double mileage);

    // 由里程和偏距计算圆弧上的点
    double getReAngle(double mileage);
    Point3D computerBorderLeftPoint(Point3D& pedal, double mileage, double offset);
    Point3D computerBorderRightPoint(Point3D& pedal, double mileage, double offset);

    // 判断线外点是否属于曲线区域
    bool besureInCurveArea(Point3D& offlinePt, double& startVerDis);

    // 由点计算偏距和里程
    double getVerDisOfPtToTangent(Point3D& point, Point3D& desPt, double angle);
    void getMileageMesFromOutPt(Point3D& point, double& startMileage, double& mileage, double& offset);
    void computerDH(Point3D& point, Point3D& desPt, double& zhLen, double& dL, double& angle, int& num);

private:
    Point3D _startPt;   // 圆弧起点
    Point3D _middlePt;  // 中间点
    Point3D _endPt;     // 终点

    Point3D _centerPt;  // 圆心

    double _radius;     // 圆弧半径
    double _length;     // 圆弧长度
    double _angle;      // 圆弧起始点的方位角

    int _direction;  // 左转还是右转
};

#endif

/*
 * Copyright (C) 2016-01-27 Dasea <dhf0214@126.com>
 *
 * Description: 缓和曲线类声明
 */


#ifndef __CURVEGEOMETRY_H__
#define __CURVEGEOMETRY_H__

#include "BaseGeometry.h"

// 曲线类型,有四种
enum CURVE_T {
    T_SZ_EY = 1, // 完整的缓和曲线, 起点半径为无穷大, 终点半径为R
    T_SYB_EYS = 2, // 非完整的缓和曲线,起点.终点半径都大于0,且起点半径大于终点半径
    T_SYS_EYB = 3, // 非完整的缓和曲线,起点.终点半径都大于0,且起点半径小于终点半径
    T_SY_EZ = 4, // 完整的缓和曲线, 起点半径为R, 终点半径为无穷大;
    T_NO
};

class CurveGeometry {
public:
    CurveGeometry();
    CurveGeometry(Point3D& startPt, double startR, double endR, double len, double angle, int direction);
    virtual ~CurveGeometry(void);

public:
    void SetGeometryPorperty(Point3D& startPt, double startR, double endR, double len, double angle, int direction);

    Point3D* GetGeometryPoints(int& ptCount, double& endAngle);
    void GetVDisAndArcLen(Point3D& resolvePt, Point3D& pedal, double& mileage, double& offset);
    Point3D GetPointByArcLen(Point3D& pedal, double mileage, double offset);

private:
    void init();
    // 参数计算
    void computerParam();
    CURVE_T getType(); // 判断曲线类型
    void computerParamA(); // 计算回旋曲线参数的平方
    void computerEmluatorPara(); // 补全曲线长度

    // 生成线需要的点个数
    int getPointsCount(void);
    // 根据里程生成点
    void calUnKownPoint(Point3D* point, double mileage);

    // 计算在ZH或HZ坐标系中的x,y
    double computerPointX0(double mileage);
    double computerPointY0(double mileage);

    // 根据x, y计算象限
    int judgeQuadrant(double x0, double y0);

    // 根据已存在参数计算点数组
    void computerOneFullCurvePoints(Point3D* points, int count, double lineGap);
    void computerTwoFullCurvePoints(Point3D* points, int count, double lineGap);
    void computerOneNoFullCurvePoints(Point3D* points, int count, double lineGap);
    void computerTwoNoFullCurvePoints(Point3D* points, int count, double lineGap);

    // 获取ZH或者HZ点
    void computerZHOrHZPoint();
    Point3D computerTwoFullCurveHZPoint();
    Point3D computerOneNoFullCurveZHPoint();
    Point3D computerTwoNoFullCurveHZPoint();

    // 计算最后一点
    void computerEndPoint();

    // 计算偏桩点
    Point3D computerBorderLeftPoint(Point3D& pedal, double mileage, double offset);
    Point3D computerBorderRightPoint(Point3D& pedal, double mileage, double offset);

    // 反算算法
    bool besureInCurveArea(Point3D& offlinePt, double& startVerDis);
    void inverseComputerOne(Point3D& offlinePt, Point3D& pedal, double& mileage, double& offset);
    void inverseComputerTwo(Point3D& offlinePt, Point3D& pedal, double& mileage, double& offset);
    void inverseComputerThree(Point3D& offlinePt, Point3D& pedal, double& mileage, double& offset);
    double getVerDisOfPtTotangent(Point3D& offlinePt, Point3D& inlinePt);

private:
    Point3D _startPt;
    Point3D _endPt;
    double _startRadius;
    double _endRadius;
    double _length;
    double _angle;
    int _direction;

    // 曲线类型: 第一缓曲, 第一不完整, 第二不完整, 第二缓曲
    CURVE_T _type;
    // 偏向系数K
    int _k;
    // ZH或HZ点坐标
    Point3D _zhPoint;
    // 计算时候的方位角(弧度)
    double _calAzimuth;
    // 曲线转角值
    double _corner;
    // 完整的曲线长
    double _fullLen;
    // 半径(_dr1: 大半径; _dr2: 小半径), 无穷大为0
    double _dr1;
    double _dr2;

    // 中间参数
    double _paramA; // 回旋曲线参数的平方
};

#endif /* __CURVEGEOMETRY_H__ */

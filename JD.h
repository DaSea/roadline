/*
 * Copyright (C) 2016-03-02 Dasea <dhf0214@126.com>
 *
 * Description: 交点法算法头文件
 */

#include "BaseGeometry.h"
#include <vector>

enum GeometryType {
    TYPE_LINE = 1,
    TYPE_CIRCLE = 2,
    TYPE_CURVE_ONE = 3,
    TYPE_CURVE_TWO = 4,
    TYPE_NO
};

struct JDGeometry {
    GeometryType _type;
    double _kd; // 起点里程

    Point3D _startPt;
    double  _length;
    double  _azimuth;
    double  _direction;
    JDGeometry() {
        _length = 0;
        _azimuth = 0;
        _type = TYPE_NO;
        _kd = 0;
        _direction = 0;
    }
    virtual ~JDGeometry() {}
};

struct JDLine:public JDGeometry {
    JDLine() {
        _type = TYPE_LINE;
    }
};

struct JDCircle:public JDGeometry {
    double  _radius;

    JDCircle() {
        _radius = 0;
        _direction = 0;
        _type = TYPE_CIRCLE;
    }
};

struct JDCurve:public JDGeometry {
    double _startRadius;
    double _endRadius;
    JDCurve() {
        _startRadius = 0;
        _endRadius = 0;
        _direction = 0;
        _type = TYPE_CURVE_ONE;
    }
};

// 三个交点之间存在的线类型
enum JDCurveType {
    JDCURVETYPE_LINE = 1,
    JDCURVETYPE_CIRCLE = 2,
    JDCURVETYPE_ONE = 3,
    JDCURVETYPE_TWO = 4,
    JDCURVETYPE_ONETWO = 5,
    JDCURVETYPE_NO = 6
};

// 交点结构体
struct JDPoint {
    Point3D _point;
    double  _kd;
    double  _curveLength1;
    double  _radius;
    double  _curveLength2;
    JDPoint() {
        _kd = 0;
        _curveLength1 = 0;
        _radius = 0;
        _curveLength2 = 0;
    }

    JDPoint& operator=(const JDPoint& other) {
        if(this == &other)
            return (*this);

        this->_kd = other._kd;
        this->_point = other._point;
        this->_curveLength1 = other._curveLength1;
        this->_curveLength2 = other._curveLength2;
        this->_radius = other._radius;
        return (*this);
    }
};

struct JDData{
    double x;
    double y;
    double lenOne;
    double radius;
    double lenTwo;
};

class JDFactory {
public:
    JDFactory();
    virtual ~JDFactory(void);

    void PreprocessPoints();
    JDCurveType GetCurveType(const JDPoint& middlePt);

    void DealPoints(JDData points[], int count, double startKD);
    void ProcessCurve();
    void printPointInfo(Point3D* points, int count);
private:
    // 处理直线段
    void computerLineParam(JDLine* line, JDPoint& startPt, Point3D& joinPt);
    // 处理第一缓和曲线
    void computerCurveOneParam(JDCurve* curve, JDPoint& startPt, JDPoint& middlePt, JDPoint& endPt);
    Point3D getCurveOneStartPt(double tangent);
    double getCurveOneTangent();
    // 处理圆曲线
    void computerCircleParam(JDCircle* circle, JDPoint& startPt, JDPoint& middlePt, JDPoint& endPt);
    Point3D computerCircleCrossPoint(double tLen, double azimuth);
    // 处理第二缓和曲线
    void computerCurveTwoParam(JDCurve* curve, JDPoint& startPt, JDPoint& middlePt, JDPoint& endPt);

    // 根据包含的曲线类型处理线元
    void dealCurveWithType(JDCurveType type, JDPoint& startPt, JDPoint& middlePt, JDPoint& endPt);
    // 存线元
    void pushCurve(JDGeometry* geometry) ;

private:
    std::vector<JDPoint> _jdPoints; // 交点数组
    std::vector<JDGeometry*> _curveList; // 曲线数组

    // 用于计算时候的临时变量
    JDPoint _startPtTemp;
    JDPoint _middlePtTemp;
    JDPoint _endPtTemp;
    double  _curveLenOneTemp;
    double  _curveLenTwoTemp;
    double  _radiusTemp;

    // 保存中间衔接点,如ZH, HZ, ZY, YZ等
    JDPoint _zhORhzPt; // ZH点
    JDPoint _zyPoint;
    JDPoint _yzPoint;
    // 缓圆点, 第二缓和曲线的起点
    JDPoint _yhPoint;
    // 临时点
    JDPoint _tempPoint;
    // 终点桩号(YZ点相连的)
    double _endKDTemp;
};

/*
 * Copyright (C) 2016-01-26 Dasea <dhf0214@126.com>
 *
 * Description: 计算线段的使用
 */
#include "LineGeometry.h"
#include "MathFunction.h"

#include <cmath>
#include <iostream>

/**
 * Constructor or Destructor
 */
LineGeometry::LineGeometry(){
    // Constructor
    _length = 0;
    _angle = 0;
}

LineGeometry::LineGeometry(Point3D& pt, double len, double angle){
    DEBUG_OUT("================直线计算===========================");
    // Constructor
    _startPt = pt;
    _length = len;
    _angle = angle;
}

LineGeometry::~LineGeometry(){
    // Destroy it
}

/**
 * @brief  设置线的属性
 *
 * @param  pt 起始点
 * @param  len 线长
 * @param  angle 线的方位角(弧度)
 */
void LineGeometry::SetGeometryPorperty(Point3D& pt, double len, double angle) {
    _startPt = pt;
    _length = len;
    _angle = angle;
}

/**
 * @brief  返回点数组,指针需要调用者释放
 *
 * @param  ptCount out, 记录点数组的点个数
 *
 * @return 头指针
 */
Point3D* LineGeometry::GetGeometryPoints(int& ptCount){
    // Calculate end point
    calculateEndPoint();

    // 返回
    Point3D *points = new Point3D[2];
    points[0] = _startPt;
    points[1] = _endPt;

    ptCount = 2;
    return (points);
}

/**
 * @brief  根据里程和偏距求偏桩点
 *
 * @param  pedal 返回垂足,及中桩点
 * @param  mileage 里程
 * @param  offset 偏距(left<0; right>0)
 *
 * @return 偏桩点
 */
Point3D LineGeometry::GetPointByArcLen(Point3D& pedal, double mileage, double offset) {
    // 如果里程超过弧长的话, 则是不合法的值
    // if(_length < mileage){
        // pedal.x = 0; pedal.y = 0; pedal.z = 0;
        // return Point3D(0, 0, 0);
    // }

    pedal = calUnKownPoint(mileage);

    Point3D point;
    if((0.0001 >= offset) && (0 < offset)){
        // 位0的话则为线上点
        point = pedal;
    }else if(0 > offset){
        // 小于0位左侧点
        offset = fabs(offset);
        point = computerBorderLeftPoint(pedal, offset);
    }else{
        // 大于0为右侧点
        point = computerBorderRightPoint(pedal, offset);
    }
    return (point);
}

/**
 * @brief  根据线外点求垂足,里程和偏距
 *
 * @param  point 线外点,偏桩点
 * @param  pedal out,垂足,中桩点
 * @param  mileage out,里程
 * @param  offset out,偏距
 */
void LineGeometry::GetVDisAndArcLen(Point3D& point, Point3D& pedal, double& mileage, double& offset) {
    calculateEndPoint();
    // 非法的点
    // if(!isInner(point)){
        // pedal.x = 0; pedal.y = 0;
        // mileage = 0;
        // offset = 0;
        // return ;
    // }

    // 判断点在矢量线段的那一侧:起点:startPt(x1, y1), 终点:endPt(x2, y2), 线外点: point(x,y)
    // f = (x2 - x1) * (y - y1)  - (x - x1)*(y2 - y1);
    // if f > 0: 在左侧, 但是从p到线需要右偏; 否则左偏
    double directionXY = (_endPt.x - _startPt.x)*(point.y - _startPt.y) - (point.x - _startPt.x)*(_endPt.y - _startPt.y);
    double direction = 1;
    if(directionXY < 0){
        direction = -1;
    }
    if(_endPt.y == _startPt.y) {
        // 垂足
        pedal.x = point.x;
        pedal.y = _startPt.y;
        // 偏距
        offset = direction * fabs(point.y - _startPt.y);
        // 里程
        mileage = fabs(point.x - _startPt.x);

        return ;
    } else if(_endPt.x == _startPt.x) {
        // 垂足
        pedal.x = _startPt.x;
        pedal.y = point.y;
        // 偏距
        offset = direction * fabs(_startPt.x - point.x);
        // 里程
        mileage = fabs(point.y - _startPt.y);

        return ;
    }

    // 求原直线方程参数y = kx + c
    double k1 = (_endPt.y - _startPt.y) / (_endPt.x - _startPt.x);
    double c1 = _startPt.y - _startPt.x * k1;
    // 垂线的公式参数k2 = -1/k1;
    double k2 = (_startPt.x - _endPt.x) / (_endPt.y - _startPt.y);
    double c2 = point.y - point.x * k2;
    // 求俩条线的交点(x1, y1)
    double x1 = (c2 - c1) / (k1 - k2);
    double y1 = k1 * x1 + c1;
    // 所以垂足为:
    pedal.x = x1; pedal.y = y1;

    // 计算偏距
    double dx2 = pow(point.x - x1, 2);
    double dy2 = pow(point.y - y1, 2);
    offset = sqrt(dx2 + dy2);
    offset *= direction;

    // 计算里程
    dx2 = pow(x1 - _startPt.x, 2);
    dy2 = pow(y1 - _startPt.y, 2);
    mileage = sqrt(dx2 + dy2);
}
/**
 * @brief  返回边桩左边点
 *
 * @param  pedal 垂足,中转点
 * @param  offset 偏桩, 偏桩距离中线的距离
 *
 * @return 边桩左边点
 */
Point3D LineGeometry::computerBorderLeftPoint(Point3D& pedal, const double offset) {
    double angle = MathFunction::JudgeAzimuth(_angle - BORDER_ANGLE);
    Point3D point;
    point.x = pedal.x + offset*cos(angle);
    point.y = pedal.y + offset*sin(angle);
    return (point);
}

/**
 * @brief  返回边桩右边点
 *
 * @param  pedal 垂足,中桩点
 * @param  offset 偏距,偏桩距离中线的距离
 *
 * @return 边桩右边点
 */
Point3D LineGeometry::computerBorderRightPoint(Point3D& pedal, const double offset) {
    double angle = MathFunction::JudgeAzimuth(_angle + BORDER_ANGLE);
    Point3D point;
    point.x = pedal.x + offset*cos(angle);
    point.y = pedal.y + offset*sin(angle);
    return (point);
}

/**
 * @brief  根据已知参数求终点
 */
void LineGeometry::calculateEndPoint(void) {
    _endPt = calUnKownPoint(_length);
}

/**
 * @brief  根据里程计算线上点
 *
 * @param  mileage 里程
 *
 * @return 点坐标
 */
Point3D LineGeometry::calUnKownPoint(double mileage) {
    Point3D point;
    point.x = _startPt.x + cos(_angle)*mileage;
    point.y = _startPt.y + sin(_angle)*mileage;
    return (point);
}

/**
 * @brief  是否在线内
 *
 * @param  point 线外点
 *
 * @return 是或者不是
 */
bool LineGeometry::isInner(const Point3D& point) {
    double startDis = (point.x - _startPt.x)*sin(_angle + BORDER_ANGLE)
                    - (point.y - _startPt.y)*cos(_angle + BORDER_ANGLE);
    double endDis = (point.x - _endPt.x)*sin(_angle + BORDER_ANGLE)
                  - (point.y - _endPt.y)*cos(_angle + BORDER_ANGLE);
    if((0 <= startDis) && (0.0 >= endDis)){
        return true;
    }
    return false;
}


/**
 * 测试接口
 */
void LineGeometry::test() {
    // 2543825.441000-508812.911100; len: 70.206818; azimuth: 3.109807
    Point3D startPtNEU(2543825.441000, 508812.911100, 0);
    double length = 70.206818;
    double angle = 3.109807;
    LineGeometry* lineAlgorithm = new LineGeometry(startPtNEU, length, angle);
    Point3D* points = NULL;
    int count = 0;
    points = lineAlgorithm->GetGeometryPoints(count);

    for (int i = 0; i < count; ++i) {
        std::cout << "第"<<i<<"点"<<points[i].x<<"-"<< points[i].y<< " - " << points[i].z<< std::endl;
    }
}

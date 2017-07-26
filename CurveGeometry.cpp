/*
 * Copyright (C) 2016-01-27 Dasea <dhf0214@126.com>
 *
 * Description: 缓和曲线定义
 */
#include "CurveGeometry.h"
#include "GlobalSet.h"
#include "MathFunction.h"

#include <cmath>

/**
 * @brief  Constructor and Destructor
 * @param startPt 起点坐标
 * @param startR 起点半径
 * @param endR 终点半径
 * @param len 弧长
 * @param angle 起点方位角
 * @param direction 偏向
 */
CurveGeometry::CurveGeometry(){
    // Constructor
    _startRadius = 0;
    _endRadius = 0;
    _length = 0;
    _angle = 0;

    init();
}
CurveGeometry::CurveGeometry(Point3D& startPt, double startR, double endR, double len, double angle, int direction){
    // Constructor
    DEBUG_OUT("===============缓和曲线=============================");
    SetGeometryPorperty(startPt, startR, endR, len, angle, direction);
    init();
}
CurveGeometry::~CurveGeometry(){
    // Destructor
}
void CurveGeometry::init() {
    _type = T_NO;
    _k = 1;
    _fullLen = _length;
    _dr1 = 0;
    _dr2 = 0;
    _calAzimuth = _angle;
    _corner = 0;
}

/**
 * @brief  设置属性
 *
 * @param  参数和上面构造函数的相同
 */
void CurveGeometry::SetGeometryPorperty(Point3D& startPt, double startR, double endR,
                                        double len, double angle, int direction){
    _startPt = startPt;
    _startPt.azimuth = angle;

    _startRadius = startR;
    _endRadius = endR;
    _length = len;
    _angle = angle;
    _direction = direction;
}

/**
 * @brief  缓和曲线反算
 *
 * @param  resolvePt    in,线外点
 * @param  pedal        out,垂足
 * @param  mileage      out,里程
 * @param  offset       out,偏距（左负，右正）
 */
void CurveGeometry::GetVDisAndArcLen(Point3D& resolvePt, Point3D& pedal, double& mileage, double& offset) {
    computerParam();
    // 算法1
    // inverseComputerOne(resolvePt, pedal, mileage, offset);

    // 算法2
    // inverseComputerTwo(resolvePt, pedal, mileage, offset);

    // 算法3
    inverseComputerThree(resolvePt, pedal, mileage, offset);
}

/**
 * @brief  正算
 *
 * @param  pedal out, 垂足
 * @param  mileage out，里程
 * @param  offset out，偏距
 *
 * @return 边桩点
 */
Point3D CurveGeometry::GetPointByArcLen(Point3D& pedal, double mileage, double offset) {
    // 如果里程超过弧长的话, 这是不合法的值, mileage为到起点的距离, 而不是ZH或HZ的距离
    if(_length < mileage) {
        pedal.x = 0;
        pedal.y = 0;
        return Point3D(0, 0, 0);
    }

    computerParam();
    if((T_SY_EZ == _type)  || (T_SYS_EYB == _type)) {
        // 第二缓和曲线: 需要计算出距离HZ点的距离
        mileage = _fullLen - mileage;
    }else if(T_SYB_EYS == _type) {
        // 第一非完整缓和曲线: 第一非完整曲线, 同样需要计算到ZH点的距离
        mileage = mileage + _fullLen - _length;
    }
    DEBUG_OUT("需要计算的里程为: %f, 偏距: %f", mileage, offset);

    Point3D pt;
    if((0.0001 >= offset) && (0 < offset)) {
        calUnKownPoint(&pt, mileage);
        pedal = pt;
    } else if(0.0 > offset) {
        // 左侧点
        calUnKownPoint(&pedal, mileage);
        pt = computerBorderLeftPoint(pedal, mileage, fabs(offset));
    } else {
        // 右侧点
        calUnKownPoint(&pedal, mileage);
        pt = computerBorderRightPoint(pedal, mileage, offset);
    }

    return (pt);
}

/**
 * @brief  生成要画出缓和曲线所需的点数组
 *
 * @param  ptCount out,点个数
 * @param  endAngle out,最后一个点的方位角
 *
 * @return 点数组
 */
Point3D* CurveGeometry::GetGeometryPoints(int& ptCount, double& endAngle){
    // 计算一些参数
    computerParam();

    int count = getPointsCount();
    DEBUG_OUT("Point count: %d", count);

    Point3D* points = new Point3D[count];
    if(NULL == points) {
        DEBUG_OUT("CurveGeometry: malloc failed!");
        return NULL;
    }

    int lineGap = static_cast<int>(GlobalSet::GetCurveLineGap());
    switch(_type) {
    case T_SZ_EY: { // 第一完整缓和曲线
        computerOneFullCurvePoints(points, count, lineGap);
    }
    break;
    case T_SY_EZ: { // 第二完整缓和曲线
        computerTwoFullCurvePoints(points, count, lineGap);
    }
    break;
    case T_SYB_EYS: { // 第一不完整缓和曲线
        computerOneNoFullCurvePoints(points, count, lineGap);
    }
    break;
    case T_SYS_EYB: {  // 第二不完整缓和曲线
        computerTwoNoFullCurvePoints(points, count, lineGap);
    }
    break;
    default:
    // No select
    break;
    }

    endAngle = points[count-1].azimuth;
    ptCount = count;
    return (points);
}

//////////////////////////////////////////////////////////////////////////////////////////////
// Private function
//////////////////////////////////////////////////////////////////////////////////////////////


/**
 * @brief  生成第一完整曲线的方位角
 *
 * @param  points out, 点数组
 * @param  count  in, 点个数
 * @param  lineGap in, 点距
 */
void CurveGeometry::computerOneFullCurvePoints(Point3D* points, int count, double lineGap) {
    double m = _fullLen - _length;
    //    calUnKownPoint(points , m);
    //    m = 2.017;
    //    calUnKownPoint(points + 1, m);
    DEBUG_OUT("Initialize mileage: %f; calAzimuth: %f", m, _calAzimuth);
    for(int i = 0; i < count - 1; ++i) {
        calUnKownPoint(points + i, m);
        m += lineGap;
    }
    //处理最后一个点
    calUnKownPoint(points+count-1, _fullLen);

    // 算一个特殊点
    // Point3D pt;
    // calUnKownPoint(&pt, 29.793);
    // DEBUG_OUT("X-Y: %f, %f, Azimuth: %f", pt.x, pt.y, pt.azimuth);
}

/**
 * @brief  生成第二完整曲线的方位角
 *
 * @param  points out, 点数组
 * @param  count  in, 点个数
 * @param  lineGap in, 点距
 */
void CurveGeometry::computerTwoFullCurvePoints(Point3D* points, int count, double lineGap) {
    double m = _fullLen;
    DEBUG_OUT("Initialize mileage: %f; calAzimuth: %f", m, _calAzimuth);
    for(int i = 0; i < count-1; ++i) {
        calUnKownPoint(points+i, m);
        m -= lineGap;
    }

    // 处理最后一点
    calUnKownPoint(points+count-1, 0);
}

/**
 * @brief  生成第一不完整曲线的方位角
 *
 * @param  points out, 点数组
 * @param  count  in, 点个数
 * @param  lineGap in, 点距
 */
void CurveGeometry::computerOneNoFullCurvePoints(Point3D* points, int count ,double lineGap) {
    double m = _fullLen - _length;
    DEBUG_OUT("Initialize mileage: %f; calAzimuth: %f", m, _calAzimuth);
    for(int i = 0; i < count - 1; ++i) {
        calUnKownPoint(points + i, m);
        m += lineGap;
    }
    //处理最后一个点
    calUnKownPoint(points+count-1, _fullLen);
}

/**
 * @brief  生成第二不完整曲线的方位角
 *
 * @param  points out, 点数组
 * @param  count  in, 点个数
 * @param  lineGap in, 点距
 */
void CurveGeometry::computerTwoNoFullCurvePoints(Point3D* points, int count, double lineGap) {
    double m = _fullLen;
    DEBUG_OUT("Initialize mileage: %f; calAzimuth: %f", m, _calAzimuth);
    for(int i = 0; i < count - 1; ++i) {
        calUnKownPoint(points + i, m);
        m -= lineGap;
    }
    //处理最后一个点
    calUnKownPoint(points+count-1, _fullLen - _length);
}

/**
 * @brief  要画出缓曲所需要的点个数
 *
 * @return 点个数
 */
int CurveGeometry::getPointsCount(void) {
    int lineGap = static_cast<int>(GlobalSet::GetCurveLineGap());
    int nCount =static_cast<int>(_length)/lineGap+1;
    if (nCount<=1) nCount=2;
    return nCount;
}

/**
 * @brief  根据里程计算点
 *
 * @param  point out, 存放地址
 * @param  mileage in, 距离ZH或HZ点的里程(曲线长)
 */
void CurveGeometry::calUnKownPoint(Point3D* point, double mileage) {
    double x = 0.0;
    double y = 0.0;

    double x0 = computerPointX0(mileage);
    double y0 = computerPointY0(mileage);

    // 计算转角
    double corner = pow(mileage, 2) / (2 * _paramA);

    // 在一ZH点为原点的坐标系中, 距离原点的长度
    double s = sqrt(pow(x0, 2) + pow(y0, 2));
    double a0 = atan2(y0, x0);

    double a1 = a0 + _calAzimuth - PI/2;
//    DEBUG_OUT("M: %f, X0, Y0:%f, %f, s: %f, a0:%f, a1: %f, corner: %f"
//              ,mileage, x0, y0, s, a0, a1, corner);

    x = _zhPoint.x + s*cos(a1);
    y = _zhPoint.y + s*sin(a1);

    if((T_SZ_EY == _type) || (T_SYB_EYS == _type)) {
        // 第一段缓和曲线
        point->azimuth = MathFunction::JudgeAzimuth(_zhPoint.azimuth + _direction * corner);
    } else {
        // 第二段缓和曲线
        point->azimuth = MathFunction::JudgeAzimuth(_startPt.azimuth + _direction * (_corner - corner));
    }
    point->x = x;
    point->y = y;
    point->z = point->azimuth;

    DEBUG_OUT("M: %f, X0, Y0:%f - %f, x-y: %f - %f", mileage, x0, y0, x, y);
}

/**
 * @brief  计算曲线计算过程中的中间参数
 */
void CurveGeometry::computerParam() {
    // 判断曲线类型
    _type = getType();
    // 计算回旋曲线参数, 及对半径重排
    computerParamA();
    // 计算完整的曲线长度
    computerEmluatorPara();
    // 计算HZ或HZ点
    computerZHOrHZPoint();
    // 计算终点
    computerEndPoint();

    DEBUG_OUT("起点坐标: %f-%f: ", _startPt.x, _startPt.y);
    DEBUG_OUT("_type: %d, azimuth:%f, paramA: %f, _length:%f, _fullLen:%f",
              _type, _startPt.azimuth, _paramA, _length, _fullLen);
    DEBUG_OUT("半径: S-E: %f, %f, calAzimuth: %f, K: %d", _dr1, _dr2, _calAzimuth, _k);
    DEBUG_OUT("HZ或者ZH点坐标为: %f, %f, 方位角: %f, 转角: %f",
              _zhPoint.x, _zhPoint.y, _zhPoint.azimuth, _corner);
}

/**
 * @brief  计算ZH点或HZ点坐标及相关参数
 */
void CurveGeometry::computerZHOrHZPoint(void) {
    switch(_type) {
    case T_SZ_EY: { // 第一完整缓和曲线
        _zhPoint = _startPt;
        _calAzimuth = _angle;
        _corner = pow(_fullLen, 2) / (2 * _paramA);
    }
    break;
    case T_SY_EZ: { // 第二完整缓和曲线
        _zhPoint = computerTwoFullCurveHZPoint();
        _calAzimuth = _zhPoint.azimuth + PI;
    }
    break;
    case T_SYB_EYS: { // 第一不完整缓和曲线
        _zhPoint = computerOneNoFullCurveZHPoint();
        _calAzimuth = _zhPoint.azimuth;
    }
    break;
    case T_SYS_EYB: {  // 第二不完整缓和曲线
        _zhPoint = computerTwoNoFullCurveHZPoint();
        _calAzimuth = _zhPoint.azimuth + PI;
    }
    break;
    default:
    // No select
    break;
    }
}

/**
 * @brief  对不完整的第一缓和曲线进行补全,计算出ZH点
 *
 * @return ZH点坐标
 */
Point3D CurveGeometry::computerOneNoFullCurveZHPoint(void) {
    // 逆向运算算出ZH点
    double Lq = _fullLen - _length;
    // 1. 起点距离ZH点的弦长
    double Cq_zh = Lq - pow(Lq, 5)/(90 * pow(_paramA, 2));
    // 2, 计算切线角
    double Tq = pow(Lq, 2) / (2 * _paramA);
    // 3, 计算弦偏角
    double Dq = pow(Lq, 2) / (6 * _paramA);
    // 4, Q-ZH的坐标方位角
    double Aq_zh = 0;
    if(_direction < 0) {
        Aq_zh = _startPt.azimuth + Tq - Dq - PI;
    } else {
        Aq_zh = _startPt.azimuth - Tq + Dq + PI;
    }
    // 5, 直缓点坐标
    Point3D point;
    if(_direction < 0) {
        point.azimuth = _startPt.azimuth + Tq;
    } else {
        point.azimuth = _startPt.azimuth - Tq;
    }
    point.azimuth = MathFunction::JudgeAzimuth(point.azimuth);
    point.x = _startPt.x + Cq_zh * cos(Aq_zh);
    point.y = _startPt.y + Cq_zh * sin(Aq_zh);
    // 计算转角值
    _corner = pow(_fullLen, 2) / (2 * _paramA);
//    DEBUG_OUT("OneNo: len: %f, 弦长: %f, 切线角: %f, 弦偏角: %f, 方位角: %f",
//              Lq, Cq_zh, Tq, Dq, Aq_zh);
    return (point);
}

/**
 * @brief  对不完整的第二缓和曲线进行补全,并计算出HZ点
 *
 * @return HZ点坐标
 */
Point3D CurveGeometry::computerTwoNoFullCurveHZPoint(void) {
    // 对于第二段不完整缓和曲线需要进行逆向运算算出HZ点
    Point3D point;
    double x = 0;
    double y = 0;

    double x0 = computerPointX0(_fullLen);
    double y0 = computerPointY0(_fullLen);

    // 在一ZH点为原点的坐标系中, 距离原点的长度
    double s = sqrt(pow(x0, 2) + pow(y0, 2));
    double a0 = atan2(y0, x0);

    // 计算转角
    double corner = pow(_fullLen, 2) / (2 * _paramA);
    _corner = corner;
    // 计算HZ点方位角
    double zhAzimuth = 0.0;
    if(_direction > 0) {
        // 右转的话, 终点方位角 = 起点方位角 + 转角
        zhAzimuth = _startPt.azimuth + corner;
    } else {
        zhAzimuth = _startPt.azimuth - corner;
    }
    // 范围判断
    zhAzimuth = MathFunction::JudgeAzimuth(zhAzimuth);

    double a1 = a0 + zhAzimuth + PI/2;
    //    DEBUG_OUT("HZ点计算:转角值: %f, X0, Y0:%f, %f, s: %f, a0:%f, a1: %f", corner, x0, y0, s, a0, a1);

    double x1 = s*cos(a1);
    double y1 = s*sin(a1);
    x = _startPt.x - x1;
    y = _startPt.y - y1;
    // DEBUG_OUT("X1: %f, y1:%f", x1, y1);

    point.azimuth = zhAzimuth;
    point.x = x;
    point.y = y;
    return (point);
}

/**
 * @brief  对于完整的第二缓和曲线计算HZ点坐标
 *
 * @return HZ点坐标
 */
Point3D CurveGeometry::computerTwoFullCurveHZPoint() {
    // 对于第二段完整缓和曲线需要进行逆向运算算出HZ点
    Point3D point;
    double x = 0;
    double y = 0;

    double x0 = computerPointX0(_fullLen);
    double y0 = computerPointY0(_fullLen);

    double n = judgeQuadrant(x0, y0);

    // 在一ZH点为原点的坐标系中, 距离原点的长度
    double s = sqrt(pow(x0, 2) + pow(y0, 2));
    double a0 = atan2(y0, x0);

    // 计算转角
    double corner = pow(_fullLen, 2) / (2 * _paramA);
    _corner = corner;
    // 计算HZ点方位角
    double zhAzimuth = 0.0;
    if(_direction > 0) {
        // 右转的话, 终点方位角 = 起点方位角 + 转角
        zhAzimuth = _startPt.azimuth + corner;
    } else {
        zhAzimuth = _startPt.azimuth - corner;
    }
    // 范围判断
    zhAzimuth = MathFunction::JudgeAzimuth(zhAzimuth);

    double a1 = a0 + zhAzimuth + PI/2;
    //    DEBUG_OUT("HZ点计算:转角值: %f, X0, Y0:%f, %f, s: %f, a0:%f, a1: %f", corner, x0, y0, s, a0, a1);

    double x1 = s*cos(a1);
    double y1 = s*sin(a1);
    x = _startPt.x - x1;
    y = _startPt.y - y1;
    // DEBUG_OUT("X1: %f, y1:%f", x1, y1);

    point.azimuth = zhAzimuth;
    point.x = x;
    point.y = y;
    return (point);
}

/**
 * @brief  计算曲线终点
 */
void CurveGeometry::computerEndPoint() {
    switch(_type) {
    case T_SZ_EY: { // 第一完整缓和曲线
        calUnKownPoint(&_endPt, _fullLen);
    }
    break;
    case T_SY_EZ: { // 第二完整缓和曲线
        _endPt = _zhPoint;
    }
    break;
    case T_SYB_EYS: { // 第一不完整缓和曲线
        calUnKownPoint(&_endPt, _fullLen);
    }
    break;
    case T_SYS_EYB: {  // 第二不完整缓和曲线
        calUnKownPoint(&_endPt, _fullLen - _length);
    }
    break;
    default:
    // No select
    break;
    }
}

/**
 * @brief  根据里程计算曲线上点在ZH或HZ坐标系中的x坐标
 *
 * @param  mileage 距离ZH或HZ点的曲线长
 *
 * @return x坐标
 */
double CurveGeometry::computerPointX0(double mileage) {
    double x0 = pow(mileage, 3)/(6 * _paramA)
              - pow(mileage, 7)/(336 * pow(_paramA, 3))
              + pow(mileage, 11)/(42240 * pow(_paramA, 5))
              - pow(mileage, 15)/(9676800 * pow(_paramA, 7))
              + pow(mileage, 19)/(3530096640UL * pow(_paramA, 9))
              - pow(mileage, 23)/(1880240947200UL * pow(_paramA, 11));
    x0 *= _k;

    return x0;

}

/**
 * @brief  根据里程计算曲线上点在ZH或HZ坐标系中的y坐标
 *
 * @param  mileage 距离ZH或HZ点的曲线长
 *
 * @return y坐标
 */
double CurveGeometry::computerPointY0(double mileage) {
     double y0 = mileage
               - pow(mileage, 5)/(40 * pow(_paramA, 2))
               + pow(mileage, 9)/(3456 * pow(_paramA, 4))
               - pow(mileage, 13)/(599040 * pow(_paramA, 6))
               + pow(mileage, 17)/(175472640UL * pow(_paramA, 8))
               - pow(mileage, 21)/(78033715200UL * pow(_paramA, 10));
    return y0;
}

/**
 * @brief  根据x,y计算判断象限
 *
 * @param  x0 x坐标
 * @param  y0 y坐标
 *
 * @return  象限判断结果
 */
int CurveGeometry::judgeQuadrant(double x0, double y0) {
    int n = 0;
    if((x0 > 0) && (y0 < 0)) {
        n = 2;
    }else if((x0 < 0) && (y0 > 0)) {
        n = 1;
    }else if((x0 < 0) && (y0 < 0)) {
        n = 1;
    }

    return (n);
}

/**
 * @brief  计算曲线的总长
 */
void CurveGeometry::computerEmluatorPara() {
    switch(_type) {
    case T_SZ_EY:
    case T_SY_EZ: {
        _fullLen = _length;
    }
    break;
    case T_SYS_EYB: { // 第二不完整缓和曲线
        _fullLen = _paramA/_dr2;
    }
    break;
    case T_SYB_EYS: { // 第一不完整缓和曲线
        _fullLen = _paramA/_dr2;
    }
    break;
    default:
    // No select
    break;
    }
}

/**
 * @brief  计算缓和曲线参数的平方
 * 公式: A^2 = l * (r1 * r2) / (r1 - r2)
 * l为曲线的长度, r1 位半径大的一边
 */
void CurveGeometry::computerParamA(void) {
    switch(_type) {
    case T_SZ_EY: { // 第一完整缓和曲线
        _dr1 = _endRadius;
        _dr2 = 0.0;
        _paramA = _length * _dr1;
        _k = _direction * -1;

    }
    break;
    case T_SY_EZ: { // 第二完整缓和曲线
        _dr1 = _startRadius;
        _dr2 = 0.0;
        _paramA = _length * _dr1;
        _k = _direction;
    }
    break;
    case T_SYB_EYS: { // 第一不完整缓和曲线
        _dr1 = _startRadius;
        _dr2 = _endRadius;
        _k = _direction * -1;

        double temp = (_dr1 * _dr2) / (_dr1 - _dr2);
        _paramA = _length * fabs(temp);
    }
    break;
    case T_SYS_EYB: {  // 第二不完整缓和曲线
        _dr1 = _endRadius;
        _dr2 = _startRadius;
        _k = _direction;

        double temp = (_dr1 * _dr2) / (_dr1 - _dr2);
        _paramA = _length * fabs(temp);
    }
    break;
    default:
    // No select
    break;
    }
}

/**
 * @brief  根据起点半径和终点半径判断曲线类型
 *
 * @return 类型
 */
CURVE_T CurveGeometry::getType() {
    if((_startRadius <= 0.00001) && (_endRadius >= 0.00001)){
        DEBUG_OUT("第一完整缓和曲线");
        return (T_SZ_EY);
    }

    if((_startRadius >= 0.00001) && (_endRadius <= 0.00001)) {
        DEBUG_OUT("第二完整缓和曲线");
        return (T_SY_EZ);
    }

    if(_startRadius > _endRadius) {
        DEBUG_OUT("第一非完整缓和曲线");
        return (T_SYB_EYS);
    }

    if(_endRadius > _startRadius) {
        DEBUG_OUT("第二非完整缓和曲线");
        return (T_SYS_EYB);
    }

    return (T_NO);
}

/**
 * @brief  计算左边桩点
 *
 * @param  pedal out，垂足
 * @param  mileage out，里程
 * @param  offset out，偏距
 *
 * @return 边桩点
 */
Point3D CurveGeometry::computerBorderLeftPoint(Point3D& pedal, double mileage, double offset) {
    // 计算转角
    double corner = pow(mileage, 2) / (2 * _paramA);
    if((T_SZ_EY == _type) || (T_SYB_EYS == _type)) {
        // 第一段缓和曲线
        corner = MathFunction::JudgeAzimuth(_zhPoint.azimuth + _direction * corner);
    } else {
        // 第二段缓和曲线
        corner = MathFunction::JudgeAzimuth(_startPt.azimuth + _direction * (_corner - corner));
    }
    double angle = MathFunction::JudgeAzimuth(corner - BORDER_ANGLE);

    Point3D pt;
    pt.x = pedal.x + offset*cos(angle);
    pt.y = pedal.y + offset*sin(angle);
    return (pt);
}

/**
 * @brief  计算右边桩点
 *
 * @param  pedal out，垂足
 * @param  mileage out，里程
 * @param  offset out，偏距
 *
 * @return 边桩点
 */
Point3D CurveGeometry::computerBorderRightPoint(Point3D& pedal, double mileage, double offset) {
    // 计算转角
    double corner = pow(mileage, 2) / (2 * _paramA);
    if((T_SZ_EY == _type) || (T_SYB_EYS == _type)) {
        // 第一段缓和曲线
        corner = MathFunction::JudgeAzimuth(_zhPoint.azimuth + _direction * corner);
    } else {
        // 第二段缓和曲线
        corner = MathFunction::JudgeAzimuth(_startPt.azimuth + _direction * (_corner - corner));
    }
    double angle = MathFunction::JudgeAzimuth(corner + BORDER_ANGLE);

    Point3D pt;
    pt.x = pedal.x + offset*cos(angle);
    pt.y = pedal.y + offset*sin(angle);
    return (pt);
}

/**
 * @brief  反算算法1， 参照网址如下：
 * http://wenku.baidu.com/link?url=s017wqDkRAgGuHzJsYhzSSBwrLWtgLbsy7-
 * LpECetmq5yYbww_srcBl4T1x15Z4TYJUPaOs3uDlY44ar2_RWHSMXaheUj2-3QHiLQnLsfZu&qq-pf-to=pcqq.c2c
 *
 * @param  offlinePt in，线外点
 * @param  pedal out，垂足
 * @param  mileage out，里程
 * @param  offset out，偏距
 */
void CurveGeometry::inverseComputerOne(Point3D& offlinePt, Point3D& pedal, double& mileage, double& offset) {
    double initM = 0;
    if(false == besureInCurveArea(offlinePt, initM)) {
        DEBUG_OUT("Invalid point!");
        mileage = 0;
        offset = 0;
        return ;
    }

    DEBUG_OUT("Initialize mileage is : %f", initM);
    // 第一个点
    Point3D point;
    calUnKownPoint(&point, initM);
    double g = MathFunction::GetLineAngle(point, offlinePt);
    double len = MathFunction::GetTwoPointLength(point, offlinePt);
    double dm = len * cos(g - point.azimuth);

    double tempMileage = initM;
    double tempAzimuth = 0;
    double tempLen = 0;
    double tempNum = 20;
    Point3D tempPt;
    while(1) {
        if((dm <= 0.001) || (0 > tempNum)) {
            pedal = tempPt;
            mileage = tempMileage;
            // offset = tempLen;
            offset = (tempPt.y - offlinePt.y) / sin(tempPt.azimuth - PI/2);
            break;
        }

        tempMileage += dm;
        if(tempMileage > _length) {
            mileage = 0;
            offset = 0;
            break;
        }
        calUnKownPoint(&tempPt, tempMileage);

        tempLen = MathFunction::GetTwoPointLength(tempPt, offlinePt);
        tempAzimuth = MathFunction::GetLineAngle(tempPt, offlinePt);
        dm = tempLen * cos(tempAzimuth - tempPt.azimuth);

        --tempNum;
    }
}
/**
 * @brief  反算算法2， 参照网址如下：
 * http://www.doc88.com/p-9883671254543.html
 *
 * @param  offlinePt in，线外点
 * @param  pedal out，垂足
 * @param  mileage out，里程
 * @param  offset out，偏距
 */
void CurveGeometry::inverseComputerTwo(Point3D& offlinePt, Point3D& pedal, double& mileage, double& offset) {
    double initM = 0;
    if(false == besureInCurveArea(offlinePt, initM)) {
        DEBUG_OUT("Invalid point!");
        mileage = 0;
        offset = 0;
        return ;
    }

    /*
     * initM为算法中的初始化里程,为距离起点的距离;
     * 对于第一完整缓和曲线,为距离ZH点的距离;
     * 对于第一不完整缓和曲线来说, 为距离起点的距离;
     * 对于第二缓和曲线, 为距离起点的距离;
     */
    Point3D tempPt;
    tempPt = _startPt;
    DEBUG_OUT("Initialize mileage is : %f", initM);

    if(initM <= 0.001) {
        // 如果小于最小误差值的话，则垂足为起点
        pedal = tempPt;
        mileage = 0;
        offset = (tempPt.y - offlinePt.y) / sin(tempPt.azimuth - PI/2);
        return ;
    }

    // 差值
    double dm = initM;
    double tempMileage = 0;  // 距离ZH(第一缓和)或起点(第二缓和)的距离
    double tempMileage1 = 0; // 距离ZH或HZ的距离(计算点时使用)
    double tempLen = 0;
    while(1) {
        double fdm = fabs(dm);
        if(fdm <= 0.001) {
            pedal = tempPt;
            mileage = tempMileage;
            offset = (tempPt.y - offlinePt.y) / sin(tempPt.azimuth - PI/2);
            break;
        }

        tempMileage += dm;
        tempMileage1 = tempMileage;
        if((T_SY_EZ == _type) || (T_SYS_EYB == _type)){
            // 第二缓曲
            tempMileage1 = _fullLen - tempMileage;
        } else if(T_SYB_EYS == _type) {
            // 第一不完整
            tempMileage1 = _fullLen - _length + tempMileage;
        }

        if(tempMileage1 > _fullLen) {
            // 如果里程大于弧长， 则错误
            // 说明不在线上
            mileage = 0;
            offset = 0;
            break;
        }

        calUnKownPoint(&tempPt, tempMileage1);
        dm = (offlinePt.y - tempPt.y)*cos(tempPt.azimuth - PI/2)
            - (offlinePt.x - tempPt.x)*sin(tempPt.azimuth - PI/2);
        DEBUG_OUT("Mileage: %f, CurveMileage: %f, dm: %f, azimuth: %f",
                  tempMileage, tempMileage, dm, tempPt.azimuth);
    }
}

/**
 * @brief  反算算法3， 参照网址如下：
 * http://www.doc88.com/p-9883671254543.html
 * 主要与法线垂距公式进行迫近
 *
 * @param  offlinePt in，线外点
 * @param  pedal out，垂足
 * @param  mileage out，里程
 * @param  offset out，偏距
 */
void CurveGeometry::inverseComputerThree(Point3D& offlinePt, Point3D& pedal, double& mileage, double& offset) {
    double initM = 0;
    if(false == besureInCurveArea(offlinePt, initM)) {
        DEBUG_OUT("Invalid point!");
        mileage = 0;
        offset = 0;
        return ;
    }

    /*
     * initM为算法中的初始化里程,为距离起点的距离;
     * 对于第一完整缓和曲线,为距离ZH点的距离;
     * 对于第一不完整缓和曲线来说, 为距离起点的距离;
     * 对于第二缓和曲线, 为距离起点的距离;
     */
    Point3D tempPt;
    tempPt = _startPt;
    DEBUG_OUT("Initialize mileage is : %f", initM);
    if(initM <= 0.0001) {
        // 如果小于最小误差值的话，则垂足为起点
        pedal = tempPt;
        mileage = 0;
        offset = (tempPt.y - offlinePt.y) / sin(tempPt.azimuth - PI/2);
        return ;
    }

    // 差值
    double dm = initM;
    double tempMileage = 0;  // 距离ZH(第一缓和)或起点(第二缓和)的距离
    double tempMileage1 = 0; // 距离ZH或HZ的距离(计算点时使用)
    double tempLen = 0;

    int searchCount = 15;
    while(1) {
        if(searchCount < 0) {
            // 进行了15步还没有找到, 说明没有合适的了, 放弃吧
            DEBUG_OUT("No find, so give the current point!");
            pedal = tempPt;
            mileage = tempMileage;
            offset = (tempPt.y - offlinePt.y) / sin(tempPt.azimuth - PI/2);
            break;
        }

        double fdm = fabs(dm);
        if(fdm <= 0.0001) {
            pedal = tempPt;
            mileage = tempMileage;
            offset = (tempPt.y - offlinePt.y) / sin(tempPt.azimuth - PI/2);
            break;
        }

        tempMileage += dm;
        tempMileage1 = tempMileage;
        // 校正到ZH或HZ点的距离
        if((T_SY_EZ == _type) || (T_SYS_EYB == _type)){
            // 第二缓曲
            tempMileage1 = _fullLen - tempMileage;
        } else if(T_SYB_EYS == _type) {
            // 第一不完整
            tempMileage1 = _fullLen - _length + tempMileage;
        }

        if(0.0001 < (tempMileage - _length)) {
            // 如果里程大于弧长还没找到, 则默认为最后一个点
            // TODO: 修改
            // calUnKownPoint(&tempPt, _fullLen);
            DEBUG_OUT("No point!=========");
            mileage = 0;
            offset = 0;
            break;
        }

        calUnKownPoint(&tempPt, tempMileage1);
        dm = (offlinePt.x - tempPt.x)*sin(tempPt.azimuth + BORDER_ANGLE)
            - (offlinePt.y - tempPt.y)*cos(tempPt.azimuth + BORDER_ANGLE);
        DEBUG_OUT("Mileage: %f, CurveMileage: %f, dm: %f, azimuth: %f",
                  tempMileage, tempMileage1, dm, tempPt.azimuth);

        --searchCount;
    }
}

/**
 * @brief  线外点是否在曲线内
 *
 * @param  offlinePt in,线外点
 * @param  startVerDis out,线外点距离中桩点法线的垂距
 *
 * @return false：不在此线内，无解；true: 在此线内；
 */
bool CurveGeometry::besureInCurveArea(Point3D& offlinePt, double &startVerDis) {
    DEBUG_OUT("StartPt: %f-%f; EndPt: %f - %f", _startPt.x, _startPt.y, _endPt.x, _endPt.y);
    double startPtVerDis = getVerDisOfPtTotangent(offlinePt, _startPt);
    double endPtVerDis = getVerDisOfPtTotangent(offlinePt, _endPt);
    DEBUG_OUT("StartVer: %f, EndVer: %f", startPtVerDis, endPtVerDis);
    // 第一缓和曲线
    if((startPtVerDis >= 0.0) && (endPtVerDis <= 0.0)) {
        startVerDis = startPtVerDis;
        return true;
    }

    return false;
}

/**
 * @brief 获得点到中桩的法线的垂距
 *
 * @param  offlinePt in，线外点
 * @param  inlinePt in，中桩点
 *
 * @return 垂距
 */
double CurveGeometry::getVerDisOfPtTotangent(Point3D& offlinePt, Point3D& inlinePt)
{
    // 利用二维坐标平移旋转公式计算点到法线的垂距
    double verDis = (offlinePt.x - inlinePt.x)*sin(inlinePt.azimuth + BORDER_ANGLE)
        - (offlinePt.y - inlinePt.y)*cos(inlinePt.azimuth + BORDER_ANGLE);
    return (verDis);
}

/*
 * Copyright (C) 2016-01-26 Dasea <dhf0214@126.com>
 *
 * Description: 圆曲线定义
 */
#include "CircleCurveGeometry.h"
#include "MathFunction.h"
#include "GlobalSet.h"

/**
 * @brief  Default Constructor
 */
CircleCurveGeometry::CircleCurveGeometry(){
    // Constructor
    _radius = 0;
    _length = 0;
    _angle = 0;
}

/**
 * @brief  三点确定一个圆弧
 *
 * @param  startPt 起点
 * @param  middlePt 中间点
 * @param  endPt 终点
 * @param  lineGap 线分割长度
 */
CircleCurveGeometry::CircleCurveGeometry(Point3D& startPt, Point3D& middlePt, Point3D& endPt){
    DEBUG_OUT("=========计算圆曲线===========");
    // Constructor
    _startPt  = startPt;
    _middlePt = middlePt;
    _endPt    = endPt;

    computerCurvePara();
}

/**
 * 俩点定弧
 */
CircleCurveGeometry::CircleCurveGeometry(Point3D& startPt, Point3D& endPt, double radius, int direction) {
    DEBUG_OUT("=========计算圆曲线===========");
    _startPt = startPt;
    _endPt = endPt;
    _radius = radius;
    _direction = direction;

    // 计算圆弧长度
    // 起点与终点之间的长度
    double startToEnd = MathFunction::GetTwoPointLength(_startPt, _endPt);
    // 由余弦定理计算转角
    double temp = pow(startToEnd, 2) / (2 * pow(radius, 2));
    double corner = acos(1 - temp);
    // 圆弧方位角
    double startEndAzimuth = MathFunction::GetLineAngle(_startPt, _endPt);
    _angle = MathFunction::JudgeAzimuth(startEndAzimuth - _direction * (corner/2));
    _startPt.azimuth = _angle;
    // 圆弧长度
    _length = radius * corner;
    DEBUG_OUT("起点方位角为: %f, 圆弧长度为: %f", _angle, _length);

    // 终点方位角
    _endPt.azimuth = MathFunction::JudgeAzimuth(_angle + getReAngle(_length));
}
/**
 * @brief  圆弧起始点,圆弧长等确定一个点
 *
 * @param  startPt 圆起始点
 * @param  radius 圆弧半径
 * @param  len 圆弧长度
 * @param  angle 圆弧方位角
 * @param  lineGap 线分割长度
 */
CircleCurveGeometry::CircleCurveGeometry(Point3D& startPt, double radius, double len,
                                        double angle, int direction){
    DEBUG_OUT("=========计算圆曲线===========");
    // Constructor
    SetGeometryPorperty(startPt, radius, len, angle, direction);

    computerEndPoint();
}

/**
 * @brief  Default Destructor
 */
CircleCurveGeometry::~CircleCurveGeometry(){
    // Destructor
}

/**
 * @brief  由圆弧起点, 半径等确定圆
 *
 * @param  startPt 圆弧起点
 * @param  radius 圆弧半径
 * @param  len 圆弧长度
 * @param  angle 圆弧方位角
 * @param  lineGap 线切割长度
 */
void CircleCurveGeometry::SetGeometryPorperty(Point3D& startPt, double radius,
                                              double len, double angle, int direction){
    _startPt = startPt;
    _radius = radius;
    _length = len;
    _angle = angle;
    _startPt.azimuth = _angle;
    _direction = direction;
}

/**
 * @brief  获取画出此圆弧所需的点
 *
 * @param  ptCount out, 点的个数
 * @param  endAngle out,终点方位角
 *
 * @return 点数组, 由调用者释放指针
 */
Point3D* CircleCurveGeometry::GetGeometryPoints(int& ptCount, double& endAngle) {
    int count =getPointsCount();
    Point3D *points=new Point3D[count];
    if(NULL == points) {
        DEBUG_OUT("Alloc memory failed!");

        ptCount = 0;
        return NULL;
    }

    int m = 0;
    int lineGap = static_cast<int>(GlobalSet::GetCircleLineGap());
    for (int i = 0; i < count-1;i++)
    {
        points[i] = calUnKownPoint(m);
        m += lineGap;
    }

    //处理最后一个点
    points[count-1]=calUnKownPoint(_length);

    endAngle = _angle + getReAngle(_length);
    endAngle = MathFunction::JudgeAzimuth(endAngle);
    points[count-1].z = endAngle;
    ptCount = count;

    // Point3D pedal;
    // Point3D pt = GetPointByArcLen(pedal, 0.000078, 0);
    // DEBUG_OUT("Point: %f - %f", pt.x, pt.y);
    return points;
}

/**
 * @brief  给定一个点,计算此点距离圆弧的垂距,及距离圆弧起点的长度
 *
 * @param  resolvePt 给定的点
 * @param  pedal     垂足
 * @param  verticalInt 返回值,垂距
 * @param  arcLen 返回值,距离起点的弧长
 */
void CircleCurveGeometry::GetVDisAndArcLen(Point3D& resolvePt, Point3D& pedal, double& mileage, double& offset){
    // 首先需要判断点是否在曲线内
    double initM = 0;
    if(false == besureInCurveArea(resolvePt, initM)) {
        mileage = 0;
        offset = 0;
        return ;
    }

    // 计算终点属性
    _endPt = calUnKownPoint(_length);
    _middlePt = calUnKownPoint(_length/2);
    // 得到圆心
    Point3D centerPt= MathFunction::GetCircleCenter(_startPt, _middlePt, _endPt);
    // 获取俩点的距离
    double len = MathFunction::GetTwoPointLength(resolvePt, centerPt);
    // 获取偏距
    double tempOffset = len - _radius;
    offset = -1.0 * tempOffset * _direction;
    // 根据余弦定理计算转角{{{
    // 计算偏桩点到起点的距离
    double len1 = MathFunction::GetTwoPointLength(resolvePt, _startPt);
    // 余弦值
    double cosval = (pow(len, 2) + pow(_radius, 2) - pow(len1, 2)) / (2 * _radius * len);
    // 转角值
    double corner = acos(cosval);
    // 则距离起点的里程为
    double tempM = corner * _radius;

    mileage = tempM;
    pedal = calUnKownPoint(mileage);
    //}}}
#if 0
    // 计算改点到线元起点的法线的垂距
    double startPtVerDis = getVerDisOfPtToTangent(resolvePt, _startPt, _angle);

    double tempMileage = 0;
    double tempOffset = 0;
    // 对于单个线段来说,起点桩号为0
    getMileageMesFromOutPt(resolvePt, startPtVerDis, tempMileage, tempOffset);

    pedal = calUnKownPoint(tempMileage);
    mileage = tempMileage;
    offset = tempOffset;
#endif
}

/**
 * @brief  返回距离圆弧起点距离为len的弧上点
 *
 * @param  pedal   垂足
 * @param  mileage 里程
 * @param  offset 偏距
 *
 * @return 点
 */
Point3D CircleCurveGeometry::GetPointByArcLen(Point3D& pedal, double mileage, double offset){
    // 如果里程超过弧长的话,则是不合法的值
    if(_length < mileage){
        pedal.x = 0; pedal.y = 0;
        return Point3D(0, 0, 0);
    }

    if((0.0001 >= offset) && (offset > 0)) {
        // 0 为线上点
        pedal = calUnKownPoint(mileage);
        return pedal;
    }else if(0 > offset) {
        // 小于0为左侧点
        offset = fabs(offset);
        return computerBorderLeftPoint(pedal, mileage, offset);
    }else{
        // 大于0为右侧点
        return computerBorderRightPoint(pedal, mileage, offset);
    }
}

/**
 * @brief  三点确定一个圆的时候计算一些中间参数,比如圆心,半径等
 */
void   CircleCurveGeometry::computerCurvePara() {
    // 确定圆曲线半径
    computerRadio();
    computerDirection();
    computerArcLen();    //圆弧长度

    double startToMidLen = MathFunction::GetTwoPointLength(_startPt, _middlePt);
    // 由余弦定理计算转角
    double temp = pow(startToMidLen, 2) / (2 * pow(_radius, 2));
    double corner = acos(1 - temp);
    double fcor = corner / 2;
    // 起点方位角
    _angle = MathFunction::GetLineAngle(_startPt, _middlePt) - _direction * fcor;
    // 终点方位角
    double endAngle = MathFunction::JudgeAzimuth(_angle + getReAngle(_length));
    _endPt.azimuth = endAngle;

    DEBUG_OUT("起点方位角:%f, 半径: %f, 弧长: %f, 偏向: %d",
            _angle, _radius, _length, _direction);
}

/**
 * @brief  计算半径
 */
void CircleCurveGeometry::computerRadio(void) {
    // 三点可以确定圆心及半径
    // 公式: (x- a)^2 + (y - b)^2 = r^2
    _centerPt = MathFunction::GetCircleCenter(_startPt, _middlePt, _endPt);

    double sqrtR = (_endPt.x - _centerPt.x) * (_endPt.x - _centerPt.x)
                + (_endPt.y - _centerPt.y) * (_endPt.y - _centerPt.y);
    _radius = sqrt(sqrtR);
    DEBUG_OUT("圆心坐标: %f, %f, 半径: %f", _centerPt.x, _centerPt.y, _radius);
}

/**
 * @brief  计算圆弧长度
 */
void  CircleCurveGeometry::computerArcLen() {
    double  angle = MathFunction::GetTwoLineAngle(_startPt, _middlePt, _endPt);
    _length = _radius*angle*2;
}

/**
 * @brief  计算线在拐点处道路的方向，左转还是右转
 * 在原方向左侧的时候为左转角，否则为右转角

 * 怎么判断坐标为（xp，yp）的点P是在直线的哪一侧呢?设直线是由其上两点（x1,y1）(x2,y2)确定的
 * 直线方向是由（x1，y1）到（x2，y2）的方向。这时若直线方程记为Ax+By+C=0
 * 则有：
 * A=y2-y1; B=x1-x2; C=x2*y1-x1*y2;
 * 这时可以计算D：
 * D=A*xp+B*yp+C
 * 若D<0，则点（xp，yp）在直线的左侧；若D>0，则点在直线的右侧；D＝0点在直线上。
 *
 */
void  CircleCurveGeometry::computerDirection() {
    double direction = (_middlePt.y-_startPt.y)*_endPt.x
                + (_startPt.x-_middlePt.x)*_endPt.y
                + ((_middlePt.x*_startPt.y)-(_middlePt.y*_startPt.x));
    if( direction < 0) {
        _direction = DIRECTION_TYPE_RIGHT;
    } else if(direction > 0) {
        _direction = DIRECTION_TYPE_LEFT;
    } else {
        _direction = DIRECTION_TYPE_NO;
    }
}

/**
 * @brief  曲线终点坐标及方位角
 */
void CircleCurveGeometry::computerEndPoint() {
    _endPt = calUnKownPoint(_length);
    _endPt.azimuth = MathFunction::JudgeAzimuth(_angle + getReAngle(_length));
}

/**
 * @brief  计算点
 *
 * @param  milage 距离起点的弧长
 *
 * @return 点信息
 */
#if 0
Point3D CircleCurveGeometry::calUnKownPoint(double mileage) {
    Point3D pt;

    // 转角(2*PI*R = 周长, 所以相对的mileage对应的角度(转角)为mileage/_radius)
    double angle = mileage/_radius;
    double dx=_radius*sin(angle);
    double dy=_radius*(1-cos(angle));
    DEBUG_OUT("Angle: %f, dx: %f; dy: %f", angle, dx, dy);

    pt.x=_startPt.x + dx*cos(_angle) + _direction*dy*sin(_angle);
    pt.y=_startPt.y + dx*sin(_angle) - _direction*dy*cos(_angle);

    return (pt);
}
#endif
Point3D CircleCurveGeometry::calUnKownPoint(double mileage) {
    Point3D pt;

    // 转角:
    double corner = mileage / _radius;
    // 弦偏角
    double xAngle = corner / 2;
    // Azh_i
    double angle = _angle + _direction*xAngle;
    // 弦长
    // double chord = 2 * _radius * sin(angle); // 按泰勒展开式展开如下
    double chord = mileage
                  - pow(mileage, 3) / (24 * pow(_radius, 2))
                  + pow(mileage, 5) / (1920 * pow(_radius, 4))
                  - pow(mileage, 7) / (322560 * pow(_radius, 6));
    // DEBUG_OUT("转角: %f, Angle: %f, 弦长: %f", corner, angle, chord);

    pt.x = _startPt.x + chord * cos(angle);
    pt.y = _startPt.y + chord * sin(angle);
    pt.z = MathFunction::JudgeAzimuth(angle);
    DEBUG_OUT("M: %f, x-y: %f - %f", mileage, pt.x, pt.y);

    return (pt);
}

/**
 * @brief  获取可以分割的点个数
 *
 * @return 点数
 */
int CircleCurveGeometry::getPointsCount(void) {
    int lineGap = static_cast<int>(GlobalSet::GetCircleLineGap());
    int count =(static_cast<int>(_length)/lineGap)+1;
    if (count <= 1) count=2;
    return count;
}

/**
 * @brief  获取mileage里程长对应的转向角
 *
 * @param  距离圆弧起点的圆弧长
 *
 * @return 转向角
 */
double CircleCurveGeometry::getReAngle(double mileage) {
    return ((_direction * mileage)/_radius);
}

/**
 * @brief  计算边桩左边点
 *
 * @param  pedal 垂足
 * @param  mileage 里程长
 * @param  offset 偏距
 *
 * @return 点
 */
Point3D  CircleCurveGeometry::computerBorderLeftPoint(Point3D& pedal, double mileage, double offset) {
    double angle = _angle + getReAngle(mileage);
    angle = MathFunction::JudgeAzimuth(angle - BORDER_ANGLE);

    Point3D zhpoint = calUnKownPoint(mileage);

    Point3D point;
    point.x = zhpoint.x + offset*cos(angle);
    point.y = zhpoint.y + offset*sin(angle);

    pedal = zhpoint;
    return (point);
}

/**
 * @brief  计算边桩右边点
 *
 * @param  pedal 垂足
 * @param  mileage 里程
 * @param  offset 偏距
 *
 * @return 点
 */
Point3D  CircleCurveGeometry::computerBorderRightPoint(Point3D& pedal, double mileage, double offset) {
    // 计算转向角
    double angle = _angle + getReAngle(mileage);
    angle = MathFunction::JudgeAzimuth(angle + BORDER_ANGLE);

    Point3D zhpoint = calUnKownPoint(mileage);

    Point3D point;
    point.x = zhpoint.x + offset*cos(angle);
    point.y = zhpoint.y + offset*sin(angle);

    pedal = zhpoint;
    return (point);
}

/**
 * @brief  判断线外点是否在曲线内
 *
 * @param  offlinePt 线外点
 * @param  startVerDis 初始化计算里程,距离起点的里程
 *
 * @return 在:true; false:不在;
 */
bool CircleCurveGeometry::besureInCurveArea(Point3D& offlinePt, double& startVerDis) {
    DEBUG_OUT("StartPt: %f-%f; EndPt: %f - %f", _startPt.x, _startPt.y, _endPt.x, _endPt.y);
    double startPtVerDis = getVerDisOfPtToTangent(offlinePt, _startPt, _startPt.azimuth);
    double endPtVerDis = getVerDisOfPtToTangent(offlinePt, _endPt, _endPt.azimuth);
    DEBUG_OUT("StartVer: %f, EndVer: %f", startPtVerDis, endPtVerDis);
    // 第一缓和曲线
    if((startPtVerDis >= 0.0) && (endPtVerDis <= 0.0)) {
        startVerDis = startPtVerDis;
        return true;
    }

    return false;
}

/**
 * @brief  获取点到中桩的法线的垂距
 *         利用二维坐标平移旋转公式计算点到法线的垂距
 *
 * @param  point 线外点坐标
 * @param  desPt 中桩上的点
 * @param  angle desPt的方位角
 *
 * @return 垂距
 */
double CircleCurveGeometry::getVerDisOfPtToTangent(Point3D& point, Point3D& desPt, double angle) {
    double verDis = (point.x-desPt.x)*sin(angle+BORDER_ANGLE) - (point.y-desPt.y)*cos(angle+BORDER_ANGLE);
    return verDis;
}

/**
 * @brief 计算里程和偏距
 *
 * @param  point 线外点
 * @param  startMileage 该圆弧的起始里程
 * @param  mileage 外部点对应的里程
 * @param  offset 外部点对应的偏距
 */
void CircleCurveGeometry::getMileageMesFromOutPt(
    Point3D& point, double& startMileage, double& mileage, double& offset) {
    double curMileage =0;
    double angle =0;
    Point3D desPt;
    int num = 10;
    computerDH(point, desPt, startMileage, curMileage, angle, num);

    mileage = startMileage;   // 曲线桩号
    if (startMileage < 0.00001) {
        // ？里程接近0,为什么间距也为0呢
        offset = 0.0;
    }else {
        //线外点到对应线上点之间的距离  负数在左边
        offset = (desPt.y - point.y)/sin(angle - RADIAN(90));
    }
}

/**
 *
 * 点到法线的垂距公式  d=(yp'-yi)*cos(Ang(i)-90) - (xp'-xi)*sin(Ai-90)
 *  里程 =d+ zhLen   ai 方位角
 *  d>0.01    完成
 * Dp = (yp-yp')/sin(Ai-90)
 *
 */
void CircleCurveGeometry::computerDH(
    Point3D& point, Point3D& desPt, double& zhLen, double& dL, double& angle,int& num) {
    zhLen = zhLen + dL;
    desPt = calUnKownPoint(zhLen);
    angle = getReAngle(zhLen);

    dL = (point.y - desPt.y)*cos(angle - RADIAN(90)) - (point.x - desPt.x)*sin(angle - RADIAN(90));
    if (fabs(dL)<=0.001||num<0)    return;
    num--;

    computerDH(point, desPt, zhLen, dL , angle, num);
}

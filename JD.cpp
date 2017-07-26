/*
 * Copyright (C) 2016-03-02 Dasea <dhf0214@126.com>
 *
 * Description: 交点法实现文件
 */
#include "JD.h"
#include "MathFunction.h"
#include "LineGeometry.h"
#include "CircleCurveGeometry.h"
#include "CurveGeometry.h"

JDFactory::JDFactory() {
    _curveLenOneTemp = 0.0;
    _curveLenTwoTemp = 0.0;
    _radiusTemp = 0.0;
    _endKDTemp = 0.0;
}

JDFactory::~JDFactory(){
    // Destructor
    int count = _curveList.size();
    for(int i = 0; i < count; ++i) {
        JDGeometry* curve = _curveList[i];
        if(NULL != curve){
            delete curve;
            curve = NULL;
        }
    }
    _curveList.clear();

    _jdPoints.clear();
}

void JDFactory::DealPoints(JDData points[], int count, double startKD) {
    double kd = startKD; // 起点桩号;
    double dTempX=0.0,dTempY=0.0;
    for (int i = 0; i < count; ++i)
    {
        JDPoint pt;
        double dX = points[i].x;
        double dY = points[i].y;
        double lenOne= points[i].lenOne;
        double radius = points[i].radius;
        double lenTwo = points[i].lenTwo;
        if(i!=0)
            kd += sqrt((dX-dTempX)*(dX-dTempX)+(dY-dTempY)*(dY-dTempY));

        dTempX= dX;
        dTempY= dY;

        pt._point.x= dX;
        pt._point.y = dY;
        pt._curveLength1 = lenOne;
        pt._curveLength2 = lenTwo;
        pt._radius = radius;
        pt._kd = kd;
        DEBUG_OUT("KD: %f; Pt:%f-%f; LenOne:%f; radius:%f; lenTwo:%f",
                  kd, dX, dY, lenOne, radius, lenTwo);

        _jdPoints.push_back(pt);
    }
    DEBUG_OUT("===============================================================");
    PreprocessPoints();

    ProcessCurve();
}

void JDFactory::ProcessCurve() {
    // 曲线测试
    int count = _curveList.size();
    for(int i = 0; i < count; ++i) {
        DEBUG_OUT("===============================================================");
        JDGeometry* geometry = _curveList[i];
        if(TYPE_LINE == geometry->_type) {
            // line
            JDLine* line = dynamic_cast<JDLine*>(geometry);
            DEBUG_OUT("直线: KD: %f; Pt: %f-%f; len: %f; azimuth: %f.",
                      line->_kd, line->_startPt.x, line->_startPt.y, line->_length, line->_azimuth);

            LineGeometry geometry(line->_startPt, line->_length, line->_azimuth);
            int ptCount = 0;
            Point3D *points = geometry.GetGeometryPoints(ptCount);
            printPointInfo(points, ptCount);
        } else if(TYPE_CIRCLE == geometry->_type) {
            // circle
            JDCircle* circle = dynamic_cast<JDCircle*>(geometry);
            DEBUG_OUT("圆弧: KD: %f; Pt: %f-%f; radius: %f, len: %f; azimuth: %f, direction: %f.",
                      circle->_kd, circle->_startPt.x, circle->_startPt.y, circle->_radius, circle->_length, circle->_azimuth, circle->_direction);


            CircleCurveGeometry geometry(circle->_startPt, circle->_radius, circle->_length,
                                  circle->_azimuth, circle->_direction);
            int ptCount = 0;
            double endAngle = 0;
            Point3D *points = geometry.GetGeometryPoints(ptCount, endAngle);
            printPointInfo(points, ptCount);
            DEBUG_OUT("终点方位角: %f", endAngle);
        } else if(TYPE_CURVE_ONE == geometry->_type) {
            JDCurve* curve = dynamic_cast<JDCurve*>(geometry);
            DEBUG_OUT("第一缓和曲线: KD: %f; Pt: %f-%f; Radius:%f-%f; len: %f; azimuth: %f, direction: %f.",
                      curve->_kd, curve->_startPt.x, curve->_startPt.y, curve->_startRadius, curve->_endRadius, curve->_length, curve->_azimuth, curve->_direction);

            CurveGeometry geometry(curve->_startPt, curve->_startRadius, curve->_endRadius,
                                   curve->_length, curve->_azimuth, curve->_direction);
            int ptCount = 0;
            double endAngle = 0;
            Point3D *points = geometry.GetGeometryPoints(ptCount, endAngle);
            printPointInfo(points, ptCount);
            DEBUG_OUT("终点方位角: %f", endAngle);
        } else if(TYPE_CURVE_TWO == geometry->_type) {
            JDCurve* curve = dynamic_cast<JDCurve*>(geometry);
            DEBUG_OUT("第二缓和曲线: KD: %f; Pt: %f-%f; Radius:%f-%f; len: %f; azimuth: %f, direction: %f.",
                      curve->_kd, curve->_startPt.x, curve->_startPt.y, curve->_startRadius, curve->_endRadius, curve->_length, curve->_azimuth, curve->_direction);


            CurveGeometry geometry(curve->_startPt, curve->_startRadius, curve->_endRadius,
                                         curve->_length, curve->_azimuth, curve->_direction);
            int ptCount = 0;
            double endAngle = 0;
            Point3D *points = geometry.GetGeometryPoints(ptCount, endAngle);
            printPointInfo(points, ptCount);
            DEBUG_OUT("终点方位角: %f", endAngle);
        }
        DEBUG_OUT("===============================================================");
    }
}

void JDFactory::printPointInfo(Point3D* points, int count) {
    for(int i = 0; i < count; ++i) {
        DEBUG_OUT("第%d个点: %f - %f", i+1, points[i].x, points[i].y);
    }
}

void JDFactory::PreprocessPoints() {
    int count = _jdPoints.size();
    if (count<3)  return;

    int i =0;
    // 从点数组中依次取点
    JDPoint startPt,middlePt,endPt;
    startPt = _jdPoints.at(i);  // 第一个点
    ++i;
    middlePt = _jdPoints.at(i); // 第二个点
    ++i;

    for(; i < count; ++i) {
        endPt = _jdPoints.at(i);

        // 由俩个点之间的点的数据判断三点之间的连线是什么形式的
        JDCurveType pCurveType = GetCurveType(middlePt);
        dealCurveWithType(pCurveType, startPt, middlePt, endPt);

        startPt = _tempPoint;
        middlePt = endPt;
    }

    //处理最后一点,肯定为直线
    JDLine* line = new JDLine();
    computerLineParam(line, startPt, middlePt._point);
    pushCurve(line);

    DEBUG_OUT("===============================================================");
    // 打印线数据
    count = _curveList.size();
    for(int i = 0; i < count; ++i) {
        JDGeometry* geometry = _curveList[i];
        if(TYPE_LINE == geometry->_type) {
            // line
            JDLine* line = dynamic_cast<JDLine*>(geometry);
            DEBUG_OUT("直线: KD: %f; Pt: %f-%f; len: %f; azimuth: %f.",
                      line->_kd, line->_startPt.x, line->_startPt.y, line->_length, line->_azimuth);
        } else if(TYPE_CIRCLE == geometry->_type) {
            // circle
            JDCircle* circle = dynamic_cast<JDCircle*>(geometry);
            DEBUG_OUT("圆弧: KD: %f; Pt: %f-%f; radius: %f, len: %f; azimuth: %f, direction: %f.",
                      circle->_kd, circle->_startPt.x, circle->_startPt.y, circle->_radius, circle->_length, circle->_azimuth, circle->_direction);
        } else if(TYPE_CURVE_ONE == geometry->_type) {
            JDCurve* curve = dynamic_cast<JDCurve*>(geometry);
            DEBUG_OUT("第一缓和曲线: KD: %f; Pt: %f-%f; Radius:%f-%f; len: %f; azimuth: %f, direction: %f.",
                      curve->_kd, curve->_startPt.x, curve->_startPt.y, curve->_startRadius, curve->_endRadius, curve->_length, curve->_azimuth, curve->_direction);
        } else if(TYPE_CURVE_TWO == geometry->_type) {
            JDCurve* curve = dynamic_cast<JDCurve*>(geometry);
            DEBUG_OUT("第二缓和曲线: KD: %f; Pt: %f-%f; Radius:%f-%f; len: %f; azimuth: %f, direction: %f.",
                      curve->_kd, curve->_startPt.x, curve->_startPt.y, curve->_startRadius, curve->_endRadius, curve->_length, curve->_azimuth, curve->_direction);


            DEBUG_OUT("===============================================================");
        }
    }
}

//得到处理类型(中间点类型)
JDCurveType JDFactory::GetCurveType(const JDPoint& middlePt)
{
    if ((middlePt._curveLength1 > 0.0001)
        && (middlePt._radius > 0.0001)
        && (middlePt._curveLength2 > 0.0001))
        return (JDCURVETYPE_ONETWO); // 包含第一，二缓和曲线,圆等

    if ((middlePt._curveLength2 > 0.0001)
        && (middlePt._radius > 0.0001))
        return (JDCURVETYPE_TWO); // 第二缓和曲线

    if  ((middlePt._curveLength1 > 0.0001)
         && (middlePt._radius > 0.0001))
        return (JDCURVETYPE_ONE);  // 第一缓和曲线

    if (middlePt._radius > 0.0001)
        return (JDCURVETYPE_CIRCLE);   // 圆弧

    if ((middlePt._curveLength1 <= 0.0001)
        && (middlePt._radius <= 0.0001)
        && (middlePt._curveLength2 <= 0.0001))
        return  (JDCURVETYPE_LINE);  // 直线

    return (JDCURVETYPE_NO);
}

void JDFactory::dealCurveWithType(JDCurveType type, JDPoint& startPt, JDPoint& middlePt, JDPoint& endPt) {
    //////////////////////////////////////////////////////////////////////////
    //主要用于进行确定三个点之间需要用那个对象线段进行处理，最后在处理
    // 这些对象的时候，再决定这个对象需要用多少个线段进行处理
    //////////////////////////////////////////////////////////////////////////
    JDPoint  pCurveMes;
    JDCurve *curveOne = NULL;
    JDLine *line = NULL;
    JDCircle *circle = NULL;
    JDCurve *curveTwo = NULL;
    switch (type)
    {
    //组合形式直线，第一缓和,圆弧，第二缓和，
    case JDCURVETYPE_ONETWO: {
        DEBUG_OUT("直线, 第一缓和, 圆弧, 第二缓和.");
        // 第一缓和曲线
        curveOne = new JDCurve();
        curveOne->_type = TYPE_CURVE_ONE;
        computerCurveOneParam(curveOne, startPt, middlePt, endPt);

        // 如果线长度小于0.001
        double lineLen = MathFunction::GetTwoPointLength(startPt._point, curveOne->_startPt);
        if(LEASET_LINE_LEN < lineLen){
            line = new JDLine();
            computerLineParam(line, startPt, curveOne->_startPt);
        }

        circle = new JDCircle();
        computerCircleParam(circle, startPt, middlePt, endPt);

        curveTwo = new JDCurve();
        curveTwo->_type = TYPE_CURVE_TWO;
        computerCurveTwoParam(curveTwo, startPt, middlePt, endPt);

        // 最后一点的桩号信息更新
        if(0 < _endKDTemp) {
            endPt._kd = _endKDTemp;
            _endKDTemp = 0.0;
        }

        _tempPoint = _zhORhzPt;
    }
    break;
    //组合形式直线,圆弧，第二缓和，
    case JDCURVETYPE_TWO: {
        DEBUG_OUT("直线, 圆弧, 第二缓和.");
        circle = new JDCircle();
        computerCircleParam(circle, startPt, middlePt, endPt);

        double lineLen = MathFunction::GetTwoPointLength(startPt._point, _zyPoint._point);
        if(LEASET_LINE_LEN < lineLen){
            line = new JDLine();
            computerLineParam(line, startPt, _zyPoint._point);
        }

        curveTwo = new JDCurve();
        curveTwo->_type = TYPE_CURVE_TWO;
        computerCurveTwoParam(curveTwo, startPt, middlePt, endPt);

        // 最后一点的桩号信息更新
        if(0 < _endKDTemp) {
            endPt._kd = _endKDTemp;
            _endKDTemp = 0.0;
        }

        _tempPoint = _zhORhzPt;
    }
    break;
    // 组合形式直线，第一缓和,圆弧，
    case JDCURVETYPE_ONE: {
        DEBUG_OUT("直线, 第一缓和, 圆弧.");
        curveOne = new JDCurve();
        curveOne->_type = TYPE_CURVE_ONE;
        computerCurveOneParam(curveOne, startPt, middlePt, endPt);

        double lineLen = MathFunction::GetTwoPointLength(startPt._point, curveOne->_startPt);
        if(LEASET_LINE_LEN < lineLen) {
            line = new JDLine();
            computerLineParam(line, startPt, curveOne->_startPt);
        }

        circle = new JDCircle();
        computerCircleParam(circle, startPt, middlePt, endPt);

        // 为啥不更新呢?
        // if(0 < _endKDTemp) {
            // endPt._kd = _endKDTemp;
            // _endKDTemp = 0.0;
        // }

        _tempPoint = _yzPoint;
    }
    break;
    // 圆弧,组合形式直线,圆弧
    case JDCURVETYPE_CIRCLE: {
        DEBUG_OUT("直线,圆弧.");
        circle = new JDCircle();
        computerCircleParam(circle, startPt, middlePt, endPt);
        // GetTransCurve(circle,startPt,middlePt,endPt,pVeCPCurveMes);
        // pCurveMes=pVeCPCurveMes.at(0);

        double lineLen = MathFunction::GetTwoPointLength(startPt._point, _zyPoint._point);
        if(LEASET_LINE_LEN < lineLen) {
            line = new JDLine();
            computerLineParam(line, startPt, _zyPoint._point);
        }

        if(_endKDTemp > 0) {
            endPt._kd = _endKDTemp;
            _endKDTemp = 0.0;
        }

        _tempPoint = _yzPoint;
    }
    break;
    // 直线，直线加直线
    case JDCURVETYPE_LINE: {
        DEBUG_OUT("直线.");
        line = new JDLine();
        computerLineParam(line, startPt, middlePt._point);

        if(_endKDTemp > 0) {
            middlePt._kd = _endKDTemp;
            _endKDTemp = 0.0;
        }

        _tempPoint = middlePt;
    }
    break;
    // 错误数据信息
    case JDCURVETYPE_NO:
        break;
    }

    //保存对象
    pushCurve(line);
    pushCurve(curveOne);
    pushCurve(circle);
    pushCurve(curveTwo);
}

void JDFactory::computerLineParam(JDLine* line, JDPoint& startPt, Point3D& joinPt) {
    // 直线起点
    line->_startPt = startPt._point;
    // 方位角:
    line->_azimuth = MathFunction::GetLineAngle(startPt._point, joinPt);
    // 线长
    line->_length = MathFunction::GetTwoPointLength(startPt._point, joinPt);
    // 起点桩号
    line->_kd = startPt._kd;

    // 终点桩号
    _endKDTemp = startPt._kd + line->_length;
}

void JDFactory::computerCurveOneParam(JDCurve* curve, JDPoint& startPt, JDPoint& middlePt, JDPoint& endPt) {
    _startPtTemp = startPt;
    _middlePtTemp = middlePt;
    _endPtTemp = endPt;
    _curveLenOneTemp = middlePt._curveLength1;
    _curveLenTwoTemp = middlePt._curveLength2;
    _radiusTemp = middlePt._radius;

    // 计算第一缓和曲线切线长
    double tangent = getCurveOneTangent();
    // 计算起始桩号
    double startKD = _middlePtTemp._kd - tangent;
    // 起点方位角
    double azimuth = MathFunction::GetLineAngle(startPt._point, middlePt._point);
    // 计算曲线的起始点
    Point3D pt = getCurveOneStartPt(tangent);
    // TODO: 曲线前面直线的终点
    // 偏向
    double direction = MathFunction::ComputerDirection(startPt._point, middlePt._point, endPt._point);

    curve->_kd = startKD;
    curve->_startPt = pt;
    curve->_azimuth = azimuth;
    curve->_startRadius = 0;
    curve->_endRadius = _radiusTemp;
    curve->_length = _curveLenOneTemp;
    curve->_direction = direction;
}

Point3D JDFactory::getCurveOneStartPt(double tangent) {
    // 计算第一缓和曲线的转向角
    double cornerAngle = MathFunction::GetLineAngle(_startPtTemp._point, _middlePtTemp._point);
    Point3D pt;
    pt.x = _middlePtTemp._point.x - tangent * cos(cornerAngle);
    pt.y = _middlePtTemp._point.y - tangent * sin(cornerAngle);
    // DEBUG_OUT("cornerAngle: %f, tangent:%f, x-y:%f - %f", cornerAngle, tangent, pt.x, pt.y);
    return (pt);
}

// 计算第一缓和曲线切线长
double JDFactory::getCurveOneTangent() {
    // 计算俩个线段的夹角
    double angle = MathFunction::GetTwoLineAngle(_startPtTemp._point, _middlePtTemp._point, _endPtTemp._point);
    // 计算第二缓和曲线的内移值
    double pTwo = MathFunction::GetCurveP(_curveLenTwoTemp, _radiusTemp);
    // 计算第一缓和曲线的内移值
    double pOne = MathFunction::GetCurveP(_curveLenOneTemp, _radiusTemp);
    // 计算第一缓和曲线的曲线增量
    double mOne = MathFunction::GetCurveM(_curveLenOneTemp, _radiusTemp);
    // 计算第一个缓和曲线的切线长
    double tangent = _radiusTemp*tan(angle/2) + (pTwo - pOne*cos(angle))/sin(angle) + mOne;
    // DEBUG_OUT("angle: %f; P: %f-%f; m:%f", angle, pOne, pTwo, mOne);

    return (tangent);
}

void JDFactory::computerCircleParam(JDCircle* circle, JDPoint& startPt, JDPoint& middlePt, JDPoint& endPt) {
    _startPtTemp = startPt;
    _middlePtTemp = middlePt;
    _endPtTemp = endPt;
    _curveLenOneTemp = middlePt._curveLength1;
    _curveLenTwoTemp = middlePt._curveLength2;
    _radiusTemp = middlePt._radius;

    // 计算第一和第二缓和曲线角
    double curveOneCorner = _curveLenOneTemp / (2 * _radiusTemp);
    double curveTwoCorner = _curveLenTwoTemp / (2 * _radiusTemp);
    // 路线转向角
    double curveCorner = MathFunction::GetTwoLineAngle(startPt._point, middlePt._point, endPt._point);
    // 则圆弧对应的转角
    double circleCorner = curveCorner - curveOneCorner - curveTwoCorner;
    // 计算圆弧长度
    double length = _radiusTemp * circleCorner;
    //  转向角
    double angleOne = MathFunction::GetLineAngle(startPt._point, middlePt._point);
    double angleTwo = MathFunction::GetLineAngle(middlePt._point, endPt._point);
    // 计算偏向
    double direction = MathFunction::ComputerDirection(angleOne, angleTwo);
    // 计算曲线内移值
    double pOne = MathFunction::GetCurveP(_curveLenOneTemp, _radiusTemp);
    double pTwo = MathFunction::GetCurveP(_curveLenTwoTemp, _radiusTemp);
    // 计算曲线的切线增量
    double mOne = MathFunction::GetCurveM(_curveLenOneTemp, _radiusTemp);
    double mTwo = MathFunction::GetCurveM(_curveLenTwoTemp, _radiusTemp);
    // 第一切线长(只有同直线连接时,才有切线长,同缓和曲线连接时,是没有的)
    double tOne = _radiusTemp*tan(curveCorner/2) + (pTwo - pOne*cos(curveCorner))/sin(curveCorner) + mOne;
    // 第二切线长(同上)
    double tTwo = _radiusTemp*tan(curveCorner/2) + (pOne - pTwo*cos(curveCorner))/sin(curveCorner) + mTwo;

    // 计算ZH点
    Point3D zhPoint;
    // zhPoint.x = middlePt._point.x - tOne*cos(angleOne);
    // zhPoint.y = middlePt._point.y - tOne*sin(angleOne);
    zhPoint = computerCircleCrossPoint(-1*tOne, angleOne);

    // 计算起点桩号
    double kd = middlePt._kd - tOne + middlePt._curveLength1;

    // 计算起点
    double dx = _radiusTemp*sin(curveOneCorner) + _curveLenOneTemp/2 - pow(_curveLenOneTemp, 3)/(240 * pow(_radiusTemp, 2));
    double dy = _radiusTemp*(1-cos(curveOneCorner))+pow(_curveLenOneTemp, 2)/(24*_radiusTemp);

    Point3D hyPoint;
    hyPoint.x=zhPoint.x + dx*cos(angleOne) - direction*dy*sin(angleOne);
    hyPoint.y=zhPoint.y + dx*sin(angleOne) + direction*dy*cos(angleOne);

    // 方位角
    double azimuth = MathFunction::JudgeAzimuth(angleOne + direction * curveOneCorner);
    // 计算终点
    double Azh_i = azimuth + direction* (circleCorner/2);
    double chord = length
                  - pow(length, 3) / (24 * pow(_radiusTemp, 2))
                  + pow(length, 5) / (1920 * pow(_radiusTemp, 4))
                  - pow(length, 7) / (322560 * pow(_radiusTemp, 6));
    Point3D pt;
    pt.x = hyPoint.x + chord * cos(Azh_i);
    pt.y = hyPoint.y + chord * sin(Azh_i);
    _yhPoint._point = pt;
    _yhPoint._kd = kd + length;
    // DEBUG_OUT("圆弧终点: %f - %f", pt.x, pt.y);

    // 组织曲线信息
    circle->_startPt = hyPoint;
    circle->_radius = _radiusTemp;
    circle->_direction = direction;
    circle->_azimuth = azimuth;
    circle->_length = length;
    circle->_kd = kd;

    // 如果是与直线相连, 则需要计算出ZY点和YZ点
    if(_curveLenOneTemp <= 0.0001) {
        // 无第一缓和曲线, 则计算ZY点
        _zyPoint._kd = _middlePtTemp._kd - tOne;
        _zyPoint._point = zhPoint;
    }

    if(_curveLenTwoTemp <= 0.0001) {
        // 无第二缓和曲线
        _yzPoint._kd = _middlePtTemp._kd - tTwo + _curveLenOneTemp + length;
        _yzPoint._point = computerCircleCrossPoint(tTwo, angleTwo);

        double endKDLen = MathFunction::GetTwoPointLength(middlePt._point, endPt._point);
        _endKDTemp = _yzPoint._kd + endKDLen - tTwo;
    }
}

Point3D JDFactory::computerCircleCrossPoint(double tLen, double azimuth) {
    Point3D pt;
    pt.x = _middlePtTemp._point.x + tLen * cos(azimuth);
    pt.y = _middlePtTemp._point.y + tLen * sin(azimuth);
    return (pt);
}

void JDFactory::computerCurveTwoParam(JDCurve* curve, JDPoint& startPt, JDPoint& middlePt, JDPoint& endPt) {
    _startPtTemp = startPt;
    _middlePtTemp = middlePt;
    _endPtTemp = endPt;
    _curveLenOneTemp = middlePt._curveLength1;
    _curveLenTwoTemp = middlePt._curveLength2;
    _radiusTemp = middlePt._radius;

    // 计算第一第二缓和曲线角
    double curveOneCorner = _curveLenOneTemp / (2 * _radiusTemp);
    double curveTwoCorner = _curveLenTwoTemp / (2 * _radiusTemp);
    // 计算路线转向角
    double curveCorner = MathFunction::GetTwoLineAngle(startPt._point, middlePt._point, endPt._point);
    // 计算交点线方位角
    double angleOne = MathFunction::GetLineAngle(startPt._point, middlePt._point);
    double angleTwo = MathFunction::GetLineAngle(middlePt._point, endPt._point);
    // 判断偏向
    double direction = MathFunction::ComputerDirection(angleOne, angleTwo);
    // 计算圆弧对应的一个转角
    double circleCorner = curveCorner - curveOneCorner - curveTwoCorner;
    // 圆曲线弧长
    double circleLen = _radiusTemp * circleCorner;
    // 计算曲线内移值
    double pOne = MathFunction::GetCurveP(_curveLenOneTemp, _radiusTemp);
    double pTwo = MathFunction::GetCurveP(_curveLenTwoTemp, _radiusTemp);
    // 计算曲线的切线增量
    double mOne = MathFunction::GetCurveM(_curveLenOneTemp, _radiusTemp);
    double mTwo = MathFunction::GetCurveM(_curveLenTwoTemp, _radiusTemp);
    // 切线长
    double tOne = _radiusTemp*tan(curveCorner/2)
                + (pTwo - pOne*cos(curveCorner))/sin(curveCorner) + mOne;
    double tTwo = _radiusTemp*tan(curveCorner/2)
                + (pOne - pTwo*cos(curveCorner))/sin(curveCorner) + mTwo;

    // 计算HZ点, 作为下一段线的一个开始 _zhORhzPt
    double twoZh = middlePt._kd + _curveLenOneTemp + _curveLenTwoTemp +circleLen - tOne;
    double endKDLen = MathFunction::GetTwoPointLength(middlePt._point, endPt._point);
    // 计算终点桩号
    double endKd = twoZh + endKDLen - tTwo;
    // 终点(HZ点)
    Point3D hzPt;
    hzPt.x = middlePt._point.x + tTwo*cos(angleTwo);
    hzPt.y = middlePt._point.y + tTwo*sin(angleTwo);
    // DEBUG_OUT("HZ点坐标为: %f -  %f", hzPt.x, hzPt.y);

    // hz点设置
    _zhORhzPt._point = hzPt;
    _zhORhzPt._kd = twoZh;
    // DEBUG_OUT("第二缓和曲线终点信息: kd: %f; x-y: %f - %f", twoZh, hzPt.x, hzPt.y);

    // 终点
    _endKDTemp = endKd;
#if 1
    // 计算YH点, 第二缓和曲线起点
    double dx = 0, dy = 0;
    dx = _curveLenTwoTemp - pow(_curveLenTwoTemp, 5);
    dx = _curveLenTwoTemp
        - pow(_curveLenTwoTemp, 5)/(40 * pow(_radiusTemp, 2) * pow(_curveLenTwoTemp, 2));
        // + pow(_curveLenTwoTemp, 9)/(3456 * pow(_radiusTemp, 4) * pow(_curveLenTwoTemp, 4))
        // - pow(_curveLenTwoTemp, 13)/(599040 * pow(_radiusTemp, 6) * pow(_curveLenTwoTemp, 6));
    dy = pow(_curveLenTwoTemp, 3)/(6 * _radiusTemp * _curveLenTwoTemp)
         - pow(_curveLenTwoTemp, 7)/(336 * pow(_radiusTemp, 3) *pow(_curveLenTwoTemp, 3));
        // + pow(_curveLenTwoTemp, 11)/(42240 * pow(_radiusTemp, 3) *pow(_curveLenTwoTemp, 3));
    Point3D yhPoint;
    yhPoint.x = hzPt.x - dx*cos(angleTwo) - direction*dy*sin(angleTwo);
    yhPoint.y = hzPt.y - dx*sin(angleTwo) + direction*dy*cos(angleTwo);
    DEBUG_OUT("第二缓和曲线起点: %f - %f", yhPoint.x, yhPoint.y);
#endif

    // 赋值
    curve->_kd = twoZh - _curveLenTwoTemp;
    curve->_direction = direction;
    // curve->_startPt = _yhPoint._point; // yhPoint;
    curve->_startPt = yhPoint;

    curve->_azimuth = MathFunction::JudgeAzimuth(angleTwo - direction*curveTwoCorner);
    curve->_length = _curveLenTwoTemp;
    curve->_startRadius = _radiusTemp;
    curve->_endRadius = 0;
}

void JDFactory::pushCurve(JDGeometry* geometry) {
    if(NULL != geometry) {
        _curveList.push_back(geometry);
    }
}
